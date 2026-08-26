// Постоянное хранилище FloatCore на ESP32: NVS вместо eeprom_var (ТЗ §12).
//
// Refloat знает только контракт VESC:
//   bool read_eeprom_var(eeprom_var *v, int address);
//   bool store_eeprom_var(eeprom_var *v, int address);
// то есть массив 32-битных слов, адресуемых индексом. main.c:1073 пишет
// (SERIALIZED_CONFIG_LENGTH-1)/4+1 = 80 слов подряд при каждом сохранении
// конфигурации.
//
// Прямо из Refloat в NVS не пишем (ТЗ §12): Refloat вызывает контракт
// eeprom_var, а перевод в NVS — целиком дело этого модуля.
//
// Модель записи: 80 отдельных nvs_commit() на одно сохранение конфигурации
// износили бы flash без нужды, поэтому слова копятся в ОЗУ, а на носитель
// уходит один blob. Коммит выполняет отдельная задача низкого приоритета
// через FC_STORAGE_FLUSH_MS после последней записи; принудительный коммит
// доступен через fc_storage_commit().

#include "fc_platform.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <string.h>

static const char *TAG = "storage";

// 128 слов: 80 нужны Refloat под конфигурацию, запас — под будущие поля
// платформы. Размер фиксирован: blob в NVS не должен менять длину при
// обновлении прошивки, иначе старая запись перестанет читаться.
#define FC_STORAGE_WORDS 128
#define FC_STORAGE_NAMESPACE "floatcore"
#define FC_STORAGE_KEY "eeprom"
#define FC_STORAGE_FLUSH_MS 150

static struct {
    uint32_t words[FC_STORAGE_WORDS];
    nvs_handle_t handle;
    bool ready;
    volatile bool dirty;
    volatile uint64_t last_write_us;
    uint64_t commits;
} g_st;

static void flush_task(void *arg);

int fc_storage_capacity(void) {
    return FC_STORAGE_WORDS;
}

bool fc_storage_init(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS требует переинициализации (%s), стираю раздел", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_open(FC_STORAGE_NAMESPACE, NVS_READWRITE, &g_st.handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open: %s", esp_err_to_name(err));
        return false;
    }

    // Незаписанное слово должно выглядеть как стёртая ячейка EEPROM (0xFFFFFFFF):
    // именно так ведёт себя носитель на VESC, и Refloat на это рассчитывает при
    // первом старте — десериализация не пройдёт и применятся значения по умолчанию.
    memset(g_st.words, 0xFF, sizeof(g_st.words));

    size_t len = sizeof(g_st.words);
    err = nvs_get_blob(g_st.handle, FC_STORAGE_KEY, g_st.words, &len);
    if (err == ESP_OK && len == sizeof(g_st.words)) {
        ESP_LOGI(TAG, "NVS: прочитано %u слов", (unsigned) (len / 4));
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS: записи ещё нет, все слова = 0xFFFFFFFF (чистый EEPROM)");
    } else {
        ESP_LOGW(TAG, "NVS: чтение не удалось (%s, len=%u) — считаем носитель чистым",
                 esp_err_to_name(err), (unsigned) len);
        memset(g_st.words, 0xFF, sizeof(g_st.words));
    }

    g_st.ready = true;
    xTaskCreatePinnedToCore(flush_task, "fc_nvs", 3072, NULL, 3, NULL, FC_CORE_HOUSEKEEPING);
    return true;
}

bool fc_storage_read(uint32_t *value, int address) {
    if (!g_st.ready || address < 0 || address >= FC_STORAGE_WORDS || !value) {
        return false;
    }
    *value = g_st.words[address];
    return true;
}

bool fc_storage_write(uint32_t value, int address) {
    if (!g_st.ready || address < 0 || address >= FC_STORAGE_WORDS) {
        return false;
    }
    if (g_st.words[address] != value) {
        g_st.words[address] = value;
        g_st.dirty = true;
    }
    g_st.last_write_us = fc_uptime_us();
    return true;
}

bool fc_storage_commit(void) {
    if (!g_st.ready) {
        return false;
    }
    if (!g_st.dirty) {
        return true;
    }
    esp_err_t err = nvs_set_blob(g_st.handle, FC_STORAGE_KEY, g_st.words, sizeof(g_st.words));
    if (err == ESP_OK) {
        err = nvs_commit(g_st.handle);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "коммит не удался: %s", esp_err_to_name(err));
        return false;
    }
    g_st.dirty = false;
    ++g_st.commits;
    ESP_LOGI(TAG, "NVS: сохранено %u слов (коммит #%llu)", FC_STORAGE_WORDS,
             (unsigned long long) g_st.commits);
    return true;
}

static void flush_task(void *arg) {
    (void) arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(FC_STORAGE_FLUSH_MS / 3));
        if (g_st.dirty && fc_uptime_us() - g_st.last_write_us >= FC_STORAGE_FLUSH_MS * 1000ULL) {
            fc_storage_commit();
        }
    }
}
