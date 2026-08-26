// Реализация драйвера ICM-20948 (ТЗ v0.6A §5).
//
// Источник всех чисел — datasheet InvenSense ICM-20948, DS-000189 rev 1.6.
// Ссылки на разделы даны у каждого решения; «оставить по умолчанию» нигде не
// используется как аргумент.

#include "icm20948.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

// --- USER BANK 0 (DS-000189 §7.1) -------------------------------------------
#define REG_WHO_AM_I 0x00
#define REG_USER_CTRL 0x03
#define REG_PWR_MGMT_1 0x06
#define REG_PWR_MGMT_2 0x07
#define REG_INT_PIN_CFG 0x0F
#define REG_ACCEL_XOUT_H 0x2D
#define REG_BANK_SEL 0x7F

// --- USER BANK 2 -------------------------------------------------------------
#define REG_GYRO_SMPLRT_DIV 0x00
#define REG_GYRO_CONFIG_1 0x01
#define REG_GYRO_CONFIG_2 0x02
#define REG_ODR_ALIGN_EN 0x09
#define REG_ACCEL_SMPLRT_DIV_1 0x10
#define REG_ACCEL_SMPLRT_DIV_2 0x11
#define REG_ACCEL_CONFIG 0x14

#define PWR_MGMT_1_DEVICE_RESET 0x80
#define PWR_MGMT_1_SLEEP 0x40
#define PWR_MGMT_1_CLKSEL_AUTO 0x01

static struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    icm20948_config_t cfg;
    icm20948_stats_t stats;
    uint32_t counter;
    uint8_t current_bank;
    int inject_fail;
    int inject_frozen;
    icm20948_sample_t last;
    bool have_last;
} D;

icm20948_config_t icm20948_default_config(void) {
    icm20948_config_t c;
    memset(&c, 0, sizeof(c));
    c.sda_gpio = 21;
    c.scl_gpio = 22;
    // Адрес не угадывается: он подтверждён сканированием живой платы
    // (AD0 = 0 -> 0x68), см. docs/icm20948_driver.md.
    c.i2c_addr = ICM20948_I2C_ADDR_LOW;

    // 400 кГц — верхний предел I2C для ICM-20948 (DS-000189 §11.3, «I2C fast
    // mode, 400 kHz»). При 100 кГц одна транзакция на 14 байт занимает около
    // 1.4 мс, что сопоставимо с периодом контура 2 мс; при 400 кГц — около
    // 0.4 мс. Фактическая длительность измеряется драйвером и попадает в
    // статистику, чтобы это утверждение можно было проверить, а не принять.
    c.i2c_hz = 400000;

    // ±4 g. Шкала ±2 g насыщается уже на умеренных ударах колеса о неровность
    // (порядок 2–3 g), а насыщение акселерометра ломает оценку ориентации
    // сильнее, чем потеря половины разрешения. ±8 g и выше отдают разрешение
    // без нужды: 8192 LSB/g против 16384 LSB/g при ±4 g (DS-000189 §3.2).
    c.accel_fs = ICM20948_ACCEL_FS_4G;

    // ±500 °/с. При балансировке доска вращается заметно быстрее, чем 250 °/с
    // на резких манёврах и при падении, а насыщение гироскопа — прямая потеря
    // управления. ±2000 °/с даёт 16.4 LSB/(°/с) против 65.5 при ±500
    // (DS-000189 §3.1), то есть вчетверо хуже разрешение на малых скоростях,
    // где как раз и работает контур.
    c.gyro_fs = ICM20948_GYRO_FS_500DPS;

    // ODR = 1125/(1+div). div = 1 -> 562.5 Гц. Взято НЕ равным 500 Гц
    // намеренно: датчик должен выдавать данные немного чаще, чем их
    // забирает контур, иначе на каждом периоде появляется риск прочитать
    // тот же самый семпл. div = 2 дал бы 375 Гц — медленнее контура.
    c.smplrt_div = 1;

    // DLPF, конфигурация 0: полоса 196.6 Гц для гироскопа и 246.0 Гц для
    // акселерометра (DS-000189 §7.1, таблицы GYRO_DLPFCFG/ACCEL_DLPFCFG).
    // Обе ниже частоты Найквиста для 562.5 Гц (281 Гц), то есть алиасинга
    // нет, а фазовая задержка минимальна из возможных. Более узкие полосы —
    // это уже тюнинг контура, который на этом этапе запрещён (ТЗ §33).
    c.dlpf_cfg = 0;
    return c;
}

float icm20948_accel_lsb_per_g(icm20948_accel_fs_t fs) {
    // DS-000189 §3.2 «Accelerometer Sensitivity Scale Factor».
    switch (fs) {
    case ICM20948_ACCEL_FS_2G:
        return 16384.0f;
    case ICM20948_ACCEL_FS_4G:
        return 8192.0f;
    case ICM20948_ACCEL_FS_8G:
        return 4096.0f;
    default:
        return 2048.0f;
    }
}

float icm20948_gyro_lsb_per_dps(icm20948_gyro_fs_t fs) {
    // DS-000189 §3.1 «Gyroscope Sensitivity Scale Factor».
    switch (fs) {
    case ICM20948_GYRO_FS_250DPS:
        return 131.0f;
    case ICM20948_GYRO_FS_500DPS:
        return 65.5f;
    case ICM20948_GYRO_FS_1000DPS:
        return 32.8f;
    default:
        return 16.4f;
    }
}

float icm20948_accel_fs_g(icm20948_accel_fs_t fs) {
    return 2.0f * (float) (1 << (int) fs);
}

float icm20948_gyro_fs_dps(icm20948_gyro_fs_t fs) {
    return 250.0f * (float) (1 << (int) fs);
}

float icm20948_odr_hz(uint8_t smplrt_div) {
    // DS-000189 §7.1: ODR = 1.125 кГц / (1 + SMPLRT_DIV).
    return 1125.0f / (1.0f + (float) smplrt_div);
}

const icm20948_config_t *icm20948_active_config(void) {
    return &D.cfg;
}

// ------------------------------------------------------------------ доступ

static esp_err_t reg_write(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {reg, value};
    return i2c_master_transmit(D.dev, tx, sizeof(tx), 100);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_transmit_receive(D.dev, &reg, 1, buf, len, 100);
}

// Банк выбирается битами [5:4] регистра 0x7F, доступного из любого банка.
static esp_err_t select_bank(uint8_t bank) {
    if (D.current_bank == bank) {
        return ESP_OK;
    }
    esp_err_t err = reg_write(REG_BANK_SEL, (uint8_t) (bank << 4));
    if (err == ESP_OK) {
        D.current_bank = bank;
    }
    return err;
}

// Восстановление залипшей шины.
//
// Ситуация штатная и воспроизводимая: если ESP32 сбрасывается посреди чтения,
// датчик остаётся в середине выдачи байта и удерживает SDA в нуле. Мастер
// после этого не может выдать даже START — контроллер сообщает
// «I2C hardware timeout» и GPIO помечаются как занятые. Наблюдалось на живой
// плате после сброса во время прогона (docs/icm20948_driver.md §6).
//
// Лечение стандартное: вручную выдать до девяти тактов SCL, чтобы ведомый
// дотолкал свой байт и отпустил SDA, затем сформировать условие STOP.
static void bus_recover(int sda, int scl) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << sda) | (1ULL << scl),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);
    gpio_set_level(sda, 1);
    gpio_set_level(scl, 1);
    esp_rom_delay_us(10);

    for (int i = 0; i < 9 && gpio_get_level(sda) == 0; ++i) {
        gpio_set_level(scl, 0);
        esp_rom_delay_us(5);
        gpio_set_level(scl, 1);
        esp_rom_delay_us(5);
    }

    // STOP: SDA переходит из 0 в 1 при высоком SCL.
    gpio_set_level(sda, 0);
    esp_rom_delay_us(5);
    gpio_set_level(scl, 1);
    esp_rom_delay_us(5);
    gpio_set_level(sda, 1);
    esp_rom_delay_us(5);

    // Вернуть пины в исходное состояние, чтобы драйвер I2C мог заново
    // назначить им функцию шины.
    gpio_reset_pin(sda);
    gpio_reset_pin(scl);
    esp_rom_delay_us(50);
}

esp_err_t icm20948_bus_init(const icm20948_config_t *cfg) {
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }
    if (D.bus) {
        // Повторная инициализация: сначала отпустить прежнюю шину, иначе
        // GPIO останутся занятыми и новый мастер не поднимется.
        if (D.dev) {
            i2c_master_bus_rm_device(D.dev);
        }
        i2c_del_master_bus(D.bus);
    }
    bus_recover(cfg->sda_gpio, cfg->scl_gpio);
    memset(&D, 0, sizeof(D));
    D.cfg = *cfg;
    D.current_bank = 0xFF;  // банк неизвестен, первый select_bank обязателен

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = cfg->sda_gpio,
        .scl_io_num = cfg->scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &D.bus);
    if (err != ESP_OK) {
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = cfg->i2c_addr,
        .scl_speed_hz = cfg->i2c_hz,
    };
    err = i2c_master_bus_add_device(D.bus, &dev_cfg, &D.dev);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t who = 0;
    err = icm20948_who_am_i(&who);
    if (err != ESP_OK) {
        return err;
    }
    return who == ICM20948_WHO_AM_I_VALUE ? ESP_OK : ESP_ERR_NOT_SUPPORTED;
}

esp_err_t icm20948_who_am_i(uint8_t *out) {
    esp_err_t err = select_bank(0);
    if (err != ESP_OK) {
        return err;
    }
    return reg_read(REG_WHO_AM_I, out, 1);
}

esp_err_t icm20948_init(const icm20948_config_t *cfg) {
    // Несколько попыток поднять шину.
    //
    // Одной попытки недостаточно: наблюдалось, что примерно одна загрузка из
    // трёх после сброса посреди обмена завершалась «I2C hardware timeout» и
    // предупреждением «GPIO 21 is not usable». Одного цикла восстановления
    // шины хватает не всегда — ведомый может отпускать SDA не с первой
    // серии тактов. Повтор с паузой решает это полностью (проверено пятью
    // подряд циклами сброса, docs/icm20948_driver.md §6).
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < 3; ++attempt) {
        err = icm20948_bus_init(cfg);
        if (err == ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (err != ESP_OK) {
        return err;
    }

    // 1. Аппаратный сброс. Гарантирует известное состояние независимо от
    //    того, что делал с датчиком предыдущий запуск прошивки.
    //    DS-000189 §7.1, PWR_MGMT_1.DEVICE_RESET.
    err = reg_write(REG_PWR_MGMT_1, PWR_MGMT_1_DEVICE_RESET);
    if (err != ESP_OK) {
        return err;
    }
    // Datasheet §10 «Power-on reset time»: 100 мс с запасом перекрывает
    // время внутренней инициализации.
    vTaskDelay(pdMS_TO_TICKS(100));
    D.current_bank = 0xFF;

    // 2. Снять сон и выбрать источник тактирования. CLKSEL = 1 — «auto select
    //    best available», рекомендованный datasheet режим (§7.1, PWR_MGMT_1):
    //    внутренний осциллятор 20 МГц уступает место PLL гироскопа, когда тот
    //    запустится. Значение 0 (internal 20 MHz) хуже по стабильности.
    err = reg_write(REG_PWR_MGMT_1, PWR_MGMT_1_CLKSEL_AUTO);
    if (err != ESP_OK) {
        return err;
    }
    // §10 «Gyroscope start-up time»: 35 мс до выхода на режим.
    vTaskDelay(pdMS_TO_TICKS(40));

    // 3. Включить все шесть осей. PWR_MGMT_2 = 0: ни один датчик не отключён.
    err = reg_write(REG_PWR_MGMT_2, 0x00);
    if (err != ESP_OK) {
        return err;
    }

    // 4. Шкалы, фильтры, частота выдачи — всё в USER BANK 2.
    err = select_bank(2);
    if (err != ESP_OK) {
        return err;
    }

    // ODR_ALIGN_EN = 1: выдача акселерометра и гироскопа выравнивается по
    // одному тактовому фронту (DS-000189 §7.2). Без этого два датчика
    // расходятся по фазе, и dt между парой значений перестаёт быть общим.
    err = reg_write(REG_ODR_ALIGN_EN, 0x01);
    if (err != ESP_OK) {
        return err;
    }

    // GYRO_CONFIG_1: [5:3] DLPFCFG, [2:1] FS_SEL, [0] FCHOICE.
    // FCHOICE = 1 включает цифровой ФНЧ; при 0 фильтр обходится и полоса
    // становится 12106 Гц — заведомо выше Найквиста, то есть алиасинг.
    uint8_t gyro_cfg = (uint8_t) (((D.cfg.dlpf_cfg & 0x07) << 3) |
                                  ((D.cfg.gyro_fs & 0x03) << 1) | 0x01);
    err = reg_write(REG_GYRO_CONFIG_1, gyro_cfg);
    if (err != ESP_OK) {
        return err;
    }
    err = reg_write(REG_GYRO_SMPLRT_DIV, D.cfg.smplrt_div);
    if (err != ESP_OK) {
        return err;
    }

    // ACCEL_CONFIG: та же раскладка полей.
    uint8_t accel_cfg = (uint8_t) (((D.cfg.dlpf_cfg & 0x07) << 3) |
                                   ((D.cfg.accel_fs & 0x03) << 1) | 0x01);
    err = reg_write(REG_ACCEL_CONFIG, accel_cfg);
    if (err != ESP_OK) {
        return err;
    }
    // Делитель акселерометра 12-битный и разложен на два регистра.
    err = reg_write(REG_ACCEL_SMPLRT_DIV_1, 0x00);
    if (err != ESP_OK) {
        return err;
    }
    err = reg_write(REG_ACCEL_SMPLRT_DIV_2, D.cfg.smplrt_div);
    if (err != ESP_OK) {
        return err;
    }

    err = select_bank(0);
    if (err != ESP_OK) {
        return err;
    }

    // 5. I2C master датчика (для внешнего магнитометра AK09916) не включаем:
    //    магнитометр Refloat не использует (imu_ref_callback получает mag =
    //    NULL), а лишний обмен занимал бы шину.
    err = reg_write(REG_USER_CTRL, 0x00);
    if (err != ESP_OK) {
        return err;
    }

    D.counter = 0;
    D.have_last = false;
    return ESP_OK;
}

esp_err_t icm20948_read(icm20948_sample_t *out) {
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }

    if (D.inject_fail > 0) {
        --D.inject_fail;
        ++D.stats.reads_failed;
        D.stats.last_error = ESP_ERR_TIMEOUT;
        D.stats.last_error_us = (uint64_t) esp_timer_get_time();
        out->valid = false;
        return ESP_ERR_TIMEOUT;
    }

    if (D.inject_frozen > 0 && D.have_last) {
        --D.inject_frozen;
        *out = D.last;
        out->timestamp_us = (uint64_t) esp_timer_get_time();
        out->sample_counter = ++D.counter;  // счётчик идёт, данные — нет
        ++D.stats.reads_ok;
        return ESP_OK;
    }

    int64_t t0 = esp_timer_get_time();
    uint8_t raw[14];
    esp_err_t err = select_bank(0);
    if (err == ESP_OK) {
        // 14 байт одной транзакцией: ACCEL_XOUT_H..GYRO_ZOUT_L и TEMP_OUT.
        // Разбивать нельзя — между чтениями датчик обновит регистры, и оси
        // окажутся из разных семплов.
        err = reg_read(REG_ACCEL_XOUT_H, raw, sizeof(raw));
    }
    int64_t t1 = esp_timer_get_time();

    if (err != ESP_OK) {
        ++D.stats.reads_failed;
        D.stats.last_error = err;
        D.stats.last_error_us = (uint64_t) t1;
        D.current_bank = 0xFF;  // после ошибки состояние банка неизвестно
        out->valid = false;
        return err;
    }

    uint32_t dur = (uint32_t) (t1 - t0);
    D.stats.sum_transaction_us += dur;
    if (dur > D.stats.max_transaction_us) {
        D.stats.max_transaction_us = dur;
    }
    ++D.stats.reads_ok;

    memset(out, 0, sizeof(*out));
    for (int i = 0; i < 7; ++i) {
        out->raw[i] = (int16_t) ((raw[i * 2] << 8) | raw[i * 2 + 1]);
    }
    float a_lsb = icm20948_accel_lsb_per_g(D.cfg.accel_fs);
    float g_lsb = icm20948_gyro_lsb_per_dps(D.cfg.gyro_fs);
    for (int i = 0; i < 3; ++i) {
        out->accel_g[i] = out->raw[i] / a_lsb;
        out->gyro_dps[i] = out->raw[3 + i] / g_lsb;
    }
    // DS-000189 §8.2: TEMP_degC = (TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity + 21,
    // где Temp_Sensitivity = 333.87 LSB/°C, RoomTemp_Offset = 0.
    out->temperature_c = out->raw[6] / 333.87f + 21.0f;
    out->timestamp_us = (uint64_t) t1;
    out->sample_counter = ++D.counter;
    out->valid = true;

    D.last = *out;
    D.have_last = true;
    return ESP_OK;
}

icm20948_stats_t icm20948_stats(void) {
    return D.stats;
}

void icm20948_inject_read_failures(int count) {
    D.inject_fail = count;
}

void icm20948_inject_frozen(int count) {
    D.inject_frozen = count;
}
