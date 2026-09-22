#include "fc_gap_port.h"

#include "fc_can_bus.h"
#include "../drivers/icm20948.h"
#include "fc_log_port.h"
#include "fc_platform.h"

#include "../../../compat/diag/fc_gap_trace.h"
#include "../../../compat/imu/fc_imu_pipeline.h"
#include "../../../compat/motor/fc_dual_motor.h"
#include "../../../compat/safety/fc_motor_gate.h"
#include "../../../compat/safety/fc_supervisor.h"

#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#define ID_A 118u
#define ID_B 100u

static uint32_t node_age(const FcCanHealth *h, uint8_t id, uint64_t now) {
    const FcCanNode *n = fc_can_health_node(h, id);
    if (n == NULL || !n->ever_seen) {
        return 0xFFFFFFFFu;
    }
    uint64_t last = n->last_status_us > n->last_seen_us ? n->last_status_us : n->last_seen_us;
    return now > last ? (uint32_t) (now - last) : 0;
}

void fc_gap_port_capture(uint64_t now_us, uint64_t prev_us, uint32_t gap_us) {
    if (!fc_gap_trace_is_event(gap_us)) {
        return;
    }

    FcGapEvent e;
    memset(&e, 0, sizeof e);
    e.timestamp_us = now_us;
    e.prev_sample_us = prev_us;
    e.gap_us = gap_us;

    FcSupervisorStatus s = fc_supervisor_status();
    e.supervisor_state = (uint32_t) s.state;
    e.supervisor_faults = s.faults;
    e.supervisor_latched = s.faults_latched;

    FcMotorGateStats g = fc_motor_gate_stats();
    e.motor_allowed = g.allowed_by_policy;
    e.motor_sent = g.physically_sent;
    e.last_motor_command_us = g.last_request_us;
    e.last_command_a = g.last_value[FC_MOTOR_REQ_CURRENT];
    e.motor_backend_present = (fc_motor_gate_backend_name()[0] != 'n');
    e.operator_armed = fc_dual_motor_stats().armed;
    e.last_command_seq = (uint32_t) g.requests_total;

    FcCanStats cs = fc_can_bus_stats();
    e.can_rx_frames = cs.frames_total;
    e.can_bus_errors = cs.bus_error_count;
    e.can_err_tx = cs.tx_error_counter;
    e.can_err_rx = cs.rx_error_counter;
    e.can_bus_off = cs.bus_off_count;
    FcCanHealth h = fc_can_bus_health();
    e.node_age_us[0] = node_age(&h, ID_A, now_us);
    e.node_age_us[1] = node_age(&h, ID_B, now_us);

    FcImuPipelineStats ps = fc_imu_pipeline_stats();
    e.pipe_polls = ps.polls;
    e.pipe_accepted = ps.accepted;
    e.pipe_duplicates = ps.duplicates;
    e.pipe_rejected = ps.rejected;
    e.pipe_suspected_skips = ps.suspected_skips;

    icm20948_stats_t rt = icm20948_stats();
    e.i2c_reads_ok = rt.reads_ok;
    e.i2c_reads_failed = rt.reads_failed;
    e.i2c_last_transaction_us = rt.max_transaction_us;

    FcTimingStats tc = fc_timing_get(FC_TIMING_CONTROL);
    e.control_missed = tc.missed;
    e.control_late = tc.late;
    e.control_max_us = tc.max_period_us;

    FcLogPortStats ls = fc_log_port_stats();
    e.log_queue_depth = ls.pending;
    e.log_drops = ls.dropped;

    e.reset_reason = (uint32_t) esp_reset_reason();
    // Занятость flash программно не наблюдаема на этой платформе: признак
    // выставлен как «неизвестно», а не как «свободна». Разница существенная:
    // ведущая версия происхождения зазора в 110 мс — именно доступ к flash
    // с отключением кэша, и подменять незнание нулём здесь нельзя.
    // Спрашиваем у хранилища, а не подставляем false (ТЗ v0.9H §5). Именно
    // эта строчка раньше говорила «НЕИЗВЕСТНО» и оставляла версию про flash
    // непроверяемой.
    uint64_t flash_since = 0;
    e.nvs_busy = fc_storage_busy(&flash_since);
    e.nvs_busy_known = true;

    fc_gap_trace_capture(&e);
}

void fc_gap_port_fill_deferred(void) {
    int idx = fc_gap_trace_next_unfilled();
    if (idx < 0) {
        return;
    }
    fc_gap_trace_fill_deferred((uint32_t) idx, (uint32_t) heap_caps_get_free_size(MALLOC_CAP_8BIT),
                               (uint32_t) heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT),
                               fc_imu_rt_stack_watermark(), 0, 0);
}

void fc_gap_port_print(void) {
    FcGapTraceStats st = fc_gap_trace_stats();
    printf("трассировка зазоров IMU (порог %" PRIu32 " мкс)\n", st.threshold_us);
    printf("  событий           всего %" PRIu32 ", из них >20 мс %" PRIu32 "\n", st.events_total,
           st.events_over_20ms);
    printf("  худший зазор      %" PRIu32 " мкс", st.max_gap_us);
    if (st.max_gap_us) {
        printf(" на %llu мкс от старта", (unsigned long long) st.max_gap_at_us);
    }
    printf("\n  сохранено         %" PRIu32 " из %u, потеряно %" PRIu32 "\n", st.stored,
           (unsigned) FC_GAP_TRACE_SLOTS, st.dropped);

    for (uint32_t i = 0; i < fc_gap_trace_count(); ++i) {
        const FcGapEvent *e = fc_gap_trace_get(i);
        if (!e) {
            continue;
        }
        printf("--- событие #%" PRIu32 "  зазор %" PRIu32 " мкс  в %llu\n", e->seq, e->gap_us,
               (unsigned long long) e->timestamp_us);
        printf("    supervisor    state=%" PRIu32 " faults=0x%08" PRIx32 " latched=0x%08" PRIx32
               "\n",
               e->supervisor_state, e->supervisor_faults, e->supervisor_latched);
        printf("    мотор         armed=%d backend=%d allowed=%llu sent=%llu last=%.3f А seq=%"
               PRIu32 "\n",
               e->operator_armed, e->motor_backend_present,
               (unsigned long long) e->motor_allowed, (unsigned long long) e->motor_sent,
               (double) e->last_command_a, e->last_command_seq);
        printf("    CAN           кадров %llu ошибок %" PRIu32 " TX/RX %" PRIu32 "/%" PRIu32
               " BUS_OFF %" PRIu32 "\n",
               (unsigned long long) e->can_rx_frames, e->can_bus_errors, e->can_err_tx,
               e->can_err_rx, e->can_bus_off);
        printf("    узлы          118 %" PRIu32 " мкс назад, 100 %" PRIu32 " мкс назад\n",
               e->node_age_us[0], e->node_age_us[1]);
        printf("    тракт         опросов %llu принято %llu дубл %llu отвергнуто %llu "
               "подозрений %llu\n",
               (unsigned long long) e->pipe_polls, (unsigned long long) e->pipe_accepted,
               (unsigned long long) e->pipe_duplicates, (unsigned long long) e->pipe_rejected,
               (unsigned long long) e->pipe_suspected_skips);
        printf("    I2C           ok=%llu failed=%llu худшая %" PRIu32 " мкс\n",
               (unsigned long long) e->i2c_reads_ok, (unsigned long long) e->i2c_reads_failed,
               e->i2c_last_transaction_us);
        printf("    контур        missed=%" PRIu32 " late=%" PRIu32 " max=%" PRIu32 " мкс\n",
               e->control_missed, e->control_late, e->control_max_us);
        printf("    журнал        очередь %" PRIu32 ", потеряно %" PRIu32 "\n", e->log_queue_depth,
               e->log_drops);
        printf("    flash занят   %s\n", e->nvs_busy_known ? (e->nvs_busy ? "да" : "нет")
                                                           : "НЕИЗВЕСТНО (нет телеметрии)");
        printf("    reset reason  %" PRIu32 "\n", e->reset_reason);
        if (e->deferred_filled) {
            printf("    ПОСЛЕ события куча %" PRIu32 "/%" PRIu32 ", запас стека IMU %" PRIu32 "\n",
                   e->heap_free, e->heap_min, e->stack_imu);
        } else {
            printf("    дорогие поля  ещё не дописаны\n");
        }
    }
}
