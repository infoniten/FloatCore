// Компактный периодический отчёт о безопасности (ТЗ v0.6A §19).
//
// Одна порция строк, которую можно взглядом сверить с ожиданиями: в каком
// состоянии супервизор, жив ли IMU, что с контуром, разрешена ли запись,
// сколько запросов тяги было и сколько из них ушло наружу.

#include "fc_platform.h"

#include "../../../compat/refloat_glue/refloat_facade.h"
#include "../../../compat/safety/fc_build_profile.h"
#include "../../../compat/safety/fc_imu_health.h"
#include "../../../compat/safety/fc_motor_gate.h"
#include "../../../compat/safety/fc_supervisor.h"

#include "esp_timer.h"

#include <inttypes.h>
#include <stdio.h>

void fc_print_safety_line(void) {
    FcSupervisorStatus sup = fc_supervisor_status();
    FcImuHealthStatus imu = fc_imu_health_status();
    FcMotorGateStats g = fc_motor_gate_stats();
    FcStorageStats st = fc_storage_stats();
    FcTimingStats loop = fc_timing_get(FC_TIMING_CONTROL);
    RefloatSnapshot rs = refloat_facade_snapshot();
    uint64_t now = (uint64_t) esp_timer_get_time();

    printf("SUPERVISOR   state=%s faults=%s(0x%08" PRIx32 ") latched=0x%08" PRIx32
           " переходов=%" PRIu32 "\n",
           fc_supervisor_state_name(sup.state), fc_supervisor_fault_name(sup.faults), sup.faults,
           sup.faults_latched, sup.transitions);
    printf("  profile    %s\n", FC_PROFILE_NAME);
    printf("  imu(hw)    %s age=%.1f ms ok=%llu errors=%llu stuck=%llu stale=%llu\n",
           fc_imu_health_state_name(imu.state),
           imu.last_good_us ? (double) (now - imu.last_good_us) / 1000.0 : -1.0,
           (unsigned long long) imu.samples_total, (unsigned long long) imu.read_errors,
           (unsigned long long) imu.stuck_events, (unsigned long long) imu.stale_events);
    printf("  loop       %.1f Hz p99=%" PRIu32 " us max=%" PRIu32 " us late=%" PRIu32
           " missed=%" PRIu32 " last_tick_age=%.1f ms\n",
           loop.iterations && loop.sum_period_us
               ? 1e6 / ((double) loop.sum_period_us / (double) loop.iterations)
               : 0.0,
           loop.p99_us, loop.max_period_us, loop.late, loop.missed,
           sup.last_loop_tick_us ? (double) (now - sup.last_loop_tick_us) / 1000.0 : -1.0);
    printf("  footpad    %s\n", refloat_facade_footpad_name(rs.footpad_state));
    printf("  cfg_write  %s (принято=%llu отклонено=%llu коммитов=%llu, последний %" PRIu32
           " мкс, худший %" PRIu32 " мкс)\n",
           fc_supervisor_config_write_allowed() ? "allowed" : "rejected",
           (unsigned long long) st.writes_accepted, (unsigned long long) st.writes_rejected,
           (unsigned long long) st.commits_done, st.last_commit_us, st.max_commit_us);
    printf("  motor      requested=%llu rejected(disarmed=%llu fault=%llu invalid=%llu) "
           "allowed=%llu sent=%llu backend=%s\n",
           (unsigned long long) g.requests_total, (unsigned long long) g.rejected_disarmed,
           (unsigned long long) g.rejected_fault, (unsigned long long) g.rejected_invalid,
           (unsigned long long) g.allowed_by_policy, (unsigned long long) g.physically_sent,
           fc_motor_gate_backend_name());
    printf("  can        %s\n", fc_can_backend_name());
}
