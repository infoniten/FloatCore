#include "fc_cpu_account.h"

void fc_cpu_window_begin(const FcCpuAccount *acc, FcCpuWindow *w, uint64_t now_us) {
    if (!acc || !w) {
        return;
    }
    w->begin_us = now_us;
    w->busy_at_begin = acc->busy_us;
    w->open = true;
}

bool fc_cpu_window_end(FcCpuAccount *acc, FcCpuWindow *w, uint64_t now_us, FcCpuSample *out) {
    if (!acc || !w || !w->open) {
        return false;
    }
    w->open = false;
    uint32_t wall = (uint32_t) (now_us - w->begin_us);
    // Разность берётся ДО собственного вклада, иначе окно учло бы само себя.
    uint64_t pre64 = acc->busy_us - w->busy_at_begin;
    uint32_t preempt = pre64 > wall ? wall : (uint32_t) pre64;
    uint32_t net = wall - preempt;
    acc->busy_us += net;
    if (out) {
        out->wall_us = wall;
        out->preempt_us = preempt;
        out->net_us = net;
    }
    return true;
}
