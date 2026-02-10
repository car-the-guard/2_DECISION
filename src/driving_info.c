#define _GNU_SOURCE
#include <string.h>
#include <time.h>
#include "driving_info.h"

static pthread_rwlock_t g_rw = PTHREAD_RWLOCK_INITIALIZER;
static dim_snapshot_t g_s; 

static uint32_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

int DIM_init(void) {
    pthread_rwlock_init(&g_rw, NULL);
    pthread_rwlock_wrlock(&g_rw);
    memset(&g_s, 0, sizeof(g_s));
    g_s.calc_ttc_sec = 999.0f; 
    g_s.decision = DIM_STATE_NORMAL;
    g_s.ts_lane_ms = now_ms();
    g_s.ts_obj_ms = now_ms();
    pthread_rwlock_unlock(&g_rw);
    return 0;
}

void DIM_deinit(void) {
    pthread_rwlock_destroy(&g_rw);
}

// [NEW] 개별 업데이트 구현
void DIM_update_speed(float mps) {
    pthread_rwlock_wrlock(&g_rw);
    g_s.cur_speed_mps = mps;
    g_s.ts_common_ms = now_ms();
    pthread_rwlock_unlock(&g_rw);
}

void DIM_update_accel(float mps2) {
    pthread_rwlock_wrlock(&g_rw);
    g_s.cur_accel_mps2 = mps2;
    pthread_rwlock_unlock(&g_rw);
}

void DIM_update_rel_speed(float mps) {
    pthread_rwlock_wrlock(&g_rw);
    g_s.rel_speed_mps = mps;
    pthread_rwlock_unlock(&g_rw);
}

void DIM_update_ultra(float cm) {
    pthread_rwlock_wrlock(&g_rw);
    g_s.ultra_dist_cm = cm;
    pthread_rwlock_unlock(&g_rw);
}

void DIM_update_heading(uint16_t deg) {
    pthread_rwlock_wrlock(&g_rw);
    g_s.heading_deg = deg;
    pthread_rwlock_unlock(&g_rw);
}

void DIM_update_lane(dim_lane_t lane) {
    pthread_rwlock_wrlock(&g_rw);
    g_s.lane = lane;
    g_s.ts_lane_ms = now_ms();
    pthread_rwlock_unlock(&g_rw);
}

void DIM_update_obj_type(dim_obj_type_t obj_type) {
    pthread_rwlock_wrlock(&g_rw);
    g_s.obj_type = obj_type;
    g_s.ts_obj_ms = now_ms();
    pthread_rwlock_unlock(&g_rw);
}

void DIM_set_decision(float ttc, dim_decision_state_t state) {
    pthread_rwlock_wrlock(&g_rw);
    g_s.calc_ttc_sec = ttc;
    g_s.decision = state;
    pthread_rwlock_unlock(&g_rw);
}

int DIM_get_snapshot(dim_snapshot_t* out) {
    if (!out) return -1;
    pthread_rwlock_rdlock(&g_rw);
    *out = g_s;
    pthread_rwlock_unlock(&g_rw);
    return 0;
}