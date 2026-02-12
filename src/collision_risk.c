#define _GNU_SOURCE
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include "collision_risk.h"

// =========================================================
// 설정 및 내부 변수
// =========================================================
static crm_config_t g_cfg;
static crm_callbacks_t g_cb;
static int g_inited = 0;

// [핵심] WARNING 상태 유지 시간 (1000ms = 1초)
static const uint32_t MIN_WARN_DURATION_MS = 1000; 
static uint32_t g_warn_start_ms = 0; // WARNING 진입 시점 기록
// [추가] WARNING 진입 후 1초 뒤 강제로 브레이크등을 CRITICAL로 변경
static const uint32_t WARN_FORCE_TO_CRIT_MS = 1000;
static uint32_t g_warn_force_start_ms = 0;
static int g_warn_force_active = 0;

static dim_decision_state_t g_prev_state = DIM_STATE_NORMAL;
static const float MIN_CLOSING_REL_SPEED_MPS = 0.10f;

static const crm_config_t default_cfg = {
    .ttc_warn_sec = 1.5f,        // 경고 TTC 기준
    .ttc_aeb_sec  = 1.2f,        // 급제동 TTC 기준
    .min_activ_speed_mps = 1.0f  
};

static uint32_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

void CRM_init(const crm_config_t* cfg, const crm_callbacks_t* cb) {
    if (cfg) g_cfg = *cfg;
    else g_cfg = default_cfg;
    if (cb) g_cb = *cb;
    g_inited = 1;
    g_prev_state = DIM_STATE_NORMAL;
    g_warn_start_ms = 0;
    g_warn_force_active = 0;
    printf(" ========= CRM STARTED (Force Warn: %dms) =========\n", MIN_WARN_DURATION_MS);
}

dim_decision_state_t CRM_get_state(void) {
    return g_prev_state;
}

static float calculate_ttc(const dim_snapshot_t* s) {
    float d = s->ultra_dist_cm / 100.0f;
    float v_rel = s->rel_speed_mps;
    float a_ego = s->cur_accel_mps2;

    if (d > 5.0f || d <= 0.0f) return 999.0f;
    if (v_rel >= 0.0f || fabsf(v_rel) < MIN_CLOSING_REL_SPEED_MPS) return 999.0f;

    if (fabsf(a_ego) < 0.1f) return d / fabsf(v_rel);

    float D = (v_rel * v_rel) + (2.0f * a_ego * d);
    if (D < 0) return 999.0f;

    float sqrt_D = sqrtf(D);
    float t1 = (-v_rel + sqrt_D) / a_ego;
    float t2 = (-v_rel - sqrt_D) / a_ego;

    float ttc = 999.0f;
    if (t1 > 0 && t2 > 0) ttc = (t1 < t2) ? t1 : t2;
    else if (t1 > 0) ttc = t1;
    else if (t2 > 0) ttc = t2;
    
    return ttc;
}

// =========================================================
// [Main Loop] 
// =========================================================
void CRM_run_step(void) {
    if (!g_inited) return;

    dim_snapshot_t s;
    if (DIM_get_snapshot(&s) != 0) return;

    uint32_t now = now_ms();
    float ttc = calculate_ttc(&s);

    // 1. 주행 여부 판단 (Critical일 땐 멈출 때까지 강제 유지)
    int is_vehicle_moving = (s.cur_speed_mps >= g_cfg.min_activ_speed_mps);
    if (g_prev_state == DIM_STATE_CRITICAL) {
        is_vehicle_moving = 1;
    }
    if (!is_vehicle_moving) ttc = 999.0f;

    // 2. TTC 기반 '목표 상태' (Raw Target) 계산
    dim_decision_state_t raw_target = DIM_STATE_NORMAL;
    
    // 객체 유효성 체크
    uint32_t age_obj_ms = now - s.ts_obj_ms;
    dim_obj_type_t obj_type = (age_obj_ms > 500u) ? DIM_OBJ_NONE : s.obj_type;
    int allow_aeb = (obj_type != DIM_OBJ_CONE && obj_type != DIM_OBJ_NONE);

    if (allow_aeb && ttc <= g_cfg.ttc_aeb_sec) {
        raw_target = DIM_STATE_CRITICAL;
    } else if (ttc <= g_cfg.ttc_warn_sec) {
        raw_target = DIM_STATE_WARNING;
    } else {
        raw_target = DIM_STATE_NORMAL;
    }

    // =========================================================
    // [핵심] 상태 전이 로직 (1초 강제 지연)
    // =========================================================
    dim_decision_state_t next_state = g_prev_state;

    // Case 1: 현재 NORMAL 상태일 때
    if (g_prev_state == DIM_STATE_NORMAL) {
        if (raw_target == DIM_STATE_WARNING || raw_target == DIM_STATE_CRITICAL) {
            // 위험 감지되면 무조건 WARNING부터 시작
            next_state = DIM_STATE_WARNING;
            g_warn_start_ms = now; // 타이머 시작
            // Start one-shot force timer: after 1s force brake lamp -> CRITICAL
            g_warn_force_active = 1;
            g_warn_force_start_ms = now;
        } else {
            next_state = DIM_STATE_NORMAL;
            g_warn_start_ms = 0;
            g_warn_force_active = 0;
        }
    }
    // Case 2: 현재 WARNING 상태일 때
    else if (g_prev_state == DIM_STATE_WARNING) {
        uint32_t elapsed = now - g_warn_start_ms;

        // 아직 1초가 안 지났으면 -> 무조건 WARNING 유지
        if (elapsed < MIN_WARN_DURATION_MS) {
            next_state = DIM_STATE_WARNING;
            
            // (디버깅용) 로그 출력
            if (raw_target == DIM_STATE_CRITICAL) {
                // printf("[CRM] Holding WARNING... (elapsed: %dms)\n", elapsed);
            }
        } 
        // 1초 지났으면 -> 이제 진짜 목표 상태로 이동 허용
        else {
            next_state = raw_target; // CRITICAL이면 가고, NORMAL이면 감

            // [히스테리시스] NORMAL로 돌아갈 때 깜빡임 방지
            if (next_state == DIM_STATE_NORMAL && ttc <= (g_cfg.ttc_warn_sec + 0.5f)) {
                next_state = DIM_STATE_WARNING;
            }
        }
    }
    // Case 3: 현재 CRITICAL 상태일 때
    else if (g_prev_state == DIM_STATE_CRITICAL) {
        // 한 번 급제동 걸리면, 차량이 멈추거나 TTC가 아주 안전해질 때까지 유지
        // (여기서는 Raw Target이 Normal이 되어도 바로 풀리지 않게 하는 로직은 생략됨, 필요시 추가)
        next_state = raw_target; 
        
        // 간단한 래칭(Latching): 급제동 중에는 웬만하면 유지
        if (next_state != DIM_STATE_CRITICAL && s.cur_speed_mps > 0.1f) {
             // 차가 아직 구르고 있다면 계속 브레이크 잡기 (선택 사항)
             // next_state = DIM_STATE_CRITICAL; 
        }
    }

    // =========================================================
    // 동작 수행
    // =========================================================
    // Calculate elapsed for force timer
    uint32_t force_elapsed = 0;
    if (g_warn_force_active) force_elapsed = now - g_warn_force_start_ms;

    switch (next_state) {
        case DIM_STATE_NORMAL:
            if (g_cb.set_aeb_cmd)      g_cb.set_aeb_cmd(0);
            if (g_cb.set_brake_lamp)   g_cb.set_brake_lamp(0);
            if (g_cb.set_pretensioner) g_cb.set_pretensioner(0);
            break;

        case DIM_STATE_WARNING:
            if (g_cb.set_aeb_cmd)      g_cb.set_aeb_cmd(0);
            // If the 1s force timer is active, keep lamp=1 for the first
            // second then send lamp=2 (CRITICAL) regardless of state changes.
            if (g_warn_force_active) {
                if (force_elapsed < WARN_FORCE_TO_CRIT_MS) {
                    if (g_cb.set_brake_lamp) g_cb.set_brake_lamp(1);
                } else {
                    if (g_cb.set_brake_lamp) g_cb.set_brake_lamp(2);
                    g_warn_force_active = 0; // one-shot
                }
            } else {
                if (g_cb.set_brake_lamp)   g_cb.set_brake_lamp(1); // 경고등
            }

            if (g_prev_state != DIM_STATE_WARNING) {
                printf("[CRM] WARNING START! (One-shot force to CRIT after 1.0s) TTC: %.2f\n", ttc);
            }
            break;

        case DIM_STATE_CRITICAL:
            if (g_cb.set_aeb_cmd)      g_cb.set_aeb_cmd(1);
            if (g_cb.set_brake_lamp)   g_cb.set_brake_lamp(2); // 급제동등
            if (g_cb.set_pretensioner) g_cb.set_pretensioner(1);

            if (g_prev_state != DIM_STATE_CRITICAL) {
                printf("[CRM] !!! CRITICAL AEB !!! (After %dms hold) TTC: %.2f\n", 
                       now - g_warn_start_ms, ttc);
            }
            break;
    }

    DIM_set_decision(ttc, next_state);
    g_prev_state = next_state;
}