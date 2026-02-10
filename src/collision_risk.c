#define _GNU_SOURCE
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include "collision_risk.h"

// =========================================================
// 내부 변수
// =========================================================
static crm_config_t g_cfg;
static crm_callbacks_t g_cb;
static int g_inited = 0;

// 이전 상태 (상태 변경 감지용)
static dim_decision_state_t g_prev_state = DIM_STATE_NORMAL;

// 기본 설정값 (설정 안 넘어올 시)
static const crm_config_t default_cfg = {
    .ttc_warn_sec = 3.0f,
    .ttc_aeb_sec  = 1.5f,  // 1.5초 이내면 급정거
    .min_activ_speed_mps = 0.5f 
};

static uint32_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

// =========================================================
// [초기화]
// =========================================================
void CRM_init(const crm_config_t* cfg, const crm_callbacks_t* cb) {
    if (cfg) g_cfg = *cfg;
    else g_cfg = default_cfg;

    if (cb) g_cb = *cb;
    g_inited = 1;
    printf(" ========= COLLISION RISK MODULE STARTED (Warn:%.1fs, AEB:%.1fs =========)\n", 
           g_cfg.ttc_warn_sec, g_cfg.ttc_aeb_sec);
}

dim_decision_state_t CRM_get_state(void) {
    return g_prev_state;
}

// =========================================================
// [핵심] TTC 계산 함수 (복잡한 공식 적용)
// 공식: TTC = (-v_rel + sqrt(v_rel^2 + 2*a*d)) / a
// =========================================================
static float calculate_ttc(const dim_snapshot_t* s) {
    // 1. 필요한 변수 추출
    float d = s->ultra_dist_cm / 100.0f; // cm -> meter 변환
    float v_rel = s->rel_speed_mps;      // 상대 속도 (m/s)
    float a_ego = s->cur_accel_mps2;     // 내 가속도 (m/s^2)

    // [예외 1] 거리가 너무 멀거나(5m 이상), 측정 불가(0)면 안전
    if (d > 5.0f || d <= 0.0f) return 999.0f;

    // [예외 2] 상대 속도가 양수(멀어지는 중)면 충돌 안 함
    if (v_rel >= 0.0f) return 999.0f;

    // -----------------------------------------------------
    // [공식 적용] 가속도(a_ego)가 거의 0일 때와 아닐 때 구분
    // -----------------------------------------------------
    
    // Case A: 등속 운동 (가속도가 거의 0) -> 단순 공식 T = d / v
    if (fabsf(a_ego) < 0.1f) {
        // v_rel이 음수(다가옴)이므로 절대값 사용
        return d / fabsf(v_rel);
    }

    // Case B: 가속 운동 (공식 적용)
    // 판별식 D = v_rel^2 + 2*a*d
    float D = (v_rel * v_rel) + (2.0f * a_ego * d);

    // D < 0 이면 허수(충돌 해가 없음 -> 안전)
    if (D < 0) return 999.0f;

    float sqrt_D = sqrtf(D);

    // 근의 공식 분자 2개 (-b ± sqrt(D))
    float t1 = (-v_rel + sqrt_D) / a_ego;
    float t2 = (-v_rel - sqrt_D) / a_ego;

    // 둘 중 양수이면서 더 작은 값(가장 빠른 충돌 시간)을 선택
    float ttc = 999.0f;
    
    if (t1 > 0 && t2 > 0) ttc = (t1 < t2) ? t1 : t2;
    else if (t1 > 0) ttc = t1;
    else if (t2 > 0) ttc = t2;
    
    return ttc;
}

// =========================================================
// [Main Loop] 주기적으로 실행
// =========================================================
void CRM_run_step(void) {
    if (!g_inited) return;

    // 1. 데이터 가져오기 (Snapshot)
    dim_snapshot_t s;
    if (DIM_get_snapshot(&s) != 0) return;

    // 2. TTC 계산
    float ttc = calculate_ttc(&s);
    
    // 3. 상태 결정 (FSM Lite)
    dim_decision_state_t next_state = DIM_STATE_NORMAL;
    uint32_t age_obj_ms = now_ms() - s.ts_obj_ms;
    dim_obj_type_t effective_obj = (age_obj_ms > 300u) ? DIM_OBJ_NONE : s.obj_type;
    int allow_aeb = (effective_obj != DIM_OBJ_CONE);
    int force_critical = (effective_obj == DIM_OBJ_OBSTACLE || effective_obj == DIM_OBJ_PERSON);

    
    // int allow_aeb = (s.obj_type != DIM_OBJ_CONE);
    // int force_critical = (s.obj_type == DIM_OBJ_OBSTACLE || s.obj_type == DIM_OBJ_PERSON);

    // 저속 주행 중이거나, 후진 중이면 AEB 무시 (노이즈 방지)
    if (s.cur_speed_mps < g_cfg.min_activ_speed_mps) {
        next_state = DIM_STATE_NORMAL;
    }

     else if (force_critical && ttc <= g_cfg.ttc_warn_sec) {
        next_state = DIM_STATE_CRITICAL;
    }

    else if (allow_aeb && ttc <= g_cfg.ttc_aeb_sec) {
        next_state = DIM_STATE_CRITICAL; // AEB !
    }

    else if (ttc <= g_cfg.ttc_warn_sec) {
        next_state = DIM_STATE_WARNING;  // 경고
    }

     // else if (ttc <= g_cfg.ttc_aeb_sec) {
    //     next_state = DIM_STATE_CRITICAL; // AEB !
    // }


    // else if (force_critical) {
    //     next_state = DIM_STATE_CRITICAL;
    // }
    
    else {
        next_state = DIM_STATE_NORMAL;
    }

    // 4. 상태 변경 시 동작 수행 (Edge Trigger)
    //    매번 보내야 안전하므로 Level Trigger로 변경함
    switch (next_state) {
        case DIM_STATE_NORMAL:
            if (g_cb.set_aeb_cmd)    g_cb.set_aeb_cmd(0); // 제어권 반환
            if (g_cb.set_brake_lamp) g_cb.set_brake_lamp(0);
            break;

        case DIM_STATE_WARNING:
            if (g_cb.set_aeb_cmd)    g_cb.set_aeb_cmd(0); // 아직 제어권은 유지
            if (g_cb.set_brake_lamp) g_cb.set_brake_lamp(1); // 브레이크등 깜빡
            printf("[CRM] WARNING! TTC: %.2f\n", ttc);
            break;

        case DIM_STATE_CRITICAL:
            if (g_cb.set_aeb_cmd)    g_cb.set_aeb_cmd(1); // ★ 제어권 뺏기 (강제 정지)
            if (g_cb.set_brake_lamp) g_cb.set_brake_lamp(2); // 브레이크등 점등
            if (g_cb.notify_accident) g_cb.notify_accident(1); // 필요 시 사고 알림
            printf("[CRM] !!!!! AEB ACTIVATED !!!!! TTC: %.2f\n", ttc);
            break;
    }

    if (g_cb.set_pretensioner) {
        // TTC가 유효하고(999 아님) 0.3초 이하이면 동작
        if (allow_aeb && ((force_critical && ttc <= g_cfg.ttc_warn_sec) || ttc <= 0.3f)){
            g_cb.set_pretensioner(1); // ON
        } else {
            g_cb.set_pretensioner(0); // OFF
        }
    }

    // 5. 결과 저장 (Dashboard가 볼 수 있게)
    DIM_set_decision(ttc, next_state);
    
    g_prev_state = next_state;
}