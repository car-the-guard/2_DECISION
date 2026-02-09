#pragma once
#include <stdint.h>
#include "driving_info.h" // 데이터 참조용

#ifdef __cplusplus
extern "C" {
#endif

// =========================================================
// [Config] 설정값 구조체
// =========================================================
typedef struct {
    float ttc_warn_sec;       // 경고 기준 (예: 3.0초)
    float ttc_aeb_sec;        // AEB 기준 (예: 2.3초)
    float min_activ_speed_mps;// AEB 작동 최소 속도 (저속 노이즈 방지)
} crm_config_t;

// =========================================================
// [Callbacks] 외부 제어 요청 (CAN 메시지 전송용)
// =========================================================
typedef struct {
    // 0:OFF, 1:Warn, 2:AEB, 3:Stop
    void (*set_brake_lamp)(int level);
    
    // 1:Enable(제어권 뺏기), 0:Disable
    void (*set_aeb_cmd)(int enable); 
    
    // 1:사고발생
    void (*notify_accident)(int level);

    void (*set_pretensioner)(int active);
} crm_callbacks_t;

// =========================================================
// [Functions]
// =========================================================

// 초기화
void CRM_init(const crm_config_t* cfg, const crm_callbacks_t* cb);

// [핵심] 주기적으로 호출되는 메인 로직 (10ms ~ 50ms 주기)
void CRM_run_step(void);

// 현재 상태 조회
dim_decision_state_t CRM_get_state(void);

#ifdef __cplusplus
}
#endif