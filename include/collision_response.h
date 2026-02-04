#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =========================================================
// [Config] 설정값
// =========================================================
typedef struct {
    // 충돌 직후 1초 동안 방향이 이만큼 틀어지면 '전복/회전'으로 판단
    uint16_t heading_swing_thresh_deg;  // 예: 30도

    // 충돌 감지 쿨다운 (센서 노이즈 방지, ms)
    uint32_t crash_cooldown_ms;         // 예: 1000ms
    
    // 로그 출력 여부
    int enable_stdout;
} cresp_config_t;

// =========================================================
// [Callbacks] 외부로 나가는 신호 (CAN 등)
// =========================================================
typedef struct {
    // 브레이크등 제어 (Level 3: 비상 점멸)
    void (*set_brake_lamp)(int level);

    // 사고 상황 전파 (2:일반사고, 3:대형사고/전복)
    void (*notify_accident)(int severity_level);
} cresp_callbacks_t;

// =========================================================
// [Functions]
// =========================================================

// 초기화
int CRESP_init(const cresp_config_t* cfg, const cresp_callbacks_t* cb);

// 감시 스레드 시작/종료
int CRESP_start(void);
void CRESP_stop(void);

// [Input] 충돌 센서 감지 시 호출 (ISR 또는 메인 루프에서)
void CRESP_trigger_impact(void);

// [Input] 현재 헤딩 값 업데이트 (흔들림 감지용)
void CRESP_update_heading(uint16_t heading_deg);

#ifdef __cplusplus
}
#endif