#ifndef CAN_H
#define CAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =========================================================
// [1] CAN ID 정의 (src/can.c의 switch문과 일치)
// =========================================================
#define CANID_COLLISION   0x08  // 충돌 감지 (RX)
#define CANID_MOTOR_CMD   0x12  // 모터 제어 (TX)
#define CANID_AEB_CTRL    0x15  // AEB 제어 (TX)
#define CANID_BRAKE_LIGHT 0x16  // 브레이크등 제어 (TX)
#define CANID_ULTRASONIC  0x24  // 초음파 센서 (RX)
#define CANID_ACCEL_FB    0x28  // 가속도 피드백 (RX)
#define CANID_REL_SPEED   0x2C  // 상대 속도 (RX)
#define CANID_AI_OBJ      0x30  // AI 객체 인식 (RX)
#define CANID_SPEED_FB    0x38  // 현재 속도 (RX)
#define CANID_AI_LANE     0x80  // AI 차선 인식 (RX)
#define CANID_HEADING     0x84  // 지자기(방향) (RX)

// =========================================================
// [2] 구조체 정의
// =========================================================

// 브레이크등 모드
typedef enum {
    BRAKE_OFF = 0,
    BRAKE_ON = 1,
    BRAKE_BLINK = 2
} canif_brake_mode_t;

// 설정 구조체
typedef struct {
    const char* ifname; // 예: "can0"
} canif_config_t;

// 모터 명령 구조체
typedef struct {
    uint8_t fwd, bwd, left, right;
} canif_motor_cmd_t;

// [핵심] 수신 핸들러 (src/can.c에서 호출하는 모든 함수 포함)
typedef struct {
    void (*on_collision)(uint8_t is_crash);
    void (*on_ultra)(uint16_t dist_cm);
    void (*on_accel)(float mps2);
    void (*on_rel_speed)(float mps);
    void (*on_ai_obj)(uint8_t type);
    void (*on_speed)(float mps);
    void (*on_ai_lane)(uint8_t lane);
    void (*on_heading)(uint16_t deg);
} canif_rx_handlers_t;

// =========================================================
// [3] 함수 선언
// =========================================================

// 초기화 및 시작/종료
int  CANIF_init(const canif_config_t* cfg, const canif_rx_handlers_t* rxh);
int  CANIF_start(void);
void CANIF_stop(void);

// 전송 함수 (TX)
int CANIF_send_motor_cmd(const canif_motor_cmd_t* cmd);
int CANIF_send_aeb(uint8_t active);      // 1: ON, 0: OFF
int CANIF_send_brake_light(uint8_t mode); // 0: OFF, 1: ON, 2: BLINK

#ifdef __cplusplus
}
#endif

#endif // CAN_H