#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =========================================================
// [Config] 설정
// =========================================================
typedef struct {
    const char* dev_path; // 예: "/dev/ttyAMA1"
    int baudrate;         // 예: 115200 (무선 보드와 맞춰야 함)
} uartif_config_t;

// =========================================================
// [Functions] 송신 전용 API
// =========================================================

// 초기화 (UART 포트 열기 및 설정)
// 성공 시 0, 실패 시 -1
int UARTIF_init(const uartif_config_t* cfg, void (*rx_callback)(const uint8_t*, uint32_t));

// UART 포트 닫기
void UARTIF_stop(void);

// [Tx] Raw 데이터 전송 (wl_sender 모듈이 이걸 호출함)
// 성공 시 0, 실패 시 -1
int UARTIF_write_raw(const uint8_t* data, uint32_t len);

#ifdef __cplusplus
}
#endif