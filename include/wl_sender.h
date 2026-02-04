#pragma once
#include <stdint.h>
#include "driving_info.h" // 데이터 참조용

#ifdef __cplusplus
extern "C" {
#endif

// =========================================================
// [Packet Struct] 이미지 기반 구조체 정의 (Packed)
// =========================================================

// 공통 헤더 (4 Byte)
typedef struct __attribute__((packed)) {
    uint8_t  type;       // WL-X 번호
    uint8_t  reserved;   // Padding
    uint16_t timestamp;  // ms 단위
} wl_header_t;

// WL-3 Payload (20 Byte)
typedef struct __attribute__((packed)) {
    uint16_t direction;     // 지자기 방향
    uint8_t  lane;          // 차선 번호
    uint8_t  severity;      // 위험도
    uint64_t accident_id;   // 사고 ID
    uint64_t accident_time; // 사고 발생 시각
} wl3_payload_t;

// WL-3 전체 패킷 (24 Byte)
typedef struct __attribute__((packed)) {
    wl_header_t header;
    wl3_payload_t payload;
} wl3_packet_t;

// WL-4 Payload (2 Byte)
typedef struct __attribute__((packed)) {
    uint16_t direction;     // 지자기 방향
} wl4_payload_t;

// WL-4 전체 패킷 (6 Byte)
typedef struct __attribute__((packed)) {
    wl_header_t header;
    wl4_payload_t payload;
} wl4_packet_t;

// =========================================================
// [Functions]
// =========================================================

// UART 전송 함수 포인터 타입 (main.c에서 uart_write 함수 연결)
typedef int (*wl_send_fn)(const uint8_t* data, uint32_t len);

// 초기화 (전송 함수 연결)
int WL_init(wl_send_fn send_func);

// [WL-3] 사고 정보 전송 (CRM/CRESP에서 호출)
// severity: 1(주의), 2(사고), 3(대형사고)
void WL_send_accident(uint8_t severity);

// [WL-4] 주행 방향 주기적 전송 (필요 시 호출)
void WL_send_direction(void);

#ifdef __cplusplus
}
#endif