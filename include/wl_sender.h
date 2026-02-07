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

// // WL-4 전체 패킷 (8 Byte)
// typedef struct __attribute__((packed)) {
//     // uint32_t raw; // (주석: 비트 연산을 위해 32비트 통으로 관리 가능)
//     uint8_t  stx;           // 0xFD 고정 (Start of Text)
    
//     // --- Header (4 Bytes) ---
//     uint8_t  type;          // WL-X 번호 (WL-4 = 4)
//     uint8_t  reserved_pad;  // 바이트 패딩 (0x00)
//     uint16_t timestamp;     // 시간 측정용 타임스탬프 (ms)

//     // --- Payload (2 Bytes) ---
//     // Little Endian 기준: 먼저 선언된 필드가 하위 비트(LSB) 점유
//     // [비트 0~6] 하위 7비트 (Reserved)
//     uint16_t reserved  : 7; 
//     // [비트 7~15] 상위 9비트 (Direction)
//     uint16_t direction : 9; 

//     uint8_t  etx;           // 0xFE 고정 (End of Text)
// } wl4_packet_t;

// [수정] WL-4 패킷 (Union 적용)
typedef struct __attribute__((packed)) {
    uint8_t  stx;           // 0xFD
    uint8_t  type;          // 4
    uint8_t  reserved_pad;  // 0x00
    uint16_t timestamp;     // 2 Bytes
    
    // [핵심] Union을 사용하여 비트 필드와 Raw 값을 동시에 접근
    union {
        // 1. 편하게 값 넣기용 (비트 필드)
        struct {
            uint16_t reserved  : 7; // [LSB] 하위 7비트
            uint16_t direction : 9; // [MSB] 상위 9비트
        } bits;
        
        // 2. 엔디안 변환 및 전송용 (16비트 통짜)
        uint16_t raw; 
    } payload;

    uint8_t  etx;           // 0xFE
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