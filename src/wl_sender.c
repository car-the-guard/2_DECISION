// src/wl_sender.c
#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <endian.h>
#include "wl_sender.h" 

// =========================================================
// [Internal State] 전역 변수
// =========================================================

// [핵심 1] 전송 함수를 저장할 변수 선언 (이게 없어서 에러남)
static wl_send_fn g_send_fn = NULL;
static uint64_t g_acc_id_counter = 0; // 사고 ID 발급용

// ---------------- time utils ----------------
static uint32_t get_uptime_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

static uint64_t get_unix_time(void) {
    return (uint64_t)time(NULL);
}

// =========================================================
// [API] 초기화 (함수 포인터 저장)
// =========================================================
int WL_init(wl_send_fn send_func) {
    if (!send_func) return -1;
    
    // main.c에서 넘겨준 UART 전송 함수를 기억해둠
    g_send_fn = send_func;
    
    return 0;
}

// =========================================================
// [API] WL-3 사고 정보 전송
// =========================================================
void WL_send_accident(uint8_t severity) {
    // 저장된 전송 함수가 없으면 아무것도 못함 (안전장치)
    if (!g_send_fn) return;

    // 1. 최신 주행 데이터 가져오기 (Heading, Lane)
    dim_snapshot_t s;
    if (DIM_get_snapshot(&s) != 0) return;

    // 2. 패킷 생성
    // (구조체 정의는 헤더 파일에 있으므로 여기선 바로 사용 가능)
    wl3_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    // Header (Type 3)
    pkt.header.type = 3;
    pkt.header.reserved = 0; // 초기화 명시
    // pkt.header.timestamp = (uint16_t)(get_uptime_ms() & 0xFFFF);
    pkt.header.timestamp = htobe16((uint16_t)(get_uptime_ms() & 0xFFFF));

    // Payload
    // pkt.payload.direction = s.heading_deg;
    pkt.payload.direction = htobe16(s.heading_deg);
    pkt.payload.lane = (uint8_t)s.lane;
    pkt.payload.severity = severity;

    // pkt.payload.accident_id = ++g_acc_id_counter; // ID 증가
    // pkt.payload.accident_time = get_unix_time();  // 현재 시각
    pkt.payload.accident_id = htobe64(++g_acc_id_counter);
    pkt.payload.accident_time = htobe64(get_unix_time());

    // 3. 저장해둔 함수 포인터로 쏘기 (UARTIF_write_raw 직접 호출 X)
    g_send_fn((const uint8_t*)&pkt, sizeof(pkt));

    printf("[WL] Sent WL-3 (Accident) Sev:%d, ID:%lu, Size:%lu\n", 
           severity, g_acc_id_counter, sizeof(pkt));
}

// =========================================================
// [API] WL-4 주행 방향 전송 (주기적 호출용)
// =========================================================
void WL_send_direction(void) {
    if (!g_send_fn) return;

    // 1. 최신 Heading 가져오기
    dim_snapshot_t s;
    if (DIM_get_snapshot(&s) != 0) return;

    // 2. 패킷 생성 (신규 구조체 적용)
    wl4_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt)); // 0으로 초기화 (reserved 필드 등)

    // STX 설정
    pkt.stx = 0xFD;

    // Header 설정 (Flattened structure)
    pkt.type = 4;
    pkt.reserved_pad = 0;
    pkt.timestamp = htobe16((uint16_t)(get_uptime_ms() & 0xFFFF));

    // [핵심] 비트 필드 사용
    pkt.payload.bits.reserved = 0;
    pkt.payload.bits.direction = s.heading_deg; // 0~360 값이 9비트에 쏙 들어감

    // 엔디안 변환 (호스트 -> 빅엔디안)
    pkt.payload.raw = htobe16(pkt.payload.raw);

    // ETX 설정
    pkt.etx = 0xFE;

    // 3. 전송 (총 8바이트)
    g_send_fn((const uint8_t*)&pkt, sizeof(pkt));

    // 디버깅 (필요 시 주석 해제)
    // printf("[WL] Sent WL-4 (Dir) Deg:%d, Size:%lu\n", s.heading_deg, sizeof(pkt));
}