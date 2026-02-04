#define _GNU_SOURCE
#include "can.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <time.h> // for timestamp

static int g_can_fd = -1;
static volatile int g_running = 0;
static pthread_t g_rx_thr;
static canif_rx_handlers_t g_rxh;

// [유틸] 현재시간(ms) 가져오기 (Timestamp용)
static uint16_t get_timestamp_u16() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    // 하위 16비트만 사용
    uint32_t ms = (ts.tv_sec * 1000) + (ts.tv_nsec / 1000000);
    return (uint16_t)(ms & 0xFFFF);
}

// [유틸] Big Endian 파싱 헬퍼
static uint16_t parse_be16(const uint8_t* d) {
    return (uint16_t)((d[0] << 8) | d[1]);
}
static uint32_t parse_be32(const uint8_t* d) {
    return (uint32_t)((d[0] << 24) | (d[1] << 16) | (d[2] << 8) | d[3]);
}
// [유틸] Big Endian 쓰기 헬퍼 (Tx용)
static void write_be16(uint8_t* d, uint16_t v) {
    d[0] = (uint8_t)((v >> 8) & 0xFF);
    d[1] = (uint8_t)(v & 0xFF);
}

static int open_can_socket(const char *ifname) {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) { perror("Socket"); return -1; }
    struct ifreq ifr;
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ioctl(s, SIOCGIFINDEX, &ifr);
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) { close(s); return -1; }
    return s;
}

// =========================================================
// [Rx Thread] Big Endian 데이터 파싱
// =========================================================
static void* rx_thread(void* arg) {
    (void)arg;
    struct can_frame fr;
    
    while (g_running) {
        int n = read(g_can_fd, &fr, sizeof(fr));
        if (n < (int)sizeof(struct can_frame)) continue;

        // [디버그] ID 확인용 (필요시 주석 해제)
        // printf("[CAN RAW] ID: 0x%X DLC: %d\n", fr.can_id, fr.can_dlc);

        switch (fr.can_id) {
            case CANID_COLLISION: // 0x08 (Uint8)
                if (g_rxh.on_collision && fr.can_dlc >= 1) 
                    g_rxh.on_collision(fr.data[0]);
                break;

            case CANID_ULTRASONIC: // 0x24 (Uint16 BE)
                if (g_rxh.on_ultra && fr.can_dlc >= 2) 
                    g_rxh.on_ultra(parse_be16(&fr.data[0]));
                break;

            case CANID_ACCEL_FB: // 0x28 (Uint32 BE, Scale: 1000)
                printf("[DEBUG 0x28] Raw: %02X %02X %02X %02X\n", 
                   fr.data[0], fr.data[1], fr.data[2], fr.data[3]);
                if (g_rxh.on_accel && fr.can_dlc >= 4) {
                    uint32_t raw = parse_be32(&fr.data[0]);
                    g_rxh.on_accel((float)raw / 1000.0f);
                }
                break;

            case CANID_REL_SPEED: // 0x2C (Int32 BE, Scale: 100)
                if (g_rxh.on_rel_speed && fr.can_dlc >= 4) {
                    // Int32 캐스팅 주의
                    int32_t raw = (int32_t)parse_be32(&fr.data[0]);
                    g_rxh.on_rel_speed((float)raw / 100.0f);
                }
                break;

            case CANID_AI_OBJ: // 0x30 (Uint8)
                if (g_rxh.on_ai_obj && fr.can_dlc >= 1)
                    g_rxh.on_ai_obj(fr.data[0]);
                break;

            case CANID_SPEED_FB: // 0x38 (Uint32 BE, Scale: 100)
                printf("[DEBUG 0x38] Raw: %02X %02X %02X %02X\n", 
                   fr.data[0], fr.data[1], fr.data[2], fr.data[3]);
                if (g_rxh.on_speed && fr.can_dlc >= 4) {
                    uint32_t raw = parse_be32(&fr.data[0]);
                    g_rxh.on_speed((float)raw / 100.0f);
                    
                }
                break;
            
            case CANID_AI_LANE: // 0x80 (Uint8)
                if (g_rxh.on_ai_lane && fr.can_dlc >= 1)
                    g_rxh.on_ai_lane(fr.data[0]);
                break;

            case CANID_HEADING: // 0x84 (Uint16 BE)
                if (g_rxh.on_heading && fr.can_dlc >= 2)
                    g_rxh.on_heading(parse_be16(&fr.data[0]));
                break;
        }
    }
    return NULL;
}

// =========================================================
// [Tx Functions] 패킷 구조(Timestamp, CRC) 준수
// =========================================================
static int send_frame(uint16_t id, const uint8_t* payload, uint8_t dlc) {
    if (g_can_fd < 0) return -1;
    struct can_frame fr;
    memset(&fr, 0, sizeof(fr));
    fr.can_id = id;
    fr.can_dlc = 8; // 항상 8바이트 (프로토콜 준수)

    // Payload 복사 (최대 4바이트)
    if (dlc > 4) dlc = 4; 
    memcpy(fr.data, payload, dlc);

    // [4][5] Timestamp (Big Endian)
    write_be16(&fr.data[4], get_timestamp_u16());

    // [7] CRC (여기서는 Dummy 0x00, 필요시 계산 로직 추가)
    fr.data[7] = 0x00;

    return write(g_can_fd, &fr, sizeof(fr));
}

int CANIF_send_motor_cmd(const canif_motor_cmd_t* cmd) {
    uint8_t data[4];
    data[0] = cmd->fwd;
    data[1] = cmd->bwd;
    data[2] = cmd->left;
    data[3] = cmd->right;
    return send_frame(CANID_MOTOR_CMD, data, 4);
}

int CANIF_send_aeb(uint8_t active) {
    uint8_t data[1];
    // 활성: 0xFF(1111_1111), 비활성: 0x0F(0000_1111) - 표 기준
    data[0] = active ? 0xFF : 0x0F; 
    return send_frame(CANID_AEB_CTRL, data, 1);
}

int CANIF_send_brake_light(uint8_t mode) {
    uint8_t data[1];
    data[0] = mode;
    return send_frame(CANID_BRAKE_LIGHT, data, 1);
}

int CANIF_init(const canif_config_t* cfg, const canif_rx_handlers_t* rxh) {
    if (rxh) g_rxh = *rxh;
    g_can_fd = open_can_socket(cfg->ifname ? cfg->ifname : "can0");
    return (g_can_fd < 0) ? -1 : 0;
}

int CANIF_start(void) {
    if (g_can_fd < 0) return -1;
    if (g_running) return 0;
    g_running = 1;
    return pthread_create(&g_rx_thr, NULL, rx_thread, NULL);
}

void CANIF_stop(void) {
    g_running = 0;
    if (g_can_fd >= 0) close(g_can_fd);
    pthread_join(g_rx_thr, NULL);
}