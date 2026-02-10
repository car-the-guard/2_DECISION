#define _GNU_SOURCE
#include "can.h"
// [추가] 보안 유틸리티 헤더 포함 (MAC 계산용)
#include "can_security_utils.h" 

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
#include <time.h>

#define ACCEL_SCALE 1000.0f
#define REL_SPEED_SCALE 100.0f
#define SPEED_SCALE 100.0f

static int g_can_fd = -1;
static volatile int g_running = 0;
static pthread_t g_rx_thr;
static canif_rx_handlers_t g_rxh;
static int g_rx_thread_started = 0;

// [전역] 프리텐셔너 송신 카운터
static uint8_t g_pt_tx_counter = 0;

// [유틸] 현재시간(ms) 가져오기 (Timestamp용)
static uint16_t get_timestamp_u16() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint32_t ms = (ts.tv_sec * 1000) + (ts.tv_nsec / 1000000);
    return (uint16_t)(ms & 0xFFFF);
}

// [유틸] Big Endian 파싱 헬퍼
static uint16_t parse_be16(const uint8_t* d) {
    uint16_t val;
    memcpy(&val, d, sizeof(uint16_t));
    return be16toh(val);
}

static uint32_t parse_be32(const uint8_t* d) {
    uint32_t val;
    memcpy(&val, d, sizeof(uint32_t));
    return be32toh(val);
}
// [유틸] Big Endian 쓰기 헬퍼 (Tx용)
static void write_be16(uint8_t* d, uint16_t v) {
    uint16_t be_v = htobe16(v);
    memcpy(d, &be_v, sizeof(uint16_t));
}

static int open_can_socket(const char *ifname) {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    // if (s < 0) { perror("Socket"); return -1; }

    if (s < 0) {
        perror("[CAN] socket");
        return -1;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);

    // ioctl(s, SIOCGIFINDEX, &ifr);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("[CAN] ioctl(SIOCGIFINDEX)");
        close(s);
        return -1;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) { close(s); return -1; }

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[CAN] bind");
        close(s);
        return -1;
    }

    return s;
}

// =========================================================
// [Rx Thread] Big Endian 데이터 파싱
// =========================================================
static void* rx_thread(void* arg) {
    (void)arg;
    struct can_frame fr;
    
    while (g_running) {
        // 읽기는 Blocking으로 해도 됨 (별도 스레드니까)
        int n = read(g_can_fd, &fr, sizeof(fr));
        if (n < (int)sizeof(struct can_frame)) continue;

        switch (fr.can_id) {
            case CANID_COLLISION:
                if (g_rxh.on_collision && fr.can_dlc >= 1) 
                    g_rxh.on_collision(fr.data[0]);
                break;

            case CANID_ULTRASONIC:
                if (g_rxh.on_ultra && fr.can_dlc >= 2) 
                    g_rxh.on_ultra(parse_be16(&fr.data[0]));
                break;

            case CANID_ACCEL_FB:
                if (g_rxh.on_accel && fr.can_dlc >= 4) {
                    uint32_t raw = parse_be32(&fr.data[0]);
                    g_rxh.on_accel((float)raw / ACCEL_SCALE);
                }
                break;

            case CANID_REL_SPEED:
                if (g_rxh.on_rel_speed && fr.can_dlc >= 4) {
                    int32_t raw = (int32_t)parse_be32(&fr.data[0]);
                    g_rxh.on_rel_speed((float)raw / REL_SPEED_SCALE);
                }
                break;

            case CANID_AI_OBJ:
                if (g_rxh.on_ai_obj && fr.can_dlc >= 1)
                    g_rxh.on_ai_obj(fr.data[0]);
                break;

            case CANID_SPEED_FB:
                if (g_rxh.on_speed && fr.can_dlc >= 4) {
                    uint32_t raw = parse_be32(&fr.data[0]);
                    g_rxh.on_speed((float)raw / SPEED_SCALE);
                }
                break;
            
            case CANID_AI_LANE:
                if (g_rxh.on_ai_lane && fr.can_dlc >= 1)
                    g_rxh.on_ai_lane(fr.data[0]);
                break;

            case CANID_HEADING:
                if (g_rxh.on_heading && fr.can_dlc >= 2)
                    g_rxh.on_heading(parse_be16(&fr.data[0]));
                break;
        }
    }
    return NULL;
}

// =========================================================
// [Tx Functions] 
// =========================================================

// 공통 전송 로직 (Internal)
static int send_raw_frame(struct can_frame* fr) {
    if (g_can_fd < 0) return -1;

    // MSG_DONTWAIT: 버퍼 꽉 찼을 때 스레드 멈춤 방지
    int ret = send(g_can_fd, fr, sizeof(struct can_frame), MSG_DONTWAIT);
    
    if (ret < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // 버퍼가 꽉 참 -> 패킷 드랍
            return 0; 
        }
        perror("[CAN TX] Error");
        return -1;
    }
    return ret;
}

// 기존 일반 메시지 전송용 (Data 0~3 페이로드, 4~5 타임스탬프)
static int send_frame(uint16_t id, const uint8_t* payload, uint8_t dlc) {
    struct can_frame fr;
    memset(&fr, 0, sizeof(fr));
    fr.can_id = id;
    fr.can_dlc = 8;

    if (dlc > 4) dlc = 4; 
    memcpy(fr.data, payload, dlc);
    write_be16(&fr.data[4], get_timestamp_u16());
    fr.data[7] = 0x00; // 일반 메시지는 보안 필드 없음

    return send_raw_frame(&fr);
}

// [신규] 프리텐셔너 전송 (보안 적용: Timestamp + Counter + MAC)
int CANIF_send_pretension(uint8_t cmd) {
    struct can_frame fr;
    memset(&fr, 0, sizeof(fr));
    
    fr.can_id = CANID_PRETENSION;
    fr.can_dlc = 8;

    // 1. Data[0]: 명령어 (0x00: OFF, 0xFF: ON)
    fr.data[0] = cmd;
    // Data[1]~[3]은 0x00 (Reserved)

    // 2. Data[4]~[5]: 타임스탬프 (Big Endian)
    write_be16(&fr.data[4], get_timestamp_u16());

    // 3. Data[6]: 카운터
    fr.data[6] = g_pt_tx_counter;

    // 4. Data[7]: MAC 계산
    // (Data[0]~Data[6]까지 7바이트 내용을 바탕으로 계산)
    fr.data[7] = compute_mac(fr.data, 7, g_pt_tx_counter);

    // 5. 전송
    int ret = send_raw_frame(&fr);

    if (ret > 0) {
        // 전송 성공 시에만 카운터 증가 (0~255 순환)
        g_pt_tx_counter++;
        
        // 디버깅용 로그 (필요시 주석 해제)
        // printf("[PT-TX] CMD:0x%02X, Cnt:%d, MAC:0x%02X\n", cmd, fr.data[6], fr.data[7]);
    }

    return ret;
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
    if (pthread_create(&g_rx_thr, NULL, rx_thread, NULL) != 0) {
        g_running = 0;
        return -1;
    }
    g_rx_thread_started = 1;
    return 0;
}

void CANIF_stop(void) {
    g_running = 0;
    // if (g_can_fd >= 0) close(g_can_fd);

    if (g_can_fd >= 0) {
        close(g_can_fd);
        g_can_fd = -1;
    }
    
    if (g_rx_thread_started) {
        pthread_join(g_rx_thr, NULL);
        g_rx_thread_started = 0;
    }
}