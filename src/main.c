#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>

// [Modules]
#include "driving_info.h"       // 데이터 저장소 (DIM)
#include "can.h"                // CAN 통신 (CANIF)
#include "uart.h"     // WL 무선 통신 (UARTIF) - (헤더 파일명 확인 필요, 보통 uart_interface.h)
#include "bluetooth.h"          // 조이스틱 통신 (BT)
#include "collision_risk.h"     // 충돌 위험 판단 (CRM)
#include "collision_response.h" // 사고 대응 (CRESP)
#include "wl_sender.h"          // WL 패킷 전송 (WL)

// =========================================================
// [Global Control]
// =========================================================
static volatile int g_running = 1;
static int g_aeb_active = 0; // AEB 작동 중인지 체크 (BT 무시용)

static void sig_handler(int sig) {
    (void)sig;
    g_running = 0;
}

// =========================================================
// [1. CAN Callbacks] 센서 데이터 수신 -> DIM 저장
// =========================================================
static void cb_can_ultrasonic(uint16_t dist_cm) { 
    DIM_update_ultra((float)dist_cm); 
}

static void cb_can_speed(float speed_mps) {
    DIM_update_speed(speed_mps);
}

static void cb_can_accel(float accel_mps2) {
    DIM_update_accel(accel_mps2);
}

static void cb_can_rel_speed(float rel_mps) {
    DIM_update_rel_speed(rel_mps);
}

static void cb_can_heading(uint16_t deg) {
    DIM_update_heading(deg);
    CRESP_update_heading(deg); 
}

// [수정 포인트 1] 인자 추가 (void -> uint8_t is_crash)
// can.h의 정의와 맞춰야 합니다.
static void cb_can_collision_detected(uint8_t is_crash) {
    (void)is_crash; // 안 쓰더라도 인자는 받아야 함
    printf("[Main] !!! PHYSICAL IMPACT DETECTED !!!\n");
    CRESP_trigger_impact(); 
}

// =========================================================
// [2. CRM (Risk) Callbacks] 위험 판단 결과 -> 제어
// =========================================================
static void cb_crm_set_brake_lamp(int level) {
    // 1. 레벨에 따른 모드 결정
    canif_brake_mode_t current_mode = BRAKE_OFF;
    
    if (level == 1) current_mode = BRAKE_ON;           // Level 1: 켜짐
    else if (level >= 2) current_mode = BRAKE_BLINK;   // Level 2, 3: 깜빡임

    // 2. 상태 변경 체크 (중복 전송 방지)
    static canif_brake_mode_t last_mode = BRAKE_OFF;

    if (current_mode != last_mode) {
        CANIF_send_brake_light(current_mode);
        // printf("Brake Light: %d -> %d\n", last_mode, current_mode); // 디버깅용
        last_mode = current_mode;
    }
}

static void cb_crm_set_aeb_cmd(int enable) {
    if (enable) {
        if (!g_aeb_active) {
            printf("[Main] AEB ENGAGED! (Force Stop)\n");
            g_aeb_active = 1;
            canif_motor_cmd_t stop = {0, 0, 0, 0};
            CANIF_send_motor_cmd(&stop);
            // AEB 신호 전송 (추가)
            CANIF_send_aeb(1);
        }
    } else {
        if (g_aeb_active) {
            printf("[Main] AEB Released.\n");
            g_aeb_active = 0;
            // AEB 해제 전송
            CANIF_send_aeb(0);
        }
    }
}

static void cb_crm_notify_accident(int severity) {
    WL_send_accident((uint8_t)severity);
}

// =========================================================
// [3. CRESP (Response) Callbacks] 사고 발생 후 처리
// =========================================================
static void cb_cresp_notify_accident(int severity) {
    WL_send_accident((uint8_t)severity);
}

// =========================================================
// [4. Bluetooth Callbacks] 운전자 조작 -> 모터 제어
// =========================================================
// =========================================================
// [4. Bluetooth Callbacks] 점사(Burst) 전송 모드
// =========================================================
// static void on_bt_cmd(const char* cmd) {
//     // 1. AEB 작동 중이면 무시
//     if (g_aeb_active) return;

//     char move_dir = 'F'; 
//     int speed_val = 0;
//     char steer_dir = 'R'; 
//     int angle_val = 0;

//     int parsed = sscanf(cmd, " %c%d %c%d", &move_dir, &speed_val, &steer_dir, &angle_val);
//     if (parsed < 2) return;
//     if (parsed < 4) { steer_dir = 'R'; angle_val = 0; }

//     // 명령 패킷 생성
//     canif_motor_cmd_t m = {0, 0, 0, 0};
//     if (move_dir == 'B') m.bwd = (uint8_t)speed_val;
//     else m.fwd = (uint8_t)speed_val;
//     if (steer_dir == 'L') m.left = (uint8_t)angle_val;
//     else m.right = (uint8_t)angle_val;

//     // ---------------------------------------------------------
//     // [점사 로직] 값이 바뀌었을 때만 -> 5발 연속 발사!
//     // ---------------------------------------------------------
//     static canif_motor_cmd_t last_m = {0, 0, 0, 0};

//     // 1. 값이 이전과 다른지 확인
//     if (m.fwd != last_m.fwd || m.bwd != last_m.bwd || 
//         m.left != last_m.left || m.right != last_m.right) {
        
//         // 2. 값이 바뀌었으면 5번 반복해서 전송 (Burst)
//         // (10번은 위험하니 5~8번 추천)
//         for (int i = 0; i < 5; i++) {
//             CANIF_send_motor_cmd(&m);
            
//             // [중요] 너무 빨리 쏘면 SPI가 체할 수 있으니 
//             // 패킷 사이에 아주 미세한 숨돌릴 틈(0.002초)을 줍니다.
//             usleep(2000); 
//         }

//         // 3. (옵션) 디버깅 로그 - 5발 쐈다고 알려줌
//         // printf("[BT] Burst Send! (x5) Val: %d\n", speed_val);

//         // 4. 현재 상태 저장
//         last_m = m;
//     }
// }

static void on_bt_cmd(const char* cmd) {
    if (g_aeb_active) return;

    char move_dir = 'F'; 
    int speed_val = 0;
    char steer_dir = 'R'; 
    int angle_val = 0;

    int parsed = sscanf(cmd, " %c%d %c%d", &move_dir, &speed_val, &steer_dir, &angle_val);
    if (parsed < 2) return;
    if (parsed < 4) { steer_dir = 'R'; angle_val = 0; }

    canif_motor_cmd_t m = {0, 0, 0, 0};
    if (move_dir == 'B') m.bwd = (uint8_t)speed_val;
    else m.fwd = (uint8_t)speed_val;
    if (steer_dir == 'L') m.left = (uint8_t)angle_val;
    else m.right = (uint8_t)angle_val;

    // ---------------------------------------------------------
    // [하이브리드 로직] 점사(Burst) + 생존 신고(Heartbeat)
    // ---------------------------------------------------------
    static canif_motor_cmd_t last_m = {0, 0, 0, 0};
    static struct timespec last_time = {0, 0};

    // 1. 현재 시간 구하기
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    // 시간 차이 계산 (밀리초 ms 단위)
    long long diff_ms = (now.tv_sec - last_time.tv_sec) * 1000 + 
                        (now.tv_nsec - last_time.tv_nsec) / 1000000;

    // 2. 값이 바뀌었는지 확인
    int is_changed = (m.fwd != last_m.fwd || m.bwd != last_m.bwd || 
                      m.left != last_m.left || m.right != last_m.right);

    // 3. 차가 움직여야 하는 상태인지 확인 (정지 상태가 아님)
    int is_moving = (m.fwd > 0 || m.bwd > 0 || m.left > 0 || m.right > 0);

    // =========================================================
    // Case A: 값이 바뀌었다! (버튼 누름 or 뗌) -> 5발 점사 (Burst)
    // =========================================================
    if (is_changed) {
        for (int i = 0; i < 5; i++) {
            CANIF_send_motor_cmd(&m);
            usleep(2000); // 2ms 간격
        }
        
        last_m = m;       // 상태 업데이트
        last_time = now;  // 시간 초기화
        // printf("[BT] Change! Burst 5 packets.\n");
    }
    // =========================================================
    // Case B: 값은 그대로인데, 누르고 있다! -> 0.2초마다 1발 (Heartbeat)
    // =========================================================
    else if (is_moving && diff_ms >= 200) { 
        // 200ms(0.2초)가 지났고, 차가 움직이는 중이라면 1발만 전송
        CANIF_send_motor_cmd(&m);
        
        last_time = now; // 시간 초기화
        // printf("[BT] Keep-alive (Heartbeat)...\n");
    }
    // =========================================================
    // Case C: 정지 상태이고 시간만 흐름 -> 아무것도 안 함 (침묵)
    // =========================================================
}

// =========================================================
// [5. AI UART Thread] 카메라 AI 정보 수신
// =========================================================
static void* ai_rx_thread(void* arg) {
    const char* dev = (const char*)arg;
    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) return NULL;

    struct termios tty;
    if(tcgetattr(fd, &tty) == 0) {
        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
        tty.c_iflag = 0; tty.c_oflag = 0; tty.c_lflag = 0;
        tty.c_cc[VMIN] = 1; tty.c_cc[VTIME] = 1;
        tcsetattr(fd, TCSANOW, &tty);
    }

    uint8_t buf[64];
    while (g_running) {
        int n = read(fd, buf, sizeof(buf));
        if (n > 0) {
            if (buf[0] == 0xA1 && n >= 2) { 
                uint8_t lane = buf[1];
                if (lane >= 1 && lane <= 3) {
                    DIM_update_lane((dim_lane_t)lane);
                }
            }
        }
        usleep(10000);
    }
    close(fd);
    return NULL;
}

// =========================================================
// [MAIN]
// =========================================================
int main(int argc, char** argv) {
    (void)argc; (void)argv;
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    printf("\n=== D3-G Decision Board System Initializing ===\n");

    DIM_init();

    canif_rx_handlers_t can_cbs = {
        .on_ultra = cb_can_ultrasonic,
        .on_speed = cb_can_speed,
        .on_accel = cb_can_accel,
        .on_rel_speed = cb_can_rel_speed,
        .on_heading = cb_can_heading,
        .on_collision = cb_can_collision_detected
    };
    
    // can.h를 사용하므로 설정
    canif_config_t can_cfg = { .ifname = "can0" };
    if (CANIF_init(&can_cfg, &can_cbs) != 0) return -1;

    uartif_config_t uart_cfg = { .dev_path = "/dev/ttyAMA0", .baudrate = 9600 };
    if (UARTIF_init(&uart_cfg) != 0) return -1;

    bt_config_t bt_cfg = { .uart_dev = "/dev/ttyAMA1", .baud = 9600 };
    if (BT_init(&bt_cfg, on_bt_cmd) != 0) fprintf(stderr, "BT Init Failed\n");

    crm_callbacks_t crm_cb = {
        .set_brake_lamp = cb_crm_set_brake_lamp,
        .set_aeb_cmd = cb_crm_set_aeb_cmd,
        .notify_accident = cb_crm_notify_accident
    };
    CRM_init(NULL, &crm_cb);

    cresp_callbacks_t cresp_cb = {
        .set_brake_lamp = cb_crm_set_brake_lamp,
        .notify_accident = cb_cresp_notify_accident
    };
    CRESP_init(NULL, &cresp_cb);

    WL_init(UARTIF_write_raw);

    CANIF_start();
    BT_start();
    CRESP_start();

    pthread_t ai_thr;
    pthread_create(&ai_thr, NULL, ai_rx_thread, (void*)"/dev/ttyAMA2");

    printf("=== System Started. Loop Running. ===\n");

    uint32_t loop_count = 0;
    
    while (g_running) {
        CRM_run_step();

        if (loop_count % 500 == 0) { 
            WL_send_direction();
        }

        if (loop_count % 5 == 0) {
            dim_snapshot_t s;
            DIM_get_snapshot(&s);
            printf("엔코더 :%4.1fm/s | 상대 속도 :%4.1fm/s | 가속도 :%4.1fm/s^2 | 초음파 :%4.1fcm | TTC :%5.1fs | AEB :%d | 방향 :%3d\n", 
                   s.cur_speed_mps,    // 1. 현재 속도 (엔코더 기반)
                   s.rel_speed_mps,    // 2. 상대 속도 (TTC 계산의 핵심)
                   s.cur_accel_mps2,   // 3. 가속도 (감속 중인지 확인용)
                   s.ultra_dist_cm,    // 4. 거리
                   s.calc_ttc_sec,     // 5. 최종 계산된 TTC
                   g_aeb_active,       // 6. AEB 발동 상태 (0 or 1)
                   s.heading_deg);     // 7. 차량 방향
        }

        usleep(10000); 
        loop_count++;
    }

    CRESP_stop();
    BT_stop();
    UARTIF_stop();
    CANIF_stop();
    DIM_deinit();
    pthread_join(ai_thr, NULL);

    return 0;
}