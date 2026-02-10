/* src/main.c */
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
#include "driving_info.h"
#include "can.h"
#include "uart.h"
#include "bluetooth.h"
#include "collision_risk.h"
#include "collision_response.h"
#include "wl_sender.h"

// =========================================================
// [Global Control]
// =========================================================
static volatile int g_running = 1;
static int g_aeb_active = 0;
static int g_brake_level_crm = 0;
static int g_brake_level_cresp = 0;
static pthread_mutex_t g_brake_lock = PTHREAD_MUTEX_INITIALIZER;

// [메모장] 현재 운전자 명령 저장용 (전역 변수)
static canif_motor_cmd_t g_current_cmd = {0, 0, 0, 0};
static struct timespec g_last_bt_time = {0, 0}; // 마지막 수신 시간
static pthread_mutex_t g_cmd_lock = PTHREAD_MUTEX_INITIALIZER;

static void sig_handler(int sig) {
    (void)sig;
    g_running = 0;
}

void on_wireless_data(const uint8_t* data, uint32_t len) {
    if (len < 4) return; // 헤더보다 작으면 무시

    uint8_t type = data[0]; // 첫 바이트가 Type

    // WL-2 (외부 사고 정보) 수신!
    if (type == 2) { 
        uint8_t severity = data[4]; // Payload 첫 바이트가 위험도라고 가정
        
        printf("[WL] External Accident Received! Level: %d\n", severity);
        
        // [대응 로직] 외부 사고 발생 시 -> 감속 or 경고
        if (severity >= 2) {
             // 예: CRM 모듈에게 "외부 위험상황" 알리기
             // CRM_notify_external_risk(severity); (이런 함수를 만들어서 호출)
        }
    }
}

// =========================================================
// [Bluetooth Callbacks] 수신 즉시 1발 발사! (One-Shot)
// =========================================================
static void on_bt_cmd(const char* cmd) {
    char move_dir = 'F'; 
    int speed_val = 0;
    char steer_dir = 'R'; 
    int angle_val = 0;

    int parsed = sscanf(cmd, " %c%d %c%d", &move_dir, &speed_val, &steer_dir, &angle_val);
    if (parsed < 2) return;
    if (parsed < 4) { steer_dir = 'R'; angle_val = 0; }

    // 임시 구조체 생성
    canif_motor_cmd_t m = {0, 0, 0, 0};
    if (move_dir == 'B') m.bwd = (uint8_t)speed_val;
    else m.fwd = (uint8_t)speed_val;
    if (steer_dir == 'L') m.left = (uint8_t)angle_val;
    else m.right = (uint8_t)angle_val;

    pthread_mutex_lock(&g_cmd_lock);
    
    // [1] 값이 바뀌었는지 확인
    int is_changed = (m.fwd != g_current_cmd.fwd || m.bwd != g_current_cmd.bwd || 
                      m.left != g_current_cmd.left || m.right != g_current_cmd.right);

    // [2] 전역 변수 업데이트 (메모장 기록)
    g_current_cmd = m;
    clock_gettime(CLOCK_MONOTONIC, &g_last_bt_time); // 시간 갱신
    
    pthread_mutex_unlock(&g_cmd_lock);

    // [3] 값이 바뀌었다면? -> 여기서 즉시 "딱 1발" 발사!
    if (is_changed && !g_aeb_active) {
        CANIF_send_motor_cmd(&m);
        // printf("[BT] Change! Sent 1 packet.\n");
    }
}

// =========================================================
// [CAN & Sensor Callbacks] (기존 유지)
// =========================================================
static void cb_can_ultrasonic(uint16_t dist_cm) { DIM_update_ultra((float)dist_cm); }
static void cb_can_speed(float speed_mps) { DIM_update_speed(speed_mps); }
static void cb_can_accel(float accel_mps2) { DIM_update_accel(accel_mps2); }
static void cb_can_rel_speed(float rel_mps) { DIM_update_rel_speed(rel_mps); }
static void cb_can_heading(uint16_t deg) { DIM_update_heading(deg); CRESP_update_heading(deg); }
// static void cb_can_ai_lane(uint8_t lane) { DIM_update_lane((dim_lane_t)lane); }
// static void cb_can_ai_obj(uint8_t obj_type) { DIM_update_obj_type((dim_obj_type_t)obj_type); }
static void cb_can_ai_lane(uint8_t lane) { DIM_update_lane((dim_lane_t)lane); }
static void cb_can_ai_obj(uint8_t obj_type) { DIM_update_obj_type((dim_obj_type_t)obj_type); }

static void cb_can_collision_detected(uint8_t is_crash) {
    (void)is_crash;
    printf("[Main] !!! PHYSICAL IMPACT DETECTED !!!\n");
    CRESP_trigger_impact(); 
}

// static void cb_crm_set_brake_lamp(int level) {
//     canif_brake_mode_t current_mode = BRAKE_OFF;
//     if (level == 1) current_mode = BRAKE_ON;
//     else if (level >= 2) current_mode = BRAKE_BLINK;

//     static canif_brake_mode_t last_mode = BRAKE_OFF;
//     if (current_mode != last_mode) {
//         CANIF_send_brake_light(current_mode);
//         last_mode = current_mode;
//     }
// }

static void update_brake_lamp(void) {
    int chosen_level = g_brake_level_cresp > 0 ? g_brake_level_cresp : g_brake_level_crm;
    canif_brake_mode_t current_mode = BRAKE_OFF;

    if (chosen_level == 1) current_mode = BRAKE_ON;
    else if (chosen_level >= 2) current_mode = BRAKE_BLINK;

    static canif_brake_mode_t last_mode = BRAKE_OFF;
    if (current_mode != last_mode) {
        CANIF_send_brake_light(current_mode);
        last_mode = current_mode;
    }
}

static void cb_crm_set_brake_lamp(int level) {
    pthread_mutex_lock(&g_brake_lock);
    g_brake_level_crm = level;
    update_brake_lamp();
    pthread_mutex_unlock(&g_brake_lock);
}

static void cb_cresp_set_brake_lamp(int level) {
    pthread_mutex_lock(&g_brake_lock);
    g_brake_level_cresp = level;
    update_brake_lamp();
    pthread_mutex_unlock(&g_brake_lock);
}

static void cb_crm_set_aeb_cmd(int enable) {
    if (enable) {
        if (!g_aeb_active) {
            printf("[Main] AEB ENGAGED! (Force Stop)\n");
            g_aeb_active = 1;
            canif_motor_cmd_t stop = {0, 0, 0, 0};
            CANIF_send_motor_cmd(&stop);
            CANIF_send_aeb(1);
        }
    } else {
        if (g_aeb_active) {
            printf("[Main] AEB Released.\n");
            g_aeb_active = 0;
            CANIF_send_aeb(0);
        }
    }
}

static void cb_crm_set_pretensioner(int active) {
    static int last_pt_state = -1; // 초기값 -1로 설정하여 최초 1회 무조건 전송

    // 상태가 변경되었을 때만 CAN 메시지 전송 (버스 부하 방지)
    if (active != last_pt_state) {
        uint8_t cmd = active ? PT_CMD_ON : PT_CMD_OFF; // can.h에 정의된 매크로 사용 (0xFF / 0x00)
        CANIF_send_pretension(cmd);
        
        if (active) printf("[Main] Safety Belt Pretensioner ACTIVATED! (TTC <= 0.3s)\n");
        else        printf("[Main] Safety Belt Pretensioner OFF.\n");

        last_pt_state = active;
    }
}

static void cb_crm_notify_accident(int severity) { WL_send_accident((uint8_t)severity); }
static void cb_cresp_notify_accident(int severity) { WL_send_accident((uint8_t)severity); }

// =========================================================
// [MAIN]
// =========================================================
int main(int argc, char** argv) {
    (void)argc; (void)argv;
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    printf("\n========= D3-G Decision Board System Started =========\n");

    DIM_init();

    canif_rx_handlers_t can_cbs = {
        .on_ultra = cb_can_ultrasonic,
        .on_speed = cb_can_speed,
        .on_accel = cb_can_accel,
        .on_rel_speed = cb_can_rel_speed,
        .on_ai_obj = cb_can_ai_obj,
        .on_heading = cb_can_heading,
        .on_collision = cb_can_collision_detected,
        .on_ai_lane = cb_can_ai_lane
    };
    
    canif_config_t can_cfg = { .ifname = "can0" };
    if (CANIF_init(&can_cfg, &can_cbs) != 0) return -1;

    // WL 송수신용 포트 사용
    uartif_config_t wl_cfg = { .dev_path = "/dev/ttyAMA2", .baudrate = 9600 };
    if (UARTIF_init(&wl_cfg, on_wireless_data) != 0) {
        fprintf(stderr, "Wireless Board Init Failed\n");
        return -1;
    }

    // 블루투스 (ttyAMA1)는 정상 작동
    bt_config_t bt_cfg = { .uart_dev = "/dev/ttyAMA1", .baud = 9600 };
    if (BT_init(&bt_cfg, on_bt_cmd) != 0) fprintf(stderr, "BT Init Failed\n");

    crm_callbacks_t crm_cb = { .set_brake_lamp = cb_crm_set_brake_lamp,
        .set_aeb_cmd = cb_crm_set_aeb_cmd,
        .notify_accident = cb_crm_notify_accident,
        .set_pretensioner = cb_crm_set_pretensioner};

    CRM_init(NULL, &crm_cb);

    // cresp_callbacks_t cresp_cb = { .set_brake_lamp = cb_crm_set_brake_lamp, .notify_accident = cb_cresp_notify_accident };
    cresp_callbacks_t cresp_cb = { .set_brake_lamp = cb_cresp_set_brake_lamp, .notify_accident = cb_cresp_notify_accident };
    CRESP_init(NULL, &cresp_cb);

    // [중요 수정] UARTIF를 껐으므로 WL 모듈도 NULL로 초기화 (에러 방지)
    WL_init(UARTIF_write_raw); 

    CANIF_start();
    BT_start();
    CRESP_start();

    // printf("=== System Started. Loop Running. ===\n");

    uint32_t loop_count = 0;
    clock_gettime(CLOCK_MONOTONIC, &g_last_bt_time); // 초기 시간 설정

    while (g_running) {
        CRM_run_step();

        if (loop_count % 500 == 0) WL_send_direction(); // (NULL이라 아무 일도 안 함)

        if (loop_count % 5 == 0) {
            dim_snapshot_t s;
            DIM_get_snapshot(&s);

            printf("[DASH] SPD: %4.1f m/s | ACC: %5.2f m/s2 | SONAR: %5.1f cm | TTC: %5.1fs | HEADING: %3d deg\n", 
                   s.cur_speed_mps,    // 엔코더 속도
                   s.cur_accel_mps2,   // 가속도
                   s.ultra_dist_cm,    // 초음파 거리
                   s.calc_ttc_sec,      // TTC (충돌 예측 시간)
                   s.heading_deg
            );
            fflush(stdout); // 즉시 화면에 표시
        }

        // ----------------------------------------------------------------
        // [스마트 하트비트]
        // 값이 유지되고 있을 때(버튼 꾹)는 0.1초마다 보내주고,
        // 정지 상태(0,0,0,0)일 때는 아예 보내지 않음 (CAN 버스 침묵)
        // ----------------------------------------------------------------
        if (loop_count % 10 == 0) { // 10ms * 10 = 100ms 주기
            if (!g_aeb_active) {
                pthread_mutex_lock(&g_cmd_lock);
                
                // Watchdog (0.5초 동안 BT 수신 없으면 강제 초기화)
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                long long diff = (now.tv_sec - g_last_bt_time.tv_sec) * 1000 + 
                                 (now.tv_nsec - g_last_bt_time.tv_nsec) / 1000000;
                
                if (diff > 500) { 
                    g_current_cmd.fwd = 0; g_current_cmd.bwd = 0;
                    g_current_cmd.left = 0; g_current_cmd.right = 0;
                }
                
                canif_motor_cmd_t cmd_to_send = g_current_cmd;
                pthread_mutex_unlock(&g_cmd_lock);

                // [핵심 조건] 차가 움직여야 할 때만 주기적으로 보냄
                // (정지 상태일 때는 on_bt_cmd에서 마지막으로 1번 보내고 끝남)
                if (cmd_to_send.fwd > 0 || cmd_to_send.bwd > 0 || 
                    cmd_to_send.left > 0 || cmd_to_send.right > 0) {
                    
                    CANIF_send_motor_cmd(&cmd_to_send);
                }
            }
        }

        usleep(10000); // 10ms tick
        loop_count++;
    }

    CRESP_stop();
    BT_stop();
    // UARTIF_stop(); // 주석 처리됨
    CANIF_stop();
    DIM_deinit();
    return 0;
}