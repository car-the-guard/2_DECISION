#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include "collision_response.h"

// ----------------- 내부 유틸 -----------------
static uint32_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

// 각도 차이 계산 (0~359)
static uint16_t ang_diff(uint16_t a, uint16_t b) {
    uint16_t d = (a > b) ? (a - b) : (b - a);
    return (d > 180) ? (360 - d) : d;
}

// ----------------- 전역 변수 -----------------
static cresp_config_t g_cfg;
static cresp_callbacks_t g_cb;
static int g_inited = 0;
static int g_running = 0;

static pthread_t g_thr;
static pthread_mutex_t g_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cv  = PTHREAD_COND_INITIALIZER;

// 데이터
static uint32_t g_last_crash_ms = 0;
static uint32_t g_collision_seq = 0;      // 생성된 이벤트 수
static uint32_t g_processed_seq = 0;      // 처리한 이벤트 수
static uint16_t g_cur_heading = 0;

// 모니터링 상태
static int      g_monitor_active = 0;
static uint16_t g_base_heading = 0;
static uint16_t g_max_swing = 0;
static uint32_t g_monitor_start_ms = 0;

// ----------------- 워커 스레드 -----------------
// 충돌 후 1초 동안의 상황을 지켜보는 역할
static void* worker_thread(void* arg) {
    (void)arg;
    
    pthread_mutex_lock(&g_mtx);

    while (g_running) {
        // 1. 충돌 이벤트 대기
        while (g_running && (g_collision_seq == g_processed_seq)) {
            pthread_cond_wait(&g_cv, &g_mtx);
        }
        if (!g_running) break;

        // 이벤트 접수
        g_processed_seq = g_collision_seq;
        uint32_t t_now = now_ms();

        // 쿨다운 체크 (중복 감지 방지)
        if (g_last_crash_ms > 0 && (t_now - g_last_crash_ms) < g_cfg.crash_cooldown_ms) {
            if(g_cfg.enable_stdout) printf("[CRESP] Crash ignored (Cooldown)\n");
            continue;
        }
        g_last_crash_ms = t_now;

        // -------------------------------------------------
        // [즉각 대응] 사고 발생 직후 바로 실행
        // -------------------------------------------------
        pthread_mutex_unlock(&g_mtx); // 콜백 호출 위해 잠시 언락

        if(g_cfg.enable_stdout) printf("[CRESP] !!! CRASH DETECTED !!! Initiating Response.\n");

        if (g_cb.set_brake_lamp) g_cb.set_brake_lamp(3);

        if (g_cb.notify_accident) g_cb.notify_accident(2);

        pthread_mutex_lock(&g_mtx); // 다시 락

        // -------------------------------------------------
        // [지연 대응] 1초간 차량 거동(회전/전복) 감시
        // -------------------------------------------------
        g_monitor_active = 1;
        g_base_heading = g_cur_heading;
        g_max_swing = 0;
        g_monitor_start_ms = now_ms();
        int severe_sent = 0;

        while (g_running && g_monitor_active) {
            uint32_t elapsed = now_ms() - g_monitor_start_ms;
            
            // 1초 경과 시 종료
            if (elapsed >= 1000) {
                g_monitor_active = 0;
                break;
            }

            // [조건 체크] 차량이 뺑글 돌았는가? (전복/스핀)
            if (!severe_sent && g_max_swing >= g_cfg.heading_swing_thresh_deg) {
                pthread_mutex_unlock(&g_mtx);
                
                // 대형 사고 알림 (Level 3)
                if(g_cfg.enable_stdout) printf("[CRESP] SEVERE ROLLOVER DETECTED!\n");
                if (g_cb.notify_accident) g_cb.notify_accident(3);
                // 브레이크등 계속 유지
                if (g_cb.set_brake_lamp) g_cb.set_brake_lamp(3);
                
                severe_sent = 1;
                pthread_mutex_lock(&g_mtx);
            }

            // 남은 시간만큼 대기 (헤딩 업데이트가 오면 깨어남)
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            
            uint32_t wait_ms = 1000 - elapsed;
            ts.tv_sec += wait_ms / 1000;
            ts.tv_nsec += (wait_ms % 1000) * 1000000L;
            if (ts.tv_nsec >= 1000000000L) {
                ts.tv_sec++;
                ts.tv_nsec -= 1000000000L;
            }
            
            pthread_cond_timedwait(&g_cv, &g_mtx, &ts);
        }
    }
    pthread_mutex_unlock(&g_mtx);
    return NULL;
}

// ----------------- Public API -----------------

int CRESP_init(const cresp_config_t* cfg, const cresp_callbacks_t* cb) {
    if (g_inited) return 0;
    
    if (cfg) g_cfg = *cfg;
    else {
        g_cfg.heading_swing_thresh_deg = 30;
        g_cfg.crash_cooldown_ms = 1000;
        g_cfg.enable_stdout = 1;
    }

    if (cb) g_cb = *cb;
    
    g_inited = 1;
    return 0;
}

int CRESP_start(void) {
    if (!g_inited) return -1;
    pthread_mutex_lock(&g_mtx);
    if (g_running) {
        pthread_mutex_unlock(&g_mtx);
        return 0;
    }
    g_running = 1;
    pthread_mutex_unlock(&g_mtx);

    if (pthread_create(&g_thr, NULL, worker_thread, NULL) != 0) {
        g_running = 0;
        return -1;
    }
    return 0;
}

void CRESP_stop(void) {
    pthread_mutex_lock(&g_mtx);
    if (!g_running) {
        pthread_mutex_unlock(&g_mtx);
        return;
    }
    g_running = 0;
    pthread_cond_broadcast(&g_cv);
    pthread_mutex_unlock(&g_mtx);
    pthread_join(g_thr, NULL);
}

// [Trigger] 충돌 발생! (CAN 0x08 수신 시 호출)
void CRESP_trigger_impact(void) {
    pthread_mutex_lock(&g_mtx);
    g_collision_seq++;
    pthread_cond_signal(&g_cv);
    pthread_mutex_unlock(&g_mtx);
}

// [Input] 헤딩 업데이트 (흔들림 감지용)
void CRESP_update_heading(uint16_t heading_deg) {
    pthread_mutex_lock(&g_mtx);
    g_cur_heading = heading_deg % 360;

    // 사고 감시 모드일 때만 계산
    if (g_monitor_active) {
        uint16_t diff = ang_diff(g_base_heading, g_cur_heading);
        if (diff > g_max_swing) {
            g_max_swing = diff;
        }
        // 값이 갱신되었으니 스레드 깨움 (바로 체크하도록)
        pthread_cond_signal(&g_cv);
    }
    pthread_mutex_unlock(&g_mtx);
}