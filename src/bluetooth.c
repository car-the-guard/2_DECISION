/* src/bluetooth.c */
#define _GNU_SOURCE
#include "bluetooth.h"
#include <pthread.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
// #include <poll.h> // poll 제거 (더 빠른 반응을 위해)

static bt_config_t g_cfg;
static bt_on_cmd_fn g_cb = NULL;
static pthread_t g_thr;
static volatile int g_running = 0;
static int g_fd = -1;
static int g_thread_started = 0;

// Baudrate 변환
static speed_t baud_to_speed(int baud) {
    switch(baud) {
        case 9600: return B9600;
        case 115200: return B115200;
        default: return B9600;
    }
}

// UART 장치 열기 (Blocking Mode로 변경)
static int internal_bt_open(const char *dev, int baud) {
    // [수정 1] O_NONBLOCK 제거! (데이터 올 때까지 운영체제가 깨워주길 기다림)
    int fd = open(dev, O_RDWR | O_NOCTTY); 
    if(fd < 0) return -1;

    struct termios t;
    if(tcgetattr(fd, &t) < 0) { close(fd); return -1; }

    // [수정 2] Raw 모드 설정 (cfmakeraw와 동일한 효과 + 흐름제어 끄기)
    t.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    t.c_oflag &= ~OPOST;
    t.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    t.c_cflag &= ~(CSIZE | PARENB);
    t.c_cflag |= CS8;
    
    // [수정 3] 하드웨어 흐름제어 끄기 (혹시 모를 딜레이 방지)
    t.c_cflag &= ~CRTSCTS; 

    cfsetospeed(&t, baud_to_speed(baud));
    cfsetispeed(&t, baud_to_speed(baud));

    // [핵심 수정 4] VMIN=1, VTIME=0
    // 의미: "최소 1바이트가 들어올 때까지 무한대기. 들어오면 즉시 리턴."
    t.c_cc[VMIN] = 1; 
    t.c_cc[VTIME] = 0; 

    if(tcsetattr(fd, TCSANOW, &t) < 0) { close(fd); return -1; }
    
    // 입력 버퍼 한 번 비우기 (찌꺼기 제거)
    tcflush(fd, TCIFLUSH);
    
    return fd;
}

// 수신 스레드
static void* bt_rx_thread(void* arg) {
    (void)arg;
    g_fd = internal_bt_open(g_cfg.uart_dev, g_cfg.baud);
    if(g_fd < 0) {
        fprintf(stderr, "[BT] Open failed: %s\n", g_cfg.uart_dev);
        return NULL;
    }
    // printf("[BT Debug] Port Opened (Blocking Mode). Waiting for data...\n");

    char buf[128]; // 명령어 조립용 버퍼
    int idx = 0;
    char tmp[64];  // 수신 버퍼

    while(g_running) {
        // [수정 5] poll 없이 바로 read 호출
        // Blocking 모드이므로 데이터가 없으면 여기서 스레드가 멈춰 있습니다. (CPU 사용률 0%)
        // 데이터가 1바이트라도 도착하면 리눅스 커널이 즉시 깨워줍니다. (인터럽트 방식)
        int n = read(g_fd, tmp, sizeof(tmp));

        if (n > 0) {
            // 읽은 데이터 처리
            for (int i = 0; i < n; i++) {
                char ch = tmp[i];
                
                // 디버깅: 반응속도 확인용 (빠르면 주석 해제)
                // printf("%c", ch); 

                if (ch == '\n' || ch == '\r') {
                    if (idx > 0) {
                        buf[idx] = '\0';
                        // 콜백 호출
                        if (g_cb) g_cb(buf);
                        idx = 0;
                    }
                }
                else if (idx < (int)(sizeof(buf) - 1)) {
                    buf[idx++] = ch;
                }
            }
            // 즉시 화면 출력 (디버깅용)
            // fflush(stdout); 
        } 
        else if (n < 0) {
            // 에러 발생 시 잠시 대기 (무한 루프 방지)
            usleep(10000);
        }
    }

    if(g_fd >= 0) { close(g_fd); g_fd = -1; }
    return NULL;
}

int BT_init(const bt_config_t* cfg, bt_on_cmd_fn cb) {
    if(!cfg) return -1;
    g_cfg = *cfg;
    g_cb = cb;
    return 0;
}

int BT_start(void) {
    if(g_running) return 0;
    g_running = 1;
    if(pthread_create(&g_thr, NULL, bt_rx_thread, NULL) != 0) {
        g_running = 0;
        return -1;
    }
    g_thread_started = 1;
    return 0;
}

void BT_stop(void) {
    if (!g_running) return;
    g_running = 0;
    // Blocking read 중일 수 있으므로 강제 종료 혹은 캔슬이 필요할 수 있으나
    // 간단한 종료를 위해 스레드 detach 혹은 그냥 둡니다.
    // 여기서는 pthread_cancel을 쓰거나, g_fd를 닫아서 read를 깨우는 방법을 씁니다.
    if (g_fd >= 0) {
        close(g_fd); // 파일 디스크립터를 닫으면 read가 에러를 뱉고 깨어남
        g_fd = -1;
    }
    if (g_thread_started) {
        pthread_join(g_thr, NULL);
        g_thread_started = 0;
    }
}