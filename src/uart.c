#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include "uart.h"

// =========================================================
// 내부 변수
// =========================================================
static int g_uart_fd = -1;
static volatile int g_running = 0;
static pthread_t g_rx_thr;
static void (*g_rx_cb)(const uint8_t* data, uint32_t len) = NULL;

// =========================================================
// [API] 초기화
// =========================================================
static void* uart_rx_thread(void* arg) {
    uint8_t buf[64];
    while (g_running) {
        if (g_uart_fd < 0) { usleep(100000); continue; }
        
        // Blocking Read (최소 1바이트)
        int n = read(g_uart_fd, buf, sizeof(buf));
        if (n > 0) {
            // 데이터가 들어오면 콜백 호출 (패킷 파싱은 위임)
            if (g_rx_cb) g_rx_cb(buf, n);
        } else {
            usleep(5000); // 에러 시 잠시 대기
        }
    }
    return NULL;
}

int UARTIF_init(const uartif_config_t* cfg, void (*rx_callback)(const uint8_t*, uint32_t)) {
    if (!cfg || !cfg->dev_path) return -1;
    
    // 1. 기존처럼 UART 열기
    g_uart_fd = open(cfg->dev_path, O_RDWR | O_NOCTTY); // O_NDELAY 제거 (Blocking 사용)
    if (g_uart_fd < 0) { perror("[UART] Open Fail"); return -1; }

    // 2. Termios 설정 (기존 코드 유지하되, VMIN=1 설정 확인)
    struct termios tty;
    tcgetattr(g_uart_fd, &tty);
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    
    // Raw Mode
    tty.c_cflag |= (CLOCAL | CREAD | CS8);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag = 0; tty.c_iflag = 0; tty.c_oflag = 0;
    
    // Blocking 설정 (최소 1바이트 대기)
    tty.c_cc[VMIN] = 1; 
    tty.c_cc[VTIME] = 0;
    
    tcsetattr(g_uart_fd, TCSANOW, &tty);
    tcflush(g_uart_fd, TCIOFLUSH);

    // 3. 콜백 저장 및 수신 스레드 시작
    g_rx_cb = rx_callback;
    g_running = 1;
    if (pthread_create(&g_rx_thr, NULL, uart_rx_thread, NULL) != 0) {
        return -1;
    }

    printf("[UART] Wireless Board Connected on %s (Rx/Tx)\n", cfg->dev_path);
    return 0;
}

// =========================================================
// [API] 종료
// =========================================================
void UARTIF_stop(void) {
    g_running = 0;
    if (g_uart_fd >= 0) close(g_uart_fd);
    pthread_join(g_rx_thr, NULL);
}

// =========================================================
// [API] 데이터 전송 (Blocking Write)
// =========================================================
int UARTIF_write_raw(const uint8_t* data, uint32_t len) {
    if (g_uart_fd < 0) return -1;
    return write(g_uart_fd, data, len);
}