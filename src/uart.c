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

// =========================================================
// [API] 초기화
// =========================================================
int UARTIF_init(const uartif_config_t* cfg) {
    if (!cfg || !cfg->dev_path) return -1;

    // 1. UART 장치 파일 열기
    // O_RDWR: 읽기/쓰기 모드 (쓰기만 해도 보통 RDWR로 엽니다)
    // O_NOCTTY: 이 터미널을 프로세스의 제어 터미널로 만들지 않음
    // O_NDELAY: Non-blocking (오픈 시 멈춤 방지)
    g_uart_fd = open(cfg->dev_path, O_RDWR | O_NOCTTY | O_NDELAY);
    if (g_uart_fd < 0) {
        perror("[UART] Open Failed");
        return -1;
    }

    // 2. 기존 플래그 초기화 (Blocking 모드로 변경)
    // 쓰기 동작이 완료될 때까지 기다리도록 설정 (데이터 유실 방지)
    fcntl(g_uart_fd, F_SETFL, 0);

    // 3. Termios 설정 (Baudrate, 8N1)
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(g_uart_fd, &tty) != 0) {
        perror("[UART] tcgetattr failed");
        close(g_uart_fd);
        g_uart_fd = -1;
        return -1;
    }

    // Baudrate 설정
    speed_t speed = B9600; // 기본값
    if (cfg->baudrate == 115200) speed = B115200;
    else if (cfg->baudrate == 9600) speed = B9600;
    
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1 설정 (8 Data bits, No parity, 1 Stop bit)
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_cflag &= ~PARENB;     // No parity
    tty.c_cflag &= ~CSTOPB;     // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;    // No hardware flow control
    
    // 필수 설정 (Local connection, Enable receiver)
    // 수신을 안 하더라도 CREAD는 보통 켜둡니다 (표준 관례)
    tty.c_cflag |= (CLOCAL | CREAD);

    // Raw Mode 설정 (특수 문자 처리 끄기)
    tty.c_lflag = 0; // No canonical, no echo, no signal chars
    tty.c_oflag = 0; // No output processing (Raw output)
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control

    // 적용
    if (tcsetattr(g_uart_fd, TCSANOW, &tty) != 0) {
        perror("[UART] tcsetattr failed");
        close(g_uart_fd);
        g_uart_fd = -1;
        return -1;
    }

    // 버퍼 비우기
    tcflush(g_uart_fd, TCIOFLUSH);

    printf("[UART] Initialized %s @ %d bps (TX Only)\n", cfg->dev_path, cfg->baudrate);
    return 0;
}

// =========================================================
// [API] 종료
// =========================================================
void UARTIF_stop(void) {
    if (g_uart_fd >= 0) {
        close(g_uart_fd);
        g_uart_fd = -1;
    }
}

// =========================================================
// [API] 데이터 전송 (Blocking Write)
// =========================================================
int UARTIF_write_raw(const uint8_t* data, uint32_t len) {
    if (g_uart_fd < 0) return -1;

    // write는 blocking 모드이므로 데이터가 버퍼에 들어갈 때까지 대기함
    int sent = write(g_uart_fd, data, len);
    
    if (sent < 0) {
        perror("[UART] Write Error");
        return -1;
    }
    
    // 보낸 길이가 요청 길이와 다르면 실패 처리
    return (sent == (int)len) ? 0 : -1;
}