// include/bluetooth.h

#ifndef BLUETOOTH_MODULE_H
#define BLUETOOTH_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char* uart_dev;
    int baud;
    int enable_stdout;
} bt_config_t;

// 콜백 함수가 이제 문자열 전체를 받음
typedef void (*bt_on_cmd_fn)(const char* cmd_str);

int BT_init(const bt_config_t* cfg, bt_on_cmd_fn cb);
int BT_start(void);
void BT_stop(void);

#ifdef __cplusplus
}
#endif

#endif