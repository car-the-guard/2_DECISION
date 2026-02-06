// ========== can_security_utils.h ==========
#ifndef CAN_SECURITY_UTILS_H
#define CAN_SECURITY_UTILS_H

#include <stdint.h>

// 비밀키 (모든 노드 동일)
extern const uint8_t SECRET_KEY[4];

// 매직 넘버
#define MAC_MAGIC_NUMBER 0x5A

/**
 * @brief MAC 값 생성
 * 
 * @param data 페이로드 데이터 (데이터 + 타임스탬프 + 카운터)
 * @param len 데이터 길이 (일반적으로 7바이트)
 * @param counter 카운터 값
 * @return 계산된 MAC 값 (1바이트)
 */
uint8_t compute_mac(const uint8_t *data, uint8_t len, uint8_t counter);

/**
 * @brief MAC 값 검증
 * 
 * @param data 수신한 페이로드 데이터 (데이터 + 타임스탬프 + 카운터)
 * @param len 데이터 길이 (일반적으로 7바이트)
 * @param counter 카운터 값
 * @param received_mac 수신한 MAC 값
 * @return 1: 검증 성공, 0: 검증 실패
 */
uint8_t verify_mac(const uint8_t *data, uint8_t len, uint8_t counter, uint8_t received_mac);

#endif