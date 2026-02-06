// ========== can_security_utils.c ==========
#include "can_security_utils.h"

// 비밀키 정의 (모든 노드 동일하게 설정)
const uint8_t SECRET_KEY[4] = {0x19, 0x99, 0x06, 0x15};

/**
 * @brief 1비트 좌측 회전 (Rotate Left)
 * 
 * @param value 회전할 값
 * @return 1비트 좌측 회전된 값
 */
static inline uint8_t rotate_left_1(uint8_t value) {
    return (value << 1) | (value >> 7);
}

/**
 * @brief MAC 값 생성
 */
uint8_t compute_mac(const uint8_t *data, uint8_t len, uint8_t counter) {
    uint8_t mac;
    uint8_t i;
    
    // 1단계: 초기화 (매직 넘버 XOR 카운터)
    mac = MAC_MAGIC_NUMBER ^ counter;
    
    // 2단계: 데이터 + 비밀키 믹싱
    for (i = 0; i < len; i++) {
        mac ^= data[i];                  // 데이터 XOR
        mac += SECRET_KEY[i % 4];        // 비밀키 ADD
        mac = rotate_left_1(mac);        // 1비트 좌측 회전
    }
    
    return mac;
}

/**
 * @brief MAC 값 검증
 */
uint8_t verify_mac(const uint8_t *data, uint8_t len, uint8_t counter, uint8_t received_mac) {
    uint8_t calculated_mac;
    
    // MAC 재계산
    calculated_mac = compute_mac(data, len, counter);
    
    // 비교
    if (calculated_mac == received_mac) {
        return 1;  // 검증 성공
    } else {
        return 0;  // 검증 실패
    }
}