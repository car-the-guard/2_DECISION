#pragma once
#include <stdint.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

// =========================================================
// [Enum] 상태 정의
// =========================================================
typedef enum {
    DIM_LANE_UNKNOWN = 0,
    DIM_LANE_1 = 1, DIM_LANE_2 = 2, DIM_LANE_3 = 3
} dim_lane_t;

typedef enum {
    DIM_OBJ_NONE = 0,
    DIM_OBJ_OBSTACLE, DIM_OBJ_CONE, DIM_OBJ_PERSON
} dim_obj_type_t;

typedef enum {
    DIM_STATE_NORMAL = 0,   
    DIM_STATE_WARNING,      
    DIM_STATE_CRITICAL      
} dim_decision_state_t;

// =========================================================
// [Struct] 주행 정보 스냅샷
// =========================================================
typedef struct {
    float rel_speed_mps;     
    float cur_accel_mps2;    
    float ultra_dist_cm;     
    float cur_speed_mps;     
    uint16_t heading_deg;    
    
    dim_lane_t lane;
    dim_obj_type_t obj_type;

    float calc_ttc_sec;             
    dim_decision_state_t decision;  

    uint32_t ts_common_ms;   
} dim_snapshot_t;

// =========================================================
// [Functions]
// =========================================================
int  DIM_init(void);
void DIM_deinit(void);

// [NEW] 개별 업데이트 함수들 (main.c 에러 해결용)
void DIM_update_speed(float mps);
void DIM_update_accel(float mps2);
void DIM_update_rel_speed(float mps);
void DIM_update_ultra(float cm);
void DIM_update_heading(uint16_t deg);
void DIM_update_lane(dim_lane_t lane);
void DIM_update_obj_type(dim_obj_type_t obj_type);

// 기존 함수
void DIM_set_decision(float ttc, dim_decision_state_t state);
int  DIM_get_snapshot(dim_snapshot_t* out);

#ifdef __cplusplus
}
#endif