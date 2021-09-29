#ifndef _CONTROL_H_
#define _CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FORWARD,
    BACK,
    RIGHT,
    LEFT,
    STOP,
    AROUND_R,
    AROUND_L,
    ACT_MAX_NUM,
} action_e;

typedef enum {
    FULL_SPEED,
    MID_SPEED,
    LOW_SPEED,
    NO_SPEED,
    SPEED_MAX_NUM,
} speed_e;

//typedef uint8_t (*walle_control_cb_t)(action_e act, speed_e spd, uint32_t timeout);
typedef uint8_t (*walle_init_drv_cb_t)(void);
typedef uint8_t (*walle_init_pwm_cb_t)(void);
typedef uint8_t (*walle_control_act_cb_t)(uint8_t act, uint32_t timeout);
typedef uint8_t (*walle_control_spd_cb_t)(speed_e spd, uint32_t param);

typedef struct _WALLEClient {
    uint32_t devid;
    char  *name;
    speed_e spd_status;
    walle_init_drv_cb_t walle_init_drv_cb;
    walle_init_pwm_cb_t walle_init_pwm_cb;
    walle_control_act_cb_t walle_control_act_cb;
    walle_control_spd_cb_t walle_control_spd_cb;
} WALLEClient_t;

extern WALLEClient_t WALLEClient;

void WALLE_Init(WALLEClient_t *thiz);

#ifdef __cplusplus
}
#endif

#endif

