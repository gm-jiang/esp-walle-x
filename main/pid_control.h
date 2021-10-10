#ifndef _PID_CONTROL_H_
#define _PID_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _pid_ctrl_t {
    float set_speed;
    float actual_speed;
    float err;
    float err_next;
    float err_last;
    float kp;
    float ki;
    float kd;
} pid_ctrl_t;


float pid_realize(pid_ctrl_t *pid, float speed);
int pid_test(void);

#ifdef __cplusplus
}
#endif

#endif

