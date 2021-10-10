#include <stdio.h>
#include <stdlib.h>
#include "pid_control.h"

static pid_ctrl_t pid;

void pid_init(pid_ctrl_t *pid)
{
    pid->set_speed = 0.0;
    pid->actual_speed = 0.0;
    pid->err = 0.0;
    pid->err_last = 0.0;
    pid->err_next = 0.0;
    pid->kp = 0.2;
    pid->ki = 0.015;
    pid->kd = 0.2;
}


float pid_realize(pid_ctrl_t *pid, float speed)
{
    pid->set_speed = speed;
    pid->err = pid->set_speed - pid->actual_speed;
    float increment_speed = pid->kp*(pid->err - pid->err_next) + pid->ki*pid->err + pid->kd*(pid->err - 2*pid->err_next + pid->err_last);
    pid->actual_speed += increment_speed;
    pid->err_last = pid->err_next;
    pid->err_next = pid->err;
    return pid->actual_speed;
}

int pid_test(void)
{
    int count = 0;
    pid_init(&pid);
    while(count < 1000)
    {
        float speed = pid_realize(&pid, 200.0);
        printf("%f\n",speed);
        count++;
    }
    return 0;
}

