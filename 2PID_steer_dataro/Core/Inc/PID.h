//
// Created by kenke on 2023/04/06.
//

#ifndef INC_2PID_STEER_DATARO_PID_H
#define INC_2PID_STEER_DATARO_PID_H

#define SETPIDGAIN(handler, Kp_input, Ki_input, Kd_input) do { \
(handler)->Kp=Kp_input;                            \
(handler)->Ki=Ki_input;                            \
(handler)->Kd=Kd_input;} while(0)

#define SETWINDUPFLAG(handler, value) ((handler)->is_windup = value)

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float pre_curr_val;
    float e;
    float e_pre;
    float e_pre_pre;
    float e_int;
    float e_diff;
    int is_windup;
} PIDparamshandleTypeDef;

int calc_PID(PIDparamshandleTypeDef *handler, float target, float curr_val, float dt);

#endif //INC_2PID_STEER_DATARO_PID_H
