//
// Created by kenke on 2023/04/06.
//

#ifndef INC_ER_Stear_PID_H
#define INC_ER_Stear_PID_H

#define SETPIDGAIN(handler, Kp_input, Ki_input, Kd_input) do { \
(handler)->Kp=Kp_input;                            \
(handler)->Ki=Ki_input;                            \
(handler)->Kd=Kd_input;} while(0)

#define SETWINDUPFLAG(handler, value) ((handler)->is_windup = value)

#define RESETPID(handler) do{ \
(handler)->e = 0;     \
(handler)->e_pre = 0;         \
(handler)->e_pre_pre = 0;     \
(handler)->e_int = 0;} while(0)

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

#endif //INC_ER_Stear_PID_H
