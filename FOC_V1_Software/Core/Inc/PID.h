#ifndef __PID_H
#define __PID_H

void PID_init(void);
float PID_angle(float error);
float PID_velocity(float error);
float PID_current_D(float error);
float PID_current_Q(float error);
float LPF_current_q(float x);
float LPF_current_d(float x);
float LPF_velocity(float x);

#endif /* __PID_H */


