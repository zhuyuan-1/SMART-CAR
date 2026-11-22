#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"

#define PWM_R1             (TCPWM_CH24_P09_0)        //右前电机1 TCPWM_CH24_P09_0
#define DIR_R1             (P09_1)                   //右前电机gpio高低  P09_1

#define PWM_L2             (TCPWM_CH09_P05_0)        //左后电机4 TCPWM_CH09_P05_0
#define DIR_L2             (P05_1)                   //左后电机gpio高低 P05_1

#define PWM_L1             (TCPWM_CH30_P10_2)        //左前电机2 对的 TCPWM_CH30_P10_2
#define DIR_L1             (P10_3)                   //左前电机gpio高低 P10_3

#define PWM_R2             (TCPWM_CH11_P05_2)        //右后电机3 对的 TCPWM_CH11_P05_2
#define DIR_R2             (P05_3)                   //右后电机gpio高低  P05_3
#define FREQ_MOTOR         (17000)                   //pwm频率

//电机初始化
void motor_init(void);
//左右电机驱动函数
void motor_out(float right1_pwm,float left1_pwm,float right2_pwm,float left2_pwm);

#endif /* CODE_MOTOR_H_ */
