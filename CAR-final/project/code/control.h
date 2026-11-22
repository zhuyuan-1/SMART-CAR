#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"
#include "pid.h"

extern uint32    data;
extern uint8     signal;
extern float    target_front;
extern float    target_left;
extern float    target_velocity_R1;            // 左前电机速度目标值
extern float    target_velocity_L1;            // 右前电机速度目标值
extern float    target_velocity_L2;            // 左后电机速度目标值
extern float    target_velocity_R2;            // 右后电机速度目标值
extern float    target_motor_turn;             // 角度目标值
extern float    target_gyro_turn;              // 角速度目标值


extern pid_param_t motor_speed_R1;         //R1速度PID参数
extern pid_param_t motor_speed_L1;         //L1速度PID参数
extern pid_param_t motor_speed_R2;         //R2速度PID参数
extern pid_param_t motor_speed_L2;         //L2速度PID参数
extern pid_param_t motor_angle_yaw;        //方向PID参数
extern pid_param_t motor_gyro_angle;       //方向角速度PID

//控制初始化

void control_init(void);
//整车pid控制
void control_use(void);
#endif