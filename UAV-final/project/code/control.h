#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"
#include "pid.h"

//定义数值
#define THROTTLE_HOVER          (6100)     // 11.2V悬停平衡占空比
#define VOL_BASE                (11.2f)   // 标准悬停电压
#define VOL_MIN                 (10.0f)    // 最低工作电压
#define VOL_MAX                 (12.0f)    // 最高带载电压
// 降落函数
#define LAND_IDLE               0   // 空闲：未触发降落，正常飞行
#define LAND_DESCENT            1   // 快速下降：高于缓降高度，以较快速度下降
#define LAND_FLARE              2   // 近地缓降：低于阈值高度，减速接近地面
#define LAND_TOUCHDOWN          3   // 触地确认：已检测到接地，等待延时后停转
#define LAND_DISARMED           4   // 已停转：电机锁定，降落完成
//全域变量
extern float y_distance;
extern float x_distance;
extern uint8 stop;                  //停止标志位
extern uint8 flow_change;          //光流启动
extern float stop_m2;              //第二核停止标志位
extern float open_uart_judge;     //车启动标志位
extern uint8  land_state;          //无人机降落状态机

//目标值
extern float target_high;           //高度占空比，之后根据TOF来设计，单位：mm
extern float target_x;              //x轴方向的目标位移
extern float target_y;              //y轴方向的目标位移
extern float target_x_speed;        //x轴方向的目标速度
extern float target_y_speed;        //y轴方向的目标速度
extern float target_z_speed;        //z轴方向的目标速度
extern float target_motor_front;    //前后目标角度roll
extern float target_motor_rollove;  //左右侧翻目标角度pitch
extern float target_motor_turn;     //左右侧翻目标角度yaw
extern float target_gyro_front;     //前后目标角速度
extern float target_gyro_rollove;   //左右侧翻目标角速度
extern float target_gyro_turn;      //方向目标角速度
extern float x_beacon;             //信标X坐标
extern float y_beacon;             //信标Y坐标
extern float x_car;                //小车X坐标
extern float y_car;                //小车Y坐标
extern float car_image_flag;       //小车方向
extern float x_pwm;
extern float y_pwm;
extern float gyro_x_f, gyro_y_f, gyro_z_f;
//pid结构体
extern pid_param_t motor_high;          //高度PID参数
extern pid_param_t motor_x;             //前后位移PID参数
extern pid_param_t motor_y;             //左右位移PID参数
extern pid_param_t motor_x_speed;       //前后速度PID参数
extern pid_param_t motor_y_speed;       //左右速度PID参数
extern pid_param_t motor_z_speed;       //上下速度PID参数
extern pid_param_t motor_front;         //前后角度PID参数
extern pid_param_t motor_rollove;       //左右侧翻角度PID参数
extern pid_param_t motor_gyro_front;    //前后陀螺仪PID参数
extern pid_param_t motor_gyro_rollove;  //左右侧翻陀螺仪PID参数
extern pid_param_t motor_turn;          //方向角度PID参数
extern pid_param_t motor_gyro_turn;     //方向陀螺仪PID参数
//调参专用
extern float now_control;
extern float target_control;

void control_Landing(void);
//飞控初始化
void control_init(void);
//飞控平衡算法
void control_Balance(void);


#endif /* CODE_CONTROL_H_ */
