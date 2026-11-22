/*********************************************************************************************************************
* CYT4BB Opensourec Library （CYT4BB 开源库）：一个用于官方 SDK 接口的二次封装开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License）的条款重新分发和/或修改
* 本软件可以适用 GPL 的第3版（即 GPL3.0），也可以选择（根据您的意愿）任何更高的版本
*
* 此开源库的发布是希望它能发挥价值，但我们并没有提供任何形式的担保
* 也没有针对特定目的的适销性或适用性的暗示担保
* 更详细的情况请参阅 GPL
*
* 您应该已经收到适用许可证的副本和 GPL 的副本
* 如果没有，请查阅 <https://www.gnu.org/licenses/>
*
* 特别声明：
* 本开源库使用 GPL3.0 开源许可协议，以上文字也是该协议的核心内容
* 有关中英文的说明请参考 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件
* 有关许可证的说明请查看 libraries 文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序，但修改内容时请保留逐飞科技的版权声明！
*
* 文件名称：         control
* 公司名称：         成都逐飞科技有限责任公司
* 版本信息：         查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境：         IAR 9.40.1
* 适用平台：         CYT4BB
* 店铺链接：         https://seekfree.taobao.com/
*
* 修改记录：
* 日期             作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "code1.h"

/********************************************************************************************************************
 * 全局变量定义
 * 包含目标速度、状态控制、角度位置PID、姿态控制系统全局变量
 ********************************************************************************************************************/
//目标值
float target_motor_turn = 0.0f;     // 车身角度目标值
float target_gyro_turn = 0.0f;      // 车身角速度目标值

// 全局变量
uint32 data = 0;
uint8 signal = 0;
float pwm_R1=0.0f;
float pwm_L1=0.0f;
float pwm_R2=0.0f;
float pwm_L2=0.0f;         
float Balance_Pwm_turn,Gyro_Pwm_turn = 0.0f;
float target_front = 0.0f;          // 前进速度目标值 前进
float target_left  = 0.0f;          // 横向速度目标值 横向
float target_velocity_R1 = 0.0f;    // 右前电机速度目标值
float target_velocity_L1 = 0.0f;    // 左前电机速度目标值
float target_velocity_L2 = 0.0f;    // 左后电机速度目标值
float target_velocity_R2 = 0.0f;    // 右后电机速度目标值

// PID结构体变量
pid_param_t motor_speed_R1;         //R1速度PID
pid_param_t motor_speed_L1;         //L1速度PID
pid_param_t motor_speed_R2;         //R2速度PID
pid_param_t motor_speed_L2;         //L2速度PID
pid_param_t motor_angle_yaw;        //方向PID参数
pid_param_t motor_gyro_angle;       //方向角速度PID

//PID初始化
void pid_init(void)
{
    // 初始化速度PID
    PidInit(&motor_speed_R1);
    PidChange(&motor_speed_R1,73.0f,1.5f,3.5f);
    PidChange_imax(&motor_speed_R1, 2000);
    
    PidInit(&motor_speed_L1);
    PidChange(&motor_speed_L1,73.0f,1.5f,3.5f);
    PidChange_imax(&motor_speed_L1, 2000);
    
    PidInit(&motor_speed_R2);
    PidChange(&motor_speed_R2,73.0f,1.5f,3.5f);
    PidChange_imax(&motor_speed_R2, 2000);
    
    PidInit(&motor_speed_L2);
    PidChange(&motor_speed_L2,73.0f,1.5f,3.5f);
    PidChange_imax(&motor_speed_L2, 2000);
    
    // 初始化角度PID
    PidInit(&motor_angle_yaw);
    PidChange(&motor_angle_yaw,0.10f,0.0f,0.05f);//0.10f,0.0f,0.05f
    PidChange_imax(&motor_angle_yaw, 2000);
    
    // 初始化角速度PID
    PidInit(&motor_gyro_angle);
    PidChange(&motor_gyro_angle,8.0f,0.02f,0.5f);//8.0f,0.02f,0.5f
    PidChange_imax(&motor_gyro_angle, 2000);
}

//PID速度计算函数
float Velocity(pid_param_t * pid, int16 encoder_data, float target_velocity)
{
    float Velocity_pwm = 0;
    Velocity_pwm = PidLocCtrl(pid,target_velocity - (float)encoder_data);
    return Velocity_pwm;
}

// 角度控制计算PID
float Balance(pid_param_t * pid ,float Angle, float Gyro, float target_angle, float target_gyro)
{
    float error = target_angle - Angle; // 车身角度偏差
    float Gyro_bias = target_gyro - Gyro; // 角速度环偏差
    /* 累加积分 */
    pid->integrator += error;
   /* 积分限幅 */
    pid->integrator = constrain_float(pid->integrator, -(pid->imax), pid->imax);
    
    pid->out_p = pid->kp * error + pid->kp1 * error * error * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * Gyro_bias;

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    return pid->out;
}

// 角速度环控制计算PID
float GyroControl(pid_param_t * pid, float target_gyro, float now_gyro)
{
    float gyro_pwm = 0;
    gyro_pwm = PidLocCtrl(pid,target_gyro - now_gyro);
    return gyro_pwm; // 返回控制量
}

//整车pid控制
void control_use(void)
{
  data = (data+1) % 4;
  //循迹
  //inc_use(-pos_y_final, pos_x_final,beacon_points,20.0f);
  
  //状态机1
  //state_machine_run(-pos_y_final, pos_x_final);
  
  //状态机2
  //state_machine_run2(-pos_y_final, pos_x_final);
  
  //状态机3
  if(!g_waypoint_trigger)
  state_machine_run3(-pos_y_final, pos_x_final);
  else
  {
    target_front = 0;
    target_left = 0;
  }
  
  //速度分解
  target_velocity_R1 = target_front - target_left;    // 右前电机速度目标值
  target_velocity_L1 = target_front + target_left;    // 左前电机速度目标值
  target_velocity_L2 = target_front - target_left;    // 左后电机速度目标值
  target_velocity_R2 = target_front + target_left;    // 右后电机速度目标值
  
  //角度控制计算
  if(signal == 0)
  {
  if(data == 0)//40ms
  {
    Balance_Pwm_turn = Balance(&motor_angle_yaw, eulerAngle.yaw, icm_data.gyro_z*PI/180, target_motor_turn, target_gyro_turn);
  }
  //角速度控制计算
  if(data % 2 == 0)//20ms
  {
    Gyro_Pwm_turn = GyroControl(&motor_gyro_angle,target_gyro_turn + Balance_Pwm_turn, icm_data.gyro_z*PI/180);
  }
  }
  //速度控制计算
  pwm_R1=Velocity(&motor_speed_R1, encoder_data_R1, target_velocity_R1 - Gyro_Pwm_turn);
  pwm_L1=Velocity(&motor_speed_L1, encoder_data_L1, target_velocity_L1 + Gyro_Pwm_turn);
  pwm_R2=Velocity(&motor_speed_R2, encoder_data_R2, target_velocity_R2 - Gyro_Pwm_turn);
  pwm_L2=Velocity(&motor_speed_L2, encoder_data_L2, target_velocity_L2 + Gyro_Pwm_turn);
  
  //printf("%f,%f,%f,%f\r\n",pwm_R1,pwm_L1,pwm_R2,pwm_L2);
  //输出控制
  motor_out(pwm_R1,pwm_L1,pwm_R2,pwm_L2);
}

//控制初始化
void control_init(void)
{
    motor_init();               //电机初始化
    encoder_init();             //编码器初始化
    imu_rx_init();              //IMU
    pid_init();                 //PID初始化
    navigation_init();         //导航初始化
    state_machine_init();      //惯导目标值设值
    car_uart_init();
}

