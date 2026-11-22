#include "code1.h"

float assistant_debug[8]={0,0,0,0,0,0,0,0}; //上位机调参8个通道初始化值
float assistant_get[8]={0,0,0,0,0,0,0,0}; //上位机示波器8个通道显示
float kp1=0,ki1=0,kd1=0;
float kp2=0,ki2=0,kd2=0;
float kp3=0,ki3=0,kd3=0;
float kp4=0,ki4=0,kd4=0;

void wifi_debug_init(void)
{
    //wifi初始化
    wifi_init();
    //选择TCP协议
    TCP_init();
    // 逐飞助手初始化，选择wifi_SPI
    seekfree_init(2);
    //上位机调参8个通道初始化
    assistant_init(assistant_debug);
}

void wifi_debug_use(void)
{
  //上位机调参8个通道读取
  seekfree_assistant_change(assistant_debug);
  //上位机调参赋值
  kp1 = assistant_debug[0];
  kd1 = assistant_debug[1];
  ki1 = assistant_debug[2];
  kp2 = assistant_debug[3];
  ki2 = assistant_debug[4];
  kd2 = assistant_debug[5];
  kp4 = assistant_debug[6];
  kd4 = assistant_debug[7];
//  PidChange(&motor_speed_R1,kp1,ki1,kd1);
//  PidChange(&motor_speed_L1,kp1,ki1,kd1);
//  PidChange(&motor_speed_R2,kp1,ki1,kd1);
//  PidChange(&motor_speed_L2,kp1,ki1,kd1);
//  PidChange(&motor_gyro_angle, kp2,ki2,kd2);
//  PidChange(&motor_angle_yaw, kp1,ki1,kd1);

  //上位机示波器通道赋值
  assistant_get[0] = data_ins;
  assistant_get[1] = g_state1;
  assistant_get[2] = pos_x_final;
  assistant_get[3] = pos_y_final;
  assistant_get[4] = encoder_data_R1;
  assistant_get[5] = encoder_data_L1;
  assistant_get[6] = encoder_data_R2;
  assistant_get[7] = encoder_data_L2;
  //上位机示波器8个通道显示
  seekfree_oscilloscope_send(8, assistant_get);
}