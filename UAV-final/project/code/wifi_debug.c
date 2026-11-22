#include "code.h"

float assistant_debug[8]={0,0,0,0,0,0,0,0}; //上位机调参8个通道初始化值
float assistant_get[8]={0,0,0,0,0,0,0,0}; //上位机示波器8个通道显示
float kp1=0,ki1=0,kd1=0;
float kp2=0,ki2=0,kd2=0;

//void wifi_debug_init(void)
//{
//    //wifi初始化
//    wifi_init();
//    //选择TCP协议
//    TCP_init();
//    // 逐飞助手初始化，选择wifi_SPI
//    seekfree_init(2);
//    //上位机调参8个通道初始化
//    assistant_init(assistant_debug);
//}
//
//void wifi_debug_use(void)
//{
//  //上位机调参8个通道读取
//  seekfree_assistant_change(assistant_debug);
//  //上位机调参赋值
//  kp1 = assistant_debug[0];
////  high = assistant_debug[1];
//  ki1 = assistant_debug[1];
//  kd1 = assistant_debug[2];
//  kp2 = assistant_debug[3];
//  ki2 = assistant_debug[4];
//  kd2 = assistant_debug[5];
//  //target_motor_turn=  assistant_debug[6];
////  target_motor_front = assistant_debug[5];
////  target_motor_rollove = assistant_debug[6];
////  stop = (uint8)assistant_debug[7];                           //调参通道8给停止位stop赋值
//  //PidChange(&motor_gyro_front, kp1, ki1, kd1);
//  //PidChange(&motor_gyro_rollove, kp1, ki1, kd1);
//  //PidChange(&motor_gyro_turn, kp1, ki1, kd1);
////   PidChange(&motor_front, kp1, ki1, kd2);       
////    PidChange(&motor_rollove, kp2, ki2, kd2);
////    PidChange(&motor_turn, kp2, ki2, kd2);
//  //  motor_front.kp1=kp1;
//  //  motor_rollove.kp1=kp1;
//  //  motor_turn.kp1=kp1;
////  PidChange(&motor_z_speed, kp1, ki1, kd1);
////  PidChange(&motor_x_speed, kp1, ki1, kd1);
////  PidChange(&motor_y_speed, kp2, ki2, kd2);
////  PidChange(&motor_x,kp1, ki1, kd1);
////  PidChange(&motor_y,kp2, ki2, kd2);
////  PidChange(&motor_high, kp2, ki2, kd2);
//  //上位机示波器通道赋值
//  assistant_get[0] = target_z_speed*1000; 
//  assistant_get[1] = speed_now[2]*1000;
//  assistant_get[2] = target_x_speed;
//  assistant_get[3] = speed_now[0]*10;
//  assistant_get[4] = target_y_speed;
//  assistant_get[5] = speed_now[1]*10;
//  assistant_get[6] = target_high;
//  assistant_get[7] = tof_now_mm;
//  //上位机示波器8个通道显示
//  seekfree_oscilloscope_send(8, assistant_get);
//}