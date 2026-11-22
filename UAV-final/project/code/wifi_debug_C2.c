#include "code1.h"

float assistant_C2get[2]={0,0}; //CPU2上位机示波器2个通道显示
float assistant_C2debug[3]={0,0,0}; //CPU2上位机调参3个通道初始化值
float now_speed = 0;        //实时速度

//void wifi_debug_c2init(void)
//{
//    //wifi初始化
//    wifi_init();
//    //选择TCP协议
//    TCP_init();
//    // 逐飞助手初始化，选择wifi_SPI
//    seekfree_init(2);
//    //上位机调参3个通道初始化
//    //assistant_init(assistant_C2debug);
//}
//void wifi_debug_c2use(void)
//{
//  //上位机调参8个通道读取
////  seekfree_assistant_change(assistant_C2debug);
//  //上位机调参赋值
////  kp1 = assistant_debug[0];
////  ki1 = assistant_debug[1];
////  kd1 = assistant_debug[2];
//
//  //上位机示波器8个通道显示
//  assistant_C2get[0] = target_x_speed_remote; 
//  assistant_C2get[1] = now_speed;
//  seekfree_oscilloscope_send(2, assistant_C2get);
//}