/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/
// **************************** 头文件区域 ****************************

#include "zf_common_headfile.h"
#include "code.h"

// **************************** 代码区域 ****************************
#define LED1                    (P19_0)
// **************************** 双核通讯 ****************************
#define DATA_LENGTH               (20)                                           // 数组数据长度
#pragma location = 0x28001000                                                   // 将下面这个数组定义到指定的RAM地址，#pragma需要手动分配地址，因此需要计算数据长度后再分配
__no_init float m7_1_data[DATA_LENGTH];                                        // 定义M7_1演示数据数组 浮点数类型  由于该数组已经在M7_1核心赋值过初值，因此此处不再初始化

#pragma location = 0x2802132c
float m7_0_data[DATA_LENGTH] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};                        // 定义 M7_0 演示数据数组 浮点数类型
// **************************** 双核通讯 ****************************
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                          // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
    
    //各配置初始化
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);// 初始化 LED1 输出 默认高电平 推挽输出模式
    led_judge_init();                   // 开启LED指示
    control_init();                     // 开启飞控初始化
    //wifi_debug_init();                  // 开启wifi调试
    KEY2_init();                        //按键初始化
    system_delay_ms(1000);             // 1s等待初始化完成之后再开启中断  
    //开启中断
    pit_ms_init(PIT_CH0,2);             // 开启中断用于姿态传感器获取数据，2ms一次
    pit_ms_init(PIT_CH1,25);            // 开启中断用于光流获取数据，25ms一次
    pit_ms_init(PIT_CH2,7);             // 开启中断用于飞控，8ms一次
    system_delay_ms(1000);              // 1s等待初始化完成之后再开启主循环
    // 此处编写用户代码 例如外设初始化代码等

    while(true)
    {
        // 此处编写需要循环执行的代码
      // **************************** 电压读取 ****************************
       voltage_adc();  // 电源总电压读取
       //imu_GyroOffset();
       //printf("%f\r\n",voltage_power_supply);
       //printf("%d,%d\r\n",flow_get.optical_TOF1_mm,flow_get.optical_TOF2_mm);
       //printf("%f,%f,%f\r\n", eulerAngle.roll, eulerAngle.pitch, eulerAngle.yaw);
       //printf("%f,%f,%f,0\r\n",gyro_x_f,gyro_y_f,gyro_z_f);
        //printf("%f,%f\r\n",-x_pwm,-y_pwm);
      // printf("\r\nIMU660RC euler_angles data:  %f, %f, %f\r\n", imu660rc_roll, imu660rc_pitch, imu660rc_yaw);
       //printf("%f,%f,%f,%f\r\n",target_x,y_car,target_y,x_car);
       //printf("%f,%f,%f,%f\r\n",comp_pixel_x ,y_car,comp_pixel_y ,x_car);
       //printf("%f,%f,%f,%f\r\n", y_car + comp_pixel_y,y_car, x_car - comp_pixel_x,x_car);
       // **************************** 按键启动 ****************************
        if(!gpio_get_level(KEY4))
       {
          system_delay_ms(150);
          if(!gpio_get_level(KEY4))
          {
            system_delay_ms(2000);
            stop = 1;
          }
       }
      // **************************** wifi调试代码 ****************************
      //wifi_debug_use();
     // **************************** 飞控保护代码 ****************************
       IMU_safe(eulerAngle.roll,eulerAngle.pitch); 
       if(stop > 1)//强制停止
        {
            pit_all_close ();
            small_driver_set_duty(0, 0, 0, 0);
        }
      // **************************** LED代码 ****************************
       gpio_toggle_level(LED1);
       led_judge_use();
      // **************************** 双核通讯代码 ****************************    
       //输出给另一个核数据
        m7_0_data[0] = voltage_power_supply;  //电池电量
        m7_0_data[1] = open_uart_judge;       //车启动标志位
        SCB_CleanInvalidateDCache_by_Addr(&m7_0_data, sizeof(m7_0_data));
        SCB_CleanInvalidateDCache_by_Addr(&m7_1_data, sizeof(m7_1_data));
        //另一个核输入给数据
        //stop_m2 = m7_1_data[0];
        x_beacon = m7_1_data[1];
        y_beacon = m7_1_data[2];
        x_car = m7_1_data[3];
        y_car = m7_1_data[4];
        car_image_flag = m7_1_data[5];
        target_x_speed = m7_1_data[6];
        target_y_speed = m7_1_data[7];
        land_state = (uint8)m7_1_data[8];
        //printf("%d\r\n",land_state);
       // **************************** 延时代码 ****************************
       //system_delay_ms(10);      
        // 此处编写需要循环执行的代码
    }
}

// **************************** 中断函数区域 ****************************
void pit0_ch0_isr()                     // 定时器通道 0 周期中断服务函数，用于姿态传感器姿态获取      
{
   pit_isr_flag_clear(PIT_CH0);
   // 此处编写中断循环执行的代码
    ICM_getEulerianAngles();
    Gyro_Filter_2ms(icm_data.gyro_x, icm_data.gyro_y, icm_data.gyro_z,
                &gyro_x_f, &gyro_y_f, &gyro_z_f);
    //******************角度归零补偿****************************/
    eulerAngle.roll = eulerAngle.roll + 0.53;
    eulerAngle.pitch = eulerAngle.pitch + 1.42;
    eulerAngle.yaw = Turn_target(eulerAngle.yaw);//yaw轴限制
    led_time ++;//指示灯时间自累
}

void pit0_ch1_isr()                     // 定时器通道 1 周期中断服务函数，用于光流获取数据    
{
    pit_isr_flag_clear(PIT_CH1);
   // 此处编写中断循环执行的代码
    optical_flow_contorl(flow_change);
    calculate_imu_velocity(0.025);
}

void pit0_ch2_isr()                     // 定时器通道 2 周期中断服务函数，用于飞控处理      
{
    pit_isr_flag_clear(PIT_CH2);
    // 此处编写中断循环执行的代码
    if(stop == 1)//启动飞控
    {
       control_Balance();
      //small_driver_set_duty(-4000, 4000, -4000, 0);//电调测试函数，输入40%占空比
    }
}
// **************************** 代码区域 ****************************