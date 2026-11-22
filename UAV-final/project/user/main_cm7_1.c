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
* 文件名称          main_cm7_1
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
// **************************** 代码区域 ****************************
#include "code1.h"
#include "zf_common_headfile.h"
// **************************** 双核通讯 ****************************
#define DATA_LENGTH               (20)                                           // 数组数据长度
#pragma location = 0x28001000                                                   // 将下面这个数组定义到指定的RAM地址，便于其他核心直接访问(开源库默认在 0x28001000 地址保留了8kb的空间用于数据交互)
float m7_1_data[DATA_LENGTH] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};                        // 定义 M7_1 演示数据数组 浮点数类型

#pragma location = 0x2802132c                                                       // 将下面这个数组定义到指定的RAM地址，#pragma需要手动分配地址，因此需要计算数据长度后再分配
__no_init float m7_0_data[DATA_LENGTH];                                        // 定义M7_0演示数据数组 浮点数类型  由于该数组已经在M7_1核心赋值过初值，因此此处不再初始化
// **************************** 双核通讯 ****************************
struct sBrightRegion br[10];

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_info_init();                  // 调试串口信息初始化
     
    // 此处编写用户代码 例如外设初始化代码等

    mt9v03x_init();      // 摄像头初始化
    //wifi_debug_init();
    point_draw_init(dst_image);
    TFT_init();         // 屏幕初始化
    KEY_init();         // 按键初始化
    IMAGE_CreatPoint(); // 图像处理初始化
    lora3a22_init();    // 遥控器初始化
    uav_uart_init();    // 有线串口初始化
    system_delay_ms(2000); 
    pit_ms_init(PIT_CH11,20); //串口中断
    pit_ms_init(PIT_CH12,10); //串口中断
    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
      // 此处编写需要循环执行的代码
      // **************************** 双核通讯 ****************************
      //m7_1_data[0] = stop_m2;          // 停止位
      m7_1_data[1] = (float)x_beacon;  // 信标x坐标
      m7_1_data[2] = (float)y_beacon;  // 信标y坐标
      m7_1_data[3] = x_car_f;          // 小车x坐标
      m7_1_data[4] = y_car_f;          // 小车y坐标
      m7_1_data[5] = (float)car_image_flag;
      m7_1_data[6] = target_x_speed;   // x目标速度
      m7_1_data[7] = target_y_speed;   // y目标速度
      m7_1_data[8] = uart_get.uav_spot;
      //printf("%f\r\n",m7_1_data[8]);
      SCB_CleanInvalidateDCache_by_Addr(&m7_0_data, sizeof(m7_0_data));
      SCB_CleanInvalidateDCache_by_Addr(&m7_1_data, sizeof(m7_1_data));
      voltage_power_supply = m7_0_data[0];//传入电池电量
      open_uart_judge = m7_0_data[1];//传入电池电量
      if(open_uart_judge > 0)
        open_judge = 1;//车启动
      // **************************** 摄像头处理 ****************************
        if(mt9v03x_finish_flag)
        {
          IMGE_CONTROL(br);
          mt9v03x_finish_flag = 0;
          //seekfree_assistant_camera_send();
        }
       // **************************** 遥控器处理 ****************************
        remote_control();
       // **************************** 按键消抖 ****************************
        if(!gpio_get_level(KEY1) || !gpio_get_level(KEY2) || !gpio_get_level(KEY3))
       {
          system_delay_ms(150);
       }
       // **************************** 屏幕菜单 ****************************
       TFT_KEYput(gpio_get_level(KEY1),gpio_get_level(KEY2),gpio_get_level(KEY3));
    }
}
void pit0_ch11_isr()                    // 车机通讯中断    
{
    pit_isr_flag_clear(PIT_CH11);
    //开启串口传输
    duta_judge = 1;
    // **************************** 串口通讯 ****************************
    uav_uart_slave();
    //更新数据
    uart_send.car_x = (uint8)(x_car_f+0.5);//x_car
    uart_send.car_y = (uint8)(y_car_f+0.5);//y_car
    uart_send.car_direction = image_flag;//image_flag
    uart_send.signs_x = x_beacon;// x_beacon
    uart_send.signs_y = y_beacon;// x_beacon
}
void pit0_ch12_isr()                    // 遥控器中断    
{
    pit_isr_flag_clear(PIT_CH12);
    //开启串口传输
    remote_judge(10);
}
// **************************** 代码区域 ****************************