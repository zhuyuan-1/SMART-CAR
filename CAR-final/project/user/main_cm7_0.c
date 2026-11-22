/*********************************************************************************************************************
* CYT4BB7_CAR1 — 控制核心 CM7_0 主函数
* 文件: main_cm7_0.c
* 说明: 运行状态机、电机控制、IMU/编码器处理的 PIT 中断
*       PIT_CH0 (5ms): IMU 姿态获取 + 编码器读取 + 导航更新
*       PIT_CH1 (10ms): 控制循环（含 IPC 参数更新检测）
*       PIT_CH2 (20ms): UART 数据接收
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "code1.h"

// 以下工程或编译选项设置请在 CMakeLists.txt 或 IDE 中进行
// 第一步：关闭编译器中打开的文件
// 第二步：project->clean，等待重新编译完成

// 本程序是开源扩展程序，允许使用者移植或调试修改
// 本程序是开源扩展程序，允许使用者移植或调试修改
// 本程序是开源扩展程序，允许使用者移植或调试修改

// **************************** 代码区域 ****************************
#define LED1                    (P19_0)

// ---- 双核 IPC 共享内存 ----
#pragma location = IPC_CM7_1_BASE_ADDR
__no_init float ipc_cm7_1_data[IPC_DATA_LEN_CM7_1];  // CM7_0 端只读（CM7_1 写入）

#pragma location = IPC_CM7_0_BASE_ADDR
float ipc_cm7_0_data[IPC_DATA_LEN_CM7_0] = {0};       // CM7_0 写入，CM7_1 读取

/********************************************************************************
* ipc_process_cm7_1 — 在 10ms PIT 中断中调用，检测来自 CM7_1 屏幕的参数更新
* ============================================================================
* IPC_CMD 位掩码 → 更新的参数:
*   bit0 (1): g_beacon_track_speed, g_inertial_nav_speed, g_charge_speed
*   bit1 (2): g_pixel_charge_threshold
*   bit2 (4): g_beacon_count, g_target_point_num, beacon_sequence[20]
*   bit3 (8): g_beacon_count, beacon_position[8][2]
********************************************************************************/
static void ipc_process_cm7_1(void)
{
    SCB_CleanInvalidateDCache_by_Addr(&ipc_cm7_1_data, sizeof(ipc_cm7_1_data));

    int cmd = (int)ipc_cm7_1_data[IPC_IDX_CMD];
    if (cmd == 0) return;

    // 速度参数（base_speed → beacon_track + inertial_nav，sprint_speed → charge）
    if (cmd & 1) {
        g_beacon_track_speed = ipc_cm7_1_data[IPC_IDX_BEACON_TRACK];
        g_inertial_nav_speed = ipc_cm7_1_data[IPC_IDX_INERTIAL_NAV];
        g_charge_speed       = ipc_cm7_1_data[IPC_IDX_CHARGE];
    }
    // 像素冲刺阈值
    if (cmd & 2) {
        g_pixel_charge_threshold = (uint32)ipc_cm7_1_data[IPC_IDX_PIXEL_THRESH];
    }
    // 信标序列
    if (cmd & 4) {
        g_beacon_count = (uint8)ipc_cm7_1_data[IPC_IDX_BEACON_COUNT];
        g_target_point_num = (uint8)ipc_cm7_1_data[IPC_IDX_TARGET_POINT_NUM];
        for (int i = 0; i < BEACON_SEQ_SIZE; i++) {
            beacon_sequence[i] = (uint8)ipc_cm7_1_data[IPC_IDX_BEACON_SEQ + i];
        }
        state_machine_init();  // 重新将信标序列展开为航点坐标
    }
    // 信标位置
    if (cmd & 8) {
        g_beacon_count = (uint8)ipc_cm7_1_data[IPC_IDX_BEACON_COUNT];
        for (int i = 0; i < BEACON_POS_SIZE; i++) {
            beacon_position[i][0] = ipc_cm7_1_data[IPC_IDX_BEACON_POS + i*2];
            beacon_position[i][1] = ipc_cm7_1_data[IPC_IDX_BEACON_POS + i*2 + 1];
        }
        state_machine_init();  // 用新位置重新展开
    }

    // 确认处理完成
    ipc_cm7_1_data[IPC_IDX_CMD] = IPC_CMD_NONE;
    ipc_cm7_0_data[IPC_ACK_IDX] = (float)cmd;
    SCB_CleanInvalidateDCache_by_Addr(&ipc_cm7_1_data, sizeof(ipc_cm7_1_data));
    SCB_CleanInvalidateDCache_by_Addr(&ipc_cm7_0_data, sizeof(ipc_cm7_0_data));
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化（必须在最前面）
    debug_init();                       // 调试串口信息初始化
    // 此处编写用户代码，例如外设初始化相关操作
    pwm_init(TCPWM_CH01_P03_0, 20000, 0);
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 初始化 LED1 灯，默认高电平，推挽输出模式
    control_init();               // 控制初始化
    //buzzer_init();              // 蜂鸣器初始化
    //pwm_init(TCPWM_CH29_P19_4, 4000, 0);
    system_delay_ms(1000);
    pit_ms_init(PIT_CH0, 5);     // 初始化 PIT_CH0 为周期中断 5ms 周期
    pit_ms_init(PIT_CH1, 10);    // 初始化 PIT_CH1 为周期中断 10ms 周期
    pit_ms_init(PIT_CH2, 20);    // 初始化 PIT_CH2 为周期中断 20ms 周期

    // 此处编写用户代码，例如外设初始化相关操作
    while(true)
    {
        // 此处编写需要循环执行的代码
        //imu_GyroOffset();
        //buzzer_on();
        //pwm_set_duty(TCPWM_CH29_P19_4, 5000); // 50%占空比
        //printf("%f,%f,%f\r\n",g_beacon_track_speed,g_inertial_nav_speed,g_charge_speed);
        pwm_set_duty(TCPWM_CH01_P03_0, 2000);
        // **************************** 主循环示例 ****************************
        if (s == 1){gpio_toggle_level(LED1);}
        system_delay_ms(200);
        // 此处编写需要循环执行的代码
    }
}

void pit0_ch0_isr()                     // 姿态数据中断
{
    pit_isr_flag_clear(PIT_CH0);
    //中断开始
    ICM_getEulerianAngles();            // rc 数据获取
    encoder_get();                      // 5ms 读取一次速度
    navigation(0.005);                  // 惯导运算
    //printf("%d,%d,%d,%d\r\n",encoder_data_R1,encoder_data_L1,encoder_data_R2,encoder_data_L2);
    //printf("%f\r\n",eulerAngle.yaw);
    //printf("%f\r\n",icm_data.gyro_z);
}

void pit0_ch1_isr()                     // 速度控制中断
{
    pit_isr_flag_clear(PIT_CH1);
    //中断开始
    ipc_process_cm7_1();                // 检测来自 CM7_1 的参数更新
    control_use();                      // 电机控制函数
}

void pit0_ch2_isr()                     // 串口中断      
{
    pit_isr_flag_clear(PIT_CH2);
    //中断开始
    //传出数据赋值
    uart_send.car_x_v = 200;
    uart_send.car_y_v ++;
    if(uart_send.car_y_v > 300)
      uart_send.car_y_v = 0;
    uart_send.uav_spot = g_waypoint_trigger;
    uart_send.car_a = 1;
    uart_send.car_b = 2;
    uart_send.car_c = 3;
    //开启串口传输
    duta_judge = 1;
    car_uart_get();
    //获取数据赋值
    g_drone_data.stop = uart_get.start;
    g_drone_data.car_x = uart_get.car_x;
    g_drone_data.car_y = uart_get.car_y;
    g_drone_data.beacon_x = uart_get.signs_x;
    g_drone_data.beacon_y = uart_get.signs_y;
    g_drone_data.feature = uart_get.car_direction;
    if(g_drone_data.car_x != 0 && g_drone_data.car_y != 0){ s = 1;}
    //printf("%d\r\n",uart_get.car_x);
}
// **************************** 代码区域 ****************************
