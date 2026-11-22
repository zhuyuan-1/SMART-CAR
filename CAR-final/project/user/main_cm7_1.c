/*********************************************************************************************************************
* CYT4BB7_CAR1 — 显示/UI 核心 CM7_1 主函数
* 文件: main_cm7_1.c
* 说明: 运行 TFT 屏幕 + 按键 + EEPROM 参数存储
*       通过 IPC 共享内存将"保存"的参数推送给 CM7_0 控制核心
********************************************************************************************************************/

// **************************** 代码区域 ****************************
#include "zf_common_headfile.h"

// 以下工程或编译选项设置请在 CMakeLists.txt 或 IDE 中进行
// 第一步：关闭编译器中打开的文件
// 第二步：project->clean，等待重新编译完成

// 本程序是开源扩展程序，允许使用者移植或调试修改
// 本程序是开源扩展程序，允许使用者移植或调试修改
// 本程序是开源扩展程序，允许使用者移植或调试修改

// **************************** 代码区域 ****************************
#include "eeprom.h"
#include "tft.h"

// ---- 双核 IPC 共享内存 ----
#pragma location = IPC_CM7_1_BASE_ADDR
float ipc_cm7_1_data[IPC_DATA_LEN_CM7_1] = {0};  // CM7_1 写入，CM7_0 读取

#pragma location = IPC_CM7_0_BASE_ADDR
__no_init float ipc_cm7_0_data[IPC_DATA_LEN_CM7_0];  // CM7_1 端只读（CM7_0 写入）


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化（必须在最前面）
    debug_info_init();                  // 调试串口信息初始化

    // 此处编写用户代码，例如外设初始化相关操作
    Flash_init();
    //Para_Init();  // 仅首次上电运行一次写入默认值，之后注释掉
    Load_All_From_Flash();  // 启动时从 Flash 加载已保存的参数
    ipc_push_params(1|2|4|8);  // 启动时把所有参数同步给 CM7_0
    KEY_init();
    TFT_init();
    tft180_clear();


    // 此处编写用户代码，例如外设初始化相关操作
    while(true)
    {
        // 此处编写需要循环执行的代码
        KEY_Debounce();
        //printf("%f,%f\r\n",ipc_cm7_1_data[1],ipc_cm7_0_data[1]);
        for(int i=0;i<ins_coord_count;i++)
        {
            //printf("%d: %f,%f\r\n",i,coord_buffer[i].x,coord_buffer[i].y);
        }
        TFT_KEYput(KEY_WasPressed(0),KEY_WasPressed(1),KEY_WasPressed(2));


        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
