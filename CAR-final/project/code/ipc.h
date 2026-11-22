/*********************************************************************************************************************
* CYT4BB7_CAR1 — 双核 IPC 共享内存通信层
* 文件: ipc.h
* 说明: 参考 UAV2 项目的共享内存 + ARM 缓存一致性模式
*       CM7_1 在"保存"时将参数写入 ipc_cm7_1_data[]
*       CM7_0 在 10ms PIT 中断中读取 ipc_cm7_1_data[]，应用修改
*       不使用中断/信号量/IPC管道 —— 纯轮询 + 命令标志位
*
* 物理布局（均在 ICFEDIT_region_RAM 内，起始地址 0x28001000，8 KB）：
*   0x28001000: ipc_cm7_1_data[128]  (512 bytes — CM7_1 → CM7_0)
*   0x28001200: ipc_cm7_0_data[20]   (80 bytes  — CM7_0 → CM7_1)
********************************************************************************************************************/

#ifndef CODE_IPC_H_
#define CODE_IPC_H_

#include "zf_common_headfile.h"

//==================================================== IPC 内存布局 ====================================================
#define IPC_DATA_LEN_CM7_1       128          // CM7_1 → CM7_0 数据数组长度
#define IPC_DATA_LEN_CM7_0       20           // CM7_0 → CM7_1 数据数组长度
#define IPC_CM7_1_BASE_ADDR      0x28001200    // ICFEDIT_region_RAM 起始地址（8 KB 共享区）
#define IPC_CM7_0_BASE_ADDR      0x28001000   // 从 CM7_1 基址偏移 512 字节

//==================================================== CM7_1 → CM7_0 索引映射 ====================================================
#define IPC_IDX_CMD              0            // 命令位掩码（float）：0=空闲, bit0=速度, bit1=像素阈值, bit2=信标序列, bit3=信标位置
#define IPC_IDX_BEACON_TRACK     1            // float: 信标追踪速度
#define IPC_IDX_INERTIAL_NAV     2            // float: 惯导航行速度
#define IPC_IDX_CHARGE           3            // float: 冲刺速度
#define IPC_IDX_PIXEL_THRESH     4            // float（使用时转为 uint32）: 像素冲刺阈值
#define IPC_IDX_BEACON_SEQ       5            // 5..44: 信标序列（40 个 uint8 以 float 形式存储）
#define IPC_IDX_BEACON_POS       45           // 45..66: 信标位置（11 个位置 × 2 个 float x,y）
#define IPC_IDX_BEACON_COUNT     67           // float（转为 uint8）: 实际信标个数
#define IPC_IDX_TARGET_POINT_NUM 68           // float（转为 uint8）: 实际序列长度
// 索引 69..127 保留，供将来扩展

//==================================================== CM7_0 → CM7_1 索引映射 ====================================================
#define IPC_ACK_IDX              0            // float: 回显已处理的命令值

//==================================================== 命令位掩码常量 ====================================================
// 以 float 存储，使用时转为 int 进行位判断
#define IPC_CMD_NONE             0.0f         // 空闲 —— 无挂起命令
#define IPC_CMD_SPEED_PARAMS     1.0f         // bit 0: 速度参数已更新
#define IPC_CMD_PIXEL_THRESH     2.0f         // bit 1: 像素冲刺阈值已更新
#define IPC_CMD_BEACON_SEQ       4.0f         // bit 2: 信标序列已更新
#define IPC_CMD_BEACON_POS       8.0f         // bit 3: 信标位置已更新

//==================================================== 共享数组声明 ====================================================
extern float ipc_cm7_1_data[IPC_DATA_LEN_CM7_1];   // CM7_1 写入，CM7_0 读取
extern float ipc_cm7_0_data[IPC_DATA_LEN_CM7_0];   // CM7_0 写入，CM7_1 读取

//==================================================== 缓存一致性说明 ====================================================
// 参考 UAV2 双核通信方式，直接调用 CMSIS 函数 SCB_CleanInvalidateDCache_by_Addr
// 写入自己的数组后 + 读取对方数组前，对两个数组各调用一次：
//   SCB_CleanInvalidateDCache_by_Addr(&ipc_cm7_1_data, sizeof(ipc_cm7_1_data));
//   SCB_CleanInvalidateDCache_by_Addr(&ipc_cm7_0_data, sizeof(ipc_cm7_0_data));
// 该函数同时执行 Clean（刷脏行到内存）和 Invalidate（标记缓存行为无效），
// 确保写入对对方核心可见，读取从物理内存重新获取。

#endif /* CODE_IPC_H_ */
