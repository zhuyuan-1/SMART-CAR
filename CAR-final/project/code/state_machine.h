/*********************************************************************************************************************
* CYT4BB7_CAR1 — 状态机模块（信标追踪 / 惯导航行 / 冲刺控制）
* 文件: state_machine.h
* 说明: 定义了五个状态的状态机，包含信标序列、信标位置等全局数据
*       四个运行时可调参数通过 IPC 从 CM7_1 屏幕修改
********************************************************************************************************************/

#ifndef CODE_STATE_MACHINE_H_
#define CODE_STATE_MACHINE_H_

#include "zf_common_headfile.h"

// 状态机五个状态
typedef enum {
    STATE_INIT          = 0,    // 初始状态：车不动，等待 feature=1
    STATE_BEACON_TRACK  = 1,    // 信标追踪：视觉引导追踪信标
    STATE_INERTIAL      = 2,    // 纯惯导：无信标时依靠惯导走向航点
    STATE_WAIT_DRONE    = 3,    // 等待无人机：视觉丢失或车偏太远
    STATE_CHARGE        = 4     // 冲刺：近距离锁定方向高速冲过信标
} drone_state_e;

// 无人机视觉数据（通过 UART 接收）
typedef struct {
    uint8  feature;           // 0=无目标, 1=车+信标, 2=仅车, 3=车离中心太远
    int16  car_x;             // 车在图像中的像素 X
    int16  car_y;             // 车在图像中的像素 Y
    int16  beacon_x;          // 信标在图像中的像素 X
    int16  beacon_y;          // 信标在图像中的像素 Y
    int16  stop;              // 无人机准备信号
} drone_vision_t;

// feature 编码
#define DRONE_FEATURE_BOTH      1   // 车+信标都在画面中
#define DRONE_FEATURE_CAR_ONLY  2   // 只有车，没有信标
#define DRONE_FEATURE_NONE      3   // 无目标

/********************************************************************************
* 运行时可配置参数一览（通过 CM7_1 屏幕修改 → IPC → CM7_0 生效）
* ============================================================================
* 屏幕路径              变量名                   默认值   范围     说明
* ---------------------------------------------------------------------------
* 参数调试→基础速度     g_beacon_track_speed     20.0    0~?     信标追踪速度
* 参数调试→基础速度     g_inertial_nav_speed     20.0    0~?     惯导航行速度
* 参数调试→冲刺速度     g_charge_speed           25.0    0~?     冲刺速度
* 参数调试→像素阈值     g_pixel_charge_threshold 32      0~500   触发冲刺的像素距离
* 惯导设置→设置数量     g_beacon_count           10      1~10    实际使用的信标个数
* 惯导设置→设置数量     g_target_point_num       10      1~40    实际使用的序列长度
* 惯导设置→信标坐标     beacon_position[8][2]    见.c    -       信标物理坐标
* 惯导设置→信标序列     beacon_sequence[20]      见.c    1~count 信标亮灯顺序
* ============================================================================
* IPC 命令位掩码:
*   bit0 (1): 速度参数  bit1 (2): 像素阈值
*   bit2 (4): 信标序列  bit3 (8): 信标位置
********************************************************************************/

// ---- 信标布局参数 ----
#define LOOP4_POINT_NUM         4       // 四个惯导循环点
#define BEACON_COUNT            10      // 最大信标个数
#define BEACON_POS_SIZE         (BEACON_COUNT + 1)  // 信标位置表大小（含原点，索引0）= 11
#define BEACON_SEQ_SIZE         40      // 信标亮灯序列最大长度
#define TARGET_POINT_NUM        10      // 实际需要遍历的航点数

// ---- 运行时可调参数（通过 IPC 从 CM7_1 屏幕修改，默认值与旧宏一致） ----
extern float    g_beacon_track_speed;       // 视觉追踪时的匀速速度（默认 20.0f）
extern float    g_inertial_nav_speed;       // 惯导航行时的速度（默认 20.0f）
extern float    g_charge_speed;             // 冲刺速度（默认 25.0f）
extern uint32   g_pixel_charge_threshold;   // 像素误差阈值，触发冲刺（默认 32）
extern uint8    g_beacon_count;             // 实际使用的信标个数（默认 7，最大 BEACON_COUNT）
extern uint8    g_target_point_num;         // 实际使用的序列长度（默认 10，最大 BEACON_SEQ_SIZE）

// ---- 编译时常量（不可运行时修改） ----
#define PIXEL_ARRIVAL_THRESHOLD  8      // 像素误差 < 8 认为已到达信标上方
#define PIXEL_DEADBAND           5      // 像素误差 < 5 死区，速度置零防振荡
#define PIXEL_SIGN_LATERAL      (1.0f)  // 像素 X 轴 → target_left 符号（负=翻转）
#define PIXEL_SIGN_FORWARD       (-1.0f) // 像素 Y 轴 → target_front 符号
#define WAYPOINT_TRIGGER_INDEX   12      // 触发航点索引（到达此航点后 g_waypoint_trigger 置 1）
#define CHARGE_TIMEOUT_TICKS     60      // 冲刺最大持续 tick 数（60×10ms=600ms）
#define FIXED_POINT_X            90      // 固定点 X 像素坐标
#define FIXED_POINT_Y            50      // 固定点 Y 像素坐标
#define ALIGN_STOP_DIST          50      // 距离大于此值 → 进入等待对齐状态
#define ALIGN_CONTINUE_DIST      30      // 距离小于此值 → 退出等待对齐状态
#define INIT_DELAY_TICKS         300     // 初始延时 3 秒 = 300 × 10ms
#define WAIT_ALIGN_MIN_TICKS     100     // 最小对齐停留 1 秒（100 × 10ms）
#define WAIT_INERTIAL_MIN_TICKS  300     // 最小惯导停留 3 秒（300 × 10ms）

// 全局数据
extern drone_vision_t g_drone_data;
extern float    beacon_points[BEACON_SEQ_SIZE][3];
extern float    beacon_position[BEACON_POS_SIZE][2];
extern uint8    beacon_sequence[BEACON_SEQ_SIZE];
extern float    loop4_points[LOOP4_POINT_NUM][2];
extern uint32   s;
extern uint32   data_ins;
extern uint8    g_waypoint_trigger;          // 到达触发航点后置 1

// 公开 API
void state_machine_init(void);
void state_machine_run(float x_now, float y_now);
void state_machine_run2(float x_now, float y_now);
void inc_use(float x_now, float y_now, float beacon_points[][3], float target_velocity);
uint8 nav_return_to_origin(float x_now, float y_now, float x_target, float y_target, float target_velocity);
static float get_car_to_fixedpoint_dist(void);
static void run_loop4_inertial(float x_now, float y_now, float vel);
void state_machine_run3(float x_now, float y_now);

#endif /* CODE_STATE_MACHINE_H_ */
