/*********************************************************************************************************************
* CYT4BB7_CAR1 — 状态机实现（有序信标 / 无序信标两种模式）
* 文件: state_machine.c
********************************************************************************************************************/

#include "code1.h"

// ---- 运行时可配置参数（通过 IPC 从 CM7_1 屏幕修改，默认值与旧宏一致） ----
float  g_beacon_track_speed     = 15.0f;
float  g_inertial_nav_speed     = 15.0f;
float  g_charge_speed           = 40.0f;
uint32 g_pixel_charge_threshold = 25;
uint8  g_beacon_count           = 10;   // 实际信标个数
uint8  g_target_point_num       = 40;   // 实际序列长度

// 状态变量
static drone_state_e g_state = STATE_INIT;
drone_vision_t g_drone_data;
uint32 data_ins = 0;
uint32 s = 0;
uint8  g_waypoint_trigger = 0;       // 到达触发航点后置 1
static uint16 g_ins_delay_ticks = 0;

// 用于 STATE_INIT 中的延时等待
static uint16 g_init_delay_ticks = 0;
static uint8  g_init_waiting = 0;     // 0=未开始等待, 1=正在等待延时

//-------------------------------------------------------------------
// 已知所有惯导位置的参数设置（有序信标模式）
//-------------------------------------------------------------------
// 七个信标亮灯顺序（按任务需求排列，值为信标编号 1-7）
uint8 beacon_sequence[BEACON_SEQ_SIZE] = {
            1,5,3,2,4,6,3,2,4,5,6,2,1,5,1,2,1,2,3,4,
            1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4
};
// 七个信标物理坐标（索引 0=原点, 1-7=信标）
float beacon_position[BEACON_POS_SIZE][2] = {
            {0.0f,   0.0f},   // 0: 原点
            {-1.0f,  2.3f},   // 1: 信标
            {-1.3f,  4.2f},   // 2: 信标
            {0.0f,   5.6f},   // 3: 信标
            {0.0f,   4.0f},   // 4: 信标
            {1.0f,   2.3f},   // 5: 信标
            {2.3f,   4.2f},   // 6: 信标
            {1.0f,   1.0f},   // 7: 信标
            {1.0f,   1.0f},   // 8: 信标
            {1.0f,   1.0f},   // 9: 信标
            {1.0f,   1.0f},   // 10:信标
};
// 展开后的航点 [x, y, beacon_id]，由 state_machine_init() 填充
float beacon_points[BEACON_SEQ_SIZE][3] = {0};

// ---- 冲刺状态锁定变量 ----
static uint16   g_charge_ticks  = 0;
static float    g_charge_dir_x  = 0.0f;   // 锁定方向的归一化 X 分量
static float    g_charge_dir_y  = 0.0f;   // 锁定方向的归一化 Y 分量

//-------------------------------------------------------------------
// 未知所有惯导位置的参数设置（无序信标模式）
//-------------------------------------------------------------------
float loop4_points[LOOP4_POINT_NUM][2] = {
    {2.5f, 2.5f},
    {2.5f, 4.5f},
    {-2.5f, 4.5f},
    {-2.5f, 2.5f}
};
static uint8 g_loop4_idx = 1;        // 当前循环点位下标 1~4

//-------------------------------------------------------------------
// 辅助函数：获取车到固定点的像素距离
//-------------------------------------------------------------------
//static float get_car_to_fixedpoint_dist(void)
//{
//    int16 dx = g_drone_data.car_x - FIXED_POINT_X;
//    int16 dy = g_drone_data.car_y - FIXED_POINT_Y;
//    return sqrtf((float)(dx*dx + dy*dy));
//}

//-------------------------------------------------------------------
// 定点惯导速度输出（输入单位：米）
// 返回值: 1=已到达目标点, 0=未到达
//-------------------------------------------------------------------
uint8 nav_return_to_origin(float x_now, float y_now, float x_target, float y_target, float target_velocity)
{
    float x_error = x_target - x_now;
    float y_error = y_target - y_now;
    float dist = sqrtf(x_error*x_error + y_error*y_error);
    float target_velocity_x = 0;
    float target_velocity_y = 0;

    // 速度分解
    if(dist > 1e-6f)
    {
        target_velocity_x = x_error / dist * target_velocity;
        target_velocity_y = y_error / dist * target_velocity;
    }

    // 死区清零 + 直接用带方向的速度
    if (fabsf(x_error) <= 0.03f)
        target_left = 0;
    else
        target_left = target_velocity_x;

    if (fabsf(y_error) <= 0.03f)
        target_front = 0;
    else
        target_front = target_velocity_y;

    // 到达判断
    if ((fabsf(y_error) <= 0.03) && (fabsf(x_error) <= 0.03))
        return 1;
    else
        return 0;
}

//-------------------------------------------------------------------
// 多点惯导速度输出（输入单位：米）
//-------------------------------------------------------------------
void inc_use(float x_now, float y_now, float beacon_points[][3], float target_velocity)
{
    static uint8 task_done = 0;

    if (task_done)
    {
        target_front = 0.0f;
        target_left  = 0.0f;
        return;
    }

    if (data_ins < g_target_point_num)
    {
        if (nav_return_to_origin(x_now, y_now,
            beacon_points[data_ins][0], beacon_points[data_ins][1], target_velocity) == 1)
        {
            data_ins++;
        }
    }
    else
    {
        if (nav_return_to_origin(x_now, y_now, 0.0f, 0.0f, target_velocity) == 1)
        {
            task_done = 1;
            target_front = 0.0f;
            target_left  = 0.0f;
        }
    }
}

//-------------------------------------------------------------------
// 状态机主逻辑 —— 有序信标模式
//-------------------------------------------------------------------
void state_machine_run(float x_now, float y_now)
{
    // 航点触发检查：到达第 WAYPOINT_TRIGGER_INDEX 个航点后置 1
    if (data_ins >= WAYPOINT_TRIGGER_INDEX) {
        g_waypoint_trigger = 1;
    }

    // 任务完成，返回原点
    if (data_ins >= g_target_point_num) {
        if (nav_return_to_origin(x_now, y_now, 0.0f, 0.0f, g_inertial_nav_speed) == 1)
        {
            data_ins = 0;
            g_state = STATE_INIT;
            target_front = 0.0f;
            target_left  = 0.0f;
        }
        return;
    }

    uint8 feature = g_drone_data.feature;

    switch (g_state) {
    // ====== 状态 0: 初始 ======
    case STATE_INIT:
        target_front = 0.0f;
        target_left  = 0.0f;

        // 检查启动条件
        if ((feature == DRONE_FEATURE_CAR_ONLY || feature == DRONE_FEATURE_BOTH)
            && s == 1 && g_drone_data.stop == 1) {
            if (!g_init_waiting) {
                // 首次满足条件，开始计时
                g_init_waiting = 1;
                g_init_delay_ticks = 0;
            }
        }
        if(g_init_waiting == 1){
            // 正在计时中
            g_init_delay_ticks++;
            if (g_init_delay_ticks >= INIT_DELAY_TICKS) {
                g_state = STATE_BEACON_TRACK;
                g_init_waiting = 0;
                g_init_delay_ticks = 0;
            }
        }
        break;

    // ====== 状态 1: 视觉追踪信标 ======
    case STATE_BEACON_TRACK:
        if (feature == DRONE_FEATURE_CAR_ONLY || feature == DRONE_FEATURE_NONE)
        // 只有车或者都没有
        {
            g_state = STATE_INERTIAL;   // 切换到惯导
        }
        else
        // 车和信标都有
        {
            int16 dx = g_drone_data.beacon_x - g_drone_data.car_x;  // X 误差
            int16 dy = g_drone_data.beacon_y - g_drone_data.car_y;  // Y 误差
            float norm = sqrtf((float)(dx * dx + dy * dy));
            if (norm < 1e-6f)
            {
                target_front = 0.0f;
                target_left  = 0.0f;
            }
            else if (norm < PIXEL_DEADBAND)
            {
                // 太近，死区，停止
                target_front = 0.0f;
                target_left  = 0.0f;
            }
            else if (norm < g_pixel_charge_threshold)
            {
                g_state = STATE_CHARGE;
                g_charge_ticks = 0;
                // 锁定归一化方向
                float inv = 1.0f / norm;
                g_charge_dir_x = (float)dx * inv;
                g_charge_dir_y = (float)dy * inv;
                // 立即输出冲刺速度
                float speed1 = PIXEL_SIGN_LATERAL * g_charge_dir_x * g_charge_speed;
                float speed2 = PIXEL_SIGN_FORWARD * g_charge_dir_y * g_charge_speed;
                if(speed1 >= 0) {
                    target_left = speed1 + 20;
                    target_front = speed2;
                }
                else {
                    target_left = speed1 - 20;
                    target_front = speed2;
                }
            }
            else
            {
                // 匀速追踪
                float inv = 1.0f / norm;
                target_left  = PIXEL_SIGN_LATERAL * (dx * inv) * g_beacon_track_speed;
                target_front = PIXEL_SIGN_FORWARD * (dy * inv) * g_beacon_track_speed;
            }
        }
        break;

    // ====== 状态 2: 纯惯导航行 ======
    case STATE_INERTIAL:
        if (feature == DRONE_FEATURE_BOTH)
        {
            // 信标重新出现，切回视觉追踪
            g_state = STATE_BEACON_TRACK;
        }
        else
        {
            if (data_ins < g_target_point_num)
            {
                uint8 arrived = nav_return_to_origin(
                    x_now, y_now,
                    beacon_points[data_ins][0],
                    beacon_points[data_ins][1],
                    g_inertial_nav_speed);
                if (arrived)
                {
                    g_ins_delay_ticks++;
                    if(g_ins_delay_ticks >= WAIT_INERTIAL_MIN_TICKS)
                    {
                        data_ins++;
                    }
                }
            }
        }
        break;

    // ====== 状态 3: 等待无人机 ======
    case STATE_WAIT_DRONE:
        target_front = 0.0f;
        target_left  = 0.0f;
        if (feature == DRONE_FEATURE_BOTH)
        {
            g_state = STATE_BEACON_TRACK;   // 两个都有 → 进入追踪
        }
        else if (feature == DRONE_FEATURE_CAR_ONLY)
        {
            g_ins_delay_ticks = 0;
            g_state = STATE_INERTIAL;        // 只有车 → 进入惯导
        }
        break;

    // ====== 状态 4: 冲刺冲过信标 ======
    case STATE_CHARGE:
        // 冲刺期间无视 feature 变化 —— 车与信标在IR画面中融合，
        // vision 会将两者识别为一个 blob，feature 会跳变（1→2 或 1→0），
        // 此时视觉数据不可靠，方向已锁定，只管按计时器冲
        g_charge_ticks++;

        if (g_charge_ticks >= CHARGE_TIMEOUT_TICKS)
        {
            // 冲刺完成，根据当前 feature 决定下一个状态
            signal = 0;
            if (feature == DRONE_FEATURE_BOTH)
            {
                int16 dx1 = g_drone_data.beacon_x - g_drone_data.car_x;
                int16 dy1 = g_drone_data.beacon_y - g_drone_data.car_y;
                float norm1 = sqrtf((float)(dx1 * dx1 + dy1 * dy1));
                if(norm1 > (g_pixel_charge_threshold))
                {
                    data_ins++;
                    g_state = STATE_BEACON_TRACK;
                }
                else
                {
                    g_state = STATE_BEACON_TRACK;
                }
            }
            else
            {
                data_ins++;
                g_ins_delay_ticks = 0;
                g_state = STATE_INERTIAL;
            }
        }
        else
        {
            // 保持冲刺：锁定方向、恒定高速
            signal = 1;
            target_left  = PIXEL_SIGN_LATERAL * g_charge_dir_x * g_charge_speed;
            target_front = PIXEL_SIGN_FORWARD * g_charge_dir_y * g_charge_speed;
        }
        break;

    default:
        g_state = STATE_INIT;
        break;
    }
}

//-------------------------------------------------------------------
// 状态机初始化：将信标亮灯序列展开为航点坐标数组
//-------------------------------------------------------------------
void state_machine_init(void)
{
    for (uint8 i = 0; i < g_target_point_num; i++)
    {
        uint8 id = beacon_sequence[i];
        if (id > BEACON_COUNT)
        {
            id = 0;     // 非法信标编号，使用原点兜底
        }
        beacon_points[i][0] = beacon_position[id][0];
        beacon_points[i][1] = beacon_position[id][1];
        beacon_points[i][2] = (float)id;    // 记录信标编号，方便调试
    }
}

//-------------------------------------------------------------------
// 状态机主逻辑 —— 无序信标模式
//-------------------------------------------------------------------
void state_machine_run2(float x_now, float y_now)
{
    uint8 feature = g_drone_data.feature;

    switch (g_state) {
    // ====== 状态 0: 初始 ======
    case STATE_INIT:
        target_front = 0.0f;
        target_left  = 0.0f;
        if ((feature == DRONE_FEATURE_CAR_ONLY || feature == DRONE_FEATURE_BOTH)
            && s == 1 && g_drone_data.stop == 1) {
            if (!g_init_waiting) {
                g_init_waiting = 1;
                g_init_delay_ticks = 0;
            }
        }
        if(g_init_waiting == 1){
            g_init_delay_ticks++;
            if (g_init_delay_ticks >= INIT_DELAY_TICKS) {
                g_state = STATE_BEACON_TRACK;
                g_init_waiting = 0;
                g_init_delay_ticks = 0;
            }
        }
        break;

    // ====== 状态 1: 视觉追踪信标 ======
    case STATE_BEACON_TRACK:
        if (feature == DRONE_FEATURE_CAR_ONLY || feature == DRONE_FEATURE_NONE)
        {
            g_state = STATE_INERTIAL;
        }
        else
        {
            int16 dx = g_drone_data.beacon_x - g_drone_data.car_x;
            int16 dy = g_drone_data.beacon_y - g_drone_data.car_y;
            float norm = sqrtf((float)(dx * dx + dy * dy));
            if (norm < 1e-6f)
            {
                target_front = 0.0f;
                target_left  = 0.0f;
            }
            else if (norm < PIXEL_DEADBAND)
            {
                target_front = 0.0f;
                target_left  = 0.0f;
            }
            else if (norm < g_pixel_charge_threshold)
            {
                g_state = STATE_CHARGE;
                g_charge_ticks = 0;
                float inv = 1.0f / norm;
                g_charge_dir_x = (float)dx * inv;
                g_charge_dir_y = (float)dy * inv;
                float speed1 = PIXEL_SIGN_LATERAL * g_charge_dir_x * g_charge_speed;
                float speed2 = PIXEL_SIGN_FORWARD * g_charge_dir_y * g_charge_speed;
                if(speed1 >= 0) {
                    target_left = speed1 + 20;
                    target_front = speed2;
                }
                else {
                    target_left = speed1 - 20;
                    target_front = speed2;
                }
            }
            else
            {
                float inv = 1.0f / norm;
                target_left  = PIXEL_SIGN_LATERAL * (dx * inv) * g_beacon_track_speed;
                target_front = PIXEL_SIGN_FORWARD * (dy * inv) * g_beacon_track_speed;
            }
        }
        break;

    // ====== 状态 2: 纯惯导航行 ======
    case STATE_INERTIAL:
        if (feature == DRONE_FEATURE_BOTH)
        {
            g_state = STATE_BEACON_TRACK;
        }
        else
        {
            run_loop4_inertial(x_now, y_now, g_inertial_nav_speed);
        }
        break;

    // ====== 状态 3: 等待无人机 ======
    case STATE_WAIT_DRONE:
        target_front = 0.0f;
        target_left  = 0.0f;
        if (feature == DRONE_FEATURE_BOTH)
        {
            g_state = STATE_BEACON_TRACK;
        }
        else if (feature == DRONE_FEATURE_CAR_ONLY)
        {
            g_state = STATE_INERTIAL;
        }
        break;

    // ====== 状态 4: 冲刺冲过信标 ======
    case STATE_CHARGE:
        g_charge_ticks++;

        if (g_charge_ticks >= CHARGE_TIMEOUT_TICKS)
        {
            signal = 0;
            if (feature == DRONE_FEATURE_CAR_ONLY || feature == DRONE_FEATURE_NONE)
                { g_state = STATE_INERTIAL; }
            else
                { g_state = STATE_BEACON_TRACK; }
        }
        else
        {
            signal = 1;
            target_left  = PIXEL_SIGN_LATERAL * g_charge_dir_x * g_charge_speed;
            target_front = PIXEL_SIGN_FORWARD * g_charge_dir_y * g_charge_speed;
        }
        break;

    default:
        g_state = STATE_INIT;
        break;
    }
}

//// 四点循环惯导执行函数：到达后自动切下一点，循环往复
//static void run_loop4_inertial(float x_now, float y_now, float vel)
//{
//    float tar_x = loop4_points[g_loop4_idx][0];
//    float tar_y = loop4_points[g_loop4_idx][1];
//    uint8 arrive_flag = nav_return_to_origin(x_now, y_now, tar_x, tar_y, vel);
//
//    // 到达直接切换下一个点，不需要停留
//    if(arrive_flag)
//    {
//        g_loop4_idx = (g_loop4_idx + 1) % LOOP4_POINT_NUM;
//    }
//}

// 四点循环惯导执行函数：到达后自动切下一点，循环往复
static void run_loop4_inertial(float x_now, float y_now, float vel)
{
    float tar_x = beacon_position[g_loop4_idx][0];
    float tar_y = beacon_position[g_loop4_idx][1];
    uint8 arrive_flag = nav_return_to_origin(x_now, y_now, tar_x, tar_y, vel);

    // 到达直接切换下一个点，不需要停留
    if(arrive_flag)
    {
        g_loop4_idx = g_loop4_idx  % 4 + 1;
    }
}
//-------------------------------------------------------------------
// 状态机主逻辑 —— 有序信标模式
//-------------------------------------------------------------------
void state_machine_run3(float x_now, float y_now)
{
    // 航点触发检查：到达第 WAYPOINT_TRIGGER_INDEX 个航点后置 1
    if (data_ins >= WAYPOINT_TRIGGER_INDEX) {
        g_waypoint_trigger = 1;
    }

    // 任务完成，返回原点
    if (data_ins >= g_target_point_num) {
        if (nav_return_to_origin(x_now, y_now, 0.0f, 0.0f, g_inertial_nav_speed) == 1)
        {
            data_ins = 0;
            g_state = STATE_INIT;
            target_front = 0.0f;
            target_left  = 0.0f;
        }
        return;
    }

    uint8 feature = g_drone_data.feature;

    switch (g_state) {
    // ====== 状态 0: 初始 ======
    case STATE_INIT:
        target_front = 0.0f;
        target_left  = 0.0f;

        // 检查启动条件
        if ((feature == DRONE_FEATURE_CAR_ONLY || feature == DRONE_FEATURE_BOTH)
            && s == 1 && g_drone_data.stop == 1) {
            if (!g_init_waiting) {
                // 首次满足条件，开始计时
                g_init_waiting = 1;
                g_init_delay_ticks = 0;
            }
        }
        if(g_init_waiting == 1){
            // 正在计时中
            g_init_delay_ticks++;
            if (g_init_delay_ticks >= INIT_DELAY_TICKS) {
                g_state = STATE_BEACON_TRACK;
                g_init_waiting = 0;
                g_init_delay_ticks = 0;
            }
        }
        break;

    // ====== 状态 1: 视觉追踪信标 ======
    case STATE_BEACON_TRACK:
        if (feature == DRONE_FEATURE_CAR_ONLY || feature == DRONE_FEATURE_NONE)
        // 只有车或者都没有
        {
            g_state = STATE_INERTIAL;   // 切换到惯导
        }
        else
        // 车和信标都有
        {
            int16 dx = g_drone_data.beacon_x - g_drone_data.car_x;  // X 误差
            int16 dy = g_drone_data.beacon_y - g_drone_data.car_y;  // Y 误差
            float norm = sqrtf((float)(dx * dx + dy * dy));
            if (norm < 1e-6f)
            {
                target_front = 0.0f;
                target_left  = 0.0f;
            }
            else if (norm < PIXEL_DEADBAND)
            {
                // 太近，死区，停止
                target_front = 0.0f;
                target_left  = 0.0f;
            }
            else if (norm < g_pixel_charge_threshold)
            {
                g_state = STATE_CHARGE;
                g_charge_ticks = 0;
                // 锁定归一化方向
                float inv = 1.0f / norm;
                g_charge_dir_x = (float)dx * inv;
                g_charge_dir_y = (float)dy * inv;
                // 立即输出冲刺速度
                float speed1 = PIXEL_SIGN_LATERAL * g_charge_dir_x * g_charge_speed;
                float speed2 = PIXEL_SIGN_FORWARD * g_charge_dir_y * g_charge_speed;
                if(speed1 >= 0) {
                    target_left = speed1 + 20;
                    target_front = speed2;
                }
                else {
                    target_left = speed1 - 20;
                    target_front = speed2;
                }
            }
            else
            {
                // 匀速追踪
                float inv = 1.0f / norm;
                target_left  = PIXEL_SIGN_LATERAL * (dx * inv) * g_beacon_track_speed;
                target_front = PIXEL_SIGN_FORWARD * (dy * inv) * g_beacon_track_speed;
            }
        }
        break;

    // ====== 状态 2: 纯惯导航行 ======
    case STATE_INERTIAL:
        if (feature == DRONE_FEATURE_BOTH)
        {
            // 信标重新出现，切回视觉追踪
            g_state = STATE_BEACON_TRACK;
        }
        else
        {
            if (data_ins < g_target_point_num)
            {
                uint8 arrived = nav_return_to_origin(
                    x_now, y_now,
                    beacon_points[data_ins][0],
                    beacon_points[data_ins][1],
                    g_inertial_nav_speed);
                if (arrived)
                {
                    data_ins++;
                }
            }
        }
        break;

    // ====== 状态 4: 冲刺冲过信标 ======
    case STATE_CHARGE:
        // 冲刺期间无视 feature 变化 —— 车与信标在IR画面中融合，
        // vision 会将两者识别为一个 blob，feature 会跳变（1→2 或 1→0），
        // 此时视觉数据不可靠，方向已锁定，只管按计时器冲
        g_charge_ticks++;
        if (g_charge_ticks >= CHARGE_TIMEOUT_TICKS)
        {
            // 冲刺完成，根据当前 feature 决定下一个状态
            signal = 0;
            if (feature == DRONE_FEATURE_BOTH)
            {
                g_state = STATE_BEACON_TRACK;
            }
            else
            {
                g_state = STATE_INERTIAL;
            }
        }
        else
        {
            // 保持冲刺：锁定方向、恒定高速
            signal = 1;
            target_left  = PIXEL_SIGN_LATERAL * g_charge_dir_x * g_charge_speed;
            target_front = PIXEL_SIGN_FORWARD * g_charge_dir_y * g_charge_speed;
        }
        break;

    default:
        g_state = STATE_INIT;
        break;
    }
}