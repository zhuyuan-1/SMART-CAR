/**
 * @file    date_processing.c
 * @brief   TOF测距与光流传感器数据处理模块
 * @note    负责双路TOF高度解算/滤波/融合、光流位移滤波/旋转补偿，
 *           最终通过卡尔曼滤波融合IMU加速度与光流/TOF观测值，输出三轴速度。
 */

#include "code.h"
#include "process_rx.h"
#include "navigation.h"

/*===========================================================================
 * 宏定义 —— 所有可调参数统一在此配置
 *===========================================================================*/
#define TOF_MIN_MM              50      // TOF最小有效距离 (mm)，小于此值为盲区
#define TOF_MAX_MM              1300    // TOF最大有效距离 (mm)，超过此值不可信
#define TOF_JUMP_THRESH         300     // [已废弃] TOF跳变检测阈值 (mm)，中值滤波已替代
#define TOF_DIFF_CHECK_THRESH   50.0f   // 双TOF一致性检查差值阈值 (mm)
#define MEDIAN_WINDOW           5       // 中值滤波滑动窗口大小 (点数)
#define FILTER_BETA             0.35f    // 一阶低通滤波系数 (当前值权重0.4，历史值权重0.6)
#define ANGLE_LIMIT_DEG         80.0f   // 姿态角限幅 (±80°)，防止tan/cos奇异
#define EPS_FLOAT               1e-6f   // 浮点比较容差
#define OPTICL_THRESHOID        3       // 光流死区阈值
#define TOF_TASK_PERIOD_MS      25      // TOF任务调用周期 (ms)，对应40Hz

/*===========================================================================
 * 静态变量 —— TOF历史滤波状态 (双路独立)
 *===========================================================================*/
static float tof1_last = 0.0f;          // TOF1 上次滤波输出高度
static float tof2_last = 0.0f;          // TOF2 上次滤波输出高度
static bool  tof1_init = false;         // TOF1 首次初始化标志
static bool  tof2_init = false;         // TOF2 首次初始化标志

/* 中值滤波环形缓冲区 (双路独立) */
static float   tof1_median_buf[MEDIAN_WINDOW] = {0};
static float   tof2_median_buf[MEDIAN_WINDOW] = {0};
static uint8_t tof1_median_idx = 0;     // 当前写入位置
static uint8_t tof1_median_cnt = 0;     // 已累积的有效点数
static uint8_t tof2_median_idx = 0;
static uint8_t tof2_median_cnt = 0;

/*===========================================================================
 * 全局变量 —— 供其他模块读取的传感器融合结果
 *===========================================================================*/
float observe_speed[3];                 // 观测速度 [X, Y, Z] (光流XY + TOF Z)
float tof_now_mm = 0.0f;                // 当前融合后的TOF高度 (mm)
float optical_xdate = 0.0f;             // 光流X方向滤波后位移
float optical_ydate = 0.0f;             // 光流Y方向滤波后位移
float optical_xdate_last = 0.0f;        // 光流X位移历史值
float optical_ydate_last = 0.0f;        // 光流Y位移历史值
float tof_high_speed = 0.0f;            // TOF高度变化率 (m/s)，经低通平滑
float optical_x_speed = 0.0f;           // 光流X方向速度输出
float optical_y_speed = 0.0f;           // 光流Y方向速度输出
float speed_now[3] = {0};               // 卡尔曼融合后的三轴速度输出
static float s_prev_dx_filt = 0.0f;     // 光流X低通滤波历史值
static float s_prev_dy_filt = 0.0f;     // 光流Y低通滤波历史值
static float rot_x_f = 0, rot_y_f = 0;  // 光流旋转补偿滤波历史值
KF_3Axis_Velocity_t speed_kf;           // 三轴卡尔曼滤波器实例
float last_heigh = 0.0f;                // 上次高度值 (用于高度变化检测)

/*===========================================================================
 * 工具函数
 *===========================================================================*/
#define GYRO_FILTER_WINDOW 10  // 窗口大小，可根据振动强度调整 (4~10)

// 在文件全局区定义三个轴的滑动滤波变量
static float buf_x[GYRO_FILTER_WINDOW] = {0};
static float buf_y[GYRO_FILTER_WINDOW] = {0};
static float buf_z[GYRO_FILTER_WINDOW] = {0};
static float sum_x = 0, sum_y = 0, sum_z = 0;
static uint8_t idx = 0;
static uint8_t window_full = 0;

// 每2ms调用一次，传入 icm_data.gyro_x, _y, _z
void Gyro_Filter_2ms(float raw_x, float raw_y, float raw_z,
                     float *filt_x, float *filt_y, float *filt_z)
{
    // 减去旧值，加入新值
    sum_x -= buf_x[idx];
    buf_x[idx] = raw_x;
    sum_x += raw_x;

    sum_y -= buf_y[idx];
    buf_y[idx] = raw_y;
    sum_y += raw_y;

    sum_z -= buf_z[idx];
    buf_z[idx] = raw_z;
    sum_z += raw_z;

    idx++;
    if (idx >= GYRO_FILTER_WINDOW) {
        idx = 0;
        window_full = 1;
    }

    if (window_full) {
        *filt_x = sum_x / GYRO_FILTER_WINDOW;
        *filt_y = sum_y / GYRO_FILTER_WINDOW;
        *filt_z = sum_z / GYRO_FILTER_WINDOW;
    } else {
        // 未满窗口时用已累积点数平均（避免起步为0）
        uint8_t n = idx == 0 ? GYRO_FILTER_WINDOW : idx;
        *filt_x = sum_x / n;
        *filt_y = sum_y / n;
        *filt_z = sum_z / n;
    }
}
/**
 * @brief   浮点绝对值 (替代标准库 fabsf，减少依赖)
 * @param   value  输入值
 * @return  |value|
 */
float my_abs(float value)
{
    return (value >= 0) ? value : -value;
}

/**
 * @brief  pitch 轴初始化归零
 * @param  pitch_gyro: pitch 当前原始角度 (度)
 * @retval 补偿后的 pitch 角度 (度)
 * @note   逻辑与 pitch 完全相同，独立静态变量。
 */
float gyro_pitch_init(float pitch_gyro)
{
    static float sum_pitch = 0.0f;
    static int count_pitch = 0;
    static float offset_pitch = 0.0f;
    static int done_pitch = 0;

    if (done_pitch) {
        return pitch_gyro - offset_pitch;
    } else {
        count_pitch++;
        if (count_pitch > 10) {           // 仅累加第11~30次的数据
            sum_pitch += pitch_gyro;
        }
        if (count_pitch == 30) {
            offset_pitch = sum_pitch / 20.0f;   // 取20次平均
            done_pitch = 1;
        }
        return pitch_gyro;          // 未完成时返回原始值
    }
}

/**
 * @brief  roll 轴初始化归零
 * @param  roll_gyro: roll 当前原始角度 (度)
 * @retval 补偿后的 roll 角度 (度)
 * @note   逻辑与 pitch 完全相同，独立静态变量。
 */
float gyro_roll_init(float roll_gyro)
{
    static float sum_roll = 0.0f;
    static int count_roll = 0;
    static float offset_roll = 0.0f;
    static int done_roll = 0;

    if (done_roll) {
        return roll_gyro - offset_roll;
    } else {
        count_roll++;
        if (count_roll > 10) {
            sum_roll += roll_gyro;
        }
        if (count_roll == 30) {
            offset_roll = sum_roll / 20.0f;
            done_roll = 1;
        }
        return roll_gyro;
    }
}
/**
 * @brief   计算陀螺仪当前角度到目标角度的最短转角差
 * @param   target_angle  目标角度 (度)
 * @param   gyro          当前陀螺仪角度 (度)
 * @return  最短角度差 (度)，范围 [-180, 180]
 * @note    将角度放大100倍转整数运算，避免浮点取模问题
 */
float Turn_gyro_fixed(float target_angle, float gyro)
{
    int32_t target_angle_x100 = (int32_t)(target_angle * 100);
    int32_t gyro_x100 = (int32_t)(gyro * 100);
    int32_t diff = gyro_x100 - target_angle_x100;

    // 归一化到 [-36000, 36000) 范围 (整数取模)
    diff = (diff % 36000 + 36000) % 36000;

    // 映射到 [-18000, 18000] 范围 (最短路径)
    if (diff > 18000) {
        diff -= 36000;
    }

    return (float)diff / 100.0f;
}

/**
 * @brief   将任意目标角度归一化到 (-180, 180] 区间
 * @param   target_angle  原始目标角度 (度)
 * @return  归一化后的角度 (度)
 */
float Turn_target(float target_angle)
{
    // Step 1: 归一化到 [0, 360) 范围
    target_angle = fmodf(target_angle, 360.0f);
    if (target_angle < 0) {
        target_angle += 360.0f;
    }

    // Step 2: 映射到 (-180, 180] 范围
    if (target_angle > 180.0f - EPS_FLOAT) {
        target_angle -= 360.0f;
    }

    // 严格处理 ±180° 边界
    if (fabsf(target_angle - 180.0f) < EPS_FLOAT ||
        fabsf(target_angle + 180.0f) < EPS_FLOAT) {
        target_angle = -180.0f + EPS_FLOAT;
    }

    return target_angle;
}

/*===========================================================================
 * TOF 高度处理流水线
 *===========================================================================*/

/**
 * @brief   根据姿态角将TOF斜距投影为垂直高度
 * @param   tof_mm     TOF原始斜距 (mm)
 * @param   pitch_deg  俯仰角 (度)
 * @param   roll_deg   横滚角 (度)
 * @return  垂直高度 (mm)，异常或超出有效范围返回 -1.0f
 * @note    公式: height = tof × |cos(pitch) × cos(roll)|
 */
float Alt_Solve(uint16_t tof_mm, float pitch_deg, float roll_deg)
{
    // 有效性检查：TOF必须在量程范围内
    if (tof_mm < TOF_MIN_MM || tof_mm > TOF_MAX_MM)
    {
        return -1.0f;
    }

    // 角度限幅：防止大倾角下投影误差过大
    pitch_deg = pitch_deg >  ANGLE_LIMIT_DEG ?  ANGLE_LIMIT_DEG : pitch_deg;
    pitch_deg = pitch_deg < -ANGLE_LIMIT_DEG ? -ANGLE_LIMIT_DEG : pitch_deg;
    roll_deg  = roll_deg  >  ANGLE_LIMIT_DEG ?  ANGLE_LIMIT_DEG : roll_deg;
    roll_deg  = roll_deg  < -ANGLE_LIMIT_DEG ? -ANGLE_LIMIT_DEG : roll_deg;

    float p = pitch_deg * DEG_TO_RAD_R;
    float r = roll_deg  * DEG_TO_RAD_R;
    float height = (float)tof_mm * fabsf(cosf(p) * cosf(r));

    return height;
}

/**
 * @brief   滑动窗口中值滤波 —— 剔除脉冲离群点
 * @param   new_val  新采样值
 * @param   buf      环形缓冲区 (长度 = MEDIAN_WINDOW)
 * @param   p_idx    当前写入位置指针 (环形递增)
 * @param   p_cnt    已累积有效点数指针 (最大 MEDIAN_WINDOW)
 * @return  当前窗口的中值
 * @note    窗口未满时 (前N-1个点) 返回已有数据的中值，自然过渡；
 *          N=5 时排序开销极小，冒泡排序即可胜任。
 */
static float MedianFilter(float new_val, float *buf, uint8_t *p_idx, uint8_t *p_cnt)
{
    uint8_t i, j;

    // 写入环形缓冲区
    buf[*p_idx] = new_val;
    *p_idx = (*p_idx + 1) % MEDIAN_WINDOW;
    if (*p_cnt < MEDIAN_WINDOW) {
        (*p_cnt)++;
    }

    // 复制到临时数组并冒泡排序 (N=5, 最大10次比较)
    float sorted[MEDIAN_WINDOW];
    for (i = 0; i < *p_cnt; i++) {
        sorted[i] = buf[i];
    }
    for (i = 0; i < *p_cnt - 1; i++) {
        for (j = i + 1; j < *p_cnt; j++) {
            if (sorted[i] > sorted[j]) {
                float tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }

    return sorted[*p_cnt / 2];
}

/**
 * @brief   单路TOF数据处理 —— 中值滤波 + 一阶低通级联
 * @param   ch       通道号 (1 或 2)
 * @param   raw_mm   TOF原始斜距 (mm)
 * @param   pitch    当前俯仰角 (度)
 * @param   roll     当前横滚角 (度)
 * @return  滤波后的垂直高度 (mm)，失败返回 -1.0f
 * @note    处理流水线:
 *          原始斜距 → Alt_Solve(投影) → 中值滤波(去脉冲) → 低通滤波(平滑) → 输出
 */
static float Tof_Single_Process(uint8_t ch, uint16_t raw_mm, float pitch, float roll)
{
    float now_h = Alt_Solve(raw_mm, pitch, roll);
    float *p_last;
    bool  *p_init;
    float *p_median_buf;
    uint8_t *p_median_idx;
    uint8_t *p_median_cnt;

    // 根据通道号选择对应的状态变量
    if (ch == 1)
    {
        p_last        = &tof1_last;
        p_init        = &tof1_init;
        p_median_buf  = tof1_median_buf;
        p_median_idx  = &tof1_median_idx;
        p_median_cnt  = &tof1_median_cnt;
    }
    else if (ch == 2)
    {
        p_last        = &tof2_last;
        p_init        = &tof2_init;
        p_median_buf  = tof2_median_buf;
        p_median_idx  = &tof2_median_idx;
        p_median_cnt  = &tof2_median_cnt;
    }
    else
    {
        return -1.0f;
    }

    // 1. 高度解算失败 (量程外或角度超限)，不喂入后续滤波器
    if (now_h < 0.0f)
    {
        return -1.0f;
    }

    // 2. 中值滤波：剔除孤立脉冲离群点，保留真实阶跃
    float median_h = MedianFilter(now_h, p_median_buf, p_median_idx, p_median_cnt);

    // 3. 首次初始化：用中值初始化历史值，不做低通
    if (!(*p_init))
    {
        *p_last = median_h;
        *p_init = true;
        return median_h;
    }

    // 4. 一阶低通滤波：平滑残余高斯噪声
    //    filtered = β × current + (1-β) × previous,  β = 0.4
    float filter_h = median_h * FILTER_BETA + *p_last * (1.0f - FILTER_BETA);
    *p_last = filter_h;

    return filter_h;
}

/**
 * @brief   双路TOF冗余融合
 * @param   h1  TOF1滤波后高度 (mm)，<0 表示无效
 * @param   h2  TOF2滤波后高度 (mm)，<0 表示无效
 * @return  融合后的高度 (mm)
 * @note    融合策略:
 *          - 双路有效 + 差值≤阈值 → 取平均
 *          - 双路有效 + 差值>阈值 → 取更接近上一帧融合值的那一路
 *          - 单路有效 → 使用该路
 *          - 双路失效 → 保持上次融合值
 */
static float Tof_Fuse(float h1, float h2)
{
    bool tof1_ok = (h1 >= 0.0f);
    bool tof2_ok = (h2 >= 0.0f);

    if (tof1_ok && tof2_ok)
    {
        // 双路有效：检查一致性
        if (fabsf(h1 - h2) > TOF_JUMP_THRESH)
        {
            // 差值过大，选择更接近上一次融合高度的那一路
            float diff1 = fabsf(h1 - tof_now_mm);
            float diff2 = fabsf(h2 - tof_now_mm);
            return (diff1 <= diff2) ? h1 : h2;
        }
        else
        {
            // 差值在阈值内，取平均
            return (h1 + h2) / 2.0f;
        }
    }
    else if (tof1_ok && !tof2_ok)
    {
        return h1;
    }
    else if (!tof1_ok && tof2_ok)
    {
        return h2;
    }
    else
    {
        // 双路均失效：保持上一次融合高度，等待传感器恢复
        return tof_now_mm;
    }
}
/**
 * @brief   TOF主接口：双路处理 + 融合 + 计算高度变化率
 * @param   pitch_deg  当前俯仰角 (度)
 * @param   roll_deg   当前横滚角 (度)
 * @note    调用周期为 TOF_TASK_PERIOD_MS (25ms)，
 *          速度计算: (当前高度 - 上次高度) / 周期，
 *          速度低通: 0.7×当前 + 0.3×历史 (比高度滤波更信任当前值)
 */
void tof_get(float pitch_deg, float roll_deg)
{
    float last_fuse  = tof_now_mm;
    float last_speed = tof_high_speed;

    // 分别处理两路TOF，得到各自的有效高度 (无效时为-1)
    float h1 = Tof_Single_Process(1, flow_get.optical_TOF1_mm, pitch_deg, roll_deg);
    float h2 = Tof_Single_Process(2, flow_get.optical_TOF2_mm, pitch_deg, roll_deg);

    // 冗余融合
    tof_now_mm = Tof_Fuse(h1, h2);

    // 计算高度变化率: mm/ms → m/s，再做低通平滑
    float speed_now = (tof_now_mm - last_fuse) / TOF_TASK_PERIOD_MS;
    tof_high_speed = speed_now * 0.5f + last_speed * 0.5f;
}

/*===========================================================================
 * 光流数据处理
 *===========================================================================*/

/**
 * @brief   光流原始位移滤波 —— 一阶低通 + 死区抑制
 * @param   dx_raw     X方向原始位移 (int16)
 * @param   dy_raw     Y方向原始位移 (int16)
 * @param   alpha      低通滤波系数 (0.0~1.0)，值越大越信任当前值
 * @param   threshold  死区阈值，滤波后位移绝对值小于此值时强制置零
 * @param   p_dx_out   输出：滤波后X位移
 * @param   p_dy_out   输出：滤波后Y位移
 * @note    死区抑制的目的：消除悬停状态下光流的微小噪声，
 *          避免飞控产生不必要的微调抖动。推荐 threshold=1~3。
 */
void optical_flow_filter_process(int16 dx_raw, int16 dy_raw, float alpha, int16 threshold,
                                  int16 *p_dx_out, int16 *p_dy_out)
{
    float dx_filt, dy_filt;
    int16 dx_out, dy_out;

    // 一阶低通滤波
    dx_filt = alpha * (float)dx_raw + (1.0f - alpha) * s_prev_dx_filt;
    dy_filt = alpha * (float)dy_raw + (1.0f - alpha) * s_prev_dy_filt;

    s_prev_dx_filt = dx_filt;
    s_prev_dy_filt = dy_filt;

    dx_out = (int16_t)dx_filt;
    dy_out = (int16_t)dy_filt;

    // 死区抑制：小于阈值的位移强制为零
    if (dx_out > -threshold && dx_out < threshold) {
        dx_out = 0;
    }
    if (dy_out > -threshold && dy_out < threshold) {
        dy_out = 0;
    }

    *p_dx_out = dx_out;
    *p_dy_out = dy_out;
}

/**
 * @brief   光流旋转补偿 —— 剔除机体旋转引起的光流分量
 * @param   raw_x, raw_y       PMW3901原始像素位移
 * @param   gyro_x, gyro_y, gyro_z  陀螺仪角速度 (rad/s)
 * @param   height             当前对地高度 (mm)
 * @param   focal_length       光流等效焦距 (像素)
 * @param   dt                 采样间隔 (s)
 * @param   comp_x, comp_y     输出：旋转补偿后的平移速度
 * @note    当前实现采用"条件更新"策略：
 *          - 角速度在 [-25, 25] 范围内 → 更新光流值 (近似静止/缓动)
 *          - 角速度超出范围 → 保持上次光流值 (快速旋转时光流不可信)
 *          旋转补偿项 (rot_x, rot_y) 当前设为0，保留接口待启用。
 */
void flow_compensate_rotation(int16_t raw_x, int16_t raw_y,
                               float gyro_x, float gyro_y, float gyro_z,
                               float height, float focal_length, float dt,
                               float *comp_x, float *comp_y)
{
    float alpht = 0.6f;
    float fx = (float)raw_x;
    float fy = (float)raw_y;
    float optical_xdate;
    float optical_ydate;

    // 旋转补偿项 (待启用: gyro→像素位移的转换)
    // 公式: rot_x = gyro_y × (π/180) × focal_length × dt
    //       rot_y =-gyro_x × (π/180) × focal_length × dt
    float rot_x = 0;
    float rot_y = 0;
    rot_x = gyro_y * (PI/180) * focal_length * dt;
    rot_y =-gyro_x * (PI/180) * focal_length * dt;
    rot_x_f = rot_x * alpht + rot_x_f * (1 - alpht);  // rot 也平滑，匹配 fx 的延迟
    rot_y_f = rot_y * alpht + rot_y_f * (1 - alpht);

    optical_xdate = (fx - rot_x_f) * alpht + optical_xdate_last * (1 - alpht);
    optical_ydate = (fy - rot_y_f) * alpht + optical_ydate_last * (1 - alpht);
    //printf("%f,%f,%f,%f\r\n",rot_x_f,rot_y_f,fx,fy);
    // X方向：陀螺仪X角速度在阈值内才更新
   //if (gyro_x < 50 && gyro_x > -50)
    {
        optical_xdate = (fx + rot_x) * alpht + optical_xdate_last * (1 - alpht);
    }
//    else
//    {
//        optical_xdate = optical_xdate_last;
//    }

    // Y方向：陀螺仪Y角速度在阈值内才更新
    //if (gyro_y < 50 && gyro_y > -50)
    {
        optical_ydate = (fy + rot_y) * alpht + optical_ydate_last * (1 - alpht);
    }
//    else
//    {
//        optical_ydate = optical_ydate_last;
//    }

    last_heigh = height;
    optical_xdate_last = optical_xdate;
    optical_ydate_last = optical_ydate;

    // 像素位移 → 实际速度转换 (除以10000为尺度因子)
    *comp_x = optical_xdate * height / 10000;
    *comp_y = optical_ydate * height / 10000;
}

/**
 * @brief   光流主控制函数 —— 每个控制周期调用一次
 * @param   flow_change  光流更新使能标志 (预留)
 * @note    处理流程:
 *          1. 读取光流模块串口数据 (TOF+光流)
 *          2. TOF高度处理 (双路滤波+融合)
 *          3. 光流位移滤波 (低通+死区)
 *          4. 光流旋转补偿 → 得到水平速度
 *          5. 组装观测速度 → 卡尔曼滤波融合 IMU加速度 → 输出三轴速度
 *
 *          坐标系约定 (observe_speed):
 *          [0] = -optical_x_speed  (机体X轴，取反与导航系对齐)
 *          [1] =  optical_y_speed  (机体Y轴)
 *          [2] = -tof_high_speed   (机体Z轴，取反使向下为正)
 */
void optical_flow_contorl(uint8 flow_change)
{
    int16 op_date_x;
    int16 op_date_y;

    // Step 1: 从光流模块串口读取原始数据
    opyical_flow_get();

    // Step 2: TOF高度处理
    tof_get(eulerAngle.pitch, eulerAngle.roll);

    // Step 3: 光流位移低通滤波 + 死区抑制
    optical_flow_filter_process(flow_get.optical_x_drift, flow_get.optical_y_drift,
                                 0.4f, 2, &op_date_x, &op_date_y);

    // Step 4: 光流旋转补偿 → 水平速度
    flow_compensate_rotation(op_date_x, op_date_y,
                              icm_data.gyro_x, icm_data.gyro_y, icm_data.gyro_z,
                              tof_now_mm, 490.0f, 0.025f,
                              &optical_x_speed, &optical_y_speed);

    // Step 5: 组装观测值 → 卡尔曼融合
    observe_speed[0] = -optical_y_speed;
    observe_speed[1] = -optical_x_speed;
    observe_speed[2] = -tof_high_speed;
    KF_3Axis_Update(&speed_kf, acc_n_filtered, observe_speed, speed_now);
    //printf("%f,%f\r\n",optical_y_speed,optical_x_speed);
    //printf("%f,%f\r\n",speed_now[0]*10,speed_now[1]*10);
}
