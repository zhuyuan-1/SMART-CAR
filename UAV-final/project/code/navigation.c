#include "navigation.h"
#include "process_rx.h"

#define IMAGE_MIDDLE_X 94
#define IMAGE_MIDDLE_Y 60
#define LPF_ALPHA  0.4f//低通滤波参数
#define DEG_TO_RAD_1 0.017453292519943295769236907684886// 度转弧度系数
// 导航系下的加速度和速度
float acc_n[3];//n系下的加速度
float acc_n_b[3];//n系下的b系的x,y加速度便于控制
float vel_imu[3] = {0, 0, 0};//n系下的速度
float acc_n_filtered[3] = {0.0f};//低通滤波后的加速度
float last_speed=0;
int nag_x_sum=0;
int nag_y_sum=0;

// 函数名称          transform_accel_to_nav
// 函数功能          将机体坐标系加速度转换到导航坐标系（N系），并消除重力分量
// 参数说明          void（通过全局变量 icm_data 和 eulerAngle 读取输入）
// 备注              加速度单位为 m/s²（已从 g 转换），ZYX 旋转顺序
void transform_accel_to_nav(void)
{
    // 将角度转换为弧度
   float roll_rad = eulerAngle.roll * DEG_TO_RAD_1;
   float pitch_rad = eulerAngle.pitch * DEG_TO_RAD_1;
   float yaw_rad = 0.0f;


    float cr = cosf(roll_rad), sr = sinf(roll_rad);
    float cp = cosf(pitch_rad), sp = sinf(pitch_rad);
    float cy = cosf(yaw_rad), sy = sinf(yaw_rad);


    // 旋转矩阵 R_b_n (机体到导航) - ZYX 顺序
    // 仅计算水平方向 X, Y 以适配光流
    float ax_b = icm_data.acc_x*9.84;
    float ay_b = icm_data.acc_y*9.84;
    float az_b = icm_data.acc_z*9.84;

    // 步骤2：先消除b系中的重力分量（关键！）
   // 重力在b系下的分量：g_b = R_n_b * [0, 0, 9.81]^T
   float gx_b = -sp * 9.80f;                  // 重力在b系X轴分量
   float gy_b = sr * cp * 9.80;            // 重力在b系Y轴分量
   float gz_b = -cr * cp * 9.80f;            // 重力在b系Z轴分量
   ax_b -= gx_b;  // 消除X轴重力干扰
   ay_b -= gy_b;  // 消除Y轴重力干扰
   az_b -= gz_b;  // 消除Z轴重力干扰

   acc_n[0] = (cp * cy) * ax_b + (sr * sp * cy - cr * sy) * ay_b + (cr * sp * cy + sr * sy) * az_b;
   acc_n[1] = (cp * sy) * ax_b + (sr * sp * sy + cr * cy) * ay_b + (cr * sp * sy - sr * cy) * az_b;
   acc_n[2] = (-sp) * ax_b + (sr * cp) * ay_b + (cr * cp) * az_b;
   acc_n_b[0] = acc_n[0]*sy+acc_n[1]*cy;
   acc_n_b[1] = acc_n[0]*cy+acc_n[1]*sy;
   acc_n_b[2] = acc_n[2];
}

// 函数名称          acc_init
// 函数功能          加速度低通滤波器初始化
// 参数说明          void
// 备注              将滤波后的加速度缓存清零
void acc_init(void)
{
    acc_n_filtered[0] = 0.0f;
    acc_n_filtered[1] = 0.0f;
    acc_n_filtered[2] = 0.0f;
}

// 函数名称          imu_update_n_acc
// 函数功能          对导航系加速度进行一阶低通滤波
// 参数说明          ax：X轴加速度（N系，m/s²）
// 参数说明          ay：Y轴加速度（N系，m/s²）
// 参数说明          az：Z轴加速度（N系，m/s²）
// 备注              滤波参数 LPF_ALPHA 根据实际效果调整，效果不好可改用截断滤波或卡尔曼滤波
void imu_update_n_acc(float ax, float ay, float az)
{
    // 一阶低通滤波
    acc_n_filtered[0] = LPF_ALPHA * acc_n_filtered[0] + (1-LPF_ALPHA) * ax;
    acc_n_filtered[1] = LPF_ALPHA * acc_n_filtered[1] + (1-LPF_ALPHA) * ay;
    acc_n_filtered[2] = LPF_ALPHA * acc_n_filtered[2] + (1-LPF_ALPHA) * az;
}

// 函数名称          calculate_imu_velocity
// 函数功能          计算IMU速度（加速度坐标转换 + 低通滤波）
// 参数说明          dt：中断间隔（单位：s）
// 备注              放在主函数或定时中断中调用
void calculate_imu_velocity(float dt)
{

    transform_accel_to_nav();
    imu_update_n_acc(acc_n_b[0], acc_n_b[1], acc_n_b[2]);
    //printf("%f,%f,%f\r\n",acc_n_filtered[0],acc_n_filtered[1],acc_n_filtered[2]);
}

// 函数名称          reset_imu_velocity
// 函数功能          IMU速度清零
// 参数说明          void
// 备注              将三轴累积速度置零
void reset_imu_velocity(void)
{
    vel_imu[0] = 0.0f;
    vel_imu[1] = 0.0f;
    vel_imu[2] = 0.0f;

}

// 函数名称          init_single_axis
// 函数功能          单轴卡尔曼滤波器参数初始化（内部复用函数）
// 参数说明          axis：单轴卡尔曼滤波结构体指针
// 参数说明          dt：采样时间（单位：s）
// 参数说明          Q00：速度过程噪声 (0,0) 元素
// 参数说明          Q01：过程噪声 (0,1) 元素
// 参数说明          Q11：偏置过程噪声 (1,1) 元素
// 参数说明          R：观测噪声
// 备注              封装单轴初始化逻辑，避免冗余代码
static  void init_single_axis(KF_Axis_t *axis, float dt, float Q00, float Q01, float Q11, float R)
    {
        axis->x[0] = 0.0f; // 初始速度
        axis->x[1] = 0.0f; // 初始加速度偏置
        // 初始协方差矩阵
        axis->P[0][0] = 1.0f; axis->P[0][1] = 0.0f;
        axis->P[1][0] = 0.0f; axis->P[1][1] = 0.01f;
        // 过程噪声矩阵
        axis->Q[0][0] = Q00; axis->Q[0][1] = Q01;
        axis->Q[1][0] = Q01; axis->Q[1][1] = Q11;
        // 观测噪声
        axis->R = R;
        axis->dt = dt;
    }

// 函数名称          KF_3Axis_Init
// 函数功能          三轴卡尔曼滤波参数初始化
// 参数说明          kf：三轴卡尔曼滤波结构体指针
// 参数说明          dt：采样时间（单位：s）
// 备注              X/Y轴使用光流参数（Q11=0.1, R=0.01），Z轴使用TOF参数（Q11=0.001, R=0.05）
void KF_3Axis_Init(KF_3Axis_Velocity_t *kf, float dt)
{
    // ------------------- X/Y轴（光流）参数 -------------------
    float Q00_xy = dt*dt * 0.05f;   // 速度过程噪声
    float Q01_xy = 0.0f;
    float Q11_xy = 0.1f;        // 偏置过程噪声（放大）
    float R_val_xy = 0.01f;        // 观测噪声（放大，降低光流权重）
    // ------------------- Z轴（TOF）参数 -------------------
    float Q00_z = dt*dt * 0.05f;   // 速度过程噪声（原参数）
    float Q01_z = 0.0f;
    float Q11_z = 0.001f;         // 偏置过程噪声（原参数）
    float R_val_z = 0.05f;        // 观测噪声（原参数，TOF精度高）

    // X轴初始化（光流参数）
    init_single_axis(&kf->x, dt, Q00_xy, Q01_xy, Q11_xy, R_val_xy);
    // Y轴初始化（光流参数）
    init_single_axis(&kf->y, dt, Q00_xy, Q01_xy, Q11_xy, R_val_xy);
    // Z轴初始化（TOF参数）
    init_single_axis(&kf->z, dt, Q00_z, Q01_z, Q11_z, R_val_z);
}

// 函数名称          KF_Axis_Update
// 函数功能          单轴卡尔曼滤波更新（预测-校正循环，内部复用函数）
// 参数说明          axis：单轴卡尔曼滤波结构体指针
// 参数说明          imu_acc：IMU该轴加速度（N系，m/s²），作为控制输入
// 参数说明          last_vel：该轴速度观测值（N系，m/s），来自光流或TOF
// 返回值            融合后的该轴速度（m/s）
// 备注              包含极值保护：速度超过 6m/s 时回退到上一帧值
static float KF_Axis_Update(KF_Axis_t *axis, float imu_acc, float last_vel)
{
    // 1. 预测步（基于IMU加速度）
    float dt = axis->dt;
    float v_pred = axis->x[0] + (imu_acc - axis->x[1]) * dt; // 预测速度
    float bias_pred = axis->x[1];                            // 预测加速度偏置

    // 修正：协方差预测（应用状态转移矩阵F）
    float P00 = axis->P[0][0];
    float P01 = axis->P[0][1];
    float P10 = axis->P[1][0];
    float P11 = axis->P[1][1];

    float P00_pred = P00 - dt*(P01 + P10) + dt*dt*P11 + axis->Q[0][0];
    float P01_pred = P01 - dt*P11 + axis->Q[0][1];
    float P10_pred = P10 - dt*P11 + axis->Q[1][0];
    float P11_pred = P11 + axis->Q[1][1];

    // 2. 更新步（融合速度）
    float deno = P00_pred + axis->R;
    if (deno < 1e-6f) deno = 1e-6f; // 防止分母为0

    float K0 = P00_pred / deno; // 卡尔曼增益K0（速度）
    float K1 = P10_pred / deno; // 卡尔曼增益K1（偏置）

    // 状态更新
    float residual = last_vel - v_pred; // 残差（观测值 - IMU预测值）
    axis->x[0] = v_pred + K0 * residual; // 速度更新
    axis->x[1] = bias_pred + K1 * residual; // 加速度偏置更新

    // 修正：协方差矩阵更新（修复P10计算，强制对称）
    axis->P[0][0] = (1 - K0) * P00_pred;
    axis->P[0][1] = (1 - K0) * P01_pred;
    axis->P[1][0] = P10_pred - K1 * P00_pred; // 核心修正：K0→K1，P10_pred→P00_pred
    axis->P[1][1] = P11_pred - K1 * P01_pred;

    // 优化：强制协方差矩阵对称，避免浮点误差累积
    axis->P[0][1] = axis->P[1][0];
    if(axis->x[0]>6)
        axis->x[0]=last_speed;
    last_speed=axis->x[0];
    return axis->x[0]; // 返回融合后的速度
}

// 函数名称          KF_3Axis_Update
// 函数功能          三轴卡尔曼滤波更新（输入IMU三轴加速度、TOF三轴速度，输出融合后三轴速度）
// 参数说明          kf：三轴卡尔曼滤波结构体指针
// 参数说明          imu_acc：IMU三轴加速度数组 [x,y,z]（N系，m/s²）
// 参数说明          last_vel：上一次三轴速度数组 [x,y,z]（N系，m/s）
// 参数说明          out_vel：输出融合后的三轴速度数组 [x,y,z]（N系，m/s）
// 备注              out_vel需提前分配内存（长度≥3），包含空指针保护
void KF_3Axis_Update(KF_3Axis_Velocity_t *kf, float imu_acc[3], float last_vel[3], float out_vel[3])
{
    // 检查指针有效性（工程化保护）
    if (kf == NULL || imu_acc == NULL || last_vel == NULL || out_vel == NULL) {
        // 错误处理：默认值
        out_vel[0] = 0.0f;
        out_vel[1] = 0.0f;
        out_vel[2] = 0.0f;
        return;
    }

    // X轴滤波更新
    out_vel[0] = KF_Axis_Update(&kf->x, imu_acc[0], last_vel[0]);
    // Y轴滤波更新
    out_vel[1] = KF_Axis_Update(&kf->y, imu_acc[1], last_vel[1]);
    // Z轴滤波更新
    out_vel[2] = KF_Axis_Update(&kf->z, imu_acc[2], last_vel[2]);
}

// 函数名称          Nag_init
// 函数功能          视觉位移累加器初始化
// 参数说明          void
// 备注              将 X/Y 方向累加位移清零
void Nag_init(void)
{
    nag_x_sum=0;
    nag_y_sum=0;
}

// 函数名称          Nag_Read
// 函数功能          根据图像中小车坐标计算相对位移（累加）
// 参数说明          image_x：图像中小车 X 坐标
// 参数说明          image_y：图像中小车 Y 坐标
// 备注              累加值限幅 ±1000，防止溢出
void Nag_Read(uint8 image_x,uint8 image_y)
{
    nag_x_sum+=IMAGE_MIDDLE_X-image_x;
    nag_y_sum+=IMAGE_MIDDLE_Y-image_y;
    nag_x_sum=(int)fminf(fmaxf((float)nag_x_sum, -1000.0f), 1000.0f);
    nag_y_sum=(int)fminf(fmaxf((float)nag_y_sum, -1000.0f), 1000.0f);
}
