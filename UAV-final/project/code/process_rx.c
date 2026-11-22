#include "process_rx.h"
//宏定义处，各种参数
#define LPF_ALPHA_E  0.7f//低通滤波参数
#define delta_T    0.002//更新时间（单位s）
#define GyroOffset_Xdata    -1.928722//-0.711828//陀螺仪校准值
#define GyroOffset_Ydata    -3.828905//-1.455508//陀螺仪校准值
#define GyroOffset_Zdata    -2.851391//-2.745385//陀螺仪校准值
#define AccOffset_Xdata    0//陀螺仪校准值
#define AccOffset_Ydata    0//陀螺仪校准值
#define AccOffset_Zdata    0//陀螺仪校准值
//全局变量
float acc_data[3]={0,0,0};            //低通滤波后加速度计原始数据
float gyro_data[3]={0,0,0};           //低通滤波后陀螺仪计原始数据
float pitch_last = 0;                 // 用于截断滤波
float roll_last = 0;                  // 用于截断滤波
float yaw_last = 0;                   // 用于截断滤波
float param_Kp = 0.0;              //比例系数（加速1dx修正权重，越大修正越快，易超调）
float param_Ki = 0.000001;               //积分系数
float I_ex, I_ey, I_ez;               //姿态误差积分项（PI控制用，修正陀螺仪长期漂移）
//结构体变量
quater_param_t_rx Q_info;
euler_param_t_rx eulerAngle;
icm_param_t_rx icm_data;

Kalman2D kf;
Attitude_2D_t att;

//------------------------------------------------------------------------------------------------------------------
//  函数名称        Kalman_init
//  函数功能        初始化六轴扩展卡尔曼滤波器结构体
//  参数说明        filter         : 滤波器实例指针
//  参数说明        dt             : 采样周期 (秒)
//  参数说明        Q_yaw          : 偏航角过程噪声方差
//  参数说明        Q_pitch_roll   : 俯仰/横滚角过程噪声方差
//  参数说明        Q_bias         : 陀螺仪零偏过程噪声方差
//  参数说明        R_pitch_roll   : 俯仰/横滚角测量噪声方差
//  返回值          无
//------------------------------------------------------------------------------------------------------------------
void Kalman2D_Init(Kalman2D* kf, float dt,
                   float Q_pitch, float Q_roll,
                   float R_pitch, float R_roll)
{
    kf->dt = dt;
    kf->Q_pitch = Q_pitch;
    kf->Q_roll  = Q_roll;
    kf->R_pitch = R_pitch;
    kf->R_roll  = R_roll;

    kf->pitch = 0.0f;
    kf->roll  = 0.0f;

    kf->P00 = 1.0f;
    kf->P01 = 0.0f;
    kf->P11 = 1.0f;
}
//------------------------------------------------------------------------------------------------------------------
//  函数名称        Kalman_update
//  函数功能        六轴扩展卡尔曼滤波更新（仅加速度计修正俯仰/横滚，无磁力计）
//  参数说明        result  : 输出滤波后的姿态及无偏角速度
//  参数说明        filter  : 滤波器实例指针
//  参数说明        Acc_X, Acc_Y, Acc_Z : 加速度计测量值 (m/s^2)
//  参数说明        Gyro_X, Gyro_Y, Gyro_Z : 陀螺仪测量值 (deg/s)
//  返回值          无
//------------------------------------------------------------------------------------------------------------------
void Kalman2D_Update(Kalman2D* kf, Attitude_2D_t* out,
                     float acc_x, float acc_y, float acc_z,
                     float gyro_x, float gyro_y, float gyro_z)
{
    float dt = kf->dt;

    // ---------- 0. 限制当前角度 ----------
    if (kf->pitch >  85.0f) kf->pitch =  85.0f;
    if (kf->pitch < -85.0f) kf->pitch = -85.0f;

    // ---------- 1. 由加速度计计算测量角度 ----------
    float meas_pitch = kf->pitch;
    float meas_roll  = kf->roll;
    float norm_axz = sqrtf(acc_x*acc_x + acc_z*acc_z);
    float norm_ayz = sqrtf(acc_y*acc_y + acc_z*acc_z);
    if (norm_axz > 0.5f && norm_ayz > 0.5f) {
        meas_pitch = atan2f(-acc_x, acc_z) * RAD_TO_DEG_R;
        meas_roll  = atan2f( acc_y, acc_z) * RAD_TO_DEG_R;
        if (meas_pitch >  90.0f) meas_pitch =  90.0f;
        if (meas_pitch < -90.0f) meas_pitch = -90.0f;
        if (meas_roll  >  90.0f) meas_roll  =  90.0f;
        if (meas_roll  < -90.0f) meas_roll  = -90.0f;
    }

    // ---------- 2. 状态预测 (陀螺仪积分) ----------
    // 注意：这里假设 gyro_y 影响 pitch，gyro_x 影响 roll
    // 请根据您的实际坐标轴调整符号
    float pitch_inc = gyro_y * dt;
    float roll_inc  = gyro_x * dt;


    float pitch_prior = kf->pitch + pitch_inc;
    float roll_prior  = kf->roll  + roll_inc;

    if (pitch_prior >  85.0f) pitch_prior =  85.0f;
    if (pitch_prior < -85.0f) pitch_prior = -85.0f;

    // ---------- 3. 协方差预测 (线性模型，F = I) ----------
    kf->P00 += kf->Q_pitch;
    kf->P11 += kf->Q_roll;
    // P01 不变 (假设状态转移无耦合)


    // ---------- 4. 新息计算 ----------
    float y_pitch = meas_pitch - pitch_prior;
    float y_roll  = meas_roll  - roll_prior;

    // 新息限幅
    if (y_pitch >  30.0f) y_pitch =  30.0f;
    if (y_pitch < -30.0f) y_pitch = -30.0f;
    if (y_roll  >  30.0f) y_roll  =  30.0f;
    if (y_roll  < -30.0f) y_roll  = -30.0f;

    // 角度归一化
    if (y_pitch >  180.0f) y_pitch -= 360.0f;
    if (y_pitch < -180.0f) y_pitch += 360.0f;
    if (y_roll  >  180.0f) y_roll  -= 360.0f;
    if (y_roll  < -180.0f) y_roll  += 360.0f;

    // ---------- 5. 卡尔曼增益 (测量独立) ----------
    float S_pitch = kf->P00 + kf->R_pitch;
    float S_roll  = kf->P11 + kf->R_roll;

    float K_pitch = kf->P00 / S_pitch;
    float K_roll  = kf->P11 / S_roll;

    // ---------- 6. 状态更新 ----------
    kf->pitch = pitch_prior + K_pitch * y_pitch;
    kf->roll  = roll_prior  + K_roll  * y_roll;

    if (kf->pitch >  85.0f) kf->pitch =  85.0f;
    if (kf->pitch < -85.0f) kf->pitch = -85.0f;

    // ---------- 7. 协方差更新 ----------
    kf->P00 *= (1.0f - K_pitch);
    kf->P11 *= (1.0f - K_roll);

    // ---------- 8. 输出 (yaw 直接传递，不滤波) ----------
    out->pitch = kf->pitch;
    out->roll  = kf->roll;
    out->yaw   = 0.0f;   // 无法估计，外部可自行累加 gyro_z * dt
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------
//函数名称          float fast_sqrt(float x)
//函数功能          快速开方倒数函数
//参数说明          x：被开方数
//参数说明          y：开方后的数
//备注
//------------------------------------------------------------------------------------------------------------------------------------------------------------
float fast_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;  // 浮点转整型（利用二进制位操作加速）
    i = 0x5f3759df - (i >> 1);  // 牛顿迭代法初始值（魔数，嵌入式常用优化）
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));  // 一次牛顿迭代，精度足够嵌入式使用
    return y;
}


//-----------------------------------------------------------------------------------------------------------------------------------------------------------
//函数名称      imu_rx_init()
//函数功能      imu初始化
//参数说明      无
//备注         在初始化处使用
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void imu_rx_init()
{
    acc_data[0]=0;
    acc_data[1]=0;
    acc_data[2]=0;
    gyro_data[0]=0;
    gyro_data[1]=0;
    gyro_data[2]=0;
    eulerAngle.Dirchange = 0;
    eulerAngle.last_yaw = 0;
    I_ex = I_ey = I_ez = 0;
    Kalman2D_Init(&kf, 0.002f,   // dt
              0.01f, 0.01f, // Q_pitch, Q_roll
              0.015f, 0.015f);    // R_pitch, R_roll
    Q_info.q0 = 1;
    Q_info.q1 = 0;
    Q_info.q2 = 0;
    Q_info.q3 = 0;
    while(1)
        {
             if(imu660rc_init(IMU660RC_QUARTERNION_120HZ))                          // 设置 imu660rc 以120HZ的速度产生中断触发信号
        {
           printf("\r\n imu660rc init error.");                                 // imu660rc 初始化失败
        }
        else
        {
           break;
        }
        //gpio_toggle_level(LED1);                                                // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
        }
    system_delay_ms(10);
//    for(int i=0; i<50; i++){
//        acc_data[0] = LPF_ALPHA_E * acc_data[0] + (1-LPF_ALPHA_E) * imu660rc_acc_x;
//        acc_data[1] = LPF_ALPHA_E * acc_data[1] + (1-LPF_ALPHA_E) * imu660rc_acc_y;
//        acc_data[2] = LPF_ALPHA_E * acc_data[2] + (1-LPF_ALPHA_E) * imu660rc_acc_z;
//        //printf("%d,%d,%d\r\n", imu660rc_acc_x, imu660rc_acc_y, imu660rc_acc_z);
//    }
//    // 2. 根据加速度计计算初始欧拉角（重力向量推导）
//    float pitch_init = atan2(-acc_data[0], sqrt(acc_data[1]*acc_data[1] + acc_data[2]*acc_data[2])) * 180.0f / PI;
//    float roll_init  = atan2(acc_data[1], acc_data[2]) * 180/PI;
//    // 3. 将初始欧拉角转换为四元数，覆盖原单位四元数
//    float pitch_rad = pitch_init * PI/180;
//    float roll_rad  = roll_init * PI/180;
//    Q_info.q0 = cos(roll_rad/2) * cos(pitch_rad/2);
//    Q_info.q1 = sin(roll_rad/2) * cos(pitch_rad/2);
//    Q_info.q2 = cos(roll_rad/2) * sin(pitch_rad/2);
//    Q_info.q3 = -sin(roll_rad/2) * sin(pitch_rad/2);
//    // 4. 归一化四元数
//    float norm = fast_sqrt(Q_info.q0*Q_info.q0 + Q_info.q1*Q_info.q1 + Q_info.q2*Q_info.q2 + Q_info.q3*Q_info.q3);
//    Q_info.q0 *= norm;
//    Q_info.q1 *= norm;
//    Q_info.q2 *= norm;
//    Q_info.q3 *= norm;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------
//函数名称      imu_GyroOffset(void)
//函数功能      获取陀螺仪校准值
//参数说明      void
//备注         当你发现你们的陀螺仪老是飘的时候就用这个重新计算陀螺仪校准值
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void imu_GyroOffset(void)
{
   float sum_x=0,sum_y=0,sum_z=0;
        for(int i=0;i<2000;i++)
        {
            printf("\r\n %f,%f,%f\r\n", gyro_data[0], gyro_data[1], gyro_data[2]);
            sum_x+=gyro_data[0];
            sum_y+=gyro_data[1];
            sum_z+=gyro_data[2];
        }
        printf("end...%f,%f,%f\r\n",sum_x/(float)2000,sum_y/(float)2000,sum_z/(float)2000);
        system_delay_ms(1000);
        //printf("%f,%f,%f\r\n",eulerAngle.pitch,eulerAngle.roll,eulerAngle.yaw);
}


//-----------------------------------------------------------------------------------------------------------------------------------------------------------
//函数名称      void imu_update_n_acc(float acc_x, float acc_y, float acc_z,float gyro_x, float gyro_y, float gyro_z)
//函数功能      一阶低通滤波
//参数说明      acc_x，acc_y，acc_z：所需要滤波的加速度计原始数据
//备注         LPF_ALPHA：滤波参数根据自己imu静止时输出的原始数据的效果实际来调
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void imu_update_acc(float acc_x, float acc_y, float acc_z,float gyro_x, float gyro_y, float gyro_z)
{
    // 一阶低通滤波
    acc_data[0] = LPF_ALPHA_E * acc_data[0] + (1-LPF_ALPHA_E) * acc_x;
    acc_data[1] = LPF_ALPHA_E * acc_data[1] + (1-LPF_ALPHA_E) * acc_y;
    acc_data[2] = LPF_ALPHA_E * acc_data[2] + (1-LPF_ALPHA_E) * acc_z;
    gyro_data[0] = LPF_ALPHA_E * gyro_data[0] + (1-LPF_ALPHA_E) * gyro_x;
    gyro_data[1] = LPF_ALPHA_E * gyro_data[1] + (1-LPF_ALPHA_E) * gyro_y;
    gyro_data[2] = LPF_ALPHA_E * gyro_data[2] + (1-LPF_ALPHA_E) * gyro_z;
    // 再判断死区（零偏处理）
    if((gyro_data[0] - GyroOffset_Xdata) <= 1.0f&&(gyro_data[0] - GyroOffset_Xdata)>-1.0)
        gyro_data[0] = GyroOffset_Xdata;   // 归零偏
    
    if((gyro_data[1] - GyroOffset_Ydata) <= 1.0f&&(gyro_data[1] - GyroOffset_Ydata)>-1.0)
        gyro_data[1] = GyroOffset_Ydata;
    
    if((gyro_data[2] - GyroOffset_Zdata) <= 1.0f&&(gyro_data[2] - GyroOffset_Zdata)>-1.0)
        gyro_data[2] = GyroOffset_Zdata;
//    acc_data[0] = imu660rc_acc_x;
//    acc_data[1] = imu660rc_acc_y;
//    acc_data[2] = imu660rc_acc_z;
//    gyro_data[0] = imu660rc_gyro_x;
//    gyro_data[1] = imu660rc_gyro_y;
//    gyro_data[2] = imu660rc_gyro_z;
//    printf("%f,%f,%f\r\n", gyro_data[0], gyro_data[1], gyro_data[2]);
//   printf("%f,%f,%f\r\n", acc_data[0], acc_data[1], acc_data[2]);
//    printf("%f,%f,%f\r\n", gyro_x, gyro_y, gyro_z);
}

//--------------------------------------------------------------------------------------------------------------------------------
//函数名称      ICM_getValues()
//函数功能      滤波校准处理数据并储存
//参数说明      无
//备注         这里的imu660rc_acc_transition(acc_data[0])换成你imu型号所适配的acc_transition比如说ra，rb
//--------------------------------------------------------------------------------------------------------------------------------
void ICM_getValues()
{

    imu_update_acc((float)imu660rc_acc_x,  (float)imu660rc_acc_y,  (float)imu660rc_acc_z, (float)imu660rc_gyro_x, (float)imu660rc_gyro_y, (float)imu660rc_gyro_z);
    //printf("%d,%d,%d\r\n", imu660rc_acc_x, imu660rc_acc_y, imu660rc_acc_z);
    icm_data.acc_x = (float)(acc_data[0]-AccOffset_Xdata)/4098.36;
    icm_data.acc_y = (float)(acc_data[1]-AccOffset_Ydata)/4098.36;
    icm_data.acc_z = (float)(acc_data[2]-AccOffset_Zdata)/4098.36;

    icm_data.gyro_x = ((float)(gyro_data[0] - GyroOffset_Xdata)/14.2857);
    icm_data.gyro_y = ((float)(gyro_data[1] - GyroOffset_Ydata)/14.2857);
    icm_data.gyro_z = ((float)(gyro_data[2] - GyroOffset_Zdata)/14.2857);
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------
//函数名称          ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az)
//函数功能          基于PI控制的四元数更新，融合陀螺仪和加速度计数据
//参数说明          输入加速度数据和陀螺仪数据
//备注
//------------------------------------------------------------------------------------------------------------------------------------------------------------
void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
    float halfT = 0.5 * delta_T;
    float vx, vy, vz;
    float ex, ey, ez;
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    //float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    //float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    float delta_2;

    float norm = fast_sqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    //vz = (q0*q0-0.5f+q3 * q3) * 2;

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;
    //printf("%f,%f\r\n",ax,vx);

    I_ex += ex * delta_T;
    I_ey += ey * delta_T;
    I_ez += ez * delta_T;
    I_ex = fminf(fmaxf(I_ex, -1.0f), 1.0f);
    I_ey = fminf(fmaxf(I_ey, -1.0f), 1.0f);
    I_ez = fminf(fmaxf(I_ez, -1.0f), 1.0f);
    //printf("%f,%f\r\n",ez,gz);
    gx = gx + param_Kp * ex + param_Ki * I_ex;
    gy = gy + param_Kp * ey + param_Ki * I_ey;
    gz = gz + param_Kp * ez + param_Ki * I_ez;




    delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);

    float dq0 = (-q1*gx - q2*gy - q3*gz) * halfT;
    float dq1 = ( q0*gx + q2*gz - q3*gy) * halfT;
    float dq2 = ( q0*gy - q1*gz + q3*gx) * halfT;
    float dq3 = ( q0*gz + q1*gy - q2*gx) * halfT;

    q0 = (1 - delta_2/8)*q0 + dq0;
    q1 = (1 - delta_2/8)*q1 + dq1;
    q2 = (1 - delta_2/8)*q2 + dq2;
    q3 = (1 - delta_2/8)*q3 + dq3;



    norm = fast_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
}

//----------------------------------------------------------------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------------------------------------------------------------------
void ICM_getEulerianAngles(void)
{
//    imu963ra_get_acc();                                                         // 获取 IMU963RA 的加速度测量数值
//    imu963ra_get_gyro();                                                        // 获取 IMU963RA 的角速度测量数值
//    imu660rc_get_acc ();
//    imu660rc_get_gyro ();
    ICM_getValues();
//     printf("%f,%f,%f\r\n", icm_data.acc_x, icm_data.acc_y, icm_data.acc_z);
//     printf("%f,%f,%f\r\n", icm_data.gyro_x, icm_data.gyro_y, icm_data.gyro_z);
//   printf("%d,%d,%d\r\n", imu660rc_gyro_x, imu660rc_gyro_y, imu660rc_gyro_z);
    ICM_AHRSupdate(icm_data.gyro_x*PI/180, icm_data.gyro_y*PI/180, icm_data.gyro_z*PI/180, icm_data.acc_x, icm_data.acc_y, icm_data.acc_z);
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    Kalman2D_Update(&kf, &att, icm_data.acc_x, icm_data.acc_y, -icm_data.acc_z,
                              icm_data.gyro_x,  icm_data.gyro_y,  icm_data.gyro_z);
//    Kalman2D_Update(&kf, &att,0, 0, 1,
//                              0.01,0.01,0.01);
//    printf("%f,%f,%f\r\n", icm_data.gyro_x, icm_data.gyro_y, icm_data.gyro_z);
//    eulerAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / PI; // pitch
//    eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / PI; // roll
      eulerAngle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180/ PI; // yaw
      eulerAngle.pitch = att.pitch;
      eulerAngle.roll = att.roll;
//   printf("%f,%f,%f\r\n", eulerAngle.yaw, eulerAngle.yaw, eulerAngle.yaw);
//      eulerAngle.yaw=imu660rc_yaw;
//      eulerAngle.pitch=imu660rc_pitch;
//     eulerAngle.roll=imu660rc_roll;
//      eulerAngle.yaw=imu660rc_yaw;
//    if (eulerAngle.yaw >= 180) { eulerAngle.yaw -= 360; }
//    else if (eulerAngle.yaw <= -180) { eulerAngle.yaw += 360; }
//    if (fabsf(eulerAngle.pitch) > 70.0f)
//    {
//        eulerAngle.roll = eulerAngle.last_roll;  // 需要定义并保存上一次值
//        eulerAngle.yaw  = eulerAngle.last_yaw;
//    }
    // 更新 last 值
    eulerAngle.last_roll = eulerAngle.roll;
    eulerAngle.last_yaw  = eulerAngle.yaw;
    //
    if((eulerAngle.yaw-eulerAngle.last_yaw) < -350) eulerAngle.Dirchange++;
    else if ((eulerAngle.yaw-eulerAngle.last_yaw) > 350) eulerAngle.Dirchange--;
    eulerAngle.last_yaw=eulerAngle.yaw;
    //******************保留两位小数****************************/
    eulerAngle.roll = roundf(eulerAngle.roll * 100.0f) / 100.0f;
    eulerAngle.pitch = roundf(eulerAngle.pitch * 100.0f) / 100.0f;
    eulerAngle.yaw = roundf(eulerAngle.yaw * 100.0f) / 100.0f;
    //******************截断滤波****************************/
    if(func_abs(pitch_last - eulerAngle.pitch) <= 0.05)
        eulerAngle.pitch = pitch_last;
    if(func_abs(roll_last - eulerAngle.roll) <= 0.05)
        eulerAngle.roll = roll_last;
    if(func_abs(yaw_last - eulerAngle.yaw) <= 0.5)
        eulerAngle.yaw = yaw_last;
    pitch_last = eulerAngle.pitch;
    roll_last = eulerAngle.roll;
    yaw_last = eulerAngle.yaw;

}









