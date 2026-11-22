#include <encoder.h>
int16 encoder_data_R1 = 0;                      //
int16 encoder_data_R2 = 0;
int16 encoder_data_L1 = 0;
int16 encoder_data_L2 = 0;

//函数名称                  encoder_get
//函数功能                  定时编码器获取，中断使用
//返回类型                  void
void encoder_get(void)
{
    encoder_data_R1 = encoder_get_count(ENCODER_R1_INDEX);                // 获取编码器计数
    encoder_clear_count(ENCODER_R1_INDEX);                                // 清空编码器计数
    encoder_data_L1 = -encoder_get_count(ENCODER_L1_INDEX);                // 获取编码器计数
    encoder_clear_count(ENCODER_L1_INDEX);                                // 清空编码器计数
    encoder_data_R2 = encoder_get_count(ENCODER_R2_INDEX);                // 获取编码器计数
    encoder_clear_count(ENCODER_R2_INDEX);                                // 清空编码器计数
    encoder_data_L2 = -encoder_get_count(ENCODER_L2_INDEX);                // 获取编码器计数
    encoder_clear_count(ENCODER_L2_INDEX);                                // 清空编码器计数
}

//函数名称                  encoder_init
//函数功能                  编码器初始化
//返回类型                  void
void encoder_init(void)
{
    encoder_quad_init(ENCODER_L1_INDEX, ENCODER_L1_QUADDEC_A , ENCODER_L1_QUADDEC_B);   // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_L2_INDEX, ENCODER_L2_QUADDEC_A , ENCODER_L2_QUADDEC_B);   // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_R1_INDEX, ENCODER_R1_QUADDEC_A , ENCODER_R1_QUADDEC_B);   // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_R2_INDEX, ENCODER_R2_QUADDEC_A , ENCODER_R2_QUADDEC_B);   // 初始化编码器模块与引脚 正交
}
