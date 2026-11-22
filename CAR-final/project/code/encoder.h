#ifndef CODE_ENCODER_H_
#define CODE_ENCODER_H_

#include "zf_common_headfile.h"

#define ENCODER_L1_INDEX                   (TC_CH07_ENCODER)                // 左前L1正交编码器2对应使用的编码器接口 TC_CH20_ENCODER
#define ENCODER_L1_QUADDEC_A               (TC_CH07_ENCODER_CH1_P07_6)      // A 相对应的引脚 TC_CH20_ENCODER_CH1_P08_1
#define ENCODER_L1_QUADDEC_B               (TC_CH07_ENCODER_CH2_P07_7)      // B 相对应的引脚 TC_CH20_ENCODER_CH2_P08_2


#define ENCODER_L2_INDEX                   (TC_CH58_ENCODER)                // 左后L2正交编码器4对应使用的编码器接口 TC_CH58_ENCODER
#define ENCODER_L2_QUADDEC_A               (TC_CH58_ENCODER_CH1_P17_3)      // A 相对应的引脚 TC_CH58_ENCODER_CH1_P17_3
#define ENCODER_L2_QUADDEC_B               (TC_CH58_ENCODER_CH2_P17_4)      // B 相对应的引脚 TC_CH58_ENCODER_CH2_P17_4


#define ENCODER_R1_INDEX                   (TC_CH20_ENCODER)                // 右前R1正交编码器1对应使用的编码器接口 TC_CH07_ENCODER
#define ENCODER_R1_QUADDEC_A               (TC_CH20_ENCODER_CH1_P08_1)      // A 相对应的引脚 TC_CH07_ENCODER_CH1_P07_6
#define ENCODER_R1_QUADDEC_B               (TC_CH20_ENCODER_CH2_P08_2)      // B 相对应的引脚 TC_CH07_ENCODER_CH2_P07_7


#define ENCODER_R2_INDEX                   (TC_CH27_ENCODER)                // 右后R2正交编码器3对应使用的编码器接口 TC_CH27_ENCODER
#define ENCODER_R2_QUADDEC_A               (TC_CH27_ENCODER_CH1_P19_2)      // A 相对应的引脚 TC_CH27_ENCODER_CH1_P19_2
#define ENCODER_R2_QUADDEC_B               (TC_CH27_ENCODER_CH2_P19_3)      // B 相对应的引脚 TC_CH27_ENCODER_CH2_P19_3

extern int16 encoder_data_R1;
extern int16 encoder_data_L1;
extern int16 encoder_data_R2;
extern int16 encoder_data_L2;


void encoder_init(void);
//定时编码器获取，中断使用
void encoder_get(void);

#endif /* CODE_ENCODER_H_ */
