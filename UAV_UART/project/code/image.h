#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_
#include"zf_common_headfile.h"
//宏定义
#define BOUNDARY_NUM            (MT9V03X_H * 3 / 2)// 边界的点数量远大于图像高度，便于保存回弯的情况
#define image_h 120     //图像高度
#define image_w 188     //图像宽度
#define white_pixel 255 //二值化
#define black_pixel 0   //二值化
#define BCN_DARK    2
#define BCN_LIGHT   1
#define BCN_NONE    0
#define DARK    0
#define LIGHT   1
#define BRIGHTNESS_MIDDLE_THRESHOLD         (240)//光点阈值


extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];//图像数组
extern uint8 dst_image[image_h][image_w];//标记数组
extern int index_min_use;
extern uint8 x_beacon;
extern uint8 y_beacon;
extern uint8  x_car;
extern uint8  y_car;
extern uint8  car_image_flag;
// 团点结构体定义
struct sBrightRegion {
    uint32 sum_x;      // 团点x坐标累加和
    uint32 sum_y;      // 团点y坐标累加和
    uint32 sum_pixel;  // 团点像素点之和
    uint16 counter;    // 团点像素个数
    uint8 center_x;    // 团点中心x坐标
    uint8 center_y;    // 团点中心y坐标
    uint8 grade_flag;  // 信标信任标记等级
    uint8 car_flag;    // 小车标志位
    uint16 x_min;      // 区域左边界
    uint16 x_max;      // 区域右边界
    uint16 y_min;      // 区域上边界
    uint16 y_max;      // 区域下边界
    float ratio;       // 团点长宽比
    float solid;       // 团点致密性
    float snr;         // SNR值
};

typedef struct
{
        int16 row;
        int16 col;
}Point_Structure;
//亮点行列结构体

extern Point_Structure point_up[64];
extern uint8 stop_count,stop_flag;
extern int16 distance[64];
extern uint8 br_index;               // 有效团点个数（最多10个）
extern uint32 image_result;          
extern uint8 image_copy[MT9V03X_H][MT9V03X_W];//复制图像
extern uint8 point_corner[10][2];
extern uint8 corner_count;
extern uint8 src_image[MT9V03X_H][MT9V03X_W];
//extern struct sBrightRegion br[10];   // 团点结构体数组（主缓存）

void point_draw_init(uint8 image[MT9V03X_H][MT9V03X_W]);

void point_draw(uint8 point_count,uint8 (*point_coordinate)[2]);
//获取图像
void Get_image(uint8(*mt9v03x_image)[image_w]);
//大津法计算图像阈值
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row);
//图像二值化
void turn_to_dst(uint8 img[][image_w],uint8 Threshold);
//建立预设点
void IMAGE_CreatPoint(void);
//亮团检测与团点结构体更新
void IMAGE_ChangeStructBr(struct sBrightRegion br[10], uint8 dst[image_h][image_w], uint8 img[image_h][image_w]);
//图像处理最后得到可以发送给cpu0的数据
void image_process(struct sBrightRegion br[10]);
//打包
uint32 pack_two_uint8_to_uint32(uint8_t a, uint8_t b);
//判断闪烁点
void accumulateForeground(uint8 *img,uint8 *preimg,uint8 *diffimg,uint8 Threshold);
//大津法计算图像阈值（逐行）
int MyOSTU(int width,int height,uint8 *Image);
//寻找闪烁坐标
void IMAGE_GetFlick(struct sBrightRegion br[10],uint8* bcn_x, uint8* bcn_y);
//特征值判断
int feature_value(struct sBrightRegion br[10]);
//图像处理
void IMGE_CONTROL(struct sBrightRegion br[10]);
#endif /* CODE_IMAGE_H_ */
