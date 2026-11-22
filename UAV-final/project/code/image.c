#include "image.h"

/* ============================================================================
   全局变量声明
   ============================================================================ */
uint8  original_image[image_h][image_w];
uint8  src_image[MT9V03X_H][MT9V03X_W];
uint8  x_beacon = 2;
uint8  y_beacon = 2;
float  x_beacon_f = 94.0f;
float  y_beacon_f = 40.0f;
uint8  x_car = 2;
uint8  y_car = 2;
float  x_car_f = 94.0f;
float  y_car_f = 40.0f;
uint8  car_image_flag = 0;
uint8  beacon_image_flag = 0;
uint8  filter_inited = 0;
uint8  image_flag = 0;
uint8  xy_x_boundary[BOUNDARY_NUM], xy_y_boundary[BOUNDARY_NUM];
uint8  xy_x_2_boundary[BOUNDARY_NUM], xy_y_2_boundary[BOUNDARY_NUM];
uint8  xy_x_3_boundary[BOUNDARY_NUM], xy_y_3_boundary[BOUNDARY_NUM];
uint8  threshold = 0;
uint16 point_cnt = 0;                               // 全局或静态变量，每次调用前清零
int16  point_detect_offset_row[8] = {0, -1, -1, -1, 0, 1, 1, 1};
int16  point_detect_offset_col[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
int car_index;
static uint8_t beacon_lost_cnt = 0; // 连续丢失计数
static uint8 car_lost_cnt = 0;  //小车丢失计数
/* 定义像素在圆形邻域中的偏移 (Bresenham 圆, 半径3) */
static const int circle_x[16] = {0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1};
static const int circle_y[16] = {-3, -3, -2, -1, 0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3};
uint8  stop_count, stop_flag;                       // 丢点时间计数和标志位

#define CORNER_MERGE_DIST  30                        // 角点合并距离阈值（像素）
#define MAX_CONTOUR_PTS    1000                     // 最大轮廓点数
#define ALPHA             0.3                       //信标滤波值
#define BEACON_LOST_MAX  150          // 允许连续丢失的最大帧数
#define BLOB_EDGE_GAP_MAX       15     // 亮团边缘间隙阈值(px)，小于此值视为"紧贴"（线缆分裂特征）
#define BEACON_CAR_MIN_DIST     35    // 信标-小车可疑距离阈值(px)，新信标出现在此距离内触发审查
typedef struct {
    int x, y;
    int score; // 角点响应得分
} CornerPoint;

//----------------------------------------------------------------------------------------------------------------------------------------------------------
//  函数名称        : point_draw_init(void *image)
//  函数功能        : 划线初始化
//  参数说明        : image - 划线图像
//  备注
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void point_draw_init(uint8 image[MT9V03X_H][MT9V03X_W])
{
//    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
//    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image[0], MT9V03X_W, MT9V03X_H);
//    seekfree_assistant_camera_boundary_config(XY_BOUNDARY, BOUNDARY_NUM, xy_x_boundary, xy_x_2_boundary, xy_x_3_boundary, xy_y_boundary, xy_y_2_boundary, xy_y_3_boundary);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------
//  函数名称        : point_draw(uint8 count, uint8 (*point_coordinate)[2], void *image)
//  函数功能        : 传入多点的坐标数组进行划线并发给上位机
//  参数说明        : point_count        - 点的数量
//  参数说明        : (*point_coordinate)[2] - 点数组
//  备注           : 使用前一定要确定你已经进行了初始化
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void point_draw(uint8 point_count, uint8 (*point_coordinate)[2])
{
    point_cnt = 0;
    for (uint8 k = 0; k < point_count; k++)
    {
        int16 x = point_coordinate[k][0];
        int16 y = point_coordinate[k][1];

        // 画水平线 (y 固定)
        int16 x_start = (x - 6 < 0) ? 0 : x - 6;
        int16 x_end   = (x + 6 >= 188) ? 187 : x + 6;
        for (uint8 xi = (uint8)x_start; xi <= (uint8)x_end; xi++)
        {
            xy_x_boundary[point_cnt] = xi;
            xy_y_boundary[point_cnt] = (uint8)y;
            point_cnt++;
        }

        // 画垂直线 (x 固定)，跳过中心点避免重复
        int16 y_start = (y - 6 < 0) ? 0 : y - 6;
        int16 y_end   = (y + 6 >= 120) ? 119 : y + 6;
        for (uint8 yi = (uint8)y_start; yi <= (uint8)y_end; yi++)
        {
            if (yi == y) continue;
            xy_x_boundary[point_cnt] = (uint8)x;
            xy_y_boundary[point_cnt] = yi;
            point_cnt++;
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
//  函数名称        : rectangle_draw
//  函数功能        : 在图像上绘制轴对齐矩形框，并将框上所有像素点加入全局发送队列
//  参数说明        : x_min, y_min - 矩形左上角坐标
//  参数说明        : x_max, y_max - 矩形右下角坐标
//  备注           : 坐标自动裁剪至 [0, 187] × [0, 119] 范围内
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void rectangle_draw(uint16 x_min, uint16 y_min, uint16 x_max, uint16 y_max, uint8 flag)
{
    // 确保边界在图像范围内
    if (x_min > 187) x_min = 187;
    if (x_max > 187) x_max = 187;
    if (y_min > 119) y_min = 119;
    if (y_max > 119) y_max = 119;

    // 若矩形无效则返回
    if (x_min > x_max || y_min > y_max) return;
    if(flag)
    {
        // 绘制上边 (y = y_min)
        for (uint8 x = (uint8)x_min; x <= x_max; x++)
        {
            xy_x_2_boundary[point_cnt] = x;
            xy_y_2_boundary[point_cnt] = (uint8)y_min;
            point_cnt++;
        }

        // 绘制下边 (y = y_max)
        if (y_max != y_min)  // 若上下边重合则跳过
        {
            for (uint8 x = (uint8)x_min; x <= x_max; x++)
            {
                xy_x_2_boundary[point_cnt] = x;
                xy_y_2_boundary[point_cnt] = (uint8)y_max;
                point_cnt++;
            }
        }

        // 绘制左边 (x = x_min)，跳过与上下边重复的角点
        for (uint8 y = (uint8)y_min + 1; y < y_max; y++)
        {
            xy_x_2_boundary[point_cnt] = (uint8)x_min;
            xy_y_2_boundary[point_cnt] = y;
            point_cnt++;
        }

        // 绘制右边 (x = x_max)，跳过与上下边重复的角点
        if (x_max != x_min)  // 若左右边重合则跳过
        {
            for (uint8 y = (uint8)y_min + 1; y < y_max; y++)
            {
                xy_x_2_boundary[point_cnt] = (uint8)x_max;
                xy_y_2_boundary[point_cnt] = y;
                point_cnt++;
            }
        }
    }
    else
    {
        // 绘制上边 (y = y_min)
       for (uint8 x = (uint8)x_min; x <= x_max; x++)
       {
           xy_x_3_boundary[point_cnt] = x;
           xy_y_3_boundary[point_cnt] = (uint8)y_min;
           point_cnt++;
       }

       // 绘制下边 (y = y_max)
       if (y_max != y_min)  // 若上下边重合则跳过
       {
           for (uint8 x = (uint8)x_min; x <= x_max; x++)
           {
               xy_x_3_boundary[point_cnt] = x;
               xy_y_3_boundary[point_cnt] = (uint8)y_max;
               point_cnt++;
           }
       }

       // 绘制左边 (x = x_min)，跳过与上下边重复的角点
       for (uint8 y = (uint8)y_min + 1; y < y_max; y++)
       {
           xy_x_3_boundary[point_cnt] = (uint8)x_min;
           xy_y_3_boundary[point_cnt] = y;
           point_cnt++;
       }

       // 绘制右边 (x = x_max)，跳过与上下边重复的角点
       if (x_max != x_min)  // 若左右边重合则跳过
       {
           for (uint8 y = (uint8)y_min + 1; y < y_max; y++)
           {
               xy_x_3_boundary[point_cnt] = (uint8)x_max;
               xy_y_3_boundary[point_cnt] = y;
               point_cnt++;
           }
       }
    }
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        is_corner
//  函数功能        判定图像中某一像素是否为 FAST-9 角点
//  算法说明        以候选点 p 为中心，检查半径为 3 的 Bresenham 圆上的 16 个像素。
//                 若圆周上存在连续 9 个点亮度均大于 p+threshold（亮弧）或均小于 p-threshold（暗弧），
//                 则认为 p 是角点。本实现通过分别计数连续亮/暗像素并检查环形闭合条件完成判定。
//  参数说明        image   : 灰度图像数据指针（按行存储）
//                  stride  : 图像每行字节数（通常等于宽度）
//                  x, y    : 待判定像素坐标
//                  threshold: 亮度差阈值（通常 20~40）
//  返回值          1 表示该点为角点，0 表示非角点
//  注意事项        调用前需保证 (x,y) 距离图像边界至少 3 像素，否则圆周采样会越界。
//------------------------------------------------------------------------------------------------------------------
static int is_corner(unsigned char* image, int stride, int x, int y, int threshold) {
    unsigned char p = image[y*stride + x];
    int consecutive_b = 0, consecutive_d = 0;
    int first_b = 0, first_d = 0;
    int i;

    for(i = 0; i < 16; i++) {
        int nx = x + circle_x[i];
        int ny = y + circle_y[i];
        unsigned char q = image[ny*stride + nx];

        if(q > p + threshold) {
            consecutive_b++; consecutive_d = 0;
            if(consecutive_b >= 9) return 1;
        }
        else if(q < p - threshold) {
            consecutive_d++; consecutive_b = 0;
            if(consecutive_d >= 9) return 1;
        }
        else {
            consecutive_b = 0;
            consecutive_d = 0;
        }

        if(i == 0) { first_b = (q > p + threshold); first_d = (q < p - threshold); }
    }

    /* 处理环形闭合条件 */
    if(consecutive_b > 0 && first_b && (consecutive_b + first_b) >= 9) return 1;
    if(consecutive_d > 0 && first_d && (consecutive_d + first_d) >= 9) return 1;

    return 0;
}
//------------------------------------------------------------------------------------------------------------------
//  全局配置
//------------------------------------------------------------------------------------------------------------------
#define MAX_REGION_W      188
#define MAX_REGION_H      120
#define MAX_REGION_SIZE   (MAX_REGION_W * MAX_REGION_H)
#define MAX_CORNERS_NMS   200
#define FAST_BORDER       3
#define FAST_THRESHOLD    20
#define MAX_OUTPUT_CORNER 10

//------------------------------------------------------------------------------------------------------------------
//  函数名称        fast9_score
//  功能            计算角点得分（带安全边界保护，永不越界）
//------------------------------------------------------------------------------------------------------------------
int fast9_score(unsigned char* image, int stride,
                CornerPoint* corners, int num_corners, int threshold) {
    if (image == NULL || corners == NULL) return 0;

    for (int i = 0; i < num_corners; i++) {
        int x = corners[i].x;
        int y = corners[i].y;

        // 安全保护：坐标必须在图像内
        if (x < FAST_BORDER || x >= (stride - FAST_BORDER) ||
            y < FAST_BORDER || y >= ((stride ? stride : 1) - FAST_BORDER)) {
            corners[i].score = 0;
            continue;
        }

        unsigned char p = image[y * stride + x];
        int max_diff = 0;

        for (int j = 0; j < 16; j++) {
            int nx = x + circle_x[j];
            int ny = y + circle_y[j];

            // 【关键保护】圆周16点绝不访问图像外
            if (nx < 0 || nx >= stride || ny < 0 || ny >= (stride ? stride : 1)) {
                continue;
            }

            unsigned char q = image[ny * stride + nx];
            int diff = abs((int)q - (int)p);
            if (diff > max_diff) max_diff = diff;
        }
        corners[i].score = max_diff;
    }
    return num_corners;
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        suppress_non_maximum_static
//  功能            非极大值抑制
//------------------------------------------------------------------------------------------------------------------
static int suppress_non_maximum_static(CornerPoint* corners, int num_corners, int radius) {
    if (num_corners <= 1 || corners == NULL) return num_corners;
    if (num_corners > MAX_CORNERS_NMS) num_corners = MAX_CORNERS_NMS;

    static CornerPoint temp[MAX_CORNERS_NMS];
    static int keep[MAX_CORNERS_NMS];

    memcpy(temp, corners, num_corners * sizeof(CornerPoint));
    memset(keep, 0, num_corners * sizeof(int));

    for (int i = 0; i < num_corners; i++) {
        if (keep[i]) continue;
        for (int j = i + 1; j < num_corners; j++) {
            if (keep[j]) continue;

            int dx = abs(temp[i].x - temp[j].x);
            int dy = abs(temp[i].y - temp[j].y);

            if (dx <= radius && dy <= radius) {
                if (temp[i].score >= temp[j].score) keep[j] = 1;
                else { keep[i] = 1; break; }
            }
        }
    }

    int new_count = 0;
    for (int i = 0; i < num_corners; i++) {
        if (!keep[i]) {
            corners[new_count++] = temp[i];
        }
    }
    return new_count;
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        fast9_detect_static
//  功能            安全 FAST-9 检测（无越界）
//------------------------------------------------------------------------------------------------------------------
int fast9_detect_static(unsigned char* image, int width, int height,
                        int threshold, CornerPoint* corners, int max_corners) {
    if (image == NULL || corners == NULL || max_corners <= 0) return 0;
    if (width < 7 || height < 7) return 0;

    int count = 0;
    int stride = width;

    for (int y = FAST_BORDER; y < height - FAST_BORDER && count < max_corners; y++) {
        for (int x = FAST_BORDER; x < width - FAST_BORDER && count < max_corners; x++) {
            if (is_corner(image, stride, x, y, threshold)) {
                corners[count].x = x;
                corners[count].y = y;
                corners[count].score = 0;
                count++;
            }
        }
    }

    fast9_score(image, stride, corners, count, threshold);
    count = suppress_non_maximum_static(corners, count, 3);

    return count;
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        detect_corners_in_bbox_static
//  功能            【最终安全版】区域角点检测（TriCore 永不崩溃）
//------------------------------------------------------------------------------------------------------------------
void detect_corners_in_bbox_static(const uint8 *img,
                                   uint16 img_width,
                                   uint16 img_height,
                                   uint16 x_min, uint16 x_max,
                                   uint16 y_min, uint16 y_max,
                                   uint8 corners_out[][2], uint8 *corner_num)
{
    int region_w;
    int region_h;
    int y, x;
    uint16 img_x;
    uint16 img_y;
    static uint8 region[MAX_REGION_SIZE];
    CornerPoint fast_corners[50];
    int num;
    uint8 output_num;
    int i;
    int lx;
    int ly;

    *corner_num = 0;

    if (img == NULL || corners_out == NULL || corner_num == NULL)
    {
        return;
    }

    region_w = (int)x_max - (int)x_min + 1;
    region_h = (int)y_max - (int)y_min + 1;

    if (region_w < 7 || region_h < 7)
    {
        return;
    }

    if (region_w * region_h > MAX_REGION_SIZE)
    {
        return;
    }

    if (x_max >= img_width || y_max >= img_height)
    {
        return;
    }

    for (y = 0; y < region_h; y++)
    {
        for (x = 0; x < region_w; x++)
        {
            img_x = x_min + x;
            img_y = y_min + y;

            if (img_x >= img_width || img_y >= img_height)
            {
                region[y * region_w + x] = 0;
            }
            else
            {
                region[y * region_w + x] = img[img_y * img_width + img_x];
            }
        }
    }

    num = fast9_detect_static(region, region_w, region_h, FAST_THRESHOLD, fast_corners, 50);

    output_num = (num > MAX_OUTPUT_CORNER) ? MAX_OUTPUT_CORNER : num;
    *corner_num = output_num;

    for (i = 0; i < output_num; i++)
    {
        lx = fast_corners[i].x;
        ly = fast_corners[i].y;

        if (lx < 0 || lx >= region_w || ly < 0 || ly >= region_h)
        {
            corners_out[i][0] = 0;
            corners_out[i][1] = 0;
            continue;
        }

        corners_out[i][0] = x_min + lx;
        corners_out[i][1] = y_min + ly;
    }
}
//------------------------------------------------------------------------------------------------------------------
//  函数名称        prune_spurs
//  函数功能        删除骨架图像中的短小毛刺（孤立端点）
//  算法说明        迭代删除8邻域中仅有1个前景像素的端点（毛刺的末端）。
//                 每次迭代毛刺缩短1像素，通常迭代2~3次即可清除长度≤3的毛刺。
//  参数说明        img_data : 骨架图像数据（原地修改）
//                  width, height : 图像尺寸
//                  iterations : 剪枝迭代次数（建议2~3）
//  注意事项        必须在细化完成后调用，调用后会略微缩短骨架端点（可接受）。
//------------------------------------------------------------------------------------------------------------------
void prune_spurs(uint8 *img_data, int width, int height, int iterations)
{
    for (int iter = 0; iter < iterations; iter++)
    {
        uint8 to_delete[MT9V03X_H * MT9V03X_W] = {0};

        for (int y = 1; y < height - 1; y++)
        {
            for (int x = 1; x < width - 1; x++)
            {
                int idx = y * width + x;
                if (img_data[idx] == 0) continue;

                int neighbors = 0;
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        if (dx == 0 && dy == 0) continue;
                        if (img_data[(y + dy) * width + (x + dx)] != 0) neighbors++;
                    }
                }
                if (neighbors == 1)
                {
                    to_delete[idx] = 1;
                }
            }
        }

        for (int i = 0; i < width * height; i++)
        {
            if (to_delete[i]) img_data[i] = 0x00;
        }
    }
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        erode_binary
//  函数功能        对二值图像执行3x3腐蚀操作
//  参数说明        src : 输入二值图像（背景0x00，前景非0）
//                  dst : 输出腐蚀后二值图像（前景保持原值，背景为0x00）
//  备注            腐蚀会使前景区域向内收缩，消除细小锯齿并削尖圆角。
//                  若需补偿坐标偏移，请将检测到的角点坐标各加1。
//------------------------------------------------------------------------------------------------------------------
void erode_binary(uint8_t src[MT9V03X_H][MT9V03X_W], uint8_t dst[MT9V03X_H][MT9V03X_W])
{
    for (int y = 0; y < MT9V03X_H; y++)
    {
        for (int x = 0; x < MT9V03X_W; x++)
        {
            if (src[y][x] == 0x00)
            {
                dst[y][x] = 0x00;
                continue;
            }

            uint8_t has_background = 0;
            for (int dy = -1; dy <= 1 && !has_background; dy++)
            {
                for (int dx = -1; dx <= 1; dx++)
                {
                    int ny = y + dy;
                    int nx = x + dx;
                    if (ny < 0 || ny >= MT9V03X_H || nx < 0 || nx >= MT9V03X_W)
                    {
                        has_background = 1;
                        break;
                    }
                    if (src[ny][nx] == 0x00)
                    {
                        has_background = 1;
                        break;
                    }
                }
            }
            dst[y][x] = has_background ? 0x00 : src[y][x];
        }
    }
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        zhang_suen_thinning_static
//  函数功能        对二值图像进行Zhang-Suen骨架化处理，将前景区域细化为单像素宽度的中心线
//  算法说明        采用经典的Zhang-Suen并行细化算法，通过反复迭代删除满足特定条件的边界像素：
//                 1. 每次迭代分为两个子迭代，分别删除东南边界和西北边界的冗余像素。
//                 2. 删除条件严格遵循4条判定规则：邻域像素数限制、连通性保持、端点保护及单像素宽度约束。
//                 3. 算法在原图像数据上原地修改，迭代至没有像素可删除时自动终止。
//  参数说明        img_data    : 输入二值图像数据指针（背景0x00，前景为任意非0值，如0x01或0x81）
//                  width       : 图像宽度（像素）
//                  height      : 图像高度（像素）
//                  temp        : 外部临时缓冲区
//                  marker      : 外部标记缓冲区
//  返回值         无
//  注意事项        1. 输入图像背景必须严格为0x00，前景为任意非0值。
//                 2. 细化过程中删除的像素会被置为0x00。
//                 3. 该算法会保持原始拓扑连通性，适合后续提取端点/拐点特征。
//------------------------------------------------------------------------------------------------------------------
void zhang_suen_thinning_static(uint8 *img_data, int width, int height, uint8 *temp, uint8 *marker)
{
    bool pixels_removed;
    int  iter = 0;

    if (width < 3 || height < 3) return;

    do
    {
        pixels_removed = false;
        memcpy(temp, img_data, width * height);
        memset(marker, 0, width * height);

        // --- 子迭代 1 ----------------------------------------------------
        for (int y = 1; y < height - 1; y++)
        {
            for (int x = 1; x < width - 1; x++)
            {
                if (temp[y * width + x] == 0) continue;

                int p2 = (temp[(y - 1) * width + x] != 0);
                int p3 = (temp[(y - 1) * width + x + 1] != 0);
                int p4 = (temp[y * width + x + 1] != 0);
                int p5 = (temp[(y + 1) * width + x + 1] != 0);
                int p6 = (temp[(y + 1) * width + x] != 0);
                int p7 = (temp[(y + 1) * width + x - 1] != 0);
                int p8 = (temp[y * width + x - 1] != 0);
                int p9 = (temp[(y - 1) * width + x - 1] != 0);

                int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
                if (B < 2 || B > 6) continue;

                int A = 0;
                int neigh[9] = {p2, p3, p4, p5, p6, p7, p8, p9, p2};
                for (int k = 0; k < 8; k++)
                {
                    if (neigh[k] == 0 && neigh[k + 1] == 1) A++;
                }
                if (A != 1) continue;

                if ((p2 * p4 * p6) == 0 && (p4 * p6 * p8) == 0)
                {
                    marker[y * width + x] = 1;
                }
            }
        }

        for (int i = 0; i < width * height; i++)
        {
            if (marker[i])
            {
                img_data[i] = 0x00;
                pixels_removed = true;
            }
        }

        memcpy(temp, img_data, width * height);
        memset(marker, 0, width * height);

        // --- 子迭代 2 ----------------------------------------------------
        for (int y = 1; y < height - 1; y++)
        {
            for (int x = 1; x < width - 1; x++)
            {
                if (temp[y * width + x] == 0) continue;

                int p2 = (temp[(y - 1) * width + x] != 0);
                int p3 = (temp[(y - 1) * width + x + 1] != 0);
                int p4 = (temp[y * width + x + 1] != 0);
                int p5 = (temp[(y + 1) * width + x + 1] != 0);
                int p6 = (temp[(y + 1) * width + x] != 0);
                int p7 = (temp[(y + 1) * width + x - 1] != 0);
                int p8 = (temp[y * width + x - 1] != 0);
                int p9 = (temp[(y - 1) * width + x - 1] != 0);

                int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
                if (B < 2 || B > 6) continue;

                int A = 0;
                int neigh[9] = {p2, p3, p4, p5, p6, p7, p8, p9, p2};
                for (int k = 0; k < 8; k++)
                {
                    if (neigh[k] == 0 && neigh[k + 1] == 1) A++;
                }
                if (A != 1) continue;

                if ((p2 * p4 * p8) == 0 && (p2 * p6 * p8) == 0)
                {
                    marker[y * width + x] = 1;
                }
            }
        }

        for (int i = 0; i < width * height; i++)
        {
            if (marker[i])
            {
                img_data[i] = 0x00;
                pixels_removed = true;
            }
        }
        iter++;
    } while (pixels_removed);
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        trace_contour
//  函数功能        8邻域轮廓跟踪，提取单连通域的轮廓点坐标（逆时针方向）
//  参数说明        img        : 输入二值图像（背景0，前景非0）
//                  start_x, start_y : 轮廓起始点坐标（必须是前景边界点）
//                  contour_x, contour_y : 输出轮廓点的x/y坐标数组
//                  max_pts    : 轮廓点最大存储数量
//  返回值         实际提取到的轮廓点个数
//------------------------------------------------------------------------------------------------------------------
//static int trace_contour(uint8 img[MT9V03X_H][MT9V03X_W],
//                         uint8 start_x, uint8 start_y,
//                         uint8 *contour_x, uint8 *contour_y, int max_pts)
//{
//    const int dx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
//    const int dy8[8] = {0, -1, -1, -1, 0, 1, 1, 1};
//
//    uint8 x = start_x, y = start_y;
//    uint8 dir = 0;
//    int   count = 0;
//
//    do
//    {
//        contour_x[count] = x;
//        contour_y[count] = y;
//        count++;
//        if (count >= max_pts) break;
//
//        uint8 search_dir = (dir + 5) % 8;
//        uint8 found = 0;
//
//        for (int i = 0; i < 8; i++)
//        {
//            int nd = (search_dir + i) % 8;
//            int nx = (int)x + dx8[nd];
//            int ny = (int)y + dy8[nd];
//
//            if (nx >= 0 && nx < MT9V03X_W && ny >= 0 && ny < MT9V03X_H)
//            {
//                if (img[ny][nx] != 0)
//                {
//                    x = (uint8)nx;
//                    y = (uint8)ny;
//                    dir = (uint8)nd;
//                    found = 1;
//                    break;
//                }
//            }
//        }
//        if (!found) break;
//    } while (!(x == start_x && y == start_y) && count < max_pts);
//
//    return count;
//}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        detect_corners_from_contour
//  函数功能        根据轮廓链码差分计算曲率，检测角点（直角/尖锐转折点）
//  参数说明        cx, cy      : 轮廓点坐标数组
//                  n           : 轮廓点总数
//                  corners     : 输出角点坐标
//                  max_corners : 最大输出角点数
//                  curv_thresh : 曲率阈值（0=自动计算）
//  返回值         实际检测到的角点数量
//------------------------------------------------------------------------------------------------------------------
//static uint8 detect_corners_from_contour(uint8 *cx, uint8 *cy, int n,
//                                         uint8 (*corners)[2], uint8 max_corners,
//                                         int curv_thresh)
//{
//    if (n < 10) return 0;
//
//    static int chain[MAX_CONTOUR_PTS];
//    static int diff[MAX_CONTOUR_PTS];
//    static int curv[MAX_CONTOUR_PTS];
//
//    // 1. 计算链码
//    for (int i = 0; i < n - 1; i++)
//    {
//        int dx = cx[i + 1] - cx[i];
//        int dy = cy[i + 1] - cy[i];
//
//        if (dx == 1 && dy == 0)       chain[i] = 0;
//        else if (dx == 1 && dy == -1) chain[i] = 1;
//        else if (dx == 0 && dy == -1) chain[i] = 2;
//        else if (dx == -1 && dy == -1) chain[i] = 3;
//        else if (dx == -1 && dy == 0) chain[i] = 4;
//        else if (dx == -1 && dy == 1) chain[i] = 5;
//        else if (dx == 0 && dy == 1)  chain[i] = 6;
//        else if (dx == 1 && dy == 1)  chain[i] = 7;
//        else chain[i] = 0;
//    }
//    chain[n - 1] = chain[0];
//
//    // 2. 计算差分
//    for (int i = 0; i < n; i++)
//    {
//        int d = chain[(i + 1) % n] - chain[i];
//        if (d < -4) d += 8;
//        if (d > 4)  d -= 8;
//        diff[i] = d;
//    }
//
//    // 3. 计算曲率
//    int half_win = n / 20;
//    if (half_win < 3) half_win = 3;
//    if (half_win > 8) half_win = 8;
//
//    int max_curv = 0;
//    for (int i = 0; i < n; i++)
//    {
//        int sum = 0;
//        for (int k = -half_win; k <= half_win; k++)
//        {
//            int idx = (i + k + n) % n;
//            sum += abs(diff[idx]);
//        }
//        curv[i] = sum;
//        if (curv[i] > max_curv) max_curv = curv[i];
//    }
//
//    // 4. 自动阈值
//    if (curv_thresh <= 0)
//    {
//        curv_thresh = max_curv * 3 / 5;
//        if (curv_thresh < 3) curv_thresh = 3;
//    }
//
//    // 5. 搜索角点
//    int win = half_win + 1;
//    uint8 cnt = 0;
//    for (int i = 0; i < n && cnt < max_corners; i++)
//    {
//        if (curv[i] < curv_thresh) continue;
//
//        int is_max = 1;
//        for (int k = -win; k <= win; k++)
//        {
//            if (k == 0) continue;
//            int idx = (i + k + n) % n;
//            if (curv[idx] > curv[i])
//            {
//                is_max = 0;
//                break;
//            }
//        }
//        if (!is_max) continue;
//
//        // 去重
//        int too_close = 0;
//        for (int j = 0; j < cnt; j++)
//        {
//            int dx = cx[i] - corners[j][0];
//            int dy = cy[i] - corners[j][1];
//            if (abs(dx) <= 10 && abs(dy) <= 10)
//            {
//                too_close = 1;
//                break;
//            }
//        }
//        if (too_close) continue;
//
//        corners[cnt][0] = cx[i];
//        corners[cnt][1] = cy[i];
//        cnt++;
//    }
//
//    return cnt;
//}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        detect_corner_in_region
//  函数功能        在指定区域内检测直角角点
//------------------------------------------------------------------------------------------------------------------
//static uint8 detect_corner_in_region(uint8 dst[MT9V03X_H][MT9V03X_W],
//                                     uint16 x_min, uint16 x_max,
//                                     uint16 y_min, uint16 y_max,
//                                     uint8 (*corner)[2])
//{
//    uint8 corner_cnt = 0;
//
//    for (int y = y_min + 1; y < y_max - 1 && y < MT9V03X_H && corner_cnt < 4; y++)
//    {
//        for (int x = x_min + 1; x < x_max - 1 && x < MT9V03X_W && corner_cnt < 4; x++)
//        {
//            if (dst[y][x] != 0x01 && dst[y][x] != 0x81) continue;
//
//            uint8 n[8];
//            n[0] = (y > 0 && x > 0) ? dst[y - 1][x - 1] : 0;
//            n[1] = (y > 0) ? dst[y - 1][x] : 0;
//            n[2] = (y > 0 && x < MT9V03X_W - 1) ? dst[y - 1][x + 1] : 0;
//            n[3] = (x < MT9V03X_W - 1) ? dst[y][x + 1] : 0;
//            n[4] = (y < MT9V03X_H - 1 && x < MT9V03X_W - 1) ? dst[y + 1][x + 1] : 0;
//            n[5] = (y < MT9V03X_H - 1) ? dst[y + 1][x] : 0;
//            n[6] = (y < MT9V03X_H - 1 && x > 0) ? dst[y + 1][x - 1] : 0;
//            n[7] = (x > 0) ? dst[y][x - 1] : 0;
//
//            uint8 is_corner = 0;
//            if ((n[1] != 0) && (n[3] != 0) && (n[5] == 0) && (n[7] == 0))
//                is_corner = 1;
//            else if ((n[3] != 0) && (n[5] != 0) && (n[1] == 0) && (n[7] == 0))
//                is_corner = 1;
//            else if ((n[5] != 0) && (n[7] != 0) && (n[1] == 0) && (n[3] == 0))
//                is_corner = 1;
//            else if ((n[7] != 0) && (n[1] != 0) && (n[3] == 0) && (n[5] == 0))
//                is_corner = 1;
//
//            if (!is_corner) continue;
//
//            uint8 dup = 0;
//            for (uint8 k = 0; k < corner_cnt; k++)
//            {
//                if (abs((int16)x - corner[k][0]) <= CORNER_MERGE_DIST &&
//                    abs((int16)y - corner[k][1]) <= CORNER_MERGE_DIST)
//                {
//                    dup = 1;
//                    break;
//                }
//            }
//            if (dup) continue;
//
//            corner[corner_cnt][0] = (uint8)x;
//            corner[corner_cnt][1] = (uint8)y;
//            corner_cnt++;
//        }
//    }
//    return corner_cnt;
//}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        is_right_angle_beacon
//  函数功能        强化直角信标判断（角点位置+几何特征双重校验）
//------------------------------------------------------------------------------------------------------------------
//static uint8 is_right_angle_beacon(uint8 dst[image_h][image_w],
//                                   uint16 x_min, uint16 x_max,
//                                   uint16 y_min, uint16 y_max,
//                                   uint8 (*corner)[2])
//{
//    uint8 corner_cnt = detect_corner_in_region(dst, x_min, x_max, y_min, y_max, corner);
//    if (corner_cnt < 2) return 0;
//
//    uint16 w = x_max - x_min + 1;
//    uint16 h = y_max - y_min + 1;
//    float ratio = (w > h) ? (float)w / h : (float)h / w;
//    if (ratio > 1.5) return 0;
//
//    uint8 corner_valid = 0;
//    for (int m = 0; m < corner_cnt; m++)
//    {
//        for (int n = m + 1; n < corner_cnt; n++)
//        {
//            int dx = abs(corner[m][0] - corner[n][0]);
//            int dy = abs(corner[m][1] - corner[n][1]);
//            if ((dx >= w - 4 && dx <= w && dy >= h - 4 && dy <= h) ||
//                (dx <= 2 && dy >= h - 4 && dy <= h) ||
//                (dy <= 2 && dx >= w - 4 && dx <= w))
//            {
//                corner_valid = 1;
//                break;
//            }
//        }
//        if (corner_valid) break;
//    }
//    if (!corner_valid) return 0;
//
//    return corner_cnt;
//}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        Get_image
//  功能           获得一副灰度图像
//------------------------------------------------------------------------------------------------------------------
void Get_image(uint8 (*mt9v03x_image)[image_w])
{
#define use_num 1
    uint8 i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < image_h; i += use_num)
    {
        for (j = 0; j < image_w; j += use_num)
        {
            original_image[row][line] = mt9v03x_image[i][j];
            line++;
        }
        line = 0;
        row++;
    }
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        otsuThreshold
//  功能           动态阈值（大津法）
//------------------------------------------------------------------------------------------------------------------
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256
    uint16 Image_Width  = col;
    uint16 Image_Height = row;
    int    X;
    uint16 Y;
    uint8 *data = image;
    int   HistGram[GrayScale] = {0};

    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32  PixelIntegralFore = 0;
    int32  PixelFore = 0;
    double OmegaBack = 0, OmegaFore = 0, MicroBack = 0, MicroFore = 0, SigmaB = 0, Sigma = 0;
    uint8  MinValue = 0, MaxValue = 0;
    uint8  Threshold = 0;

    for (Y = 0; Y < Image_Height; Y++)
    {
        for (X = 0; X < Image_Width; X++)
        {
            HistGram[(int)data[Y * Image_Width + X]]++;
        }
    }

    for (MinValue = 0; MinValue < 255 && HistGram[MinValue] == 0; MinValue++);
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--);

    if (MaxValue == MinValue)       return MaxValue;
    if (MinValue + 1 == MaxValue)   return MinValue;

    for (Y = MinValue; Y <= MaxValue; Y++)
        Amount += HistGram[Y];

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
        PixelIntegral += HistGram[Y] * Y;

    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
        PixelBack = PixelBack + HistGram[Y];
        PixelFore = Amount - PixelBack;
        OmegaBack = (double)PixelBack / Amount;
        OmegaFore = (double)PixelFore / Amount;
        PixelIntegralBack += HistGram[Y] * Y;
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;
        MicroBack = (double)PixelIntegralBack / PixelBack;
        MicroFore = (double)PixelIntegralFore / PixelFore;
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
        if (Sigma > SigmaB)
        {
            SigmaB = Sigma;
            Threshold = (uint8)Y;
        }
    }
    return Threshold;
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        turn_to_dst
//  函数功能        进行亮点标记（0x01亮且未访问，0x81亮但已访问，0x00不亮）
//------------------------------------------------------------------------------------------------------------------
uint8 dst_image[image_h][image_w];

void turn_to_dst(uint8 (*img)[image_w], uint8 Threshold)
{
    uint8 i, j;
    uint8 image_threshold = Threshold;
    for (i = 0; i < image_h; i++)
    {
        for (j = 0; j < image_w; j++)
        {
            if (img[i][j] > image_threshold)
                dst_image[i][j] = 0x01;
            else
                dst_image[i][j] = 0x00;
        }
    }
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        IMAGE_CreatPoint
//  函数功能        建立预设点
//------------------------------------------------------------------------------------------------------------------
uint16 count_point = 0;
uint8  fond_xy[3000][2] = {{0, 0}};

void IMAGE_CreatPoint()
{
    uint8 i, j;

    for (i = 0; i < 5; i += 2)
        for (j = 0; j < 188; j += 2)
        { fond_xy[count_point][0] = j; fond_xy[count_point][1] = i; count_point++; }

    for (i = 1; i < 5; i += 2)
        for (j = 1; j < 188; j += 2)
        { fond_xy[count_point][0] = j; fond_xy[count_point][1] = i; count_point++; }

    for (i = 6; i < 15; i += 4)
        for (j = 1; j < 188; j += 2)
        { fond_xy[count_point][0] = j; fond_xy[count_point][1] = i; count_point++; }

    for (i = 8; i < 15; i += 4)
        for (j = 0; j < 188; j += 2)
        { fond_xy[count_point][0] = j; fond_xy[count_point][1] = i; count_point++; }

    for (i = 17; i < 120; i += 4)
        for (j = 0; j < 188; j += 4)
        { fond_xy[count_point][0] = j; fond_xy[count_point][1] = i; count_point++; }

    for (i = 19; i < 120; i += 4)
        for (j = 2; j < 188; j += 4)
        { fond_xy[count_point][0] = j; fond_xy[count_point][1] = i; count_point++; }
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        get_bg_mean_std
//  函数功能        在亮团周围取背景，计算背景均值与标准差
//------------------------------------------------------------------------------------------------------------------
static void get_bg_mean_std(uint8 img[image_h][image_w],
                            uint16 x_min, uint16 x_max,
                            uint16 y_min, uint16 y_max,
                            float *bg_mean, float *bg_std)
{
    int32  sum = 0, sum_sq = 0;
    uint32 cnt = 0;

    int sx = (x_min - 3) > 0 ? (x_min - 3) : 0;
    int ex = (x_max + 3) < image_w ? (x_max + 3) : image_w - 1;
    int sy = (y_min - 3) > 0 ? (y_min - 3) : 0;
    int ey = (y_max + 3) < image_h ? (y_max + 3) : image_h - 1;

    for (int y = sy; y <= ey; y++)
    {
        for (int x = sx; x <= ex; x++)
        {
            if (x >= x_min && x <= x_max && y >= y_min && y <= y_max)
                continue;

            uint8 val = img[y][x];
            sum += val;
            sum_sq += (uint16)val * val;
            cnt++;
        }
    }

    if (cnt < 1)
    {
        *bg_mean = 0;
        *bg_std = 1;
        return;
    }

    *bg_mean = (float)sum / cnt;
    float var = (sum_sq - (float)(sum * sum) / cnt) / (cnt - 1);
    if (var < 0) var = 0;
    *bg_std = sqrtf(var);
    if (*bg_std < 1) *bg_std = 1;
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        IMAGE_ChangeStructBr
//  函数功能        亮团检测与团点结构体更新（适配188×120图像）
//------------------------------------------------------------------------------------------------------------------
uint8 br_index = 0;
uint8 outline[15000][2] = {{0, 0}};

void IMAGE_ChangeStructBr(struct sBrightRegion br[10], uint8 dst[image_h][image_w], uint8 img[image_h][image_w])
{
    uint16 i, j;
    uint32 sum_x = 0, sum_y = 0, sum_pixel = 0;
    uint16 counter = 0;
    uint16 last_end = 0, out_start = 0, out_end = 0;
    uint8  n_x, n_y, l_x, r_x, u_y, d_y;
    uint8  x_max, y_max, x_min, y_min;
    uint8  center_x = 0, center_y = 0;
    br_index = 0;

    for (i = 0; i < 2924; i++)
    {
        if (fond_xy[i][1] < image_h && fond_xy[i][0] < image_w &&
            dst[fond_xy[i][1]][fond_xy[i][0]] == 0x01)
        {
            outline[0][0] = fond_xy[i][0];
            outline[0][1] = fond_xy[i][1];
            out_start = 0;
            last_end = 0;
            out_end = 1;
            counter = 1;
            sum_pixel = img[outline[0][1]][outline[0][0]];
            sum_x = outline[0][0];
            sum_y = outline[0][1];
            x_min = x_max = outline[0][0];
            y_min = y_max = outline[0][1];

            while (last_end != out_end)
            {
                last_end = out_end;
                for (j = out_start; j < out_end; j++)
                {
                    n_x = outline[j][0];
                    n_y = outline[j][1];
                    l_x = n_x - 1;
                    r_x = n_x + 1;
                    u_y = n_y - 1;
                    d_y = n_y + 1;

                    if (n_x > 0 && dst[n_y][l_x] == 0x01)
                    {
                        dst[n_y][l_x] = 0x81;
                        sum_x += l_x;
                        sum_y += n_y;
                        sum_pixel += img[n_y][l_x];
                        outline[counter][0] = l_x;
                        outline[counter][1] = n_y;
                        if (l_x < x_min) x_min = l_x;
                        if (l_x > x_max) x_max = l_x;
                        if (n_y < y_min) y_min = n_y;
                        if (n_y > y_max) y_max = n_y;
                        counter++;
                    }
                    if (n_x < image_w - 1 && dst[n_y][r_x] == 0x01)
                    {
                        dst[n_y][r_x] = 0x81;
                        sum_x += r_x;
                        sum_y += n_y;
                        sum_pixel += img[n_y][r_x];
                        outline[counter][0] = r_x;
                        outline[counter][1] = n_y;
                        if (r_x < x_min) x_min = r_x;
                        if (r_x > x_max) x_max = r_x;
                        if (n_y < y_min) y_min = n_y;
                        if (n_y > y_max) y_max = n_y;
                        counter++;
                    }
                    if (n_y > 0 && dst[u_y][n_x] == 0x01)
                    {
                        dst[u_y][n_x] = 0x81;
                        sum_x += n_x;
                        sum_y += u_y;
                        sum_pixel += img[u_y][n_x];
                        outline[counter][0] = n_x;
                        outline[counter][1] = u_y;
                        if (n_x < x_min) x_min = n_x;
                        if (n_x > x_max) x_max = n_x;
                        if (u_y < y_min) y_min = u_y;
                        if (u_y > y_max) y_max = u_y;
                        counter++;
                    }
                    if (n_y < image_h - 1 && dst[d_y][n_x] == 0x01)
                    {
                        dst[d_y][n_x] = 0x81;
                        sum_x += n_x;
                        sum_y += d_y;
                        sum_pixel += img[d_y][n_x];
                        outline[counter][0] = n_x;
                        outline[counter][1] = d_y;
                        if (n_x < x_min) x_min = n_x;
                        if (n_x > x_max) x_max = n_x;
                        if (d_y < y_min) y_min = d_y;
                        if (d_y > y_max) y_max = d_y;
                        counter++;
                    }
                }
                out_start = out_end;
                out_end = counter;
                if (out_end >= 15000) break;
            }

            if (counter >= 2 && counter <= 1000)
            {
                center_x = (uint8)(sum_x * 1.0 / counter + 0.5);
                center_y = (uint8)(sum_y * 1.0 / counter + 0.5);
                uint16 w = x_max - x_min + 1;
                uint16 h = y_max - y_min + 1;
                float bg_mean, bg_std;
                get_bg_mean_std(img, x_min, x_max, y_min, y_max, &bg_mean, &bg_std);
                float target_mean = (float)sum_pixel / counter;
                float snr = (target_mean - bg_mean) / bg_std;

                if (br_index >= 10)
                    br_index = 10;
                else
                {
                    br[br_index].sum_x = sum_x;
                    br[br_index].sum_y = sum_y;
                    br[br_index].sum_pixel = sum_pixel;
                    br[br_index].counter = counter;
                    br[br_index].center_x = center_x;
                    br[br_index].center_y = center_y;
                    br[br_index].snr = snr;
                    br[br_index].solid = (float)counter / (w * h);
                    br[br_index].x_min = x_min;
                    br[br_index].x_max = x_max;
                    br[br_index].y_min = y_min;
                    br[br_index].y_max = y_max;
                    if (w > h) br[br_index].ratio = (float)w / h;
                    else       br[br_index].ratio = (float)h / w;
                    br_index++;
                }
            }
        }
    }
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        accumulateForeground
//  函数功能        对图像进行差分并累加
//------------------------------------------------------------------------------------------------------------------
void accumulateForeground(uint8 *img, uint8 *preimg, uint8 *diffimg, uint8 Threshold)
{
    int16 i, j, k;
    static uint8 first = 1;
    uint8 temp;
    uint8 cn = 1;
    uint16 imgstep = MT9V03X_W * cn;

    for (j = 0; j < MT9V03X_H; j++, img += imgstep - cn, preimg += imgstep - cn, diffimg += imgstep - cn)
    {
        for (k = 0; k < cn; k++, img++, preimg++, diffimg++)
        {
            for (i = 0; i < imgstep; i += cn)
            {
                if (!first)
                {
                    temp = (uint8)abs(img[i] - preimg[i]);
                    temp = temp > Threshold ? 255 : 0;
                    diffimg[i] = temp;
                    preimg[i] = img[i];
                    img[i] = img[i] > Threshold ? 255 : 0;
                }
                else
                {
                    preimg[i] = img[i];
                }
            }
        }
    }
    first = 0;
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        MyOSTU
//  函数功能        OTSU（大津法）自适应阈值计算函数
//------------------------------------------------------------------------------------------------------------------
#define GrayScale 256
int pixel[GrayScale] = {0};

int MyOSTU(int width, int height, uint8 *Image)
{
    int threshold = 0;
    int32_t sum_gray = 0;
    int32_t sum_pix_num = 0;
    int32_t pl_pix_num = 0;
    int32_t p2_pix_num = 0;
    int32_t p1_sum_gray = 0;
    float m1 = 0;
    float m2 = 0;
    float V = 0;
    float variance = 0;
    int i, j, k = 0;

    for (i = 0; i < 256; i++)
        pixel[i] = 0;

    for (i = 0; i < height; i++)
        for (j = 0; j < width; j++)
            pixel[(int)Image[i * width + j]]++;

    for (k = 0; k < GrayScale; k++)
    {
        sum_gray += k * pixel[k];
        sum_pix_num += pixel[k];
    }

    for (k = 0; k < GrayScale - 1; k++)
    {
        pl_pix_num += pixel[k];
        p2_pix_num = sum_pix_num - pl_pix_num;

        if (pl_pix_num == 0 || p2_pix_num == 0) continue;

        p1_sum_gray += k * pixel[k];
        m1 = (float)p1_sum_gray / pl_pix_num;
        m2 = (float)(sum_gray - p1_sum_gray) / p2_pix_num;
        V = pl_pix_num * p2_pix_num * (m1 - m2) * (m1 - m2);

        if (V > variance)
        {
            variance = V;
            threshold = k;
        }
    }
    return threshold;
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        myabs
//  绝对值函数
//------------------------------------------------------------------------------------------------------------------
int16 myabs(int8 a)
{
    if (a >= 0) return a;
    else        return -a;
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        IMAGE_GetFlick
//  函数功能        寻找闪烁坐标
//------------------------------------------------------------------------------------------------------------------
uint8 br_mem[10][20][2];
uint8 br_grade[10][3];
uint8 empty_mem[10] = {0};
uint8 x_offset = 60, y_offset = 10;
uint8 y_light[60] = {
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
    10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
};
#define FLICK_FRAME 20
#define BR_GROUP    10
uint8 bcn_state[20] = {0};

void IMAGE_GetFlick(struct sBrightRegion br[10], uint8 *bcn_x, uint8 *bcn_y)
{
    int16 i, j, k;
    int16 theory_light = 0;
    int16 real_light = 0;
    static uint8 no_bcn_cnt = 0;

    for (i = 0; i < BR_GROUP; i++)
    {
        for (j = FLICK_FRAME - 1; j > 0; j--)
        {
            br_mem[i][j][0] = br_mem[i][j - 1][0];
            br_mem[i][j][1] = br_mem[i][j - 1][1];
        }
        br_mem[i][0][0] = 0;
        br_mem[i][0][1] = 0;
        if (empty_mem[i] == 1)
        {
            for (j = 0; j < 20; j++)
                if (br_mem[i][j][0] != 0 || br_mem[i][j][1] != 0) break;
            if (j == 20) empty_mem[i] = 0;
        }
    }

    if (br_index == 0)
    {
        no_bcn_cnt++;
        if (no_bcn_cnt > 10)
        {
            no_bcn_cnt = 10;
            *bcn_x = 0;
            *bcn_y = 0;
        }
    }
    else
    {
        no_bcn_cnt = 0;
        for (i = 0; i < br_index; i++)
        {
            for (j = 0; j < BR_GROUP; j++)
            {
                k = 0;
                if (empty_mem[j] == 0) continue;
                while (k < 20 && br_mem[j][k][0] == 0 && br_mem[j][k][1] == 0) k++;
                if (k == 20) empty_mem[j] = 0;
                else
                {
                    if (myabs((int16)br[i].center_x - (int16)br_mem[j][k][0]) < x_offset &&
                        myabs((int16)br[i].center_y - (int16)br_mem[j][k][1]) < y_offset)
                    {
                        br_mem[j][0][0] = br[i].center_x;
                        br_mem[j][0][1] = br[i].center_y;
                        break;
                    }
                }
            }
            if (j == BR_GROUP)
            {
                for (j = 0; j < BR_GROUP; j++)
                {
                    if (empty_mem[j] == 0)
                    {
                        br_mem[j][0][1] = br[i].center_y;
                        br_mem[j][0][0] = br[i].center_x;
                        empty_mem[j] = 1;
                        break;
                    }
                }
            }
        }

        for (i = 0; i < BR_GROUP; i++)
        {
            if (empty_mem[i] == 0)
            {
                br_grade[i][2] = 0;
                br_grade[i][0] = br_grade[i][1] = 0;
                bcn_state[i] = BCN_NONE;
            }
            else
            {
                for (j = 0; j < FLICK_FRAME; j++)
                {
                    if (br_mem[i][j][0] != 0 || br_mem[i][j][1] != 0)
                    {
                        if (j == 0) bcn_state[i] = BCN_LIGHT;
                        else        bcn_state[i] = BCN_DARK;
                        br_grade[i][0] = br_mem[i][j][0];
                        br_grade[i][1] = br_mem[i][j][1];
                        theory_light = y_light[br_mem[i][j][1]];
                        break;
                    }
                }
                real_light = 0;
                for (k = FLICK_FRAME - 1; k >= j; k--)
                    if (br_mem[i][k][0] != 0 || br_mem[i][k][1] != 0)
                        real_light++;

                if (myabs(theory_light - real_light) < 3)
                    br_grade[i][2] = 3;
                else if (myabs(theory_light - real_light) < 6)
                    br_grade[i][2] = 2;
                else if (myabs(theory_light - real_light) < 9)
                    br_grade[i][2] = 1;
                else
                    br_grade[i][2] = 0;
            }
        }

        for (i = 3; i >= 1; i--)
        {
            for (j = 0; j < BR_GROUP; j++)
            {
                if (br_grade[j][2] == i)
                {
                    *bcn_x = br_grade[j][0];
                    *bcn_y = br_grade[j][1];
                    return;
                }
            }
        }
        *bcn_x = 0;
        *bcn_y = 0;
    }
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        pack_three_uint8_to_uint32
//  功能            将三个uint8数据打包到一个uint32数据中
//------------------------------------------------------------------------------------------------------------------
uint32_t pack_three_uint8_to_uint32(uint8_t a, uint8_t b, uint8_t c)
{
    uint32_t pack_data = 0;
    pack_data |= ((uint32_t)a << 16) | ((uint32_t)b << 8) | (uint32_t)c;
    return pack_data;
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        image_process
//  函数功能        将图像得到的数据处理后发给CPU0
//------------------------------------------------------------------------------------------------------------------
uint32 image_result;
int    index_min_use;

void image_process(struct sBrightRegion br[10])
{
    uint8  index_min = 0;
    double distance_min = 88888888;
    uint8  max_grade = 0;

    if (br_index != 0)
    {
        feature_value(br);

        max_grade = 0;
        for (int i = 0; i < br_index; i++)
            if (br[i].grade_flag > max_grade)
                max_grade = br[i].grade_flag;

        if (max_grade == 0)
        {
            x_beacon = 2;
            y_beacon = 2;
        }
        else
        {
            distance_min = 99999.0;
            index_min = 0;
            for (int i = 0; i < br_index; i++)
            {
                if (br[i].grade_flag == max_grade)
                {
                    double cx = br[i].center_x;
                    double cy = br[i].center_y;
                    double current_dist = sqrt((cx - 94) * (cx - 94) + (cy - 60) * (cy - 60));
                    if (current_dist < distance_min)
                    {
                        distance_min = current_dist;
                        index_min = (uint8)i;
                    }
                }
            }
        }
        image_result = pack_three_uint8_to_uint32((uint8)br_index, br[index_min].center_x, br[index_min].center_y);
        index_min_use = index_min;
    }
    else
    {
        image_result = pack_three_uint8_to_uint32(0, 2, 2);
    }
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        feature_value
//  函数功能        判断团点特征值并融合SNR确定团点优先级
//------------------------------------------------------------------------------------------------------------------
int feature_value(struct sBrightRegion br[10])
{
    uint8 k = 0;
    int selected_car_index = -1;   // 最终保留的小车亮团索引

    // 第一轮：初步分类
    for (int i = 0; i < br_index; i++)
    {
      if (br[i].ratio >= 1.6 || br[i].solid <= 0.58 || br[i].snr < 6.0 || br[i].counter<10)
        {
            br[i].grade_flag = 0;
            if(br[i].counter>15)
            br[i].car_flag = 1;    // 标记为疑似车辆（L形）亮团
            k++;
        }
        else if (br[i].ratio >= 1.3)
        {
            br[i].car_flag = 0;
            if (br[i].snr >= 6 && br[i].snr <= 9)
                br[i].grade_flag = 1;
            else
                br[i].grade_flag = 2;
        }
        else
        {
            br[i].car_flag = 0;
            if (br[i].snr >= 6 && br[i].snr <= 9)
                br[i].grade_flag = 3;
            else
                br[i].grade_flag = 4;
        }
    }

    // 如果有多个 car_flag == 1 的亮团，只保留最亮的一个
    if (k > 1)
    {
        int brightest_idx = -1;
        uint32 max_sum_pixel = 0;   // 假设用 sum_pixel 表示总亮度

        // 找到总亮度最大的 car_flag == 1 亮团
        for (int i = 0; i < br_index; i++)
        {
            if (br[i].car_flag == 1)
            {
                if (br[i].sum_pixel > max_sum_pixel)
                {
                    max_sum_pixel = br[i].sum_pixel;
                    brightest_idx = i;
                }
            }
        }

        // 将其他的 car_flag 取消
        for (int i = 0; i < br_index; i++)
        {
            if (br[i].car_flag == 1 && i != brightest_idx)
            {
                br[i].car_flag = 0;
                br[i].grade_flag = 0;   // 降级为无效
            }
        }

        selected_car_index = brightest_idx;
    }
    else if (k == 1)
    {
        // 只有一个 car_flag == 1 的亮团，直接记录索引
        for (int i = 0; i < br_index; i++)
        {
            if (br[i].car_flag == 1)
            {
                selected_car_index = i;
                break;
            }
        }
    }
    else
    {
        selected_car_index = -1;   // 没有车辆亮团
    }

    return selected_car_index;
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        has_close_neighbor
//  函数功能         检查目标亮团是否有紧邻亮团（判断是否为线缆分裂产物）
//  参数说明         br          : 亮团数组
//  参数说明         target_idx  : 目标亮团下标
//  返回        1 = 有紧邻(疑似分裂), 0 = 无紧邻
//  备注          计算两个外接矩形之间的最小间隙：
//                 gap_x = 右侧blob左边界 - 左侧blob右边界
//                 gap_y = 下方blob上边界 - 上方blob下边界
//                 若 gap_x < BLOB_EDGE_GAP_MAX && gap_y < BLOB_EDGE_GAP_MAX
//                 说明两个亮团边缘紧贴，疑似同一光源被分割。
//                 注意：gap 为负表示两个矩形有重叠。
//------------------------------------------------------------------------------------------------------------------
uint8 has_close_neighbor(struct sBrightRegion br[10], uint8 target_idx)
{
    uint8 j;
    int16 gap_x, gap_y;
    int16 a_min, a_max, b_min, b_max;

    for (j = 0; j < br_index; j++)
    {
        if (j == target_idx) continue;

        // 计算两个外接矩形在 X 方向的最小间隙
        // 间隙 = 右侧blob左边界 - 左侧blob右边界
        // 间隙为负 => 矩形在 X 方向有重叠
        a_min = (int16)br[target_idx].x_min;
        a_max = (int16)br[target_idx].x_max;
        b_min = (int16)br[j].x_min;
        b_max = (int16)br[j].x_max;

        if (a_min < b_min) {
            gap_x = b_min - a_max;
        } else {
            gap_x = a_min - b_max;
        }

        // 计算两个外接矩形在 Y 方向的最小间隙
        a_min = (int16)br[target_idx].y_min;
        a_max = (int16)br[target_idx].y_max;
        b_min = (int16)br[j].y_min;
        b_max = (int16)br[j].y_max;

        if (a_min < b_min) {
            gap_y = b_min - a_max;
        } else {
            gap_y = a_min - b_max;
        }

        // 两个方向间隙都 < 阈值，亮团边缘紧贴，疑似分裂
        if (gap_x < BLOB_EDGE_GAP_MAX && gap_y < BLOB_EDGE_GAP_MAX)
        {
            return 1;
        }
    }
    return 0;
}

//------------------------------------------------------------------------------------------------------------------
//  函数名称        IMGE_CONTROL
//  函数功能        图像处理主控制函数
//------------------------------------------------------------------------------------------------------------------
uint8 image_copy[MT9V03X_H][MT9V03X_W];
uint8 point_corner[10][2];
uint8 corner_count;

void IMGE_CONTROL(struct sBrightRegion br[10])
{
    static uint8  last_beacon_valid = 0;
    //static uint8  last_car_valid = 0;
    static float    last_beacon_x = 94.0f;
    static float    last_beacon_y = 60.0f;
    static float    last_car_x = 94.0f;
    static float    last_car_y = 50.0f;
    corner_count = 0;
    point_cnt = 0;
//    memset(xy_x_boundary, 0, sizeof(xy_x_boundary));
//    memset(xy_y_boundary, 0, sizeof(xy_y_boundary));
//    memset(xy_x_2_boundary, 0, sizeof(xy_x_2_boundary));
//    memset(xy_y_2_boundary, 0, sizeof(xy_y_2_boundary));
//    memset(xy_x_3_boundary, 0, sizeof(xy_x_3_boundary));
//    memset(xy_y_3_boundary, 0, sizeof(xy_y_3_boundary));
//    memset(point_corner, 0, sizeof(point_corner));
    memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);

    threshold = (uint8)MyOSTU(MT9V03X_W, MT9V03X_H, image_copy[0]);
    threshold = threshold > 15 ? threshold : 15;
    turn_to_dst(image_copy, threshold);
    IMAGE_ChangeStructBr(br, dst_image, image_copy);

    if (br_index != 0)
    {
        car_index = feature_value(br);

        // ---- 小车部分（保持不变） ----
        if (car_index >= 0 && car_index < br_index)
        {
            x_car = br[car_index].center_x;
            y_car = br[car_index].center_y;
            car_image_flag = 1;
            car_lost_cnt=0;
            last_car_x = (float)x_car;
            last_car_y = (float)y_car;
            rectangle_draw(br[car_index].x_min, br[car_index].y_min, br[car_index].x_max, br[car_index].y_max, 0);
        } 
        else 
        {
            car_lost_cnt++;
              if(car_lost_cnt>BEACON_LOST_MAX)
              {
                x_car = 94;
                y_car = 50;
                car_lost_cnt=0;
                last_car_x = 94.0f;
                last_car_y = 50.0f;
                car_image_flag = 0;
              }
            else
            {
              x_car = (uint8)(last_car_x+0.5);
              y_car = (uint8)(last_car_y+0.5);
              car_image_flag = 1;
            }
        }

        // 确定最高等级（用于画图和备用选目标）
        int max_grade = 0;
        for (int i = 0; i < br_index; i++)
        {
            //rectangle_draw(br[i].x_min, br[i].y_min, br[i].x_max, br[i].y_max, br[i].car_flag);
            corner_count = 0;
            if (br[i].grade_flag > max_grade)
                max_grade = br[i].grade_flag;
        }

        // ---- 信标选择 ----
        int candidate_idx = -1;   // 本帧理想候选索引
        uint8 matched_to_history = 0;   // 标记：是否通过历史位置匹配到信标

        // 1. 优先尝试匹配历史信标（只要等级有效且距离近）
        if (last_beacon_valid)
        {
            for (int i = 0; i < br_index; i++)
            {
                if (br[i].grade_flag > 0)  // 等级有效即可
                {
                    float dx = br[i].center_x - last_beacon_x;
                    float dy = br[i].center_y - last_beacon_y;
                    if (sqrtf(dx * dx + dy * dy) < 20.0f)
                    {
                        candidate_idx = i;
                        matched_to_history = 1;   // 标记：已通过历史匹配找到
                        break;
                    }
                }
            }
        }

        // 2. 没匹配到历史信标，退回到选最高等级里离中心最近的
        if (candidate_idx == -1 && max_grade > 0)
        {
            float distance_min = 99999.0f;
            for (int i = 0; i < br_index; i++)
            {
                if (br[i].grade_flag == max_grade)
                {
                    float cx = br[i].center_x;
                    float cy = br[i].center_y;
                    float dist = sqrtf((cx - 94.0f) * (cx - 94.0f) + (cy - 60.0f) * (cy - 60.0f));
                    if (dist < distance_min)
                    {
                        distance_min = dist;
                        candidate_idx = i;
                    }
                }
            }
        }
            // 3. 过滤：线缆分裂假信标拒绝
        //    若信标候选未通过历史匹配（新出现），且小车可见，
        //    且候选紧贴小车，且有紧邻亮团（线缆分裂特征），
        //    则拒绝此候选，防止线缆遮挡导致的误识别。
        if (candidate_idx != -1 && !matched_to_history && car_image_flag == 1)
        {
            float dx_car = (float)br[candidate_idx].center_x - (float)x_car;
            float dy_car = (float)br[candidate_idx].center_y - (float)y_car;
            float dist_to_car = sqrtf(dx_car * dx_car + dy_car * dy_car);

            if (dist_to_car < (float)BEACON_CAR_MIN_DIST
                && has_close_neighbor(br, (uint8)candidate_idx))
            {
                // 拒绝：三个条件同时满足 =>
                // 1) 新出现的信标     (matched_to_history == 0)
                // 2) 紧贴小车         (dist_to_car < 20px)
                // 3) 有紧邻亮团       (has_close_neighbor == 1)
                // => 线缆分裂假信标，拒绝
                candidate_idx = -1;
            }
        }

        // 3. 应用丢失保持机制
        if (candidate_idx != -1)
        {
            // 成功找到信标
            rectangle_draw(br[candidate_idx].x_min, br[candidate_idx].y_min, br[candidate_idx].x_max, br[candidate_idx].y_max, 1);
            beacon_lost_cnt = 0;
            beacon_image_flag = 1;

            // 原始坐标（后续可滤波）
            x_beacon = br[candidate_idx].center_x;
            y_beacon = br[candidate_idx].center_y;

            // 更新记忆
            last_beacon_x = (float)x_beacon;
            last_beacon_y = (float)y_beacon;
            last_beacon_valid = 1;
        }
        else
        {
            // 本帧没找到信标
            beacon_lost_cnt++;
            if (beacon_lost_cnt > 8 || !last_beacon_valid)
            {
                // 真正丢失
                beacon_image_flag = 0;
                x_beacon = 2;
                y_beacon = 2;
                last_beacon_valid = 0;
                last_beacon_x = 94.0f;
                last_beacon_y = 60.0f;
            }
            else
            {
                // 短暂丢失，保持上一帧坐标
                x_beacon = (uint8)(last_beacon_x + 0.5);
                y_beacon = (uint8)(last_beacon_y + 0.5);
                beacon_image_flag = 1;
            }
        }

        // ---- 信标坐标低通滤波（建议添加） ----
        static float filter_beacon_x = 94.0f;
        static float filter_beacon_y = 60.0f;
        static uint8_t beacon_filter_inited = 0;

        if (beacon_image_flag)
        {
            if (!beacon_filter_inited)
            {
                filter_beacon_x = (float)x_beacon;
                filter_beacon_y = (float)y_beacon;
                beacon_filter_inited = 1;
            }
            else
            {
                filter_beacon_x = ALPHA * (float)x_beacon + (1.0f - ALPHA) * filter_beacon_x;
                filter_beacon_y = ALPHA * (float)y_beacon + (1.0f - ALPHA) * filter_beacon_y;
            }
            x_beacon = (uint8)(filter_beacon_x + 0.5f);
            y_beacon = (uint8)(filter_beacon_y + 0.5f);
        }
        else
        {
            beacon_filter_inited = 0;
        }
    }
    else
    {
        // br_index == 0 时的默认值
        x_beacon = (uint8)(last_beacon_x+0.5);
        y_beacon = (uint8)(last_beacon_y+0.5);
        x_car = (uint8)(last_car_x+0.5);
        y_car = (uint8)(last_car_y+0.5);
        beacon_image_flag = 0;
        car_image_flag = 0;
    }

    // 最终状态封装
    if (beacon_image_flag == 1 && car_image_flag == 1)
        image_flag = 1;
    else if (car_image_flag == 0)
        image_flag = 3;
    else
        image_flag = 2;

    // 小车坐标滤波（保持不变）
    if (car_image_flag)
    {
        if (!filter_inited)
        {
            x_car_f = (float)x_car;
            y_car_f = (float)y_car;
            filter_inited = 1;
        }
        else
        {
            x_car_f = ALPHA * (float)x_car + (1.0f - ALPHA) * x_car_f;
            y_car_f = ALPHA * (float)y_car + (1.0f - ALPHA) * y_car_f;
        }
    }
    else
    {
        filter_inited = 0;
    }

}
