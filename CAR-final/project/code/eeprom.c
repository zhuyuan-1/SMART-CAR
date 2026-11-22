/*********************************************************************************************************************
* CYT4BB7_CAR1 — EEPROM（Work Flash）参数存储实现
* 文件: eeprom.c
* 说明: 基于 Flash 的"读-修改-写"模式实现参数持久化
********************************************************************************************************************/

#include "eeprom.h"

//==================================================== 全局变量 ====================================================
uint8    para_update_flag;
uint8    para_update_display_flag;
uint8    speed_para_set = 0;          // 当前参数组编号
float    test = 0.0f;
//==================================================== 全局变量 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// Flash_init — 初始化 Flash 硬件
//-------------------------------------------------------------------------------------------------------------------
void Flash_init(void)
{
    flash_init();
    para_update_flag = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// EEPROM_Update_All_Para — 从 Flash 加载所有参数到全局变量
// 读取整个 FLASH_PAGE_INDEX 页面到缓冲区，然后提取各参数值
// 注意：大部分 PID 更新逻辑已注释，需要时请取消注释并连接到实际的 PID 结构体变量
//-------------------------------------------------------------------------------------------------------------------
void EEPROM_Update_All_Para(void)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, FLASH_PAGE_LENGTH);
    test = flash_union_buffer[speed_para_set * para_num + motor_l_p].float_type;
    // 电机L/R PID、舵机 PID、目标速度 PID 等加载逻辑已注释 ——
    // 需要时取消注释并连接到实际的 PID 结构体变量
}

//-------------------------------------------------------------------------------------------------------------------
// EEPROM_Read_Data — 从 Flash 读取单个 float 参数
//-------------------------------------------------------------------------------------------------------------------
float EEPROM_Read_Data(parameter para)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, FLASH_PAGE_LENGTH);
    return flash_union_buffer[para].float_type;
}

//-------------------------------------------------------------------------------------------------------------------
// EEPROM_Write_Data — 写入单个 float 参数到 Flash
// 采用"读-修改-写"策略：读取整页 → 修改目标位置 → 写回整页
//-------------------------------------------------------------------------------------------------------------------
void EEPROM_Write_Data(parameter para, float data)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, FLASH_PAGE_LENGTH);
    flash_union_buffer[para].float_type = data;
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, FLASH_PAGE_LENGTH);
}

//-------------------------------------------------------------------------------------------------------------------
// EEPROM_Write_Data_uint16 — 写入单个 uint16 参数到 Flash
//-------------------------------------------------------------------------------------------------------------------
void EEPROM_Write_Data_uint16(parameter para, uint16 data)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, FLASH_PAGE_LENGTH);
    flash_union_buffer[para].uint16_type = data;
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, FLASH_PAGE_LENGTH);
}

//-------------------------------------------------------------------------------------------------------------------
// EEPROM_Write_Data_uint32 — 写入单个 uint32 参数到 Flash
//-------------------------------------------------------------------------------------------------------------------
void EEPROM_Write_Data_uint32(parameter para, uint32 data)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, FLASH_PAGE_LENGTH);
    flash_union_buffer[para].uint32_type = data;
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, FLASH_PAGE_LENGTH);
}

//-------------------------------------------------------------------------------------------------------------------
// Para_Init — 参数初始化：写入出厂默认值到 Flash
// 警告：只在首次上电时调用一次，反复使用会损伤 Flash
//-------------------------------------------------------------------------------------------------------------------
void Para_Init(void)
{
    EEPROM_Write_Data(motor_l_p,     59.469f);
    EEPROM_Write_Data(motor_l_i,     2.804f);
    EEPROM_Write_Data(motor_r_p,     59.710f);
    EEPROM_Write_Data(motor_r_i,     2.808f);

    // PID/速度 默认值
    EEPROM_Save_PID(angle_p,  0.001f, 0.0f, 0.0f);
    EEPROM_Save_PID(speed_p,  0.001f, 0.0f, 0.0f);
    EEPROM_Save_PID(pos_p,    0.001f, 0.0f, 0.0f);
    EEPROM_Write_Data(base_speed,   15.0f);
    EEPROM_Write_Data(sprint_speed, 40.0f);
    EEPROM_Write_Data_uint32(ins_count, 0);
    EEPROM_Write_Data_uint32(pixel_charge_threshold, 25);
    EEPROM_Write_Data_uint32(beacon_count, 10);
    EEPROM_Write_Data_uint32(target_point_num, 40);

    // 初始化信标数据（Flash 页 9）
    {
        // 默认信标亮灯顺序（40 个）
        uint8 default_seq[40] = {
            1,4,3,5,1,2,3,4,2,3,5,1,6,3,5,3,1,2,3,4,
            1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4
        };
        beacon_seq_write_all(default_seq, 40);

        // 默认信标物理坐标
        float default_pos[11][2] = {
            {0.0f,   0.0f},   // 0: 原点
            {-1.0f,  2.0f},   // 1: 信标
            {-1.3f,  4.2f},   // 2: 信标
            {0.0f,   5.0f},   // 3: 信标
            {0.0f,   4.0f},   // 4: 信标
            {1.0f,   2.4f},   // 5: 信标
            {2.3f,   4.2f},   // 6: 信标
            {1.0f,   1.0f},   // 7: 信标
            {1.0f,   1.0f},   // 8: 信标
            {1.0f,   1.0f},   // 9: 信标
            {1.0f,   1.0f},   // 10:信标
        };
        beacon_pos_write_all(default_pos, 11);
    }

    // 初始化惯导坐标为 0
    {
        coord_point_t zero_buf[COORD_MAX_COUNT];
        for(uint32 i = 0; i < COORD_MAX_COUNT; i++) { zero_buf[i].x = 0.0f; zero_buf[i].y = 0.0f; }
        coord_write_all(zero_buf, COORD_MAX_COUNT);
    }

    para_update_flag         = 1;     // 通知参数已更新
    para_update_display_flag = 1;     // 通知需要刷新显示
}

//-------------------------------------------------------------------------------------------------------------------
// EEPROM_Save_PID — 批量保存一组 PID（P/I/D）到 Flash
// 比三次调用 EEPROM_Write_Data 更高效（只写入一次 Flash）
//-------------------------------------------------------------------------------------------------------------------
void EEPROM_Save_PID(parameter p_enum, float kp, float ki, float kd)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, FLASH_PAGE_LENGTH);
    flash_union_buffer[speed_para_set * para_num + p_enum].float_type = kp;
    flash_union_buffer[speed_para_set * para_num + p_enum + 1].float_type = ki;
    flash_union_buffer[speed_para_set * para_num + p_enum + 2].float_type = kd;
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, FLASH_PAGE_LENGTH);
}

//-------------------------------------------------------------------------------------------------------------------
// EEPROM_Load_PID — 批量从 Flash 读取一组 PID（P/I/D）
//-------------------------------------------------------------------------------------------------------------------
void EEPROM_Load_PID(parameter p_enum, float *kp, float *ki, float *kd)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, FLASH_PAGE_LENGTH);
    *kp = flash_union_buffer[speed_para_set * para_num + p_enum].float_type;
    *ki = flash_union_buffer[speed_para_set * para_num + p_enum + 1].float_type;
    *kd = flash_union_buffer[speed_para_set * para_num + p_enum + 2].float_type;
}

//==================================================== 信标数据批量存储 ====================================================
// 使用独立 Flash 页（BEACON_DATA_PAGE_INDEX = 9）
// 页面布局：[0..4] = 信标序列（5 个 uint32 字，每字打包 4 个 uint8）
//           [5..20] = 信标位置（16 个 uint32 字 = 8 个位置 × 2 个 float）
//==================================================== 信标数据批量存储 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// beacon_seq_read_all — 从 Flash 批量读取信标序列到 uint8 数组
// 序列以打包方式存储：每 4 个 uint8 值存入 1 个 uint32 字，共 5 个字对应 20 个值
//-------------------------------------------------------------------------------------------------------------------
void beacon_seq_read_all(uint8 *dst, uint32 count)
{
    zf_assert(count <= 40);
    zf_assert(dst != NULL);

    uint32 buf[10] = {0};   // 40 values = 10 words
    flash_read_page(FLASH_SECTION_INDEX, BEACON_DATA_PAGE_INDEX, buf, 10);
    for (uint32 i = 0; i < count; i++) {
        uint32 word = buf[i / 4];
        uint32 shift = (i % 4) * 8;
        dst[i] = (uint8)((word >> shift) & 0xFF);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// beacon_seq_write_all — 将信标序列 uint8 数组批量写入 Flash
// 保留位置数据：先读取完整的 21 字页面，修改序列部分，再写回
//-------------------------------------------------------------------------------------------------------------------
void beacon_seq_write_all(const uint8 *src, uint32 count)
{
    zf_assert(count <= 40);
    zf_assert(src != NULL);

    // 先读取整页（保留位置数据），修改序列部分，再写回
    uint32 buf[32] = {0};  // 10 个字序列 + 22 个字位置(11信标×2) = 32 个字
    flash_read_page(FLASH_SECTION_INDEX, BEACON_DATA_PAGE_INDEX, buf, 32);

    // 打包：每 4 个 uint8 → 1 个 uint32
    for (uint32 i = 0; i < 10; i++) buf[i] = 0;
    for (uint32 i = 0; i < count; i++) {
        buf[i / 4] |= ((uint32)src[i]) << ((i % 4) * 8);
    }

    flash_erase_page(FLASH_SECTION_INDEX, BEACON_DATA_PAGE_INDEX);
    flash_write_page(FLASH_SECTION_INDEX, BEACON_DATA_PAGE_INDEX, buf, 32);
}

//-------------------------------------------------------------------------------------------------------------------
// beacon_pos_read_all — 从 Flash 批量读取信标位置到 float[count][2] 数组
// 位置数据从页面第10个字开始（跳过10个序列字）
//-------------------------------------------------------------------------------------------------------------------
void beacon_pos_read_all(float (*dst)[2], uint32 count)
{
    zf_assert(count <= 11);
    zf_assert(dst != NULL);

    uint32 buf[32] = {0};  // 10 seq + 22 pos = 32 words
    flash_read_page(FLASH_SECTION_INDEX, BEACON_DATA_PAGE_INDEX, buf, 32);

    // 通过联合体转换 uint32 → float
    uint32 *src32 = &buf[10];   // 位置数据从第10个字开始
    float *dst_f = (float *)dst;
    for (uint32 i = 0; i < count * 2; i++) {
        union { uint32 u; float f; } conv;
        conv.u = src32[i];
        dst_f[i] = conv.f;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// beacon_pos_write_all — 将信标位置 float[count][2] 数组批量写入 Flash
// 保留序列数据：先读取完整页面，修改位置部分，再写回
//-------------------------------------------------------------------------------------------------------------------
void beacon_pos_write_all(const float (*src)[2], uint32 count)
{
    zf_assert(count <= 11);
    zf_assert(src != NULL);

    uint32 buf[32] = {0};  // 10 seq + 22 pos = 32 words
    flash_read_page(FLASH_SECTION_INDEX, BEACON_DATA_PAGE_INDEX, buf, 32);

    // 通过联合体转换 float → uint32
    uint32 *dst32 = &buf[10];   // 位置数据从第10个字开始
    const float *src_f = (const float *)src;
    for (uint32 i = 0; i < count * 2; i++) {
        union { uint32 u; float f; } conv;
        conv.f = src_f[i];
        dst32[i] = conv.u;
    }

    flash_erase_page(FLASH_SECTION_INDEX, BEACON_DATA_PAGE_INDEX);
    flash_write_page(FLASH_SECTION_INDEX, BEACON_DATA_PAGE_INDEX, buf, 32);
}

//-------------------------------------------------------------------------------------------------------------------
// beacon_data_erase_all — 擦除信标数据存储页
//-------------------------------------------------------------------------------------------------------------------
void beacon_data_erase_all(void)
{
    flash_erase_page(FLASH_SECTION_INDEX, BEACON_DATA_PAGE_INDEX);
}

//==================================================== 惯导坐标批量存储 ====================================================
// 使用独立 Flash 页（COORD_PAGE_INDEX = 10）
// 直接批量读写，不走 flash_union_buffer
// 容量说明：每页 2048 字节，每个坐标点 8 字节（2 个 float）
//==================================================== 惯导坐标批量存储 ====================================================

//-------------------------------------------------------------------------------------------------------------------
// coord_read_all — 从 Flash 批量读取坐标点到目标数组
//-------------------------------------------------------------------------------------------------------------------
void coord_read_all(coord_point_t *dst, uint32 count)
{
    zf_assert(count <= COORD_MAX_COUNT);
    zf_assert(dst != NULL);

    // 直接批量读取到用户数组
    flash_read_page(FLASH_SECTION_INDEX, COORD_PAGE_INDEX,
                    (uint32 *)dst, count * sizeof(coord_point_t) / sizeof(uint32));
}

//-------------------------------------------------------------------------------------------------------------------
// coord_write_all — 将坐标点数组批量写入 Flash（先擦除整页，再写入）
//-------------------------------------------------------------------------------------------------------------------
void coord_write_all(const coord_point_t *src, uint32 count)
{
    zf_assert(count <= COORD_MAX_COUNT);
    zf_assert(src != NULL);

    flash_erase_page(FLASH_SECTION_INDEX, COORD_PAGE_INDEX);
    flash_write_page(FLASH_SECTION_INDEX, COORD_PAGE_INDEX,
                     (const uint32 *)src, count * sizeof(coord_point_t) / sizeof(uint32));
}

//-------------------------------------------------------------------------------------------------------------------
// coord_erase_all — 擦除坐标存储页
//-------------------------------------------------------------------------------------------------------------------
void coord_erase_all(void)
{
    flash_erase_page(FLASH_SECTION_INDEX, COORD_PAGE_INDEX);
}
