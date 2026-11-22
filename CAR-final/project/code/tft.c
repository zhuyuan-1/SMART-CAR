/*********************************************************************************************************************
* CYT4BB7_CAR1 — TFT 屏幕菜单状态机实现
* 文件: tft.c
* 说明: 基于 tft_show 变量的多级菜单系统
*       KEY1=下/减  KEY2=上/加  KEY3=确认/选择
*
* 菜单结构:
*   case 0:  主菜单
*   case 1:  参数调试子菜单 (PID / 基础速度 / 冲刺速度 / 像素阈值)
*   case 2:  PID 调试子菜单 (角度环 / 速度环 / 位置环)
*   case 3:  PID 编辑器 (Kp/Ki/Kd, 步进, +/-, 保存, 返回)
*   case 4:  像素阈值编辑器 (新增)
*   case 5:  信标序列编辑器 (新增)
*   case 6:  基础速度编辑器
*   case 7:  冲刺速度编辑器
*   case 8:  惯导设置 (数量设置 / 坐标编辑 / 信标序列 / 信标坐标)
*   case 9:  坐标编辑器
*   case 10: 预留
*   case 11: 菜单设置 (占位)
*   case 12: 电量显示
*   case 13: 串口查看
*   case 14: 低电压警告
*   case 15: 预留
*   case 16: 信标位置编辑器 (新增)
********************************************************************************************************************/

#include "tft.h"

int tft_show = 0;               // 当前显示页面编号
float voltage_power_supply_tft = 0;  // 电源电压
float now_start = 0;            // 开始运行标志位
float a = 600;
float V_a = 0;
float stop_m2 = 0;
float acc = 1;                  // 调节步长
int y = 35;                     // 纵向光标位置
int x = 5;                      // 横向光标位置
float pid_get[3] = {0,0,0};
// 加减标志位: =1 加, =0 不动, =-1 减
int add = 0;

// ---- PID/速度编辑缓冲区 ----
float angle_pid[3] = {0, 0, 0};      // 角度环 Kp/Ki/Kd
float speed_pid[3] = {0, 0, 0};      // 速度环 Kp/Ki/Kd
float pos_pid[3] = {0, 0, 0};        // 位置环 Kp/Ki/Kd
float base_speed_val = 0;             // 基础速度值
float sprint_speed_val = 0;           // 冲刺速度值
uint8 focused_param = 0;              // 0=Kp, 1=Ki, 2=Kd
float *current_pid_group = angle_pid; // 指向当前 PID 组的指针
uint32 ins_coord_count = 0;               // 惯导坐标数量
coord_point_t coord_buffer[COORD_MAX_COUNT];  // 坐标编辑 RAM 缓冲区
uint32 coord_edit_index = 0;                    // 当前编辑的坐标索引（从0开始）
uint8  ins_edit_mode = 0;                       // 0=菜单模式, 1=设置数量模式
float serial_data[4] = {0, 0, 0, 0};           // 串口/UART 输入数据（预留）

// ---- 新增：像素冲刺阈值 + 信标数据 RAM 缓冲区 ----
uint32 pixel_threshold_val = 32;                                         // 像素冲刺阈值（屏幕可修改，默认 32）
uint8  beacon_seq_ram[40] = {
    1,2,3,7,5,4,3,2,7,3,1,1,6,2,1,7,5,4,2,1,
    1,2,3,7,5,4,3,2,7,3,1,1,6,2,1,7,5,4,2,1
}; // 信标序列 RAM 副本（40 个条目）
float  beacon_pos_ram[11][2];                                            // 信标位置 RAM 副本（11 个位置 × x,y）
uint32 beacon_seq_edit_index = 0;                                        // 信标序列当前编辑索引
uint32 beacon_pos_edit_index = 0;                                        // 信标位置当前编辑索引
uint8  beacon_seq_edit_mode = 0;                                         // 0=查看列表, 1=编辑值
uint8  g_beacon_count_tft = 10;                                           // 信标个数（屏幕可修改，最大10）
uint8  g_target_point_num_tft = 10;                                       // 序列长度（屏幕可修改）



//-------------------------------------------------------------------------------------------------------------------
// TFT_init — TFT 屏幕初始化，设置显示方向和颜色
//-------------------------------------------------------------------------------------------------------------------
void TFT_init(void)
{
    tft180_set_dir(TFT180_PORTAIT_180);
    tft180_set_color(RGB565_BLACK, RGB565_WHITE);
    tft180_init();
}

//-------------------------------------------------------------------------------------------------------------------
// KEY_init — 按键 GPIO 初始化，配置为输入上拉
// KEY1=P20_0, KEY2=P20_1, KEY3=P20_2
// 默认高电平（未按下），按下为低电平
//-------------------------------------------------------------------------------------------------------------------
void KEY_init(void)
{
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);  // 初始化 KEY1，输入，默认高电平，内部上拉
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);  // 初始化 KEY2，输入，默认高电平，内部上拉
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);  // 初始化 KEY3，输入，默认高电平，内部上拉
}


//==================================================== 按键消抖模块 ====================================================
// 使用 TCPWM0 硬件计数器作为时间基准，无需软件延时
// TCPWM0->GRP[2].CNT[6] 运行在 8MHz, 1ms = 8000 ticks
//==================================================== 按键消抖模块 ====================================================

static inline uint32 get_tick_raw(void)
{
    return Cy_Tcpwm_Counter_GetCounter(
        (volatile stc_TCPWM_GRP_CNT_t*)&TCPWM0->GRP[2].CNT[6]
    );
}

#define KEY_DEBOUNCE_MS      25
#define KEY_DEBOUNCE_TICKS   (KEY_DEBOUNCE_MS * 8000)
#define KEY_COUNT             3

static uint32 key_last_tick[KEY_COUNT]      = {0};
static uint8  key_last_raw[KEY_COUNT]       = {1, 1, 1};
static uint8  key_stable[KEY_COUNT]         = {1, 1, 1};
static uint8  key_prev_stable[KEY_COUNT]    = {1, 1, 1};

//-------------------------------------------------------------------------------------------------------------------
// KEY_Debounce — 按键消抖处理
// 检测按键原始电平变化，等待稳定周期后更新状态
// 每个主循环调用一次（在 TFT_KEYput 之前）
//-------------------------------------------------------------------------------------------------------------------
void KEY_Debounce(void)
{
    uint32 now = get_tick_raw();
    // KEY1
    {
        uint8 raw = gpio_get_level(KEY1);
        if (raw != key_last_raw[0])
            { key_last_raw[0] = raw; key_last_tick[0] = now; }
        else if ((now - key_last_tick[0]) >= KEY_DEBOUNCE_TICKS)
        {
            if (raw != key_stable[0])
                { key_prev_stable[0] = key_stable[0]; key_stable[0] = raw; }
        }
    }
    // KEY2
    {
        uint8 raw = gpio_get_level(KEY2);
        if (raw != key_last_raw[1])
            { key_last_raw[1] = raw; key_last_tick[1] = now; }
        else if ((now - key_last_tick[1]) >= KEY_DEBOUNCE_TICKS)
        {
            if (raw != key_stable[1])
                { key_prev_stable[1] = key_stable[1]; key_stable[1] = raw; }
        }
    }
    // KEY3
    {
        uint8 raw = gpio_get_level(KEY3);
        if (raw != key_last_raw[2])
            { key_last_raw[2] = raw; key_last_tick[2] = now; }
        else if ((now - key_last_tick[2]) >= KEY_DEBOUNCE_TICKS)
        {
            if (raw != key_stable[2])
                { key_prev_stable[2] = key_stable[2]; key_stable[2] = raw; }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// KEY_WasPressed — 检测按键下降沿（按下事件）
// 比较 key_prev_stable（上次稳定值）和 key_stable（当前稳定值）
// 检测到下降沿后立即清除（prev 同步为 stable），防止同一次按下被重复响应
//-------------------------------------------------------------------------------------------------------------------
uint8 KEY_WasPressed(uint8 idx)
{
    if (key_prev_stable[idx] == 1 && key_stable[idx] == 0)
    {
        key_prev_stable[idx] = key_stable[idx];
        return 1;
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// KEY_DebugPrint — 调试辅助：通过串口打印指定按键的内部状态
//-------------------------------------------------------------------------------------------------------------------
void KEY_DebugPrint(uint8 idx)
{
    uint8 raw;
    if      (idx == 0) raw = gpio_get_level(KEY1);
    else if (idx == 1) raw = gpio_get_level(KEY2);
    else                raw = gpio_get_level(KEY3);
    printf("K%d r=%d s=%d p=%d lr=%d t=%d\n",
           idx, raw, key_stable[idx], key_prev_stable[idx],
           key_last_raw[idx], get_tick_raw());
}

//-------------------------------------------------------------------------------------------------------------------
// Load_All_From_Flash — 从 Flash 一次性加载所有参数到 RAM 变量
// 包括: 角度环/速度环/位置环 PID、基础/冲刺速度、惯导坐标、像素阈值、信标数据
// 在每次 Flash 写入保存后调用，保持 RAM 同步
//-------------------------------------------------------------------------------------------------------------------
void Load_All_From_Flash(void)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, FLASH_PAGE_LENGTH);
    // PID
    angle_pid[0] = flash_union_buffer[speed_para_set*para_num+angle_p].float_type;
    angle_pid[1] = flash_union_buffer[speed_para_set*para_num+angle_p+1].float_type;
    angle_pid[2] = flash_union_buffer[speed_para_set*para_num+angle_p+2].float_type;
    speed_pid[0] = flash_union_buffer[speed_para_set*para_num+speed_p].float_type;
    speed_pid[1] = flash_union_buffer[speed_para_set*para_num+speed_p+1].float_type;
    speed_pid[2] = flash_union_buffer[speed_para_set*para_num+speed_p+2].float_type;
    pos_pid[0] = flash_union_buffer[speed_para_set*para_num+pos_p].float_type;
    pos_pid[1] = flash_union_buffer[speed_para_set*para_num+pos_p+1].float_type;
    pos_pid[2] = flash_union_buffer[speed_para_set*para_num+pos_p+2].float_type;
    // 速度
    base_speed_val   = flash_union_buffer[speed_para_set*para_num+base_speed].float_type;
    sprint_speed_val = flash_union_buffer[speed_para_set*para_num+sprint_speed].float_type;
    // 惯导坐标
    ins_coord_count = flash_union_buffer[speed_para_set*para_num+ins_count].uint32_type;
    if(ins_coord_count > COORD_MAX_COUNT) ins_coord_count = 0;
    coord_read_all(coord_buffer, ins_coord_count);

    // 像素冲刺阈值
    pixel_threshold_val = flash_union_buffer[speed_para_set*para_num+pixel_charge_threshold].uint32_type;
    if(pixel_threshold_val > 1000) pixel_threshold_val = 32;  // 合理性检查

    // 信标数据（从 Flash 页 9 读取）
    beacon_seq_read_all(beacon_seq_ram, 40);
    beacon_pos_read_all(beacon_pos_ram, 11);

    // 信标个数 + 序列长度
    g_beacon_count_tft = (uint8)flash_union_buffer[speed_para_set*para_num+beacon_count].uint32_type;
    if(g_beacon_count_tft > 10) g_beacon_count_tft = 10;
    g_target_point_num_tft = (uint8)flash_union_buffer[speed_para_set*para_num+target_point_num].uint32_type;
    if(g_target_point_num_tft > 40) g_target_point_num_tft = 10;
}

//-------------------------------------------------------------------------------------------------------------------
// ipc_push_params — 将当前修改的参数推送到共享内存，供 CM7_0 读取
// cmd_mask 位掩码: bit0=速度, bit1=像素阈值, bit2=信标序列, bit3=信标位置
// 在每次"保存"操作后调用
//-------------------------------------------------------------------------------------------------------------------
void ipc_push_params(uint8 cmd_mask)
{
    if (cmd_mask & 1) {  // 速度参数
        ipc_cm7_1_data[IPC_IDX_BEACON_TRACK] = base_speed_val;
        ipc_cm7_1_data[IPC_IDX_INERTIAL_NAV] = base_speed_val;
        ipc_cm7_1_data[IPC_IDX_CHARGE]       = sprint_speed_val;
    }
    if (cmd_mask & 2) {  // 像素阈值
        ipc_cm7_1_data[IPC_IDX_PIXEL_THRESH] = (float)pixel_threshold_val;
    }
    if (cmd_mask & 4) {  // 信标序列 + 数量
        ipc_cm7_1_data[IPC_IDX_BEACON_COUNT] = (float)g_beacon_count_tft;
        ipc_cm7_1_data[IPC_IDX_TARGET_POINT_NUM] = (float)g_target_point_num_tft;
        for (int i = 0; i < 40; i++)
            ipc_cm7_1_data[IPC_IDX_BEACON_SEQ + i] = (float)beacon_seq_ram[i];
    }
    if (cmd_mask & 8) {  // 信标位置 + 数量
        ipc_cm7_1_data[IPC_IDX_BEACON_COUNT] = (float)g_beacon_count_tft;
        for (int i = 0; i < 8; i++) {
            ipc_cm7_1_data[IPC_IDX_BEACON_POS + i*2]     = beacon_pos_ram[i][0];
            ipc_cm7_1_data[IPC_IDX_BEACON_POS + i*2 + 1] = beacon_pos_ram[i][1];
        }
    }
    ipc_cm7_1_data[IPC_IDX_CMD] = (float)cmd_mask;
    SCB_CleanInvalidateDCache_by_Addr(&ipc_cm7_1_data, sizeof(ipc_cm7_1_data));
}


//-------------------------------------------------------------------------------------------------------------------
// TFT_KEYput — 按键菜单状态机
// 根据按键输入和当前菜单层级驱动屏幕显示与导航
// 涵盖所有菜单功能：参数调试、PID 编辑、速度编辑、惯导设置、坐标编辑、
//                   像素阈值编辑、信标序列编辑、信标位置编辑、串口查看等
// 每个主循环调用一次，内部通过 tft_show 状态变量驱动 switch-case
// 低电压检测（case 14）具有最高优先级：当电压低于 9.7V 时强制切换
//-------------------------------------------------------------------------------------------------------------------
void TFT_KEYput(uint8 get_key1, uint8 get_key2, uint8 get_key3)
{
    // 赋值速度
    V_a = a;
    // 低电压检测
    if((voltage_power_supply_tft <= 9.7) && (voltage_power_supply_tft >= 4) )
    {
        tft_show = 14;
        tft180_clear();
    }

    // ---- 菜单状态机 ----
    tft180_show_chinese(88, 0, 16, chinese_wu[0], 1, RGB565_BLUE);
    tft180_show_chinese(104, 0, 16, chinese_er[0], 1, RGB565_BLUE);
    switch(tft_show)
    {
        // ====================================================================
        case 0 :// 主菜单
        // ====================================================================
        {
            tft180_show_chinese(5, 0, 16, chinese_caidan[0], 2, RGB565_BLACK);
            tft180_show_chinese(5, 18, 16, chinese_canshutiaoshi[0], 4, RGB565_BLACK);
            tft180_show_chinese(5, 36, 16, chinese_guandaoshezhi[0], 4, RGB565_BLACK);
            tft180_show_chinese(5, 54, 16, chinese_caidanshezhi[0], 4, RGB565_BLACK);
            tft180_show_chinese(5, 72, 16, chinese_dianliangxianshi[0], 4, RGB565_BLACK);
            tft180_show_chinese(5, 90, 16, chinese_chuankouchakan[0], 4, RGB565_BLACK);

            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y, RGB565_RED);

            if(get_key1)
            {
                y += 18; tft180_clear();
                if(y >= 108) y = 35;
            }
            if(get_key2)
            {
                y -= 18; tft180_clear();
                if(y <= 34) y = 107;
            }
            if(get_key3)
            {
                tft180_clear();
                switch(y)
                {
                    case 35:  tft_show = 1;  y = 143; break; // 参数调试
                    case 53:  tft_show = 8;  y = 53; break; // 惯导设置
                    case 71:  tft_show = 11; y = 143; break; // 菜单设置
                    case 89:  tft_show = 12; y = 143; break; // 电量显示
                    case 107: tft_show = 13; y = 143; break; // 串口查看
                }
            }
            break;
        }

        // ====================================================================
        case 1 :// 参数调试子菜单
        // ====================================================================
        {
            tft180_show_chinese(5, 0, 16, chinese_canshutiaoshi[0], 4, RGB565_BLACK);
            tft180_show_string(5, 18, "PID");  tft180_show_chinese(32, 18, 16, chinese_pidtiaoshi[6], 2, RGB565_BLACK);
            tft180_show_chinese(5, 36, 16, chinese_jichusudu[0], 4, RGB565_BLACK);
            tft180_show_chinese(5, 54, 16, chinese_chongcisudu[0], 4, RGB565_BLACK);
            tft180_show_chinese(5, 72, 16, chinese_xiangsu[0], 4, RGB565_BLACK);      // 像素阈值（新增）
            tft180_show_chinese(5, 90, 16, chinese_return[0], 5, RGB565_BLACK);

            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y, RGB565_RED);

            if(get_key1)
            {
                if(y == 71) y = 89;         // 冲刺 → 像素
                else if(y == 89) y = 107;   // 像素 → 返回
                else y += 18;
                tft180_clear();
                if(y >= 108) y = 35;
            }
            if(get_key2)
            {
                if(y == 107) y = 89;        // 返回 → 像素
                else if(y == 89) y = 71;    // 像素 → 冲刺
                else y -= 18;
                tft180_clear();
                if(y <= 34) y = 107;
            }
            if(get_key3)
            {
                switch(y)
                {
                    case 35:  tft_show = 2; tft180_clear(); y = 143; break; // PID 调试
                    case 53:  tft_show = 6; tft180_clear(); y = 143; break; // 基础速度
                    case 71:  tft_show = 7; tft180_clear(); y = 143; break; // 冲刺速度
                    case 89:  tft_show = 4; tft180_clear(); y = 143; break; // 像素阈值（新增）
                    case 107: tft_show = 0; tft180_clear(); y = 35; break;  // 返回
                }
            }
            break;
        }

        // ====================================================================
        case 2 :// PID 调试子菜单
        // ====================================================================
        {
            tft180_show_string(5, 0, "PID");  tft180_show_chinese(32, 0, 16, chinese_pidtiaoshi[6], 2, RGB565_BLACK);
            tft180_show_chinese(5, 18, 16, chinese_jiaoduhuan[0], 3, RGB565_BLACK);
            tft180_show_chinese(5, 36, 16, chinese_suduhuan[0], 3, RGB565_BLACK);
            tft180_show_chinese(5, 54, 16, chinese_weizhihuan[0], 3, RGB565_BLACK);
            tft180_show_chinese(5, 90, 16, chinese_return[0], 5, RGB565_BLACK);

            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y, RGB565_RED);

            if(get_key1)
            {
                if(y == 71) y = 107; // 跳过位置PID与返回之间的空隙
                else y += 18;
                tft180_clear();
                if(y >= 126) y = 35;
            }
            if(get_key2)
            {
                if(y == 107) y = 71; // 跳过空隙
                else y -= 18;
                tft180_clear();
                if(y <= 34) y = 107;
            }
            if(get_key3)
            {
                switch(y)
                {
                    case 35: // 角度环 PID
                        tft_show = 3; tft180_clear();
                        current_pid_group = angle_pid;
                        EEPROM_Load_PID(angle_p, &angle_pid[0], &angle_pid[1], &angle_pid[2]);
                        if(!(angle_pid[0]<1e8f&&angle_pid[0]>-1e8f)){angle_pid[0]=0;angle_pid[1]=0;angle_pid[2]=0;}
                        focused_param = 0; y = 35; x = 25; acc = 0.001; break;
                    case 53: // 速度环 PID
                        tft_show = 3; tft180_clear();
                        current_pid_group = speed_pid;
                        EEPROM_Load_PID(speed_p, &speed_pid[0], &speed_pid[1], &speed_pid[2]);
                        if(!(speed_pid[0]<1e8f&&speed_pid[0]>-1e8f)){speed_pid[0]=0;speed_pid[1]=0;speed_pid[2]=0;}
                        focused_param = 0; y = 35; x = 25; acc = 0.001; break;
                    case 71: // 位置环 PID
                        tft_show = 3; tft180_clear();
                        current_pid_group = pos_pid;
                        EEPROM_Load_PID(pos_p, &pos_pid[0], &pos_pid[1], &pos_pid[2]);
                        if(!(pos_pid[0]<1e8f&&pos_pid[0]>-1e8f)){pos_pid[0]=0;pos_pid[1]=0;pos_pid[2]=0;}
                        focused_param = 0; y = 35; x = 25; acc = 0.001; break;
                    case 107: // 返回
                        tft_show = 1; tft180_clear(); y = 143; break;
                }
            }
            break;
        }

        // ====================================================================
        case 3 :// PID 编辑器（角度环/速度环/位置环共用此 case）
        // ====================================================================
        {
            // 标题：根据当前 PID 组显示不同名称
            if      (current_pid_group == angle_pid) tft180_show_chinese(5, 0, 16, chinese_jiaoduhuan[0], 3, RGB565_BLACK);
            else if (current_pid_group == speed_pid) tft180_show_chinese(5, 0, 16, chinese_suduhuan[0], 3, RGB565_BLACK);
            else                                     tft180_show_chinese(5, 0, 16, chinese_weizhihuan[0], 3, RGB565_BLACK);

            // Kp/Ki/Kd 值显示
            tft180_show_string(5, 18, "Kp:"); tft180_show_float(60, 18, current_pid_group[0], 2, 3);
            tft180_show_string(5, 36, "Ki:"); tft180_show_float(60, 36, current_pid_group[1], 2, 3);
            tft180_show_string(5, 54, "Kd:"); tft180_show_float(60, 54, current_pid_group[2], 2, 3);

            // 步进大小
            tft180_show_chinese(5, 72, 16, chinese_changedobble[0], 4, RGB565_BLACK);
            tft180_set_font(TFT180_6X8_FONT);
            tft180_show_char(70, 76, '*');
            tft180_show_float(78, 76, acc, 1, 3);
            tft180_set_font(TFT180_8X16_FONT);

            // +/- 选择器
            tft180_show_char(25, 90, '+');
            tft180_show_char(85, 90, '-');
            if(y == 107)
            {
                if(x == 85)      { tft180_show_char(16, 90, '('); tft180_show_char(34, 90, ')'); }
                else if(x == 0)  { tft180_show_char(76, 90, '('); tft180_show_char(94, 90, ')'); }
            }

            // 保存 / 返回
            tft180_show_chinese(5, 108, 16, chinese_baocun[0], 2, RGB565_BLACK);
            tft180_show_chinese(5, 126, 16, chinese_returnlast[0], 6, RGB565_BLACK);

            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y, RGB565_RED);

            // ---- KEY1（下移） ----
            if(get_key1)
            {
                if(y == 107) // +/- 行
                {
                    if(x == 25)      { tft180_clear(); add = 1; x = 85; }
                    else if(x == 85) { tft180_clear(); x = 0; add = -1; }
                    else             { tft180_clear(); y += 18; x = 25; add = 0; }
                }
                else if(y == 89) // 步进行 → 跳到保存行
                {
                    y += 18; tft180_clear();
                }
                else // Kp/Ki/Kd 或保存/返回行
                {
                    y += 18; tft180_clear();
                }
                if(y >= 144) y = 35;
            }

            // ---- KEY2（上移） ----
            if(get_key2)
            {
                if(y == 107)
                {
                    if(x == 25)      { tft180_clear(); add = 1; x = 85; }
                    else if(x == 85) { tft180_clear(); x = 0; add = -1; }
                    else             { tft180_clear(); y -= 18; x = 25; add = 0; }
                }
                else if(y == 35) // Kp 行 → 回绕到底部
                {
                    y = 143; tft180_clear();
                }
                else
                {
                    y -= 18; tft180_clear();
                }
                if(y <= 34) y = 143;
            }

            // ---- KEY3（确认） ----
            if(get_key3)
            {
                switch(y)
                {
                    case 35: // Kp → 设置 focused_param，跳转到 +/- 行
                        focused_param = 0; y = 107; tft180_clear(); break;
                    case 53: // Ki
                        focused_param = 1; y = 107; tft180_clear(); break;
                    case 71: // Kd
                        focused_param = 2; y = 107; tft180_clear(); break;
                    case 89: // 步进循环：0.001 → 0.01 → 0.1 → 1.0 → 0.001
                        acc *= 10;
                        if(acc > 1.0f) acc = 0.001f;
                        tft180_clear(); break;
                    case 107: // 应用 +/-
                        current_pid_group[focused_param] += (float)add * acc;
                        tft180_clear(); break;
                    case 125: // 保存到 Flash
                    {
                        if      (current_pid_group == angle_pid)
                            {EEPROM_Save_PID(angle_p, current_pid_group[0], current_pid_group[1], current_pid_group[2]); Load_All_From_Flash();}
                        else if (current_pid_group == speed_pid)
                            {EEPROM_Save_PID(speed_p, current_pid_group[0], current_pid_group[1], current_pid_group[2]); Load_All_From_Flash();}
                        else
                            {EEPROM_Save_PID(pos_p, current_pid_group[0], current_pid_group[1], current_pid_group[2]); Load_All_From_Flash();}
                        tft_show = 2; tft180_clear(); y = 35; break;
                    }
                    case 143: // 返回 PID 菜单（不保存）
                        tft_show = 2; tft180_clear(); y = 35; break;
                    default: break;
                }
            }
            break;
        }

        // ====================================================================
        case 4 :// 像素冲刺阈值编辑器（新增）
        // ====================================================================
        {
            tft180_show_chinese(5, 0, 16, chinese_xiangsu[0], 4, RGB565_BLACK);
            tft180_show_uint(10, 36, pixel_threshold_val, 4);

            tft180_show_chinese(5, 72, 16, chinese_changedobble[0], 4, RGB565_BLACK);
            tft180_set_font(TFT180_6X8_FONT);
            tft180_show_char(70, 76, '*'); tft180_show_float(78, 76, acc, 1, 3);
            tft180_set_font(TFT180_8X16_FONT);

            tft180_show_char(25, 90, '+'); tft180_show_char(85, 90, '-');
            if(y == 107)
            {
                if(x == 85)      { tft180_show_char(16, 90, '('); tft180_show_char(34, 90, ')'); }
                else if(x == 0)  { tft180_show_char(76, 90, '('); tft180_show_char(94, 90, ')'); }
            }

            tft180_show_chinese(5, 108, 16, chinese_baocun[0], 2, RGB565_BLACK);
            tft180_show_chinese(5, 126, 16, chinese_returnlast[0], 6, RGB565_BLACK);

            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y, RGB565_RED);

            if(get_key1)
            {
                if(y == 107)
                {
                    if(x == 25)      { tft180_clear(); add = 1; x = 85; }
                    else if(x == 85) { tft180_clear(); x = 0; add = -1; }
                    else             { tft180_clear(); y += 18; x = 25; add = 0; }
                }
                else { y += 18; tft180_clear(); }
                if(y >= 144) y = 89;
            }
            if(get_key2)
            {
                if(y == 107)
                {
                    if(x == 25)      { tft180_clear(); add = 1; x = 85; }
                    else if(x == 85) { tft180_clear(); x = 0; add = -1; }
                    else             { tft180_clear(); y -= 18; x = 25; add = 0; }
                }
                else { y -= 18; tft180_clear(); }
                if(y <= 88) y = 143;
            }
            if(get_key3)
            {
                switch(y)
                {
                    case 89:  // 步进循环：1 ↔ 10
                        acc *= 10; if(acc > 10) acc = 1; tft180_clear(); break;
                    case 107: // 应用 +/-
                        { int v = (int)pixel_threshold_val + add * (int)acc;
                          if(v < 0) v = 0; if(v > 500) v = 500;
                          pixel_threshold_val = (uint32)v; tft180_clear(); } break;
                    case 125: // 保存 → EEPROM + IPC 推送
                        EEPROM_Write_Data_uint32(pixel_charge_threshold, pixel_threshold_val);
                        Load_All_From_Flash();
                        ipc_push_params(2);
                        tft_show = 1; tft180_clear(); y = 143; break;
                    case 143: // 返回
                        tft_show = 1; tft180_clear(); y = 143; break;
                }
            }
            break;
        }

        // ====================================================================
        case 6 :// 基础速度编辑器
        // ====================================================================
        {
            tft180_show_chinese(5, 0, 16, chinese_jichusudu[0], 4, RGB565_BLACK);
            tft180_show_float(10, 36, base_speed_val, 4, 2);

            tft180_show_chinese(5, 72, 16, chinese_changedobble[0], 4, RGB565_BLACK);
            tft180_set_font(TFT180_6X8_FONT);
            tft180_show_char(70, 76, '*'); tft180_show_float(78, 76, acc, 1, 3);
            tft180_set_font(TFT180_8X16_FONT);

            tft180_show_char(25, 90, '+'); tft180_show_char(85, 90, '-');
            if(y == 107)
            {
                if(x == 85)      { tft180_show_char(16, 90, '('); tft180_show_char(34, 90, ')'); }
                else if(x == 0)  { tft180_show_char(76, 90, '('); tft180_show_char(94, 90, ')'); }
            }

            tft180_show_chinese(5, 108, 16, chinese_baocun[0], 2, RGB565_BLACK);
            tft180_show_chinese(5, 126, 16, chinese_returnlast[0], 6, RGB565_BLACK);

            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y, RGB565_RED);

            if(get_key1)
            {
                if(y == 107)
                {
                    if(x == 25)      { tft180_clear(); add = 1; x = 85; }
                    else if(x == 85) { tft180_clear(); x = 0; add = -1; }
                    else             { tft180_clear(); y += 18; x = 25; add = 0; }
                }
                else { y += 18; tft180_clear(); }
                if(y >= 144) y = 89;
            }
            if(get_key2)
            {
                if(y == 107)
                {
                    if(x == 25)      { tft180_clear(); add = 1; x = 85; }
                    else if(x == 85) { tft180_clear(); x = 0; add = -1; }
                    else             { tft180_clear(); y -= 18; x = 25; add = 0; }
                }
                else { y -= 18; tft180_clear(); }
                if(y <= 88) y = 143;
            }
            if(get_key3)
            {
                switch(y)
                {
                    case 89:  acc *= 10; if(acc > 100) acc = 1; tft180_clear(); break;
                    case 107: base_speed_val += (float)add * acc; tft180_clear(); break;
                    case 125: // 保存 → EEPROM + IPC 推送
                        EEPROM_Write_Data(base_speed, base_speed_val);
                        Load_All_From_Flash();
                        ipc_push_params(1);
                        tft_show = 1; tft180_clear(); y = 143; break;
                    case 143: tft_show = 1; tft180_clear(); y = 143; break;
                }
            }
            break;
        }

        // ====================================================================
        case 7 :// 冲刺速度编辑器
        // ====================================================================
        {
            tft180_show_chinese(5, 0, 16, chinese_chongcisudu[0], 4, RGB565_BLACK);
            tft180_show_float(10, 36, sprint_speed_val, 4, 2);

            tft180_show_chinese(5, 72, 16, chinese_changedobble[0], 4, RGB565_BLACK);
            tft180_set_font(TFT180_6X8_FONT);
            tft180_show_char(70, 76, '*'); tft180_show_float(78, 76, acc, 1, 3);
            tft180_set_font(TFT180_8X16_FONT);

            tft180_show_char(25, 90, '+'); tft180_show_char(85, 90, '-');
            if(y == 107)
            {
                if(x == 85)      { tft180_show_char(16, 90, '('); tft180_show_char(34, 90, ')'); }
                else if(x == 0)  { tft180_show_char(76, 90, '('); tft180_show_char(94, 90, ')'); }
            }

            tft180_show_chinese(5, 108, 16, chinese_baocun[0], 2, RGB565_BLACK);
            tft180_show_chinese(5, 126, 16, chinese_returnlast[0], 6, RGB565_BLACK);

            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y, RGB565_RED);

            if(get_key1)
            {
                if(y == 107)
                {
                    if(x == 25)      { tft180_clear(); add = 1; x = 85; }
                    else if(x == 85) { tft180_clear(); x = 0; add = -1; }
                    else             { tft180_clear(); y += 18; x = 25; add = 0; }
                }
                else { y += 18; tft180_clear(); }
                if(y >= 144) y = 89;
            }
            if(get_key2)
            {
                if(y == 107)
                {
                    if(x == 25)      { tft180_clear(); add = 1; x = 85; }
                    else if(x == 85) { tft180_clear(); x = 0; add = -1; }
                    else             { tft180_clear(); y -= 18; x = 25; add = 0; }
                }
                else { y -= 18; tft180_clear(); }
                if(y <= 88) y = 143;
            }
            if(get_key3)
            {
                switch(y)
                {
                    case 89:  acc *= 10; if(acc > 100) acc = 1; tft180_clear(); break;
                    case 107: sprint_speed_val += (float)add * acc; tft180_clear(); break;
                    case 125: // 保存 → EEPROM + IPC 推送
                        EEPROM_Write_Data(sprint_speed, sprint_speed_val);
                        Load_All_From_Flash();
                        ipc_push_params(1);
                        tft_show = 1; tft180_clear(); y = 143; break;
                    case 143: tft_show = 1; tft180_clear(); y = 143; break;
                }
            }
            break;
        }

        // ====================================================================
        case 8 :// 惯导设置
        // ====================================================================
        {
            if(ins_edit_mode == 1)
            {
                // ---- 设置数量模式（信标个数 + 序列长度） ----
                tft180_show_chinese(5, 0, 16, chinese_zuobiaoshezhi[4], 2, RGB565_BLACK);
                tft180_show_chinese(37, 0, 16, chinese_shuliang[0], 2, RGB565_BLACK);
                // 信标个数
                tft180_show_string(5, 18, "Beacon:");  tft180_show_uint(70, 18, g_beacon_count_tft, 2);
                tft180_show_string(90, 18, "/10");
                // 序列长度
                tft180_show_string(5, 36, "SeqLen:");  tft180_show_uint(75, 36, g_target_point_num_tft, 2);
                tft180_show_string(95, 36, "/40");
                // 步进
                tft180_show_chinese(5, 72, 16, chinese_changedobble[0], 4, RGB565_BLACK);
                tft180_set_font(TFT180_6X8_FONT);
                tft180_show_char(70, 76, '*'); tft180_show_float(78, 76, acc, 1, 3);
                tft180_set_font(TFT180_8X16_FONT);
                tft180_show_char(25, 90, '+'); tft180_show_char(85, 90, '-');
                if(y == 107)
                {
                    if(x == 85)      { tft180_show_char(16, 90, '('); tft180_show_char(34, 90, ')'); }
                    else if(x == 0)  { tft180_show_char(76, 90, '('); tft180_show_char(94, 90, ')'); }
                }
                tft180_show_chinese(5, 108, 16, chinese_baocun[0], 2, RGB565_BLACK);
                tft180_show_chinese(5, 126, 16, chinese_returnlast[0], 6, RGB565_BLACK);
                tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y, RGB565_RED);
                if(get_key1)
                {
                    if(y == 107) { if(x==25){tft180_clear();add=1;x=85;}else if(x==85){tft180_clear();x=0;add=-1;}else{tft180_clear();y+=18;x=25;add=0;} }
                    else { y += 18; tft180_clear(); if(y >= 144) y = 35; }
                }
                if(get_key2)
                {
                    if(y == 107) { if(x==25){tft180_clear();add=1;x=85;}else if(x==85){tft180_clear();x=0;add=-1;}else{tft180_clear();y-=18;x=25;add=0;} }
                    else { y -= 18; tft180_clear(); if(y <= 34) y = 143; }
                }
                if(get_key3)
                {
                    switch(y)
                    {
                        case 35: // 选中信标个数
                            focused_param = 0; y = 107; x = 25; tft180_clear(); break;
                        case 53: // 选中序列长度
                            focused_param = 1; y = 107; x = 25; tft180_clear(); break;
                        case 89: acc = (acc >= 10) ? 1 : 10; tft180_clear(); break;  // 步进 1↔10
                        case 107: // 应用 +/-
                            if(add != 0) {
                                if(focused_param == 0) {
                                    int v = (int)g_beacon_count_tft + add * (int)acc;
                                    if(v < 1) v = 1; if(v > 10) v = 10;
                                    g_beacon_count_tft = (uint8)v;
                                } else {
                                    int v = (int)g_target_point_num_tft + add * (int)acc;
                                    if(v < 1) v = 1; if(v > 40) v = 40;
                                    g_target_point_num_tft = (uint8)v;
                                }
                                tft180_clear();
                            }
                            break;
                        case 125: // 保存
                            EEPROM_Write_Data_uint32(beacon_count, g_beacon_count_tft);
                            EEPROM_Write_Data_uint32(target_point_num, g_target_point_num_tft);
                            Load_All_From_Flash();
                            ipc_push_params(4);  // 推送到 CM7_0
                            ins_edit_mode = 0; tft180_clear(); y = 53; break;
                        case 143: // 返回
                            ins_edit_mode = 0; tft180_clear(); y = 53; break;
                    }
                }
            }
            else
            {
                // ---- 菜单模式 ----
                tft180_show_chinese(5, 0, 16, chinese_guandaoshezhi[0], 4, RGB565_BLACK);
                // 信息行: "B:10/10 L:40/40" (x结束=117, <128)
                tft180_show_string(5, 18, "B:");  tft180_show_uint(21, 18, g_beacon_count_tft, 2);
                tft180_show_string(38, 18, "/10 L:");
                tft180_show_uint(77, 18, g_target_point_num_tft, 2);
                tft180_show_string(93, 18, "/40");
                // 菜单项
                tft180_show_chinese(5, 36, 16, chinese_zuobiaoshezhi[4], 2, RGB565_BLACK);
                tft180_show_chinese(37, 36, 16, chinese_shuliang[0], 2, RGB565_BLACK);    // 设置数量
                tft180_show_chinese(5, 54, 16, chinese_xinbiaozuobiao[0], 4, RGB565_BLACK);    // 信标坐标
                tft180_show_chinese(5, 72, 16, chinese_xinbiaoxulie[0], 4, RGB565_BLACK);    // 信标序列
                tft180_show_chinese(5, 90, 16, chinese_return[0], 5, RGB565_BLACK);
                tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y, RGB565_RED);
                if(get_key1) { y += 18; tft180_clear(); if(y > 107) y = 53; }
                if(get_key2) { y -= 18; tft180_clear(); if(y < 53)  y = 107; }
                if(get_key3)
                {
                    switch(y)
                    {
                        case 53:  ins_edit_mode = 1; y = 35; acc = 1; tft180_clear(); break;                    // 设置数量
                        case 71:  beacon_pos_read_all(beacon_pos_ram, 11); beacon_pos_edit_index = 0;
                                   acc = 0.1f; tft_show = 9; tft180_clear(); y = 35; break;                     // 信标坐标
                        case 89:  beacon_seq_read_all(beacon_seq_ram, 40); beacon_seq_edit_index = 0;
                                   add = 1; x = 25;
                                   tft_show = 5; tft180_clear(); y = 35; break;                                 // 信标序列
                        case 107: tft_show = 0; tft180_clear(); y = 35; break;                                 // 返回
                    }
                }
            }
            break;
        }

        // ====================================================================
        case 5 :// 信标序列编辑器
        // ====================================================================
        {
            tft180_show_string(5, 0, "BeaconSeq");
            tft180_show_string(5, 18, "No.");
            tft180_show_uint(32, 18, beacon_seq_edit_index + 1, 2);
            tft180_show_string(52, 18, "/");
            tft180_show_uint(60, 18, g_target_point_num_tft, 2);

            // 每行显示 5 个值，高亮当前值
            uint32 row = beacon_seq_edit_index / 5;
            uint32 start = row * 5;
            for (uint32 i = 0; i < 5 && (start + i) < 40; i++) {
                uint32 idx = start + i;
                uint8 val = beacon_seq_ram[idx];
                uint16 xpos = 5 + (uint16)(i * 24);
                if (idx == beacon_seq_edit_index)
                    { tft180_show_char(xpos, 40, '>'); }
                if(val >= 1 && val <= g_beacon_count_tft)
                    { tft180_show_uint(xpos + 8, 40, val, 1); }
                else
                    { tft180_show_string(xpos + 8, 40, "?"); }
            }

            // +/- 行: x=25对应+, x=85对应-
            tft180_show_char(25, 72, '+');  tft180_show_char(85, 72, '-');
            if(y == 89) {
                if(x == 25)      { tft180_show_char(16, 72, '('); tft180_show_char(34, 72, ')'); }
                else if(x == 85) { tft180_show_char(76, 72, '('); tft180_show_char(94, 72, ')'); }
            }
            // 保存 / 返回
            tft180_show_chinese(5, 90, 16, chinese_baocun[0], 2, RGB565_BLACK);
            tft180_show_chinese(5, 108, 16, chinese_return[0], 5, RGB565_BLACK);

            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y, RGB565_RED);

            // ---- KEY1 ----
            if(get_key1) {
                if(y == 35) {
                    // 值行：切换序列索引
                    beacon_seq_edit_index = (beacon_seq_edit_index + 1 < 40) ? beacon_seq_edit_index + 1 : 0;
                } else if(y == 89) {
                    // +/- 行: x 循环 25(+) → 85(-) → 0 → 跳行
                    if(x == 25)      { add = -1; x = 85; }      // 移到 -，add=-1
                    else if(x == 85) { add = 0;  x = 0;  }      // 取消选择
                    else             { y = 107; x = 25; }       // → 保存行
                } else if(y == 107) { y = 143; }
                else if(y == 143) { y = 35; }
                tft180_clear();
            }
            // ---- KEY2 ----
            if(get_key2) {
                if(y == 35) {
                    // 值行：切换序列索引
                    beacon_seq_edit_index = (beacon_seq_edit_index > 0) ? beacon_seq_edit_index - 1 : 39;
                } else if(y == 89) {
                    // +/- 行: 同 KEY1，对称操作
                    if(x == 85)      { add = 1;  x = 25; }      // 移到 +，add=1
                    else if(x == 25) { add = 0;  x = 0;  }      // 取消选择
                    else             { y = 35;  x = 25; }       // → 回值行
                } else if(y == 107) { y = 89; x = 25; add = 1; }
                else if(y == 143) { y = 107; }
                tft180_clear();
            }
            // ---- KEY3 ----
            if(get_key3)
            {
                switch(y)
                {
                    case 35: // 从值行进入操作区，默认选+
                        add = 1; y = 89; x = 25; tft180_clear(); break;
                    case 89: // 应用 +/-
                        { int v = (int)beacon_seq_ram[beacon_seq_edit_index] + add;
                          if(v < 1) v = g_beacon_count_tft;
                          if(v > g_beacon_count_tft) v = 1;
                          beacon_seq_ram[beacon_seq_edit_index] = (uint8)v;
                          tft180_clear(); }
                        break;
                    case 107: // 保存
                        beacon_seq_write_all(beacon_seq_ram, 40); Load_All_From_Flash();
                        ipc_push_params(4);
                        tft_show = 8; tft180_clear(); y = 89; break;
                    case 143: // 返回
                        tft_show = 8; tft180_clear(); y = 89; break;
                    default: break;
                }
            }
            break;
        }

        // ====================================================================
        case 9 :// 信标坐标编辑器
        // ====================================================================
        {
            // 标题: Beacon No.X/8
            tft180_show_string(5, 0, "Beacon");
            tft180_show_uint(60, 0, beacon_pos_edit_index + 1, 2);
            tft180_show_string(80, 0, "/11");
            // X 和 Y 值
            tft180_show_string(5, 18, "X:");
            tft180_show_float(30, 18, beacon_pos_ram[beacon_pos_edit_index][0], 3, 3);
            tft180_show_string(5, 36, "Y:");
            tft180_show_float(30, 36, beacon_pos_ram[beacon_pos_edit_index][1], 3, 3);
            // 高亮当前选中的字段
            if(y == 35 && x != 25) { tft180_show_char(19, 18, '>'); }
            if(y == 53 && x != 25) { tft180_show_char(19, 36, '>'); }
            // 步进
            tft180_show_chinese(5, 72, 16, chinese_changedobble[0], 4, RGB565_BLACK);
            tft180_set_font(TFT180_6X8_FONT);
            tft180_show_char(70, 76, '*'); tft180_show_float(78, 76, acc, 1, 3);
            tft180_set_font(TFT180_8X16_FONT);
            // +/-
            tft180_show_char(25, 90, '+'); tft180_show_char(85, 90, '-');
            if(y == 107)
            {
                if(x == 85)      { tft180_show_char(16, 90, '('); tft180_show_char(34, 90, ')'); }
                else if(x == 0)  { tft180_show_char(76, 90, '('); tft180_show_char(94, 90, ')'); }
            }
            // 保存&前进 / 保存&退出
            if(beacon_pos_edit_index + 1 < 11)
            {
                tft180_show_chinese(5, 108, 16, chinese_baocun[0], 2, RGB565_BLACK);
                tft180_show_string(37, 111, "+next");
            }
            else
            {
                tft180_show_chinese(5, 108, 16, chinese_baocun[0], 2, RGB565_BLACK);
                tft180_show_string(37, 111, "&exit");
            }
            tft180_show_chinese(5, 126, 16, chinese_return[0], 5, RGB565_BLACK);
            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y, RGB565_RED);
            // ---- KEY1 ----
            if(get_key1)
            {
                if(y == 107) { if(x==25){tft180_clear();add=1;x=85;}else if(x==85){tft180_clear();x=0;add=-1;}else{tft180_clear();y+=18;x=25;add=0;} }
                else { y += 18; tft180_clear(); if(y >= 144) y = 35; }
            }
            // ---- KEY2 ----
            if(get_key2)
            {
                if(y == 107) { if(x==25){tft180_clear();add=1;x=85;}else if(x==85){tft180_clear();x=0;add=-1;}else{tft180_clear();y-=18;x=25;add=0;} }
                else { y -= 18; tft180_clear(); if(y <= 34) y = 143; }
            }
            // ---- KEY3 ----
            if(get_key3)
            {
                switch(y)
                {
                    case 35: // 选中 X
                        focused_param = 0; y = 107; x = 25; tft180_clear(); break;
                    case 53: // 选中 Y
                        focused_param = 1; y = 107; x = 25; tft180_clear(); break;
                    case 89: // 步进循环: 0.1 ↔ 1.0
                        if(acc >= 1.0f) acc = 0.1f; else acc = 1.0f; tft180_clear(); break;
                    case 107: // 应用 +/- 到选中字段
                        if(add != 0)
                        {
                            if(focused_param == 0)
                                beacon_pos_ram[beacon_pos_edit_index][0] += (float)add * acc;
                            else
                                beacon_pos_ram[beacon_pos_edit_index][1] += (float)add * acc;
                            tft180_clear();
                        }
                        break;
                    case 125: // 保存 & 前进
                        beacon_pos_edit_index++;
                        if(beacon_pos_edit_index >= 11)
                        {
                            beacon_pos_write_all(beacon_pos_ram, 8); Load_All_From_Flash();
                            ipc_push_params(8);
                            tft_show = 8; tft180_clear(); y = 71;
                        }
                        else { tft180_clear(); }
                        break;
                    case 143: // 返回
                        tft_show = 8; tft180_clear(); y = 71; break;
                }
            }
            break;
        }

        // ====================================================================
        case 10 :// 预留（原坐标满警告）
        // ====================================================================
        {
            tft_show = 8; tft180_clear(); y = 53; break;
        }

        // ====================================================================
        case 11 :// 菜单设置（占位）
        // ====================================================================
        {
            tft180_show_chinese(5, 0, 16, chinese_caidanshezhi[0], 4, RGB565_BLACK);
            tft180_show_chinese(5, 72, 16, chinese_return[0], 5, RGB565_BLACK);

            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y, RGB565_RED);

            if(get_key1) { y += 18; tft180_clear(); if(y >= 90) y = 89; }
            if(get_key2) { y -= 18; tft180_clear(); if(y <= 88) y = 89; }
            if(get_key3 && y == 89)
            {
                tft_show = 0; tft180_clear(); y = 35;
            }
            break;
        }

        // ====================================================================
        case 12 :// 电量显示
        // ====================================================================
        {
            tft180_show_chinese(5, 0, 16, chinese_dianliangxianshi[0], 4, RGB565_BLACK);
            tft180_show_chinese(5, 18, 16, chinese_dianliang[0], 4, RGB565_BLACK);
            tft180_show_float(5, 36, voltage_power_supply_tft, 2, 3);
            tft180_show_chinese(5, 90, 16, chinese_return[0], 5, RGB565_BLACK);

            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y, RGB565_RED);

            if(get_key1) { y += 18; tft180_clear(); if(y >= 108) y = 107; }
            if(get_key2) { y -= 18; tft180_clear(); if(y <= 106) y = 107; }
            if(get_key3 && y == 107)
            {
                tft_show = 0; tft180_clear(); y = 35;
            }
            break;
        }

        // ====================================================================
        case 13 :// 串口查看
        // ====================================================================
        {
            tft180_show_chinese(5, 0, 16, chinese_chuankouchakan[0], 4, RGB565_BLACK);
            tft180_show_string(5, 18, "S1:"); tft180_show_float(40, 18, serial_data[0], 2, 3);
            tft180_show_string(5, 36, "S2:"); tft180_show_float(40, 36, serial_data[1], 2, 3);
            tft180_show_string(5, 54, "S3:"); tft180_show_float(40, 54, serial_data[2], 2, 3);
            tft180_show_string(5, 72, "S4:"); tft180_show_float(40, 72, serial_data[3], 2, 3);
            tft180_show_chinese(5, 90, 16, chinese_return[0], 5, RGB565_BLACK);

            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y, RGB565_RED);

            if(get_key1) { y += 18; tft180_clear(); if(y >= 108) y = 107; }
            if(get_key2) { y -= 18; tft180_clear(); if(y <= 106) y = 107; }
            if(get_key3 && y == 107)
            {
                tft_show = 0; tft180_clear(); y = 35;
            }
            break;
        }

        // ====================================================================
        case 14 :// 低电压警告
        // ====================================================================
        {
            tft180_show_chinese(5, 0, 16, chinese_vlow[0], 3, RGB565_RED);
            tft180_show_chinese(5, 18, 16, chinese_pleaselow[0], 4, RGB565_RED);
            tft180_show_float(5, 36, voltage_power_supply_tft, 2, 3);
            break;
        }

        // ====================================================================
        case 15 :// 预留
        // ====================================================================
        {
            break;
        }

        // ====================================================================
        case 16 :// 重定向到信标坐标编辑器（case 9）
        // ====================================================================
        {
            tft_show = 9; tft180_clear(); y = 35; break;
        }

    }

}
