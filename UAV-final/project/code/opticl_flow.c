#include "optical_flow.h"
optical_flow flow_get;                                                           //光流数据读取结构体
uint8 uart_get_data[128];                                                       // 串口接收数据缓冲区
uint8 fifo_get_data[128];                                                       // fifo 输出读出缓冲区

uint8  get_data = 0;                                                            // 接收数据变量
uint32 fifo_data_count = 0;                                                     // fifo 数据个数
uint8 communication_judge = 0;                                                  // 串口通信状态机标志位
uint8 out_judge = 0;                                                            // 输出标志位
uint8 duta_judge = 1;                                                           // 串口发送标志位
uint8 duta[15]={0};                                                             // 数据包
uint16 time_uart = 25000;                                                       // 25ms,以us形式发送
fifo_struct uart_data_fifo;

// 超时计数需要设为静态，避免每次调用重置
static uint32 comm_timeout_counter = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       串口通信主机状态机函数
// 参数说明       void
// 返回参数       void
// 使用示例       communication_master();
//-------------------------------------------------------------------------------------------------------------------

void opyical_flow_get(void)
{
    uint8 date_judge = 1;
    uint32 len = 15;                           // 数据包长度标志位
    
    // 超时逻辑修正：静态变量持续计数，达到阈值重置
    comm_timeout_counter++;
    if(comm_timeout_counter > 1000)            // 超时计数（需根据主循环频率调整）
    {
        comm_timeout_counter = 0;
        date_judge = 1;
        communication_judge = 0;               // 超时后重置状态机
    }

    fifo_data_count = fifo_used(&uart_data_fifo);  // 查看 fifo 是否有数据
    
    switch(communication_judge)
    {
        case 0: {    // 建立单片机与单片机之间的串口通信
            if(date_judge)  // 只在需要时发送握手包
            {
                uint8 a[2] = {0xA5, 0xB6};
                uart_write_buffer(UART_INDEX, a, 2);
                date_judge = 0;
            }
            
            if(fifo_data_count != 0)            // 读取到数据了
            {
                fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);
                
                // 检查握手应答（增加边界检查）
                if(fifo_data_count >= 2 && fifo_get_data[0] == 0xA5 && fifo_get_data[1] == 0xB6)
                {
                    communication_judge = 1;
                    comm_timeout_counter = 0;   // 收到应答重置超时计数
                }
            }
        } break;
        
        case 1: {
            fifo_data_count = fifo_used(&uart_data_fifo);
            
            if(fifo_data_count >= len)          // 确保数据长度足够
            {
                fifo_read_buffer(&uart_data_fifo, fifo_get_data, &len, FIFO_READ_AND_CLEAN);
                
                // 检查帧头帧尾
                if(fifo_get_data[0] == 0xDF && fifo_get_data[14] == 0xEA)
                {
                    // 光流数据解析：高低位顺序调整
                    flow_get.optical_x_drift = (int16)((fifo_get_data[7] << 8) | fifo_get_data[8]);
                    flow_get.optical_y_drift = (int16)((fifo_get_data[9] << 8) | fifo_get_data[10]);
                    flow_get.optical_x_drift_i += flow_get.optical_x_drift;
                    flow_get.optical_y_drift_i += flow_get.optical_y_drift;
                    flow_get.optical_timespan= (uint16)((fifo_get_data[12] << 8) | fifo_get_data[13]);
                    flow_get.optical_TOF1_mm = (uint16)((fifo_get_data[2] << 8) | fifo_get_data[3]);
                    flow_get.optical_TOF2_mm = (uint16)((fifo_get_data[4] << 8) | fifo_get_data[5]);
                    comm_timeout_counter = 0;   // 收到数据重置超时计数
                }
            }
        } break;
        
        default: break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       串口通信初始化函数
// 参数说明       void
// 返回参数       void
// 使用示例       communication_init();
//-------------------------------------------------------------------------------------------------------------------
void optical_flow_init(void)
 {
   fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 128);              // 初始化 fifo 挂载缓冲区
   uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // 初始化串口
   uart_rx_interrupt(UART_INDEX, 1);                                           // 开启 UART_INDEX 的接收中断
   fifo_clear (&uart_data_fifo);
   //初始化各个参数
   flow_get.optical_x_drift= 0;
   flow_get.optical_timespan = 0;
   flow_get.optical_TOF1_mm = 0;
   flow_get.optical_TOF2_mm = 0;
   flow_get.optical_y_drift = 0;
   flow_get.optical_x_drift_i = 0;
   flow_get.optical_y_drift_i = 0;
 //  uart_write_string(UART_INDEX, "UART INIT SUCCESS.");                        // 输出测试信息
 //  uart_write_byte(UART_INDEX, '\r');                                          // 输出回车
//   uart_write_byte(UART_INDEX, '\n');                                          // 输出换行

 }
 
//-------------------------------------------------------------------------------------------------------------------
// 函数简介       UART_INDEX 的接收中断处理函数 这个函数将在 UART_INDEX 对应的中断调用
// 参数说明       void
// 返回参数       void
// 使用示例       uart_rx_interrupt_handler();
//-------------------------------------------------------------------------------------------------------------------
void flow_uart_interrupt_handler (void)
{
//    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
    if(uart_query_byte(UART_INDEX, &get_data))                                  // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    {
        fifo_write_element(&uart_data_fifo, get_data);                   // 将数据写入 fifo
    }
}
