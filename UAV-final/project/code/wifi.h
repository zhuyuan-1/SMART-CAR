#ifndef CODE_WIFIS_H_
#define CODE_WIFI_H_

#include "zf_common_headfile.h"


#define WIFI_SSID_TEST          "xrxsyy"        //名称
#define WIFI_PASSWORD_TEST      "999999999"  //密码
// 如果需要连接的WIFI 没有密码则需要将 这里 替换为 NULL

#define WIFI_TARGET_IP      "10.209.174.134"       // 连接目标的 IP
#define WIFI_TARGET_PORT    "8086"                 // 连接目标的端口
#define WIFI_LOCAL_PORT     "6666"                 // 本机的端口 0：随机  可设置范围2048-65535  默认 6666

//WiFi初始化
void wifi_init(void);
//TCP连接
void TCP_init(void);
// 逐飞助手初始化
void seekfree_init(int judge);
//逐飞助手调参初始化
void assistant_init(float a[]);

//wifi串口接收
void wifi_get(uint8 wifi_spi_get_data_buffer[]);
//wifi串口接受字符并返回输出
void wifi_sendreturn(uint8 wifi_spi_get_data_buffer[]);

//逐飞助手波形设置
void seekfree_oscilloscope_send(int seekfree_data, float a[]);
//逐飞助手调参设置
void seekfree_assistant_change(float a[]);
//逐飞助手摄像头发送图像
void seekfree_camera_send(void);

#endif /* CODE_PID_H_ */
