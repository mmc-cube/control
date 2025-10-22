#ifndef __UART_BSP_H__
#define __UART_BSP_H__

#include "main.h"

#define BUFF_SIZE	25

extern uint8_t rx_buff[BUFF_SIZE];

typedef __packed struct
{
  int16_t ch0;   // 原 SBUS CH1：右手水平摇杆，向右为正，居中≈0
  int16_t ch1;   // 原 SBUS CH2：右手垂直摇杆，向前为正，居中≈0
  int16_t ch2;   // 原 SBUS CH3：左手垂直摇杆，向上为正，居中≈0
  int16_t ch3;   // 原 SBUS CH4：左手水平摇杆，向右为正，居中≈0
  int16_t roll;  // 原 SBUS CH5：扩展通道，可映射为姿态/自定义功能
  uint8_t sw1;   // 原 SBUS CH6：SWA 拨杆（两段/三段开关，内部用 0/1/2 表示）
  uint8_t sw2;   // 原 SBUS CH7：SWD 拨杆（两段/三段开关，内部用 0/1/2 表示）
	 uint16_t online;  
	uint32_t sbus_recever_time;
} rc_info_t;

extern rc_info_t rc;           // 新遥控器解析后的原始通道数据
 
#define rc_Init {0, 0, 0, 0, 0, 0, 0, 0, 0}

extern int my_printf(UART_HandleTypeDef *huart, const char *format, ...);
typedef struct
{
    uint16_t online;             // 遥控是否在线：1 表示在线，0 表示离线
		uint32_t sbus_recever_time; // 最近一次成功接收到遥控数据的时间戳（ms）

    struct
    {
        int16_t ch[10];          // 原始 SBUS 10 通道数值（0~2047），兼容旧逻辑使用
    } rc;

    struct
    {
        /* STICK VALUE */
        int16_t left_vert;       // 左手垂直轴（原 CH3，向上为正）
        int16_t left_hori;       // 左手水平轴（原 CH4，向右为正）
        int16_t right_vert;      // 右手垂直轴（原 CH2，向前为正）
        int16_t right_hori;      // 右手水平轴（原 CH1，向右为正）
    } joy;
		
		struct
		{
			//l1 l2 r2 r1
			uint8_t swa;//2-Stop 原 CH5：左上拨杆，两段开关（0=下/关闭，1=上/开启）
			uint8_t swb;//3-Stop 原 CH6：左上三段拨杆（0=下，1=中，2=上）
			uint8_t swc;//3-Stop 原 CH7：右上三段拨杆（0=下，1=中，2=上）
			uint8_t swd;//2-Stop 原 CH8：右上拨杆，两段开关（0=下/关闭，1=上/开启）
		} toggle;

    struct
    {
        /* VAR VALUE */
        float a;                 // 原 CH9：附加旋钮/滑块 A（0~2047 对应 0~100%）
        float b;                 // 原 CH10：附加旋钮/滑块 B（0~2047 对应 0~100%）
    } var;

    struct
    {
        /* KEY VALUE */
        uint8_t l;               // 左侧按键/自定义开关（需按映射填充）
			uint8_t r;               // 右侧按键/自定义开关（需按映射填充）
    } key;
} remoter_t;

//extern remoter_t remoter;

#endif /*__UART_BSP_H__ */
