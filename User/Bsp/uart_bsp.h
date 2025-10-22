#ifndef __UART_BSP_H__
#define __UART_BSP_H__

#include "main.h"

#define BUFF_SIZE	25

extern uint8_t rx_buff[BUFF_SIZE];

typedef __packed struct
{
  int16_t ch0;   // ԭ SBUS CH1������ˮƽҡ�ˣ�����Ϊ�������С�0
  int16_t ch1;   // ԭ SBUS CH2�����ִ�ֱҡ�ˣ���ǰΪ�������С�0
  int16_t ch2;   // ԭ SBUS CH3�����ִ�ֱҡ�ˣ�����Ϊ�������С�0
  int16_t ch3;   // ԭ SBUS CH4������ˮƽҡ�ˣ�����Ϊ�������С�0
  int16_t roll;  // ԭ SBUS CH5����չͨ������ӳ��Ϊ��̬/�Զ��幦��
  uint8_t sw1;   // ԭ SBUS CH6��SWA ���ˣ�����/���ο��أ��ڲ��� 0/1/2 ��ʾ��
  uint8_t sw2;   // ԭ SBUS CH7��SWD ���ˣ�����/���ο��أ��ڲ��� 0/1/2 ��ʾ��
	 uint16_t online;  
	uint32_t sbus_recever_time;
} rc_info_t;

extern rc_info_t rc;           // ��ң�����������ԭʼͨ������
 
#define rc_Init {0, 0, 0, 0, 0, 0, 0, 0, 0}

extern int my_printf(UART_HandleTypeDef *huart, const char *format, ...);
typedef struct
{
    uint16_t online;             // ң���Ƿ����ߣ�1 ��ʾ���ߣ�0 ��ʾ����
		uint32_t sbus_recever_time; // ���һ�γɹ����յ�ң�����ݵ�ʱ�����ms��

    struct
    {
        int16_t ch[10];          // ԭʼ SBUS 10 ͨ����ֵ��0~2047�������ݾ��߼�ʹ��
    } rc;

    struct
    {
        /* STICK VALUE */
        int16_t left_vert;       // ���ִ�ֱ�ᣨԭ CH3������Ϊ����
        int16_t left_hori;       // ����ˮƽ�ᣨԭ CH4������Ϊ����
        int16_t right_vert;      // ���ִ�ֱ�ᣨԭ CH2����ǰΪ����
        int16_t right_hori;      // ����ˮƽ�ᣨԭ CH1������Ϊ����
    } joy;
		
		struct
		{
			//l1 l2 r2 r1
			uint8_t swa;//2-Stop ԭ CH5�����ϲ��ˣ����ο��أ�0=��/�رգ�1=��/������
			uint8_t swb;//3-Stop ԭ CH6���������β��ˣ�0=�£�1=�У�2=�ϣ�
			uint8_t swc;//3-Stop ԭ CH7���������β��ˣ�0=�£�1=�У�2=�ϣ�
			uint8_t swd;//2-Stop ԭ CH8�����ϲ��ˣ����ο��أ�0=��/�رգ�1=��/������
		} toggle;

    struct
    {
        /* VAR VALUE */
        float a;                 // ԭ CH9��������ť/���� A��0~2047 ��Ӧ 0~100%��
        float b;                 // ԭ CH10��������ť/���� B��0~2047 ��Ӧ 0~100%��
    } var;

    struct
    {
        /* KEY VALUE */
        uint8_t l;               // ��ఴ��/�Զ��忪�أ��谴ӳ����䣩
			uint8_t r;               // �Ҳఴ��/�Զ��忪�أ��谴ӳ����䣩
    } key;
} remoter_t;

//extern remoter_t remoter;

#endif /*__UART_BSP_H__ */
