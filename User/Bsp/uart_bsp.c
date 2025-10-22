#include "uart_bsp.h"
#include "string.h"
#include "usart.h"
#include "math.h"


#define SBUS_HEAD 0X0F
#define SBUS_END 0X00
#define REMOTE_RC_OFFSET 1024
#define REMOTE_TOGGLE_DUAL_VAL 1024
#define REMOTE_TOGGLE_THRE_VAL_A 600
#define REMOTE_TOGGLE_THRE_VAL_B 1400
#define DEAD_AREA	120

rc_info_t rc = rc_Init;

#define abs(x) ((x) < 0 ? -(x) : (x))

uint8_t rx_buff[BUFF_SIZE];
remoter_t remoter;

void sbus_frame_parse(remoter_t *remoter, uint8_t *buf)
{
    if ((buf[0] != SBUS_HEAD) || (buf[24] != SBUS_END))
        return;

    if (buf[23] == 0x0C)
        remoter->online = 0;
    else
        remoter->online = 1;

    remoter->rc.ch[0] = ((buf[1] | buf[2] << 8) & 0x07FF);
    remoter->rc.ch[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
    remoter->rc.ch[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF);
    remoter->rc.ch[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF);
    remoter->rc.ch[4] = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
    remoter->rc.ch[5] = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
    remoter->rc.ch[6] = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
    remoter->rc.ch[7] = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
    remoter->rc.ch[8] = ((buf[12] | buf[13] << 8) & 0x07FF);
    remoter->rc.ch[9] = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);
		
		remoter->joy.left_hori = remoter->rc.ch[4-1] - REMOTE_RC_OFFSET;
		remoter->joy.left_vert = remoter->rc.ch[3-1]- REMOTE_RC_OFFSET;
		remoter->joy.right_hori = remoter->rc.ch[1-1] - REMOTE_RC_OFFSET;
		remoter->joy.right_vert = remoter->rc.ch[2-1] - REMOTE_RC_OFFSET;
		
		if( remoter->joy.left_hori > -DEAD_AREA && remoter->joy.left_hori < DEAD_AREA)
			remoter->joy.left_hori = 0;
		if( remoter->joy.right_hori > -DEAD_AREA && remoter->joy.right_hori < DEAD_AREA)
			remoter->joy.right_hori = 0;
		if( remoter->joy.right_vert > -DEAD_AREA && remoter->joy.right_vert < DEAD_AREA)
			remoter->joy.right_vert = 0;
		
		if(remoter->rc.ch[5-1] < REMOTE_TOGGLE_DUAL_VAL)
		{
			remoter->toggle.swa = 0;
		}
		else if(remoter->rc.ch[5-1] >= REMOTE_TOGGLE_DUAL_VAL)
		{
			remoter->toggle.swa = 1;
		}
		
		if(remoter->rc.ch[6-1] < REMOTE_TOGGLE_THRE_VAL_A)
		{
			remoter->toggle.swb = 0;
		}
		else if(remoter->rc.ch[6-1] >= REMOTE_TOGGLE_THRE_VAL_A && remoter->rc.ch[6-1] <= REMOTE_TOGGLE_THRE_VAL_B)
		{
			remoter->toggle.swb = 1;
		}
		else if(remoter->rc.ch[6-1] >= REMOTE_TOGGLE_THRE_VAL_B)
		{
			remoter->toggle.swb = 2;
		}
		
		if(remoter->rc.ch[7-1] < REMOTE_TOGGLE_THRE_VAL_A)
		{
			remoter->toggle.swc = 0;
		}
		else if(remoter->rc.ch[7-1] >= REMOTE_TOGGLE_THRE_VAL_A && remoter->rc.ch[7-1] <= REMOTE_TOGGLE_THRE_VAL_B)
		{
			remoter->toggle.swc = 1;
		}
		else if(remoter->rc.ch[7-1] >= REMOTE_TOGGLE_THRE_VAL_B)
		{
			remoter->toggle.swc = 2;
		}
		
		if(remoter->rc.ch[8-1] < REMOTE_TOGGLE_DUAL_VAL)
		{
			remoter->toggle.swd = 0;
		}
		else if(remoter->rc.ch[8-1] >= REMOTE_TOGGLE_DUAL_VAL)
		{
			remoter->toggle.swd = 1;
		}
		
		remoter->var.a = remoter->rc.ch[9-1];
		remoter->var.b = remoter->rc.ch[10-1];
		
		remoter->sbus_recever_time = HAL_GetTick();
		
		
		
}

void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{
	
	//?????????????????????rc??????????????????????
  rc->ch0 = (buff[0] | buff[1] << 8) & 0x07FF;//??buff[0]??buff[1]???????ch0????????????????????11¦Ë???????0x07FF??¦Ë??
  rc->ch0 -= 1024;//??????????364??1684????????????????1024???????????0
  rc->ch1 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  rc->ch3 -= 1024;
  rc->roll = (buff[16] | (buff[17] << 8)) & 0x07FF;  //????????
  rc->roll -= 1024;
 
  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (buff[5] >> 4) & 0x0003;//sw1??sw2?????????????¦Ë????¨®?
    
  if ((abs(rc->ch0) > 660) || \
      (abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660))
      
  {
    memset(rc, 0, sizeof(rc_info_t));//???????????????????660???????????????????rc?????????????????
  }	
		rc->sbus_recever_time = HAL_GetTick();
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{

	if(huart->Instance == UART5)
	{
		if (Size <= BUFF_SIZE)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2); // ????????????
			rc_callback_handler(&rc, rx_buff);
			
//			memset(rx_buff, 0, BUFF_SIZE);
		}
		else  // ??????????????BUFF_SIZE????????
		{	
			HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2); // ????????????
			memset(rx_buff, 0, BUFF_SIZE);							   
		}
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == UART5)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff, BUFF_SIZE*2); // ????????????????
		memset(rx_buff, 0, BUFF_SIZE);							   // ??????????		
	}
}
