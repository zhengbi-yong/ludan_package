/**
  *********************************************************************
  * @file      observe_task.c/h
  * @brief     �������ǶԻ����˶��ٶȹ��ƣ��������ƴ�
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "connect_task.h"

#include "cmsis_os.h"
#include "bsp_usart1.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
extern UART_HandleTypeDef huart1;
extern send_data_t send_data;
														 															 
#define MOTOR_NUM 14
#define FRAME_LENGTH 72
#define LOAD_LENGTH 71
extern chassis_t chassis_move;																 															 														 

uint32_t OBSERVE_TIME=1;//����������3ms	
															 
void 	Connect_task(void)
{
  while(1)
	{  	
		send_data.tx[0]=FRAME_HEADER;
		for(int i=0;i<MOTOR_NUM;i++)
		{
			send_data.tx[1+i*5]=chassis_move.joint_motor[i].para.p_int>>8;
			send_data.tx[2+i*5]=chassis_move.joint_motor[i].para.p_int;
			send_data.tx[3+i*5]=chassis_move.joint_motor[i].para.v_int>>4;
			send_data.tx[4+i*5]=((chassis_move.joint_motor[i].para.v_int&0x0F)<<4)|(chassis_move.joint_motor[i].para.t_int>>8);
			send_data.tx[5+i*5]=chassis_move.joint_motor[i].para.t_int;
		}
		
		send_data.tx[LOAD_LENGTH]=Check_Sum(LOAD_LENGTH,send_data.tx); 
		
		//HAL_UART_Transmit_DMA(&huart1, (uint8_t *)send_data.tx, sizeof(send_data.tx));
		
		CDC_Transmit_HS((uint8_t *)send_data.tx,FRAME_LENGTH);
		
	  osDelay(OBSERVE_TIME);
	}
}


