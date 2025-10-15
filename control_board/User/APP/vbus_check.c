/**
  *********************************************************************
  * @file      VBUS_Check_task.c/h
  * @brief     ���������ڼ���ص�ѹ������⵽��ص�ѹ����22Vʱ��ʧ�����е��
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
#include "vbus_check.h"
#include "adc.h"
#include "cmsis_os.h"
#include "gpio.h"

#include "chassisR_task.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "body_task.h"	
#include "tim.h"
extern chassis_t chassis_move;
extern body_t robot_body;

static uint16_t adc_val[2];
static float vbus;


static uint16_t calibration_value=378;

uint8_t loss_voltage = 0;

#define vbus_threhold_disable (22.2f)

#define vbus_threhold_call (22.6f)

void VBUS_Check_task(void)
{
	
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val,2);
	
	while(1)
	{
		vbus = ((adc_val[1]+calibration_value)*3.3f/65535)*11.0f;
		
		if(6.0f<vbus&&vbus<vbus_threhold_call)
		{//��ص�ѹС��22.6V������������
			Buzzer_ON;
		}
		else
		{
			Buzzer_OFF;
		}
		if(6.0f<vbus&&vbus<vbus_threhold_disable)
		{//��ص�ѹС��22.2V��ʧ�ܵ��
			loss_voltage=1;
			Power_OUT2_OFF;
			Power_OUT1_OFF;
			
			//��ص�ѹ���ˣ�ʧ�ܵ��
			for(int j=0;j<7;j++)
			{
				disable_motor_mode(&hfdcan1,chassis_move.joint_motor[7].para.id,chassis_move.joint_motor[7].mode);
				disable_motor_mode(&hfdcan1,chassis_move.joint_motor[8].para.id,chassis_move.joint_motor[8].mode);
				disable_motor_mode(&hfdcan1,chassis_move.joint_motor[9].para.id,chassis_move.joint_motor[9].mode);
				disable_motor_mode(&hfdcan1,chassis_move.joint_motor[10].para.id,chassis_move.joint_motor[10].mode);
				disable_motor_mode(&hfdcan1,chassis_move.joint_motor[11].para.id,chassis_move.joint_motor[11].mode);
				disable_motor_mode(&hfdcan1,chassis_move.joint_motor[12].para.id,chassis_move.joint_motor[12].mode);
				disable_motor_mode(&hfdcan1,chassis_move.joint_motor[13].para.id,chassis_move.joint_motor[13].mode);
				osDelay(5);
			}
			for(int j=0;j<7;j++)
			{
				disable_motor_mode(&hfdcan2,chassis_move.joint_motor[0].para.id,chassis_move.joint_motor[0].mode);
				disable_motor_mode(&hfdcan2,chassis_move.joint_motor[1].para.id,chassis_move.joint_motor[1].mode);
				disable_motor_mode(&hfdcan2,chassis_move.joint_motor[2].para.id,chassis_move.joint_motor[2].mode);
				disable_motor_mode(&hfdcan2,chassis_move.joint_motor[3].para.id,chassis_move.joint_motor[3].mode);
				disable_motor_mode(&hfdcan2,chassis_move.joint_motor[4].para.id,chassis_move.joint_motor[4].mode);
				disable_motor_mode(&hfdcan2,chassis_move.joint_motor[5].para.id,chassis_move.joint_motor[5].mode);
				disable_motor_mode(&hfdcan2,chassis_move.joint_motor[6].para.id,chassis_move.joint_motor[6].mode);
				osDelay(5);
			}
			for(int j=0;j<7;j++)
			{
				disable_motor_mode(&hfdcan3,robot_body.loin_motor.para.id,robot_body.loin_motor.mode);
				osDelay(5);
			}			
		}
		else
		{
			Power_OUT2_ON;
			Power_OUT1_ON;
						
			loss_voltage=0;
		}
		
		osDelay(100);
	}
}


