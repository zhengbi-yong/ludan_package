/**
  *********************************************************************
  * @file      body_task.c/h
  * @brief     该任务控制腰部的一个电机，是DM6006，这个电机挂载在can3总线上
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "body_task.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "chassisR_task.h"

extern INS_t INS;
extern chassis_t chassis_move;															
body_t robot_body;

uint32_t BODY_TIME=1;	

float my_kd3=1.1f;
float my_kp3=90.0f;
float my_pos3=0.0f;
float my_pos3_set=0.0f;
int c=0;
uint8_t record_flag=0;
FDCAN_ProtocolStatusTypeDef fdcan_protocol_status;
FDCAN_ErrorCountersTypeDef err_cnt;
//void mit_ctrl3(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
void slope_following(float *target,float *set,float acc)
{
	if(*target > *set)
	{
		*set = *set + acc;
		if(*set >= *target)
		{
			*set = *target;
		}		
	}
	else if(*target < *set)
	{
		*set = *set - acc;
		if(*set <= *target)
		{
			*set = *target;
		}	
	}
}

void Body_task(void)
{
	osDelay(3000);
  body_init(&robot_body);

	while(1)
	{	
		if(chassis_move.joint_motor[0].para.kp_int_test!=0&&chassis_move.start_flag==1&&record_flag==0)	
		{
			record_flag=1;
			my_pos3_set=robot_body.loin_motor.para.pos;
		}
		//chassis_move.joint_motor[0].para.kp_int_test!=0&&
		if(chassis_move.joint_motor[0].para.kp_int_test!=0&&chassis_move.start_flag==1)	
		{
			slope_following(&my_pos3,&my_pos3_set,0.001f);
			//腰电机
			mit_ctrl3(&hfdcan3,0x09, my_pos3_set, 0.0f,my_kp3, my_kd3,0.0f);//6006 腰电机
		}
		else
		{ 
			record_flag=0;
			//腰电机
			mit_ctrl3(&hfdcan3,0x09, 0.0f, 0.0f,0.0f, my_kd3,0.0f);//6006 腰电机
			//HAL_FDCAN_GetProtocolStatus(&hfdcan3, &fdcan_protocol_status);
			//HAL_FDCAN_GetErrorCounters(&hfdcan3, &err_cnt);		
		}
		if(c==1)
		{
		  save_motor_zero(&hfdcan3,0x09, MIT_MODE);
		  osDelay(BODY_TIME);			
		}
		osDelay(BODY_TIME);
	}
}

void body_init(body_t *body)
{
	joint_motor_init(&body->loin_motor,0x09,MIT_MODE);//腰电机
	
	//腰电机
	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan3,body->loin_motor.para.id,body->loin_motor.mode);
	  osDelay(20);
	}
}








