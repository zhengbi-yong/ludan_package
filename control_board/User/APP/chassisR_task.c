/**
  *********************************************************************
  * @file      chassisR_task.c/h
  * @brief     该任务控制右腿的五个电机，都是DM4340，这五个电机挂载在can1总线上
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "chassisR_task.h"
#include "fdcan.h"
#include "cmsis_os.h"
													
chassis_t chassis_move;

uint32_t CHASSR_TIME=1;	

float my_kd2=0.0f;
float my_vel2=0.0f;
float my_kp2=0.0f;
float my_pos2=0.0f;
float my_tor2=0.0f;
int a=0;
void ChassisR_task(void)
{
	chassis_move.start_flag=1;
	osDelay(2000);
    ChassisR_init(&chassis_move);

	dm6248p_fbdata_init(&chassis_move.joint_motor[7]);
  dm6248p_fbdata_init(&chassis_move.joint_motor[8]);
	dm6248p_fbdata_init(&chassis_move.joint_motor[9]);
	dm4340_fbdata_init(&chassis_move.joint_motor[10]);
	dm4340_fbdata_init(&chassis_move.joint_motor[11]);
	dm4340_fbdata_init(&chassis_move.joint_motor[12]);
	dm4340_fbdata_init(&chassis_move.joint_motor[13]);
  chassis_move.start_flag=1;
	while(1)
	{	
		if(chassis_move.start_flag==1)	
		{
			mit_ctrl_test(&hfdcan1,0x01,&chassis_move.joint_motor[7]);
			mit_ctrl_test(&hfdcan1,0x02,&chassis_move.joint_motor[8]);
			mit_ctrl_test(&hfdcan1,0x0A,&chassis_move.joint_motor[9]);
			mit_ctrl_test(&hfdcan1,0x05,&chassis_move.joint_motor[10]);
			mit_ctrl_test(&hfdcan1,0x07,&chassis_move.joint_motor[11]);
			mit_ctrl_test(&hfdcan1,0x08,&chassis_move.joint_motor[12]);
			mit_ctrl_test(&hfdcan1,0x09,&chassis_move.joint_motor[13]);
		}
		else
		{ //void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
		  mit_ctrl2(&hfdcan1,0x01, 0.0f, 0.0f,0.0f, 0.0f,my_tor2);//right_pitch
		  mit_ctrl2(&hfdcan1,0x02, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right_roll
		  mit_ctrl2(&hfdcan1,0x0A, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right_yaw
		  mit_ctrl2(&hfdcan1,0x05, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right_calf
		  mit_ctrl2(&hfdcan1,0x07, my_pos2, my_vel2,my_kp2, my_kd2,0.0f);//right_foot		
      	  mit_ctrl2(&hfdcan1,0x08, my_pos2, my_vel2,my_kp2, my_kd2,0.0f);//right_foot	
      	  mit_ctrl2(&hfdcan1,0x09, my_pos2, my_vel2,my_kp2, my_kd2,0.0f);//right_foot				
		}
		if(a==1)
		{
			// save_motor_zero(&hfdcan1,0x05, MIT_MODE);
			// osDelay(CHASSR_TIME);
			save_motor_zero(&hfdcan1,0x04, MIT_MODE);
			// osDelay(CHASSR_TIME);
			// save_motor_zero(&hfdcan1,0x03, MIT_MODE);
			// osDelay(CHASSR_TIME);
			// save_motor_zero(&hfdcan1,0x02, MIT_MODE);
			// osDelay(CHASSR_TIME);
			// save_motor_zero(&hfdcan1,0x01, MIT_MODE);
			osDelay(CHASSR_TIME);
		}
		osDelay(CHASSR_TIME);
	}
}

void ChassisR_init(chassis_t *chassis)
{
	joint_motor_init(&chassis->joint_motor[7],1,MIT_MODE);
	joint_motor_init(&chassis->joint_motor[8],0x2,MIT_MODE);
	joint_motor_init(&chassis->joint_motor[9],0xA,MIT_MODE);
	joint_motor_init(&chassis->joint_motor[10],5,MIT_MODE);
	joint_motor_init(&chassis->joint_motor[11],7,MIT_MODE);
	joint_motor_init(&chassis->joint_motor[12],8,MIT_MODE);
	joint_motor_init(&chassis->joint_motor[13],9,MIT_MODE);
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan1,chassis->joint_motor[7].para.id,chassis->joint_motor[7].mode);
	  osDelay(100);
	}	
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hfdcan1,chassis->joint_motor[8].para.id,chassis->joint_motor[8].mode);
	  osDelay(100);
	}
	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan1,chassis->joint_motor[9].para.id,chassis->joint_motor[9].mode);
	  osDelay(100);
	}
	




	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan1,chassis->joint_motor[10].para.id,chassis->joint_motor[10].mode);
	  osDelay(100);
	}
	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan1,chassis->joint_motor[11].para.id,chassis->joint_motor[11].mode);
	  osDelay(100);
	}
	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan1,chassis->joint_motor[12].para.id,chassis->joint_motor[12].mode);
	  osDelay(100);
	}
	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan1,chassis->joint_motor[13].para.id,chassis->joint_motor[13].mode);
	  osDelay(100);
	}
	
}


/*
void ChassisR_init(chassis_t *chassis)
{
    // 初始化所有电机的参数结构体
    joint_motor_init(&chassis->joint_motor[7], 1, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[8], 2, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[9], 3, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[10], 5, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[11], 7, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[12], 8, MIT_MODE);
    joint_motor_init(&chassis->joint_motor[13], 9, MIT_MODE);
    
    // 短暂延时，确保系统稳定
    osDelay(100);

    // 循环为所有电机发送一次使能指令
    // 注意：这里的数组索引是从7到13
    for(int i = 7; i <= 13; i++)
    {
        enable_motor_mode(&hfdcan1, chassis->joint_motor[i].para.id, chassis->joint_motor[i].mode);
        osDelay(20); // 留出20ms的间隔，防止CAN总线拥堵，也给电机响应时间
    }
    
    // 在这里，最好能有一个机制来查询并确认所有电机都已进入MIT模式
    // (这需要你的驱动库支持读取电机状态的功能)
}
*/

void mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}





