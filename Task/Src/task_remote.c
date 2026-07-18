#include "robot_def.h"
#include "include.h"
#include "tool_calc.h"
#include "sbus.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "recv_send.h"


void Task_Remote_Entry(void *argument)
{
       Chassis_Cmd_t chassis_cmd={0};
       Mech_Cmd_t mech_cmd={MODE_LOCK_ALL,0,0,0};

    Robot_Mode_e current_mode = MODE_LOCK_ALL;

    const TickType_t xFrequency = pdMS_TO_TICKS(10);
   TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
					
		if (channels.tsw1 == RC_SW_DOWN && channels.tsw2 == RC_SW_DOWN)     current_mode = MODE_LOCK_ALL;
		else if (channels.tsw1 == RC_SW_MID && channels.tsw2 == RC_SW_MID)  current_mode = MODE_CHASSIS_FULL;
		else if (channels.tsw1 == RC_SW_MID && channels.tsw2 == RC_SW_UP)   current_mode = MODE_RELAY_2D;
    else if (channels.tsw1 == RC_SW_UP && channels.tsw2 == RC_SW_MID)   current_mode = MODE_SERVO_2D;
    else if (channels.tsw1 == RC_SW_UP && channels.tsw2 == RC_SW_UP)
	{
     current_mode = MODE_SHANGWEIJI;

    if (channels.ssw1 == RC_SW_UP && channels.ssw2 == RC_SW_UP)
    {
        if (channels.roll_2 < 1500)
          send_id(1); //�ص���ʼ��
        else
        {
            send_id(0); //���ɿ���
            current_mode = MODE_CHASSIS_FULL;
        }
    }
    else if (channels.ssw1 == RC_SW_UP && channels.ssw2 == RC_SW_DOWN)
    {
        send_id(2); //�Ƶ��Ÿ˼�
    }
    else if (channels.ssw1 == RC_SW_DOWN && channels.ssw2 == RC_SW_UP)
    {
        send_id(4); //�Խ����
    }
    else if (channels.ssw1 == RC_SW_DOWN && channels.ssw2 == RC_SW_DOWN)
    {
        send_id(3); //�Խ�
    }
 }
        chassis_cmd.mode=current_mode;
        mech_cmd.mode = current_mode;
        
        switch(current_mode)
        { 
            case MODE_CHASSIS_FULL:
                    chassis_cmd.vy = rc_process_channel(channels.ch4, MAX_SPEED_VY, RC_EXPO_RATIO, LPF_RC_ALPHA_VY, RC_CH_VY_IDX);
                    chassis_cmd.vx = rc_process_channel(channels.ch2, MAX_SPEED_VX, RC_EXPO_RATIO, LPF_RC_ALPHA_VX, RC_CH_VX_IDX);
                    chassis_cmd.omega = rc_process_channel(channels.ch1, MAX_SPEED_OMEGA, RC_EXPO_RATIO, LPF_RC_ALPHA_OMEGA, RC_CH_OMEGA_IDX); 
                    mech_cmd.ch3_raw=channels.roll_1;mech_cmd.lift_speed=channels.ssw1; break;
            
            case MODE_SERVO_2D:
                  chassis_cmd.vy = rc_process_channel(channels.ch4, MAX_SPEED_VY, RC_EXPO_RATIO, LPF_RC_ALPHA_VY, RC_CH_VY_IDX);
                  chassis_cmd.vx = rc_process_channel(channels.ch2, MAX_SPEED_VX, RC_EXPO_RATIO, LPF_RC_ALPHA_VX, RC_CH_VX_IDX);
                  chassis_cmd.omega = 0;
                  mech_cmd.ch2_raw=channels.ch1; mech_cmd.ch3_raw=channels.ch3;mech_cmd.lift_speed=0; break;
            case MODE_RELAY_2D:
                 mech_cmd.ch2_raw=channels.ch2; mech_cmd.ch3_raw=channels.ch4;mech_cmd.lift_speed=channels.ch1; break;
            case MODE_SHANGWEIJI:  break;                  
            case MODE_LOCK_ALL:
             default:
                chassis_cmd.vx = chassis_cmd.vy = chassis_cmd.omega = 0;mech_cmd.ch3_raw=0; mech_cmd.ch2_raw=0;mech_cmd.lift_speed=0;break;
        }       
    xQueueOverwrite(Chassis_Cmd_QHandle, &chassis_cmd);
    xQueueOverwrite(Mech_Cmd_QHandle, &mech_cmd);
        
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

}

