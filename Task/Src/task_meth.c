#include "robot_def.h"
#include "include.h"
#include "recv_send.h"

void Task_Mech_Entry(void *argument)
{
    Mech_Cmd_t cmd={MODE_LOCK_ALL,0,0,0};
   
    const TickType_t xFrequency = pdMS_TO_TICKS(3);
    TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {    
       
       if (osMessageQueueGet(Mech_Cmd_QHandle, &cmd, NULL, 0) == osOK)
        {
        
        switch (cmd.mode)
            {
            case MODE_SERVO_2D:Mech_Servo_Logic(cmd.ch2_raw,cmd.ch3_raw);break;

            case MODE_RELAY_2D:Mech_Suction_Logic(cmd.ch2_raw,cmd.ch3_raw,cmd.lift_speed);break;
							
					  case MODE_SHANGWEIJI:Host_Servo_Logic(g_cmd);
							
            case MODE_CHASSIS_FULL:Mech_Lift_SetTarget(cmd.ch3_raw,cmd.lift_speed);break;

            case MODE_LOCK_ALL:
            default:
                // 锁死/安全模式
                Pneumatic_Gripper_Ctrl(VALVE_ON);
                Pneumatic_Suction_Ctrl(VALVE_OFF);
                can1_tx_table.target_current[4]=0.0f;
						    can2_tx_table.target_current[5] =0.0f;
                can1_tx_table.update_flag = 0;
                break;
            }
        }
				
				Mech_Lift_PID_Update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
