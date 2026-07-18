#include "robot_def.h"
#include "include.h"
#include "Imu_fusion.h"

void Task_Chassis_Entry(void *argument)
{
    Chassis_Cmd_t cmd = {0};
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    TickType_t xLastWakeTime = xTaskGetTickCount();
 
  for(;;)
  {
      Chassis_Cmd_t new_cmd;
      
     IMU_Update(0.001f);
        
        if (osMessageQueueGet(Chassis_Cmd_QHandle, &new_cmd, NULL, 0) == osOK)
        {
            cmd = new_cmd;
        }
     if (cmd.mode == 0)
        {
            Swerve_Stop();
            
           for(int i=0; i<4; i++) can2_tx_table.target_current[i] = 0; 
           can2_tx_table.update_flag = 0;
        }    
    else
       {
         for (int i = 0; i < 4; i++) {
         swerve_modules[i].current_angle_deg = motor_6020[i].current_angle_deg;
        }
        Chassis_Control(cmd.mode,-cmd.vx,-cmd.vy,-cmd.omega,0.001f);   // YawHold + inverse kinematics
//    Swerve_Update(-cmd.vx,-cmd.vy,-cmd.omega);

            // PID(dt0.001)
        Swerve_Execute(0.001f);
       }
 
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
