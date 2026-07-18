#include "robot_def.h"
#include "include.h"
#include "tool_calc.h"

static uint8_t servo_trigger_lock = 1;  // 扳机保险
//static uint8_t servo_trigger_flag = 1; 
static uint8_t claw_at_work = 0; 
static uint8_t gimbal_at_work = 0; 

//static uint8_t suction_initialized = 0;
//static float suction_home_pos = 0;
//static float suction_target_pos = 0;
static uint8_t suction_flag=1;
static uint8_t suction_state=1;
 uint8_t motor_lock =1;
 
static float lift_target_pos = POS_DOWN; 

void Host_Servo_Logic(float input_val) {

    // 触发夹取 
    if (input_val==1) //关夹爪
    {
        Pneumatic_Gripper_Ctrl(VALVE_OFF);

    }
    else if (input_val==2)//把杆横过来
   {
       Servo_SetAngle(&servo_gimbal,330.0f);
   }
    else if (input_val == 3) //辅助担架伸出
    {
        Servo_SetAngle(&servo_claw, 270.0f);
    }
    else if (input_val ==4)//辅助担架收回
    {
        Servo_SetAngle(&servo_claw, 30.0f);
         
    }
    else if (input_val == 5) //把杆竖过来
    {
        Servo_SetAngle(&servo_claw, 48.0f);
    }
} 

/* ---三重夹爪机构 --- */
void Mech_Servo_Logic(float input_val, float roll_val)
{
    float stick_val = apply_deadzone(2.0f * (input_val - 1000.0f) / 1000.0f - 1.0f, 0.1f);
    float roll_stick_val = apply_deadzone(2.0f * (roll_val - 1000.0f) / 1000.0f - 1.0f, 0.1f);

    if (stick_val == 0.0f && roll_stick_val == -1.0f) {
        servo_trigger_lock = 1;
    }

    // 触发夹取 
    if (stick_val > 0.5f && servo_trigger_lock)
    {  
        Servo_SetAngle(&servo_gimbal,48.0f);
        Pneumatic_Gripper_Ctrl(VALVE_OFF); // 夹爪关闭

         gimbal_at_work = 1;
        claw_at_work = 1;

        servo_trigger_lock = 0;  // 锁住，防连发 
    }
    // 触发归位 
    else if (stick_val < -0.5f && servo_trigger_lock)
    {
  
        if (gimbal_at_work)
        {
            Servo_SetAngle(&servo_gimbal,330.0f);
            gimbal_at_work = 0; // 云台回位
        }
        servo_trigger_lock = 0;

    }

    if (roll_stick_val > 0.5f && servo_trigger_lock == 1)
    {
        if (claw_at_work)
        {
            Servo_SetAngle(&servo_claw, 180.0f);
            claw_at_work = 0;
        }
        else if (!claw_at_work)
        {
           Servo_SetAngle(&servo_claw, 25.0f); 
           claw_at_work = 1;
        }
         servo_trigger_lock = 0;
    }

}

/* --- 云台/吸盘控制 --- */
void Mech_Suction_Logic(float input_val,float input_pn,float roll_val)
{
    float stick_val = apply_deadzone(2.0f*(input_val - 1000.0f)/1000.0f-1.0f, 0.1f);
    float stick_pn = apply_deadzone(2.0f*(input_pn - 1000.0f)/1000.0f-1.0f, 0.1f);
    float stick_roll = apply_deadzone(2.0f*(roll_val - 1000.0f)/1000.0f-1.0f, 0.1f);

    if (stick_val > 0.5f) // 正拨
    {
        // 目标：零点 + 90度 
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1000);
    }
    else if (stick_val < -0.5f)
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1290);
    }

    if (stick_pn > 0.5f)
    {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
    }
    else if (stick_pn < -0.5f)
    {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
    }

    //扳机保险
    if (stick_roll == 0.0f) {
        suction_state = 1;
				motor_lock=1;
    }

    if (stick_roll > 0.5f && suction_state == 1)
    {
        if (suction_flag == 1)
        {
//            Pneumatic_Suction_Ctrl(VALVE_ON);
			 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_SET);

					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
            suction_flag = 2; // 更新状态为开
        }
        else if (suction_flag == 2)
        {
//            Pneumatic_Suction_Ctrl(VALVE_OFF);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_RESET);

            suction_flag = 1;
  
        }
        suction_state = 0;
    }
		else if(stick_roll < -0.5f&&motor_lock==1)
		{HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_15);
		 motor_lock =0;
		}
}


///* 抬升机构控制逻辑 */吗
//void Mech_Lift_Logic(float input_val, float roll_val)
//{
//    static float lift_target_pos = POS_DOWN; 
//    
//    // 归一化输入并处理死区
//    float stick_lift = apply_deadzone(2.0f*(input_val - 1000.0f)/1000.0f-1.0f, 0.1f);
//    float stick_roll = apply_deadzone(2.0f*(roll_val - 1000.0f)/1000.0f-1.0f, 0.1f);

////	  if (stick_roll<0.0f) {
////        motor_lock = 1;
////    }

//    // 目标位置状态机切换
//	if(stick_roll < 0.5f)
//	{ if (stick_lift < -0.5f) {
//        lift_target_pos = POS_DOWN;
//    }
//    else if (stick_lift > 0.5f) {
//        lift_target_pos = POS_UP_One;
//    }
//	}
//	else if (stick_roll>0.5f) {
//        lift_target_pos = POS_UP_Two;
//  }

//    // 重力补偿
//    int16_t gravity_comp = 200;

//    int16_t out_motor5 = (int16_t)pid_call_2(lift_target_pos, 5, 0.003f) + gravity_comp;
//    int16_t out_motor6 = (int16_t)pid_call_2(lift_target_pos, 6, 0.003f) + gravity_comp;

//    can2_tx_table.target_current[4] = out_motor5;
//    can2_tx_table.target_current[5] = out_motor6;
//    can2_tx_table.update_flag = 1;
//}

//void Mech_Lift_SetTarget(float input_val, float roll_val)
//{

//    float stick_lift = apply_deadzone(2.0f*(input_val - 1000.0f)/1000.0f-1.0f, 0.1f);
//    float stick_roll = apply_deadzone(2.0f*(roll_val - 1000.0f)/1000.0f-1.0f, 0.1f);

//    // 目标位置状态机切换
//    if(stick_roll < 0.5f) { 
//		 if (stick_lift < -0.5f) {
//            lift_target_pos = POS_DOWN; 
//        } 
//		 else if((stick_lift>-0.5f)&&(stick_lift<0.5f))
//		 {
//			 lift_target_pos = -8192.0f*150.0f; 

//		 }
//			else if (stick_lift > 0.5f) {
//            lift_target_pos = POS_UP_One;
//        }
//    } 
//		else if (stick_roll > 0.5f) {
//        lift_target_pos = POS_UP_Two;
//    }
//}

void Mech_Lift_PID_Update()
{
    // 重力补偿
    int16_t gravity_comp = 200;

    int16_t out_motor5 = (int16_t)pid_call_2(lift_target_pos, 5, 0.003f) + gravity_comp;
    int16_t out_motor6 = (int16_t)pid_call_2(lift_target_pos, 6, 0.003f) + 200;

    can2_tx_table.target_current[4] = out_motor5;
    can2_tx_table.target_current[5] = out_motor6;
    can2_tx_table.update_flag = 1;  // 让 CAN 发送任务把新值发出去
}

void Mech_Lift_SetTarget(float input_val, float roll_val)
{
    static uint8_t lift_initialized = 0;
    static uint8_t last_stick_active = 0;

    float stick_lift = apply_deadzone(2.0f*(input_val - 1000.0f)/1000.0f-1.0f, 0.1f);
    float stick_roll = apply_deadzone(2.0f*(roll_val - 1000.0f)/1000.0f-1.0f, 0.1f);

    float fb_pos = 0.5f * (motor_can2[4].total_angle + motor_can2[5].total_angle);

    /* 上电初始化 */
    if (lift_initialized == 0 && motor_can2[4].msg_cnt > 50 && motor_can2[5].msg_cnt > 50)
    {
        lift_target_pos = fb_pos;
        lift_initialized = 1;
        last_stick_active = 0;
    }

	  float direction = 0.0f;
    if (stick_roll < -0.5f) direction = -1.0f;      // 正转
    else if (stick_roll > 0.5f) direction = 1.0f;     // 反转
   
		if (lift_initialized) 
    {
        const float MAX_SPEED = 6000.0f; 
        uint8_t stick_active = (stick_lift > 0.02f);

        if (stick_active&& (direction != 0.0f))
        {
            lift_target_pos += stick_lift * direction * MAX_SPEED;
            last_stick_active = 1;
        }
        else
        {
            /* 松手瞬间，把目标钉在当前位置，避免继续追远目标 */
            if (last_stick_active)
            {
                lift_target_pos = fb_pos;

                last_stick_active = 0;
            }
        }
//				if (lift_target_pos < 0.0f) lift_target_pos = 0.0f;
//        if (lift_target_pos > 7000.0f) lift_target_pos = 7000.0f;
    }
}
