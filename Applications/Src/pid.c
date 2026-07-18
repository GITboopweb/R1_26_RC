#include "pid.h"
#include "tool_calc.h"
#include <math.h>
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_init(pid_type_def *pid, uint8_t mode, const float PID[4], float max_out, float max_iout)
{
    if (pid == 0 || PID == 0)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->K_ff=PID[3];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->max_plan_vel=35000.0f;
    pid->max_plan_acc=200000.0f;// approx 0.15s from stop to max_plan_vel

    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
    pid->prev_fdb_init=0;
    pid->I_separation_limit = 250.0f;
    pid->variable_I_enable = 1.0f;
}

float PID_calc(pid_type_def *pid, float ref, float set,float dt)
{
    if (pid == 0)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

    // �������壺��������Χ��һ��
    const float half_range = 4096.0f;
    const float full_range = 8192.0f;

    if (!pid->prev_fdb_init)
   {
    pid->prev_fdb = ref;
    pid->prev_set = set;
    pid->planned_pos = ref;
    pid->planned_vel = 0.0f;
    pid->prev_fdb_init = 1;
   }    
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
      if (fabs(pid->error[0]) < 300.0f)
      { pid->Iout += pid->Ki * pid->error[0];
        pid->Iout = LIMIT(pid->Iout, -pid->max_iout, pid->max_iout);
      } 
      else 
        {pid->Iout=0.0f;}
      
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
      
        float unsat_out = pid->Pout + pid->Iout + pid->Dout;
        pid->out = LIMIT(unsat_out, -pid->max_out, pid->max_out);
      
    }
    else if(pid->mode == PID_VARY_INT)
    {
        /* ���ٻ��� PID */
        pid->Pout = pid->Kp * pid->error[0];

        {
            float err_abs = fabsf(pid->error[0]);
            float scale = 1.0f / (1.0f+ err_abs / PID_VINT_SCALE_FACTOR);
            pid->Iout += (pid->Ki * pid->error[0]) * scale;
        }

        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];

        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    
    else if(pid->mode == PID_SUCTION)
    {
      if (pid->error[0] > half_range) pid->error[0] -= full_range;
      else if (pid->error[0] < -half_range) pid->error[0] += full_range;
        
        pid->Pout = pid->Kp * pid->error[0];
      if (fabs(pid->error[0]) < 300.0f)
      { pid->Iout += pid->Ki * pid->error[0];
        pid->Iout = LIMIT(pid->Iout, -pid->max_iout, pid->max_iout);
      }
      else 
       { pid->Iout=0.0f;}
      
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
      
        float unsat_out = pid->Pout + pid->Iout + pid->Dout;
        pid->out = LIMIT(unsat_out, -pid->max_out, pid->max_out);
    }

    else if(pid->mode==PID_VARY_D_ON_INCOM_D)
  {
    //  �������ι滮λ��
    update_trapezoidal_planner(pid, set, dt, half_range, full_range);

    // ���滮λ����Ϊ PID ���趨��
    pid->set = pid->planned_pos;

    //  ����������
    float unsat_out = compute_pid_output(pid, ref, dt, half_range, full_range);
    pid->out = LIMIT(unsat_out, -pid->max_out, pid->max_out);

    //  ���淴����ʷ
    pid->prev_fdb = ref;

  } 
  return pid->out;
}


void PID_clear(pid_type_def *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}



