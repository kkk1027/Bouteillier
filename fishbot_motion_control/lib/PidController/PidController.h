#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

class PidController
{
    public:
        PidController()=default;
        PidController(float kp,float ki,float kd);
    private:
        float target_;
        float kp_;
        float ki_;
        float kd_;
        float out_min_;
        float out_max_;

        float err_;
        float err_sum_;
        float derr_;
        float prev_err_;
        float integral_up_=2500;
    public:
        float update(float current); 
        void update_target(float target);
        void update_pid(float kp,float ki,float kd);    
        void reset();
        void clear_error();   // 清零误差与积分（保留 PID 增益和输出限幅）
        void out_limit(float min ,float max);
};

#endif
