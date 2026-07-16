#include "PidController.h"
#include "Arduino.h"

PidController::PidController(float kp,float ki,float kd)
{
    kp_=kp;
    ki_=ki;
    kd_=kd;
}
        // float update(float current); 
        // void update_target(float target);
        // void update_pid(float kp,float ki,float kd);    
        // void reset();
        // void out_limit(float);
float PidController::update(float current)
{
    err_=target_-current;
    derr_=prev_err_-err_;
    prev_err_=err_;

    // 计算预钳位输出，判断是否饱和
    float output_pre = kp_*err_ + ki_*err_sum_ + kd_*derr_;

    // 抗积分饱和：输出饱和且误差仍在推离方向时，停止积分累积
    bool sat_high = (output_pre > out_max_ && err_ > 0);
    bool sat_low  = (output_pre < out_min_ && err_ < 0);
    if (!sat_high && !sat_low) {
        err_sum_ += err_;
    }
    if(err_sum_>integral_up_)err_sum_=integral_up_;
    if(err_sum_<-1*integral_up_)err_sum_=-1*integral_up_;

    // 用更新后的积分重新计算输出
    float output = kp_*err_ + ki_*err_sum_ + kd_*derr_;
    if(output>out_max_)output=out_max_;
    if(output<out_min_)output=out_min_;

    return output;
}
void PidController::update_target(float target)
{
    target_=target;
}
void PidController::update_pid(float kp,float ki,float kd)
{
    kp_=kp;
    ki_=ki;
    kd_=kd;

}
void PidController::reset()
{
    err_=0;
    derr_=0;
    err_sum_=0;
    prev_err_=0;
    kp_=0;
    ki_=0;
    kd_=0;
    integral_up_=2500;
    out_max_=0;
    out_min_=0;

}
void PidController::clear_error()
{
    // 仅清零误差累积项，保留 PID 增益和输出限幅不变
    err_=0;
    derr_=0;
    err_sum_=0;
    prev_err_=0;
}
void PidController::out_limit(float min ,float max)
{
    out_max_=max;
    out_min_=min;
}
