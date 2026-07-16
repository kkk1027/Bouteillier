#include "Kinematics.h"
void Kinematics::set_motor_param(uint8_t id, float per_pulse_distance)
{
    motor_param[id].per_pulese_distance = per_pulse_distance;
}
void Kinematics::kinematics_forward(float left_speed, float right_speed, float *out_linear_speed, float *out_angular_speed)
{
    *out_linear_speed = (left_speed + right_speed) / 2;
    *out_angular_speed = (right_speed - left_speed) / wheel_distance;
}
void Kinematics::kinematics_inverse(float linear_speed, float angular_speed, float *out_left_speed, float *out_right_speed)
{
    *out_left_speed = linear_speed - angular_speed * wheel_distance / 2;
    *out_right_speed = linear_speed + angular_speed * wheel_distance / 2;
}
void Kinematics::update_motor_speed(uint64_t current_time, int32_t left_tick, int32_t right_tick)
{
    int16_t delta_ticks[2] = {0, 0};

    int16_t dt = millis() - last_update_time;
    delta_ticks[0] = left_tick - motor_param[0].last_encoder_ticks;
    delta_ticks[1] = right_tick - motor_param[1].last_encoder_ticks;

    motor_param[0].motor_speed = (delta_ticks[0] * 105.805) / dt;
    motor_param[1].motor_speed = (delta_ticks[1] * 105.805) / dt;

    motor_param[0].last_encoder_ticks = left_tick;
    motor_param[1].last_encoder_ticks = right_tick;
    last_update_time = current_time;

    update_odom(dt);
}
int16_t Kinematics::get_motor_speed(uint8_t id)
{
    if (id < 0 || id > 1)
    {
        return -1;
    }
    return motor_param[id].motor_speed;
}
void Kinematics::set_wheel_distance(float distance)
{
    wheel_distance = distance;
}
void Kinematics::update_odom(uint16_t dt)
{
    float dt_s = float(dt)/1000.0;
    this->kinematics_forward(motor_param[0].motor_speed,motor_param[1].motor_speed,&odom.linear_speed,&odom.angular_speed);
    odom.linear_speed=odom.linear_speed/1000.0;
    odom.angle+=odom.angular_speed*dt_s;
    TransAngleInPi(odom.angle,odom.angle);
    float delta_distance=odom.linear_speed*dt_s;
    odom.x+=delta_distance*std::cos(odom.angle);
    odom.y+=delta_distance*std::sin(odom.angle);

}
odom_t &Kinematics::get_odom()
{
    return odom;
}
void Kinematics::TransAngleInPi(float angle, float &out_angle)
{
    if(angle>PI)
    {
        out_angle-=2*PI;
    }else if(angle<-PI)
    {
        out_angle+=2*PI;
    }
}
