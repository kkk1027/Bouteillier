#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__
#include "Arduino.h"
typedef struct
{
    float per_pulese_distance;
    int16_t motor_speed;
    int64_t last_encoder_ticks;
} motor_param_t;
typedef struct
{
    float x;
    float y;
    float angular_speed;
    float linear_speed;
    float angle;
} odom_t;

class Kinematics
{
private:
    motor_param_t motor_param[2];
    odom_t odom;
    uint64_t last_update_time = 0;
    float wheel_distance = 0.0;
    

public:
    Kinematics(/* args */) = default;
    ~Kinematics() = default;
    void set_motor_param(uint8_t id, float per_pulse_distance);
    void kinematics_forward(float left_speed, float right_speed, float *out_linear_speed, float *out_angular_speed);
    void kinematics_inverse(float left_speed, float right_speed, float *out_linear_speed, float *out_angular_speed);
    void update_odom(uint16_t id);
    odom_t& get_odom();
    void TransAngleInPi(float angle,float &out_angle);
    void update_motor_speed(uint64_t current_time, int32_t left_tick, int32_t right_tick);
    int16_t get_motor_speed(uint8_t id);
    void set_wheel_distance(float distance);
};

#endif