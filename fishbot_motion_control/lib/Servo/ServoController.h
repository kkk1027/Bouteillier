#ifndef __SERVO_CONTROLLER_H__
#define __SERVO_CONTROLLER_H__

#include <Arduino.h>

class ServoController {
public:
    ServoController();
    void init(uint8_t pin);              // 初始化舵机
    void setAngle(int angle);        // 设置舵机角度 (0-180度)
    void setTimeStamp();  // 直接设置脉冲宽度 (微秒)
    void reset();                        // 复位到初始位置
    bool isMoving();                     // 检查舵机是否在运动
    
private:
    int8_t button_pin=15;
    int8_t led_pin=27;
    int8_t led_logic=0;
    uint8_t servo_pin;
    uint8_t current_angle;
    uint16_t min_pulse = 500;   // 最小脉冲宽度 (0度)
    uint16_t max_pulse = 2500;  // 最大脉冲宽度 (180度)
    unsigned long last_move_time;
    bool is_initialized = false;
    bool status=false;
    
    void writePulse(uint16_t pulse_width);
};

#endif