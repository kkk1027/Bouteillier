// ServoController.cpp
#include "ServoController.h"
#include <Arduino.h>
#define FREQ 50
#define CHANNEL 0
#define RESOLUTION 8


ServoController::ServoController() {
    current_angle = 90; // 默认中间位置
    last_move_time = 0;
    is_initialized = false;
}

void ServoController::init(uint8_t pin) {
    servo_pin = pin;
    is_initialized = true;
    ledcSetup(CHANNEL,FREQ,RESOLUTION);
    ledcAttachPin(servo_pin,CHANNEL);

    // 初始化到中间位置
    reset();
    Serial.printf("舵机初始化完成，引脚: %d\n", pin);
}

void ServoController::setAngle(int angle) {
    if (!is_initialized) return;
    
    // 限制角度范围
    // if (angle > 180) angle = 180;
    
    int min_angle=0.5/20*pow(2,RESOLUTION);
    int max_angle=2.5/20*pow(2,RESOLUTION);

    // 计算脉冲宽度
    int pulse_width = (max_angle-min_angle)*angle/180 +min_angle;
    // if (pulse_width < min_pulse) pulse_width = min_pulse;
    // if (pulse_width > max_pulse) pulse_width = max_pulse;

    ledcWrite(CHANNEL,pulse_width);    
    setTimeStamp();

    Serial.printf("舵机设置角度: %d度, 脉冲: %dus\n", angle, pulse_width);
}

void ServoController::setTimeStamp() {
    if (!is_initialized) return;
        
    last_move_time = millis();
}

void ServoController::reset() {
    setAngle(0); // 复位到中间位置
    Serial.println("舵机复位到90度");
}

bool ServoController::isMoving() {
    // 检查是否在运动 (最近500ms内移动过)
    return (millis() - last_move_time) < 500;
}

