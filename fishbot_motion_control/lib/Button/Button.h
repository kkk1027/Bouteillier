#ifndef __BUTTON_H__
#define __BUTTON_H__

#include <Arduino.h>

class Button {
public:
    Button()=default;
    void init(uint8_t pin);              // 初始化舵机
    bool ifPress();        // 设置舵机角度 (0-180度)
    
private:
    int8_t button_pin=15;
    bool status=false;
};

#endif