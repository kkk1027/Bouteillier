#include <Arduino.h>
#include <Button.h>
void Button::init(uint8_t pin)  
{
    pinMode(pin,INPUT_PULLDOWN);

}           // 初始化舵机
bool Button::ifPress()
{
    // 读取按钮状态（下拉模式：按下为HIGH，释放为LOW）
    if (digitalRead(button_pin) == HIGH)
    {
        // 消抖延迟
        delay(20);
        
        // 再次确认按下状态
        if (digitalRead(button_pin) == HIGH)
        {
            // 防止重复触发：等待按钮释放
            while(digitalRead(button_pin) == HIGH)
            {
                delay(10);
            }
            return true;
        }
    }
    return false;
}