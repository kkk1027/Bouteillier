#include <Arduino.h>
#include <Esp32PcntEncoder.h>
#include <Esp32McpwmMotor.h>
#include <PidController.h>
#include <Kinematics.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>
#include <micro_ros_utilities/string_utilities.h>
#include "std_srvs/srv/trigger.h"
#include <std_srvs/srv/set_bool.h>
#include <Button.h>
#include "esp_task_wdt.h"  
#include "DFRobotDFPlayerMini.h"
#include <mutex>
#include "Wire.h"
#include <MPU6050_light.h>
#include <std_msgs/msg/float32.h>
std::mutex motor_mutex;
DFRobotDFPlayerMini player;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_node_t node;
rcl_subscription_t sub_cmd_vel;
geometry_msgs__msg__Twist msg_cmd_vel;
rcl_subscription_t sub_nav_status;
std_msgs__msg__Bool msg_nav_status;
rcl_publisher_t pub_odom;
nav_msgs__msg__Odometry msg_odom;
rcl_service_t command_service;
std_srvs__srv__SetBool_Request command_req;
std_srvs__srv__SetBool_Response command_res;
rcl_timer_t timer;
Esp32McpwmMotor motor;
Esp32PcntEncoder encoders[2];
Kinematics kinematics;
Button button;
PidController pid_controller[2];
bool nav_node_running = false;
unsigned long last_cmd_vel_time = 0;
float target_linear_speed = 0.0;
float target_angular_speed = 0.0;
float out_left_speed = 0.0;
float out_right_speed = 0.0;
bool motor_enabled = true;
const int batteryPin = 34;                    // ADC引脚
const float voltageDivider = 5.17;            // 分压比
rcl_publisher_t pub_battery_voltage;          // 电压发布器
rcl_publisher_t pub_battery_percent;          // 电量百分比发布器
std_msgs__msg__Float32 msg_battery_voltage;
std_msgs__msg__Float32 msg_battery_percent;
bool need_restart = false;
unsigned long restart_request_time = 0;
// 低电量播报相关
bool low_battery_alerted = false;      // 是否已播报过低电量
const float low_battery_threshold = 11.4;  // 低电量阈值（20%电量对应的电压）
const float critical_battery_threshold = 11.1;  // 临界电量阈值（0%）
// ==================== IMU 新增变量 ====================
MPU6050 mpu(Wire);
rcl_publisher_t pub_imu;
sensor_msgs__msg__Imu msg_imu;
float getBatteryVoltage();
int voltageToPercent(float voltage);
// ==================== IMU 新增函数 ====================
float getBatteryVoltage() {
    // 读取前停止电机更新（可选）
    std::lock_guard<std::mutex> lock(motor_mutex);
    
    const int numSamples = 20;  // 增加到20次采样
    long sum = 0;
    
    // 先等待一下，让电压稳定
    delayMicroseconds(500);
    
    for (int i = 0; i < numSamples; i++) {
        sum += analogRead(batteryPin);
        delayMicroseconds(200);  // 增加采样间隔
    }
    int adcValue = sum / numSamples;
    float pinVoltage = (adcValue / 4095.0) * 3.3;
    return pinVoltage * voltageDivider;
}
int voltageToPercent(float voltage) {
    // 3串锂电池：满电12.6V，截止11.1V
    if (voltage >= 12.5) return 100;
    if (voltage <= 11.1) return 0;
    // 线性插值
    return (int)((voltage - 11.1) / (12.5 - 11.1) * 100);
}
void publish_imu_data()
{
    static unsigned long last_update = 0;
    unsigned long now = millis();
    
    if (now - last_update >= 10) {
        mpu.update();
        last_update = now;
        
        int64_t stamp = rmw_uros_epoch_millis();
        msg_imu.header.stamp.sec = static_cast<int32_t>(stamp / 1000);
        msg_imu.header.stamp.nanosec = static_cast<int32_t>((stamp % 1000) * 1e6);
        msg_imu.header.frame_id = micro_ros_string_utilities_set(msg_imu.header.frame_id, "imu_link");
        
        float roll = mpu.getAngleX() * M_PI / 180.0;
        float pitch = mpu.getAngleY() * M_PI / 180.0;
        float yaw = mpu.getAngleZ() * M_PI / 180.0;
        
        msg_imu.orientation.x = 0;
        msg_imu.orientation.y = 0;
        msg_imu.orientation.z = sin(yaw / 2.0);
        msg_imu.orientation.w = cos(yaw / 2.0);
        
        for (int i = 0; i < 9; i++) {
            msg_imu.orientation_covariance[i] = 0.0;
            msg_imu.angular_velocity_covariance[i] = 0.0;
            msg_imu.linear_acceleration_covariance[i] = 0.0;
        }
        msg_imu.angular_velocity_covariance[0] = 0.01;
        msg_imu.angular_velocity_covariance[4] = 0.01;
        msg_imu.angular_velocity_covariance[8] = 0.01;
        
        msg_imu.angular_velocity.x = mpu.getGyroX() * M_PI / 180.0;
        msg_imu.angular_velocity.y = mpu.getGyroY() * M_PI / 180.0;
        msg_imu.angular_velocity.z = mpu.getGyroZ() * M_PI / 180.0;
        
        msg_imu.linear_acceleration.x = mpu.getAccX();
        msg_imu.linear_acceleration.y = mpu.getAccY();
        msg_imu.linear_acceleration.z = mpu.getAccZ();
        
        if (rcl_publish(&pub_imu, &msg_imu, NULL) != RCL_RET_OK) {
            Serial.println("err: imu pub failed!");
        }
    }
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    odom_t odom = kinematics.get_odom();
    int64_t stamp = rmw_uros_epoch_millis();
    msg_odom.header.stamp.sec = static_cast<int32_t>(stamp / 1000);
    msg_odom.header.stamp.nanosec = static_cast<int32_t>((stamp % 1000) * 1e6);
    msg_odom.pose.pose.position.x = odom.x;
    msg_odom.pose.pose.position.y = odom.y;
    msg_odom.pose.pose.orientation.w = cos(odom.angle * 0.5);
    msg_odom.pose.pose.orientation.x = 0;
    msg_odom.pose.pose.orientation.y = 0;
    msg_odom.pose.pose.orientation.z = sin(odom.angle * 0.5);
    msg_odom.twist.twist.linear.x = odom.linear_speed;
    msg_odom.twist.twist.angular.z = odom.angular_speed;
    if (rcl_publish(&pub_odom, &msg_odom, NULL) != RCL_RET_OK)
    {
        Serial.println("err:odom pub failed!");
    }
    
    // 新增：发布 IMU 数据
    publish_imu_data();
    // 电池电量发布（改为每5分钟发布一次）
    static unsigned long last_battery_pub = 0;
    unsigned long now = millis();
    if (now - last_battery_pub >= 60000) {
        float voltage = getBatteryVoltage();
        int percent = voltageToPercent(voltage);
        
        msg_battery_voltage.data = voltage;
        msg_battery_percent.data = (float)percent;
        
        rcl_publish(&pub_battery_voltage, &msg_battery_voltage, NULL);
        rcl_publish(&pub_battery_percent, &msg_battery_percent, NULL);
        
        Serial.printf("电池电压: %.2fV, 电量: %d%%\n", voltage, percent);
        // ========== 低电量语音播报 ==========
        // 低电量播报（电压低于阈值且未播报过）
        if (voltage <= low_battery_threshold && !low_battery_alerted) {
            low_battery_alerted = true;
            player.play(7);  // 播放低电量语音
            Serial.println("低电量警告！播放语音提示");
        }
        // 回到正常电压后重置标志，允许下次再播报
        else if (voltage > low_battery_threshold + 0.3) {  // 加0.3V滞回，避免反复触发
            low_battery_alerted = false;
        }
        last_battery_pub = now;
    }
}

void twist_callback(const void* msg_in)
{
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msg_in;
    last_cmd_vel_time = millis();
    if (!motor_enabled) {
        motor_enabled = true;
        Serial.println("收到twist指令，启用电机");
    }
    target_linear_speed = msg->linear.x * 1000;
    target_angular_speed = msg->angular.z;
    kinematics.kinematics_inverse(target_linear_speed, target_angular_speed, &out_left_speed, &out_right_speed);
    Serial.printf("OUT:left_speed=%f,right_speed=%f\n", out_left_speed, out_right_speed);
    pid_controller[0].update_target(out_left_speed);
    pid_controller[1].update_target(out_right_speed);
}
void nav_node_status_callback(const void* msg_in) {
    const std_msgs__msg__Bool* msg = (const std_msgs__msg__Bool*)msg_in;
    nav_node_running = msg->data;
    Serial.printf("导航节点状态更新: %s\n", nav_node_running ? "运行中" : "已关闭");
    
    if (!nav_node_running) {
        last_cmd_vel_time = millis();  
    }
}

void command_callback(const void* req, void* res) {
    std_srvs__srv__SetBool_Request* request = 
        (std_srvs__srv__SetBool_Request*)req;
    std_srvs__srv__SetBool_Response* response = 
        (std_srvs__srv__SetBool_Response*)res;
    
    if (request->data == true) {  // true = 重启
        Serial.println("收到重启命令，1秒后执行...");
        
        // 先回复
        response->success = true;
        response->message.data = (char*)"ESP32 即将重启";
        response->message.size = strlen(response->message.data);
        response->message.capacity = response->message.size + 1;
        
        // 安全关闭
        motor.updateMotorSpeed(0, 0);
        motor.updateMotorSpeed(1, 0);
        
        need_restart = true;
        restart_request_time = millis();
    } 
    else {  // false = 舵机动作
        Serial.println("执行舵机动作");
        
        // ====== 关键修复：禁用电机 + 清零 PID，防止 loop() 重启用 ======
        // command_callback 在 microros_task 中运行，loop() 在 Arduino 主
        // 线程同时运行。如果不设置 motor_enabled=false，loop() 会立即用
        // PID 输出重新驱动电机（PID 目标可能来自 Nav2 最后一个非零角速度
        // 指令的积分残余），造成「到达后原地旋转」。
        // 同时清零 PID 积分项，确保下次导航从零开始。
        motor_enabled = false;
        pid_controller[0].clear_error();
        pid_controller[1].clear_error();
        pid_controller[0].update_target(0);
        pid_controller[1].update_target(0);
        target_linear_speed = 0.0;
        target_angular_speed = 0.0;
        last_cmd_vel_time = millis();
        motor.updateMotorSpeed(0, 0);
        motor.updateMotorSpeed(1, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        player.play(1);
        unsigned long start_time = millis();
        const unsigned long timeout = 60000;
        
        while (millis() - start_time < timeout) {
            if (button.ifPress()) {
                player.play(4);
                // 恢复电机使能（下次 cmd_vel 到来时 twist_callback 也会
                // 自动恢复，这里显式恢复更安全）
                motor_enabled = true;
                response->success = true;
                response->message.data = (char*)"舵机动作执行完成";
                response->message.size = strlen(response->message.data);
                response->message.capacity = response->message.size + 1;
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        // 超时也恢复电机使能
        motor_enabled = true;
        response->success = false;
        response->message.data = (char*)"舵机动作超时";
        response->message.size = strlen(response->message.data);
        response->message.capacity = response->message.size + 1;
    }
}

void microros_task(void* args)
{
    IPAddress agent_ip;
    agent_ip.fromString("192.168.2.128");
    set_microros_wifi_transports("kkk", "12345678", agent_ip, 8888);

    delay(2000);
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pid_project", "", &support);
    unsigned int num_handlers = 7;
    rclc_executor_init(&executor, &support.context, num_handlers, &allocator);
    rclc_service_init_default(&command_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), "/esp32_command");
    rclc_subscription_init_best_effort(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
    rclc_subscription_init_default(&sub_nav_status, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/nav_node_status");    
    rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &twist_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_nav_status, &msg_nav_status, &nav_node_status_callback, ON_NEW_DATA);
    // 初始化电池电量发布器
    rclc_publisher_init_default(&pub_battery_voltage, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/battery_voltage");
    rclc_publisher_init_default(&pub_battery_percent, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/battery_percent");
    rclc_executor_add_service(&executor, &command_service, &command_req, &command_res, command_callback);
    msg_odom.header.frame_id = micro_ros_string_utilities_set(msg_odom.header.frame_id, "odom");
    msg_odom.child_frame_id = micro_ros_string_utilities_set(msg_odom.child_frame_id, "base_footprint");
    rclc_publisher_init_best_effort(&pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom");
    
    // 新增：初始化 IMU 发布器
    rclc_publisher_init_best_effort(&pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu_raw");
    
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), timer_callback);
    rclc_executor_add_timer(&executor, &timer);
    
    while (!rmw_uros_epoch_synchronized())
    {
        rmw_uros_sync_session(1000);
        delay(10);
    }
    player.play(6);
    delay(1000);
    rclc_executor_spin(&executor);
}

void setup()
{
    esp_task_wdt_init(65, true);
    esp_task_wdt_add(NULL);
    
    delay(300);
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RX, TX);
    if (player.begin(Serial2)) {
        player.volume(60);     
    }
    // 初始化ADC (电池电压测量)
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    analogSetPinAttenuation(batteryPin, ADC_11db);
    // 新增：MPU6050 初始化
    Wire.begin(18, 19);
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    int retry_count = 0;
    while (status != 0 && retry_count++ < 50) {
        delay(100);
        status = mpu.begin();
    }
    if (status != 0) {
        Serial.println("MPU6050 init FAILED, continuing without IMU...");
    }
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    mpu.calcOffsets();
    Serial.println("IMU Ready!");
    
    button.init(15);
    motor.attachMotor(0, 22, 23);
    motor.attachMotor(1, 12, 13);
    encoders[0].init(0, 32, 33);
    encoders[1].init(1, 26, 25);
    last_cmd_vel_time = millis();
    pid_controller[0].update_pid(0.625, 0.125, 0.0);
    pid_controller[1].update_pid(0.710, 0.125, 0.0);
    pid_controller[0].out_limit(-100, 100);
    pid_controller[1].out_limit(-100, 100);

    kinematics.set_wheel_distance(212.3);
    kinematics.set_motor_param(0, 0.10805);
    kinematics.set_motor_param(1, 0.10805);
    
    xTaskCreate(microros_task, "microros_task", 32768, NULL, 1, NULL);
}

void loop()
{
    if (need_restart) {
        if (millis() - restart_request_time >= 500) {  // 等待500ms确保响应发送
            Serial.println("正在重启...");
            ESP.restart();
        }
        return;  // 重启前不再执行其他操作
    }
    static unsigned long last_wdt_feed = 0;
    if (millis() - last_wdt_feed >= 5000) {
        esp_task_wdt_reset();
        last_wdt_feed = millis();
    }
    delay(10);
    std::lock_guard<std::mutex> lock(motor_mutex);
    kinematics.update_motor_speed(millis(), encoders[0].getTicks(), encoders[1].getTicks());
    bool no_twist = (millis() - last_cmd_vel_time >= 3000);     
    if (motor_enabled && no_twist && !nav_node_running) {
        motor_enabled = false;
        motor.updateMotorSpeed(0, 0);
        motor.updateMotorSpeed(1, 0);
        pid_controller[0].update_target(0);
        pid_controller[1].update_target(0);
    }
    if (motor_enabled) {
        motor.updateMotorSpeed(0, pid_controller[0].update(kinematics.get_motor_speed(0)));
        motor.updateMotorSpeed(1, pid_controller[1].update(kinematics.get_motor_speed(1)));
    }
}