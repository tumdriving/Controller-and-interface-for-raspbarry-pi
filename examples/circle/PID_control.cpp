#include <iostream>
#include <cmath>
#include <chrono>
#include "PID_control.h"
#include "rec/robotino/api2/all.h"

float PID::calculate(float target, float current, float dt)
{
    float error = target - current;
    integral += error * dt;
    float derivative = (error - previous_error) / dt;
    previous_error = error;
    return kp * error + ki * integral + kd * derivative;
}

void State::update(float vx, float vy, float omega, float dt) {
    float ntheta_rad = rec::robotino::api2::deg2rad(this->theta + omega * dt);
    float theta_rad = rec::robotino::api2::deg2rad(this->theta);
    if (omega != 0) {
        float R = vy / omega;
        this->x += R * (sinf(ntheta_rad) - sinf(theta_rad));
        this->y -= R * (cosf(ntheta_rad) - cosf(theta_rad));
    } else {
        this->x += vx * dt * cosf(ntheta_rad);
        this->y += vy * dt * sinf(ntheta_rad);
    }
    this->theta += omega * dt;
    std::cout << "x: " << this->x << " y: " << this->y << " vel: " << vy << std::endl;
}

void State::getCurState(float &x, float &y, float &theta) {
    x = this->x;
    y = this->y;
    theta = this->theta;
}

// 获取当前时间（毫秒）
float getCurrentTime()
{
    return std::chrono::duration<float, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
}