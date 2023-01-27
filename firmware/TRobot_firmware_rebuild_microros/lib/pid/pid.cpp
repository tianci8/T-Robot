/*
 * @Author: Tianci Zhang
 * @Email: tianci_zhang@tju.edu.cn
 * @Date: 2022-11-19 20:37:15
 * @LastEditors: Tianci Zhang
 * @LastEditTime: 2023-01-26 18:20:54
 * @FilePath: \TRobot_firmware\lib\pid\pid.cpp
 * @Description:
 *
 * Copyright (c) 2022 by tianci_zhang@tju.edu.cn, All Rights Reserved.
 */

#include "Arduino.h"
#include "pid.h"

PID::PID(float min_val, float max_val, float kp, float ki, float kd) : min_val_(min_val),
                                                                       max_val_(max_val),
                                                                       kp_(kp),
                                                                       ki_(ki),
                                                                       kd_(kd)
{
}

double PID::compute(float setpoint, float measured_value)
{
    double error;
    double pid;

    // setpoint is constrained between min and max to prevent pid from having too much error
    error = setpoint - measured_value;
    integral_ += error;
    derivative_ = error - prev_error_;

    if (setpoint == 0 && error == 0)
    {
        integral_ = 0;
        derivative_ = 0;
    }

    pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
    prev_error_ = error;

    return constrain(pid, min_val_, max_val_);
}

void PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}
