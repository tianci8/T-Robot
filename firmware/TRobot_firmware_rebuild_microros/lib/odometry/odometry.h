/*
 * @Author: Tianci Zhang
 * @Email: tianci_zhang@tju.edu.cn
 * @Date: 2022-11-15 10:56:10
 * @LastEditors: Tianci Zhang
 * @LastEditTime: 2023-01-26 18:20:59
 * @FilePath: \TRobot_firmware\lib\odometry\odometry.h
 * @Description:
 *
 * Copyright (c) 2022 by Tianci Zhang, All Rights Reserved.
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include <nav_msgs/msg/odometry.h>

class Odometry
{
public:
    Odometry();
    void update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z);
    nav_msgs__msg__Odometry getData();

private:
    const void euler_to_quat(float x, float y, float z, float *q);

    nav_msgs__msg__Odometry odom_msg_;
    float x_pos_;
    float y_pos_;
    float heading_;
};

#endif