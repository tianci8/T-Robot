/*
 * @Author: Tianci Zhang
 * @Email: tianci_zhang@tju.edu.cn
 * @Date: 2022-11-15 10:56:10
 * @LastEditors: Tianci Zhang
 * @LastEditTime: 2023-01-26 18:20:34
 * @FilePath: \TRobot_firmware\lib\odometry\odometry.cpp
 * @Description:
 *
 * Copyright (c) 2022 by Tianci Zhang, All Rights Reserved.
 */

#include "odometry.h"

Odometry::Odometry() : x_pos_(0.0),
                       y_pos_(0.0),
                       heading_(0.0)
{
    // odom_msg_.header.frame_id = micro_ros_string_utilities_set(odom_msg_.header.frame_id, "odom");
    // odom_msg_.child_frame_id = micro_ros_string_utilities_set(odom_msg_.child_frame_id, "base_footprint");
    // odom_msg_.header.frame_id.data = "odom"; // micro_ros foxy版本写法，其他高版本使用上面两行
    // odom_msg_.child_frame_id.data = "base_footprint";

    odom_msg_.header.frame_id.data = (char *)malloc(100 * sizeof(char));
    char string3[] = "odom";
    memcpy(odom_msg_.header.frame_id.data, string3, strlen(string3) + 1);
    odom_msg_.header.frame_id.size = strlen(odom_msg_.header.frame_id.data);
    odom_msg_.header.frame_id.capacity = 100;

    odom_msg_.child_frame_id.data = (char *)malloc(100 * sizeof(char));
    char string2[] = "base_footprint"; // base_footprint
    memcpy(odom_msg_.child_frame_id.data, string2, strlen(string2) + 1);
    odom_msg_.child_frame_id.size = strlen(odom_msg_.child_frame_id.data);
    odom_msg_.child_frame_id.capacity = 100;
}

void Odometry::update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z)
{
    float delta_heading = angular_vel_z * vel_dt; // radians
    float cos_h = cos(heading_);
    float sin_h = sin(heading_);
    float delta_x = (linear_vel_x * cos_h - linear_vel_y * sin_h) * vel_dt; // m
    float delta_y = (linear_vel_x * sin_h + linear_vel_y * cos_h) * vel_dt; // m

    // calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    // calculate robot's heading in quaternion angle
    // ROS has a function to calculate yaw in quaternion angle
    float q[4];
    euler_to_quat(0, 0, heading_, q);

    // robot's position in x,y, and z
    odom_msg_.pose.pose.position.x = x_pos_;
    odom_msg_.pose.pose.position.y = y_pos_;
    odom_msg_.pose.pose.position.z = 0.0;

    // robot's heading in quaternion
    odom_msg_.pose.pose.orientation.x = (double)q[1];
    odom_msg_.pose.pose.orientation.y = (double)q[2];
    odom_msg_.pose.pose.orientation.z = (double)q[3];
    odom_msg_.pose.pose.orientation.w = (double)q[0];

    odom_msg_.pose.covariance[0] = 0.001;
    odom_msg_.pose.covariance[7] = 0.001;
    odom_msg_.pose.covariance[35] = 0.001;

    // linear speed from encoders
    odom_msg_.twist.twist.linear.x = linear_vel_x;
    odom_msg_.twist.twist.linear.y = linear_vel_y;
    odom_msg_.twist.twist.linear.z = 0.0;

    // angular speed from encoders
    odom_msg_.twist.twist.angular.x = 0.0;
    odom_msg_.twist.twist.angular.y = 0.0;
    odom_msg_.twist.twist.angular.z = angular_vel_z;

    odom_msg_.twist.covariance[0] = 0.0001;
    odom_msg_.twist.covariance[7] = 0.0001;
    odom_msg_.twist.covariance[35] = 0.0001;
}

nav_msgs__msg__Odometry Odometry::getData()
{
    return odom_msg_;
}

const void Odometry::euler_to_quat(float roll, float pitch, float yaw, float *q)
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;
}