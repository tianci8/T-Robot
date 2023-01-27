/*
 * @Author: Tianci Zhang
 * @Email: tianci_zhang@tju.edu.cn
 * @Date: 2022-11-15 17:25:02
 * @LastEditors: Tianci Zhang
 * @LastEditTime: 2023-01-26 18:19:13
 * @FilePath: \TRobot_firmware\lib\imu\imu_interface.h
 * @Description:
 *
 * Copyright (c) 2022 by Tianci Zhang, All Rights Reserved.
 */

#ifndef IMU_INTERFACE
#define IMU_INTERFACE

#include <sensor_msgs/msg/imu.h>

class IMUInterface
{
protected:
    sensor_msgs__msg__Imu imu_msg_;
    float accel_cov_ = 0.00001;
    float gyro_cov_ = 0.00001;

public:
    IMUInterface()
    {
        // imu_msg_.header.frame_id = micro_ros_string_utilities_set(imu_msg_.header.frame_id, "imu_link"); //micro_ros foxy版本不可用，高版本可用
        // imu_msg_.header.frame_id.data="imu_link";  //使用该语句代替。编译有warning，建议后续使用高版本micro_ros

        imu_msg_.header.frame_id.data = (char *)malloc(100 * sizeof(char));
        char string1[] = "imu_link";
        memcpy(imu_msg_.header.frame_id.data, string1, strlen(string1) + 1);
        imu_msg_.header.frame_id.size = strlen(imu_msg_.header.frame_id.data);
        imu_msg_.header.frame_id.capacity = 100;
    }

    virtual geometry_msgs__msg__Vector3 readAccelerometer() = 0;
    virtual geometry_msgs__msg__Vector3 readGyroscope() = 0;
    virtual geometry_msgs__msg__Quaternion getQuaternion() = 0;
    virtual bool startSensor() = 0;
    virtual void calibrateGyro() = 0;

    bool init()
    {
        //检测imu是否连接，然后进行校准
        bool sensor_ok = startSensor();
        if (sensor_ok)
            calibrateGyro();
        return sensor_ok;
    }

    sensor_msgs__msg__Imu getData()
    {
        imu_msg_.angular_velocity = readGyroscope();
        if (imu_msg_.angular_velocity.x > -0.01 && imu_msg_.angular_velocity.x < 0.01)
            imu_msg_.angular_velocity.x = 0;

        if (imu_msg_.angular_velocity.y > -0.01 && imu_msg_.angular_velocity.y < 0.01)
            imu_msg_.angular_velocity.y = 0;

        if (imu_msg_.angular_velocity.z > -0.01 && imu_msg_.angular_velocity.z < 0.01)
            imu_msg_.angular_velocity.z = 0;

        imu_msg_.angular_velocity_covariance[0] = gyro_cov_;
        imu_msg_.angular_velocity_covariance[4] = gyro_cov_;
        imu_msg_.angular_velocity_covariance[8] = gyro_cov_;

        imu_msg_.linear_acceleration = readAccelerometer();
        imu_msg_.linear_acceleration_covariance[0] = accel_cov_;
        imu_msg_.linear_acceleration_covariance[4] = accel_cov_;
        imu_msg_.linear_acceleration_covariance[8] = accel_cov_;

        imu_msg_.orientation = getQuaternion();

        return imu_msg_;
    }
};

#endif
