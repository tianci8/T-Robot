/*
 * @Author: Tianci Zhang
 * @Email: tianci_zhang@tju.edu.cn
 * @Date: 2022-11-15 17:27:36
 * @LastEditors: Tianci Zhang
 * @LastEditTime: 2023-01-26 18:16:46
 * @FilePath: \TRobot_firmware\lib\imu\default_imu.h
 * @Description:
 *
 * Copyright (c) 2022 by Tianci Zhang, All Rights Reserved.
 */

#ifndef DEFAULT_IMU
#define DEFAULT_IMU

// include IMU base interface
#include "imu_interface.h"

// include sensor API headers
#include "MPU9250.h"

class MPU9250IMU : public IMUInterface
{
private:
    MPU9250 mpu;

    geometry_msgs__msg__Vector3 accel_;
    geometry_msgs__msg__Vector3 gyro_;
    geometry_msgs__msg__Quaternion quat_;

public:
    MPU9250IMU()
    {
    }

    bool startSensor() override
    {
        return mpu.setup(0x68);
    }

    void calibrateGyro()
    {
        mpu.calibrateAccelGyro();
        delay(1000);
        mpu.calibrateMag();
    }

    void update()
    {
        mpu.update();
    }

    geometry_msgs__msg__Vector3 readAccelerometer() override
    {
        // mpu.update();
        accel_.x = (double)(mpu.getAccX());
        accel_.y = (double)(mpu.getAccY());
        accel_.z = (double)(mpu.getAccZ());
        return accel_;
    }

    geometry_msgs__msg__Vector3 readGyroscope() override
    {
        // mpu.update();
        gyro_.x = (double)(mpu.getGyroX());
        gyro_.y = (double)(mpu.getGyroY());
        gyro_.z = (double)(mpu.getGyroZ());
        return gyro_;
    }

    geometry_msgs__msg__Quaternion getQuaternion() override
    {
        quat_.w = (double)(mpu.getQuaternionW());
        quat_.x = (double)(mpu.getQuaternionX());
        quat_.y = (double)(mpu.getQuaternionY());
        quat_.z = (double)(mpu.getQuaternionZ());
        return quat_;
    }
};

#endif