/*
 * @Author: Tianci Zhang
 * @Email: tianci_zhang@tju.edu.cn
 * @Date: 2022-11-16 16:27:10
 * @LastEditors: Tianci Zhang
 * @LastEditTime: 2023-01-26 18:15:20
 * @FilePath: \TRobot_firmware\lib\config\base_config.h
 * @Description: 
 * 
 * Copyright (c) 2022 by Tianci Zhang, All Rights Reserved. 
 */

#ifndef BASE_CONFIG_H
#define BASE_CONFIG_H

#define LED_PIN 33 //LED pin
// PID参数  自行修改
#define K_P 1.6
#define K_I 0.2
#define K_D 0.5

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)  
         BACK
*/

//robot的一些参数
#define MOTOR_MAX_RPM 230                   // motor's max RPM          
#define MAX_RPM_RATIO 0.5                  // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
#define MOTOR_OPERATING_VOLTAGE 12          // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 12          // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 12     // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV1 (358*4)              // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 (358*4)              // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 (358*4)             // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 (358*4)             // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.048                // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.1105           // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.112   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN

// INVERT ENCODER COUNTS
#define MOTOR1_ENCODER_INV false 
#define MOTOR2_ENCODER_INV false 
#define MOTOR3_ENCODER_INV false 
#define MOTOR4_ENCODER_INV false 

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV false
#define MOTOR2_INV false
#define MOTOR3_INV false
#define MOTOR4_INV false

//esp32 dev默认I2C SDA SCL为21 22，注意避开这两个引脚。不同系列芯片默认引脚不同，按需修改
// ENCODER PINS
#define MOTOR1_ENCODER_A 27
#define MOTOR1_ENCODER_B 23

#define MOTOR2_ENCODER_A 39
#define MOTOR2_ENCODER_B 36

#define MOTOR3_ENCODER_A 5
#define MOTOR3_ENCODER_B 4

#define MOTOR4_ENCODER_A 25
#define MOTOR4_ENCODER_B 26

// MOTOR PINS
#define MOTOR1_IN_A 17
#define MOTOR1_IN_B 16

#define MOTOR2_IN_A 14
#define MOTOR2_IN_B 15

#define MOTOR3_IN_A 19
#define MOTOR3_IN_B 18

#define MOTOR4_IN_A 12
#define MOTOR4_IN_B 13

#define PWM_MAX (255-PWMOFFSET)
#define PWM_MIN -PWM_MAX
#define PWMOFFSET 49

#endif
