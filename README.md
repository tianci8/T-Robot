# T-Robot

<img src="https://s2.loli.net/2023/01/27/2GZabjpWzDtgSEm.jpg" alt="T-Robot1" style="zoom:50%;" />

- T-Robot是一个DIY的移动机器人

- 全面拥抱ROS2生态，可实现2D激光雷达地图重建和自主导航

- 纯兴趣向，可作为ROS2入门参考使用

- 底层采用micro-ros，彻底拥抱ROS2生态

- 上层应用ROS2进行各个功能模块的快速开发

- 本项目借鉴和学习了以下仓库：[linorobot2](https://github.com/linorobot/linorobot2) [fishbot](https://github.com/fishros/fishbot) [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)

# 0 仓库资料说明

- images：一些照片
- firmware：底层固件
- ros2_packages：ros功能包
- 3D-models：三维模型

# 1 机器人硬件部分

**均为淘宝采购，为避免广告嫌疑，不放链接~**

- **机器人底盘布局**

<img src="https://s2.loli.net/2022/12/07/dWQGn3xtAmE46aI.png" alt="image-20221207200401126" style="zoom:50%;" />

- **主控**：**esp32** (ESP-WROOM-32)+**旭日x3派**(2GB)

- **激光雷达**：LD14

- **IMU**：MPU9250

- **电机**： WGA12-N20直流减速电机 (12V, 1:50减速比)

- **电机驱动芯片**：4颗RZ7889

- **编码器**：AB双相增量式磁性霍尔编码器

- **电源**：12V锂电池；

  - 12V驱动电机；外置12V转5V模块提供DC-5V输出(最大电流4A)；

- **轮子**：4麦轮，直径48mm

- **结构件**：切割亚克力板、3D打印

  

# 2 底层固件部分

## 2.0 固件说明

- 此固件基于Arduino框架，适用于esp32系列；不推荐使用ArduinoIDE进行编译(可能报错)；

- 推荐使用VScode+platformio，固件相关依赖可通过`platformio.ini`自动安装；

- 基于[micro_ros_arduino#v2.0.5foxy](https://github.com/micro-ROS/micro_ros_arduino#v2.0.5-foxy),可自行更改为其他版本，如`galactic`,`humble`等(可能需要极少的修改)；

- 由于`micro_ros_arduino`并不是原生支持`platformio`，因此在`platformio.ini`中需要手动指定相应硬件平台的`libmicroros.a`，使用如下命令(以esp32为例)：

  ```bash
  build_flags = 
    -L $PROJECT_DIR/lib/micro_ros_arduino/src/esp32/ -l libmicroros
  ```

  对官方micro_ros_arduino库进行了rebuild，因此不可以直接使用官方的库

- 若要使用官方micro_ros_arduino，需要更改odomertry message的Qos为default：

  - **改为default会导致发布频率降低**；

  ```c++
    //!!! 更改rclc_publisher_init_best_effort为rclc_publisher_init_default !!!
  RCCHECK(rclc_publisher_init_best_effort(
  ​    &odom_publisher,
  ​    &node,
  ​    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
  ​    "odom"));
  ```

- 使用wifi与上位机进行通讯；

- 若使用串口，确保串口波特率在480600bps以上，并修改src/main.cpp：

  ```c++
  set_microros_transports();
  //set_microros_wifi_transports("WIFI名称", "密码", "上位机IP", 8888);
  ```


## 2.1 各模块说明

### 2.1.1 lib/config

包含大部分宏定义，引脚定义等。

### 2.1.2 lib/encoder

c++封装的适用于esp32的编码器库，原作者仓库地址：[ESP32encoder](https://github.com/madhephaestus/ESP32Encoder.git)

- 本项目对其进行了必要修改，增加了`getRPM()`函数，以直接获取转速信息。

  ```c++
  float ESP32Encoder::getRPM(int counts_per_rev)  //counts_per_rev：每转一圈的编码器计数
  {
    counts_per_rev_ = counts_per_rev;
    int64_t encoder_ticks = getCount();
    // this function calculates the motor's RPM based on encoder ticks and delta time
    unsigned long current_time = micros();
    unsigned long dt = current_time - prev_update_time_;
    // convert the time from milliseconds to minutes
    double dtm = (double)dt / 60000000;
    double delta_ticks = encoder_ticks - prev_encoder_ticks_;
    // calculate wheel's speed (RPM)
    prev_update_time_ = current_time;
    prev_encoder_ticks_ = encoder_ticks;
    return ((delta_ticks / counts_per_rev_) / dtm);
  }
  ```

- 可配置实现不同的编码器计数方式：

```c++
//file: ESP32encoder.h
void attachSingleEdge(int aPintNumber, int bPinNumber); //1倍
void attachHalfQuad(int aPintNumber, int bPinNumber);  //2倍
void attachFullQuad(int aPintNumber, int bPinNumber);  //4倍
```

### 2.1.3 lib/imu

- `imu_interface.h`

  - 定义了IMU的`ros2 imu message`格式，可直接发送给ROS/ROS2使用；

  - `IMUInterface()`构造函数中初始化了`imu_msg.header`相关信息，不同版本写法略有不同；

  - 抽象了4个虚函数，方便适配不同类型的IMU；

    ```c++
    //file: imu_interface.h
    virtual geometry_msgs__msg__Vector3 readAccelerometer() = 0;
    virtual geometry_msgs__msg__Vector3 readGyroscope() = 0;
    virtual bool startSensor() = 0;
    virtual void calibrateGyro()=0;
    ```

- `default_imu.h`

  - 依赖[MPU9250库](https://github.com/hideakitai/MPU9250.git)，该库可通过platformio.ini文件进行自动安装，也可以手动下载添加；
  - 对`imu_interface.h`中的4个虚函数进行了实现；

### 	2.1.4 lib/kinematics

c++封装的适用于求解小车底盘正逆运动学的库。适用于两轮差分、阿克曼、麦轮等。该库原地址：[kinematics](https://github.com/linorobot/linorobot2_hardware/tree/galactic/firmware/lib/kinematics)

小车轮距、轮径、转速等信息在`lib/config`中定义；

- 核心函数1：由线速度和角速度推算小车车轮转速

```c++
rpm getRPM(float linear_x, float linear_y, float angular_z);
```

- 核心函数2：由小车车轮转速推算线速度和角速度

```c++
velocities getVelocities(int rpm1, int rpm2, int rpm3, int rpm4);
```

### 	2.1.5 lib/motor

`motor_interface.h`抽象了电机的各个功能，如正转、反转、停机等；

`motor.h`继承了`motor_interface`并对其各个功能进行了实现，并添加了`pwm_offset`;

`pwm_offset`是为了解决一些电机在低占空比pwm驱动下，电机不转动的问题；

### 	2.1.6 lib/odemetry

定义了里程计`nav_msgs/msg/odometry`，用于发布`odometry message`；

- 核心函数1：获取当前的`odometry message`数据

  ```c++
  nav_msgs__msg__Odometry Odometry::getData()
  {
    return odom_msg_;
  }
  ```

- 核心函数2：传入速度信息，更新`odometry message`数据

```c++
void update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z);
```

### 	2.1.7 lib/pid

顾名思义，电机的PID控制，不多解释了，看代码~

### 	2.1.8 lib/micro_ros_arduino

基于官方的[micro_ros_arduino#v2.0.5foxy](https://github.com/micro-ROS/micro_ros_arduino#v2.0.5-foxy)，对其进行了必要修改，并进行了rebuild：

- 修改`UCLIENT_CUSTOM_TRANSPORT_MTU=1024`，使得采用best_effort的传输策略时，可以传输大于512Bytes的数据(主要是odometry message)；
- 修改`RMW_UXRCE_MAX_HISTORY=2`，减少RAM使用，避免内存溢出；
- 具体rebuild步骤可参考：https://github.com/micro-ROS/micro_ros_arduino/issues/1221#issuecomment-1329923473

### 	2.1.9  src/main.cpp

- 初始化编码器、pid控制、电机等
- 创建2个publisher: odom和imu
- 创建1个subscriber: cmd_vel
- 创建3个软件定时器：2个用于publisher发布；1个用于控制机器人移动；

# 3 ROS功能包部分

## 3.1 ld14

- 激光雷达的功能包，一般由雷达厂家提供

## 3.2 robot_bringup

- 放置robot开机启动的程序(目前只有一个default.launch.py)

### 	3.2.1 default.launch.py

- 依次启动激光雷达、robot_description、micro_agent等等

## 3.3 robot_description

- 主要放置机器人描述文件，即urdf

## 3.4 robot_cartographer

- 利用cartoprapher进行建图，具体信息自行学习cartographer。

# 4 已知问题

- IMU坐标系未与ROS默认的坐标系对齐，需要重新对齐，后续有进度再更新。
- 因为IMU坐标系问题，采用EKF融合IMU和Odometry会出错，故直接使用编码器直出Odom。
- Odom的更新频率目前维持在40Hz，再高无法达到，似乎串口波特率到极限了；可尝试wifi通讯。
- 未添加camera驱动，因为后续有使用camera进行再次开发的想法，等待后续更新。



