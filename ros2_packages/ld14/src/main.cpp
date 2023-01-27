/*************************************************************************************
@company: Copyright (C) 2022, LD Robot, WHEELTEC (Dongguan) Co., Ltd
@product: LD14
@filename: main.cpp
@brief:
@version:       date:       author:            comments:
@v2.0           22-4-25      marketing1,Tues          ROS2
*************************************************************************************/
#include <iostream>
#include "cmd_interface_linux.h"
#include <stdio.h>
#include "lipkg.h"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include "transform.h"

#define ANGLE_TO_RADIAN(angle) ((angle)*3141.59/180000)

extern int angle1_start,angle2_start,angle3_start,angle4_start;
extern int angle1_end,angle2_end,angle3_end,angle4_end;

extern 	bool flag_parted;
rclcpp::Node::SharedPtr l_node;

int main(int argc , char **argv)
{
	rclcpp::init(argc, argv);
	l_node=rclcpp::Node::make_shared("ld14_node");
	//ros::init(argc, argv, "product");
	//ros::NodeHandle nh;                    /* create a ROS Node */

	std::string product;
	std::string port_name="/dev/LD14";

	float scan_duration;
	float scan_time;
	

	rclcpp::Time start_scan_time;
	rclcpp::Time end_scan_time;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub;
        start_scan_time=l_node->now();
        end_scan_time=l_node->now();

	l_node->declare_parameter<std::string>("product", "LD14");
	l_node->get_parameter<std::string>("product", product);

	l_node->declare_parameter<std::string>("port_name", "/dev/LD14");
	l_node->get_parameter<std::string>("port_name", port_name);

	l_node->declare_parameter<bool>("flag_parted", "False");
	l_node->get_parameter<bool>("flag_parted", flag_parted);

	l_node->declare_parameter<int>("angle1_start", 1);
	l_node->get_parameter<int>("angle1_start", angle1_start);
	l_node->declare_parameter<int>("angle1_end", 90);
	l_node->get_parameter<int>("angle1_end", angle1_end);


	l_node->declare_parameter<int>("angle2_start", 92);
	l_node->get_parameter<int>("angle2_start", angle2_start);
	l_node->declare_parameter<int>("angle2_end", 93);
	l_node->get_parameter<int>("angle2_end", angle2_end);


	l_node->declare_parameter<int>("angle3_start", 94);
	l_node->get_parameter<int>("angle3_start", angle3_start);
	l_node->declare_parameter<int>("angle3_end", 95);
	l_node->get_parameter<int>("angle3_end", angle3_end);



	l_node->declare_parameter<int>("angle4_start", 96);
	l_node->get_parameter<int>("angle4_start", angle4_start);
	l_node->declare_parameter<int>("angle4_end", 97);
	l_node->get_parameter<int>("angle4_end", angle4_end);



	LiPkg * pkg;

	
	int32_t ver=0;
        //printf("f=%d\n",strcmp(product.c_str(),"LD14"));
	if(strcmp(product.c_str(),"LD00")==0)
	{
		ver = 0;
		pkg = new LD00_LiPkg; 
	}else if(strcmp(product.c_str(),"LD03")==0)
	{
	    ver = 3;
		pkg = new LD03_LiPkg;
	}
	else if(strcmp(product.c_str(),"LD08")==0)
	{
		ver = 8;
		pkg = new LD08_LiPkg;
	}else if(strcmp(product.c_str(),"LD14")==0)
	{
		ver = 9;
		pkg = new LD14_LiPkg; 
	}
	else
	{
		std::cout << "USAGE: Start ldlidar LD0* "<<std::endl;
		std::cout << "Usage : LD0* ->  LD00 LD03 LD08 LD14  "<<std::endl; /* print Usage */
		//return -1;
	}
	 
    /* test code */
	#ifdef USE_SLBF
		std::cout << "USE_SLBF"<<std::endl; 
	#endif
	#ifdef USE_SLBI
		std::cout << "USE_SLBI"<<std::endl; 
	#endif

	char product_ver[5]={0};   				/*production version*/
	strcpy(product_ver,product.c_str());
	
	char topic_name[20]={0};
	strcat(topic_name,product_ver); 
	strcat(topic_name,"/LDLiDAR");
	lidar_pub = l_node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 3);


    CmdInterfaceLinux cmd_port(ver);
    std::vector<std::pair<std::string, std::string> > device_list;
    //std::string port_name;
    cmd_port.GetCmdDevices(device_list);

	if(port_name.empty() == false)
	{
		std::cout<<"FOUND LiDAR_" <<  product_ver  <<std::endl;
 printf("ld14-0");
		cmd_port.SetReadCallback([&pkg](const char *byte, size_t len) {
			if(pkg->Parse((uint8_t*)byte, len))
			{
				pkg->AssemblePacket();  
			}
		});
	       
		cmd_port.Open(port_name);
		sensor_msgs::msg::LaserScan scan;
		scan.header.stamp = l_node->get_clock()->now();
		scan.header.frame_id = "laser";
		scan.range_min = 0.3;
		scan.range_max = 30.0;

		while (rclcpp::ok()) 
		{
			if (pkg->IsFrameReady())
			{				
				FrameData data = pkg->GetFrameData();
				start_scan_time = l_node->get_clock()->now();
				scan_time =(start_scan_time - end_scan_time ).seconds(); 
                scan_duration = scan_time / data.len;  

				scan.angle_min = ANGLE_TO_RADIAN(data.angle_min)-3.1415926;
				scan.angle_max = ANGLE_TO_RADIAN(data.angle_max)-3.1415926;
				scan.angle_increment = (scan.angle_max - scan.angle_min)/data.len;
				scan.ranges.resize(data.len);
				scan.intensities.resize(data.len);
				for(int i=data.len;i>=0;i--) 
				{
					if(i<data.len/2)
					{
						scan.ranges[i+data.len/2] = data.distance[i]/1000.f;
						if (scan.ranges[i+data.len/2] <= scan.range_min || scan.ranges[i+data.len/2] >= scan.range_max||(scan.ranges[i+data.len/2] ==0.0))
	                    scan.ranges[i+data.len/2] = std::numeric_limits<float>::infinity();  // 如果不在角度不在扫描范围内，将该角度对应的目标距离置为无穷大
						scan.intensities[i+data.len/2] = data.intensities[i];
					}
					else{
						scan.ranges[i-data.len/2] = data.distance[i]/1000.f;
						if (scan.ranges[i-data.len/2] <= scan.range_min || scan.ranges[i-data.len/2] >= scan.range_max||(scan.ranges[i-data.len/2] ==0.0))
                    	scan.ranges[i-data.len/2] = std::numeric_limits<float>::infinity();  // 如果不在角度不在扫描范围内，将该角度对应的目标距离置为无穷大
						scan.intensities[i-data.len/2] = data.intensities[i];
					}

				}
				for(int i=0;i<data.len;i++) 
				{
					scan.ranges[i] = data.distance[i]/1000.f;
					if (scan.ranges[i] <= scan.range_min || scan.ranges[i] >= scan.range_max||(scan.ranges[i] ==0.0))
                    scan.ranges[i] = std::numeric_limits<float>::infinity();  // 如果不在角度不在扫描范围内，将该角度对应的目标距离置为无穷大
					scan.intensities[i] = data.intensities[i];
				}
				scan.header.stamp = l_node->get_clock()->now();
				//static uint16_t last_times_stamp = 0;
				//uint16_t dealt_times_stamp = 0;
				//uint16_t tmp_times_stamp = ros::Time::now();
				//if (tmp_times_stamp - last_times_stamp < 0){
				//	dealt_times_stamp = tmp_times_stamp - last_times_stamp + 30000; 
				//}else
				//{
				//	dealt_times_stamp = tmp_times_stamp - last_times_stamp;
				//}
				//last_times_stamp = tmp_times_stamp;
				//scan.time_increment = dealt_times_stamp;
				scan.scan_time=scan_time;
				scan.time_increment = scan_duration;
				lidar_pub->publish(scan);
				end_scan_time = l_node->get_clock()->now();
				// printf("min:%f,max:%f,len:%d\n",data.angle_min,data.angle_max,data.len);
				// std::cout << "lidar speed (Hz): " << pkg->GetSpeed() << std::endl;
				//std::cout << "lidar dealt time stamp: " << dealt_times_stamp << std::endl;
				
			}
		}
	}
	else
	{
		std::cout<<"Can't find LiDAR"<< argv[1] << std::endl;
	}


    return 0;
}

