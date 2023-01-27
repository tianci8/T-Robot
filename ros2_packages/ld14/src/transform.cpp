/**
* @file         transform.c
* @author       Weyne Chen
* @version      V01
* @date         2020.04.26
* @brief         
* @note          
* @attention    COPYRIGHT WEYNE
**/

#include "transform.h"
#include <iostream>

/*!
	\brief     transfer the origin to the center of lidar circle
	\param[in]
	  \arg  version  lidar version, different versions have different parameter settings
      \arg  data     lidar raw data
      \arg  to_right_hand  a flag whether convert to right-hand coordinate system

	\param[out]  data
	\retval      Data after coordinate conversion
*/
SlTransform::SlTransform(LDVersion version,  bool to_right_hand)
{
    switch (version)
    {
    case LDVersion::LD_ZERO:
    case LDVersion::LD_NINE:
        offset_x = 8.1;
        offset_y = -22.5156;
        break;

    case LDVersion::LD_THREE:
    case LDVersion::LD_EIGHT:
        offset_x = 5.9;
        offset_y = -20.14;
        break;
    
    case LDVersion::LD_FOURTEENTH:
        offset_x = 5.9;
        offset_y = -20.14;
        break;

    default:
        break;
    }
}

Points2D SlTransform::Transform(const Points2D &data)
{
    Points2D tmp2;
    for (auto n : data)
    {
        /*Filter out invalid data*/
        if (n.distance == 0)
        {
            continue;
        }
        /*transfer the origin to the center of lidar circle*/
        /*The default direction of radar rotation is clockwise*/
        /*transfer to the right-hand coordinate system*/
        float right_hand = (360.f - n.angle);
        double x = n.distance + offset_x;
        double y = n.distance * 0.11923 + offset_y;
        double d = sqrt(x * x + y * y);
        double shift = atan(y / x) * 180.f / 3.14159;
        /*Choose whether to use the right-hand system according to the flag*/
        double angle;
        if (to_right_hand)
            angle = right_hand + shift;
        else
            angle = n.angle - shift;

        if (angle > 360)
        {
            angle -= 360;
        }
        if (angle < 0)
        {
            angle += 360;
        }

        tmp2.push_back(PointData(angle, n.distance, n.confidence,x,y));
    }

    return tmp2;
}

SlTransform::~SlTransform()
{
}


/*
1.Internal conversion of lidar data, transfer the origin to the center of lidar circle, and transfer to the right-hand coordinate system
2.TODO : Transfer to machine center (this part needs to be completed by customers according to their own machine structure)
3.TODO : Real time calibration of rotation and translation deviation through IMU data (this part needs to be completed by customers according to their own equipment)
*/
void LD00_LiPkg::Transform(std::vector<PointData> &tmp)
{
	SlTransform trans(LDVersion::LD_ZERO);
	tmp = trans.Transform(tmp);
}
void LD03_LiPkg::Transform(std::vector<PointData> &tmp)
{
	SlTransform trans(LDVersion::LD_THREE);
	tmp = trans.Transform(tmp);
}
void LD08_LiPkg::Transform(std::vector<PointData> &tmp)
{
	SlTransform trans(LDVersion::LD_EIGHT);
	tmp = trans.Transform(tmp);
}
void LD14_LiPkg::Transform(std::vector<PointData> &tmp)  
{
	SlTransform trans(LDVersion::LD_FOURTEENTH);
	tmp = trans.Transform(tmp);
    // std::cout << "Transform LD_FOURTEENTH !!" << std::endl;
}

/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
