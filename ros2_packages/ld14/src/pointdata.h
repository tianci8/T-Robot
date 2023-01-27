#ifndef _POINT_DATA_H_
#define _POINT_DATA_H_

#include <stdint.h>
#include <vector>
#include <iostream>

struct PointData
{
    //极坐标表示方式
    float angle;
    uint16_t distance;
    uint8_t confidence;
    //直角坐标表示方式
    double x;
    double y;
    PointData(float angle, uint16_t distance, uint8_t confidence , double x = 0, double y = 0)
    {
        this->angle = angle;
        this->distance = distance;
        this->confidence = confidence;
        this->x = x;
        this->y = y;
    }
    PointData() {}
    friend std::ostream& operator<<(std::ostream &os , const PointData &data)
    {
        os << data.angle << " "<< data.distance << " " << (int)data.confidence << " "<<data.x << " "<<data.y;
        return  os;
    }
};


typedef std::vector<PointData> Points2D;

#endif
