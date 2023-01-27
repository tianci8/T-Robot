/**
* @file         transform.h
* @author       Weyne Chen
* @version      V01
* @date         2020.04.26
* @brief         
* @note          
* @attention    COPYRIGHT WEYNE
**/

#ifndef __TRANSFORM_H
#define __TRANSFORM_H

#include "lipkg.h"
#include <vector>
#include <math.h>
#include <algorithm>

enum class LDVersion
{
    LD_ZERO,  /*Zero  generation lidar*/
    LD_THREE, /*Third generation lidar*/
    LD_EIGHT, /*Eight generation radar*/
    LD_NINE,  /*Nine  generation radar*/
    LD_FOURTEENTH /*Fourteenth generation radar*/
};

class SlTransform
{
private:
    bool to_right_hand = true;
    double offset_x;
    double offset_y;

public:
    SlTransform(LDVersion version,  bool to_right_hand = false);
    Points2D Transform(const Points2D &data);
    ~SlTransform();
};


#endif
/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
