/**
* @file         lipkg.h
* @author       LD Robot
* @version      V01

* @brief         
* @note          
* @attention    COPYRIGHT LDROBOT
**/

#ifndef __LIPKG_H
#define __LIPKG_H
#include <stdint.h>
#include <vector>
#include <array>
#include <iostream>
#include "pointdata.h"

enum
{
	PKG_HEADER = 0x54,
	PKG_VER_LEN = 0x2C,
	POINT_PER_PACK = 12,
};


typedef struct  __attribute__((packed))
{
	uint16_t distance;
	uint8_t  confidence;
}LidarPointStructDef;

typedef struct  __attribute__((packed))
{
	uint8_t           header;
	uint8_t           ver_len;
	uint16_t          speed;
	uint16_t          start_angle;
	LidarPointStructDef point[POINT_PER_PACK];
	uint16_t          end_angle;
	uint16_t          timestamp;
	uint8_t           crc8;
}LiDARFrameTypeDef;


typedef std::vector<PointData> Points2D;

struct FrameData
{
	float angle_min;
	float angle_max;
	uint32_t len;
	std::vector<uint16_t> distance;
	std::vector<uint8_t> intensities;
};


class LiPkg
{
public:
	LiPkg();
	double GetSpeed(void);	/*Lidar spin speed (Hz)*/
	uint16_t GetTimestamp(void) { return mTimestamp; }   /*time stamp of the packet */
	bool IsPkgReady(void) { return mIsPkgReady; }/*a packet is ready */
	bool IsFrameReady(void) { return mIsFrameReady; }/*Lidar data frame is ready*/
	long GetErrorTimes(void) { return mErrorTimes; }/*the number of errors in parser process of lidar data frame*/
	const std::array<PointData, POINT_PER_PACK>& GetPkgData(void);/*original data package*/
	bool Parse(const uint8_t* data , long len);/*parse single packet*/
	virtual void Transform(std::vector<PointData> &tmp)=0;/*transform raw data to stantard data */
	bool AssemblePacket();/*combine stantard data into data frames and calibrate*/
	const FrameData& GetFrameData(void) { mIsFrameReady = false; return mFrameData; }
	
private:
	uint16_t mTimestamp;
	double mSpeed;
	std::vector<uint8_t> mDataTmp;
	std::vector<int> parted_max,parted_min;

	long mErrorTimes;
	std::array<PointData, POINT_PER_PACK>mOnePkg;
	std::vector<PointData> mFrameTemp;
	FrameData mFrameData;
	bool mIsPkgReady;
	bool mIsFrameReady;
};

class LD00_LiPkg : public LiPkg
{
	public:
		virtual void Transform(std::vector<PointData> &tmp);
};

class LD03_LiPkg : public LiPkg
{
	public:
		virtual void Transform(std::vector<PointData> &tmp);
};

class LD08_LiPkg : public LiPkg
{
	public:
		virtual void Transform(std::vector<PointData> &tmp);
};

class LD14_LiPkg : public LiPkg
{
	public:
		virtual void Transform(std::vector<PointData> &tmp); 
};


#endif
/********************* (C) COPYRIGHT LD Robot *******END OF FILE ********/
