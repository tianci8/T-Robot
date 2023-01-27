/*************************************************************************************
@company: Copyright (C) 2022, LD Robot, WHEELTEC (Dongguan) Co., Ltd
@product: LD14
@filename: lipkg.cpp 
@brief:
@version:       date:       author:            comments:
@v2.0           22-4-25     LD Robot,Tues          ROS2
*************************************************************************************/
#include "lipkg.h"
#include <math.h>
#include <algorithm>

#ifdef USE_SLBI
	#include "slbi.h"
#endif

#ifdef USE_SLBF
	#include "slbf.h"
#endif
//#define ANGLE_TO_RADIAN(angle) ((angle)*3141.59/180000)

int flag_parted;
float change_angle;
int angle1_start,angle2_start,angle3_start,angle4_start;
int angle1_end,angle2_end,angle3_end,angle4_end;

static const uint8_t CrcTable[256] =
{
	0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
	0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
	0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
	0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
	0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
	0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
	0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
	0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
	0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
	0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
	0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
	0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
	0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
	0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
	0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
	0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
	0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
	0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
	0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
	0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
	0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
	0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};


LiPkg::LiPkg():
	mTimestamp(0),
	mSpeed(0),
	mErrorTimes(0),
	mIsFrameReady(false),
	mIsPkgReady(false)
{
    parted_min.push_back(angle1_start);
    parted_min.push_back(angle2_start);
    parted_min.push_back(angle3_start);
    parted_min.push_back(angle4_start);

    parted_max.push_back(angle1_end);
    parted_max.push_back(angle2_end);
    parted_max.push_back(angle3_end);
    parted_max.push_back(angle4_end);
    
	  	for(int j=0;j<parted_max.size();++j)
	{
		change_angle=360-parted_min[j];
		parted_min[j]=360-parted_max[j];
		parted_max[j]=change_angle;
	}

}

double LiPkg::GetSpeed(void)
{
	return mSpeed/360.0;
}
/*
bool jdg_angle(double angle_s,double angle_e,double angle_c){
	if(angle_s<=angle_e){
		return angle_c>=angle_s&&angle_c<angle_e;
	}
	else{
		return angle_c>=angle_s||angle_c<=angle_e;
	}	
}
*/
bool LiPkg::Parse(const uint8_t * data, long len)
{
	for (int i = 0; i < len; i++)
	{
		mDataTmp.push_back(*(data + i));
	}

	if (mDataTmp.size() < sizeof(LiDARFrameTypeDef))
		return false;

	if (mDataTmp.size() > sizeof(LiDARFrameTypeDef) * 100)
	{
		mErrorTimes++;
		mDataTmp.clear();
		return false;
	}

	uint16_t start = 0;

	while (start < mDataTmp.size() - 2)
	{
		start = 0;
		while (start < mDataTmp.size() - 2)
		{
			if ((mDataTmp[start] == PKG_HEADER) && (mDataTmp[start + 1] == PKG_VER_LEN))
			{
				break;
			}

			if ((mDataTmp[start] == PKG_HEADER) && (mDataTmp[start + 1] == (PKG_VER_LEN | (0x07 << 5))))
			{
				break;
			}
			start++;
		}

		if (start != 0)
		{
			mErrorTimes++;
			for (int i = 0; i < start; i++)
			{
				mDataTmp.erase(mDataTmp.begin());
			}
		}

		if (mDataTmp.size() < sizeof(LiDARFrameTypeDef))
			return false;

	
		LiDARFrameTypeDef* pkg = (LiDARFrameTypeDef *)mDataTmp.data();
		uint8_t crc = 0;

		for (uint32_t i = 0; i < sizeof(LiDARFrameTypeDef) - 1; i++)
		{
			crc = CrcTable[(crc ^ mDataTmp[i]) & 0xff];
		}

		if (crc == pkg->crc8)
		{
			double diff = (pkg->end_angle / 100 - pkg->start_angle / 100 + 360) % 360;
			if (diff > (double)pkg->speed*POINT_PER_PACK / 2300 * 3 / 2)
			{
				mErrorTimes++;
			}
			else
			{
				mSpeed = pkg->speed;
				mTimestamp = pkg->timestamp;
				uint32_t diff = ((uint32_t)pkg->end_angle + 36000 - (uint32_t)pkg->start_angle) % 36000;
				float step = diff / (POINT_PER_PACK - 1) / 100.0;
				float start = (double)pkg->start_angle / 100.0;
				float end = (double)(pkg->end_angle % 36000) / 100.0;
				PointData data;
				for (int i = 0; i < POINT_PER_PACK; i++)
				{
					data.distance = pkg->point[i].distance;
					data.angle = start + i * step;
					if (data.angle >= 360.0)
					{
						data.angle -= 360.0;
					}
					data.confidence = pkg->point[i].confidence;
					mOnePkg[i] = data;
					mFrameTemp.push_back(PointData(data.angle, data.distance, data.confidence));
				}
				//prevent angle invert
				mOnePkg.back().angle = end;

				mIsPkgReady = true;
			}

			for (uint32_t i = 0; i < sizeof(LiDARFrameTypeDef); i++)
			{
				mDataTmp.erase(mDataTmp.begin());
			}

			if (mDataTmp.size() < sizeof(LiDARFrameTypeDef))
			{
				break;
			}
		}
		else
		{
			mErrorTimes++;
			/*only remove header,not all frame,because lidar data may contain head*/
			for (int i = 0; i < 2; i++)
			{
				mDataTmp.erase(mDataTmp.begin());
			}
		}
	}

	return true;
}

bool LiPkg::AssemblePacket()
{
	float last_angle = 0;
	std::vector<PointData> tmp;

    //parted_max=par_data.parted_max;
	//~ for(int i=0;i<parted_min.size();i++)
	//~ {
	//~ std::cout<<parted_min[i]<<std::endl;
	//~ }
	int count = 0;
	double angle_cur;
	for (auto n : mFrameTemp)
	{
		/*wait for enough data, need enough data to show a circle*/
		if (n.angle - last_angle < (-350.f)) /* enough data has been obtained */
		{
			mFrameData.len = tmp.size();
			Transform(tmp);                  /*transform raw data to stantard data */

			if (tmp.size() == 0)
			{
				mFrameTemp.clear();
				mIsFrameReady = false;
				return false;
			}
			

			#ifdef USE_SLBI
				Slbi sb(mSpeed);
				sb.FindBarcode(tmp);
			#endif

			#ifdef USE_SLBF
				Slbf sb(mSpeed);
			tmp = sb.NearFilter(tmp);
			#endif

			std::sort(tmp.begin(), tmp.end(), [](PointData a, PointData b) {return a.angle < b.angle; });
			
			
			mFrameData.angle_min = tmp[0].angle;
			mFrameData.angle_max = tmp.back().angle;
			
			mFrameData.distance.clear();
			
			mFrameData.intensities.clear();
			
			
			mFrameData.distance.resize(mFrameData.len);
			mFrameData.intensities.resize(mFrameData.len);
			
			
			float step = (mFrameData.angle_max - mFrameData.angle_min)/mFrameData.len;
			
			
			float angle_acc = mFrameData.angle_min;
			  
			int tmp_count = 0;
			/* interpolation method */
			int tmp_range_min=0;
			
			
	 for (uint32_t i=0;i< mFrameData.len;i++)
			{
			angle_cur=i*360/mFrameData.len;	
			if(angle_acc >= tmp[tmp_count].angle){
				mFrameData.distance[i] = tmp[tmp_count].distance;
				mFrameData.intensities[i] = tmp[tmp_count].confidence;
				tmp_count ++;
			if(tmp_count == tmp.size()){
				break;
			}	
			}
			else{
			mFrameData.distance[i] = 0;
				mFrameData.intensities[i] = 0;
			}

			if(flag_parted){
			for(int j=0;j<parted_max.size();++j)
			{
				if((i>(parted_min[j]*mFrameData .len / 360))&&(i<=(parted_max[j]*mFrameData.len / 360))){
				mFrameData.distance[i] = std::numeric_limits<float>::infinity();  // 如果不在角度不在扫描范围内，将该角度对应的目标距离置为无穷大
				mFrameData.intensities[i] = 0;
				}
			
			}}		
			angle_acc += step;
			}

			std::vector<PointData>tmp2;
			
			for (uint32_t i = count; i < mFrameTemp.size(); i++)
			{
				tmp2.push_back(mFrameTemp[i]);
			}
			mFrameTemp.clear();
			mFrameTemp = tmp2;
			mIsFrameReady = true;
			return true;
		}
		else
		{
			tmp.push_back(n);  /* getting data */
		}
		count++;
		last_angle = n.angle;
	}

	return false;
}

const std::array<PointData, POINT_PER_PACK>& LiPkg::GetPkgData(void)
{
	mIsPkgReady = false;
	return mOnePkg;
}

/********************* (C) COPYRIGHT LD Robot *******END OF FILE ********/
