#ifndef __SLBF_H_
#define __SLBF_H_

#include <stdint.h>
#include <vector>
#include "pointdata.h"

class Slbf
{
private:
    const int CONFIDENCE_HIGH = 200; 
	const int CONFIDENCE_LOW = 92; 
    const int SCAN_FRE = 2300;     /*Default scanning frequency, which can be changed according to radar protocol*/
    double curr_speed;
	bool enable_strict_policy; /*whether strict filtering is enabled within 300 mm, the effective value may be lost, and the time sequence of recharging needs to be disabled*/
    Slbf() = delete;
    Slbf(const Slbf &) = delete;
    Slbf &operator=(const Slbf &) = delete;
public:
    Slbf(int speed , bool strict_policy = true);
	Points2D NearFilter(const Points2D &tmp) const;
	void EnableStrictPolicy(bool enable);
    ~Slbf();
};

#endif