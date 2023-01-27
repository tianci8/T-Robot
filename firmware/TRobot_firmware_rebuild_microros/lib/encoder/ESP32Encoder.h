/*
 * @Author: Tianci Zhang
 * @Email: tianci_zhang@tju.edu.cn
 * @Date: 2022-11-19 21:14:54
 * @LastEditors: Tianci Zhang
 * @LastEditTime: 2022-11-19 21:35:21
 * @FilePath: \test_mpu9250_with_motor\lib\encoder\ESP32Encoder.h
 * @Description: 
 * 
 * Copyright (c) 2022 by tianci_zhang@tju.edu.cn, All Rights Reserved. 
 */
#pragma once
#include <driver/gpio.h>
#include <driver/pcnt.h>
#define MAX_ESP32_ENCODERS PCNT_UNIT_MAX
#define 	_INT16_MAX 32766
#define  	_INT16_MIN -32766
#define ISR_CORE_USE_DEFAULT (0xffffffff)

enum encType {
	single,
	half,
	full
};

enum puType {
	UP,
	DOWN,
	NONE
};

class ESP32Encoder;

typedef void (*enc_isr_cb_t)(void*);

class ESP32Encoder {
public:
	/**
	 * @brief Construct a new ESP32Encoder object
	 *
	 * @param always_interrupt set to true to enable interrupt on every encoder pulse, otherwise false
	 * @param enc_isr_cb callback executed on every encoder ISR, gets a pointer to
	 * 	the ESP32Encoder instance as an argument, no effect if always_interrupt is
	 * 	false
	 */
	ESP32Encoder(bool always_interrupt=false, enc_isr_cb_t enc_isr_cb=nullptr, void* enc_isr_cb_data=nullptr);
	~ESP32Encoder();
	void attachHalfQuad(int aPintNumber, int bPinNumber);
	void attachFullQuad(int aPintNumber, int bPinNumber);
	void attachSingleEdge(int aPintNumber, int bPinNumber);
	int64_t getCount();
	int64_t clearCount();
	int64_t pauseCount();
	int64_t resumeCount();
	void detach();
	[[deprecated("Replaced by detach")]] void detatch();
	bool isAttached(){return attached;}
	void setCount(int64_t value);
	void setFilter(uint16_t value);
	static ESP32Encoder *encoders[MAX_ESP32_ENCODERS];
	bool always_interrupt;
	gpio_num_t aPinNumber;
	gpio_num_t bPinNumber;
	pcnt_unit_t unit;
	int countsMode = 2;
	volatile int64_t count=0;
	pcnt_config_t r_enc_config;
	static enum puType useInternalWeakPullResistors;
	static uint32_t isrServiceCpuCore;
	enc_isr_cb_t _enc_isr_cb;
	void* _enc_isr_cb_data;
	float getRPM(int counts_per_rev);

private:
	static bool attachedInterrupt;
	void attach(int aPintNumber, int bPinNumber, enum encType et);
	int64_t getCountRaw();
	bool attached;
  bool direction;
  bool working;
  unsigned long prev_update_time_;
  int64_t prev_encoder_ticks_;
  int counts_per_rev_;
};

//Added by Sloeber
#pragma once
