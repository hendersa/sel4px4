/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file simulator.h
 * A device simulator
 */

#pragma once

#include <semaphore.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_attitude.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_rc_input.h>
#include <uORB/uORB.h>
#include <v1.0/mavlink_types.h>
#include <v1.0/common/mavlink.h>
#ifndef __PX4_QURT
#include <sys/socket.h>
#include <netinet/in.h>
#endif

namespace simulator {

// FIXME - what is the endianness of these on actual device?
#pragma pack(push, 1)
struct RawAccelData {
		float temperature;
        float x;
        float y;
        float z;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RawMagData {
		float temperature;
        float x;
        float y;
        float z;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RawMPUData {
	float	accel_x;
	float	accel_y;
	float	accel_z;
	float	temp;
	float	gyro_x;
	float	gyro_y;
	float	gyro_z;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RawBaroData {
	float pressure;
	float altitude;
	float temperature;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct RawGPSData {
 int32_t lat;
 int32_t lon;
 int32_t alt;
 uint16_t eph;
 uint16_t epv;
 uint16_t vel;
 int16_t vn;
 int16_t ve;
 int16_t vd;
 uint16_t cog;
 uint8_t fix_type;
 uint8_t satellites_visible;
};
#pragma pack(pop)

template <typename RType> class Report {
public:
	Report(int readers) :
			_readidx(0),
        	_max_readers(readers),
        	_report_len(sizeof(RType))
	{
        	sem_init(&_lock, 0, _max_readers);
	}

	~Report() {};

	bool copyData(void *outbuf, int len)
	{
		if (len != _report_len) {
			return false;
		}
		read_lock();
		memcpy(outbuf, &_buf[_readidx], _report_len);
		read_unlock();
		return true;
	}
	void writeData(void *inbuf)
	{
		write_lock();
		memcpy(&_buf[!_readidx], inbuf, _report_len);
		_readidx = !_readidx;
		write_unlock();
	}

protected:
	void read_lock() { sem_wait(&_lock); }
	void read_unlock() { sem_post(&_lock); }
	void write_lock() 
	{
		for (int i=0; i<_max_readers; i++) {
                	sem_wait(&_lock);
        	}
	}
	void write_unlock()
	{
		for (int i=0; i<_max_readers; i++) {
			sem_post(&_lock);
		}
	}

	int _readidx;
	sem_t _lock;
	const int _max_readers;
	const int _report_len;
	RType _buf[2];
};

};

class Simulator {
public:
	static Simulator *getInstance();

	enum sim_dev_t {
		SIM_GYRO,
		SIM_ACCEL,
		SIM_MAG
	};

	struct sample {
		float x;
		float y;
		float z;
		sample() : x(0), y(0), z(0) {}
		sample(float a, float b, float c) : x(a), y(b), z(c) {}
	};

	static int start(int argc, char *argv[]);

	bool getRawAccelReport(uint8_t *buf, int len);
	bool getMagReport(uint8_t *buf, int len);
	bool getMPUReport(uint8_t *buf, int len);
	bool getBaroSample(uint8_t *buf, int len);
	bool getGPSSample(uint8_t *buf, int len);

	void write_MPU_data(void *buf);
	void write_accel_data(void *buf);
	void write_mag_data(void *buf);
	void write_baro_data(void *buf);
	void write_gps_data(void *buf);

private:
	Simulator() :
	_accel(1),
	_mpu(1),
	_baro(1),
	_mag(1),
	_gps(1),
	_sensor_combined_pub(nullptr)
#ifndef __PX4_QURT
	,
	_rc_channels_pub(nullptr),
	_actuator_outputs_sub(-1),
	_vehicle_attitude_sub(-1),
	_manual_sub(-1),
	_sensor{},
	_rc_input{},
	_actuators{},
	_attitude{},
	_manual{}
#endif
	{}
	~Simulator() { _instance=NULL; }

#ifndef __PX4_QURT
	void updateSamples();
#endif

	static Simulator *_instance;

	// simulated sensor instances
	simulator::Report<simulator::RawAccelData> 	_accel;
	simulator::Report<simulator::RawMPUData>	_mpu;
	simulator::Report<simulator::RawBaroData>	_baro;
	simulator::Report<simulator::RawMagData> 	_mag;
	simulator::Report<simulator::RawGPSData>	_gps;

	// uORB publisher handlers
	orb_advert_t _accel_pub;
	orb_advert_t _baro_pub;
	orb_advert_t _gyro_pub;
	orb_advert_t _mag_pub;
	orb_advert_t _sensor_combined_pub;

	// class methods
	void publishSensorsCombined();

#ifndef __PX4_QURT
	// uORB publisher handlers
	orb_advert_t _rc_channels_pub;

	// uORB subscription handlers
	int _actuator_outputs_sub;
	int _vehicle_attitude_sub;
	int _manual_sub;

	// uORB data containers
	struct sensor_combined_s _sensor;
	struct rc_input_values _rc_input;
	struct actuator_outputs_s _actuators;
	struct vehicle_attitude_s _attitude;
	struct manual_control_setpoint_s _manual;

	int _fd;
	unsigned char _buf[200];
	struct sockaddr_in _srcaddr;
	socklen_t _addrlen = sizeof(_srcaddr);

	void poll_actuators();
	void handle_message(mavlink_message_t *msg);
	void send_controls();
	void pack_actuator_message(mavlink_hil_controls_t &actuator_msg);
	void send_mavlink_message(const uint8_t msgid, const void *msg, uint8_t component_ID);
	void update_sensors(struct sensor_combined_s *sensor, mavlink_hil_sensor_t *imu);
	void update_gps(mavlink_hil_gps_t *gps_sim);
	static void *sending_trampoline(void *);
	void send();
#endif
};
