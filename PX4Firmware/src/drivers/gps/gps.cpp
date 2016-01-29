/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file gps.cpp
 * Driver for the GPS on a serial port
 */

#include <nuttx/clock.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/i2c.h>
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/err.h>
#include <drivers/drv_gps.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/satellite_info.h>

#include <board_config.h>

#include "ubx.h"
#include "mtk.h"
#include "ashtech.h"


#define TIMEOUT_5HZ 500
#define RATE_MEASUREMENT_PERIOD 5000000

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/* class for dynamic allocation of satellite info data */
class GPS_Sat_Info
{
public:
	struct satellite_info_s 	_data;
};


class GPS : public device::CDev
{
public:
	GPS(const char *uart_path, bool fake_gps, bool enable_sat_info);
	virtual ~GPS();

	virtual int			init();

	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void				print_info();

private:

	bool				_task_should_exit;				///< flag to make the main worker task exit
	int				_serial_fd;					///< serial interface to GPS
	unsigned			_baudrate;					///< current baudrate
	char				_port[20];					///< device / serial port path
	volatile int			_task;						///< worker task
	bool				_healthy;					///< flag to signal if the GPS is ok
	bool				_baudrate_changed;				///< flag to signal that the baudrate with the GPS has changed
	bool				_mode_changed;					///< flag that the GPS mode has changed
	gps_driver_mode_t		_mode;						///< current mode
	GPS_Helper			*_Helper;					///< instance of GPS parser
	GPS_Sat_Info			*_Sat_Info;					///< instance of GPS sat info data object
	struct vehicle_gps_position_s	_report_gps_pos;				///< uORB topic for gps position
	orb_advert_t			_report_gps_pos_pub;				///< uORB pub for gps position
	struct satellite_info_s		*_p_report_sat_info;				///< pointer to uORB topic for satellite info
	orb_advert_t			_report_sat_info_pub;				///< uORB pub for satellite info
	float				_rate;						///< position update rate
	bool				_fake_gps;					///< fake gps output


	/**
	 * Try to configure the GPS, handle outgoing communication to the GPS
	 */
	void			 	config();

	/**
	 * Trampoline to the worker task
	 */
	static void			task_main_trampoline(void *arg);


	/**
	 * Worker task: main GPS thread that configures the GPS and parses incoming data, always running
	 */
	void				task_main(void);

	/**
	 * Set the baudrate of the UART to the GPS
	 */
	int				set_baudrate(unsigned baud);

	/**
	 * Send a reset command to the GPS
	 */
	void				cmd_reset();

};


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int gps_main(int argc, char *argv[]);

namespace
{

GPS	*g_dev;

}


GPS::GPS(const char *uart_path, bool fake_gps, bool enable_sat_info) :
	CDev("gps", GPS0_DEVICE_PATH),
	_task_should_exit(false),
	_healthy(false),
	_mode_changed(false),
	_mode(GPS_DRIVER_MODE_UBX),
	_Helper(nullptr),
	_Sat_Info(nullptr),
	_report_gps_pos_pub(-1),
	_p_report_sat_info(nullptr),
	_report_sat_info_pub(-1),
	_rate(0.0f),
	_fake_gps(fake_gps)
{
	/* store port name */
	strncpy(_port, uart_path, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	/* we need this potentially before it could be set in task_main */
	g_dev = this;
	memset(&_report_gps_pos, 0, sizeof(_report_gps_pos));

	/* create satellite info data object if requested */
	if (enable_sat_info) {
		_Sat_Info = new(GPS_Sat_Info);
		_p_report_sat_info = &_Sat_Info->_data;
		memset(_p_report_sat_info, 0, sizeof(*_p_report_sat_info));
	}

	_debug_enabled = true;
}

GPS::~GPS()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1)
		task_delete(_task);

	g_dev = nullptr;

}

int
GPS::init()
{
	int ret = ERROR;

	/* do regular cdev init */
	if (CDev::init() != OK)
		goto out;

	/* start the GPS driver worker task */
	_task = task_spawn_cmd("gps", SCHED_DEFAULT,
				SCHED_PRIORITY_SLOW_DRIVER, 1500, (main_t)&GPS::task_main_trampoline, nullptr);

	if (_task < 0) {
		warnx("task start failed: %d", errno);
		return -errno;
	}

	ret = OK;
out:
	return ret;
}

int
GPS::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	lock();

	int ret = OK;

	switch (cmd) {
	case SENSORIOCRESET:
		cmd_reset();
		break;

	default:
		/* give it to parent if no one wants it */
		ret = CDev::ioctl(filp, cmd, arg);
		break;
	}

	unlock();

	return ret;
}

void
GPS::task_main_trampoline(void *arg)
{
	g_dev->task_main();
}

void
GPS::task_main()
{

	/* open the serial port */
	_serial_fd = ::open(_port, O_RDWR);

	if (_serial_fd < 0) {
		log("failed to open serial port: %s err: %d", _port, errno);
		/* tell the dtor that we are exiting, set error code */
		_task = -1;
		_exit(1);
	}

	uint64_t last_rate_measurement = hrt_absolute_time();
	unsigned last_rate_count = 0;

	/* loop handling received serial bytes and also configuring in between */
	while (!_task_should_exit) {

		if (_fake_gps) {

			_report_gps_pos.timestamp_position = hrt_absolute_time();
			_report_gps_pos.lat = (int32_t)47.378301e7f;
			_report_gps_pos.lon = (int32_t)8.538777e7f;
			_report_gps_pos.alt = (int32_t)1200e3f;
			_report_gps_pos.timestamp_variance = hrt_absolute_time();
			_report_gps_pos.s_variance_m_s = 10.0f;
			_report_gps_pos.c_variance_rad = 0.1f;
			_report_gps_pos.fix_type = 3;
			_report_gps_pos.eph = 0.9f;
			_report_gps_pos.epv = 1.8f;
			_report_gps_pos.timestamp_velocity = hrt_absolute_time();
			_report_gps_pos.vel_n_m_s = 0.0f;
			_report_gps_pos.vel_e_m_s = 0.0f;
			_report_gps_pos.vel_d_m_s = 0.0f;
			_report_gps_pos.vel_m_s = sqrtf(_report_gps_pos.vel_n_m_s * _report_gps_pos.vel_n_m_s + _report_gps_pos.vel_e_m_s * _report_gps_pos.vel_e_m_s + _report_gps_pos.vel_d_m_s * _report_gps_pos.vel_d_m_s);
			_report_gps_pos.cog_rad = 0.0f;
			_report_gps_pos.vel_ned_valid = true;

			//no time and satellite information simulated

			if (!(_pub_blocked)) {
				if (_report_gps_pos_pub > 0) {
					orb_publish(ORB_ID(vehicle_gps_position), _report_gps_pos_pub, &_report_gps_pos);

				} else {
					_report_gps_pos_pub = orb_advertise(ORB_ID(vehicle_gps_position), &_report_gps_pos);
				}
			}

			usleep(2e5);

		} else {

			if (_Helper != nullptr) {
				delete(_Helper);
				/* set to zero to ensure parser is not used while not instantiated */
				_Helper = nullptr;
			}

			switch (_mode) {
			case GPS_DRIVER_MODE_UBX:
				_Helper = new UBX(_serial_fd, &_report_gps_pos, _p_report_sat_info);
				break;

			case GPS_DRIVER_MODE_MTK:
				_Helper = new MTK(_serial_fd, &_report_gps_pos);
				break;

			case GPS_DRIVER_MODE_ASHTECH:
				_Helper = new ASHTECH(_serial_fd, &_report_gps_pos, _p_report_sat_info);
				break;

			default:
				break;
			}

			unlock();

			if (_Helper->configure(_baudrate) == 0) {
				unlock();

				// GPS is obviously detected successfully, reset statistics
				_Helper->reset_update_rates();

				int helper_ret;
				while ((helper_ret = _Helper->receive(TIMEOUT_5HZ)) > 0 && !_task_should_exit) {
	//				lock();
					/* opportunistic publishing - else invalid data would end up on the bus */

					if (!(_pub_blocked)) {
						if (helper_ret & 1) {
							if (_report_gps_pos_pub > 0) {
								orb_publish(ORB_ID(vehicle_gps_position), _report_gps_pos_pub, &_report_gps_pos);

							} else {
								_report_gps_pos_pub = orb_advertise(ORB_ID(vehicle_gps_position), &_report_gps_pos);
							}
						}
						if (_p_report_sat_info && (helper_ret & 2)) {
							if (_report_sat_info_pub > 0) {
								orb_publish(ORB_ID(satellite_info), _report_sat_info_pub, _p_report_sat_info);

							} else {
								_report_sat_info_pub = orb_advertise(ORB_ID(satellite_info), _p_report_sat_info);
							}
						}
					}

					if (helper_ret & 1) {	// consider only pos info updates for rate calculation */
						last_rate_count++;
					}

					/* measure update rate every 5 seconds */
					if (hrt_absolute_time() - last_rate_measurement > RATE_MEASUREMENT_PERIOD) {
						_rate = last_rate_count / ((float)((hrt_absolute_time() - last_rate_measurement)) / 1000000.0f);
						last_rate_measurement = hrt_absolute_time();
						last_rate_count = 0;
						_Helper->store_update_rates();
						_Helper->reset_update_rates();
					}

					if (!_healthy) {
						const char *mode_str = "unknown";

						switch (_mode) {
						case GPS_DRIVER_MODE_UBX:
							mode_str = "UBX";
							break;

						case GPS_DRIVER_MODE_MTK:
							mode_str = "MTK";
							break;

						case GPS_DRIVER_MODE_ASHTECH:
							mode_str = "ASHTECH";
							break;

						default:
							break;
						}

						warnx("module found: %s", mode_str);
						_healthy = true;
					}
				}

				if (_healthy) {
					warnx("module lost");
					_healthy = false;
					_rate = 0.0f;
				}

				lock();
			}

			lock();

			/* select next mode */
			switch (_mode) {
			case GPS_DRIVER_MODE_UBX:
				_mode = GPS_DRIVER_MODE_MTK;
				break;

			case GPS_DRIVER_MODE_MTK:
				_mode = GPS_DRIVER_MODE_ASHTECH;
				break;

			case GPS_DRIVER_MODE_ASHTECH:
				_mode = GPS_DRIVER_MODE_UBX;
				break;

			default:
				break;
			}
		}

	}

	warnx("exiting");

	::close(_serial_fd);

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}



void
GPS::cmd_reset()
{
#ifdef GPIO_GPS_NRESET
	warnx("Toggling GPS reset pin");
	stm32_configgpio(GPIO_GPS_NRESET);
	stm32_gpiowrite(GPIO_GPS_NRESET, 0);
	usleep(100);
	stm32_gpiowrite(GPIO_GPS_NRESET, 1);
	warnx("Toggled GPS reset pin");
#endif
}

void
GPS::print_info()
{
	switch (_mode) {
	case GPS_DRIVER_MODE_UBX:
		warnx("protocol: UBX");
		break;

	case GPS_DRIVER_MODE_MTK:
		warnx("protocol: MTK");
		break;

	case GPS_DRIVER_MODE_ASHTECH:
		warnx("protocol: ASHTECH");
		break;

	default:
		break;
	}

	warnx("port: %s, baudrate: %d, status: %s", _port, _baudrate, (_healthy) ? "OK" : "NOT OK");
	warnx("sat info: %s", (_p_report_sat_info != nullptr) ? "enabled" : "disabled");

	if (_report_gps_pos.timestamp_position != 0) {
		warnx("position lock: %dD, satellites: %d, last update: %8.4fms ago", (int)_report_gps_pos.fix_type,
				_report_gps_pos.satellites_used, (double)(hrt_absolute_time() - _report_gps_pos.timestamp_position) / 1000.0);
		warnx("lat: %d, lon: %d, alt: %d", _report_gps_pos.lat, _report_gps_pos.lon, _report_gps_pos.alt);
		warnx("vel: %.2fm/s, %.2fm/s, %.2fm/s", (double)_report_gps_pos.vel_n_m_s,
			(double)_report_gps_pos.vel_e_m_s, (double)_report_gps_pos.vel_d_m_s);
		warnx("eph: %.2fm, epv: %.2fm", (double)_report_gps_pos.eph, (double)_report_gps_pos.epv);
		warnx("rate position: \t%6.2f Hz", (double)_Helper->get_position_update_rate());
		warnx("rate velocity: \t%6.2f Hz", (double)_Helper->get_velocity_update_rate());
		warnx("rate publication:\t%6.2f Hz", (double)_rate);

	}

	usleep(100000);
}

/**
 * Local functions in support of the shell command.
 */
namespace gps
{

GPS	*g_dev;

void	start(const char *uart_path, bool fake_gps, bool enable_sat_info);
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start(const char *uart_path, bool fake_gps, bool enable_sat_info)
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new GPS(uart_path, fake_gps, enable_sat_info);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(GPS0_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		errx(1, "open: %s\n", GPS0_DEVICE_PATH);
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "start failed");
}

/**
 * Stop the driver.
 */
void
stop()
{
	delete g_dev;
	g_dev = nullptr;

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(GPS0_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "reset failed");

	exit(0);
}

/**
 * Print the status of the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "not running");

	g_dev->print_info();

	exit(0);
}

} // namespace


int
gps_main(int argc, char *argv[])
{

	/* set to default */
	const char *device_name = GPS_DEFAULT_UART_PORT;
	bool fake_gps = false;
	bool enable_sat_info = false;

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		/* work around getopt unreliability */
		if (argc > 3) {
			if (!strcmp(argv[2], "-d")) {
				device_name = argv[3];

			} else {
				goto out;
			}
		}

		/* Detect fake gps option */
		for (int i = 2; i < argc; i++) {
			if (!strcmp(argv[i], "-f"))
				fake_gps = true;
		}

		/* Detect sat info option */
		for (int i = 2; i < argc; i++) {
			if (!strcmp(argv[i], "-s"))
				enable_sat_info = true;
		}

		gps::start(device_name, fake_gps, enable_sat_info);
	}

	if (!strcmp(argv[1], "stop"))
		gps::stop();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		gps::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		gps::reset();

	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status"))
		gps::info();

out:
	errx(1, "unrecognized command, try 'start', 'stop', 'test', 'reset' or 'status' [-d /dev/ttyS0-n][-f][-s]");
}
