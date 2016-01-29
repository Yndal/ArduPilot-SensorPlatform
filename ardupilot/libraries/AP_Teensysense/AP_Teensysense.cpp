#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include "AP_Teensysense.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <drivers/drv_teensysense.h>
#include <stdio.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_Teensysense::AP_Teensysense() : 
	_initialised(false) 
{
	
}

void AP_Teensysense::init() {
	_sensor_fd = open(TEENSY0_DEVICE_PATH, O_RDONLY);
	_initialised = true;
	//measure();
}

bool AP_Teensysense::read(int16_t *val) {
	if (_initialised) {
		if (_sensor_fd != -1) {
			struct teensy_sensor_report report;
			::read(_sensor_fd, &report, sizeof(report));
			*val = report.i_value;

			//hal.console->printf("Value: %d\n", report.i_value);

			return report.isnew;
		}
		else {
			hal.console->println("Teensy Error: sysfs");
		}
	}
	else {
		hal.console->println("Teensy Error: not initialized");
	}
	return -1;
}

void AP_Teensysense::measure(){
	if(_initialised){
		if(_sensor_fd != -1){
			::read(_sensor_fd, nullptr, 0);
		} else {
			hal.console->println("Teensy Error: sysfs");
		}
	} else {
		hal.console->println("Teensy Error: not initialized");
	}
}


#endif