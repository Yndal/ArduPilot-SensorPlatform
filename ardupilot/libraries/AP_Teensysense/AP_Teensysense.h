#ifndef AP_TEENSYSENSE_H
#define AP_TEENSYSENSE_H

#include <inttypes.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_Common.h>

class AP_Teensysense {
public:
	AP_Teensysense();
	void init();
	bool read(int16_t*);
	void measure();

private:
	int16_t _sensor_value;
	int 	_sensor_fd;
	bool	_initialised;

};

#endif