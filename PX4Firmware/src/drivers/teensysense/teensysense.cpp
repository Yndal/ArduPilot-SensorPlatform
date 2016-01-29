/**
 * @file teensysense.cpp
 *
 * Driver for Teensy I2C slave
 *
 * @author Lars Toft Jacobsen <lars@boxed.dk> <latj@itu.dk>
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>

#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <board_config.h>

#include <drivers/drv_teensysense.h>

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

//#define ADDR			PX4_I2C_OBDEV_TEENSY	/**< I2C adress of Teensy 3.x slave */
#define ADDR            0x44
#define CMD_MEASURE		0x10
#define CMD_COLLECT		0x11
#define CMD_TYPE		0x20
#define CMD_TEST		0x40
#define TEENSY_CONVERSION_INTERVAL	(5000)	/* milliseconds */

class TEENSYSENSE : public device::I2C
{
public:
	TEENSYSENSE(int bus, int teensysense);
	virtual ~TEENSYSENSE();

	virtual int		init();
	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	//virtual ssize_t	write(struct file *filp, const char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);
	
private:
	work_s			_work;

	bool			_running;
	bool			_should_run;
	uint8_t			_type;
	bool			_isnew;
	bool			_healthy;
	//int16_t			_conversionInterval = TEENSY_CONVERSION_INTERVAL;

	struct teensy_sensor_report _report;

	void			start();
	void			stop();
	static void		cycle_trampoline(void *arg);
	void			cycle();
	int				test();
	void			readSingle();
	void			measure();
	void 			collect();
};


namespace
{
TEENSYSENSE *g_teensysense = nullptr;
}

void teensysense_usage();

extern "C" __EXPORT int teensysense_main(int argc, char *argv[]);

TEENSYSENSE::TEENSYSENSE(int bus, int teensysense) :
	I2C("teensysense", TEENSY0_DEVICE_PATH, bus, teensysense, 100000 /* maximum speed supported */),
	_running(false),
	_should_run(true),
	_healthy(false),
	_isnew(false)
{
	memset(&_work, 0, sizeof(_work));
}

TEENSYSENSE::~TEENSYSENSE()
{
}

int
TEENSYSENSE::init()
{
	int ret;
	ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	// ask for type - use as liveness indicator
	// const uint8_t msg = CMD_TYPE;
	// uint8_t buf[4];
	// int result = transfer(&msg, 1, buf, 1);
	// if (result == OK) {
	// 	_type = buf[0];
	// 	_healthy = true;
	// }
	// else {
	// 	return result;
	// }
	_type = 0;
	_healthy = true;
	// we've made it this far
	warnx("Teensy initialized");
	return OK;
}

int
TEENSYSENSE::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret = -EINVAL;

	switch (cmd) {
		case TEENSY_SENSOR_START:
			start();
			ret = OK;
			break;
		case TEENSY_SENSOR_STOP:
			stop();
			ret = OK;
			break;
		case TEENSY_SENSOR_TEST:
			warnx("Starting test");
			ret = test();
			break;
		case TEENSY_SENSOR_READ:
			warnx("Requesting single value");
			readSingle();
			ret = OK;
			break;
		default:
			ret = CDev::ioctl(filp, cmd, arg);
			break;
	}

	return ret;
}

ssize_t
TEENSYSENSE::read(struct file *filp, char *buffer, size_t buflen)
{
	if(buflen == 0) {
		measure();
		return 0;
	} else {
		//Collect the data
		collect(); 
		//Copy and return these data
		struct teensy_sensor_report *teensy_buf = reinterpret_cast<struct teensy_sensor_report *>(buffer);
		memcpy(teensy_buf, &_report, buflen);
		return sizeof(*teensy_buf);
	}
}

/*ssize_t	
TEENSYSENSE::write(struct file *filp, const char *buffer, size_t buflen){
	const uint8_t cmd = CMD_MEASURE;
	transfer(&cmd, 1, nullptr, 0);
	measure();
	return 0;

}*/


void
TEENSYSENSE::start()
{
	warnx("Scheduling cycle");
	//measure();
	//work_queue(LPWORK, &_work, (worker_t)&TEENSYSENSE::cycle_trampoline, this, 5);
}

void
TEENSYSENSE::stop()
{
	work_cancel(LPWORK, &_work);
}

void
TEENSYSENSE::cycle_trampoline(void *arg)
{
	
	TEENSYSENSE *dev = (TEENSYSENSE *)arg;

	dev->cycle();
}


/**
 * Main loop function
 */
void
TEENSYSENSE::cycle()
{
	// if (!_should_run) {
	// 	_running = false;
	// 	return;
	// }

	/*measure();
	//usleep(_conversionInterval);//TEENSY_CONVERSION_INTERVAL);
	/collect();*/

	/* re-queue ourselves to run again later */
	/*_running = true;
	work_queue(LPWORK, &_work, (worker_t)&TEENSYSENSE::cycle_trampoline, this, USEC2TICK(1000000));
	*/
}

int
TEENSYSENSE::test()
{
	const uint8_t msg = CMD_TEST;
	uint8_t buf[4];
	// uint8_t addr = ADDR;
	warnx("Probing test register...");
	int result = transfer(&msg, 1, buf, 1);
	if (result == OK) {
		warnx("That went well!");
		warnx("The answer to life, the universe and everything: %d", buf[0]);
	}
	else {
		warnx("Oh no! No answer :(");
	}

	return result;
}

void
TEENSYSENSE::readSingle()
{
	warnx("Read single value: %d", _report.i_value);
}

void
TEENSYSENSE::collect()
{
	const uint8_t cmd = CMD_COLLECT;
	uint8_t buf[3];

	transfer(&cmd, 1, buf, 3); // read three bytes
	int16_t result = ((buf[0] << 8) | buf[1]); // unpack int result
	_isnew = ((buf[2] & 0x01) == 1); // set new flag (true if new value)

	// put data in report 
	_report.i_value = result;
	_report.isnew = _isnew;
}

void
TEENSYSENSE::measure()
{
	const uint8_t cmd = CMD_MEASURE;
	transfer(&cmd, 1, nullptr, 0);
}

void
teensysense_usage()
{
	warnx("missing command: try 'start', 'test', 'read'");
	warnx("options:");
	warnx("    -b i2cbus (%d)", PX4_I2C_BUS_EXPANSION);
	warnx("    -a addr (0x%x)", ADDR);
}

int
teensysense_main(int argc, char *argv[])
{
	int i2cdevice = -1;
	int teensyadr = ADDR; /* 7bit */

	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "a:b:")) != EOF) {
		switch (ch) {
		case 'a':
			teensyadr = strtol(optarg, NULL, 0);
			break;

		case 'b':
			i2cdevice = strtol(optarg, NULL, 0);
			break;

		default:
			teensysense_usage();
			exit(0);
		}
	}

        if (optind >= argc) {
            teensysense_usage();
            exit(1);
        }

	const char *verb = argv[optind];

	int fd;
	int ret;

	if (!strcmp(verb, "start")) {
		if (g_teensysense != nullptr)
			errx(1, "already started");

		if (i2cdevice == -1) {
			i2cdevice = PX4_I2C_BUS_EXPANSION;
			g_teensysense = new TEENSYSENSE(PX4_I2C_BUS_EXPANSION, teensyadr);

			if (g_teensysense != nullptr && OK != g_teensysense->init()) {
				delete g_teensysense;
				g_teensysense = nullptr;
			}

			if (g_teensysense == nullptr) {
				errx(1, "init failed");
			}
		}

		if (g_teensysense == nullptr) {
			g_teensysense = new TEENSYSENSE(i2cdevice, teensyadr);

			if (g_teensysense == nullptr)
				errx(1, "new failed");

			if (OK != g_teensysense->init()) {
				delete g_teensysense;
				g_teensysense = nullptr;
				errx(1, "init failed");
			}
		}

		fd = open(TEENSY0_DEVICE_PATH, 0);
		if (fd == -1) {
			errx(1, "Unable to open " TEENSY0_DEVICE_PATH);
		}
		ret = ioctl(fd, TEENSY_SENSOR_START, 0);
		close(fd);
		exit(0);
	}

	/* need the driver past this point */
	if (g_teensysense == nullptr) {
		warnx("not started");
		teensysense_usage();
		exit(1);
	}

	if (!strcmp(verb, "test")) {
		fd = open(TEENSY0_DEVICE_PATH, O_RDONLY);

		if (fd == -1) {
			errx(1, "Unable to open " TEENSY0_DEVICE_PATH);
		}

		ret = ioctl(fd, TEENSY_SENSOR_TEST, 0);;

		close(fd);
		exit(ret);
	}

	if (!strcmp(verb, "read")) {
		fd = open(TEENSY0_DEVICE_PATH, O_RDONLY);

		if (fd == -1) {
			errx(1, "Unable to open " TEENSY0_DEVICE_PATH);
		}

		ret = ioctl(fd, TEENSY_SENSOR_READ, 0);;

		close(fd);
		exit(ret);
	}

	teensysense_usage();
	exit(0);
}
