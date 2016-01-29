/**
 * @file px4_teensytest.cpp
 * Minimal application example for PX4 autopilot.
 */
 
#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <drivers/drv_teensysense.h>
 
__EXPORT int px4_teensytest_main(int argc, char *argv[]);
 
int px4_teensytest_main(int argc, char *argv[])
{
	printf("Hello Sky!\n");
	int fd = open(TEENSY0_DEVICE_PATH, O_RDONLY);
	printf("fd: %d\n", fd);
	ioctl(fd, TEENSY_SENSOR_READ, 0);
	struct teensy_sensor_report report;
	report.type = 1;
	report.value = 42;
	printf("report struct size: %d\n", sizeof(report));
	ssize_t nobytes = read(fd, &report, sizeof(report));
	printf("read %d bytes\n", nobytes);
	printf("got value: %d\n", report.value);
	close(fd);
	return OK;
}