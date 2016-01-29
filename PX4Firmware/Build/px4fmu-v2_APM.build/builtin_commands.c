/* builtin command list - automatically generated, do not edit */
#include <nuttx/config.h>
#include <nuttx/binfmt/builtin.h>
extern int sercon_main(int argc, char *argv[]);
extern int serdis_main(int argc, char *argv[]);
extern int ArduPilot_main(int argc, char *argv[]);
extern int adc_main(int argc, char *argv[]);
extern int batt_smbus_main(int argc, char *argv[]);
extern int bl_update_main(int argc, char *argv[]);
extern int boardinfo_main(int argc, char *argv[]);
extern int ets_airspeed_main(int argc, char *argv[]);
extern int fmu_main(int argc, char *argv[]);
extern int hmc5883_main(int argc, char *argv[]);
extern int l3gd20_main(int argc, char *argv[]);
extern int ll40ls_main(int argc, char *argv[]);
extern int lsm303d_main(int argc, char *argv[]);
extern int mb12xx_main(int argc, char *argv[]);
extern int meas_airspeed_main(int argc, char *argv[]);
extern int mixer_main(int argc, char *argv[]);
extern int mkblctrl_main(int argc, char *argv[]);
extern int motor_test_main(int argc, char *argv[]);
extern int mpu6000_main(int argc, char *argv[]);
extern int ms5611_main(int argc, char *argv[]);
extern int mtd_main(int argc, char *argv[]);
extern int nshterm_main(int argc, char *argv[]);
extern int oreoled_main(int argc, char *argv[]);
extern int perf_main(int argc, char *argv[]);
extern int pwm_main(int argc, char *argv[]);
extern int pwm_input_main(int argc, char *argv[]);
extern int px4flow_main(int argc, char *argv[]);
extern int px4io_main(int argc, char *argv[]);
extern int reboot_main(int argc, char *argv[]);
extern int reflect_main(int argc, char *argv[]);
extern int rgbled_main(int argc, char *argv[]);
extern int teensysense_main(int argc, char *argv[]);
extern int tone_alarm_main(int argc, char *argv[]);
extern int top_main(int argc, char *argv[]);
extern int trone_main(int argc, char *argv[]);
extern int uorb_main(int argc, char *argv[]);
extern int usb_connected_main(int argc, char *argv[]);
extern int ver_main(int argc, char *argv[]);
const struct builtin_s g_builtins[] = {
    {"sercon", SCHED_PRIORITY_DEFAULT, 2048, sercon_main},
    {"serdis", SCHED_PRIORITY_DEFAULT, 2048, serdis_main},
    {"ArduPilot", SCHED_PRIORITY_DEFAULT, 4096, ArduPilot_main},
    {"adc", SCHED_PRIORITY_DEFAULT, CONFIG_PTHREAD_STACK_DEFAULT, adc_main},
    {"batt_smbus", SCHED_PRIORITY_DEFAULT, CONFIG_PTHREAD_STACK_DEFAULT, batt_smbus_main},
    {"bl_update", SCHED_PRIORITY_DEFAULT, 4096, bl_update_main},
    {"boardinfo", SCHED_PRIORITY_DEFAULT, CONFIG_PTHREAD_STACK_DEFAULT, boardinfo_main},
    {"ets_airspeed", SCHED_PRIORITY_DEFAULT, 1200, ets_airspeed_main},
    {"fmu", SCHED_PRIORITY_DEFAULT, 1200, fmu_main},
    {"hmc5883", SCHED_PRIORITY_DEFAULT, 1200, hmc5883_main},
    {"l3gd20", SCHED_PRIORITY_DEFAULT, 1200, l3gd20_main},
    {"ll40ls", SCHED_PRIORITY_DEFAULT, CONFIG_PTHREAD_STACK_DEFAULT, ll40ls_main},
    {"lsm303d", SCHED_PRIORITY_DEFAULT, 1200, lsm303d_main},
    {"mb12xx", SCHED_PRIORITY_DEFAULT, CONFIG_PTHREAD_STACK_DEFAULT, mb12xx_main},
    {"meas_airspeed", SCHED_PRIORITY_DEFAULT, 1200, meas_airspeed_main},
    {"mixer", SCHED_PRIORITY_DEFAULT, 4096, mixer_main},
    {"mkblctrl", SCHED_PRIORITY_DEFAULT, CONFIG_PTHREAD_STACK_DEFAULT, mkblctrl_main},
    {"motor_test", SCHED_PRIORITY_DEFAULT, 4096, motor_test_main},
    {"mpu6000", SCHED_PRIORITY_DEFAULT, 1200, mpu6000_main},
    {"ms5611", SCHED_PRIORITY_DEFAULT, CONFIG_PTHREAD_STACK_DEFAULT, ms5611_main},
    {"mtd", SCHED_PRIORITY_DEFAULT, CONFIG_PTHREAD_STACK_DEFAULT, mtd_main},
    {"nshterm", SCHED_PRIORITY_DEFAULT-30, 1500, nshterm_main},
    {"oreoled", SCHED_PRIORITY_DEFAULT, CONFIG_PTHREAD_STACK_DEFAULT, oreoled_main},
    {"perf", SCHED_PRIORITY_DEFAULT, 1800, perf_main},
    {"pwm", SCHED_PRIORITY_DEFAULT, 1800, pwm_main},
    {"pwm_input", SCHED_PRIORITY_DEFAULT, CONFIG_PTHREAD_STACK_DEFAULT, pwm_input_main},
    {"px4flow", SCHED_PRIORITY_DEFAULT, CONFIG_PTHREAD_STACK_DEFAULT, px4flow_main},
    {"px4io", SCHED_PRIORITY_DEFAULT, 1200, px4io_main},
    {"reboot", SCHED_PRIORITY_DEFAULT, 800, reboot_main},
    {"reflect", SCHED_PRIORITY_DEFAULT, CONFIG_PTHREAD_STACK_DEFAULT, reflect_main},
    {"rgbled", SCHED_PRIORITY_DEFAULT, CONFIG_PTHREAD_STACK_DEFAULT, rgbled_main},
    {"teensysense", SCHED_PRIORITY_DEFAULT, CONFIG_PTHREAD_STACK_DEFAULT, teensysense_main},
    {"tone_alarm", SCHED_PRIORITY_DEFAULT, CONFIG_PTHREAD_STACK_DEFAULT, tone_alarm_main},
    {"top", SCHED_PRIORITY_DEFAULT, 1700, top_main},
    {"trone", SCHED_PRIORITY_DEFAULT, 1200, trone_main},
    {"uorb", SCHED_PRIORITY_DEFAULT, 2048, uorb_main},
    {"usb_connected", SCHED_PRIORITY_DEFAULT, CONFIG_PTHREAD_STACK_DEFAULT, usb_connected_main},
    {"ver", SCHED_PRIORITY_DEFAULT, 1024, ver_main},
    {NULL, 0, 0, NULL}
};
const int g_builtin_count = 38;
