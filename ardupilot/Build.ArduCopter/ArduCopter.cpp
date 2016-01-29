// BUILDROOT=/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/Build.ArduCopter HAL_BOARD=HAL_BOARD_PX4 HAL_BOARD_SUBTYPE= TOOLCHAIN=NATIVE EXTRAFLAGS=-DGIT_VERSION="3af13266" -DNUTTX_GIT_VERSION="d48fa307" -DPX4_GIT_VERSION="60013b22"
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/ArduCopter.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "APM:Copter V3.3-dev"
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *  ArduCopter Version 3.0
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini 
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen, 
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to:	Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera       :Auto Compass Declination
 *  Amilcar Lucas       :Camera mount library
 *  Andrew Tridgell     :General development, Mavlink Support
 *  Angel Fernandez     :Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid     :Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel	        :DCM, Libraries, Control law advice
 *  Emile Castelnuovo   :VRBrain port, bug fixes
 *  Gregory Fletcher    :Camera mount orientation math
 *  Guntars             :Arming safety suggestion
 *  HappyKillmore       :Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle         :Alpha testing
 *  James Goppert       :Mavlink Support
 *  Jani Hiriven        :Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland	:PPM Encoder
 *  Jose Julio          :Stabilization Control laws, MPU6k driver
 *  Julien Dubois       :PosHold flight mode
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine          :Tri Support, Graphics
 *  Leonard Hall        :Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini     :Lead tester
 *  Michael Oborne      :Mission Planner GCS
 *  Mike Smith          :Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraction Layer (HAL)
 *  Robert Lefebvre     :Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  Sandro Tognana      :PosHold flight mode
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/diydrones/ardupilot/graphs/contributors
 *  Wiki: http://copter.ardupilot.com/
 *  Requires modified version of Arduino, which can be found here: http://ardupilot.com/downloads/?category=6
 *
 */

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdio.h>
#include <stdarg.h>

// Common dependencies
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Menu.h>
#include <AP_Param.h>
#include <StorageManager.h>
// AP_HAL
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_VRBRAIN.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>

// Application dependencies
#include <GCS.h>
#include <GCS_MAVLink.h>        // MAVLink GCS definitions
#include <AP_SerialManager.h>   // Serial manager library
#include <AP_GPS.h>             // ArduPilot GPS library
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_AHRS.h>
#include <AP_NavEKF.h>
#include <AP_Mission.h>         // Mission command library
#include <AP_Rally.h>           // Rally point library
#include <AC_PID.h>             // PID library
#include <AC_PI_2D.h>           // PID library (2-axis)
#include <AC_HELI_PID.h>        // Heli specific Rate PID library
#include <AC_P.h>               // P library
#include <AC_AttitudeControl.h> // Attitude control library
#include <AC_AttitudeControl_Heli.h> // Attitude control library for traditional helicopter
#include <AC_PosControl.h>      // Position control library
#include <RC_Channel.h>         // RC Channel Library
#include <AP_Motors.h>          // AP Motors library
#include <AP_RangeFinder.h>     // Range finder library
#include <AP_OpticalFlow.h>     // Optical Flow library
#include <Filter.h>             // Filter library
#include <AP_Buffer.h>          // APM FIFO Buffer
#include <AP_Relay.h>           // APM relay
#include <AP_ServoRelayEvents.h>
#include <AP_Camera.h>          // Photo or video camera
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Airspeed.h>        // needed for AHRS build
#include <AP_Vehicle.h>         // needed for AHRS build
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
#include <AC_WPNav.h>     		// ArduCopter waypoint navigation library
#include <AC_Circle.h>          // circle navigation library
#include <AP_Declination.h>     // ArduPilot Mega Declination Helper Library
#include <AC_Fence.h>           // Arducopter Fence library
#include <SITL.h>               // software in the loop support
#include <AP_Scheduler.h>       // main loop scheduler
#include <AP_RCMapper.h>        // RC input mapping library
#include <AP_Notify.h>          // Notify library
#include <AP_BattMonitor.h>     // Battery monitor library
#include <AP_BoardConfig.h>     // board configuration library
#include <AP_Frsky_Telem.h>
#include <AP_Teensysense.h>
#if SPRAYER == ENABLED
#include <AC_Sprayer.h>         // crop sprayer library
#endif
#if EPM_ENABLED == ENABLED
#include <AP_EPM.h>				// EPM cargo gripper stuff
#endif
#if PARACHUTE == ENABLED
#include <AP_Parachute.h>		// Parachute release library
#endif
#include <AP_LandingGear.h>     // Landing Gear library
#include <AP_Terrain.h>

// AP_HAL to Arduino compatibility layer
#include "compat.h"
// Configuration
#include "defines.h"
#include "config.h"
#include "config_channels.h"

// key aircraft parameters passed to multiple libraries
#line 1 "autogenerated"
  void setup()  ;
 static void compass_accumulate(void) ;
 static void barometer_accumulate(void) ;
  static void perf_update(void) ;
  void loop() ;
 static void fast_loop() ;
 static void rc_loop() ;
 static void throttle_loop() ;
 static void update_mount() ;
 static void update_batt_compass(void) ;
 static void ten_hz_logging_loop() ;
 static void fifty_hz_logging_loop() ;
 static void full_rate_logging_loop() ;
 static void three_hz_loop() ;
 static void one_hz_loop() ;
 static void update_GPS(void) ;
  static void init_simple_bearing() ;
 void update_simple_mode(void) ;
 void update_super_simple_bearing(bool force_update) ;
  static void read_AHRS(void) ;
 static void update_altitude() ;
 void set_home_state(enum HomeState new_home_state) ;
 bool home_is_set() ;
 void set_auto_armed(bool b) ;
 void set_simple_mode(uint8_t b) ;
 static void set_failsafe_radio(bool b) ;
 void set_failsafe_battery(bool b) ;
 static void set_failsafe_gcs(bool b) ;
 void set_land_complete(bool b) ;
 void set_land_complete_maybe(bool b) ;
  void set_pre_arm_check(bool b) ;
  void set_pre_arm_rc_check(bool b) ;
 float get_smoothing_gain() ;
 static void get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out) ;
 static float get_pilot_desired_yaw_rate(int16_t stick_angle) ;
 static float get_roi_yaw() ;
  static float get_look_ahead_yaw() ;
 static void update_thr_average() ;
 static void set_throttle_takeoff() ;
 static int16_t get_pilot_desired_throttle(int16_t throttle_control) ;
 static int16_t get_pilot_desired_climb_rate(int16_t throttle_control) ;
 static int16_t get_non_takeoff_throttle() ;
 static int16_t get_throttle_pre_takeoff(int16_t throttle_control) ;
 static float get_surface_tracking_climb_rate(int16_t target_rate, float current_alt_target, float dt) ;
 static void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle) ;
 static void update_poscon_alt_max() ;
  static void gcs_send_heartbeat(void) ;
  static void gcs_send_deferred(void) ;
  static NOINLINE void send_heartbeat(mavlink_channel_t chan) ;
  static NOINLINE void send_attitude(mavlink_channel_t chan) ;
 static NOINLINE void send_limits_status(mavlink_channel_t chan) ;
   static NOINLINE void send_extended_status1(mavlink_channel_t chan) ;
  static void NOINLINE send_location(mavlink_channel_t chan) ;
  static void NOINLINE send_nav_controller_output(mavlink_channel_t chan) ;
 static void NOINLINE send_simstate(mavlink_channel_t chan) ;
  static void NOINLINE send_hwstatus(mavlink_channel_t chan) ;
  static void NOINLINE send_servo_out(mavlink_channel_t chan) ;
  static void NOINLINE send_radio_out(mavlink_channel_t chan) ;
  static void NOINLINE send_vfr_hud(mavlink_channel_t chan) ;
  static void NOINLINE send_current_waypoint(mavlink_channel_t chan) ;
 static void NOINLINE send_rangefinder(mavlink_channel_t chan) ;
  static void NOINLINE send_statustext(mavlink_channel_t chan) ;
 static bool telemetry_delayed(mavlink_channel_t chan) ;
 static void mavlink_delay_cb() ;
 static void gcs_send_message(enum ap_message id) ;
 static void gcs_data_stream_send(void) ;
 static void gcs_check_input(void) ;
  static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str) ;
  static bool print_log_menu(void) ;
  static void do_erase_logs(void) ;
 static void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float rate_target, float rate_min, float rate_max, float new_gain_rp, float new_gain_rd, float new_gain_sp) ;
 static void Log_Write_AutoTuneDetails(float angle_cd, float rate_cds) ;
 static void Log_Write_Current() ;
 static void Log_Write_Optflow() ;
 static void Log_Write_Nav_Tuning() ;
 static void Log_Write_Control_Tuning() ;
 static void Log_Write_Performance() ;
 static void Log_Write_Attitude() ;
 static void Log_Write_Rate() ;
 static void Log_Write_MotBatt() ;
 static void Log_Write_Startup() ;
 static void Log_Write_Event(uint8_t id) ;
 UNUSED_FUNCTION static void Log_Write_Data(uint8_t id, int16_t value) ;
 UNUSED_FUNCTION  static void Log_Write_Data(uint8_t id, uint16_t value) ;
 static void Log_Write_Data(uint8_t id, int32_t value) ;
 static void Log_Write_Data(uint8_t id, uint32_t value) ;
 UNUSED_FUNCTION static void Log_Write_Data(uint8_t id, float value) ;
 static void Log_Write_Error(uint8_t sub_system, uint8_t error_code) ;
  static void Log_Write_Baro(void) ;
 static void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page) ;
 static void start_logging()  ;
  static void Log_Write_Startup() ;
 static void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float rate_target, float rate_min, float rate_max, float new_gain_rp, float new_gain_rd, float new_gain_sp) ;
 static void Log_Write_AutoTuneDetails(float angle_cd, float rate_cds) ;
 static void Log_Write_Current() ;
 static void Log_Write_Attitude() ;
 static void Log_Write_Rate() ;
 static void Log_Write_MotBatt() ;
 static void Log_Write_Data(uint8_t id, int16_t value);
 static void Log_Write_Data(uint8_t id, uint16_t value);
 static void Log_Write_Data(uint8_t id, int32_t value);
 static void Log_Write_Data(uint8_t id, uint32_t value);
 static void Log_Write_Data(uint8_t id, float value);
 static void Log_Write_Event(uint8_t id);
 static void Log_Write_Optflow() ;
 static void Log_Write_Nav_Tuning() ;
 static void Log_Write_Control_Tuning() ;
 static void Log_Write_Performance() ;
 static void Log_Write_Error(uint8_t sub_system, uint8_t error_code) ;
 static void Log_Write_Baro(void) ;
  static void load_parameters(void) ;
 void userhook_init() ;
 void userhook_FastLoop() ;
 void userhook_50Hz() ;
 void userhook_MediumLoop() ;
 void userhook_SlowLoop() ;
 void userhook_SuperSlowLoop() ;
 static void update_home_from_EKF() ;
 static bool set_home_to_current_location() ;
 static bool set_home_to_current_location_and_lock() ;
 static bool set_home_and_lock(const Location& loc) ;
 static bool set_home(const Location& loc) ;
 static bool far_from_EKF_origin(const Location& loc) ;
 static void set_system_time_from_GPS() ;
 static void exit_mission() ;
 static void do_RTL(void) ;
 static bool verify_takeoff() ;
 static bool verify_land() ;
  static bool verify_loiter_unlimited() ;
 static bool verify_loiter_time() ;
 static bool verify_RTL() ;
  static bool verify_wait_delay() ;
  static bool verify_change_alt() ;
  static bool verify_within_distance() ;
 static bool verify_yaw() ;
 static void do_take_picture() ;
 static void log_picture() ;
 static uint8_t mavlink_compassmot(mavlink_channel_t chan) ;
  static void delay(uint32_t ms) ;
  static uint32_t millis() ;
  static uint32_t micros() ;
 static bool acro_init(bool ignore_checks) ;
 static void acro_run() ;
 static void get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out) ;
 static bool althold_init(bool ignore_checks) ;
 static void althold_run() ;
 static bool auto_init(bool ignore_checks) ;
 static void auto_run() ;
 static void auto_takeoff_start(float final_alt_above_home) ;
 static void auto_takeoff_run() ;
 static void auto_wp_start(const Vector3f& destination) ;
 static void auto_wp_run() ;
 static void auto_spline_run() ;
 static void auto_land_start() ;
 static void auto_land_start(const Vector3f& destination) ;
 static void auto_land_run() ;
 static void auto_rtl_start() ;
 void auto_rtl_run() ;
 static void auto_circle_movetoedge_start() ;
 static void auto_circle_start() ;
 void auto_circle_run() ;
 void auto_nav_guided_start() ;
 void auto_nav_guided_run() ;
 bool auto_loiter_start() ;
 void auto_loiter_run() ;
 uint8_t get_default_auto_yaw_mode(bool rtl) ;
 void set_auto_yaw_mode(uint8_t yaw_mode) ;
 static void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, uint8_t relative_angle) ;
 static void set_auto_yaw_roi(const Location &roi_location) ;
 float get_auto_heading(void) ;
 static bool autotune_init(bool ignore_checks) ;
 static void autotune_stop() ;
 static bool autotune_start(bool ignore_checks) ;
 static void autotune_run() ;
 static void autotune_attitude_control() ;
 static void autotune_backup_gains_and_initialise() ;
 static void autotune_load_orig_gains() ;
 static void autotune_load_tuned_gains() ;
 static void autotune_load_intra_test_gains() ;
 static void autotune_load_twitch_gains() ;
 static void autotune_save_tuning_gains() ;
 void autotune_update_gcs(uint8_t message_id) ;
 inline bool autotune_roll_enabled() ;
  inline bool autotune_pitch_enabled() ;
  inline bool autotune_yaw_enabled() ;
 void autotune_twitching_test(float measurement, float target, float &measurement_min, float &measurement_max) ;
 void autotune_updating_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max) ;
 void autotune_updating_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max) ;
 void autotune_updating_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio, float target, float measurement_max) ;
 void autotune_updating_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float target, float measurement_max) ;
 void autotune_updating_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max) ;
 void autotune_twitching_measure_acceleration(float &rate_of_change, float rate_measurement, float &rate_measurement_max) ;
 static bool circle_init(bool ignore_checks) ;
 static void circle_run() ;
 static bool drift_init(bool ignore_checks) ;
 static void drift_run() ;
 int16_t get_throttle_assist(float velz, int16_t pilot_throttle_scaled) ;
 static bool flip_init(bool ignore_checks) ;
 static void flip_run() ;
 static bool guided_init(bool ignore_checks) ;
 static void guided_takeoff_start(float final_alt_above_home) ;
 static void guided_pos_control_start() ;
 static void guided_vel_control_start() ;
 static void guided_posvel_control_start() ;
 static void guided_set_destination(const Vector3f& destination) ;
 static void guided_set_velocity(const Vector3f& velocity) ;
 static void guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity) ;
 static void guided_run() ;
 static void guided_takeoff_run() ;
 static void guided_pos_control_run() ;
 static void guided_vel_control_run() ;
 static void guided_posvel_control_run() ;
 static void guided_limit_clear() ;
 static void guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm) ;
 static void guided_limit_init_time_and_pos() ;
 static bool guided_limit_check() ;
 static bool land_init(bool ignore_checks) ;
 static void land_run() ;
 static void land_gps_run() ;
 static void land_nogps_run() ;
 static float get_land_descent_speed() ;
 static void land_do_not_use_GPS() ;
 static void set_mode_land_with_pause() ;
 static bool landing_with_GPS() ;
 static bool loiter_init(bool ignore_checks) ;
 static void loiter_run() ;
 static bool poshold_init(bool ignore_checks) ;
 static void poshold_run() ;
 static void poshold_update_pilot_lean_angle(float &lean_angle_filtered, float &lean_angle_raw) ;
 static int16_t poshold_mix_controls(float mix_ratio, int16_t first_control, int16_t second_control) ;
 static void poshold_update_brake_angle_from_velocity(int16_t &brake_angle, float velocity) ;
 static void poshold_update_wind_comp_estimate() ;
 static void poshold_get_wind_comp_lean_angles(int16_t &roll_angle, int16_t &pitch_angle) ;
 static void poshold_roll_controller_to_pilot_override() ;
 static void poshold_pitch_controller_to_pilot_override() ;
 static bool rtl_init(bool ignore_checks) ;
 static void rtl_run() ;
 static void rtl_climb_start() ;
 static void rtl_return_start() ;
 static void rtl_climb_return_run() ;
 static void rtl_loiterathome_start() ;
 static void rtl_loiterathome_run() ;
 static void rtl_descent_start() ;
 static void rtl_descent_run() ;
 static void rtl_land_start() ;
 static void rtl_land_run() ;
 static float get_RTL_alt() ;
 static bool sport_init(bool ignore_checks) ;
 static void sport_run() ;
 static bool stabilize_init(bool ignore_checks) ;
 static void stabilize_run() ;
 static bool stop_init(bool ignore_checks) ;
 static void stop_run() ;
 void crash_check() ;
 void parachute_check() ;
 static void parachute_release() ;
 static void parachute_manual_release() ;
 void ekf_check() ;
 static bool ekf_over_threshold() ;
 static void failsafe_ekf_event() ;
 static void failsafe_ekf_off_event(void) ;
 static void esc_calibration_startup_check() ;
 static void esc_calibration_passthrough() ;
 static void esc_calibration_auto() ;
 static void failsafe_radio_on_event() ;
 static void failsafe_radio_off_event() ;
  static void failsafe_battery_event(void) ;
 static void failsafe_gcs_check() ;
 static void failsafe_gcs_off_event(void) ;
 static void set_mode_RTL_or_land_with_pause() ;
  static void update_events() ;
 void failsafe_enable() ;
 void failsafe_disable() ;
 void failsafe_check() ;
 void fence_check() ;
 static void fence_send_mavlink_status(mavlink_channel_t chan) ;
 static bool set_mode(uint8_t mode) ;
 static void update_flight_mode() ;
 static void exit_mode(uint8_t old_control_mode, uint8_t new_control_mode) ;
 static bool mode_requires_GPS(uint8_t mode) ;
 static bool mode_has_manual_throttle(uint8_t mode) ;
 static bool mode_allows_arming(uint8_t mode, bool arming_from_gcs) ;
 static void notify_flight_mode(uint8_t mode) ;
 static void heli_init() ;
 static int16_t get_pilot_desired_collective(int16_t control_in) ;
 static void check_dynamic_flight(void) ;
 static void update_heli_control_dynamics(void) ;
 static void heli_update_landing_swash() ;
 static void heli_update_rotor_speed_targets() ;
 static bool heli_acro_init(bool ignore_checks) ;
 static void heli_acro_run() ;
 static void get_pilot_desired_yaw_rate(int16_t yaw_in, float &yaw_out) ;
 static bool heli_stabilize_init(bool ignore_checks) ;
 static void heli_stabilize_run() ;
 static void read_inertia() ;
 static void read_inertial_altitude() ;
 static bool land_complete_maybe() ;
 static void update_land_detector() ;
 static void update_throttle_low_comp() ;
 static void landinggear_update();
 static void update_notify() ;
 static void motor_test_output() ;
 static bool mavlink_motor_test_check(mavlink_channel_t chan, bool check_rc) ;
 static uint8_t mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type, uint16_t throttle_value, float timeout_sec) ;
 static void motor_test_stop() ;
 static void arm_motors_check() ;
 static void auto_disarm_check() ;
 static bool init_arm_motors(bool arming_from_gcs) ;
 static bool pre_arm_checks(bool display_failure) ;
 static void pre_arm_rc_checks() ;
 static bool pre_arm_gps_checks(bool display_failure) ;
 static bool arm_checks(bool display_failure, bool arming_from_gcs) ;
 static void init_disarm_motors() ;
 static void motors_output() ;
 static void run_nav_updates(void) ;
 static void calc_position() ;
 static void calc_distance_and_bearing() ;
 static void calc_wp_distance() ;
 static void calc_wp_bearing() ;
 static void calc_home_distance_and_bearing() ;
 static void run_autopilot() ;
 static void perf_info_reset() ;
 static void perf_ignore_this_loop() ;
 static void perf_info_check_loop_time(uint32_t time_in_micros) ;
 static uint16_t perf_info_get_num_loops() ;
 static uint32_t perf_info_get_max_time() ;
 static uint32_t perf_info_get_min_time() ;
 static uint16_t perf_info_get_num_long_running() ;
 Vector3f pv_location_to_vector(const Location& loc) ;
 Vector3f pv_location_to_vector_with_default(const Location& loc, const Vector3f& default_posvec) ;
 float pv_alt_above_origin(float alt_above_home_cm) ;
 float pv_alt_above_home(float alt_above_origin_cm) ;
 float pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination) ;
 float pv_get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination) ;
  static void default_dead_zones() ;
  static void init_rc_in() ;
 static void init_rc_out() ;
 void output_min() ;
  static void read_radio() ;
 static void set_throttle_and_failsafe(uint16_t throttle_pwm) ;
 static void set_throttle_zero_flag(int16_t throttle_control) ;
  static void init_barometer(bool full_calibration) ;
 static void read_barometer(void) ;
 static void init_sonar(void) ;
 static int16_t read_sonar(void) ;
 static void init_compass() ;
 static void init_optflow() ;
 static void update_optical_flow(void) ;
 static void read_battery(void) ;
 void read_receiver_rssi(void) ;
 void epm_update() ;
  static void report_batt_monitor() ;
  static void report_frame() ;
  static void report_radio() ;
  static void report_ins() ;
  static void report_flight_modes() ;
  void report_optflow() ;
  static void print_radio_values() ;
  static void print_switch(uint8_t p, uint8_t m, bool b) ;
  static void print_accel_offsets_and_scaling(void) ;
  static void print_gyro_offsets(void) ;
 static void report_compass() ;
  static void print_blanks(int16_t num) ;
  static void print_divider(void) ;
  static void print_enabled(bool b) ;
  static void report_version() ;
  static void read_control_switch() ;
 static bool check_if_auxsw_mode_used(uint8_t auxsw_mode_check) ;
 static bool check_duplicate_auxsw(void) ;
  static void reset_control_switch() ;
 static uint8_t read_3pos_switch(int16_t radio_in) ;
 static void read_aux_switches() ;
 static void init_aux_switches() ;
 static void init_aux_switch_function(int8_t ch_option, uint8_t ch_flag) ;
 static void do_aux_switch_function(int8_t ch_function, uint8_t ch_flag) ;
 static void save_trim() ;
 static void auto_trim() ;
  static void init_ardupilot() ;
 static void startup_ground(bool force_gyro_cal) ;
 static bool position_ok() ;
 static bool optflow_position_ok() ;
 static void update_auto_armed() ;
  static void check_usb_mux(void) ;
 static void frsky_telemetry_send(void) ;
 static bool should_log(uint32_t mask) ;
  static void print_hit_enter() ;
 static void tuning() ;
#line 170 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/ArduCopter.pde"
static AP_Vehicle::MultiCopter aparm;

// Local modules
#include "Parameters.h"

// Heli modules
#include "heli.h"

////////////////////////////////////////////////////////////////////////////////
// cliSerial
////////////////////////////////////////////////////////////////////////////////
// cliSerial isn't strictly necessary - it is an alias for hal.console. It may
// be deprecated in favor of hal.console in later releases.
static AP_HAL::BetterStream* cliSerial;

// N.B. we need to keep a static declaration which isn't guarded by macros
// at the top to cooperate with the prototype mangler.

////////////////////////////////////////////////////////////////////////////////
// AP_HAL instance
////////////////////////////////////////////////////////////////////////////////

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

////////////////////////////////////////////////////////////////////////////////
// Parameters
////////////////////////////////////////////////////////////////////////////////
//
// Global parameters are all contained within the 'g' class.
//
static Parameters g;

// main loop scheduler
static AP_Scheduler scheduler;

// AP_Notify instance
static AP_Notify notify;

// used to detect MAVLink acks from GCS to stop compassmot
static uint8_t command_ack_counter;

// has a log download started?
static bool in_log_download;

////////////////////////////////////////////////////////////////////////////////
// prototypes
////////////////////////////////////////////////////////////////////////////////
static void update_events(void);
static void print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode);
static void gcs_send_text_fmt(const prog_char_t *fmt, ...);


////////////////////////////////////////////////////////////////////////////////
// Dataflash
////////////////////////////////////////////////////////////////////////////////
#if defined(HAL_BOARD_LOG_DIRECTORY)
static DataFlash_File DataFlash(HAL_BOARD_LOG_DIRECTORY);
#else
static DataFlash_Empty DataFlash;
#endif


////////////////////////////////////////////////////////////////////////////////
// the rate we run the main loop at
////////////////////////////////////////////////////////////////////////////////
#if MAIN_LOOP_RATE == 400
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_400HZ;
#else
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_100HZ;
#endif

////////////////////////////////////////////////////////////////////////////////
// Sensors
////////////////////////////////////////////////////////////////////////////////
//
// There are three basic options related to flight sensor selection.
//
// - Normal flight mode. Real sensors are used.
// - HIL Attitude mode. Most sensors are disabled, as the HIL
//   protocol supplies attitude information directly.
// - HIL Sensors mode. Synthetic sensors are configured that
//   supply data from the simulation.
//

static AP_GPS  gps;

static AP_Teensysense teensy;

// flight modes convenience array
static AP_Int8 *flight_modes = &g.flight_mode1;

static AP_Baro barometer;

static Compass compass;

AP_InertialSensor ins;

////////////////////////////////////////////////////////////////////////////////
// SONAR
#if CONFIG_SONAR == ENABLED
static RangeFinder sonar;
static bool sonar_enabled = true; // enable user switch for sonar
#endif

// Inertial Navigation EKF
#if AP_AHRS_NAVEKF_AVAILABLE
AP_AHRS_NavEKF ahrs(ins, barometer, gps, sonar);
#else
AP_AHRS_DCM ahrs(ins, barometer, gps);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
SITL sitl;
#endif

// Mission library
// forward declaration to keep compiler happy
static bool start_command(const AP_Mission::Mission_Command& cmd);
static bool verify_command(const AP_Mission::Mission_Command& cmd);
static void exit_mission();
AP_Mission mission(ahrs, &start_command, &verify_command, &exit_mission);

////////////////////////////////////////////////////////////////////////////////
// Optical flow sensor
////////////////////////////////////////////////////////////////////////////////
#if OPTFLOW == ENABLED
static OpticalFlow optflow;
#endif

// gnd speed limit required to observe optical flow sensor limits
static float ekfGndSpdLimit;
// scale factor applied to velocity controller gain to prevent optical flow noise causing excessive angle demand noise
static float ekfNavVelGainScaler;

////////////////////////////////////////////////////////////////////////////////
// GCS selection
////////////////////////////////////////////////////////////////////////////////
static AP_SerialManager serial_manager;
static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;
static GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];

////////////////////////////////////////////////////////////////////////////////
// User variables
////////////////////////////////////////////////////////////////////////////////
#ifdef USERHOOK_VARIABLES
 #include USERHOOK_VARIABLES
#endif

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////

/* Radio values
 *               Channel assignments
 *                       1	Ailerons (rudder if no ailerons)
 *                       2	Elevator
 *                       3	Throttle
 *                       4	Rudder (if we have ailerons)
 *                       5	Mode - 3 position switch
 *                       6  User assignable
 *                       7	trainer switch - sets throttle nominal (toggle switch), sets accels to Level (hold > 1 second)
 *                       8	TBD
 *               Each Aux channel can be configured to have any of the available auxiliary functions assigned to it.
 *               See libraries/RC_Channel/RC_Channel_aux.h for more information
 */

//Documentation of GLobals:
static union {
    struct {
        uint8_t unused1             : 1; // 0
        uint8_t simple_mode         : 2; // 1,2     // This is the state of simple mode : 0 = disabled ; 1 = SIMPLE ; 2 = SUPERSIMPLE
        uint8_t pre_arm_rc_check    : 1; // 3       // true if rc input pre-arm checks have been completed successfully
        uint8_t pre_arm_check       : 1; // 4       // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
        uint8_t auto_armed          : 1; // 5       // stops auto missions from beginning until throttle is raised
        uint8_t logging_started     : 1; // 6       // true if dataflash logging has started
        uint8_t land_complete       : 1; // 7       // true if we have detected a landing
        uint8_t new_radio_frame     : 1; // 8       // Set true if we have new PWM data to act on from the Radio
        uint8_t CH7_flag            : 2; // 9,10    // ch7 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH8_flag            : 2; // 11,12   // ch8 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH9_flag            : 2; // 13,14   // ch9 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH10_flag           : 2; // 15,16   // ch10 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH11_flag           : 2; // 17,18   // ch11 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t CH12_flag           : 2; // 19,20   // ch12 aux switch : 0 is low or false, 1 is center or true, 2 is high
        uint8_t usb_connected       : 1; // 21      // true if APM is powered from USB connection
        uint8_t rc_receiver_present : 1; // 22      // true if we have an rc receiver present (i.e. if we've ever received an update
        uint8_t compass_mot         : 1; // 23      // true if we are currently performing compassmot calibration
        uint8_t motor_test          : 1; // 24      // true if we are currently performing the motors test
        uint8_t initialised         : 1; // 25      // true once the init_ardupilot function has completed.  Extended status to GCS is not sent until this completes
        uint8_t land_complete_maybe : 1; // 26      // true if we may have landed (less strict version of land_complete)
        uint8_t throttle_zero       : 1; // 27      // true if the throttle stick is at zero, debounced
        uint8_t system_time_set     : 1; // 28      // true if the system time has been set from the GPS
        uint8_t gps_base_pos_set    : 1; // 29      // true when the gps base position has been set (used for RTK gps only)
        enum HomeState home_state   : 2; // 30,31   // home status (unset, set, locked)
    };
    uint32_t value;
} ap;

////////////////////////////////////////////////////////////////////////////////
// Radio
////////////////////////////////////////////////////////////////////////////////
// This is the state of the flight control system
// There are multiple states defined such as STABILIZE, ACRO,
static int8_t control_mode = STABILIZE;
// Structure used to detect changes in the flight mode control switch
static struct {
    int8_t debounced_switch_position;   // currently used switch position
    int8_t last_switch_position;        // switch position in previous iteration
    uint32_t last_edge_time_ms;         // system time that switch position was last changed
} control_switch_state;

static RCMapper rcmap;

// board specific config
static AP_BoardConfig BoardConfig;

// receiver RSSI
static uint8_t receiver_rssi;

////////////////////////////////////////////////////////////////////////////////
// Failsafe
////////////////////////////////////////////////////////////////////////////////
static struct {
    uint8_t rc_override_active  : 1; // 0   // true if rc control are overwritten by ground station
    uint8_t radio               : 1; // 1   // A status flag for the radio failsafe
    uint8_t battery             : 1; // 2   // A status flag for the battery failsafe
    uint8_t gcs                 : 1; // 4   // A status flag for the ground station failsafe
    uint8_t ekf                 : 1; // 5   // true if ekf failsafe has occurred

    int8_t radio_counter;                  // number of iterations with throttle below throttle_fs_value

    uint32_t last_heartbeat_ms;             // the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
} failsafe;

////////////////////////////////////////////////////////////////////////////////
// Motor Output
////////////////////////////////////////////////////////////////////////////////
#if FRAME_CONFIG == QUAD_FRAME
 #define MOTOR_CLASS AP_MotorsQuad
#elif FRAME_CONFIG == TRI_FRAME
 #define MOTOR_CLASS AP_MotorsTri
#elif FRAME_CONFIG == HEXA_FRAME
 #define MOTOR_CLASS AP_MotorsHexa
#elif FRAME_CONFIG == Y6_FRAME
 #define MOTOR_CLASS AP_MotorsY6
#elif FRAME_CONFIG == OCTA_FRAME
 #define MOTOR_CLASS AP_MotorsOcta
#elif FRAME_CONFIG == OCTA_QUAD_FRAME
 #define MOTOR_CLASS AP_MotorsOctaQuad
#elif FRAME_CONFIG == HELI_FRAME
 #define MOTOR_CLASS AP_MotorsHeli
#elif FRAME_CONFIG == SINGLE_FRAME
 #define MOTOR_CLASS AP_MotorsSingle
#elif FRAME_CONFIG == COAX_FRAME
 #define MOTOR_CLASS AP_MotorsCoax
#else
 #error Unrecognised frame type
#endif

#if FRAME_CONFIG == HELI_FRAME  // helicopter constructor requires more arguments
static MOTOR_CLASS motors(g.rc_1, g.rc_2, g.rc_3, g.rc_4, g.rc_7, g.rc_8, g.heli_servo_1, g.heli_servo_2, g.heli_servo_3, g.heli_servo_4, MAIN_LOOP_RATE);
#elif FRAME_CONFIG == TRI_FRAME  // tri constructor requires additional rc_7 argument to allow tail servo reversing
static MOTOR_CLASS motors(g.rc_1, g.rc_2, g.rc_3, g.rc_4, g.rc_7, MAIN_LOOP_RATE);
#elif FRAME_CONFIG == SINGLE_FRAME  // single constructor requires extra servos for flaps
static MOTOR_CLASS motors(g.rc_1, g.rc_2, g.rc_3, g.rc_4, g.single_servo_1, g.single_servo_2, g.single_servo_3, g.single_servo_4, MAIN_LOOP_RATE);
#elif FRAME_CONFIG == COAX_FRAME  // single constructor requires extra servos for flaps
static MOTOR_CLASS motors(g.rc_1, g.rc_2, g.rc_3, g.rc_4, g.single_servo_1, g.single_servo_2, MAIN_LOOP_RATE);
#else
static MOTOR_CLASS motors(g.rc_1, g.rc_2, g.rc_3, g.rc_4, MAIN_LOOP_RATE);
#endif

////////////////////////////////////////////////////////////////////////////////
// GPS variables
////////////////////////////////////////////////////////////////////////////////
// We use atan2 and other trig techniques to calaculate angles
// We need to scale the longitude up to make these calcs work
// to account for decreasing distance between lines of longitude away from the equator
static float scaleLongUp = 1;
// Sometimes we need to remove the scaling for distance calcs
static float scaleLongDown = 1;

////////////////////////////////////////////////////////////////////////////////
// Location & Navigation
////////////////////////////////////////////////////////////////////////////////
// This is the angle from the copter to the next waypoint in centi-degrees
static int32_t wp_bearing;
// The location of home in relation to the copter in centi-degrees
static int32_t home_bearing;
// distance between plane and home in cm
static int32_t home_distance;
// distance between plane and next waypoint in cm.
static uint32_t wp_distance;
static uint8_t land_state;              // records state of land (flying to location, descending)

////////////////////////////////////////////////////////////////////////////////
// Auto
////////////////////////////////////////////////////////////////////////////////
static AutoMode auto_mode;   // controls which auto controller is run

////////////////////////////////////////////////////////////////////////////////
// Guided
////////////////////////////////////////////////////////////////////////////////
static GuidedMode guided_mode;  // controls which controller is run (pos or vel)

////////////////////////////////////////////////////////////////////////////////
// RTL
////////////////////////////////////////////////////////////////////////////////
RTLState rtl_state;  // records state of rtl (initial climb, returning home, etc)
bool rtl_state_complete; // set to true if the current state is completed

////////////////////////////////////////////////////////////////////////////////
// Circle
////////////////////////////////////////////////////////////////////////////////
bool circle_pilot_yaw_override; // true if pilot is overriding yaw

////////////////////////////////////////////////////////////////////////////////
// SIMPLE Mode
////////////////////////////////////////////////////////////////////////////////
// Used to track the orientation of the copter for Simple mode. This value is reset at each arming
// or in SuperSimple mode when the copter leaves a 20m radius from home.
static float simple_cos_yaw = 1.0;
static float simple_sin_yaw;
static int32_t super_simple_last_bearing;
static float super_simple_cos_yaw = 1.0;
static float super_simple_sin_yaw;


// Stores initial bearing when armed - initial simple bearing is modified in super simple mode so not suitable
static int32_t initial_armed_bearing;

////////////////////////////////////////////////////////////////////////////////
// Throttle variables
////////////////////////////////////////////////////////////////////////////////
static float throttle_average;              // estimated throttle required to hover
static int16_t desired_climb_rate;          // pilot desired climb rate - for logging purposes only


////////////////////////////////////////////////////////////////////////////////
// ACRO Mode
////////////////////////////////////////////////////////////////////////////////
static float acro_level_mix;                // scales back roll, pitch and yaw inversely proportional to input from pilot

////////////////////////////////////////////////////////////////////////////////
// Loiter control
////////////////////////////////////////////////////////////////////////////////
static uint16_t loiter_time_max;                // How long we should stay in Loiter Mode for mission scripting (time in seconds)
static uint32_t loiter_time;                    // How long have we been loitering - The start time in millis

////////////////////////////////////////////////////////////////////////////////
// Flip
////////////////////////////////////////////////////////////////////////////////
static Vector3f flip_orig_attitude;         // original copter attitude before flip

////////////////////////////////////////////////////////////////////////////////
// Battery Sensors
////////////////////////////////////////////////////////////////////////////////
static AP_BattMonitor battery;

////////////////////////////////////////////////////////////////////////////////
// FrSky telemetry support
#if FRSKY_TELEM_ENABLED == ENABLED
static AP_Frsky_Telem frsky_telemetry(ahrs, battery);
#endif

////////////////////////////////////////////////////////////////////////////////
// Altitude
////////////////////////////////////////////////////////////////////////////////
// The cm/s we are moving up or down based on filtered data - Positive = UP
static int16_t climb_rate;
// The altitude as reported by Sonar in cm - Values are 20 to 700 generally.
static int16_t sonar_alt;
static uint8_t sonar_alt_health;    // true if we can trust the altitude from the sonar
static float target_sonar_alt;      // desired altitude in cm above the ground
static int32_t baro_alt;            // barometer altitude in cm above home
static float baro_climbrate;        // barometer climbrate in cm/s
static LowPassFilterVector3f land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF); // accelerations for land detector test

////////////////////////////////////////////////////////////////////////////////
// 3D Location vectors
////////////////////////////////////////////////////////////////////////////////
// Current location of the copter (altitude is relative to home)
static struct   Location current_loc;


////////////////////////////////////////////////////////////////////////////////
// Navigation Yaw control
////////////////////////////////////////////////////////////////////////////////
// auto flight mode's yaw mode
static uint8_t auto_yaw_mode = AUTO_YAW_LOOK_AT_NEXT_WP;
// Yaw will point at this location if auto_yaw_mode is set to AUTO_YAW_ROI
static Vector3f roi_WP;
// bearing from current location to the yaw_look_at_WP
static float yaw_look_at_WP_bearing;
// yaw used for YAW_LOOK_AT_HEADING yaw_mode
static int32_t yaw_look_at_heading;
// Deg/s we should turn
static int16_t yaw_look_at_heading_slew;
// heading when in yaw_look_ahead_bearing
static float yaw_look_ahead_bearing;



////////////////////////////////////////////////////////////////////////////////
// Delay Mission Scripting Command
////////////////////////////////////////////////////////////////////////////////
static int32_t condition_value;  // used in condition commands (eg delay, change alt, etc.)
static uint32_t condition_start;


////////////////////////////////////////////////////////////////////////////////
// IMU variables
////////////////////////////////////////////////////////////////////////////////
// Integration time (in seconds) for the gyros (DCM algorithm)
// Updated with the fast loop
static float G_Dt = 0.02;

////////////////////////////////////////////////////////////////////////////////
// Inertial Navigation
////////////////////////////////////////////////////////////////////////////////
static AP_InertialNav_NavEKF inertial_nav(ahrs);

////////////////////////////////////////////////////////////////////////////////
// Attitude, Position and Waypoint navigation objects
// To-Do: move inertial nav up or other navigation variables down here
////////////////////////////////////////////////////////////////////////////////
#if FRAME_CONFIG == HELI_FRAME
AC_AttitudeControl_Heli attitude_control(ahrs, aparm, motors, g.p_stabilize_roll, g.p_stabilize_pitch, g.p_stabilize_yaw,
                        g.pid_rate_roll, g.pid_rate_pitch, g.pid_rate_yaw);
#else
AC_AttitudeControl attitude_control(ahrs, aparm, motors, g.p_stabilize_roll, g.p_stabilize_pitch, g.p_stabilize_yaw,
                        g.pid_rate_roll, g.pid_rate_pitch, g.pid_rate_yaw);
#endif
AC_PosControl pos_control(ahrs, inertial_nav, motors, attitude_control,
                        g.p_alt_hold, g.p_vel_z, g.pid_accel_z,
                        g.p_pos_xy, g.pi_vel_xy);
static AC_WPNav wp_nav(inertial_nav, ahrs, pos_control, attitude_control);
static AC_Circle circle_nav(inertial_nav, ahrs, pos_control);

////////////////////////////////////////////////////////////////////////////////
// Performance monitoring
////////////////////////////////////////////////////////////////////////////////
static int16_t pmTest1;

// System Timers
// --------------
// Time in microseconds of main control loop
static uint32_t fast_loopTimer;
// Counter of main loop executions.  Used for performance monitoring and failsafe processing
static uint16_t mainLoop_count;
// Loiter timer - Records how long we have been in loiter
static uint32_t rtl_loiter_start_time;

// Used to exit the roll and pitch auto trim function
static uint8_t auto_trim_counter;

// Reference to the relay object
static AP_Relay relay;

// handle repeated servo and relay events
static AP_ServoRelayEvents ServoRelayEvents(relay);

//Reference to the camera object (it uses the relay object inside it)
#if CAMERA == ENABLED
  static AP_Camera camera(&relay);
#endif

// a pin for reading the receiver RSSI voltage.
static AP_HAL::AnalogSource* rssi_analog_source;

#if CLI_ENABLED == ENABLED
    static int8_t   setup_show (uint8_t argc, const Menu::arg *argv);
#endif

// Camera/Antenna mount tracking and stabilisation stuff
// --------------------------------------
#if MOUNT == ENABLED
// current_loc uses the baro/gps soloution for altitude rather than gps only.
static AP_Mount camera_mount(ahrs, current_loc);
#endif

////////////////////////////////////////////////////////////////////////////////
// AC_Fence library to reduce fly-aways
////////////////////////////////////////////////////////////////////////////////
#if AC_FENCE == ENABLED
AC_Fence    fence(inertial_nav);
#endif

////////////////////////////////////////////////////////////////////////////////
// Rally library
////////////////////////////////////////////////////////////////////////////////
#if AC_RALLY == ENABLED
AP_Rally rally(ahrs);
#endif

////////////////////////////////////////////////////////////////////////////////
// Crop Sprayer
////////////////////////////////////////////////////////////////////////////////
#if SPRAYER == ENABLED
static AC_Sprayer sprayer(&inertial_nav);
#endif

////////////////////////////////////////////////////////////////////////////////
// EPM Cargo Griper
////////////////////////////////////////////////////////////////////////////////
#if EPM_ENABLED == ENABLED
static AP_EPM epm;
#endif

////////////////////////////////////////////////////////////////////////////////
// Parachute release
////////////////////////////////////////////////////////////////////////////////
#if PARACHUTE == ENABLED
static AP_Parachute parachute(relay);
#endif

////////////////////////////////////////////////////////////////////////////////
// Landing Gear Controller
////////////////////////////////////////////////////////////////////////////////
static AP_LandingGear landinggear;

////////////////////////////////////////////////////////////////////////////////
// terrain handling
#if AP_TERRAIN_AVAILABLE
AP_Terrain terrain(ahrs, mission, rally);
#endif

////////////////////////////////////////////////////////////////////////////////
// function definitions to keep compiler from complaining about undeclared functions
////////////////////////////////////////////////////////////////////////////////
static bool pre_arm_checks(bool display_failure);

////////////////////////////////////////////////////////////////////////////////
// Top-level logic
////////////////////////////////////////////////////////////////////////////////

// setup the var_info table
AP_Param param_loader(var_info);

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 2.5ms units) and the maximum time they are expected to take (in
  microseconds)
  1    = 400hz
  2    = 200hz
  4    = 100hz
  8    = 50hz
  20   = 20hz
  40   = 10hz
  133  = 3hz
  400  = 1hz
  4000 = 0.1hz
  
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { rc_loop,               4,     10 },
    { throttle_loop,         8,     45 },
    { update_GPS,            8,     90 },
#if OPTFLOW == ENABLED
    { update_optical_flow,   2,     20 },
#endif
    { update_batt_compass,  40,     72 },
    { read_aux_switches,    40,      5 },
    { arm_motors_check,     40,      1 },
    { auto_trim,            40,     14 },
    { update_altitude,      40,    100 },
    { run_nav_updates,       8,     80 },
    { update_thr_average,   40,     10 },
    { three_hz_loop,       133,      9 },
    { compass_accumulate,    8,     42 },
    { barometer_accumulate,  8,     25 },
#if FRAME_CONFIG == HELI_FRAME
    { check_dynamic_flight,  8,     10 },
#endif
    { update_notify,         8,     10 },
    { one_hz_loop,         400,     42 },
    { ekf_check,            40,      2 },
    { crash_check,          40,      2 },
    { landinggear_update,   40,      1 },
    { gcs_check_input,       1,    550 },
    { gcs_send_heartbeat,  400,    150 },
    { gcs_send_deferred,     8,    720 },
    { gcs_data_stream_send,  8,    950 },
#if COPTER_LEDS == ENABLED
    { update_copter_leds,   40,      5 },
#endif
    { update_mount,          8,     45 },
    { ten_hz_logging_loop,  40,     30 },
    { fifty_hz_logging_loop, 8,     22 },
    { full_rate_logging_loop,1,     22 },
    { perf_update,        4000,     20 },
    { read_receiver_rssi,   40,      5 },
#if FRSKY_TELEM_ENABLED == ENABLED
    { frsky_telemetry_send, 80,     10 },
#endif
#if EPM_ENABLED == ENABLED
    { epm_update,           40,     10 },
#endif
#ifdef USERHOOK_FASTLOOP
    { userhook_FastLoop,     4,     10 },
#endif
#ifdef USERHOOK_50HZLOOP
    { userhook_50Hz,         8,     10 },
#endif
#ifdef USERHOOK_MEDIUMLOOP
    { userhook_MediumLoop,  40,     10 },
#endif
#ifdef USERHOOK_SLOWLOOP
    { userhook_SlowLoop,    120,    10 },
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    { userhook_SuperSlowLoop,400,   10 },
#endif
};

void setup() 
{
    cliSerial = hal.console;

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    // setup storage layout for copter
    StorageManager::set_layout_copter();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));

    // setup initial performance counters
    perf_info_reset();
    fast_loopTimer = hal.scheduler->micros();
}

/*
  if the compass is enabled then try to accumulate a reading
 */
static void compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }
}

/*
  try to accumulate a baro reading
 */
static void barometer_accumulate(void)
{
    barometer.accumulate();
}

static void perf_update(void)
{
    if (should_log(MASK_LOG_PM))
        Log_Write_Performance();
    if (scheduler.debug()) {
        gcs_send_text_fmt(PSTR("PERF: %u/%u %lu %lu\n"),
                          (unsigned)perf_info_get_num_long_running(),
                          (unsigned)perf_info_get_num_loops(),
                          (unsigned long)perf_info_get_max_time(),
                          (unsigned long)perf_info_get_min_time());
    }
    perf_info_reset();
    pmTest1 = 0;
}

void loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    uint32_t timer = micros();

    // check loop time
    perf_info_check_loop_time(timer - fast_loopTimer);

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.0f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // Execute the fast loop
    // ---------------------
    fast_loop();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    uint32_t time_available = (timer + MAIN_LOOP_MICROS) - micros();
    scheduler.run(time_available);
}


// Main loop - 100hz
static void fast_loop()
{

    // IMU DCM Algorithm
    // --------------------
    read_AHRS();

    // run low level rate controllers that only require IMU data
    attitude_control.rate_controller_run();
    
#if FRAME_CONFIG == HELI_FRAME
    update_heli_control_dynamics();
#endif //HELI_FRAME

    // send outputs to the motors library
    motors_output();

    // Inertial Nav
    // --------------------
    read_inertia();

    // run the attitude controllers
    update_flight_mode();

    // update home from EKF if necessary
    update_home_from_EKF();

    // check if we've landed
    update_land_detector();
}

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
static void rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    read_control_switch();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
static void throttle_loop()
{
    // get altitude and climb rate from inertial lib
    read_inertial_altitude();

    // update throttle_low_comp value (controls priority of throttle vs attitude control)
    update_throttle_low_comp();

    // check auto_armed status
    update_auto_armed();

#if FRAME_CONFIG == HELI_FRAME
    // update rotor speed
    heli_update_rotor_speed_targets();

    // update trad heli swash plate movement
    heli_update_landing_swash();
#endif
}

// update_mount - update camera mount position
// should be run at 50hz
static void update_mount()
{
#if MOUNT == ENABLED
    // update camera mount's position
    camera_mount.update();
#endif

#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
#endif
}

// update_batt_compass - read battery and compass
// should be called at 10hz
static void update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    read_battery();

    if(g.compass_enabled) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle((float)g.rc_3.servo_out/1000.0f);
        compass.read();
        // log compass information
        if (should_log(MASK_LOG_COMPASS)) {
            DataFlash.Log_Write_Compass(compass);
        }
    }
}

// ten_hz_logging_loop
// should be run at 10hz
static void ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_Rate();
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        Log_Write_MotBatt();
    }
    if (should_log(MASK_LOG_RCIN)) {
        DataFlash.Log_Write_RCIN();
    }
    if (should_log(MASK_LOG_RCOUT)) {
        DataFlash.Log_Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (mode_requires_GPS(control_mode) || landing_with_GPS())) {
        Log_Write_Nav_Tuning();
    }
}

// fifty_hz_logging_loop
// should be run at 50hz
static void fifty_hz_logging_loop()
{
#if HIL_MODE != HIL_MODE_DISABLED
    // HIL for a copter needs very fast update of the servo values
    gcs_send_message(MSG_RADIO_OUT);
#endif

#if HIL_MODE == HIL_MODE_DISABLED
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        Log_Write_Rate();
    }

    // log IMU data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_FAST)) {
        DataFlash.Log_Write_IMU(ins);
    }
#endif
}

// full_rate_logging_loop
// should be run at the MAIN_LOOP_RATE
static void full_rate_logging_loop()
{
    if (should_log(MASK_LOG_IMU_FAST)) {
        DataFlash.Log_Write_IMU(ins);
    }
}

// three_hz_loop - 3.3hz loop
static void three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();

#if AC_FENCE == ENABLED
    // check if we have breached a fence
    fence_check();
#endif // AC_FENCE_ENABLED

#if SPRAYER == ENABLED
    sprayer.update();
#endif

    update_events();

    // update ch6 in flight tuning
    tuning();
}

// one_hz_loop - runs at 1Hz
static void one_hz_loop()
{
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    // perform pre-arm checks & display failures every 30 seconds
    static uint8_t pre_arm_display_counter = 15;
    pre_arm_display_counter++;
    if (pre_arm_display_counter >= 30) {
        pre_arm_checks(true);
        pre_arm_display_counter = 0;
    }else{
        pre_arm_checks(false);
    }

    // auto disarm checks
    auto_disarm_check();

    if (!motors.armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.set_orientation();

        // check the user hasn't updated the frame orientation
        motors.set_frame_orientation(g.frame_orientation);
    }

    // update assigned functions and enable auxiliar servos
    RC_Channel_aux::enable_aux_servos();

    check_usb_mux();

#if AP_TERRAIN_AVAILABLE
    terrain.update();

    // tell the rangefinder our height, so it can go into power saving
    // mode if available
#if CONFIG_SONAR == ENABLED
    float height;
    if (terrain.height_above_terrain(height, true)) {
        sonar.set_estimated_terrain_height(height);
    }
#endif
#endif

    // update position controller alt limits
    update_poscon_alt_max();
}

// called at 50hz
static void update_GPS(void)
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];   // time of last gps message
    bool gps_updated = false;

    gps.update();

    // log after every gps message
    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);

            // log GPS message
            if (should_log(MASK_LOG_GPS)) {
                DataFlash.Log_Write_GPS(gps, i, current_loc.alt);
            }

            gps_updated = true;
        }
    }

    if (gps_updated) {
        // set system time if necessary
        set_system_time_from_GPS();

        // checks to initialise home and take location based pictures
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {

#if CAMERA == ENABLED
            if (camera.update_location(current_loc) == true) {
                do_take_picture();
            }
#endif
        }
    }
}

static void
init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

    // log the simple bearing to dataflash
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
}

// update_simple_mode - rotates pilot input if we are in simple mode
void update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (ap.simple_mode == 0 || !ap.new_radio_frame) {
        return;
    }

    // mark radio frame as consumed
    ap.new_radio_frame = false;

    if (ap.simple_mode == 1) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = g.rc_1.control_in*simple_cos_yaw - g.rc_2.control_in*simple_sin_yaw;
        pitchx = g.rc_1.control_in*simple_sin_yaw + g.rc_2.control_in*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = g.rc_1.control_in*super_simple_cos_yaw - g.rc_2.control_in*super_simple_sin_yaw;
        pitchx = g.rc_1.control_in*super_simple_sin_yaw + g.rc_2.control_in*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    g.rc_1.control_in = rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw();
    g.rc_2.control_in = -rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw();
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void update_super_simple_bearing(bool force_update)
{
    // check if we are in super simple mode and at least 10m from home
    if(force_update || (ap.simple_mode == 2 && home_distance > SUPER_SIMPLE_RADIUS)) {
        // check the bearing to home has changed by at least 5 degrees
        if (labs(super_simple_last_bearing - home_bearing) > 500) {
            super_simple_last_bearing = home_bearing;
            float angle_rad = radians((super_simple_last_bearing+18000)/100);
            super_simple_cos_yaw = cosf(angle_rad);
            super_simple_sin_yaw = sinf(angle_rad);
        }
    }
}

static void read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
#if HIL_MODE != HIL_MODE_DISABLED
    // update hil before ahrs update
    gcs_check_input();
#endif

    ahrs.update();
}

// read baro and sonar altitude at 10hz
static void update_altitude()
{
    // read in baro altitude
    read_barometer();

    // read in sonar altitude
    sonar_alt           = read_sonar();

    // write altitude info to dataflash logs
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
    }
}

AP_HAL_MAIN();

#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/AP_State.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// set_home_state - update home state
void set_home_state(enum HomeState new_home_state)
{
    // if no change, exit immediately
    if (ap.home_state == new_home_state)
        return;

    // update state
    ap.home_state = new_home_state;

    // log if home has been set
    if (new_home_state == HOME_SET_NOT_LOCKED || new_home_state == HOME_SET_AND_LOCKED) {
        Log_Write_Event(DATA_SET_HOME);
    }
}

// home_is_set - returns true if home positions has been set (to GPS location, armed location or EKF origin)
bool home_is_set()
{
    return (ap.home_state == HOME_SET_NOT_LOCKED || ap.home_state == HOME_SET_AND_LOCKED);
}

// ---------------------------------------------
void set_auto_armed(bool b)
{
    // if no change, exit immediately
    if( ap.auto_armed == b )
        return;

    ap.auto_armed = b;
    if(b){
        Log_Write_Event(DATA_AUTO_ARMED);
    }
}

// ---------------------------------------------
void set_simple_mode(uint8_t b)
{
    if(ap.simple_mode != b){
        if(b == 0){
            Log_Write_Event(DATA_SET_SIMPLE_OFF);
        }else if(b == 1){
            Log_Write_Event(DATA_SET_SIMPLE_ON);
        }else{
            // initialise super simple heading
            update_super_simple_bearing(true);
            Log_Write_Event(DATA_SET_SUPERSIMPLE_ON);
        }
        ap.simple_mode = b;
    }
}

// ---------------------------------------------
static void set_failsafe_radio(bool b)
{
    // only act on changes
    // -------------------
    if(failsafe.radio != b) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        failsafe.radio = b;

        if (failsafe.radio == false) {
            // We've regained radio contact
            // ----------------------------
            failsafe_radio_off_event();
        }else{
            // We've lost radio contact
            // ------------------------
            failsafe_radio_on_event();
        }

        // update AP_Notify
        AP_Notify::flags.failsafe_radio = b;
    }
}


// ---------------------------------------------
void set_failsafe_battery(bool b)
{
    failsafe.battery = b;
    AP_Notify::flags.failsafe_battery = b;
}

// ---------------------------------------------
static void set_failsafe_gcs(bool b)
{
    failsafe.gcs = b;
}

// ---------------------------------------------
void set_land_complete(bool b)
{
    // if no change, exit immediately
    if( ap.land_complete == b )
        return;

    if(b){
        Log_Write_Event(DATA_LAND_COMPLETE);
    }else{
        Log_Write_Event(DATA_NOT_LANDED);
    }
    ap.land_complete = b;
}

// ---------------------------------------------

// set land complete maybe flag
void set_land_complete_maybe(bool b)
{
    // if no change, exit immediately
    if (ap.land_complete_maybe == b)
        return;

    if (b) {
        Log_Write_Event(DATA_LAND_COMPLETE_MAYBE);
    }
    ap.land_complete_maybe = b;
}

// ---------------------------------------------

void set_pre_arm_check(bool b)
{
    if(ap.pre_arm_check != b) {
        ap.pre_arm_check = b;
        AP_Notify::flags.pre_arm_check = b;
    }
}

void set_pre_arm_rc_check(bool b)
{
    if(ap.pre_arm_rc_check != b) {
        ap.pre_arm_rc_check = b;
    }
}

#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/Attitude.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// get_smoothing_gain - returns smoothing gain to be passed into attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth
//      result is a number from 2 to 12 with 2 being very sluggish and 12 being very crisp
float get_smoothing_gain()
{
    return (2.0f + (float)g.rc_feel_rp/10.0f);
}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
static void get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out)
{
    float angle_max = constrain_float(aparm.angle_max,1000,8000);
    float scaler = (float)angle_max/(float)ROLL_PITCH_INPUT_MAX;

    // scale roll_in, pitch_in to correct units
    roll_in *= scaler;
    pitch_in *= scaler;

    // do circular limit
    float total_in = pythagorous2((float)pitch_in, (float)roll_in);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // do lateral tilt to euler roll conversion
    roll_in = (18000/M_PI_F) * atanf(cosf(pitch_in*(M_PI_F/18000))*tanf(roll_in*(M_PI_F/18000)));

    // return
    roll_out = roll_in;
    pitch_out = pitch_in;
}

// get_pilot_desired_heading - transform pilot's yaw input into a desired heading
// returns desired angle in centi-degrees
// To-Do: return heading as a float?
static float get_pilot_desired_yaw_rate(int16_t stick_angle)
{
    // convert pilot input to the desired yaw rate
    return stick_angle * g.acro_yaw_p;
}

/*************************************************************
 * yaw controllers
 *************************************************************/

// get_roi_yaw - returns heading towards location held in roi_WP
// should be called at 100hz
static float get_roi_yaw()
{
    static uint8_t roi_yaw_counter = 0;     // used to reduce update rate to 100hz

    roi_yaw_counter++;
    if (roi_yaw_counter >= 4) {
        roi_yaw_counter = 0;
        yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), roi_WP);
    }

    return yaw_look_at_WP_bearing;
}

static float get_look_ahead_yaw()
{
    // Commanded Yaw to automatically look ahead.
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D && gps.ground_speed_cm() > YAW_LOOK_AHEAD_MIN_SPEED) {
        yaw_look_ahead_bearing = gps.ground_course_cd();
    }
    return yaw_look_ahead_bearing;
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// update_thr_average - update estimated throttle required to hover (if necessary)
//  should be called at 100hz
static void update_thr_average()
{
    // ensure throttle_average has been initialised
    if( throttle_average == 0 ) {
        throttle_average = g.throttle_mid;
        // update position controller
        pos_control.set_throttle_hover(throttle_average);
    }

    // if not armed or landed exit
    if (!motors.armed() || ap.land_complete) {
        return;
    }

    // get throttle output
    int16_t throttle = g.rc_3.servo_out;

    // calc average throttle if we are in a level hover
    if (throttle > g.throttle_min && abs(climb_rate) < 60 && labs(ahrs.roll_sensor) < 500 && labs(ahrs.pitch_sensor) < 500) {
        throttle_average = throttle_average * 0.99f + (float)throttle * 0.01f;
        // update position controller
        pos_control.set_throttle_hover(throttle_average);
    }
}

// set_throttle_takeoff - allows parents to tell throttle controller we are taking off so I terms can be cleared
static void
set_throttle_takeoff()
{
    // tell position controller to reset alt target and reset I terms
    pos_control.init_takeoff();

    // tell motors to do a slow start
    motors.slow_start(true);
}

// get_pilot_desired_throttle - transform pilot's throttle input to make cruise throttle mid stick
// used only for manual throttle modes
// returns throttle output 0 to 1000
static int16_t get_pilot_desired_throttle(int16_t throttle_control)
{
    int16_t throttle_out;

    int16_t mid_stick = g.rc_3.get_control_mid();

    // ensure reasonable throttle values
    throttle_control = constrain_int16(throttle_control,0,1000);
    g.throttle_mid = constrain_int16(g.throttle_mid,300,700);

    // check throttle is above, below or in the deadband
    if (throttle_control < mid_stick) {
        // below the deadband
        throttle_out = g.throttle_min + ((float)(throttle_control-g.throttle_min))*((float)(g.throttle_mid - g.throttle_min))/((float)(mid_stick-g.throttle_min));
    }else if(throttle_control > mid_stick) {
        // above the deadband
        throttle_out = g.throttle_mid + ((float)(throttle_control-mid_stick)) * (float)(1000-g.throttle_mid) / (float)(1000-mid_stick);
    }else{
        // must be in the deadband
        throttle_out = g.throttle_mid;
    }

    return throttle_out;
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to
// climb rate in cm/s.  we use radio_in instead of control_in to get the full range
// without any deadzone at the bottom
static int16_t get_pilot_desired_climb_rate(int16_t throttle_control)
{
    int16_t desired_rate = 0;

    // throttle failsafe check
    if( failsafe.radio ) {
        return 0;
    }

    int16_t mid_stick = g.rc_3.get_control_mid();
    int16_t deadband_top = mid_stick + g.throttle_deadzone;
    int16_t deadband_bottom = mid_stick - g.throttle_deadzone;

    // ensure a reasonable throttle value
    throttle_control = constrain_int16(throttle_control,g.throttle_min,1000);

    // ensure a reasonable deadzone
    g.throttle_deadzone = constrain_int16(g.throttle_deadzone, 0, 400);

    // check throttle is above, below or in the deadband
    if (throttle_control < deadband_bottom) {
        // below the deadband
        desired_rate = (int32_t)g.pilot_velocity_z_max * (throttle_control-deadband_bottom) / (deadband_bottom-g.throttle_min);
    }else if (throttle_control > deadband_top) {
        // above the deadband
        desired_rate = (int32_t)g.pilot_velocity_z_max * (throttle_control-deadband_top) / (1000-deadband_top);
    }else{
        // must be in the deadband
        desired_rate = 0;
    }

    // desired climb rate for logging
    desired_climb_rate = desired_rate;

    return desired_rate;
}

// get_non_takeoff_throttle - a throttle somewhere between min and mid throttle which should not lead to a takeoff
static int16_t get_non_takeoff_throttle()
{
    return (g.throttle_mid / 2.0f);
}

// get_throttle_pre_takeoff - convert pilot's input throttle to a throttle output before take-off
// used only for althold, loiter, hybrid flight modes
// returns throttle output 0 to 1000
static int16_t get_throttle_pre_takeoff(int16_t throttle_control)
{
    int16_t throttle_out;

    // exit immediately if throttle_control is zero
    if (throttle_control <= 0) {
        return 0;
    }

    // calculate mid stick and deadband
    int16_t mid_stick = g.rc_3.get_control_mid();
    int16_t deadband_top = mid_stick + g.throttle_deadzone;

    // sanity check throttle input
    throttle_control = constrain_int16(throttle_control,0,1000);

    // sanity check throttle_mid
    g.throttle_mid = constrain_int16(g.throttle_mid,300,700);

    // sanity check throttle_min vs throttle_mid
    if (g.throttle_min > get_non_takeoff_throttle()) {
        return g.throttle_min;
    }

    // check throttle is below top of deadband
    if (throttle_control < deadband_top) {
        throttle_out = g.throttle_min + ((float)(throttle_control-g.throttle_min))*((float)(get_non_takeoff_throttle() - g.throttle_min))/((float)(deadband_top-g.throttle_min));
    }else{
        // must be in the deadband
        throttle_out = get_non_takeoff_throttle();
    }

    return throttle_out;
}

// get_surface_tracking_climb_rate - hold copter at the desired distance above the ground
//      returns climb rate (in cm/s) which should be passed to the position controller
static float get_surface_tracking_climb_rate(int16_t target_rate, float current_alt_target, float dt)
{
    static uint32_t last_call_ms = 0;
    float distance_error;
    float velocity_correction;
    float current_alt = inertial_nav.get_altitude();

    uint32_t now = millis();

    // reset target altitude if this controller has just been engaged
    if (now - last_call_ms > SONAR_TIMEOUT_MS) {
        target_sonar_alt = sonar_alt + current_alt_target - current_alt;
    }
    last_call_ms = now;

    // adjust sonar target alt if motors have not hit their limits
    if ((target_rate<0 && !motors.limit.throttle_lower) || (target_rate>0 && !motors.limit.throttle_upper)) {
        target_sonar_alt += target_rate * dt;
    }

    // do not let target altitude get too far from current altitude above ground
    // Note: the 750cm limit is perhaps too wide but is consistent with the regular althold limits and helps ensure a smooth transition
    target_sonar_alt = constrain_float(target_sonar_alt,sonar_alt-pos_control.get_leash_down_z(),sonar_alt+pos_control.get_leash_up_z());

    // calc desired velocity correction from target sonar alt vs actual sonar alt (remove the error already passed to Altitude controller to avoid oscillations)
    distance_error = (target_sonar_alt - sonar_alt) - (current_alt_target - current_alt);
    velocity_correction = distance_error * g.sonar_gain;
    velocity_correction = constrain_float(velocity_correction, -THR_SURFACE_TRACKING_VELZ_MAX, THR_SURFACE_TRACKING_VELZ_MAX);

    // return combined pilot climb rate + rate to correct sonar alt error
    return (target_rate + velocity_correction);
}

// set_accel_throttle_I_from_pilot_throttle - smoothes transition from pilot controlled throttle to autopilot throttle
static void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle)
{
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    g.pid_accel_z.set_integrator(pilot_throttle-throttle_average);
}

// updates position controller's maximum altitude using fence and EKF limits
static void update_poscon_alt_max()
{
    float alt_limit_cm = 0.0f;  // interpreted as no limit if left as zero

#if AC_FENCE == ENABLED
    // set fence altitude limit in position controller
    if ((fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        alt_limit_cm = pv_alt_above_origin(fence.get_safe_alt()*100.0f);
    }
#endif

    // get alt limit from EKF (limited during optical flow flight)
    float ekf_limit_cm = 0.0f;
    if (inertial_nav.get_hgt_ctrl_limit(ekf_limit_cm)) {
        if ((alt_limit_cm <= 0.0f) || (ekf_limit_cm < alt_limit_cm)) {
            alt_limit_cm = ekf_limit_cm;
        }
    }

    // pass limit to pos controller
    pos_control.set_alt_max(alt_limit_cm);
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/GCS_Mavlink.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// default sensors are present and healthy: gyro, accelerometer, barometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS | MAV_SYS_STATUS_AHRS)

// forward declarations to make compiler happy
static bool do_guided(const AP_Mission::Mission_Command& cmd);

// use this to prevent recursion during sensor init
static bool in_mavlink_delay;

// true if we are out of time in our event timeslice
static bool	gcs_out_of_time;


// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (txspace < MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_ ## id ## _LEN) return false

static void gcs_send_heartbeat(void)
{
    gcs_send_message(MSG_HEARTBEAT);
}

static void gcs_send_deferred(void)
{
    gcs_send_message(MSG_RETRY_DEFERRED);
}

/*
 *  !!NOTE!!
 *
 *  the use of NOINLINE separate functions for each message type avoids
 *  a compiler bug in gcc that would cause it to use far more stack
 *  space than is needed. Without the NOINLINE we use the sum of the
 *  stack needed for each message type. Please be careful to follow the
 *  pattern below when adding any new messages
 */

static NOINLINE void send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = ap.land_complete ? MAV_STATE_STANDBY : MAV_STATE_ACTIVE;
    uint32_t custom_mode = control_mode;

    // set system as critical if any failsafe have triggered
    if (failsafe.radio || failsafe.battery || failsafe.gcs || failsafe.ekf)  {
        system_status = MAV_STATE_CRITICAL;
    }

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
    switch (control_mode) {
    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
    case POSHOLD:
    case STOP:
        base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    }

    // all modes except INITIALISING have some form of manual
    // override if stick mixing is enabled
    base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif

    // we are armed if we are not initialising
    if (motors.armed()) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    mavlink_msg_heartbeat_send(
        chan,
#if (FRAME_CONFIG == QUAD_FRAME)
        MAV_TYPE_QUADROTOR,
#elif (FRAME_CONFIG == TRI_FRAME)
        MAV_TYPE_TRICOPTER,
#elif (FRAME_CONFIG == HEXA_FRAME || FRAME_CONFIG == Y6_FRAME)
        MAV_TYPE_HEXAROTOR,
#elif (FRAME_CONFIG == OCTA_FRAME || FRAME_CONFIG == OCTA_QUAD_FRAME)
        MAV_TYPE_OCTOROTOR,
#elif (FRAME_CONFIG == HELI_FRAME)
        MAV_TYPE_HELICOPTER,
#elif (FRAME_CONFIG == SINGLE_FRAME)  //because mavlink did not define a singlecopter, we use a rocket
        MAV_TYPE_ROCKET,
#elif (FRAME_CONFIG == COAX_FRAME)  //because mavlink did not define a singlecopter, we use a rocket
        MAV_TYPE_ROCKET,
#else
  #error Unrecognised frame type
#endif
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
}

static NOINLINE void send_attitude(mavlink_channel_t chan)
{
    const Vector3f &gyro = ins.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        millis(),
        ahrs.roll,
        ahrs.pitch,
        ahrs.yaw,
        gyro.x,
        gyro.y,
        gyro.z);
}

#if AC_FENCE == ENABLED
static NOINLINE void send_limits_status(mavlink_channel_t chan)
{
    fence_send_mavlink_status(chan);
}
#endif


static NOINLINE void send_extended_status1(mavlink_channel_t chan)
{
    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    // default sensors present
    control_sensors_present = MAVLINK_SENSOR_PRESENT_DEFAULT;

    // first what sensors/controllers we have
    if (g.compass_enabled) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG; // compass present
    }
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (optflow.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
    if (ap.rc_receiver_present) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }

    // all present sensors enabled by default except altitude and position control and motors which we will set individually
    control_sensors_enabled = control_sensors_present & (~MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL &
                                                         ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);

    switch (control_mode) {
    case ALT_HOLD:
    case AUTO:
    case GUIDED:
    case LOITER:
    case RTL:
    case CIRCLE:
    case LAND:
    case OF_LOITER:
    case POSHOLD:
    case STOP:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;
    case SPORT:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        break;
    }

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }

    // default to all healthy except baro, compass, gps and receiver which we set individually
    control_sensors_health = control_sensors_present & ~(MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                                                         MAV_SYS_STATUS_SENSOR_3D_MAG |
                                                         MAV_SYS_STATUS_SENSOR_GPS |
                                                         MAV_SYS_STATUS_SENSOR_RC_RECEIVER);
    if (barometer.all_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
    if (g.compass_enabled && compass.healthy() && ahrs.use_compass()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#if OPTFLOW == ENABLED
    if (optflow.healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif
    if (ap.rc_receiver_present && !failsafe.radio) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
    if (!ins.get_gyro_health_all() || !ins.gyro_calibrated_ok_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
    if (!ins.get_accel_health_all()) {
        control_sensors_health &= ~MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    }

    if (ahrs.initialised() && !ahrs.healthy()) {
        // AHRS subsystem is unhealthy
        control_sensors_health &= ~MAV_SYS_STATUS_AHRS;
    }

    int16_t battery_current = -1;
    int8_t battery_remaining = -1;

    if (battery.has_current() && battery.healthy()) {
        battery_remaining = battery.capacity_remaining_pct();
        battery_current = battery.current_amps() * 100;
    }

#if AP_TERRAIN_AVAILABLE
    switch (terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        // To-Do: restore unhealthy terrain status reporting once terrain is used in copter
        //control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        //control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        //break;
    case AP_Terrain::TerrainStatusOK:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

#if CONFIG_SONAR == ENABLED
    if (sonar.num_sensors() > 0) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (sonar.has_data()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif

    if (!ap.initialised || ins.calibrating()) {
        // while initialising the gyros and accels are not enabled
        control_sensors_enabled &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
        control_sensors_health &= ~(MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL);
    }

    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
        (uint16_t)(scheduler.load_average(MAIN_LOOP_MICROS) * 1000),
        battery.voltage() * 1000, // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
        0, // comm drops %,
        0, // comm drops in pkts,
        0, 0, 0, 0);

}

static void NOINLINE send_location(mavlink_channel_t chan)
{
    uint32_t fix_time;
    // if we have a GPS fix, take the time as the last fix time. That
    // allows us to correctly calculate velocities and extrapolate
    // positions.
    // If we don't have a GPS fix then we are dead reckoning, and will
    // use the current boot time as the fix time.
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
        fix_time = gps.last_fix_time_ms();
    } else {
        fix_time = millis();
    }
    const Vector3f &vel = inertial_nav.get_velocity();
    mavlink_msg_global_position_int_send(
        chan,
        fix_time,
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        (ahrs.get_home().alt + current_loc.alt) * 10UL,      // millimeters above sea level
        current_loc.alt * 10,           // millimeters above ground
        vel.x,                          // X speed cm/s (+ve North)
        vel.y,                          // Y speed cm/s (+ve East)
        vel.z,                          // Z speed cm/s (+ve up)
        ahrs.yaw_sensor);               // compass heading in 1/100 degree
}

static void NOINLINE send_nav_controller_output(mavlink_channel_t chan)
{
    const Vector3f &targets = attitude_control.angle_ef_targets();
    mavlink_msg_nav_controller_output_send(
        chan,
        targets.x / 1.0e2f,
        targets.y / 1.0e2f,
        targets.z / 1.0e2f,
        wp_bearing / 1.0e2f,
        wp_distance / 1.0e2f,
        pos_control.get_alt_error() / 1.0e2f,
        0,
        0);
}

// report simulator state
static void NOINLINE send_simstate(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    sitl.simstate_send(chan);
#endif
}

static void NOINLINE send_hwstatus(mavlink_channel_t chan)
{
    mavlink_msg_hwstatus_send(
        chan,
        hal.analogin->board_voltage()*1000,
        hal.i2c->lockup_count());
}

static void NOINLINE send_servo_out(mavlink_channel_t chan)
{
#if HIL_MODE != HIL_MODE_DISABLED
    // normalized values scaled to -10000 to 10000
    // This is used for HIL.  Do not change without discussing with HIL maintainers

#if FRAME_CONFIG == HELI_FRAME
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0, // port 0
        g.rc_1.servo_out,
        g.rc_2.servo_out,
        g.rc_3.radio_out,
        g.rc_4.servo_out,
        0,
        0,
        0,
        0,
        receiver_rssi);
#else
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0,         // port 0
        g.rc_1.servo_out,
        g.rc_2.servo_out,
        g.rc_3.radio_out,
        g.rc_4.servo_out,
        10000 * g.rc_1.norm_output(),
        10000 * g.rc_2.norm_output(),
        10000 * g.rc_3.norm_output(),
        10000 * g.rc_4.norm_output(),
        receiver_rssi);
#endif
#endif // HIL_MODE
}

static void NOINLINE send_radio_out(mavlink_channel_t chan)
{
    mavlink_msg_servo_output_raw_send(
        chan,
        micros(),
        0,     // port
        hal.rcout->read(0),
        hal.rcout->read(1),
        hal.rcout->read(2),
        hal.rcout->read(3),
        hal.rcout->read(4),
        hal.rcout->read(5),
        hal.rcout->read(6),
        hal.rcout->read(7));
}

static void NOINLINE send_vfr_hud(mavlink_channel_t chan)
{
    mavlink_msg_vfr_hud_send(
        chan,
        gps.ground_speed(),
        gps.ground_speed(),
        (ahrs.yaw_sensor / 100) % 360,
        g.rc_3.servo_out/10,
        current_loc.alt / 100.0f,
        climb_rate / 100.0f);
}

static void NOINLINE send_current_waypoint(mavlink_channel_t chan)
{
    mavlink_msg_mission_current_send(chan, mission.get_current_nav_index());
}

#if CONFIG_SONAR == ENABLED
static void NOINLINE send_rangefinder(mavlink_channel_t chan)
{
    // exit immediately if sonar is disabled
    if (!sonar.has_data()) {
        return;
    }
    mavlink_msg_rangefinder_send(
            chan,
            sonar.distance_cm() * 0.01f,
            sonar.voltage_mv() * 0.001f);
}
#endif

static void NOINLINE send_statustext(mavlink_channel_t chan)
{
    mavlink_statustext_t *s = &gcs[chan-MAVLINK_COMM_0].pending_status;
    mavlink_msg_statustext_send(
        chan,
        s->severity,
        s->text);
}

// are we still delaying telemetry to try to avoid Xbee bricking?
static bool telemetry_delayed(mavlink_channel_t chan)
{
    uint32_t tnow = millis() >> 10;
    if (tnow > (uint32_t)g.telem_delay) {
        return false;
    }
    if (chan == MAVLINK_COMM_0 && hal.gpio->usb_connected()) {
        // this is USB telemetry, so won't be an Xbee
        return false;
    }
    // we're either on the 2nd UART, or no USB cable is connected
    // we need to delay telemetry by the TELEM_DELAY time
    return true;
}


// try to send a message, return false if it won't fit in the serial tx buffer
bool GCS_MAVLINK::try_send_message(enum ap_message id)
{
    uint16_t txspace = comm_get_txspace(chan);

    if (telemetry_delayed(chan)) {
        return false;
    }

#if HIL_MODE != HIL_MODE_SENSORS
    // if we don't have at least 250 micros remaining before the main loop
    // wants to fire then don't send a mavlink message. We want to
    // prioritise the main flight control loop over communications
    if (scheduler.time_available_usec() < 250 && motors.armed()) {
        gcs_out_of_time = true;
        return false;
    }
#endif

    switch(id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        gcs[chan-MAVLINK_COMM_0].last_heartbeat_time = hal.scheduler->millis();
        send_heartbeat(chan);
        break;

    case MSG_EXTENDED_STATUS1:
        // send extended status only once vehicle has been initialised
        // to avoid unnecessary errors being reported to user
        if (ap.initialised) {
            CHECK_PAYLOAD_SIZE(SYS_STATUS);
            send_extended_status1(chan);
            CHECK_PAYLOAD_SIZE(POWER_STATUS);
            gcs[chan-MAVLINK_COMM_0].send_power_status();
        }
        break;

    case MSG_EXTENDED_STATUS2:
        CHECK_PAYLOAD_SIZE(MEMINFO);
        gcs[chan-MAVLINK_COMM_0].send_meminfo();
        break;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        send_attitude(chan);
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        send_location(chan);
        break;

    case MSG_LOCAL_POSITION:
        CHECK_PAYLOAD_SIZE(LOCAL_POSITION_NED);
        send_local_position(ahrs);
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
        send_nav_controller_output(chan);
        break;

    case MSG_GPS_RAW:
        return gcs[chan-MAVLINK_COMM_0].send_gps_raw(gps);

    case MSG_SYSTEM_TIME:
        CHECK_PAYLOAD_SIZE(SYSTEM_TIME);
        gcs[chan-MAVLINK_COMM_0].send_system_time(gps);
        break;

    case MSG_SERVO_OUT:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
        send_servo_out(chan);
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        gcs[chan-MAVLINK_COMM_0].send_radio_in(receiver_rssi);
        break;

    case MSG_RADIO_OUT:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        send_radio_out(chan);
        break;

    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        send_vfr_hud(chan);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        gcs[chan-MAVLINK_COMM_0].send_raw_imu(ins, compass);
        break;

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        gcs[chan-MAVLINK_COMM_0].send_scaled_pressure(barometer);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        gcs[chan-MAVLINK_COMM_0].send_sensor_offsets(ins, compass, barometer);
        break;

    case MSG_CURRENT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_CURRENT);
        send_current_waypoint(chan);
        break;

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        gcs[chan-MAVLINK_COMM_0].queued_param_send();
        break;

    case MSG_NEXT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        gcs[chan-MAVLINK_COMM_0].queued_waypoint_send();
        break;

    case MSG_RANGEFINDER:
#if CONFIG_SONAR == ENABLED
        CHECK_PAYLOAD_SIZE(RANGEFINDER);
        send_rangefinder(chan);
#endif
        break;

    case MSG_TERRAIN:
#if AP_TERRAIN_AVAILABLE
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        terrain.send_request(chan);
#endif
        break;

    case MSG_CAMERA_FEEDBACK:
#if CAMERA == ENABLED
        CHECK_PAYLOAD_SIZE(CAMERA_FEEDBACK);
        camera.send_feedback(chan, gps, ahrs, current_loc);
#endif
        break;

    case MSG_STATUSTEXT:
        CHECK_PAYLOAD_SIZE(STATUSTEXT);
        send_statustext(chan);
        break;

    case MSG_LIMITS_STATUS:
#if AC_FENCE == ENABLED
        CHECK_PAYLOAD_SIZE(LIMITS_STATUS);
        send_limits_status(chan);
#endif
        break;

    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        gcs[chan-MAVLINK_COMM_0].send_ahrs(ahrs);
        break;

    case MSG_SIMSTATE:
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        send_simstate(chan);
#endif
#if AP_AHRS_NAVEKF_AVAILABLE
        CHECK_PAYLOAD_SIZE(AHRS2);
        gcs[chan-MAVLINK_COMM_0].send_ahrs2(ahrs);
#endif
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        send_hwstatus(chan);
        break;

    case MSG_MOUNT_STATUS:
#if MOUNT == ENABLED
        CHECK_PAYLOAD_SIZE(MOUNT_STATUS);    
        camera_mount.status_msg(chan);
#endif // MOUNT == ENABLED
        break;

    case MSG_BATTERY2:
        CHECK_PAYLOAD_SIZE(BATTERY2);
        gcs[chan-MAVLINK_COMM_0].send_battery2(battery);
        break;

    case MSG_OPTICAL_FLOW:
#if OPTFLOW == ENABLED
        CHECK_PAYLOAD_SIZE(OPTICAL_FLOW);
        gcs[chan-MAVLINK_COMM_0].send_opticalflow(ahrs, optflow);
#endif
        break;

    case MSG_GIMBAL_REPORT:
#if MOUNT == ENABLED
        CHECK_PAYLOAD_SIZE(GIMBAL_REPORT);
        camera_mount.send_gimbal_report(chan);
#endif
        break;

    case MSG_EKF_STATUS_REPORT:
#if AP_AHRS_NAVEKF_AVAILABLE
        CHECK_PAYLOAD_SIZE(EKF_STATUS_REPORT);
        ahrs.get_NavEKF().send_status_report(chan);
#endif
        break;

    case MSG_FENCE_STATUS:
    case MSG_WIND:
        // unused
        break;

    case MSG_RETRY_DEFERRED:
        break; // just here to prevent a warning
    }

    return true;
}


const AP_Param::GroupInfo GCS_MAVLINK::var_info[] PROGMEM = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Stream rate of RAW_IMU, SCALED_IMU2, SCALED_PRESSURE, and SENSOR_OFFSETS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK, streamRates[0],  0),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Stream rate of SYS_STATUS, MEMINFO, MISSION_CURRENT, GPS_RAW_INT, NAV_CONTROLLER_OUTPUT, and LIMITS_STATUS to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK, streamRates[1],  0),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS_RAW to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK, streamRates[2],  0),

    // @Param: RAW_CTRL
    // @DisplayName: Raw Control stream rate to ground station
    // @Description: Stream rate of RC_CHANNELS_SCALED (HIL only) to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK, streamRates[3],  0),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Stream rate of GLOBAL_POSITION_INT to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK, streamRates[4],  0),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Stream rate of ATTITUDE and SIMSTATE (SITL only) to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK, streamRates[5],  0),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Stream rate of VFR_HUD to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK, streamRates[6],  0),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Stream rate of AHRS, HWSTATUS, and SYSTEM_TIME to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK, streamRates[7],  0),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Stream rate of PARAM_VALUE to ground station
    // @Units: Hz
    // @Range: 0 10
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK, streamRates[8],  0),
    AP_GROUPEND
};


// see if we should send a stream now. Called at 50Hz
bool GCS_MAVLINK::stream_trigger(enum streams stream_num)
{
    if (stream_num >= NUM_STREAMS) {
        return false;
    }
    float rate = (uint8_t)streamRates[stream_num].get();

    // send at a much lower rate while handling waypoints and
    // parameter sends
    if ((stream_num != STREAM_PARAMS) &&
        (waypoint_receiving || _queued_parameter != NULL)) {
        rate *= 0.25f;
    }

    if (rate <= 0) {
        return false;
    }

    if (stream_ticks[stream_num] == 0) {
        // we're triggering now, setup the next trigger point
        if (rate > 50) {
            rate = 50;
        }
        stream_ticks[stream_num] = (50 / rate) - 1 + stream_slowdown;
        return true;
    }

    // count down at 50Hz
    stream_ticks[stream_num]--;
    return false;
}

void
GCS_MAVLINK::data_stream_send(void)
{
    if (waypoint_receiving) {
        // don't interfere with mission transfer
        return;
    }

    if (!in_mavlink_delay && !motors.armed()) {
        handle_log_send(DataFlash);
    }

    gcs_out_of_time = false;

    if (_queued_parameter != NULL) {
        if (streamRates[STREAM_PARAMS].get() <= 0) {
            streamRates[STREAM_PARAMS].set(10);
        }
        if (stream_trigger(STREAM_PARAMS)) {
            send_message(MSG_NEXT_PARAM);
        }
        // don't send anything else at the same time as parameters
        return;
    }

    if (gcs_out_of_time) return;

    if (in_mavlink_delay) {
        // don't send any other stream types while in the delay callback
        return;
    }

    if (stream_trigger(STREAM_RAW_SENSORS)) {
        send_message(MSG_RAW_IMU1);
        send_message(MSG_RAW_IMU2);
        send_message(MSG_RAW_IMU3);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTENDED_STATUS)) {
        send_message(MSG_EXTENDED_STATUS1);
        send_message(MSG_EXTENDED_STATUS2);
        send_message(MSG_CURRENT_WAYPOINT);
        send_message(MSG_GPS_RAW);
        send_message(MSG_NAV_CONTROLLER_OUTPUT);
        send_message(MSG_LIMITS_STATUS);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_POSITION)) {
        send_message(MSG_LOCATION);
        send_message(MSG_LOCAL_POSITION);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_RAW_CONTROLLER)) {
        send_message(MSG_SERVO_OUT);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_RC_CHANNELS)) {
        send_message(MSG_RADIO_OUT);
        send_message(MSG_RADIO_IN);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA1)) {
        send_message(MSG_ATTITUDE);
        send_message(MSG_SIMSTATE);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA2)) {
        send_message(MSG_VFR_HUD);
    }

    if (gcs_out_of_time) return;

    if (stream_trigger(STREAM_EXTRA3)) {
        send_message(MSG_AHRS);
        send_message(MSG_HWSTATUS);
        send_message(MSG_SYSTEM_TIME);
        send_message(MSG_RANGEFINDER);
#if AP_TERRAIN_AVAILABLE
        send_message(MSG_TERRAIN);
#endif
        send_message(MSG_BATTERY2);
        send_message(MSG_MOUNT_STATUS);
        send_message(MSG_OPTICAL_FLOW);
        send_message(MSG_GIMBAL_REPORT);
        send_message(MSG_EKF_STATUS_REPORT);
    }
}


void GCS_MAVLINK::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
    do_guided(cmd);
}

void GCS_MAVLINK::handle_change_alt_request(AP_Mission::Mission_Command &cmd)
{
    // add home alt if needed
    if (cmd.content.location.flags.relative_alt) {
        cmd.content.location.alt += ahrs.get_home().alt;
    }

    // To-Do: update target altitude for loiter or waypoint controller depending upon nav mode
}

void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
    uint8_t result = MAV_RESULT_FAILED;         // assume failure.  Each messages id is responsible for return ACK or NAK if required

    switch (msg->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT:      // MAV ID: 0
    {
        // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
        if(msg->sysid != g.sysid_my_gcs) break;
        failsafe.last_heartbeat_ms = millis();
        pmTest1++;
        break;
    }

    case MAVLINK_MSG_ID_SET_MODE:       // MAV ID: 11
    {
        handle_set_mode(msg, set_mode);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:         // MAV ID: 20
    {
        handle_param_request_read(msg);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:         // MAV ID: 21
    {
        // mark the firmware version in the tlog
        send_text_P(SEVERITY_LOW, PSTR(FIRMWARE_STRING));

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
        send_text_P(SEVERITY_LOW, PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION));
#endif
        send_text_P(SEVERITY_LOW, PSTR("Frame: " FRAME_CONFIG_STRING));
        handle_param_request_list(msg);
        break;
    }

    case MAVLINK_MSG_ID_PARAM_SET:     // 23
    {
        handle_param_set(msg, &DataFlash);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST: // MAV ID: 38
    {
        handle_mission_write_partial_list(mission, msg);
        break;
    }

    // GCS has sent us a command from GCS, store to EEPROM
    case MAVLINK_MSG_ID_MISSION_ITEM:           // MAV ID: 39
    {
        handle_mission_item(msg, mission);
        break;
    }

    // read an individual command from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_MISSION_REQUEST:     // MAV ID: 40
    {
        handle_mission_request(mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:    // MAV ID: 41
    {
        handle_mission_set_current(mission, msg);
        break;
    }

    // GCS request the full list of commands, we return just the number and leave the GCS to then request each command individually
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:       // MAV ID: 43
    {
        handle_mission_request_list(mission, msg);
        break;
    }

    // GCS provides the full number of commands it wishes to upload
    //  individual commands will then be sent from the GCS using the MAVLINK_MSG_ID_MISSION_ITEM message
    case MAVLINK_MSG_ID_MISSION_COUNT:          // MAV ID: 44
    {
        handle_mission_count(mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:      // MAV ID: 45
    {
        handle_mission_clear_all(mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:    // MAV ID: 66
    {
        handle_request_data_stream(msg, false);
        break;
    }

    case MAVLINK_MSG_ID_GIMBAL_REPORT:
    {
#if MOUNT == ENABLED
        handle_gimbal_report(camera_mount, msg);
#endif
        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:       // MAV ID: 70
    {
        // allow override of RC channel values for HIL
        // or for complete GCS control of switch position
        // and RC PWM values.
        if(msg->sysid != g.sysid_my_gcs) break;                         // Only accept control from our gcs
        mavlink_rc_channels_override_t packet;
        int16_t v[8];
        mavlink_msg_rc_channels_override_decode(msg, &packet);

        v[0] = packet.chan1_raw;
        v[1] = packet.chan2_raw;
        v[2] = packet.chan3_raw;
        v[3] = packet.chan4_raw;
        v[4] = packet.chan5_raw;
        v[5] = packet.chan6_raw;
        v[6] = packet.chan7_raw;
        v[7] = packet.chan8_raw;
        hal.rcin->set_overrides(v, 8);

        // record that rc are overwritten so we can trigger a failsafe if we lose contact with groundstation
        failsafe.rc_override_active = true;
        // a RC override message is consiered to be a 'heartbeat' from the ground station for failsafe purposes
        failsafe.last_heartbeat_ms = millis();
        break;
    }

    // Pre-Flight calibration requests
    case MAVLINK_MSG_ID_COMMAND_LONG:       // MAV ID: 76
    {
        // decode packet
        mavlink_command_long_t packet;
        mavlink_msg_command_long_decode(msg, &packet);

        switch(packet.command) {

        case MAV_CMD_NAV_TAKEOFF:
            // param4 : yaw angle   (not supported)
            // param5 : latitude    (not supported)
            // param6 : longitude   (not supported)
            // param7 : altitude [metres]
            if (motors.armed() &&  control_mode == GUIDED) {
                set_auto_armed(true);
                float takeoff_alt = packet.param7 * 100;      // Convert m to cm
                takeoff_alt = max(takeoff_alt,current_loc.alt);
                takeoff_alt = max(takeoff_alt,100.0f);
                guided_takeoff_start(takeoff_alt);
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
            break;

        case MAV_CMD_NAV_LOITER_UNLIM:
            if (set_mode(LOITER)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            if (set_mode(RTL)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_NAV_LAND:
            if (set_mode(LAND)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_CONDITION_YAW:
            // param1 : target angle [0-360]
            // param2 : speed during change [deg per second]
            // param3 : direction (-1:ccw, +1:cw)
            // param4 : relative offset (1) or absolute angle (0)
            if ((packet.param1 >= 0.0f)   &&
            	(packet.param1 <= 360.0f) &&
            	((packet.param4 == 0) || (packet.param4 == 1))) {
            	set_auto_yaw_look_at_heading(packet.param1, packet.param2, (int8_t)packet.param3, (uint8_t)packet.param4);
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
            break;

        case MAV_CMD_DO_CHANGE_SPEED:
            // param1 : unused
            // param2 : new speed in m/s
            // param3 : unused
            // param4 : unused
            if (packet.param2 > 0.0f) {
                wp_nav.set_speed_xy(packet.param2 * 100.0f);
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_FAILED;
            }
            break;

        case MAV_CMD_DO_SET_HOME:
            // param1 : use current (1=use current location, 0=use specified location)
            // param5 : latitude
            // param6 : longitude
            // param7 : altitude (absolute)
            result = MAV_RESULT_FAILED; // assume failure
            if(packet.param1 == 1 || (packet.param5 == 0 && packet.param6 == 0 && packet.param7 == 0)) {
                if (set_home_to_current_location_and_lock()) {
                    result = MAV_RESULT_ACCEPTED;
                }
            } else {
                Location new_home_loc;
                new_home_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
                new_home_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
                new_home_loc.alt = (int32_t)(packet.param7 * 100.0f);
                if (!far_from_EKF_origin(new_home_loc)) {
                    if (set_home_and_lock(new_home_loc)) {
                        result = MAV_RESULT_ACCEPTED;
                    }
                }
            }
            break;

        case MAV_CMD_DO_SET_ROI:
            // param1 : regional of interest mode (not supported)
            // param2 : mission index/ target id (not supported)
            // param3 : ROI index (not supported)
            // param5 : x / lat
            // param6 : y / lon
            // param7 : z / alt
            Location roi_loc;
            roi_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
            roi_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
            roi_loc.alt = (int32_t)(packet.param7 * 100.0f);
            set_auto_yaw_roi(roi_loc);
            result = MAV_RESULT_ACCEPTED;
            break;

        case MAV_CMD_MISSION_START:
            if (set_mode(AUTO)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_CALIBRATION:
            // exit immediately if armed
            if (motors.armed()) {
                result = MAV_RESULT_FAILED;
                break;
            }
            if (packet.param1 == 1) {
                // gyro offset calibration
                ins.init_gyro();
                // reset ahrs gyro bias
                if (ins.gyro_calibrated_ok_all()) {
                    ahrs.reset_gyro_drift();
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else if (packet.param3 == 1) {
                // fast barometer calibration
                init_barometer(false);
                result = MAV_RESULT_ACCEPTED;
            } else if (packet.param4 == 1) {
                result = MAV_RESULT_UNSUPPORTED;
            } else if (packet.param5 == 1) {
                // 3d accel calibration
                float trim_roll, trim_pitch;
                // this blocks
                AP_InertialSensor_UserInteract_MAVLink interact(this);
                if(ins.calibrate_accel(&interact, trim_roll, trim_pitch)) {
                    // reset ahrs's trim to suggested values from calibration routine
                    ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
                    result = MAV_RESULT_ACCEPTED;
                } else {
                    result = MAV_RESULT_FAILED;
                }
            } else if (packet.param6 == 1) {
                // compassmot calibration
                result = mavlink_compassmot(chan);
            }
            break;

        case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
            if (packet.param1 == 2) {
                // save first compass's offsets
                compass.set_and_save_offsets(0, packet.param2, packet.param3, packet.param4);
                result = MAV_RESULT_ACCEPTED;
            }
            if (packet.param1 == 5) {
                // save secondary compass's offsets
                compass.set_and_save_offsets(1, packet.param2, packet.param3, packet.param4);
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_COMPONENT_ARM_DISARM:
            if (packet.param1 == 1.0f) {
                // attempt to arm and return success or failure
                if (init_arm_motors(true)) {
                    result = MAV_RESULT_ACCEPTED;
                }
            } else if (packet.param1 == 0.0f && (mode_has_manual_throttle(control_mode) || ap.land_complete))  {
                init_disarm_motors();
                result = MAV_RESULT_ACCEPTED;
            } else {
                result = MAV_RESULT_UNSUPPORTED;
            }
            break;

        case MAV_CMD_DO_SET_SERVO:
            if (ServoRelayEvents.do_set_servo(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_SERVO:
            if (ServoRelayEvents.do_repeat_servo(packet.param1, packet.param2, packet.param3, packet.param4*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_SET_RELAY:
            if (ServoRelayEvents.do_set_relay(packet.param1, packet.param2)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_REPEAT_RELAY:
            if (ServoRelayEvents.do_repeat_relay(packet.param1, packet.param2, packet.param3*1000)) {
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (packet.param1 == 1 || packet.param1 == 3) {
                AP_Notify::events.firmware_update = 1;
                update_notify();
                hal.scheduler->delay(50);
                // when packet.param1 == 3 we reboot to hold in bootloader
                hal.scheduler->reboot(packet.param1 == 3);
                result = MAV_RESULT_ACCEPTED;
            }
            break;

        case MAV_CMD_DO_FENCE_ENABLE:
#if AC_FENCE == ENABLED
            result = MAV_RESULT_ACCEPTED;
            switch ((uint16_t)packet.param1) {
                case 0:
                    fence.enable(false);
                    break;
                case 1:
                    fence.enable(true);
                    break;
                default:
                    result = MAV_RESULT_FAILED;
                    break;
            }
#else
            // if fence code is not included return failure
            result = MAV_RESULT_FAILED;
#endif
            break;

#if PARACHUTE == ENABLED
        case MAV_CMD_DO_PARACHUTE:
            // configure or release parachute
            result = MAV_RESULT_ACCEPTED;
            switch ((uint16_t)packet.param1) {
                case PARACHUTE_DISABLE:
                    parachute.enabled(false);
                    Log_Write_Event(DATA_PARACHUTE_DISABLED);
                    break;
                case PARACHUTE_ENABLE:
                    parachute.enabled(true);
                    Log_Write_Event(DATA_PARACHUTE_ENABLED);
                    break;
                case PARACHUTE_RELEASE:
                    // treat as a manual release which performs some additional check of altitude
                    parachute_manual_release();
                    break;
                default:
                    result = MAV_RESULT_FAILED;
                    break;
            }
#endif

        case MAV_CMD_DO_MOTOR_TEST:
            // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
            // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
            // param3 : throttle (range depends upon param2)
            // param4 : timeout (in seconds)
            result = mavlink_motor_test_start(chan, (uint8_t)packet.param1, (uint8_t)packet.param2, (uint16_t)packet.param3, packet.param4);
            break;

#if EPM_ENABLED == ENABLED
        case MAV_CMD_DO_GRIPPER:
            // param1 : gripper number (ignored)
            // param2 : action (0=release, 1=grab). See GRIPPER_ACTIONS enum.
            if(!epm.enabled()) {
                result = MAV_RESULT_FAILED;
            } else {
                result = MAV_RESULT_ACCEPTED;
                switch ((uint8_t)packet.param2) {
                    case GRIPPER_ACTION_RELEASE:
                        epm.release();
                        break;
                    case GRIPPER_ACTION_GRAB:
                        epm.grab();
                        break;
                    default:
                        result = MAV_RESULT_FAILED;
                        break;
                }
            }
            break;
#endif

        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: {
            if (packet.param1 == 1) {
                gcs[chan-MAVLINK_COMM_0].send_autopilot_version();
                result = MAV_RESULT_ACCEPTED;
            }
            break;
        }

        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
        }

        // send ACK or NAK
        mavlink_msg_command_ack_send_buf(msg, chan, packet.command, result);

        break;
    }

    case MAVLINK_MSG_ID_COMMAND_ACK:        // MAV ID: 77
    {
        command_ack_counter++;
        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:     // MAV ID: 84
    {
        // decode packet
        mavlink_set_position_target_local_ned_t packet;
        mavlink_msg_set_position_target_local_ned_decode(msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if ((control_mode != GUIDED) && !(control_mode == AUTO && auto_mode == Auto_NavGuided)) {
            break;
        }

        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;

        /*
         * for future use:
         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
         * bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
         * bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
         */

        if (!pos_ignore && !vel_ignore && acc_ignore) {
            Vector3f pos_ned = Vector3f(packet.x * 100.0f, packet.y * 100.0f, -packet.z * 100.0f);
            pos_ned.z = pv_alt_above_origin(pos_ned.z);
            guided_set_destination_posvel(pos_ned, Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
        } else if (pos_ignore && !vel_ignore && acc_ignore) {
            guided_set_velocity(Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            Vector3f pos_ned = Vector3f(packet.x * 100.0f, packet.y * 100.0f, -packet.z * 100.0f);
            pos_ned.z = pv_alt_above_origin(pos_ned.z);
            guided_set_destination(pos_ned);
        } else {
            result = MAV_RESULT_FAILED;
        }

        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:    // MAV ID: 86
    {
        // decode packet
        mavlink_set_position_target_global_int_t packet;
        mavlink_msg_set_position_target_global_int_decode(msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if ((control_mode != GUIDED) && !(control_mode == AUTO && auto_mode == Auto_NavGuided)) {
            break;
        }

        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;

        /*
         * for future use:
         * bool force           = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE;
         * bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
         * bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
         */

        Vector3f pos_ned;

        if(!pos_ignore) {
            Location loc;
            loc.lat = packet.lat_int;
            loc.lng = packet.lon_int;
            loc.alt = packet.alt*100;
            switch (packet.coordinate_frame) {
                case MAV_FRAME_GLOBAL_RELATIVE_ALT:
                case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                    loc.flags.relative_alt = true;
                    loc.flags.terrain_alt = false;
                    break;
                case MAV_FRAME_GLOBAL_TERRAIN_ALT:
                case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                    loc.flags.relative_alt = true;
                    loc.flags.terrain_alt = true;
                    break;
                case MAV_FRAME_GLOBAL:
                case MAV_FRAME_GLOBAL_INT:
                default:
                    loc.flags.relative_alt = false;
                    loc.flags.terrain_alt = false;
                    break;
            }
            pos_ned = pv_location_to_vector(loc);
        }

        if (!pos_ignore && !vel_ignore && acc_ignore) {
            guided_set_destination_posvel(pos_ned, Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
        } else if (pos_ignore && !vel_ignore && acc_ignore) {
            guided_set_velocity(Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f));
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            guided_set_destination(pos_ned);
        } else {
            result = MAV_RESULT_FAILED;
        }

        break;
    }

#if HIL_MODE != HIL_MODE_DISABLED
    case MAVLINK_MSG_ID_HIL_STATE:          // MAV ID: 90
    {
        mavlink_hil_state_t packet;
        mavlink_msg_hil_state_decode(msg, &packet);

        // set gps hil sensor
        Location loc;
        loc.lat = packet.lat;
        loc.lng = packet.lon;
        loc.alt = packet.alt/10;
        Vector3f vel(packet.vx, packet.vy, packet.vz);
        vel *= 0.01f;

        gps.setHIL(0, AP_GPS::GPS_OK_FIX_3D,
                   packet.time_usec/1000,
                   loc, vel, 10, 0, true);

        // rad/sec
        Vector3f gyros;
        gyros.x = packet.rollspeed;
        gyros.y = packet.pitchspeed;
        gyros.z = packet.yawspeed;

        // m/s/s
        Vector3f accels;
        accels.x = packet.xacc * (GRAVITY_MSS/1000.0);
        accels.y = packet.yacc * (GRAVITY_MSS/1000.0);
        accels.z = packet.zacc * (GRAVITY_MSS/1000.0);

        ins.set_gyro(0, gyros);

        ins.set_accel(0, accels);

        barometer.setHIL(packet.alt*0.001f);
        compass.setHIL(packet.roll, packet.pitch, packet.yaw);

        break;
    }
#endif //  HIL_MODE != HIL_MODE_DISABLED

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:       // MAV ID: 109
    {
        handle_radio_status(msg, DataFlash, should_log(MASK_LOG_PM));
        break;
    }

    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
    case MAVLINK_MSG_ID_LOG_ERASE:
        in_log_download = true;
        // fallthru
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        if (!in_mavlink_delay && !motors.armed()) {
            handle_log_message(msg, DataFlash);
        }
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        in_log_download = false;
        if (!in_mavlink_delay && !motors.armed()) {
            handle_log_message(msg, DataFlash);
        }
        break;

#if HAL_CPU_CLASS > HAL_CPU_CLASS_16
    case MAVLINK_MSG_ID_SERIAL_CONTROL:
        handle_serial_control(msg, gps);
        break;

    case MAVLINK_MSG_ID_GPS_INJECT_DATA:
        handle_gps_inject(msg, gps);
        result = MAV_RESULT_ACCEPTED;
        break;

#endif

#if CAMERA == ENABLED
    case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:      // MAV ID: 202
        break;

    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
        camera.control_msg(msg);
        log_picture();
        break;
#endif // CAMERA == ENABLED

#if MOUNT == ENABLED
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE:        // MAV ID: 204
        camera_mount.configure_msg(msg);
        break;

    case MAVLINK_MSG_ID_MOUNT_CONTROL:
        camera_mount.control_msg(msg);
        break;
#endif // MOUNT == ENABLED

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE
        terrain.handle_data(chan, msg);
#endif
        break;

#if AC_RALLY == ENABLED
    // receive a rally point from GCS and store in EEPROM
    case MAVLINK_MSG_ID_RALLY_POINT: {
        mavlink_rally_point_t packet;
        mavlink_msg_rally_point_decode(msg, &packet);

        if (packet.idx >= rally.get_rally_total() ||
            packet.idx >= rally.get_rally_max()) {
            send_text_P(SEVERITY_LOW,PSTR("bad rally point message ID"));
            break;
        }

        if (packet.count != rally.get_rally_total()) {
            send_text_P(SEVERITY_LOW,PSTR("bad rally point message count"));
            break;
        }

        RallyLocation rally_point;
        rally_point.lat = packet.lat;
        rally_point.lng = packet.lng;
        rally_point.alt = packet.alt;
        rally_point.break_alt = packet.break_alt;
        rally_point.land_dir = packet.land_dir;
        rally_point.flags = packet.flags;

        if (!rally.set_rally_point_with_index(packet.idx, rally_point)) {
            send_text_P(SEVERITY_HIGH, PSTR("error setting rally point"));
        }

        break;
    }

    //send a rally point to the GCS
    case MAVLINK_MSG_ID_RALLY_FETCH_POINT: {
        //send_text_P(SEVERITY_HIGH, PSTR("## getting rally point in GCS_Mavlink.pde 1")); // #### TEMP

        mavlink_rally_fetch_point_t packet;
        mavlink_msg_rally_fetch_point_decode(msg, &packet);

        //send_text_P(SEVERITY_HIGH, PSTR("## getting rally point in GCS_Mavlink.pde 2")); // #### TEMP

        if (packet.idx > rally.get_rally_total()) {
            send_text_P(SEVERITY_LOW, PSTR("bad rally point index"));
            break;
        }

        //send_text_P(SEVERITY_HIGH, PSTR("## getting rally point in GCS_Mavlink.pde 3")); // #### TEMP

        RallyLocation rally_point;
        if (!rally.get_rally_point_with_index(packet.idx, rally_point)) {
           send_text_P(SEVERITY_LOW, PSTR("failed to set rally point"));
           break;
        }

        //send_text_P(SEVERITY_HIGH, PSTR("## getting rally point in GCS_Mavlink.pde 4")); // #### TEMP

        mavlink_msg_rally_point_send_buf(msg,
                                         chan, msg->sysid, msg->compid, packet.idx,
                                         rally.get_rally_total(), rally_point.lat, rally_point.lng,
                                         rally_point.alt, rally_point.break_alt, rally_point.land_dir,
                                         rally_point.flags);

        //send_text_P(SEVERITY_HIGH, PSTR("## getting rally point in GCS_Mavlink.pde 5")); // #### TEMP

        break;
    }
#endif // AC_RALLY == ENABLED

    case MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST:
        gcs[chan-MAVLINK_COMM_0].send_autopilot_version();
        break;

    case MAVLINK_MSG_ID_LED_CONTROL:
        // send message to Notify
        AP_Notify::handle_led_control(msg);
        break;

    }     // end switch
} // end handle mavlink


/*
 *  a delay() callback that processes MAVLink packets. We set this as the
 *  callback in long running library initialisation routines to allow
 *  MAVLink to process packets while waiting for the initialisation to
 *  complete
 */
static void mavlink_delay_cb()
{
    static uint32_t last_1hz, last_50hz, last_5s;
    if (!gcs[0].initialised || in_mavlink_delay) return;

    in_mavlink_delay = true;

    uint32_t tnow = millis();
    if (tnow - last_1hz > 1000) {
        last_1hz = tnow;
        gcs_send_heartbeat();
        gcs_send_message(MSG_EXTENDED_STATUS1);
    }
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        gcs_check_input();
        gcs_data_stream_send();
        gcs_send_deferred();
        notify.update();
    }
    if (tnow - last_5s > 5000) {
        last_5s = tnow;
        gcs_send_text_P(SEVERITY_LOW, PSTR("Initialising APM..."));
    }
    check_usb_mux();

    in_mavlink_delay = false;
}

/*
 *  send a message on both GCS links
 */
static void gcs_send_message(enum ap_message id)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].send_message(id);
        }
    }
}

/*
 *  send data streams in the given rate range on both links
 */
static void gcs_data_stream_send(void)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].data_stream_send();
        }
    }
}

/*
 *  look for incoming commands on the GCS links
 */
static void gcs_check_input(void)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
#if CLI_ENABLED == ENABLED
            gcs[i].update(g.cli_enabled==1?run_cli:NULL);
#else
            gcs[i].update(NULL);
#endif
        }
    }
}

static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].send_text_P(severity, str);
        }
    }
}

/*
 *  send a low priority formatted message to the GCS
 *  only one fits in the queue, so if you send more than one before the
 *  last one gets into the serial buffer then the old one will be lost
 */
static void gcs_send_text_fmt(const prog_char_t *fmt, ...)
{
    va_list arg_list;
    gcs[0].pending_status.severity = (uint8_t)SEVERITY_LOW;
    va_start(arg_list, fmt);
    hal.util->vsnprintf_P((char *)gcs[0].pending_status.text,
            sizeof(gcs[0].pending_status.text), fmt, arg_list);
    va_end(arg_list);
    gcs[0].send_message(MSG_STATUSTEXT);
    for (uint8_t i=1; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].pending_status = gcs[0].pending_status;
            gcs[i].send_message(MSG_STATUSTEXT);
        }
    }
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/Log.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash log memory
// Code to interact with the user to dump or erase logs

#if CLI_ENABLED == ENABLED
// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static bool     print_log_menu(void);
static int8_t   dump_log(uint8_t argc,                  const Menu::arg *argv);
static int8_t   erase_logs(uint8_t argc,                const Menu::arg *argv);
static int8_t   select_logs(uint8_t argc,               const Menu::arg *argv);

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
const struct Menu::command log_menu_commands[] PROGMEM = {
    {"dump",        dump_log},
    {"erase",       erase_logs},
    {"enable",      select_logs},
    {"disable",     select_logs}
};

// A Macro to create the Menu
MENU2(log_menu, "Log", log_menu_commands, print_log_menu);

static bool
print_log_menu(void)
{
    cliSerial->printf_P(PSTR("logs enabled: "));

    if (0 == g.log_bitmask) {
        cliSerial->printf_P(PSTR("none"));
    }else{
        if (g.log_bitmask & MASK_LOG_ATTITUDE_FAST) cliSerial->printf_P(PSTR(" ATTITUDE_FAST"));
        if (g.log_bitmask & MASK_LOG_ATTITUDE_MED) cliSerial->printf_P(PSTR(" ATTITUDE_MED"));
        if (g.log_bitmask & MASK_LOG_GPS) cliSerial->printf_P(PSTR(" GPS"));
        if (g.log_bitmask & MASK_LOG_PM) cliSerial->printf_P(PSTR(" PM"));
        if (g.log_bitmask & MASK_LOG_CTUN) cliSerial->printf_P(PSTR(" CTUN"));
        if (g.log_bitmask & MASK_LOG_NTUN) cliSerial->printf_P(PSTR(" NTUN"));
        if (g.log_bitmask & MASK_LOG_RCIN) cliSerial->printf_P(PSTR(" RCIN"));
        if (g.log_bitmask & MASK_LOG_IMU) cliSerial->printf_P(PSTR(" IMU"));
        if (g.log_bitmask & MASK_LOG_CMD) cliSerial->printf_P(PSTR(" CMD"));
        if (g.log_bitmask & MASK_LOG_CURRENT) cliSerial->printf_P(PSTR(" CURRENT"));
        if (g.log_bitmask & MASK_LOG_RCOUT) cliSerial->printf_P(PSTR(" RCOUT"));
        if (g.log_bitmask & MASK_LOG_OPTFLOW) cliSerial->printf_P(PSTR(" OPTFLOW"));
        if (g.log_bitmask & MASK_LOG_COMPASS) cliSerial->printf_P(PSTR(" COMPASS"));
        if (g.log_bitmask & MASK_LOG_CAMERA) cliSerial->printf_P(PSTR(" CAMERA"));
    }

    cliSerial->println();

    DataFlash.ListAvailableLogs(cliSerial);

    return(true);
}

#if CLI_ENABLED == ENABLED
static int8_t
dump_log(uint8_t argc, const Menu::arg *argv)
{
    int16_t dump_log;
    uint16_t dump_log_start;
    uint16_t dump_log_end;
    uint16_t last_log_num;

    // check that the requested log number can be read
    dump_log = argv[1].i;
    last_log_num = DataFlash.find_last_log();

    if (dump_log == -2) {
        DataFlash.DumpPageInfo(cliSerial);
        return(-1);
    } else if (dump_log <= 0) {
        cliSerial->printf_P(PSTR("dumping all\n"));
        Log_Read(0, 1, 0);
        return(-1);
    } else if ((argc != 2) || ((uint16_t)dump_log <= (last_log_num - DataFlash.get_num_logs())) || (static_cast<uint16_t>(dump_log) > last_log_num)) {
        cliSerial->printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    Log_Read((uint16_t)dump_log, dump_log_start, dump_log_end);
    return (0);
}
#endif

static int8_t
erase_logs(uint8_t argc, const Menu::arg *argv)
{
    in_mavlink_delay = true;
    do_erase_logs();
    in_mavlink_delay = false;
    return 0;
}

static int8_t
select_logs(uint8_t argc, const Menu::arg *argv)
{
    uint16_t bits;

    if (argc != 2) {
        cliSerial->printf_P(PSTR("missing log type\n"));
        return(-1);
    }

    bits = 0;

    // Macro to make the following code a bit easier on the eye.
    // Pass it the capitalised name of the log option, as defined
    // in defines.h but without the LOG_ prefix.  It will check for
    // that name as the argument to the command, and set the bit in
    // bits accordingly.
    //
    if (!strcasecmp_P(argv[1].str, PSTR("all"))) {
        bits = ~0;
    } else {
 #define TARG(_s)        if (!strcasecmp_P(argv[1].str, PSTR(# _s))) bits |= MASK_LOG_ ## _s
        TARG(ATTITUDE_FAST);
        TARG(ATTITUDE_MED);
        TARG(GPS);
        TARG(PM);
        TARG(CTUN);
        TARG(NTUN);
        TARG(RCIN);
        TARG(IMU);
        TARG(CMD);
        TARG(CURRENT);
        TARG(RCOUT);
        TARG(OPTFLOW);
        TARG(COMPASS);
        TARG(CAMERA);
 #undef TARG
    }

    if (!strcasecmp_P(argv[0].str, PSTR("enable"))) {
        g.log_bitmask.set_and_save(g.log_bitmask | bits);
    }else{
        g.log_bitmask.set_and_save(g.log_bitmask & ~bits);
    }

    return(0);
}

static int8_t
process_logs(uint8_t argc, const Menu::arg *argv)
{
    log_menu.run();
    return 0;
}
#endif // CLI_ENABLED

static void do_erase_logs(void)
{
    gcs_send_text_P(SEVERITY_HIGH, PSTR("Erasing logs\n"));
    DataFlash.EraseAll();
    gcs_send_text_P(SEVERITY_HIGH, PSTR("Log erase complete\n"));
}

#if AUTOTUNE_ENABLED == ENABLED
struct PACKED log_AutoTune {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t axis;           // roll or pitch
    uint8_t tune_step;      // tuning PI or D up or down
    float   rate_target;    // target achieved rotation rate
    float   rate_min;       // maximum achieved rotation rate
    float   rate_max;       // maximum achieved rotation rate
    float   new_gain_rp;    // newly calculated gain
    float   new_gain_rd;    // newly calculated gain
    float   new_gain_sp;    // newly calculated gain
};

// Write an Autotune data packet
static void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float rate_target, float rate_min, float rate_max, float new_gain_rp, float new_gain_rd, float new_gain_sp)
{
    struct log_AutoTune pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AUTOTUNE_MSG),
        time_ms     : hal.scheduler->millis(),
        axis        : axis,
        tune_step   : tune_step,
        rate_target : rate_target,
        rate_min    : rate_min,
        rate_max    : rate_max,
        new_gain_rp : new_gain_rp,
        new_gain_rd : new_gain_rd,
        new_gain_sp : new_gain_sp
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_AutoTuneDetails {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    float    angle_cd;      // lean angle in centi-degrees
    float    rate_cds;      // current rotation rate in centi-degrees / second
};

// Write an Autotune data packet
static void Log_Write_AutoTuneDetails(float angle_cd, float rate_cds)
{
    struct log_AutoTuneDetails pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AUTOTUNEDETAILS_MSG),
        time_ms     : hal.scheduler->millis(),
        angle_cd    : angle_cd,
        rate_cds    : rate_cds
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif

// Write a Current data packet
static void Log_Write_Current()
{
    DataFlash.Log_Write_Current(battery, g.rc_3.servo_out);

    // also write power status
    DataFlash.Log_Write_Power();
}

struct PACKED log_Optflow {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t surface_quality;
    float flow_x;
    float flow_y;
    float body_x;
    float body_y;
};

// Write an optical flow packet
static void Log_Write_Optflow()
{
 #if OPTFLOW == ENABLED
    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }
    const Vector2f &flowRate = optflow.flowRate();
    const Vector2f &bodyRate = optflow.bodyRate();
    struct log_Optflow pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
        time_ms         : hal.scheduler->millis(),
        surface_quality : optflow.quality(),
        flow_x          : flowRate.x,
        flow_y          : flowRate.y,
        body_x          : bodyRate.x,
        body_y          : bodyRate.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
 #endif     // OPTFLOW == ENABLED
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    float    desired_pos_x;
    float    desired_pos_y;
    float    pos_x;
    float    pos_y;
    float    desired_vel_x;
    float    desired_vel_y;
    float    vel_x;
    float    vel_y;
    float    desired_accel_x;
    float    desired_accel_y;
};

// Write an Nav Tuning packet
static void Log_Write_Nav_Tuning()
{
    const Vector3f &pos_target = pos_control.get_pos_target();
    const Vector3f &vel_target = pos_control.get_vel_target();
    const Vector3f &accel_target = pos_control.get_accel_target();
    const Vector3f &position = inertial_nav.get_position();
    const Vector3f &velocity = inertial_nav.get_velocity();

    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NAV_TUNING_MSG),
        time_ms         : hal.scheduler->millis(),
        desired_pos_x   : pos_target.x,
        desired_pos_y   : pos_target.y,
        pos_x           : position.x,
        pos_y           : position.y,
        desired_vel_x   : vel_target.x,
        desired_vel_y   : vel_target.y,
        vel_x           : velocity.x,
        vel_y           : velocity.y,
        desired_accel_x : accel_target.x,
        desired_accel_y : accel_target.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t  throttle_in;
    int16_t  angle_boost;
    int16_t  throttle_out;
    float    desired_alt;
    float    inav_alt;
    int32_t  baro_alt;
    int16_t  desired_sonar_alt;
    int16_t  sonar_alt;
    int16_t  desired_climb_rate;
    int16_t  climb_rate;
};

// Write a control tuning packet
static void Log_Write_Control_Tuning()
{
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_ms             : hal.scheduler->millis(),
        throttle_in         : g.rc_3.control_in,
        angle_boost         : attitude_control.angle_boost(),
        throttle_out        : g.rc_3.servo_out,
        desired_alt         : pos_control.get_alt_target() / 100.0f,
        inav_alt            : inertial_nav.get_altitude() / 100.0f,
        baro_alt            : baro_alt,
        desired_sonar_alt   : (int16_t)target_sonar_alt,
        sonar_alt           : sonar_alt,
        desired_climb_rate  : (int16_t)pos_control.get_vel_target_z(),
        climb_rate          : climb_rate
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint16_t num_long_running;
    uint16_t num_loops;
    uint32_t max_time;
    int16_t  pm_test;
    uint8_t i2c_lockup_count;
    uint16_t ins_error_count;
};

// Write a performance monitoring packet
static void Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        num_long_running : perf_info_get_num_long_running(),
        num_loops        : perf_info_get_num_loops(),
        max_time         : perf_info_get_max_time(),
        pm_test          : pmTest1,
        i2c_lockup_count : hal.i2c->lockup_count(),
        ins_error_count  : ins.error_count()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Write a mission command. Total length : 36 bytes
static void Log_Write_Cmd(const AP_Mission::Mission_Command &cmd)
{
    mavlink_mission_item_t mav_cmd = {};
    AP_Mission::mission_cmd_to_mavlink(cmd,mav_cmd);
    DataFlash.Log_Write_MavCmd(mission.num_commands(),mav_cmd);
}

// Write an attitude packet
static void Log_Write_Attitude()
{
    Vector3f targets = attitude_control.angle_ef_targets();
    DataFlash.Log_Write_Attitude(ahrs, targets);

#if AP_AHRS_NAVEKF_AVAILABLE
 #if OPTFLOW == ENABLED
    DataFlash.Log_Write_EKF(ahrs,optflow.enabled());
 #else
    DataFlash.Log_Write_EKF(ahrs,false);
 #endif
    DataFlash.Log_Write_AHRS2(ahrs);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    sitl.Log_Write_SIMSTATE(DataFlash);
#endif
}

struct PACKED log_Rate {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    float   control_roll;
    float   roll;
    float   roll_out;
    float   control_pitch;
    float   pitch;
    float   pitch_out;
    float   control_yaw;
    float   yaw;
    float   yaw_out;
    float   control_accel;
    float   accel;
    float   accel_out;
};

// Write an rate packet
static void Log_Write_Rate()
{
    const Vector3f &rate_targets = attitude_control.rate_bf_targets();
    const Vector3f &accel_target = pos_control.get_accel_target();
    struct log_Rate pkt_rate = {
        LOG_PACKET_HEADER_INIT(LOG_RATE_MSG),
        time_ms         : hal.scheduler->millis(),
        control_roll    : (float)rate_targets.x,
        roll            : (float)(ahrs.get_gyro().x * AC_ATTITUDE_CONTROL_DEGX100),
        roll_out        : (float)(motors.get_roll()),
        control_pitch   : (float)rate_targets.y,
        pitch           : (float)(ahrs.get_gyro().y * AC_ATTITUDE_CONTROL_DEGX100),
        pitch_out       : (float)(motors.get_pitch()),
        control_yaw     : (float)rate_targets.z,
        yaw             : (float)(ahrs.get_gyro().z * AC_ATTITUDE_CONTROL_DEGX100),
        yaw_out         : (float)(motors.get_yaw()),
        control_accel   : (float)accel_target.z,
        accel           : (float)(-(ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f),
        accel_out       : (float)(motors.get_throttle_out())
    };
    DataFlash.WriteBlock(&pkt_rate, sizeof(pkt_rate));
}

struct PACKED log_MotBatt {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    float   lift_max;
    float   bat_volt;
    float   bat_res;
    float   th_limit;
};

// Write an rate packet
static void Log_Write_MotBatt()
{
    struct log_MotBatt pkt_mot = {
        LOG_PACKET_HEADER_INIT(LOG_MOTBATT_MSG),
        time_ms         : hal.scheduler->millis(),
        lift_max        : (float)(motors.get_lift_max()),
        bat_volt        : (float)(motors.get_batt_voltage_filt()),
        bat_res         : (float)(motors.get_batt_resistance()),
        th_limit        : (float)(motors.get_throttle_limit())
    };
    DataFlash.WriteBlock(&pkt_mot, sizeof(pkt_mot));
}

struct PACKED log_Startup {
    LOG_PACKET_HEADER;
};

// Write Startup packet
static void Log_Write_Startup()
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Event {
    LOG_PACKET_HEADER;
    uint8_t id;
};

// Wrote an event packet
static void Log_Write_Event(uint8_t id)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Event pkt = {
            LOG_PACKET_HEADER_INIT(LOG_EVENT_MSG),
            id  : id
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
UNUSED_FUNCTION
static void Log_Write_Data(uint8_t id, int16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
UNUSED_FUNCTION 
static void Log_Write_Data(uint8_t id, uint16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
static void Log_Write_Data(uint8_t id, int32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
static void Log_Write_Data(uint8_t id, uint32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint8_t id;
    float data_value;
};

// Write a float data packet
UNUSED_FUNCTION
static void Log_Write_Data(uint8_t id, float value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            id          : id,
            data_value  : value
        };
        DataFlash.WriteBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Error {
    LOG_PACKET_HEADER;
    uint8_t sub_system;
    uint8_t error_code;
};

// Write an error packet
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code)
{
    struct log_Error pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ERROR_MSG),
        sub_system    : sub_system,
        error_code    : error_code,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

static void Log_Write_Baro(void)
{
    DataFlash.Log_Write_Baro(barometer);
}

static const struct LogStructure log_structure[] PROGMEM = {
    LOG_COMMON_STRUCTURES,
#if AUTOTUNE_ENABLED == ENABLED
    { LOG_AUTOTUNE_MSG, sizeof(log_AutoTune),
      "ATUN", "IBBffffff",       "TimeMS,Axis,TuneStep,RateTarg,RateMin,RateMax,RP,RD,SP" },
    { LOG_AUTOTUNEDETAILS_MSG, sizeof(log_AutoTuneDetails),
      "ATDE", "Iff",          "TimeMS,Angle,Rate" },
#endif
    { LOG_OPTFLOW_MSG, sizeof(log_Optflow),       
      "OF",   "IBffff",   "TimeMS,Qual,flowX,flowY,bodyX,bodyY" },
    { LOG_NAV_TUNING_MSG, sizeof(log_Nav_Tuning),       
      "NTUN", "Iffffffffff", "TimeMS,DPosX,DPosY,PosX,PosY,DVelX,DVelY,VelX,VelY,DAccX,DAccY" },
    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Ihhhffecchh", "TimeMS,ThrIn,AngBst,ThrOut,DAlt,Alt,BarAlt,DSAlt,SAlt,DCRt,CRt" },
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance), 
      "PM",  "HHIhBH",    "NLon,NLoop,MaxT,PMT,I2CErr,INSErr" },
    { LOG_RATE_MSG, sizeof(log_Rate),
      "RATE", "Iffffffffffff",  "TimeMS,RDes,R,ROut,PDes,P,POut,YDes,Y,YOut,ADes,A,AOut" },
    { LOG_MOTBATT_MSG, sizeof(log_MotBatt),
      "MOTB", "Iffff",  "TimeMS,LiftMax,BatVolt,BatRes,ThLimit" },
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "",            "" },
    { LOG_EVENT_MSG, sizeof(log_Event),         
      "EV",   "B",           "Id" },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),         
      "D16",   "Bh",         "Id,Value" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),         
      "DU16",  "BH",         "Id,Value" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),         
      "D32",   "Bi",         "Id,Value" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),         
      "DU32",  "BI",         "Id,Value" },
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),         
      "DFLT",  "Bf",         "Id,Value" },
    { LOG_ERROR_MSG, sizeof(log_Error),         
      "ERR",   "BB",         "Subsys,ECode" },
};

#if CLI_ENABLED == ENABLED
// Read the DataFlash log memory
static void Log_Read(uint16_t log_num, uint16_t start_page, uint16_t end_page)
{
    cliSerial->printf_P(PSTR("\n" FIRMWARE_STRING
                             "\nFree RAM: %u\n"
                             "\nFrame: " FRAME_CONFIG_STRING "\n"),
                        (unsigned) hal.util->available_memory());

    cliSerial->println_P(PSTR(HAL_BOARD_NAME));

    DataFlash.LogReadProcess(log_num, start_page, end_page, 
                             print_flight_mode,
                             cliSerial);
}
#endif // CLI_ENABLED

// start a new log
static void start_logging() 
{
    if (g.log_bitmask != 0) {
        if (!ap.logging_started) {
            ap.logging_started = true;
            in_mavlink_delay = true;
            DataFlash.StartNewLog();
            in_mavlink_delay = false;
            DataFlash.Log_Write_Message_P(PSTR(FIRMWARE_STRING));

#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
            DataFlash.Log_Write_Message_P(PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION));
#endif

            // write system identifier as well if available
            char sysid[40];
            if (hal.util->get_system_id(sysid)) {
                DataFlash.Log_Write_Message(sysid);
            }
            DataFlash.Log_Write_Message_P(PSTR("Frame: " FRAME_CONFIG_STRING));

            // log the flight mode
            DataFlash.Log_Write_Mode(control_mode);
        }
        // enable writes
        DataFlash.EnableWrites(true);
    }
}

#else // LOGGING_ENABLED

static void Log_Write_Startup() {}
#if AUTOTUNE_ENABLED == ENABLED
static void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float rate_target, float rate_min, float rate_max, float new_gain_rp, float new_gain_rd, float new_gain_sp) {}
static void Log_Write_AutoTuneDetails(float angle_cd, float rate_cds) {}
#endif
static void Log_Write_Current() {}
static void Log_Write_Attitude() {}
static void Log_Write_Rate() {}
static void Log_Write_MotBatt() {}
static void Log_Write_Data(uint8_t id, int16_t value){}
static void Log_Write_Data(uint8_t id, uint16_t value){}
static void Log_Write_Data(uint8_t id, int32_t value){}
static void Log_Write_Data(uint8_t id, uint32_t value){}
static void Log_Write_Data(uint8_t id, float value){}
static void Log_Write_Event(uint8_t id){}
static void Log_Write_Optflow() {}
static void Log_Write_Nav_Tuning() {}
static void Log_Write_Control_Tuning() {}
static void Log_Write_Performance() {}
static void Log_Write_Cmd(const AP_Mission::Mission_Command &cmd) {}
static void Log_Write_Error(uint8_t sub_system, uint8_t error_code) {}
static void Log_Write_Baro(void) {}
static int8_t process_logs(uint8_t argc, const Menu::arg *argv) {
    return 0;
}

#endif // LOGGING_DISABLED
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/Parameters.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  ArduCopter parameter definitions
 *
 */

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, {def_value : def} }
#define ASCALAR(v, name, def) { aparm.v.vtype, name, Parameters::k_param_ ## v, &aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, &v, {group_info : class::var_info} }

const AP_Param::Info var_info[] PROGMEM = {

    // @Param: TEENSY_SEN_F
    // @DisplayName: Teensy sensor update frequency
    // @Description: How often the sensor should measure in seconds
    // @Range: 1 10
    // @User: Standard
    GSCALAR(teensy_sen_f, "TEENSY_SEN_F", TEENSY_SEN_F_DEFAULT),

    // @Param: TEENSY_SEN_T
    // @DisplayName: Teensy sensor - measure time
    // @Description: How long time the sensor needs to do a measurement
    // @Range: 1 10
    // @User: Standard
    GSCALAR(teensy_sen_t, "TEENSY_SEN_T", TEENSY_SEN_T_DEFAULT),


    // @Param: SYSID_SW_MREV
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version, "SYSID_SW_MREV",   0),

    // @Param: SYSID_SW_TYPE
    // @DisplayName: Software Type
    // @Description: This is used by the ground station to recognise the software type (eg ArduPlane vs ArduCopter)
    // @Values: 0:ArduPlane,4:AntennaTracker,10:Copter,20:Rover
    // @User: Advanced
    GSCALAR(software_type,  "SYSID_SW_TYPE",   Parameters::k_software_type),

    // @Param: SYSID_THISMAV
    // @DisplayName: Mavlink version
    // @Description: Allows reconising the mavlink version
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_this_mav, "SYSID_THISMAV",   MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: My ground station number
    // @Description: Allows restricting radio overrides to only come from my ground station
    // @Values: 255:Mission Planner and DroidPlanner, 252: AP Planner 2
    // @User: Advanced
    GSCALAR(sysid_my_gcs,   "SYSID_MYGCS",     255),

#if CLI_ENABLED == ENABLED
    // @Param: CLI_ENABLED
    // @DisplayName: CLI Enable
    // @Description: This enables/disables the checking for three carriage returns on telemetry links on startup to enter the diagnostics command line interface
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(cli_enabled,    "CLI_ENABLED",    0),
#endif

    // @Param: PILOT_THR_FILT
    // @DisplayName: Throttle filter cutoff
    // @Description: Throttle filter cutoff (Hz) - active whenever altitude control is inactive - 0 to disable
    // @User: Advanced
    // @Units: Hz
    // @Range: 0 10
    // @Increment: .5
    GSCALAR(throttle_filt,  "PILOT_THR_FILT",     0),

    // @Group: SERIAL
    // @Path: ../libraries/AP_SerialManager/AP_SerialManager.cpp
    GOBJECT(serial_manager, "SERIAL",   AP_SerialManager),

    // @Param: TELEM_DELAY
    // @DisplayName: Telemetry startup delay
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Advanced
    // @Units: seconds
    // @Range: 0 10
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY",     0),

    // @Param: RTL_ALT
    // @DisplayName: RTL Altitude
    // @Description: The minimum altitude the model will move to before Returning to Launch.  Set to zero to return at current altitude.
    // @Units: Centimeters
    // @Range: 0 8000
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_altitude,   "RTL_ALT",     RTL_ALT),

    // @Param: RNGFND_GAIN
    // @DisplayName: Rangefinder gain
    // @Description: Used to adjust the speed with which the target altitude is changed when objects are sensed below the copter
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(sonar_gain,     "RNGFND_GAIN",           SONAR_GAIN_DEFAULT),

    // @Param: FS_BATT_ENABLE
    // @DisplayName: Battery Failsafe Enable
    // @Description: Controls whether failsafe will be invoked when battery voltage or current runs low
    // @Values: 0:Disabled,1:Land,2:RTL
    // @User: Standard
    GSCALAR(failsafe_battery_enabled, "FS_BATT_ENABLE", FS_BATT_DISABLED),

    // @Param: FS_BATT_VOLTAGE
    // @DisplayName: Failsafe battery voltage
    // @Description: Battery voltage to trigger failsafe. Set to 0 to disable battery voltage failsafe. If the battery voltage drops below this voltage then the copter will RTL
    // @Units: Volts
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(fs_batt_voltage,        "FS_BATT_VOLTAGE", FS_BATT_VOLTAGE_DEFAULT),

    // @Param: FS_BATT_MAH
    // @DisplayName: Failsafe battery milliAmpHours
    // @Description: Battery capacity remaining to trigger failsafe. Set to 0 to disable battery remaining failsafe. If the battery remaining drops below this level then the copter will RTL
    // @Units: mAh
    // @Increment: 50
    // @User: Standard
    GSCALAR(fs_batt_mah,            "FS_BATT_MAH", FS_BATT_MAH_DEFAULT),

    // @Param: FS_GCS_ENABLE
    // @DisplayName: Ground Station Failsafe Enable
    // @Description: Controls whether failsafe will be invoked (and what action to take) when connection with Ground station is lost for at least 5 seconds. NB. The GCS Failsafe is only active when RC_OVERRIDE is being used to control the vehicle.
    // @Values: 0:Disabled,1:Enabled always RTL,2:Enabled Continue with Mission in Auto Mode
    // @User: Standard
    GSCALAR(failsafe_gcs, "FS_GCS_ENABLE", FS_GCS_ENABLED_ALWAYS_RTL),

    // @Param: GPS_HDOP_GOOD
    // @DisplayName: GPS Hdop Good
    // @Description: GPS Hdop value at or below this value represent a good position.  Used for pre-arm checks
    // @Range: 100 900
    // @User: Advanced
    GSCALAR(gps_hdop_good, "GPS_HDOP_GOOD", GPS_HDOP_GOOD_DEFAULT),

    // @Param: MAG_ENABLE
    // @DisplayName: Compass enable/disable
    // @Description: Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(compass_enabled,        "MAG_ENABLE",   MAGNETOMETER),

    // @Param: SUPER_SIMPLE
    // @DisplayName: Super Simple Mode
    // @Description: Bitmask to enable Super Simple mode for some flight modes. Setting this to Disabled(0) will disable Super Simple Mode
    // @Values: 0:Disabled,1:Mode1,2:Mode2,3:Mode1+2,4:Mode3,5:Mode1+3,6:Mode2+3,7:Mode1+2+3,8:Mode4,9:Mode1+4,10:Mode2+4,11:Mode1+2+4,12:Mode3+4,13:Mode1+3+4,14:Mode2+3+4,15:Mode1+2+3+4,16:Mode5,17:Mode1+5,18:Mode2+5,19:Mode1+2+5,20:Mode3+5,21:Mode1+3+5,22:Mode2+3+5,23:Mode1+2+3+5,24:Mode4+5,25:Mode1+4+5,26:Mode2+4+5,27:Mode1+2+4+5,28:Mode3+4+5,29:Mode1+3+4+5,30:Mode2+3+4+5,31:Mode1+2+3+4+5,32:Mode6,33:Mode1+6,34:Mode2+6,35:Mode1+2+6,36:Mode3+6,37:Mode1+3+6,38:Mode2+3+6,39:Mode1+2+3+6,40:Mode4+6,41:Mode1+4+6,42:Mode2+4+6,43:Mode1+2+4+6,44:Mode3+4+6,45:Mode1+3+4+6,46:Mode2+3+4+6,47:Mode1+2+3+4+6,48:Mode5+6,49:Mode1+5+6,50:Mode2+5+6,51:Mode1+2+5+6,52:Mode3+5+6,53:Mode1+3+5+6,54:Mode2+3+5+6,55:Mode1+2+3+5+6,56:Mode4+5+6,57:Mode1+4+5+6,58:Mode2+4+5+6,59:Mode1+2+4+5+6,60:Mode3+4+5+6,61:Mode1+3+4+5+6,62:Mode2+3+4+5+6,63:Mode1+2+3+4+5+6
    // @User: Standard
    GSCALAR(super_simple,   "SUPER_SIMPLE",     0),

    // @Param: RTL_ALT_FINAL
    // @DisplayName: RTL Final Altitude
    // @Description: This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission.  Set to zero to land.
    // @Units: Centimeters
    // @Range: -1 1000
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_alt_final,  "RTL_ALT_FINAL", RTL_ALT_FINAL),

    // @Param: RSSI_PIN
    // @DisplayName: Receiver RSSI sensing pin
    // @Description: This selects an analog pin for the receiver RSSI voltage. It assumes the voltage is RSSI_RANGE for max rssi, 0V for minimum
    // @Values: -1:Disabled, 0:APM2 A0, 1:APM2 A1, 2:APM2 A2, 13:APM2 A13, 103:Pixhawk SBUS
    // @User: Standard
    GSCALAR(rssi_pin,            "RSSI_PIN",         -1),

    // @Param: RSSI_RANGE
    // @DisplayName: Receiver RSSI voltage range
    // @Description: Receiver RSSI voltage range
    // @Units: Volt
    // @Values: 3.3:3.3V, 5:5V
    // @User: Standard
    GSCALAR(rssi_range,          "RSSI_RANGE",         5.0f),

    // @Param: WP_YAW_BEHAVIOR
    // @DisplayName: Yaw behaviour during missions
    // @Description: Determines how the autopilot controls the yaw during missions and RTL
    // @Values: 0:Never change yaw, 1:Face next waypoint, 2:Face next waypoint except RTL, 3:Face along GPS course
    // @User: Standard
    GSCALAR(wp_yaw_behavior,  "WP_YAW_BEHAVIOR",    WP_YAW_BEHAVIOR_DEFAULT),

    // @Param: RTL_LOIT_TIME
    // @DisplayName: RTL loiter time
    // @Description: Time (in milliseconds) to loiter above home before begining final descent
    // @Units: ms
    // @Range: 0 60000
    // @Increment: 1000
    // @User: Standard
    GSCALAR(rtl_loiter_time,      "RTL_LOIT_TIME",    RTL_LOITER_TIME),

    // @Param: LAND_SPEED
    // @DisplayName: Land speed
    // @Description: The descent speed for the final stage of landing in cm/s
    // @Units: cm/s
    // @Range: 30 200
    // @Increment: 10
    // @User: Standard
    GSCALAR(land_speed,             "LAND_SPEED",   LAND_SPEED),

    // @Param: PILOT_VELZ_MAX
    // @DisplayName: Pilot maximum vertical speed
    // @Description: The maximum vertical velocity the pilot may request in cm/s
    // @Units: Centimeters/Second
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_velocity_z_max,     "PILOT_VELZ_MAX",   PILOT_VELZ_MAX),

    // @Param: PILOT_ACCEL_Z
    // @DisplayName: Pilot vertical acceleration
    // @Description: The vertical acceleration used when pilot is controlling the altitude
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_accel_z,  "PILOT_ACCEL_Z",    PILOT_ACCEL_Z_DEFAULT),

    // @Param: THR_MIN
    // @DisplayName: Throttle Minimum
    // @Description: The minimum throttle that will be sent to the motors to keep them spinning
    // @Units: Percent*10
    // @Range: 0 300
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_min,   "THR_MIN",          THR_MIN_DEFAULT),

    // @Param: FS_THR_ENABLE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
    // @Values: 0:Disabled,1:Enabled always RTL,2:Enabled Continue with Mission in Auto Mode,3:Enabled always LAND
    // @User: Standard
    GSCALAR(failsafe_throttle,  "FS_THR_ENABLE",   FS_THR_DISABLED),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on channel 3 below which throttle sailsafe triggers
    // @Range: 925 1100
    // @Units: pwm
    // @Increment: 1
    // @User: Standard
    GSCALAR(failsafe_throttle_value, "FS_THR_VALUE",      FS_THR_VALUE_DEFAULT),

    // @Param: THR_MID
    // @DisplayName: Throttle Mid Position
    // @Description: The throttle output (0 ~ 1000) when throttle stick is in mid position.  Used to scale the manual throttle so that the mid throttle stick position is close to the throttle required to hover
    // @User: Standard
    // @Range: 300 700
    // @Units: Percent*10
    // @Increment: 1
    GSCALAR(throttle_mid,        "THR_MID",    THR_MID_DEFAULT),

    // @Param: THR_DZ
    // @DisplayName: Throttle deadzone
    // @Description: The deadzone above and below mid throttle.  Used in AltHold, Loiter, PosHold flight modes
    // @User: Standard
    // @Range: 0 300
    // @Units: pwm
    // @Increment: 1
    GSCALAR(throttle_deadzone,  "THR_DZ",    THR_DZ_DEFAULT),

    // @Param: FLTMODE1
    // @DisplayName: Flight Mode 1
    // @Description: Flight mode when Channel 5 pwm is <= 1230
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,16:PosHold
    // @User: Standard
    GSCALAR(flight_mode1, "FLTMODE1",               FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @DisplayName: Flight Mode 2
    // @Description: Flight mode when Channel 5 pwm is >1230, <= 1360
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,16:PosHold
    // @User: Standard
    GSCALAR(flight_mode2, "FLTMODE2",               FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @DisplayName: Flight Mode 3
    // @Description: Flight mode when Channel 5 pwm is >1360, <= 1490
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,16:PosHold
    // @User: Standard
    GSCALAR(flight_mode3, "FLTMODE3",               FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @DisplayName: Flight Mode 4
    // @Description: Flight mode when Channel 5 pwm is >1490, <= 1620
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,16:PosHold
    // @User: Standard
    GSCALAR(flight_mode4, "FLTMODE4",               FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @DisplayName: Flight Mode 5
    // @Description: Flight mode when Channel 5 pwm is >1620, <= 1749
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,16:PosHold
    // @User: Standard
    GSCALAR(flight_mode5, "FLTMODE5",               FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @DisplayName: Flight Mode 6
    // @Description: Flight mode when Channel 5 pwm is >=1750
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,16:PosHold
    // @User: Standard
    GSCALAR(flight_mode6, "FLTMODE6",               FLIGHT_MODE_6),

    // @Param: SIMPLE
    // @DisplayName: Simple mode bitmask
    // @Description: Bitmask which holds which flight modes use simple heading mode (eg bit 0 = 1 means Flight Mode 0 uses simple mode)
    // @User: Advanced
    GSCALAR(simple_modes, "SIMPLE",                 0),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: 4 byte bitmap of log types to enable
    // @Values: 830:Default,894:Default+RCIN,958:Default+IMU,1854:Default+Motors,-6146:NearlyAll-AC315,45054:NearlyAll,131070:All+DisarmedLogging,131071:All+FastATT,262142:All+MotBatt,393214:All+FastIMU,0:Disabled
    // @User: Standard
    GSCALAR(log_bitmask,    "LOG_BITMASK",          DEFAULT_LOG_BITMASK),

    // @Param: ESC
    // @DisplayName: ESC Calibration
    // @Description: Controls whether ArduCopter will enter ESC calibration on the next restart.  Do not adjust this parameter manually.
    // @User: Advanced
    // @Values: 0:Normal Start-up, 1:Start-up in ESC Calibration mode if throttle high, 2:Start-up in ESC Calibration mode regardless of throttle
    GSCALAR(esc_calibrate, "ESC",                   0),

    // @Param: TUNE
    // @DisplayName: Channel 6 Tuning
    // @Description: Controls which parameters (normally PID gains) are being tuned with transmitter's channel 6 knob
    // @User: Standard
    // @Values: 0:None,1:Stab Roll/Pitch kP,4:Rate Roll/Pitch kP,5:Rate Roll/Pitch kI,21:Rate Roll/Pitch kD,3:Stab Yaw kP,6:Rate Yaw kP,26:Rate Yaw kD,14:Altitude Hold kP,7:Throttle Rate kP,34:Throttle Accel kP,35:Throttle Accel kI,36:Throttle Accel kD,42:Loiter Speed,12:Loiter Pos kP,22:Velocity XY kP,28:Velocity XY kI,10:WP Speed,25:Acro RollPitch kP,40:Acro Yaw kP,13:Heli Ext Gyro,17:OF Loiter kP,18:OF Loiter kI,19:OF Loiter kD,30:AHRS Yaw kP,31:AHRS kP,38:Declination,39:Circle Rate,41:RangeFinder Gain,46:Rate Pitch kP,47:Rate Pitch kI,48:Rate Pitch kD,49:Rate Roll kP,50:Rate Roll kI,51:Rate Roll kD,52:Rate Pitch FF,53:Rate Roll FF,54:Rate Yaw FF
    GSCALAR(radio_tuning, "TUNE",                   0),

    // @Param: TUNE_LOW
    // @DisplayName: Tuning minimum
    // @Description: The minimum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
    // @User: Standard
    // @Range: 0 32767
    GSCALAR(radio_tuning_low, "TUNE_LOW",           0),

    // @Param: TUNE_HIGH
    // @DisplayName: Tuning maximum
    // @Description: The maximum value that will be applied to the parameter currently being tuned with the transmitter's channel 6 knob
    // @User: Standard
    // @Range: 0 32767
    GSCALAR(radio_tuning_high, "TUNE_HIGH",         1000),

    // @Param: FRAME
    // @DisplayName: Frame Orientation (+, X or V)
    // @Description: Controls motor mixing for multicopters.  Not used for Tri or Traditional Helicopters.
    // @Values: 0:Plus, 1:X, 2:V, 3:H, 4:V-Tail, 5:A-Tail, 10:Y6B (New)
    // @User: Standard
    GSCALAR(frame_orientation, "FRAME",             AP_MOTORS_X_FRAME),

    // @Param: CH7_OPT
    // @DisplayName: Channel 7 option
    // @Description: Select which function if performed when CH7 is above 1800 pwm
    // @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 8:Multi Mode, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 12:ResetToArmedYaw, 13:Super Simple Mode, 14:Acro Trainer, 16:Auto, 17:AutoTune, 18:Land, 19:EPM, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 29:Landing Gear, 30:Lost Copter Sound
    // @User: Standard
    GSCALAR(ch7_option, "CH7_OPT",                  AUXSW_DO_NOTHING),

    // @Param: CH8_OPT
    // @DisplayName: Channel 8 option
    // @Description: Select which function if performed when CH8 is above 1800 pwm
    // @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 8:Multi Mode, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 12:ResetToArmedYaw, 13:Super Simple Mode, 14:Acro Trainer, 16:Auto, 17:AutoTune, 18:Land, 19:EPM, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 29:Landing Gear, 30:Lost Copter Sound
    // @User: Standard
    GSCALAR(ch8_option, "CH8_OPT",                  AUXSW_DO_NOTHING),

    // @Param: CH9_OPT
    // @DisplayName: Channel 9 option
    // @Description: Select which function if performed when CH9 is above 1800 pwm
    // @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 8:Multi Mode, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 12:ResetToArmedYaw, 13:Super Simple Mode, 14:Acro Trainer, 16:Auto, 17:AutoTune, 18:Land, 19:EPM, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 29:Landing Gear, 30:Lost Copter Sound
    // @User: Standard
    GSCALAR(ch9_option, "CH9_OPT",                  AUXSW_DO_NOTHING),

    // @Param: CH10_OPT
    // @DisplayName: Channel 10 option
    // @Description: Select which function if performed when CH10 is above 1800 pwm
    // @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 8:Multi Mode, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 12:ResetToArmedYaw, 13:Super Simple Mode, 14:Acro Trainer, 16:Auto, 17:AutoTune, 18:Land, 19:EPM, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 29:Landing Gear, 30:Lost Copter Sound
    // @User: Standard
    GSCALAR(ch10_option, "CH10_OPT",                AUXSW_DO_NOTHING),

    // @Param: CH11_OPT
    // @DisplayName: Channel 11 option
    // @Description: Select which function if performed when CH11 is above 1800 pwm
    // @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 8:Multi Mode, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 12:ResetToArmedYaw, 13:Super Simple Mode, 14:Acro Trainer, 16:Auto, 17:AutoTune, 18:Land, 19:EPM, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 29:Landing Gear, 30:Lost Copter Sound
    // @User: Standard
    GSCALAR(ch11_option, "CH11_OPT",                AUXSW_DO_NOTHING),

    // @Param: CH12_OPT
    // @DisplayName: Channel 12 option
    // @Description: Select which function if performed when CH12 is above 1800 pwm
    // @Values: 0:Do Nothing, 2:Flip, 3:Simple Mode, 4:RTL, 5:Save Trim, 7:Save WP, 8:Multi Mode, 9:Camera Trigger, 10:RangeFinder, 11:Fence, 12:ResetToArmedYaw, 13:Super Simple Mode, 14:Acro Trainer, 16:Auto, 17:AutoTune, 18:Land, 19:EPM, 21:Parachute Enable, 22:Parachute Release, 23:Parachute 3pos, 24:Auto Mission Reset, 25:AttCon Feed Forward, 26:AttCon Accel Limits, 27:Retract Mount, 28:Relay On/Off, 29:Landing Gear, 30:Lost Copter Sound
    // @User: Standard
    GSCALAR(ch12_option, "CH12_OPT",                AUXSW_DO_NOTHING),

    // @Param: ARMING_CHECK
    // @DisplayName: Arming check
    // @Description: Allows enabling or disabling of pre-arming checks of receiver, accelerometer, barometer, compass and GPS
    // @Values: 0:Disabled, 1:Enabled, -3:Skip Baro, -5:Skip Compass, -9:Skip GPS, -17:Skip INS, -33:Skip Params/Sonar, -65:Skip RC, 127:Skip Voltage
    // @User: Standard
    GSCALAR(arming_check, "ARMING_CHECK",           ARMING_CHECK_ALL),

    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all flight modes
    // @Units: Centi-degrees
    // @Range 1000 8000
    // @User: Advanced
    ASCALAR(angle_max, "ANGLE_MAX",                 DEFAULT_ANGLE_MAX),

    // @Param: RC_FEEL_RP
    // @DisplayName: RC Feel Roll/Pitch
    // @Description: RC feel for roll/pitch which controls vehicle response to user input with 0 being extremely soft and 100 being crisp
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    // @Values: 0:Very Soft, 25:Soft, 50:Medium, 75:Crisp, 100:Very Crisp
    GSCALAR(rc_feel_rp, "RC_FEEL_RP",  RC_FEEL_RP_VERY_CRISP),

#if POSHOLD_ENABLED == ENABLED
    // @Param: PHLD_BRAKE_RATE
    // @DisplayName: PosHold braking rate
    // @Description: PosHold flight mode's rotation rate during braking in deg/sec
    // @Units: deg/sec
    // @Range: 4 12
    // @User: Advanced
    GSCALAR(poshold_brake_rate, "PHLD_BRAKE_RATE",  POSHOLD_BRAKE_RATE_DEFAULT),

    // @Param: PHLD_BRAKE_ANGLE
    // @DisplayName: PosHold braking angle max
    // @Description: PosHold flight mode's max lean angle during braking in centi-degrees
    // @Units: Centi-degrees
    // @Range: 2000 4500
    // @User: Advanced
    GSCALAR(poshold_brake_angle_max, "PHLD_BRAKE_ANGLE",  POSHOLD_BRAKE_ANGLE_DEFAULT),
#endif

    // @Param: LAND_REPOSITION
    // @DisplayName: Land repositioning
    // @Description: Enables user input during LAND mode, the landing phase of RTL, and auto mode landings.
    // @Values: 0:No repositioning, 1:Repositioning
    // @User: Advanced
    GSCALAR(land_repositioning, "LAND_REPOSITION",     LAND_REPOSITION_DEFAULT),

    // @Param: EKF_CHECK_THRESH
    // @DisplayName: EKF check compass and velocity variance threshold
    // @Description: Allows setting the maximum acceptable compass and velocity variance (0 to disable check)
    // @Values: 0:Disabled, 0.6:Strict, 0.8:Default, 1.0:Relaxed
    // @User: Advanced
    GSCALAR(ekfcheck_thresh, "EKF_CHECK_THRESH",    EKFCHECK_THRESHOLD_DEFAULT),

#if FRAME_CONFIG ==     HELI_FRAME
    // @Group: HS1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(heli_servo_1,    "HS1_", RC_Channel),
    // @Group: HS2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(heli_servo_2,    "HS2_", RC_Channel),
    // @Group: HS3_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(heli_servo_3,    "HS3_", RC_Channel),
    // @Group: HS4_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(heli_servo_4,    "HS4_", RC_Channel),

    // @Param: H_STAB_COL_MIN
    // @DisplayName: Heli Stabilize Throttle Collective Minimum
    // @Description: Helicopter's minimum collective position while pilot directly controls collective in stabilize mode
    // @Range: 0 500
    // @Units: Percent*10
    // @Increment: 1
    // @User: Standard
    GSCALAR(heli_stab_col_min, "H_STAB_COL_MIN", HELI_STAB_COLLECTIVE_MIN_DEFAULT),

    // @Param: H_STAB_COL_MAX
    // @DisplayName: Stabilize Throttle Maximum
    // @Description: Helicopter's maximum collective position while pilot directly controls collective in stabilize mode
    // @Range: 500 1000
    // @Units: Percent*10
    // @Increment: 1
    // @User: Standard
    GSCALAR(heli_stab_col_max, "H_STAB_COL_MAX", HELI_STAB_COLLECTIVE_MAX_DEFAULT),
#endif

    // RC channel
    //-----------
    // @Group: RC1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_1,    "RC1_", RC_Channel),
    // @Group: RC2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_2,    "RC2_", RC_Channel),
    // @Group: RC3_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_3,    "RC3_", RC_Channel),
    // @Group: RC4_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(rc_4,    "RC4_", RC_Channel),
    // @Group: RC5_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_5,    "RC5_", RC_Channel_aux),
    // @Group: RC6_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_6,    "RC6_", RC_Channel_aux),
    // @Group: RC7_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_7,    "RC7_", RC_Channel_aux),
    // @Group: RC8_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_8,    "RC8_", RC_Channel_aux),

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // @Group: RC9_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_9,                    "RC9_", RC_Channel_aux),
#endif

    // @Group: RC10_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_10,                    "RC10_", RC_Channel_aux),
    // @Group: RC11_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_11,                    "RC11_", RC_Channel_aux),

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // @Group: RC12_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_12,                   "RC12_", RC_Channel_aux),

    // @Group: RC13_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_13,                   "RC13_", RC_Channel_aux),

    // @Group: RC14_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp,../libraries/RC_Channel/RC_Channel_aux.cpp
    GGROUP(rc_14,                   "RC14_", RC_Channel_aux),
#endif

    // @Param: RC_SPEED
    // @DisplayName: ESC Update Speed
    // @Description: This is the speed in Hertz that your ESCs will receive updates
    // @Units: Hz
    // @Range: 50 490
    // @Increment: 1
    // @User: Advanced
    GSCALAR(rc_speed, "RC_SPEED",              RC_FAST_SPEED),

    // @Param: ACRO_RP_P
    // @DisplayName: Acro Roll and Pitch P gain
    // @Description: Converts pilot roll and pitch into a desired rate of rotation in ACRO and SPORT mode.  Higher values mean faster rate of rotation.
    // @Range: 1 10
    // @User: Standard
    GSCALAR(acro_rp_p,                 "ACRO_RP_P",           ACRO_RP_P),

    // @Param: ACRO_YAW_P
    // @DisplayName: Acro Yaw P gain
    // @Description: Converts pilot yaw input into a desired rate of rotation in ACRO, Stabilize and SPORT modes.  Higher values mean faster rate of rotation.
    // @Range: 1 10
    // @User: Standard
    GSCALAR(acro_yaw_p,                 "ACRO_YAW_P",           ACRO_YAW_P),

    // @Param: ACRO_BAL_ROLL
    // @DisplayName: Acro Balance Roll
    // @Description: rate at which roll angle returns to level in acro mode.  A higher value causes the vehicle to return to level faster.
    // @Range: 0 3
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(acro_balance_roll,      "ACRO_BAL_ROLL",    ACRO_BALANCE_ROLL),

    // @Param: ACRO_BAL_PITCH
    // @DisplayName: Acro Balance Pitch
    // @Description: rate at which pitch angle returns to level in acro mode.  A higher value causes the vehicle to return to level faster.
    // @Range: 0 3
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(acro_balance_pitch,     "ACRO_BAL_PITCH",   ACRO_BALANCE_PITCH),

    // @Param: ACRO_TRAINER
    // @DisplayName: Acro Trainer
    // @Description: Type of trainer used in acro mode
    // @Values: 0:Disabled,1:Leveling,2:Leveling and Limited
    // @User: Advanced
    GSCALAR(acro_trainer,   "ACRO_TRAINER",     ACRO_TRAINER_LIMITED),

    // @Param: ACRO_EXPO
    // @DisplayName: Acro Expo
    // @Description: Acro roll/pitch Expo to allow faster rotation when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @User: Advanced
    GSCALAR(acro_expo,  "ACRO_EXPO",    ACRO_EXPO_DEFAULT),

    // PID controller
    //---------------

    // @Param: RATE_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
    // @Range: 0.08 0.30
    // @Increment: 0.005
    // @User: Standard

    // @Param: RATE_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Standard

    // @Param: RATE_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: Percent*10
    // @User: Standard

    // @Param: RATE_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.001 0.02
    // @Increment: 0.001
    // @User: Standard
#if FRAME_CONFIG == HELI_FRAME
    GGROUP(pid_rate_roll,     "RATE_RLL_", AC_HELI_PID),
#else
    GGROUP(pid_rate_roll,     "RATE_RLL_", AC_PID),
#endif

    // @Param: RATE_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
    // @Range: 0.08 0.30
    // @Increment: 0.005
    // @User: Standard

    // @Param: RATE_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 0.5
    // @Increment: 0.01
    // @User: Standard

    // @Param: RATE_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: Percent*10
    // @User: Standard

    // @Param: RATE_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.001 0.02
    // @Increment: 0.001
    // @User: Standard
#if FRAME_CONFIG == HELI_FRAME
    GGROUP(pid_rate_pitch,    "RATE_PIT_", AC_HELI_PID),
#else
    GGROUP(pid_rate_pitch,    "RATE_PIT_", AC_PID),
#endif

    // @Param: RATE_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
    // @Range: 0.150 0.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RATE_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.010 0.05
    // @Increment: 0.01
    // @User: Standard

    // @Param: RATE_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: Percent*10
    // @User: Standard

    // @Param: RATE_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.02
    // @Increment: 0.001
    // @User: Standard
#if FRAME_CONFIG == HELI_FRAME
    GGROUP(pid_rate_yaw,      "RATE_YAW_", AC_HELI_PID),
#else
    GGROUP(pid_rate_yaw,      "RATE_YAW_", AC_PID),
#endif

    // @Param: VEL_XY_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain.  Converts the difference between desired velocity to a target acceleration
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: VEL_XY_I
    // @DisplayName: Velocity (horizontal) I gain
    // @Description: Velocity (horizontal) I gain.  Corrects long-term difference in desired velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: VEL_XY_IMAX
    // @DisplayName: Velocity (horizontal) integrator maximum
    // @Description: Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced
    GGROUP(pi_vel_xy,   "VEL_XY_",  AC_PI_2D),

    // @Param: VEL_Z_P
    // @DisplayName: Velocity (vertical) P gain
    // @Description: Velocity (vertical) P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard
    GGROUP(p_vel_z,     "VEL_Z_", AC_P),

    // @Param: ACCEL_Z_P
    // @DisplayName: Throttle acceleration controller P gain
    // @Description: Throttle acceleration controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.500 1.500
    // @User: Standard

    // @Param: ACCEL_Z_I
    // @DisplayName: Throttle acceleration controller I gain
    // @Description: Throttle acceleration controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: ACCEL_Z_IMAX
    // @DisplayName: Throttle acceleration controller I gain maximum
    // @Description: Throttle acceleration controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 1000
    // @Units: Percent*10
    // @User: Standard

    // @Param: ACCEL_Z_D
    // @DisplayName: Throttle acceleration controller D gain
    // @Description: Throttle acceleration controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard
    GGROUP(pid_accel_z, "ACCEL_Z_", AC_PID),

    // P controllers
    //--------------
    // @Param: STB_RLL_P
    // @DisplayName: Roll axis stabilize controller P gain
    // @Description: Roll axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
    // @Range: 3.000 12.000
    // @User: Standard
    GGROUP(p_stabilize_roll,       "STB_RLL_", AC_P),

    // @Param: STB_PIT_P
    // @DisplayName: Pitch axis stabilize controller P gain
    // @Description: Pitch axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
    // @Range: 3.000 12.000
    // @User: Standard
    GGROUP(p_stabilize_pitch,      "STB_PIT_", AC_P),

    // @Param: STB_YAW_P
    // @DisplayName: Yaw axis stabilize controller P gain
    // @Description: Yaw axis stabilize (i.e. angle) controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
    // @Range: 3.000 6.000
    // @User: Standard
    GGROUP(p_stabilize_yaw,        "STB_YAW_", AC_P),

    // @Param: POS_Z_P
    // @DisplayName: Position (vertical) controller P gain
    // @Description: Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard
    GGROUP(p_alt_hold,              "POS_Z_", AC_P),

    // @Param: POS_XY_P
    // @DisplayName: Position (horizonal) controller P gain
    // @Description: Loiter position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
    // @Range: 0.500 2.000
    // @User: Standard
    GGROUP(p_pos_xy,                "POS_XY_", AC_P),

    // variables not in the g class which contain EEPROM saved variables

#if CAMERA == ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera,           "CAM_", AP_Camera),
#endif

    // @Group: RELAY_
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay,                  "RELAY_", AP_Relay),

#if EPM_ENABLED == ENABLED
	// @Group: EPM_
    // @Path: ../libraries/AP_EPM/AP_EPM.cpp
    GOBJECT(epm,            "EPM_", AP_EPM),
#endif

#if PARACHUTE == ENABLED
	// @Group: CHUTE_
    // @Path: ../libraries/AP_Parachute/AP_Parachute.cpp
    GOBJECT(parachute,		"CHUTE_", AP_Parachute),
#endif

    // @Group: LGR_
    // @Path: ../libraries/AP_LandingGear/AP_LandingGear.cpp
    GOBJECT(landinggear,    "LGR_", AP_LandingGear),

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/Compass.cpp
    GOBJECT(compass,        "COMPASS_", Compass),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,            "INS_", AP_InertialSensor),

    // @Group: WPNAV_
    // @Path: ../libraries/AC_WPNav/AC_WPNav.cpp
    GOBJECT(wp_nav, "WPNAV_",       AC_WPNav),

    // @Group: CIRCLE_
    // @Path: ../libraries/AC_WPNav/AC_Circle.cpp
    GOBJECT(circle_nav, "CIRCLE_",  AC_Circle),

#if FRAME_CONFIG == HELI_FRAME
    // @Group: ATC_
    // @Path: ../libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.cpp
    GOBJECT(attitude_control, "ATC_", AC_AttitudeControl_Heli),
#else
    // @Group: ATC_
    // @Path: ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp
    GOBJECT(attitude_control, "ATC_", AC_AttitudeControl),
#endif

    // @Group: POSCON_
    // @Path: ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    GOBJECT(pos_control, "PSC", AC_PosControl),

    // @Group: SR0_
    // @Path: GCS_Mavlink.pde
    GOBJECTN(gcs[0],  gcs0,       "SR0_",     GCS_MAVLINK),

    // @Group: SR1_
    // @Path: GCS_Mavlink.pde
    GOBJECTN(gcs[1],  gcs1,       "SR1_",     GCS_MAVLINK),

#if MAVLINK_COMM_NUM_BUFFERS > 2
    // @Group: SR2_
    // @Path: GCS_Mavlink.pde
    GOBJECTN(gcs[2],  gcs2,       "SR2_",     GCS_MAVLINK),
#endif

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

#if MOUNT == ENABLED
    // @Group: MNT_
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT",  AP_Mount),
#endif

    // @Group: BATT_
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery,                "BATT",         AP_BattMonitor),

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),

#if SPRAYER == ENABLED
    // @Group: SPRAY_
    // @Path: ../libraries/AC_Sprayer/AC_Sprayer.cpp
    GOBJECT(sprayer,                "SPRAY_",       AC_Sprayer),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    GOBJECT(sitl, "SIM_", SITL),
#endif

    // @Group: GND_
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "GND_", AP_Baro),

    // GPS driver
    // @Group: GPS_
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS_", AP_GPS),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

#if AC_FENCE == ENABLED
    // @Group: FENCE_
    // @Path: ../libraries/AC_Fence/AC_Fence.cpp
    GOBJECT(fence,      "FENCE_",   AC_Fence),
#endif

#if AC_RALLY == ENABLED
    // @Group: RALLY_
    // @Path: ../libraries/AP_Rally/AP_Rally.cpp
    GOBJECT(rally,      "RALLY_",   AP_Rally),
#endif

#if FRAME_CONFIG ==     HELI_FRAME
    // @Group: H_
    // @Path: ../libraries/AP_Motors/AP_MotorsHeli.cpp
    GOBJECT(motors, "H_",           AP_MotorsHeli),

#elif FRAME_CONFIG == SINGLE_FRAME
    // @Group: SS1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(single_servo_1,    "SS1_", RC_Channel),
    // @Group: SS2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(single_servo_2,    "SS2_", RC_Channel),
    // @Group: SS3_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(single_servo_3,    "SS3_", RC_Channel),
    // @Group: SS4_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(single_servo_4,    "SS4_", RC_Channel),
    // @Group: MOT_
    // @Path: ../libraries/AP_Motors/AP_MotorsSingle.cpp
    GOBJECT(motors, "MOT_",           AP_MotorsSingle),

#elif FRAME_CONFIG == COAX_FRAME
    // @Group: SS1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(single_servo_1,    "SS1_", RC_Channel),
    // @Group: SS2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GGROUP(single_servo_2,    "SS2_", RC_Channel),
    // @Group: MOT_
    // @Path: ../libraries/AP_Motors/AP_MotorsCoax.cpp
    GOBJECT(motors, "MOT_",           AP_MotorsCoax),

#else
    // @Group: MOT_
    // @Path: ../libraries/AP_Motors/AP_Motors_Class.cpp
    GOBJECT(motors, "MOT_",         AP_Motors),
#endif

    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap, "RCMAP_",        RCMapper),

#if AP_AHRS_NAVEKF_AVAILABLE
    // @Group: EKF_
    // @Path: ../libraries/AP_NavEKF/AP_NavEKF.cpp
    GOBJECTN(ahrs.get_NavEKF(), NavEKF, "EKF_", NavEKF),
#endif

    // @Group: MIS_
    // @Path: ../libraries/AP_Mission/AP_Mission.cpp
    GOBJECT(mission, "MIS_",       AP_Mission),

#if CONFIG_SONAR == ENABLED
    // @Group: RNGFND
    // @Path: ../libraries/AP_RangeFinder/RangeFinder.cpp
    GOBJECT(sonar,   "RNGFND", RangeFinder),
#endif

#if AP_TERRAIN_AVAILABLE
    // @Group: TERRAIN_
    // @Path: ../libraries/AP_Terrain/AP_Terrain.cpp
    GOBJECT(terrain,                "TERRAIN_", AP_Terrain),
#endif

#if OPTFLOW == ENABLED
    // @Group: FLOW
    // @Path: ../libraries/AP_OpticalFlow/OpticalFlow.cpp
    GOBJECT(optflow,   "FLOW", OpticalFlow),
#endif

    // @Param: AUTOTUNE_AXIS_BITMASK
    // @DisplayName: Autotune axis bitmask
    // @Description: 1-byte bitmap of axes to autotune
    // @Values: 7:All,1:Roll Only,2:Pitch Only,4:Yaw Only,3:Roll and Pitch,5:Roll and Yaw,6:Pitch and Yaw
    // @User: Standard
    GSCALAR(autotune_axis_bitmask, "AUTOTUNE_AXES", 7),  // AUTOTUNE_AXIS_BITMASK_DEFAULT

    // @Param: AUTOTUNE_AGGRESSIVENESS
    // @DisplayName: autotune_aggressiveness
    // @Description: autotune_aggressiveness. Defines the bounce back used to detect size of the D term.
    // @Range: 0.05 0.10
    // @User: Standard
    GSCALAR(autotune_aggressiveness, "AUTOTUNE_AGGR", 0.1f),

    AP_VAREND
};

/*
  This is a conversion table from old parameter values to new
  parameter names. The startup code looks for saved values of the old
  parameters and will copy them across to the new parameters if the
  new parameter does not yet have a saved value. It then saves the new
  value.

  Note that this works even if the old parameter has been removed. It
  relies on the old k_param index not being removed

  The second column below is the index in the var_info[] table for the
  old object. This should be zero for top level parameters.
 */
const AP_Param::ConversionInfo conversion_table[] PROGMEM = {
    { Parameters::k_param_battery_monitoring, 0,      AP_PARAM_INT8,  "BATT_MONITOR" },
    { Parameters::k_param_battery_volt_pin,   0,      AP_PARAM_INT8,  "BATT_VOLT_PIN" },
    { Parameters::k_param_battery_curr_pin,   0,      AP_PARAM_INT8,  "BATT_CURR_PIN" },
    { Parameters::k_param_volt_div_ratio,     0,      AP_PARAM_FLOAT, "BATT_VOLT_MULT" },
    { Parameters::k_param_curr_amp_per_volt,  0,      AP_PARAM_FLOAT, "BATT_AMP_PERVOLT" },
    { Parameters::k_param_pack_capacity,      0,      AP_PARAM_INT32, "BATT_CAPACITY" },
    { Parameters::k_param_log_bitmask_old,    0,      AP_PARAM_INT16, "LOG_BITMASK" },
    { Parameters::k_param_serial0_baud,       0,      AP_PARAM_INT16, "SERIAL0_BAUD" },
    { Parameters::k_param_serial1_baud,       0,      AP_PARAM_INT16, "SERIAL1_BAUD" },
    { Parameters::k_param_serial2_baud,       0,      AP_PARAM_INT16, "SERIAL2_BAUD" },
};

static void load_parameters(void)
{
    if (!AP_Param::check_var_info()) {
        cliSerial->printf_P(PSTR("Bad var table\n"));
        hal.scheduler->panic(PSTR("Bad var table"));
    }

    // change the default for the AHRS_GPS_GAIN for ArduCopter
    // if it hasn't been set by the user
    if (!ahrs.gps_gain.load()) {
        ahrs.gps_gain.set_and_save(1.0);
    }
    // disable centrifugal force correction, it will be enabled as part of the arming process
    ahrs.set_correct_centrifugal(false);
    hal.util->set_soft_armed(false);

    // setup different AHRS gains for ArduCopter than the default
    // but allow users to override in their config
    if (!ahrs._kp.load()) {
        ahrs._kp.set_and_save(0.1);
    }
    if (!ahrs._kp_yaw.load()) {
        ahrs._kp_yaw.set_and_save(0.1);
    }

    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        cliSerial->printf_P(PSTR("Firmware change: erasing EEPROM...\n"));
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        cliSerial->println_P(PSTR("done."));
    } else {
        uint32_t before = micros();
        // Load all auto-loaded EEPROM variables
        AP_Param::load_all();
        AP_Param::convert_old_parameters(&conversion_table[0], sizeof(conversion_table)/sizeof(conversion_table[0]));
        cliSerial->printf_P(PSTR("load_all took %luus\n"), micros() - before);
    }
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/UserCode.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    teensy.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
uint8_t sensorCounter = 0;
bool measuring = false;
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    sensorCounter++;

    int16_t interval = g.teensy_sen_f; // frequency
    int16_t timeToMeasure = g.teensy_sen_t; // time needed to perform measure
    
    hal.console->printf("Time: %d/%d.\n", sensorCounter, interval);
    
    //Start measuring so it is available when needed
    if(!measuring && interval - timeToMeasure <= sensorCounter){
        measuring = true;
        hal.console->println("Requesting measurement...");
        teensy.measure();
    }

    if(interval <= sensorCounter){
        
        int16_t value = 0;
        const Location &loc = gps.location();

        //hal.console->printf("Lat: %d, Lon: %d\n", loc.lat, loc.lng);
        uint64_t usec = gps.time_epoch_usec();
        //uint32_t fix = gps.last_fix_time_ms();
        
        if (teensy.read(&value)) {
            //Reset counting variables, etc.
            measuring = false;
            sensorCounter = 0;    
            
            mavlink_msg_int_sensor_send(MAVLINK_COMM_1, value, usec, loc.alt, loc.lng, loc.lat);
            hal.console->printf("Value: %d, interval: %d. ", value, interval);
            hal.console->println("Mavlink message (200) sent");
            
        } else {
            hal.console->printf("No or old sensor data...\n");
        }
    }
}
#endif
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/commands.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * the home_state has a number of possible values (see enum HomeState in defines.h's)
 *   HOME_UNSET             = home is not set, no GPS positions yet received
 *   HOME_SET_NOT_LOCKED    = home is set to EKF origin or armed location (can be moved)
 *   HOME_SET_AND_LOCKED    = home has been set by user, cannot be moved except by user initiated do-set-home command
 */

// checks if we should update ahrs/RTL home position from the EKF
static void update_home_from_EKF()
{
    // exit immediately if home already set
    if (ap.home_state != HOME_UNSET) {
        return;
    }

    // move home to current ekf location (this will set home_state to HOME_SET)
    set_home_to_current_location();
}

// set_home_to_current_location - set home to current GPS location
static bool set_home_to_current_location() {
    // get current location from EKF
    Location temp_loc;
    if (inertial_nav.get_location(temp_loc)) {
        return set_home(temp_loc);
    }
    return false;
}

// set_home_to_current_location_and_lock - set home to current location and lock so it cannot be moved
static bool set_home_to_current_location_and_lock()
{
    if (set_home_to_current_location()) {
        set_home_state(HOME_SET_AND_LOCKED);
        return true;
    }
    return false;
}

// set_home_and_lock - sets ahrs home (used for RTL) to specified location and locks so it cannot be moved
//  unless this function is called again
static bool set_home_and_lock(const Location& loc)
{
    if (set_home(loc)) {
        set_home_state(HOME_SET_AND_LOCKED);
        return true;
    }
    return false;
}

// set_home - sets ahrs home (used for RTL) to specified location
//  initialises inertial nav and compass on first call
//  returns true if home location set successfully
static bool set_home(const Location& loc)
{
    // check location is valid
    if (loc.lat == 0 && loc.lng == 0) {
        return false;
    }

    // set ahrs home (used for RTL)
    ahrs.set_home(loc);

    // init inav and compass declination
    if (ap.home_state == HOME_UNSET) {
        // Set compass declination automatically
        if (g.compass_enabled) {
            compass.set_initial_location(gps.location().lat, gps.location().lng);
        }
        // update navigation scalers.  used to offset the shrinking longitude as we go towards the poles
        scaleLongDown = longitude_scale(loc);
        scaleLongUp   = 1.0f/scaleLongDown;
        // record home is set
        set_home_state(HOME_SET_NOT_LOCKED);

        // log new home position which mission library will pull from ahrs
        if (should_log(MASK_LOG_CMD)) {
            AP_Mission::Mission_Command temp_cmd;
            if (mission.read_cmd_from_storage(0, temp_cmd)) {
                Log_Write_Cmd(temp_cmd);
            }
        }
    }

    // return success
    return true;
}

// far_from_EKF_origin - checks if a location is too far from the EKF origin
//  returns true if too far
static bool far_from_EKF_origin(const Location& loc)
{
    // check distance to EKF origin
    const struct Location &ekf_origin = inertial_nav.get_origin();
    if (get_distance(ekf_origin, loc) > EKF_ORIGIN_MAX_DIST_M) {
        return true;
    }

    // close enough to origin
    return false;
}

// checks if we should update ahrs/RTL home position from GPS
static void set_system_time_from_GPS()
{
    // exit immediately if system time already set
    if (ap.system_time_set) {
        return;
    }

    // if we have a 3d lock and valid location
    if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        // set system clock for log timestamps
        hal.util->set_system_clock(gps.time_epoch_usec());
        ap.system_time_set = true;
        Log_Write_Event(DATA_SYSTEM_TIME_SET);
    }
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/commands_logic.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// forward declarations to make compiler happy
static void do_takeoff(const AP_Mission::Mission_Command& cmd);
static void do_nav_wp(const AP_Mission::Mission_Command& cmd);
static void do_land(const AP_Mission::Mission_Command& cmd);
static void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
static void do_circle(const AP_Mission::Mission_Command& cmd);
static void do_loiter_time(const AP_Mission::Mission_Command& cmd);
static void do_spline_wp(const AP_Mission::Mission_Command& cmd);
#if NAV_GUIDED == ENABLED
static void do_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
static void do_guided_limits(const AP_Mission::Mission_Command& cmd);
#endif
static void do_wait_delay(const AP_Mission::Mission_Command& cmd);
static void do_within_distance(const AP_Mission::Mission_Command& cmd);
static void do_change_alt(const AP_Mission::Mission_Command& cmd);
static void do_yaw(const AP_Mission::Mission_Command& cmd);
static void do_change_speed(const AP_Mission::Mission_Command& cmd);
static void do_set_home(const AP_Mission::Mission_Command& cmd);
static void do_roi(const AP_Mission::Mission_Command& cmd);
static void do_mount_control(const AP_Mission::Mission_Command& cmd);
#if CAMERA == ENABLED
static void do_digicam_configure(const AP_Mission::Mission_Command& cmd);
static void do_digicam_control(const AP_Mission::Mission_Command& cmd);
#endif
#if PARACHUTE == ENABLED
static void do_parachute(const AP_Mission::Mission_Command& cmd);
#endif
#if EPM_ENABLED == ENABLED
static void do_gripper(const AP_Mission::Mission_Command& cmd);
#endif
static bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
static bool verify_circle(const AP_Mission::Mission_Command& cmd);
static bool verify_spline_wp(const AP_Mission::Mission_Command& cmd);
#if NAV_GUIDED == ENABLED
static bool verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
#endif
static void auto_spline_start(const Vector3f& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Vector3f& next_spline_destination);

// start_command - this function will be called when the ap_mission lib wishes to start a new command
static bool start_command(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: logging when new commands start/end
    if (should_log(MASK_LOG_CMD)) {
        Log_Write_Cmd(cmd);
    }

    switch(cmd.id) {

    ///
    /// navigation commands
    ///
    case MAV_CMD_NAV_TAKEOFF:                   // 22
        do_takeoff(cmd);
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // 16  Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:              // 21 LAND to Waypoint
        do_land(cmd);
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // 17 Loiter indefinitely
        do_loiter_unlimited(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              //18 Loiter N Times
        do_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:              // 19
        do_loiter_time(cmd);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:             //20
        do_RTL();
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:           // 82  Navigate to Waypoint using spline
        do_spline_wp(cmd);
        break;

#if NAV_GUIDED == ENABLED
    case MAV_CMD_NAV_GUIDED_ENABLE:             // 92  accept navigation commands from external nav computer
        do_nav_guided_enable(cmd);
        break;
#endif

    //
    // conditional commands
    //
    case MAV_CMD_CONDITION_DELAY:             // 112
        do_wait_delay(cmd);
        break;

    case MAV_CMD_CONDITION_DISTANCE:             // 114
        do_within_distance(cmd);
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:             // 113
        do_change_alt(cmd);
        break;

    case MAV_CMD_CONDITION_YAW:             // 115
        do_yaw(cmd);
        break;

    ///
    /// do commands
    ///
    case MAV_CMD_DO_CHANGE_SPEED:             // 178
        do_change_speed(cmd);
        break;

    case MAV_CMD_DO_SET_HOME:             // 179
        do_set_home(cmd);
        break;

    case MAV_CMD_DO_SET_SERVO:
        ServoRelayEvents.do_set_servo(cmd.content.servo.channel, cmd.content.servo.pwm);
        break;
        
    case MAV_CMD_DO_SET_RELAY:
        ServoRelayEvents.do_set_relay(cmd.content.relay.num, cmd.content.relay.state);
        break;
        
    case MAV_CMD_DO_REPEAT_SERVO:
        ServoRelayEvents.do_repeat_servo(cmd.content.repeat_servo.channel, cmd.content.repeat_servo.pwm,
                                         cmd.content.repeat_servo.repeat_count, cmd.content.repeat_servo.cycle_time * 1000.0f);
        break;
        
    case MAV_CMD_DO_REPEAT_RELAY:
        ServoRelayEvents.do_repeat_relay(cmd.content.repeat_relay.num, cmd.content.repeat_relay.repeat_count,
                                         cmd.content.repeat_relay.cycle_time * 1000.0f);
        break;

    case MAV_CMD_DO_SET_ROI:                // 201
        // point the copter and camera at a region of interest (ROI)
        do_roi(cmd);
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:          // 205
        // point the camera to a specified angle
        do_mount_control(cmd);
        break;

#if CAMERA == ENABLED
    case MAV_CMD_DO_CONTROL_VIDEO:                      // Control on-board camera capturing. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|
        do_digicam_configure(cmd);
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|
        do_digicam_control(cmd);
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        camera.set_trigger_distance(cmd.content.cam_trigg_dist.meters);
        break;
#endif

#if PARACHUTE == ENABLED
    case MAV_CMD_DO_PARACHUTE:                          // Mission command to configure or release parachute
        do_parachute(cmd);
        break;
#endif

#if EPM_ENABLED == ENABLED
    case MAV_CMD_DO_GRIPPER:                            // Mission command to control EPM gripper
        do_gripper(cmd);
        break;
#endif

#if NAV_GUIDED == ENABLED
    case MAV_CMD_DO_GUIDED_LIMITS:                      // 220  accept guided mode limits
        do_guided_limits(cmd);
        break;
#endif

    default:
        // do nothing with unrecognized MAVLink messages
        break;
    }

    // always return success
    return true;
}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

// verify_command - this will be called repeatedly by ap_mission lib to ensure the active commands are progressing
//  should return true once the active navigation command completes successfully
//  called at 10hz or higher
static bool verify_command(const AP_Mission::Mission_Command& cmd)
{
    switch(cmd.id) {

    //
    // navigation commands
    //
    case MAV_CMD_NAV_TAKEOFF:
        return verify_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:
        return verify_land();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:
        return verify_spline_wp(cmd);
        break;

#if NAV_GUIDED == ENABLED
    case MAV_CMD_NAV_GUIDED_ENABLE:
        return verify_nav_guided_enable(cmd);
        break;
#endif

    ///
    /// conditional commands
    ///
    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();
        break;

    case MAV_CMD_CONDITION_CHANGE_ALT:
        return verify_change_alt();
        break;

    case MAV_CMD_CONDITION_YAW:
        return verify_yaw();
        break;

#if PARACHUTE == ENABLED
    case MAV_CMD_DO_PARACHUTE:
        // assume parachute was released successfully
        return true;
        break;
#endif

    default:
        // return true if we do not recognise the command so that we move on to the next command
        return true;
        break;
    }
}

// exit_mission - function that is called once the mission completes
static void exit_mission()
{
    // play a tone
    AP_Notify::events.mission_complete = 1;
    // if we are not on the ground switch to loiter or land
    if(!ap.land_complete) {
        // try to enter loiter but if that fails land
        if(!auto_loiter_start()) {
            set_mode(LAND);
        }
    }else{
#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (ap.throttle_zero || failsafe.radio) {
            init_disarm_motors();
        }
#else
        // if we've landed it's safe to disarm
        init_disarm_motors();
#endif
    }
}

/********************************************************************************/
//
/********************************************************************************/

// do_RTL - start Return-to-Launch
static void do_RTL(void)
{
    // start rtl in auto flight mode
    auto_rtl_start();
}

/********************************************************************************/
//	Nav (Must) commands
/********************************************************************************/

// do_takeoff - initiate takeoff navigation command
static void do_takeoff(const AP_Mission::Mission_Command& cmd)
{
    // Set wp navigation target to safe altitude above current position
    float takeoff_alt = cmd.content.location.alt;
    takeoff_alt = max(takeoff_alt,current_loc.alt);
    takeoff_alt = max(takeoff_alt,100.0f);
    auto_takeoff_start(takeoff_alt);
}

// do_nav_wp - initiate move to next waypoint
static void do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    const Vector3f &curr_pos = inertial_nav.get_position();
    const Vector3f local_pos = pv_location_to_vector_with_default(cmd.content.location, curr_pos);

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = abs(cmd.p1);

    // Set wp navigation target
    auto_wp_start(local_pos);
    // if no delay set the waypoint as "fast"
    if (loiter_time_max == 0 ) {
        wp_nav.set_fast_waypoint(true);
    }
}

// do_land - initiate landing procedure
static void do_land(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: check if we have already landed

    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        // set state to fly to location
        land_state = LAND_STATE_FLY_TO_LOCATION;

        // calculate and set desired location above landing target
        Vector3f pos = pv_location_to_vector(cmd.content.location);
        pos.z = inertial_nav.get_altitude();
        auto_wp_start(pos);
    }else{
        // set landing state
        land_state = LAND_STATE_DESCENDING;

        // initialise landing controller
        auto_land_start();
    }
}

// do_loiter_unlimited - start loitering with no end conditions
// note: caller should set yaw_mode
static void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    Vector3f target_pos;

    // get current position
    Vector3f curr_pos = inertial_nav.get_position();

    // default to use position provided
    target_pos = pv_location_to_vector(cmd.content.location);

    // use current location if not provided
    if(cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
        wp_nav.get_wp_stopping_point_xy(target_pos);
    }

    // use current altitude if not provided
    // To-Do: use z-axis stopping point instead of current alt
    if( cmd.content.location.alt == 0 ) {
        target_pos.z = curr_pos.z;
    }

    // start way point navigator and provide it the desired location
    auto_wp_start(target_pos);
}

// do_circle - initiate moving in a circle
static void do_circle(const AP_Mission::Mission_Command& cmd)
{
    Vector3f curr_pos = inertial_nav.get_position();
    Vector3f circle_center = pv_location_to_vector(cmd.content.location);
    uint8_t circle_radius_m = HIGHBYTE(cmd.p1); // circle radius held in high byte of p1
    bool move_to_edge_required = false;

    // set target altitude if not provided
    if (cmd.content.location.alt == 0) {
        circle_center.z = curr_pos.z;
    } else {
        move_to_edge_required = true;
    }

    // set lat/lon position if not provided
    // To-Do: use previous command's destination if it was a straight line or spline waypoint command
    if (cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
        circle_center.x = curr_pos.x;
        circle_center.y = curr_pos.y;
    } else {
        move_to_edge_required = true;
    }

    // set circle controller's center
    circle_nav.set_center(circle_center);

    // set circle radius
    if (circle_radius_m != 0) {
        circle_nav.set_radius((float)circle_radius_m * 100.0f);
    }

    // check if we need to move to edge of circle
    if (move_to_edge_required) {
        // move to edge of circle (verify_circle) will ensure we begin circling once we reach the edge
        auto_circle_movetoedge_start();
    } else {
        // start circling
        auto_circle_start();
    }
}

// do_loiter_time - initiate loitering at a point for a given time period
// note: caller should set yaw_mode
static void do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    Vector3f target_pos;

    // get current position
    Vector3f curr_pos = inertial_nav.get_position();

    // default to use position provided
    target_pos = pv_location_to_vector(cmd.content.location);

    // use current location if not provided
    if(cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
        wp_nav.get_wp_stopping_point_xy(target_pos);
    }

    // use current altitude if not provided
    if( cmd.content.location.alt == 0 ) {
        target_pos.z = curr_pos.z;
    }

    // start way point navigator and provide it the desired location
    auto_wp_start(target_pos);

    // setup loiter timer
    loiter_time     = 0;
    loiter_time_max = cmd.p1;     // units are (seconds)
}

// do_spline_wp - initiate move to next waypoint
static void do_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    const Vector3f& curr_pos = inertial_nav.get_position();
    Vector3f local_pos = pv_location_to_vector_with_default(cmd.content.location, curr_pos);

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = abs(cmd.p1);

    // determine segment start and end type
    bool stopped_at_start = true;
    AC_WPNav::spline_segment_end_type seg_end_type = AC_WPNav::SEGMENT_END_STOP;
    AP_Mission::Mission_Command temp_cmd;
    Vector3f next_destination;      // end of next segment

    // if previous command was a wp_nav command with no delay set stopped_at_start to false
    // To-Do: move processing of delay into wp-nav controller to allow it to determine the stopped_at_start value itself?
    uint16_t prev_cmd_idx = mission.get_prev_nav_cmd_index();
    if (prev_cmd_idx != AP_MISSION_CMD_INDEX_NONE) {
        if (mission.read_cmd_from_storage(prev_cmd_idx, temp_cmd)) {
            if ((temp_cmd.id == MAV_CMD_NAV_WAYPOINT || temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT) && temp_cmd.p1 == 0) {
                stopped_at_start = false;
            }
        }
    }

    // if there is no delay at the end of this segment get next nav command
    if (cmd.p1 == 0 && mission.get_next_nav_cmd(cmd.index+1, temp_cmd)) {
        // if the next nav command is a waypoint set end type to spline or straight
        if (temp_cmd.id == MAV_CMD_NAV_WAYPOINT) {
            seg_end_type = AC_WPNav::SEGMENT_END_STRAIGHT;
            next_destination = pv_location_to_vector_with_default(temp_cmd.content.location, local_pos);
        }else if (temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT) {
            seg_end_type = AC_WPNav::SEGMENT_END_SPLINE;
            next_destination = pv_location_to_vector_with_default(temp_cmd.content.location, local_pos);
        }
    }

    // set spline navigation target
    auto_spline_start(local_pos, stopped_at_start, seg_end_type, next_destination);
}

#if NAV_GUIDED == ENABLED
// do_nav_guided_enable - initiate accepting commands from external nav computer
static void do_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 > 0) {
        // initialise guided limits
        guided_limit_init_time_and_pos();

        // set spline navigation target
        auto_nav_guided_start();
    }
}
#endif  // NAV_GUIDED


#if PARACHUTE == ENABLED
// do_parachute - configure or release parachute
static void do_parachute(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.p1) {
        case PARACHUTE_DISABLE:
            parachute.enabled(false);
            Log_Write_Event(DATA_PARACHUTE_DISABLED);
            break;
        case PARACHUTE_ENABLE:
            parachute.enabled(true);
            Log_Write_Event(DATA_PARACHUTE_ENABLED);
            break;
        case PARACHUTE_RELEASE:
            parachute_release();
            break;
        default:
            // do nothing
            break;
    }
}
#endif

#if EPM_ENABLED == ENABLED
// do_gripper - control EPM gripper
static void do_gripper(const AP_Mission::Mission_Command& cmd)
{
    // Note: we ignore the gripper num parameter because we only support one gripper
    switch (cmd.content.gripper.action) {
        case GRIPPER_ACTION_RELEASE:
            epm.release();
            Log_Write_Event(DATA_EPM_RELEASE);
            break;
        case GRIPPER_ACTION_GRAB:
            epm.grab();
            Log_Write_Event(DATA_EPM_GRAB);
            break;
        default:
            // do nothing
            break;
    }
}
#endif

#if NAV_GUIDED == ENABLED
// do_guided_limits - pass guided limits to guided controller
static void do_guided_limits(const AP_Mission::Mission_Command& cmd)
{
    guided_limit_set(cmd.p1 * 1000, // convert seconds to ms
                     cmd.content.guided_limits.alt_min * 100.0f,    // convert meters to cm
                     cmd.content.guided_limits.alt_max * 100.0f,    // convert meters to cm
                     cmd.content.guided_limits.horiz_max * 100.0f); // convert meters to cm
}
#endif

/********************************************************************************/
//	Verify Nav (Must) commands
/********************************************************************************/

// verify_takeoff - check if we have completed the takeoff
static bool verify_takeoff()
{
    // have we reached our target altitude?
    return wp_nav.reached_wp_destination();
}

// verify_land - returns true if landing has been completed
static bool verify_land()
{
    bool retval = false;

    switch( land_state ) {
        case LAND_STATE_FLY_TO_LOCATION:
            // check if we've reached the location
            if (wp_nav.reached_wp_destination()) {
                // get destination so we can use it for loiter target
                Vector3f dest = wp_nav.get_wp_destination();

                // initialise landing controller
                auto_land_start(dest);

                // advance to next state
                land_state = LAND_STATE_DESCENDING;
            }
            break;

        case LAND_STATE_DESCENDING:
            // rely on THROTTLE_LAND mode to correctly update landing status
            retval = ap.land_complete;
            break;

        default:
            // this should never happen
            // TO-DO: log an error
            retval = true;
            break;
    }

    // true is returned if we've successfully landed
    return retval;
}

// verify_nav_wp - check if we have reached the next way point
static bool verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if( !wp_nav.reached_wp_destination() ) {
        return false;
    }

    // play a tone
    AP_Notify::events.waypoint_complete = 1;

    // start timer if necessary
    if(loiter_time == 0) {
        loiter_time = millis();
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs_send_text_fmt(PSTR("Reached Command #%i"),cmd.index);
        return true;
    }else{
        return false;
    }
}

static bool verify_loiter_unlimited()
{
    return false;
}

// verify_loiter_time - check if we have loitered long enough
static bool verify_loiter_time()
{
    // return immediately if we haven't reached our destination
    if (!wp_nav.reached_wp_destination()) {
        return false;
    }

    // start our loiter timer
    if( loiter_time == 0 ) {
        loiter_time = millis();
    }

    // check if loiter timer has run out
    return (((millis() - loiter_time) / 1000) >= loiter_time_max);
}

// verify_circle - check if we have circled the point enough
static bool verify_circle(const AP_Mission::Mission_Command& cmd)
{
    // check if we've reached the edge
    if (auto_mode == Auto_CircleMoveToEdge) {
        if (wp_nav.reached_wp_destination()) {
            Vector3f curr_pos = inertial_nav.get_position();
            Vector3f circle_center = pv_location_to_vector(cmd.content.location);

            // set target altitude if not provided
            if (circle_center.z == 0) {
                circle_center.z = curr_pos.z;
            }

            // set lat/lon position if not provided
            if (cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
                circle_center.x = curr_pos.x;
                circle_center.y = curr_pos.y;
            }

            // start circling
            auto_circle_start();
        }
        return false;
    }

    // check if we have completed circling
    return fabsf(circle_nav.get_angle_total()/(2*M_PI)) >= (float)LOWBYTE(cmd.p1);
}

// externs to remove compiler warning
extern bool rtl_state_complete;

// verify_RTL - handles any state changes required to implement RTL
// do_RTL should have been called once first to initialise all variables
// returns true with RTL has completed successfully
static bool verify_RTL()
{
    return (rtl_state_complete && (rtl_state == FinalDescent || rtl_state == Land));
}

// verify_spline_wp - check if we have reached the next way point using spline
static bool verify_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if( !wp_nav.reached_wp_destination() ) {
        return false;
    }

    // start timer if necessary
    if(loiter_time == 0) {
        loiter_time = millis();
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs_send_text_fmt(PSTR("Reached Command #%i"),cmd.index);
        return true;
    }else{
        return false;
    }
}

#if NAV_GUIDED == ENABLED
// verify_nav_guided - check if we have breached any limits
static bool verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    // if disabling guided mode then immediately return true so we move to next command
    if (cmd.p1 == 0) {
        return true;
    }

    // check time and position limits
    return guided_limit_check();
}
#endif  // NAV_GUIDED


/********************************************************************************/
//	Condition (May) commands
/********************************************************************************/

static void do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value = cmd.content.delay.seconds * 1000;     // convert seconds to milliseconds
}

static void do_change_alt(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: store desired altitude in a variable so that it can be verified later
}

static void do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value  = cmd.content.distance.meters * 100;
}

static void do_yaw(const AP_Mission::Mission_Command& cmd)
{
	set_auto_yaw_look_at_heading(
		cmd.content.yaw.angle_deg,
		cmd.content.yaw.turn_rate_dps,
		cmd.content.yaw.direction,
		cmd.content.yaw.relative_angle);
}


/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

static bool verify_wait_delay()
{
    if (millis() - condition_start > (uint32_t)max(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

static bool verify_change_alt()
{
    // To-Do: use recorded target altitude to verify we have reached the target
    return true;
}

static bool verify_within_distance()
{
    // update distance calculation
    calc_wp_distance();
    if (wp_distance < max(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

// verify_yaw - return true if we have reached the desired heading
static bool verify_yaw()
{
    // set yaw mode if it has been changed (the waypoint controller often retakes control of yaw as it executes a new waypoint command)
    if (auto_yaw_mode != AUTO_YAW_LOOK_AT_HEADING) {
        set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
    }

    // check if we are within 2 degrees of the target heading
    if (labs(wrap_180_cd(ahrs.yaw_sensor-yaw_look_at_heading)) <= 200) {
        return true;
    }else{
        return false;
    }
}

/********************************************************************************/
//	Do (Now) commands
/********************************************************************************/

// do_guided - start guided mode
static bool do_guided(const AP_Mission::Mission_Command& cmd)
{
    Vector3f pos_or_vel;    // target location or velocity

    // only process guided waypoint if we are in guided mode
    if (control_mode != GUIDED && !(control_mode == AUTO && auto_mode == Auto_NavGuided)) {
        return false;
    }

    // switch to handle different commands
    switch (cmd.id) {

        case MAV_CMD_NAV_WAYPOINT:
            // set wp_nav's destination
            pos_or_vel = pv_location_to_vector(cmd.content.location);
            guided_set_destination(pos_or_vel);
            return true;
            break;

        case MAV_CMD_CONDITION_YAW:
            do_yaw(cmd);
            return true;
            break;

        default:
            // reject unrecognised command
            return false;
            break;
    }

    return true;
}

static void do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.speed.target_ms > 0) {
        wp_nav.set_speed_xy(cmd.content.speed.target_ms * 100.0f);
    }
}

static void do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if(cmd.p1 == 1 || (cmd.content.location.lat == 0 && cmd.content.location.lng == 0 && cmd.content.location.alt == 0)) {
        set_home_to_current_location();
    } else {
        if (!far_from_EKF_origin(cmd.content.location)) {
            set_home(cmd.content.location);
        }
    }
}

// do_roi - starts actions required by MAV_CMD_NAV_ROI
//          this involves either moving the camera to point at the ROI (region of interest)
//          and possibly rotating the copter to point at the ROI if our mount type does not support a yaw feature
//	TO-DO: add support for other features of MAV_CMD_DO_SET_ROI including pointing at a given waypoint
static void do_roi(const AP_Mission::Mission_Command& cmd)
{
    set_auto_yaw_roi(cmd.content.location);
}

// do_digicam_configure Send Digicam Configure message with the camera library
static void do_digicam_configure(const AP_Mission::Mission_Command& cmd)
{
#if CAMERA == ENABLED
    camera.configure_cmd(cmd);
#endif
}

// do_digicam_control Send Digicam Control message with the camera library
static void do_digicam_control(const AP_Mission::Mission_Command& cmd)
{
#if CAMERA == ENABLED
    camera.control_cmd(cmd);
    log_picture();
#endif
}

// do_take_picture - take a picture with the camera library
static void do_take_picture()
{
#if CAMERA == ENABLED
    camera.trigger_pic(true);
    log_picture();
#endif
}

// log_picture - log picture taken and send feedback to GCS
static void log_picture()
{
    gcs_send_message(MSG_CAMERA_FEEDBACK);
    if (should_log(MASK_LOG_CAMERA)) {
        DataFlash.Log_Write_Camera(ahrs, gps, current_loc);
    }
}

// point the camera to a specified angle
static void do_mount_control(const AP_Mission::Mission_Command& cmd)
{
#if MOUNT == ENABLED
    camera_mount.set_angle_targets(cmd.content.mount_control.roll, cmd.content.mount_control.pitch, cmd.content.mount_control.yaw);
#endif
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/compassmot.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  compass/motor interference calibration
 */

// setup_compassmot - sets compass's motor interference parameters
static uint8_t mavlink_compassmot(mavlink_channel_t chan)
{
    int8_t   comp_type;                 // throttle or current based compensation
    Vector3f compass_base[COMPASS_MAX_INSTANCES];           // compass vector when throttle is zero
    Vector3f motor_impact[COMPASS_MAX_INSTANCES];           // impact of motors on compass vector
    Vector3f motor_impact_scaled[COMPASS_MAX_INSTANCES];    // impact of motors on compass vector scaled with throttle
    Vector3f motor_compensation[COMPASS_MAX_INSTANCES];     // final compensation to be stored to eeprom
    float    throttle_pct;              // throttle as a percentage 0.0 ~ 1.0
    float    throttle_pct_max = 0.0f;   // maximum throttle reached (as a percentage 0~1.0)
    float    current_amps_max = 0.0f;   // maximum current reached
    float    interference_pct[COMPASS_MAX_INSTANCES];       // interference as a percentage of total mag field (for reporting purposes only)
    uint32_t last_run_time;
    uint32_t last_send_time;
    bool     updated = false;           // have we updated the compensation vector at least once
    uint8_t  command_ack_start = command_ack_counter;

    // exit immediately if we are already in compassmot
    if (ap.compass_mot) {
        // ignore restart messages
        return 1;
    }else{
        ap.compass_mot = true;
    }

    // check compass is enabled
    if (!g.compass_enabled) {
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("compass disabled\n"));
        ap.compass_mot = false;
        return 1;
    }

    // check compass health
    compass.read();
    for (uint8_t i=0; i<compass.get_count(); i++) {
        if (!compass.healthy(i)) {
            gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("check compass"));
            ap.compass_mot = false;
            return 1;
        }
    }

    // check if radio is calibrated
    pre_arm_rc_checks();
    if (!ap.pre_arm_rc_check) {
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("RC not calibrated"));
        ap.compass_mot = false;
        return 1;
    }

    // check throttle is at zero
    read_radio();
    if (g.rc_3.control_in != 0) {
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("thr not zero"));
        ap.compass_mot = false;
        return 1;
    }

    // check we are landed
    if (!ap.land_complete) {
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("Not landed"));
        ap.compass_mot = false;
        return 1;
    }

    // disable cpu failsafe
    failsafe_disable();

    // initialise compass
    init_compass();

    // default compensation type to use current if possible
    if (battery.has_current()) {
        comp_type = AP_COMPASS_MOT_COMP_CURRENT;
    }else{
        comp_type = AP_COMPASS_MOT_COMP_THROTTLE;
    }

    // send back initial ACK
    mavlink_msg_command_ack_send(chan, MAV_CMD_PREFLIGHT_CALIBRATION,0);

    // flash leds
    AP_Notify::flags.esc_calibration = true;

    // warn user we are starting calibration
    gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("STARTING CALIBRATION"));

    // inform what type of compensation we are attempting
    if (comp_type == AP_COMPASS_MOT_COMP_CURRENT) {
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("CURRENT"));
    } else{
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("THROTTLE"));
    }

    // disable throttle and battery failsafe
    g.failsafe_throttle = FS_THR_DISABLED;
    g.failsafe_battery_enabled = FS_BATT_DISABLED;

    // disable motor compensation
    compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    for (uint8_t i=0; i<compass.get_count(); i++) {
        compass.set_motor_compensation(i, Vector3f(0,0,0));
    }

    // get initial compass readings
    last_run_time = millis();
    while ( millis() - last_run_time < 500 ) {
        compass.accumulate();
    }
    compass.read();

    // store initial x,y,z compass values
    // initialise interference percentage
    for (uint8_t i=0; i<compass.get_count(); i++) {
        compass_base[i] = compass.get_field(i);
        interference_pct[i] = 0.0f;
    }

    // enable motors and pass through throttle
    init_rc_out();
    output_min();
    motors.armed(true);

    // initialise run time
    last_run_time = millis();
    last_send_time = millis();

    // main run while there is no user input and the compass is healthy
    while (command_ack_start == command_ack_counter && compass.healthy(compass.get_primary()) && motors.armed()) {
        // 50hz loop
        if (millis() - last_run_time < 20) {
            // grab some compass values
            compass.accumulate();
            hal.scheduler->delay(5);
            continue;
        }
        last_run_time = millis();

        // read radio input
        read_radio();
        
        // pass through throttle to motors
        motors.throttle_pass_through(g.rc_3.radio_in);
        
        // read some compass values
        compass.read();
        
        // read current
        read_battery();
        
        // calculate scaling for throttle
        throttle_pct = (float)g.rc_3.control_in / 1000.0f;
        throttle_pct = constrain_float(throttle_pct,0.0f,1.0f);

        // if throttle is near zero, update base x,y,z values
        if (throttle_pct <= 0.0f) {
            for (uint8_t i=0; i<compass.get_count(); i++) {
                compass_base[i] = compass_base[i] * 0.99f + compass.get_field(i) * 0.01f;
            }

            // causing printing to happen as soon as throttle is lifted
        } else {

            // calculate diff from compass base and scale with throttle
            for (uint8_t i=0; i<compass.get_count(); i++) {
                motor_impact[i] = compass.get_field(i) - compass_base[i];
            }

            // throttle based compensation
            if (comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
                // for each compass
                for (uint8_t i=0; i<compass.get_count(); i++) {
                    // scale by throttle
                    motor_impact_scaled[i] = motor_impact[i] / throttle_pct;
                    // adjust the motor compensation to negate the impact
                    motor_compensation[i] = motor_compensation[i] * 0.99f - motor_impact_scaled[i] * 0.01f;
                }

                updated = true;
            } else {
                // for each compass
                for (uint8_t i=0; i<compass.get_count(); i++) {
                    // current based compensation if more than 3amps being drawn
                    motor_impact_scaled[i] = motor_impact[i] / battery.current_amps();
                
                    // adjust the motor compensation to negate the impact if drawing over 3amps
                    if (battery.current_amps() >= 3.0f) {
                        motor_compensation[i] = motor_compensation[i] * 0.99f - motor_impact_scaled[i] * 0.01f;
                        updated = true;
                    }
                }
            }

            // calculate interference percentage at full throttle as % of total mag field
            if (comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
                for (uint8_t i=0; i<compass.get_count(); i++) {
                    // interference is impact@fullthrottle / mag field * 100
                    interference_pct[i] = motor_compensation[i].length() / (float)COMPASS_MAGFIELD_EXPECTED * 100.0f;
                }
            }else{
                for (uint8_t i=0; i<compass.get_count(); i++) {
                    // interference is impact/amp * (max current seen / max throttle seen) / mag field * 100
                    interference_pct[i] = motor_compensation[i].length() * (current_amps_max/throttle_pct_max) / (float)COMPASS_MAGFIELD_EXPECTED * 100.0f;
                }
            }

            // record maximum throttle and current
            throttle_pct_max = max(throttle_pct_max, throttle_pct);
            current_amps_max = max(current_amps_max, battery.current_amps());

        }
        if (hal.scheduler->millis() - last_send_time > 500) {
            last_send_time = hal.scheduler->millis();
            mavlink_msg_compassmot_status_send(chan, 
                                               g.rc_3.control_in,
                                               battery.current_amps(),
                                               interference_pct[compass.get_primary()],
                                               motor_compensation[compass.get_primary()].x,
                                               motor_compensation[compass.get_primary()].y,
                                               motor_compensation[compass.get_primary()].z);
        }
    }

    // stop motors
    motors.output_min();
    motors.armed(false);

    // set and save motor compensation
    if (updated) {
        compass.motor_compensation_type(comp_type);
        for (uint8_t i=0; i<compass.get_count(); i++) {
            compass.set_motor_compensation(i, motor_compensation[i]);
        }
        compass.save_motor_compensation();
        // display success message
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("Calibration Successful!"));
    } else {
        // compensation vector never updated, report failure
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH, PSTR("Failed!"));
        compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    }

    // display new motor offsets and save
    report_compass();

    // turn off notify leds
    AP_Notify::flags.esc_calibration = false;

    // re-enable cpu failsafe
    failsafe_enable();

    // re-enable failsafes
    g.failsafe_throttle.load();
    g.failsafe_battery_enabled.load();

    // flag we have completed
    ap.compass_mot = false;

    return 0;
}

#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/compat.pde"


static void delay(uint32_t ms)
{
    hal.scheduler->delay(ms);
}

static uint32_t millis()
{
    return hal.scheduler->millis();
}

static uint32_t micros()
{
    return hal.scheduler->micros();
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/control_acro.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_acro.pde - init and run calls for acro flight mode
 */

// acro_init - initialise acro controller
static bool acro_init(bool ignore_checks)
{
    // always successfully enter acro
    return true;
}

// acro_run - runs the acro controller
// should be called at 100hz or more
static void acro_run()
{
    float target_roll, target_pitch, target_yaw;
    int16_t pilot_throttle_scaled;

    // if motors not running reset angle targets
    if(!motors.armed() || g.rc_3.control_in <= 0) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // convert the input to the desired body frame rate
    get_pilot_desired_angle_rates(g.rc_1.control_in, g.rc_2.control_in, g.rc_4.control_in, target_roll, target_pitch, target_yaw);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);

    // run attitude controller
    attitude_control.rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);

    // output pilot's throttle without angle boost
    attitude_control.set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);
}


// get_pilot_desired_angle_rates - transform pilot's roll pitch and yaw input into a desired lean angle rates
// returns desired angle rates in centi-degrees-per-second
static void get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out)
{
    float rate_limit;
    Vector3f rate_ef_level, rate_bf_level, rate_bf_request;

    // apply circular limit to pitch and roll inputs
    float total_in = pythagorous2((float)pitch_in, (float)roll_in);

    if (total_in > ROLL_PITCH_INPUT_MAX) {
        float ratio = (float)ROLL_PITCH_INPUT_MAX / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }
    
    // calculate roll, pitch rate requests
    if (g.acro_expo <= 0) {
        rate_bf_request.x = roll_in * g.acro_rp_p;
        rate_bf_request.y = pitch_in * g.acro_rp_p;
    } else {
        // expo variables
        float rp_in, rp_in3, rp_out;

        // range check expo
        if (g.acro_expo > 1.0f) {
            g.acro_expo = 1.0f;
        }

        // roll expo
        rp_in = float(roll_in)/ROLL_PITCH_INPUT_MAX;
        rp_in3 = rp_in*rp_in*rp_in;
        rp_out = (g.acro_expo * rp_in3) + ((1 - g.acro_expo) * rp_in);
        rate_bf_request.x = ROLL_PITCH_INPUT_MAX * rp_out * g.acro_rp_p;

        // pitch expo
        rp_in = float(pitch_in)/ROLL_PITCH_INPUT_MAX;
        rp_in3 = rp_in*rp_in*rp_in;
        rp_out = (g.acro_expo * rp_in3) + ((1 - g.acro_expo) * rp_in);
        rate_bf_request.y = ROLL_PITCH_INPUT_MAX * rp_out * g.acro_rp_p;
    }

    // calculate yaw rate request
    rate_bf_request.z = yaw_in * g.acro_yaw_p;

    // calculate earth frame rate corrections to pull the copter back to level while in ACRO mode

    if (g.acro_trainer != ACRO_TRAINER_DISABLED) {
        // Calculate trainer mode earth frame rate command for roll
        int32_t roll_angle = wrap_180_cd(ahrs.roll_sensor);
        rate_ef_level.x = -constrain_int32(roll_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_roll;

        // Calculate trainer mode earth frame rate command for pitch
        int32_t pitch_angle = wrap_180_cd(ahrs.pitch_sensor);
        rate_ef_level.y = -constrain_int32(pitch_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_pitch;

        // Calculate trainer mode earth frame rate command for yaw
        rate_ef_level.z = 0;

        // Calculate angle limiting earth frame rate commands
        if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
            if (roll_angle > aparm.angle_max){
                rate_ef_level.x -=  g.acro_balance_roll*(roll_angle-aparm.angle_max);
            }else if (roll_angle < -aparm.angle_max) {
                rate_ef_level.x -=  g.acro_balance_roll*(roll_angle+aparm.angle_max);
            }

            if (pitch_angle > aparm.angle_max){
                rate_ef_level.y -=  g.acro_balance_pitch*(pitch_angle-aparm.angle_max);
            }else if (pitch_angle < -aparm.angle_max) {
                rate_ef_level.y -=  g.acro_balance_pitch*(pitch_angle+aparm.angle_max);
            }
        }

        // convert earth-frame level rates to body-frame level rates
        attitude_control.frame_conversion_ef_to_bf(rate_ef_level, rate_bf_level);

        // combine earth frame rate corrections with rate requests
        if (g.acro_trainer == ACRO_TRAINER_LIMITED) {
            rate_bf_request.x += rate_bf_level.x;
            rate_bf_request.y += rate_bf_level.y;
            rate_bf_request.z += rate_bf_level.z;
        }else{
            acro_level_mix = constrain_float(1-max(max(abs(roll_in), abs(pitch_in)), abs(yaw_in))/4500.0, 0, 1)*ahrs.cos_pitch();

            // Scale leveling rates by stick input
            rate_bf_level = rate_bf_level*acro_level_mix;

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabs(fabs(rate_bf_request.x)-fabs(rate_bf_level.x));
            rate_bf_request.x += rate_bf_level.x;
            rate_bf_request.x = constrain_float(rate_bf_request.x, -rate_limit, rate_limit);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabs(fabs(rate_bf_request.y)-fabs(rate_bf_level.y));
            rate_bf_request.y += rate_bf_level.y;
            rate_bf_request.y = constrain_float(rate_bf_request.y, -rate_limit, rate_limit);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabs(fabs(rate_bf_request.z)-fabs(rate_bf_level.z));
            rate_bf_request.z += rate_bf_level.z;
            rate_bf_request.z = constrain_float(rate_bf_request.z, -rate_limit, rate_limit);
        }
    }

    // hand back rate request
    roll_out = rate_bf_request.x;
    pitch_out = rate_bf_request.y;
    yaw_out = rate_bf_request.z;
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/control_althold.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_althold.pde - init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
static bool althold_init(bool ignore_checks)
{
    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
static void althold_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    int16_t target_climb_rate;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(g.rc_3.control_in)-throttle_average);
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

    // get pilot desired climb rate
    target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

    // check for pilot requested take-off
    if (ap.land_complete && target_climb_rate > 0) {
        // indicate we are taking off
        set_land_complete(false);
        // clear i term when we're taking off
        set_throttle_takeoff();
    }

    // reset target lean angles and heading while landed
    if (ap.land_complete) {
        attitude_control.set_throttle_out_unstabilized(get_throttle_pre_takeoff(g.rc_3.control_in),true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(g.rc_3.control_in)-throttle_average);
    }else{
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        // body-frame rate controller is run directly from 100hz loop

        // call throttle controller
        if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
            // if sonar is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
    }
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/control_auto.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_auto.pde - init and run calls for auto flight mode
 *
 * This file contains the implementation for Land, Waypoint navigation and Takeoff from Auto mode
 * Command execution code (i.e. command_logic.pde) should:
 *      a) switch to Auto flight mode with set_mode() function.  This will cause auto_init to be called
 *      b) call one of the three auto initialisation functions: auto_wp_start(), auto_takeoff_start(), auto_land_start()
 *      c) call one of the verify functions auto_wp_verify(), auto_takeoff_verify, auto_land_verify repeated to check if the command has completed
 * The main loop (i.e. fast loop) will call update_flight_modes() which will in turn call auto_run() which, based upon the auto_mode variable will call
 *      correct auto_wp_run, auto_takeoff_run or auto_land_run to actually implement the feature
 */

/*
 *  While in the auto flight mode, navigation or do/now commands can be run.
 *  Code in this file implements the navigation commands
 */

// auto_init - initialise auto controller
static bool auto_init(bool ignore_checks)
{
    if ((position_ok() && mission.num_commands() > 1) || ignore_checks) {
        auto_mode = Auto_Loiter;

        // stop ROI from carrying over from previous runs of the mission
        // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
        if (auto_yaw_mode == AUTO_YAW_ROI) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }

        // initialise waypoint and spline controller
        wp_nav.wp_and_spline_init();

        // clear guided limits
        guided_limit_clear();

        // start/resume the mission (based on MIS_RESTART parameter)
        mission.start_or_resume();
        return true;
    }else{
        return false;
    }
}

// auto_run - runs the auto controller
//      should be called at 100hz or more
//      relies on run_autopilot being called at 10hz which handles decision making and non-navigation related commands
static void auto_run()
{
    // call the correct auto controller
    switch (auto_mode) {

    case Auto_TakeOff:
        auto_takeoff_run();
        break;

    case Auto_WP:
    case Auto_CircleMoveToEdge:
        auto_wp_run();
        break;

    case Auto_Land:
        auto_land_run();
        break;

    case Auto_RTL:
        auto_rtl_run();
        break;

    case Auto_Circle:
        auto_circle_run();
        break;

    case Auto_Spline:
        auto_spline_run();
        break;

    case Auto_NavGuided:
#if NAV_GUIDED == ENABLED
        auto_nav_guided_run();
#endif
        break;

    case Auto_Loiter:
        auto_loiter_run();
        break;
    }
}

// auto_takeoff_start - initialises waypoint controller to implement take-off
static void auto_takeoff_start(float final_alt_above_home)
{
    auto_mode = Auto_TakeOff;

    // initialise wpnav destination
    Vector3f target_pos = inertial_nav.get_position();
    target_pos.z = pv_alt_above_origin(final_alt_above_home);
    wp_nav.set_wp_destination(target_pos);

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    // tell motors to do a slow start
    motors.slow_start(true);
}

// auto_takeoff_run - takeoff in auto mode
//      called by auto_run at 100hz or more
static void auto_takeoff_run()
{
    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        // initialise wpnav targets
        wp_nav.shift_wp_origin_to_current_pos();
        // reset attitude control targets
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        // tell motors to do a slow start
        motors.slow_start(true);
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
static void auto_wp_start(const Vector3f& destination)
{
    auto_mode = Auto_WP;

    // initialise wpnav
    wp_nav.set_wp_destination(destination);

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    }
}

// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
static void auto_wp_run()
{
    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        // To-Do: reset waypoint origin to current location because copter is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        // tell motors to do a slow start
        motors.slow_start(true);
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
        if (target_yaw_rate != 0) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(),true);
    }
}

// auto_spline_start - initialises waypoint controller to implement flying to a particular destination using the spline controller
//  seg_end_type can be SEGMENT_END_STOP, SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE.  If Straight or Spline the next_destination should be provided
static void auto_spline_start(const Vector3f& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Vector3f& next_destination)
{
    auto_mode = Auto_Spline;

    // initialise wpnav
    wp_nav.set_spline_destination(destination, stopped_at_start, seg_end_type, next_destination);

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    }
}

// auto_spline_run - runs the auto spline controller
//      called by auto_run at 100hz or more
static void auto_spline_run()
{
    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        // To-Do: reset waypoint origin to current location because copter is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        // tell motors to do a slow start
        motors.slow_start(true);
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
        if (target_yaw_rate != 0) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // run waypoint controller
    wp_nav.update_spline();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(), true);
    }
}

// auto_land_start - initialises controller to implement a landing
static void auto_land_start()
{
    // set target to stopping point
    Vector3f stopping_point;
    wp_nav.get_loiter_stopping_point_xy(stopping_point);

    // call location specific land start function
    auto_land_start(stopping_point);
}

// auto_land_start - initialises controller to implement a landing
static void auto_land_start(const Vector3f& destination)
{
    auto_mode = Auto_Land;

    // initialise loiter target destination
    wp_nav.init_loiter_target(destination);

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// auto_land_run - lands in auto mode
//      called by auto_run at 100hz or more
static void auto_land_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.land_complete) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        // set target to current position
        wp_nav.init_loiter_target();
        return;
    }

    // relax loiter targets if we might be landed
    if (land_complete_maybe()) {
        wp_nav.loiter_soften_for_landing();
    }

    // process pilot's input
    if (!failsafe.radio) {
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // process pilot's roll and pitch input
            roll_control = g.rc_1.control_in;
            pitch_control = g.rc_2.control_in;
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

    // process roll, pitch inputs
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

    // run loiter controller
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call z-axis position controller
    float cmb_rate = get_land_descent_speed();
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();

    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
}

// auto_rtl_start - initialises RTL in AUTO flight mode
static void auto_rtl_start()
{
    auto_mode = Auto_RTL;

    // call regular rtl flight mode initialisation and ask it to ignore checks
    rtl_init(true);
}

// auto_rtl_run - rtl in AUTO flight mode
//      called by auto_run at 100hz or more
void auto_rtl_run()
{
    // call regular rtl flight mode run function
    rtl_run();
}

// auto_circle_movetoedge_start - initialise waypoint controller to move to edge of a circle with it's center at the specified location
//  we assume the caller has set the circle's circle with circle_nav.set_center()
//  we assume the caller has performed all required GPS_ok checks
static void auto_circle_movetoedge_start()
{
    // check our distance from edge of circle
    Vector3f circle_edge;
    circle_nav.get_closest_point_on_circle(circle_edge);

    // set the state to move to the edge of the circle
    auto_mode = Auto_CircleMoveToEdge;

    // initialise wpnav to move to edge of circle
    wp_nav.set_wp_destination(circle_edge);

    // if we are outside the circle, point at the edge, otherwise hold yaw
    const Vector3f &curr_pos = inertial_nav.get_position();
    const Vector3f &circle_center = circle_nav.get_center();
    float dist_to_center = pythagorous2(circle_center.x - curr_pos.x, circle_center.y - curr_pos.y);
    if (dist_to_center > circle_nav.get_radius() && dist_to_center > 500) {
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    } else {
        // vehicle is within circle so hold yaw to avoid spinning as we move to edge of circle
        set_auto_yaw_mode(AUTO_YAW_HOLD);
    }
}

// auto_circle_start - initialises controller to fly a circle in AUTO flight mode
static void auto_circle_start()
{
    auto_mode = Auto_Circle;

    // initialise circle controller
    // center was set in do_circle so initialise with current center
    circle_nav.init(circle_nav.get_center());
}

// auto_circle_run - circle in AUTO flight mode
//      called by auto_run at 100hz or more
void auto_circle_run()
{
    // call circle controller
    circle_nav.update();

    // call z-axis position controller
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), circle_nav.get_yaw(),true);
}

#if NAV_GUIDED == ENABLED
// auto_nav_guided_start - hand over control to external navigation controller in AUTO mode
void auto_nav_guided_start()
{
    auto_mode = Auto_NavGuided;

    // call regular guided flight mode initialisation
    guided_init(true);

    // initialise guided start time and position as reference for limit checking
    guided_limit_init_time_and_pos();
}

// auto_nav_guided_run - allows control by external navigation controller
//      called by auto_run at 100hz or more
void auto_nav_guided_run()
{
    // call regular guided flight mode run function
    guided_run();
}
#endif  // NAV_GUIDED

// auto_loiter_start - initialises loitering in auto mode
//  returns success/failure because this can be called by exit_mission
bool auto_loiter_start()
{
    // return failure if GPS is bad
    if (!position_ok()) {
        return false;
    }
    auto_mode = Auto_Loiter;

    Vector3f origin = inertial_nav.get_position();

    // calculate stopping point
    Vector3f stopping_point;
    pos_control.get_stopping_point_xy(stopping_point);
    pos_control.get_stopping_point_z(stopping_point);

    // initialise waypoint controller target to stopping point
    wp_nav.set_wp_origin_and_destination(origin, stopping_point);

    // hold yaw at current heading
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    return true;
}

// auto_loiter_run - loiter in AUTO flight mode
//      called by auto_run at 100hz or more
void auto_loiter_run()
{
    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.land_complete) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // accept pilot input of yaw
    float target_yaw_rate = 0;
    if(!failsafe.radio) {
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

    // run waypoint and z-axis postion controller
    wp_nav.update_wpnav();
    pos_control.update_z_controller();
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
}

// get_default_auto_yaw_mode - returns auto_yaw_mode based on WP_YAW_BEHAVIOR parameter
// set rtl parameter to true if this is during an RTL
uint8_t get_default_auto_yaw_mode(bool rtl)
{
    switch (g.wp_yaw_behavior) {

        case WP_YAW_BEHAVIOR_NONE:
            return AUTO_YAW_HOLD;
            break;

        case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL:
            if (rtl) {
                return AUTO_YAW_HOLD;
            }else{
                return AUTO_YAW_LOOK_AT_NEXT_WP;
            }
            break;

        case WP_YAW_BEHAVIOR_LOOK_AHEAD:
            return AUTO_YAW_LOOK_AHEAD;
            break;

        case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP:
        default:
            return AUTO_YAW_LOOK_AT_NEXT_WP;
            break;
    }
}

// set_auto_yaw_mode - sets the yaw mode for auto
void set_auto_yaw_mode(uint8_t yaw_mode)
{
    // return immediately if no change
    if (auto_yaw_mode == yaw_mode) {
        return;
    }
    auto_yaw_mode = yaw_mode;

    // perform initialisation
    switch (auto_yaw_mode) {

    case AUTO_YAW_LOOK_AT_NEXT_WP:
        // wpnav will initialise heading when wpnav's set_destination method is called
        break;

    case AUTO_YAW_ROI:
        // point towards a location held in yaw_look_at_WP
        yaw_look_at_WP_bearing = ahrs.yaw_sensor;
        break;

    case AUTO_YAW_LOOK_AT_HEADING:
        // keep heading pointing in the direction held in yaw_look_at_heading
        // caller should set the yaw_look_at_heading
        break;

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        yaw_look_ahead_bearing = ahrs.yaw_sensor;
        break;

    case AUTO_YAW_RESETTOARMEDYAW:
        // initial_armed_bearing will be set during arming so no init required
        break;
    }
}

// set_auto_yaw_look_at_heading - sets the yaw look at heading for auto mode
static void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, uint8_t relative_angle)
{
    // get current yaw target
    int32_t curr_yaw_target = attitude_control.angle_ef_targets().z;

    // get final angle, 1 = Relative, 0 = Absolute
    if (relative_angle == 0) {
        // absolute angle
        yaw_look_at_heading = wrap_360_cd(angle_deg * 100);
    } else {
        // relative angle
        if (direction < 0) {
            angle_deg = -angle_deg;
        }
        yaw_look_at_heading = wrap_360_cd((angle_deg*100+curr_yaw_target));
    }

    // get turn speed
    if (turn_rate_dps == 0 ) {
        // default to regular auto slew rate
        yaw_look_at_heading_slew = AUTO_YAW_SLEW_RATE;
    }else{
        int32_t turn_rate = (wrap_180_cd(yaw_look_at_heading - curr_yaw_target) / 100) / turn_rate_dps;
        yaw_look_at_heading_slew = constrain_int32(turn_rate, 1, 360);    // deg / sec
    }

    // set yaw mode
    set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);

    // TO-DO: restore support for clockwise and counter clockwise rotation held in cmd.content.yaw.direction.  1 = clockwise, -1 = counterclockwise
}

// set_auto_yaw_roi - sets the yaw to look at roi for auto mode
static void set_auto_yaw_roi(const Location &roi_location)
{
    // if location is zero lat, lon and altitude turn off ROI
    if (roi_location.alt == 0 && roi_location.lat == 0 && roi_location.lng == 0) {
        // set auto yaw mode back to default assuming the active command is a waypoint command.  A more sophisticated method is required to ensure we return to the proper yaw control for the active command
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
#if MOUNT == ENABLED
        // switch off the camera tracking if enabled
        if (camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
            camera_mount.set_mode_to_default();
        }
#endif  // MOUNT == ENABLED
    }else{
#if MOUNT == ENABLED
        // check if mount type requires us to rotate the quad
        if(!camera_mount.has_pan_control()) {
            roi_WP = pv_location_to_vector(roi_location);
            set_auto_yaw_mode(AUTO_YAW_ROI);
        }
        // send the command to the camera mount
        camera_mount.set_roi_target(roi_location);

        // TO-DO: expand handling of the do_nav_roi to support all modes of the MAVLink.  Currently we only handle mode 4 (see below)
        //      0: do nothing
        //      1: point at next waypoint
        //      2: point at a waypoint taken from WP# parameter (2nd parameter?)
        //      3: point at a location given by alt, lon, lat parameters
        //      4: point at a target given a target id (can't be implemented)
#else
        // if we have no camera mount aim the quad at the location
        roi_WP = pv_location_to_vector(roi_location);
        set_auto_yaw_mode(AUTO_YAW_ROI);
#endif  // MOUNT == ENABLED
    }
}

// get_auto_heading - returns target heading depending upon auto_yaw_mode
// 100hz update rate
float get_auto_heading(void)
{
    switch(auto_yaw_mode) {

    case AUTO_YAW_ROI:
        // point towards a location held in roi_WP
        return get_roi_yaw();
        break;

    case AUTO_YAW_LOOK_AT_HEADING:
        // keep heading pointing in the direction held in yaw_look_at_heading with no pilot input allowed
        return yaw_look_at_heading;
        break;

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        return get_look_ahead_yaw();
        break;

    case AUTO_YAW_RESETTOARMEDYAW:
        // changes yaw to be same as when quad was armed
        return initial_armed_bearing;
        break;

    case AUTO_YAW_LOOK_AT_NEXT_WP:
    default:
        // point towards next waypoint.
        // we don't use wp_bearing because we don't want the copter to turn too much during flight
        return wp_nav.get_yaw();
        break;
    }
}

#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/control_autotune.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if AUTOTUNE_ENABLED == ENABLED

/*
 * control_autotune.pde - init and run calls for autotune flight mode
 *
 * Instructions:
 *      1) Set up one flight mode switch position to be AltHold.
 *      2) Set the Ch7 Opt or Ch8 Opt to AutoTune to allow you to turn the auto tuning on/off with the ch7 or ch8 switch.
 *      3) Ensure the ch7 or ch8 switch is in the LOW position.
 *      4) Wait for a calm day and go to a large open area.
 *      5) Take off and put the vehicle into AltHold mode at a comfortable altitude.
 *      6) Set the ch7/ch8 switch to the HIGH position to engage auto tuning:
 *          a) You will see it twitch about 20 degrees left and right for a few minutes, then it will repeat forward and back.
 *          b) Use the roll and pitch stick at any time to reposition the copter if it drifts away (it will use the original PID gains during repositioning and between tests).
 *             When you release the sticks it will continue auto tuning where it left off.
 *          c) Move the ch7/ch8 switch into the LOW position at any time to abandon the autotuning and return to the origin PIDs.
 *          d) Make sure that you do not have any trim set on your transmitter or the autotune may not get the signal that the sticks are centered.
 *      7) When the tune completes the vehicle will change back to the original PID gains.
 *      8) Put the ch7/ch8 switch into the LOW position then back to the HIGH position to test the tuned PID gains.
 *      9) Put the ch7/ch8 switch into the LOW position to fly using the original PID gains.
 *      10) If you are happy with the autotuned PID gains, leave the ch7/ch8 switch in the HIGH position, land and disarm to save the PIDs permanently.
 *          If you DO NOT like the new PIDS, switch ch7/ch8 LOW to return to the original PIDs. The gains will not be saved when you disarm
 *
 * What it's doing during each "twitch":
 *      a) invokes 90 deg/sec rate request
 *      b) records maximum "forward" roll rate and bounce back rate
 *      c) when copter reaches 20 degrees or 1 second has passed, it commands level
 *      d) tries to keep max rotation rate between 80% ~ 100% of requested rate (90deg/sec) by adjusting rate P
 *      e) increases rate D until the bounce back becomes greater than 10% of requested rate (90deg/sec)
 *      f) decreases rate D until the bounce back becomes less than 10% of requested rate (90deg/sec)
 *      g) increases rate P until the max rotate rate becomes greater than the request rate (90deg/sec)
 *      h) invokes a 20deg angle request on roll or pitch
 *      i) increases stab P until the maximum angle becomes greater than 110% of the requested angle (20deg)
 *      j) decreases stab P by 25%
 *
 * Notes: AUTOTUNE should not be set-up as a flight mode, it should be invoked only from the ch7/ch8 switch.
 *
 */

#define AUTOTUNE_AXIS_BITMASK_ROLL            1
#define AUTOTUNE_AXIS_BITMASK_PITCH           2
#define AUTOTUNE_AXIS_BITMASK_YAW             4

#define AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS  500     // restart tuning if pilot has left sticks in middle for 2 seconds
#define AUTOTUNE_TESTING_STEP_TIMEOUT_MS    500     // timeout for tuning mode's testing step
#define AUTOTUNE_LEVEL_ANGLE_CD             300     // angle which qualifies as level
#define AUTOTUNE_LEVEL_RATE_RP_CD          1000     // rate which qualifies as level for roll and pitch
#define AUTOTUNE_LEVEL_RATE_Y_CD            750     // rate which qualifies as level for yaw
#define AUTOTUNE_REQUIRED_LEVEL_TIME_MS     500     // time we require the copter to be level
#define AUTOTUNE_RD_STEP                  0.05f     // minimum increment when increasing/decreasing Rate D term
#define AUTOTUNE_RP_STEP                  0.05f     // minimum increment when increasing/decreasing Rate P term
#define AUTOTUNE_SP_STEP                  0.05f     // minimum increment when increasing/decreasing Stab P term
#define AUTOTUNE_RD_TEST_NOISE            0.05f     // Rate D gains are reduced to 50% of their maximum value discovered during tuning
#define AUTOTUNE_RD_BACKOFF                4.0f     // Rate D gains are reduced to 50% of their maximum value discovered during tuning
#define AUTOTUNE_RP_BACKOFF                1.0f     // Rate P gains are reduced to 97.5% of their maximum value discovered during tuning
#define AUTOTUNE_SP_BACKOFF                1.0f     // Stab P gains are reduced to 60% of their maximum value discovered during tuning
#define AUTOTUNE_PI_RATIO_FOR_TESTING      0.1f     // I is set 10x smaller than P during testing
#define AUTOTUNE_PI_RATIO_FINAL            2.5f     // I is set 1x P after testing
#define AUTOTUNE_YAW_PI_RATIO_FINAL        0.1f     // I is set 1x P after testing
#define AUTOTUNE_RD_MIN                  0.004f     // minimum Rate D value
#define AUTOTUNE_RD_MAX                  0.050f     // maximum Rate D value
#define AUTOTUNE_RLPF_MIN                  1.0f     // minimum Rate Yaw filter value
#define AUTOTUNE_RLPF_MAX                 10.0f     // maximum Rate Yaw filter value
#define AUTOTUNE_RP_MIN                   0.01f     // minimum Rate P value
#define AUTOTUNE_RP_MAX                    2.0f     // maximum Rate P value
#define AUTOTUNE_SP_MAX                   20.0f     // maximum Stab P value
#define AUTOTUNE_SP_MIN                    0.5f     // maximum Stab P value
#define AUTOTUNE_ACCEL_RP_BACKOFF          1.0f     // back off from maximum acceleration
#define AUTOTUNE_ACCEL_Y_BACKOFF          0.75f     // back off from maximum acceleration
#define AUTOTUNE_RP_ACCEL_MIN          36000.0f     // Minimum acceleration for Roll and Pitch
#define AUTOTUNE_Y_ACCEL_MIN            9000.0f     // Minimum acceleration for Yaw
#define AUTOTUNE_Y_FILT_FREQ              10.0f     // Minimum acceleration for Roll and Pitch
#define AUTOTUNE_SUCCESS_COUNT                4     // how many successful iterations we need to freeze at current gains
#define AUTOTUNE_D_UP_DOWN_MARGIN          0.2f     // The margin below the target that we tune D in

// roll and pitch axes
#define AUTOTUNE_TARGET_ANGLE_RLLPIT_CD     2000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_RATE_RLLPIT_CDS     9000    // target roll/pitch rate during AUTOTUNE_STEP_TWITCHING step
#define AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD 1000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS 4500    // target roll/pitch rate during AUTOTUNE_STEP_TWITCHING step

// yaw axis
#define AUTOTUNE_TARGET_ANGLE_YAW_CD        1000    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_RATE_YAW_CDS        3000    // target yaw rate during AUTOTUNE_STEP_TWITCHING step
#define AUTOTUNE_TARGET_MIN_ANGLE_YAW_CD     500    // target angle during TESTING_RATE step that will cause us to move to next step
#define AUTOTUNE_TARGET_MIN_RATE_YAW_CDS    1500    // target yaw rate during AUTOTUNE_STEP_TWITCHING step

// Auto Tune message ids for ground station
#define AUTOTUNE_MESSAGE_STARTED 0
#define AUTOTUNE_MESSAGE_STOPPED 1
#define AUTOTUNE_MESSAGE_SUCCESS 2
#define AUTOTUNE_MESSAGE_FAILED 3
#define AUTOTUNE_MESSAGE_SAVED_GAINS 4

// autotune modes (high level states)
enum AutoTuneTuneMode {
    AUTOTUNE_MODE_UNINITIALISED = 0,        // autotune has never been run
    AUTOTUNE_MODE_TUNING = 1,               // autotune is testing gains
    AUTOTUNE_MODE_SUCCESS = 2,              // tuning has completed, user is flight testing the new gains
    AUTOTUNE_MODE_FAILED = 3,               // tuning has failed, user is flying on original gains
};

// steps performed while in the tuning mode
enum AutoTuneStepType {
    AUTOTUNE_STEP_WAITING_FOR_LEVEL = 0,    // autotune is waiting for vehicle to return to level before beginning the next twitch
    AUTOTUNE_STEP_TWITCHING = 1,            // autotune has begun a twitch and is watching the resulting vehicle movement
    AUTOTUNE_STEP_UPDATE_GAINS = 2          // autotune has completed a twitch and is updating the gains based on the results
};

// things that can be tuned
enum AutoTuneAxisType {
    AUTOTUNE_AXIS_ROLL = 0,                 // roll axis is being tuned (either angle or rate)
    AUTOTUNE_AXIS_PITCH = 1,                // pitch axis is being tuned (either angle or rate)
    AUTOTUNE_AXIS_YAW = 2,                  // pitch axis is being tuned (either angle or rate)
};

// mini steps performed while in Tuning mode, Testing step
enum AutoTuneTuneType {
    AUTOTUNE_TYPE_RD_UP = 0,                // rate D is being tuned up
    AUTOTUNE_TYPE_RD_DOWN = 1,              // rate D is being tuned down
    AUTOTUNE_TYPE_RP_UP = 2,                // rate P is being tuned up
    AUTOTUNE_TYPE_SP_DOWN = 3,              // angle P is being tuned up
    AUTOTUNE_TYPE_SP_UP = 4                 // angle P is being tuned up
};

// autotune_state_struct - hold state flags
struct autotune_state_struct {
    AutoTuneTuneMode    mode                : 2;    // see AutoTuneTuneMode for what modes are allowed
    uint8_t             pilot_override      : 1;    // 1 = pilot is overriding controls so we suspend tuning temporarily
    AutoTuneAxisType    axis                : 2;    // see AutoTuneAxisType for which things can be tuned
    uint8_t             positive_direction  : 1;    // 0 = tuning in negative direction (i.e. left for roll), 1 = positive direction (i.e. right for roll)
    AutoTuneStepType    step                : 2;    // see AutoTuneStepType for what steps are performed
    AutoTuneTuneType    tune_type           : 3;    // see AutoTuneTuneType
    uint8_t             ignore_next         : 1;    // 1 = ignore the next test
} autotune_state;

// variables
static uint32_t autotune_override_time;                         // the last time the pilot overrode the controls
static float    autotune_test_min;                              // the minimum angular rate achieved during TESTING_RATE step
static float    autotune_test_max;                              // the maximum angular rate achieved during TESTING_RATE step
static uint32_t autotune_step_start_time;                       // start time of current tuning step (used for timeout checks)
static uint32_t autotune_step_stop_time;                        // start time of current tuning step (used for timeout checks)
static int8_t   autotune_counter;                               // counter for tuning gains
static float    autotune_target_rate, autotune_start_rate;      // target and start rate
static float    autotune_target_angle, autotune_start_angle;    // target and start angles
static float    autotune_desired_yaw;                           // yaw heading during tune
static float    rate_max, autotune_test_accel_max;              // maximum acceleration variables

LowPassFilterFloat  rotation_rate_filt;                         // filtered rotation rate in radians/second

// backup of currently being tuned parameter values
static float    orig_roll_rp = 0, orig_roll_ri, orig_roll_rd, orig_roll_sp;
static float    orig_pitch_rp = 0, orig_pitch_ri, orig_pitch_rd, orig_pitch_sp;
static float    orig_yaw_rp = 0, orig_yaw_ri, orig_yaw_rd, orig_yaw_rLPF, orig_yaw_sp;

// currently being tuned parameter values
static float    tune_roll_rp, tune_roll_rd, tune_roll_sp, tune_roll_accel;
static float    tune_pitch_rp, tune_pitch_rd, tune_pitch_sp, tune_pitch_accel;
static float    tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp, tune_yaw_accel;

// autotune_init - should be called when autotune mode is selected
static bool autotune_init(bool ignore_checks)
{
    bool success = true;

    switch (autotune_state.mode) {
        case AUTOTUNE_MODE_FAILED:
        // autotune has been run but failed so reset state to uninitialized
            autotune_state.mode = AUTOTUNE_MODE_UNINITIALISED;
            // no break to allow fall through to restart the tuning

        case AUTOTUNE_MODE_UNINITIALISED:
            // autotune has never been run
            success = autotune_start(false);
            if (success) {
                // so store current gains as original gains
                autotune_backup_gains_and_initialise();
                // advance mode to tuning
                autotune_state.mode = AUTOTUNE_MODE_TUNING;
                // send message to ground station that we've started tuning
                autotune_update_gcs(AUTOTUNE_MESSAGE_STARTED);
            }
            break;

        case AUTOTUNE_MODE_TUNING:
            // we are restarting tuning after the user must have switched ch7/ch8 off so we restart tuning where we left off
            success = autotune_start(false);
            if (success) {
                // reset gains to tuning-start gains (i.e. low I term)
                autotune_load_intra_test_gains();
                // write dataflash log even and send message to ground station
                Log_Write_Event(DATA_AUTOTUNE_RESTART);
                autotune_update_gcs(AUTOTUNE_MESSAGE_STARTED);
            }
            break;

        case AUTOTUNE_MODE_SUCCESS:
            // we have completed a tune and the pilot wishes to test the new gains in the current flight mode
            // so simply apply tuning gains (i.e. do not change flight mode)
            autotune_load_tuned_gains();
            Log_Write_Event(DATA_AUTOTUNE_PILOT_TESTING);
            break;
    }

    return success;
}

// autotune_stop - should be called when the ch7/ch8 switch is switched OFF
static void autotune_stop()
{
    // set gains to their original values
    autotune_load_orig_gains();

    // re-enable angle-to-rate request limits
    attitude_control.limit_angle_to_rate_request(true);

    // log off event and send message to ground station
    autotune_update_gcs(AUTOTUNE_MESSAGE_STOPPED);
    Log_Write_Event(DATA_AUTOTUNE_OFF);

    // Note: we leave the autotune_state.mode as it was so that we know how the autotune ended
    // we expect the caller will change the flight mode back to the flight mode indicated by the flight mode switch
}

// autotune_start - Initialize autotune flight mode
static bool autotune_start(bool ignore_checks)
{
    // only allow flip from Stabilize or AltHold flight modes
    if (control_mode != STABILIZE && control_mode != ALT_HOLD) {
        return false;
    }

    // ensure throttle is above zero
    if (g.rc_3.control_in <= 0) {
        return false;
    }

    // ensure we are flying
    if (!motors.armed() || !ap.auto_armed || ap.land_complete) {
        return false;
    }

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    return true;
}

// autotune_run - runs the autotune flight mode
// should be called at 100hz or more
static void autotune_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    int16_t target_climb_rate;

    // if not auto armed set throttle to zero and exit immediately
    // this should not actually be possible because of the autotune_init() checks
    if (!ap.auto_armed) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(g.rc_3.control_in)-throttle_average);
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

    // get pilot desired climb rate
    target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

    // check for pilot requested take-off - this should not actually be possible because of autotune_init() checks
    if (ap.land_complete && target_climb_rate > 0) {
        // indicate we are taking off
        set_land_complete(false);
        // clear i term when we're taking off
        set_throttle_takeoff();
    }

    // reset target lean angles and heading while landed
    if (ap.land_complete) {
        // move throttle to between minimum and non-takeoff-throttle to keep us on the ground
        attitude_control.set_throttle_out_unstabilized(get_throttle_pre_takeoff(g.rc_3.control_in),true,g.throttle_filt);
        pos_control.set_alt_target_to_current_alt();
    }else{
        // check if pilot is overriding the controls
        if (target_roll != 0 || target_pitch != 0 || target_yaw_rate != 0.0f || target_climb_rate != 0) {
            if (!autotune_state.pilot_override) {
                autotune_state.pilot_override = true;
                // set gains to their original values
                autotune_load_orig_gains();
                attitude_control.limit_angle_to_rate_request(true);
            }
            // reset pilot override time
            autotune_override_time = millis();
        }else if (autotune_state.pilot_override) {
            // check if we should resume tuning after pilot's override
            if (millis() - autotune_override_time > AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS) {
                autotune_state.pilot_override = false;             // turn off pilot override
                // set gains to their intra-test values (which are very close to the original gains)
                // autotune_load_intra_test_gains(); //I think we should be keeping the originals here to let the I term settle quickly
                autotune_state.step = AUTOTUNE_STEP_WAITING_FOR_LEVEL; // set tuning step back from beginning
                autotune_desired_yaw = ahrs.yaw_sensor;
            }
        }

        // if pilot override call attitude controller
        if (autotune_state.pilot_override || autotune_state.mode != AUTOTUNE_MODE_TUNING) {
            attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        }else{
            // somehow get attitude requests from autotuning
            autotune_attitude_control();
        }

        // call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
    }
}

// autotune_attitude_controller - sets attitude control targets during tuning
static void autotune_attitude_control()
{
    float rotation_rate = 0.0f;        // rotation rate in radians/second
    float lean_angle = 0.0f;
    const float direction_sign = autotune_state.positive_direction ? 1.0 : -1.0;

    // check tuning step
    switch (autotune_state.step) {

    case AUTOTUNE_STEP_WAITING_FOR_LEVEL:
        // Note: we should be using intra-test gains (which are very close to the original gains but have lower I)
        // re-enable rate limits
        attitude_control.limit_angle_to_rate_request(true);

        // hold level attitude
        attitude_control.angle_ef_roll_pitch_yaw( 0.0f, 0.0f, autotune_desired_yaw, true);

        // hold the copter level for 0.5 seconds before we begin a twitch
        // reset counter if we are no longer level
        if ((labs(ahrs.roll_sensor) > AUTOTUNE_LEVEL_ANGLE_CD) ||
                (labs(ahrs.pitch_sensor) > AUTOTUNE_LEVEL_ANGLE_CD) ||
                (labs(wrap_180_cd(ahrs.yaw_sensor-(int32_t)autotune_desired_yaw)) > AUTOTUNE_LEVEL_ANGLE_CD) ||
                ((ToDeg(ahrs.get_gyro().x) * 100.0f) > AUTOTUNE_LEVEL_RATE_RP_CD) ||
                ((ToDeg(ahrs.get_gyro().y) * 100.0f) > AUTOTUNE_LEVEL_RATE_RP_CD) ||
                ((ToDeg(ahrs.get_gyro().z) * 100.0f) > AUTOTUNE_LEVEL_RATE_Y_CD) ) {
            autotune_step_start_time = millis();
        }

        // if we have been level for a sufficient amount of time (0.5 seconds) move onto tuning step
        if (millis() - autotune_step_start_time >= AUTOTUNE_REQUIRED_LEVEL_TIME_MS) {
            // initiate variables for next step
            autotune_state.step = AUTOTUNE_STEP_TWITCHING;
            autotune_step_start_time = millis();
            autotune_step_stop_time = autotune_step_start_time + AUTOTUNE_TESTING_STEP_TIMEOUT_MS;
            autotune_test_max = 0.0f;
            autotune_test_min = 0.0f;
            rotation_rate_filt.reset(0.0f);
            rate_max = 0.0f;
            // set gains to their to-be-tested values
            autotune_load_twitch_gains();
        }

        switch (autotune_state.axis) {
        case AUTOTUNE_AXIS_ROLL:
            autotune_target_rate = constrain_float(attitude_control.max_rate_step_bf_roll(), AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
            autotune_target_angle = constrain_float(attitude_control.max_angle_step_bf_roll(), AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD, AUTOTUNE_TARGET_ANGLE_RLLPIT_CD);
            autotune_start_rate = ToDeg(ahrs.get_gyro().x) * 100.0f;
            autotune_start_angle = ahrs.roll_sensor;
            rotation_rate_filt.set_cutoff_frequency(g.pid_rate_roll.filt_hz()*2.0f);
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate_filt.reset(autotune_start_rate);
            } else {
                rotation_rate_filt.reset(0);
            }
        break;
        case AUTOTUNE_AXIS_PITCH:
            autotune_target_rate = constrain_float(attitude_control.max_rate_step_bf_pitch(), AUTOTUNE_TARGET_MIN_RATE_RLLPIT_CDS, AUTOTUNE_TARGET_RATE_RLLPIT_CDS);
            autotune_target_angle = constrain_float(attitude_control.max_angle_step_bf_pitch(), AUTOTUNE_TARGET_MIN_ANGLE_RLLPIT_CD, AUTOTUNE_TARGET_ANGLE_RLLPIT_CD);
            autotune_start_rate = ToDeg(ahrs.get_gyro().y) * 100.0f;
            autotune_start_angle = ahrs.pitch_sensor;
            rotation_rate_filt.set_cutoff_frequency(g.pid_rate_pitch.filt_hz()*2.0f);
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate_filt.reset(autotune_start_rate);
            } else {
                rotation_rate_filt.reset(0);
            }
            break;
        case AUTOTUNE_AXIS_YAW:
            autotune_target_rate = constrain_float(attitude_control.max_rate_step_bf_yaw()/1.5f, AUTOTUNE_TARGET_MIN_RATE_YAW_CDS, AUTOTUNE_TARGET_RATE_YAW_CDS);
            autotune_target_angle = constrain_float(attitude_control.max_angle_step_bf_yaw(), AUTOTUNE_TARGET_MIN_ANGLE_YAW_CD, AUTOTUNE_TARGET_ANGLE_YAW_CD);
            autotune_start_rate = ToDeg(ahrs.get_gyro().z) * 100.0f;
            autotune_start_angle = ahrs.yaw_sensor;
            rotation_rate_filt.set_cutoff_frequency(AUTOTUNE_Y_FILT_FREQ);
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate_filt.reset(autotune_start_rate);
            } else {
                rotation_rate_filt.reset(0);
            }
            break;
        }
        break;

    case AUTOTUNE_STEP_TWITCHING:
        // Run the twitching step
        // Note: we should be using intra-test gains (which are very close to the original gains but have lower I)

        // disable rate limits
        attitude_control.limit_angle_to_rate_request(false);

        if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
            // Testing increasing stabilize P gain so will set lean angle target
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                // request roll to 20deg
                attitude_control.angle_ef_roll_pitch_rate_ef_yaw( direction_sign * autotune_target_angle + autotune_start_angle, 0.0f, 0.0f);
                break;
            case AUTOTUNE_AXIS_PITCH:
                // request pitch to 20deg
                attitude_control.angle_ef_roll_pitch_rate_ef_yaw( 0.0f, direction_sign * autotune_target_angle + autotune_start_angle, 0.0f);
                break;
            case AUTOTUNE_AXIS_YAW:
                // request pitch to 20deg
                attitude_control.angle_ef_roll_pitch_yaw( 0.0f, 0.0f, wrap_180_cd_float(direction_sign * autotune_target_angle + autotune_start_angle), false);
                break;
            }
        } else {
            // Testing rate P and D gains so will set body-frame rate targets.
            // Rate controller will use existing body-frame rates and convert to motor outputs
            // for all axes except the one we override here.
            attitude_control.angle_ef_roll_pitch_rate_ef_yaw( 0.0f, 0.0f, 0.0f);
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                // override body-frame roll rate
                attitude_control.rate_bf_roll_target(direction_sign * autotune_target_rate + autotune_start_rate);
                break;
            case AUTOTUNE_AXIS_PITCH:
                // override body-frame pitch rate
                attitude_control.rate_bf_pitch_target(direction_sign * autotune_target_rate + autotune_start_rate);
                break;
            case AUTOTUNE_AXIS_YAW:
                // override body-frame yaw rate
                attitude_control.rate_bf_yaw_target(direction_sign * autotune_target_rate + autotune_start_rate);
                break;
            }
        }

        // capture this iterations rotation rate and lean angle
        // Add filter to measurements
        switch (autotune_state.axis) {
        case AUTOTUNE_AXIS_ROLL:
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().x) * 100.0f), MAIN_LOOP_SECONDS);
            } else {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().x) * 100.0f - autotune_start_rate), MAIN_LOOP_SECONDS);
            }
            lean_angle = direction_sign * (ahrs.roll_sensor - (int32_t)autotune_start_angle);
            break;
        case AUTOTUNE_AXIS_PITCH:
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().y) * 100.0f), MAIN_LOOP_SECONDS);
            } else {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().y) * 100.0f - autotune_start_rate), MAIN_LOOP_SECONDS);
            }
            lean_angle = direction_sign * (ahrs.pitch_sensor - (int32_t)autotune_start_angle);
            break;
        case AUTOTUNE_AXIS_YAW:
            if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().z) * 100.0f), MAIN_LOOP_SECONDS);
            } else {
                rotation_rate = rotation_rate_filt.apply(direction_sign * (ToDeg(ahrs.get_gyro().z) * 100.0f - autotune_start_rate), MAIN_LOOP_SECONDS);
            }
            lean_angle = direction_sign * wrap_180_cd(ahrs.yaw_sensor-(int32_t)autotune_start_angle);
            break;
        }

        switch (autotune_state.tune_type) {
        case AUTOTUNE_TYPE_RD_UP:
        case AUTOTUNE_TYPE_RD_DOWN:
            autotune_twitching_test(rotation_rate, autotune_target_rate, autotune_test_min, autotune_test_max);
            autotune_twitching_measure_acceleration(autotune_test_accel_max, rotation_rate, rate_max);
            if (lean_angle >= autotune_target_angle) {
                autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
            }
            break;
        case AUTOTUNE_TYPE_RP_UP:
            autotune_twitching_test(rotation_rate, autotune_target_rate, autotune_test_min, autotune_test_max);
            autotune_twitching_measure_acceleration(autotune_test_accel_max, rotation_rate, rate_max);
            if (lean_angle >= autotune_target_angle) {
                autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
            }
            break;
        case AUTOTUNE_TYPE_SP_DOWN:
        case AUTOTUNE_TYPE_SP_UP:
            autotune_twitching_test(lean_angle, autotune_target_angle, autotune_test_min, autotune_test_max);
            autotune_twitching_measure_acceleration(autotune_test_accel_max, rotation_rate - direction_sign * autotune_start_rate, rate_max);
            break;
        }

        // log this iterations lean angle and rotation rate
        Log_Write_AutoTuneDetails(lean_angle, rotation_rate);
        Log_Write_Rate();
        break;

    case AUTOTUNE_STEP_UPDATE_GAINS:

        // re-enable rate limits
        attitude_control.limit_angle_to_rate_request(true);

        // log the latest gains
        if ((autotune_state.tune_type == AUTOTUNE_TYPE_SP_DOWN) || (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)) {
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_angle, autotune_test_min, autotune_test_max, tune_roll_rp, tune_roll_rd, tune_roll_sp);
                break;
            case AUTOTUNE_AXIS_PITCH:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_angle, autotune_test_min, autotune_test_max, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp);
                break;
            case AUTOTUNE_AXIS_YAW:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_angle, autotune_test_min, autotune_test_max, tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp);
                break;
            }
        } else {
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_rate, autotune_test_min, autotune_test_max, tune_roll_rp, tune_roll_rd, tune_roll_sp);
                break;
            case AUTOTUNE_AXIS_PITCH:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_rate, autotune_test_min, autotune_test_max, tune_pitch_rp, tune_pitch_rd, tune_pitch_sp);
                break;
            case AUTOTUNE_AXIS_YAW:
                Log_Write_AutoTune(autotune_state.axis, autotune_state.tune_type, autotune_target_rate, autotune_test_min, autotune_test_max, tune_yaw_rp, tune_yaw_rLPF, tune_yaw_sp);
                break;
            }
        }

        // Check results after mini-step to increase rate D gain
        switch (autotune_state.tune_type) {
        case AUTOTUNE_TYPE_RD_UP:
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                autotune_updating_d_up(tune_roll_rd, AUTOTUNE_RD_MIN, AUTOTUNE_RD_MAX, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_PITCH:
                autotune_updating_d_up(tune_pitch_rd, AUTOTUNE_RD_MIN, AUTOTUNE_RD_MAX, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_YAW:
                autotune_updating_d_up(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RLPF_MAX, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            }
            break;
        // Check results after mini-step to decrease rate D gain
        case AUTOTUNE_TYPE_RD_DOWN:
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                autotune_updating_d_down(tune_roll_rd, AUTOTUNE_RD_MIN, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_PITCH:
                autotune_updating_d_down(tune_pitch_rd, AUTOTUNE_RD_MIN, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_YAW:
                autotune_updating_d_down(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            }
            break;
        // Check results after mini-step to increase rate P gain
        case AUTOTUNE_TYPE_RP_UP:
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                autotune_updating_p_up_d_down(tune_roll_rd, AUTOTUNE_RD_MIN, AUTOTUNE_RD_STEP, tune_roll_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_PITCH:
                autotune_updating_p_up_d_down(tune_pitch_rd, AUTOTUNE_RD_MIN, AUTOTUNE_RD_STEP, tune_pitch_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_YAW:
                autotune_updating_p_up_d_down(tune_yaw_rLPF, AUTOTUNE_RLPF_MIN, AUTOTUNE_RD_STEP, tune_yaw_rp, AUTOTUNE_RP_MIN, AUTOTUNE_RP_MAX, AUTOTUNE_RP_STEP, autotune_target_rate, autotune_test_min, autotune_test_max);
                break;
            }
            break;
        // Check results after mini-step to increase stabilize P gain
        case AUTOTUNE_TYPE_SP_DOWN:
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                autotune_updating_p_down(tune_roll_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_PITCH:
                autotune_updating_p_down(tune_pitch_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_YAW:
                autotune_updating_p_down(tune_yaw_sp, AUTOTUNE_SP_MIN, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            }
            break;
        // Check results after mini-step to increase stabilize P gain
        case AUTOTUNE_TYPE_SP_UP:
            switch (autotune_state.axis) {
            case AUTOTUNE_AXIS_ROLL:
                autotune_updating_p_up(tune_roll_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_PITCH:
                autotune_updating_p_up(tune_pitch_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            case AUTOTUNE_AXIS_YAW:
                autotune_updating_p_up(tune_yaw_sp, AUTOTUNE_SP_MAX, AUTOTUNE_SP_STEP, autotune_target_angle, autotune_test_max);
                break;
            }
            break;
        }

        // we've complete this step, finalize pids and move to next step
        if (autotune_counter >= AUTOTUNE_SUCCESS_COUNT) {

            // reset counter
            autotune_counter = 0;

            // move to the next tuning type
            switch (autotune_state.tune_type) {
            case AUTOTUNE_TYPE_RD_UP:
                autotune_state.tune_type++;
                break;
            case AUTOTUNE_TYPE_RD_DOWN:
                autotune_state.tune_type++;
                switch (autotune_state.axis) {
                case AUTOTUNE_AXIS_ROLL:
                    tune_roll_rd = max(AUTOTUNE_RD_MIN, tune_roll_rd * (1.0f-AUTOTUNE_RD_BACKOFF*max(0.0f,(g.autotune_aggressiveness-AUTOTUNE_RD_TEST_NOISE))));
                    tune_roll_rp = max(AUTOTUNE_RP_MIN, tune_roll_rp * (1.0f-AUTOTUNE_RD_BACKOFF*max(0.0f,(g.autotune_aggressiveness-AUTOTUNE_RD_TEST_NOISE))));
                    break;
                case AUTOTUNE_AXIS_PITCH:
                    tune_pitch_rd = max(AUTOTUNE_RD_MIN, tune_pitch_rd * (1.0f-AUTOTUNE_RD_BACKOFF*max(0.0f,(g.autotune_aggressiveness-AUTOTUNE_RD_TEST_NOISE))));
                    tune_pitch_rp = max(AUTOTUNE_RP_MIN, tune_pitch_rp * (1.0f-AUTOTUNE_RD_BACKOFF*max(0.0f,(g.autotune_aggressiveness-AUTOTUNE_RD_TEST_NOISE))));
                    break;
                case AUTOTUNE_AXIS_YAW:
                    tune_yaw_rLPF = max(AUTOTUNE_RLPF_MIN, tune_yaw_rLPF * (1.0f-AUTOTUNE_RD_BACKOFF*max(0.0f,(g.autotune_aggressiveness-AUTOTUNE_RD_TEST_NOISE))));
                    tune_yaw_rp = max(AUTOTUNE_RP_MIN, tune_yaw_rp * (1.0f-AUTOTUNE_RD_BACKOFF*max(0.0f,(g.autotune_aggressiveness-AUTOTUNE_RD_TEST_NOISE))));
                    break;
                }
                break;
            case AUTOTUNE_TYPE_RP_UP:
                autotune_state.tune_type++;
                switch (autotune_state.axis) {
                case AUTOTUNE_AXIS_ROLL:
                    tune_roll_rp = max(AUTOTUNE_RP_MIN, tune_roll_rp * AUTOTUNE_RP_BACKOFF);
                    break;
                case AUTOTUNE_AXIS_PITCH:
                    tune_pitch_rp = max(AUTOTUNE_RP_MIN, tune_pitch_rp * AUTOTUNE_RP_BACKOFF);
                    break;
                case AUTOTUNE_AXIS_YAW:
                    tune_yaw_rp = max(AUTOTUNE_RP_MIN, tune_yaw_rp * AUTOTUNE_RP_BACKOFF);
                    break;
                }
                break;
            case AUTOTUNE_TYPE_SP_DOWN:
                autotune_state.tune_type++;
                break;
            case AUTOTUNE_TYPE_SP_UP:
                // we've reached the end of a D-up-down PI-up-down tune type cycle
                autotune_state.tune_type = AUTOTUNE_TYPE_RD_UP;

                // advance to the next axis
                bool autotune_complete = false;
                switch (autotune_state.axis) {
                case AUTOTUNE_AXIS_ROLL:
                    tune_roll_sp = max(AUTOTUNE_SP_MIN, tune_roll_sp * AUTOTUNE_SP_BACKOFF);
                    tune_roll_accel = max(AUTOTUNE_RP_ACCEL_MIN, autotune_test_accel_max * AUTOTUNE_ACCEL_RP_BACKOFF);
                    if (autotune_pitch_enabled()) {
                        autotune_state.axis = AUTOTUNE_AXIS_PITCH;
                    } else if (autotune_yaw_enabled()) {
                        autotune_state.axis = AUTOTUNE_AXIS_YAW;
                    } else {
                        autotune_complete = true;
                    }
                    break;
                case AUTOTUNE_AXIS_PITCH:
                    tune_pitch_sp = max(AUTOTUNE_SP_MIN, tune_pitch_sp * AUTOTUNE_SP_BACKOFF);
                    tune_pitch_accel = max(AUTOTUNE_RP_ACCEL_MIN, autotune_test_accel_max * AUTOTUNE_ACCEL_RP_BACKOFF);
                    if (autotune_yaw_enabled()) {
                        autotune_state.axis = AUTOTUNE_AXIS_YAW;
                    } else {
                        autotune_complete = true;
                    }
                    break;
                case AUTOTUNE_AXIS_YAW:
                    tune_yaw_sp = max(AUTOTUNE_SP_MIN, tune_yaw_sp * AUTOTUNE_SP_BACKOFF);
                    tune_yaw_accel = max(AUTOTUNE_Y_ACCEL_MIN, autotune_test_accel_max * AUTOTUNE_ACCEL_Y_BACKOFF);
                    autotune_complete = true;
                    break;
                }

                // if we've just completed all axes we have successfully completed the autotune
                    // change to TESTING mode to allow user to fly with new gains
                if (autotune_complete) {
                    autotune_state.mode = AUTOTUNE_MODE_SUCCESS;
                    autotune_update_gcs(AUTOTUNE_MESSAGE_SUCCESS);
                    Log_Write_Event(DATA_AUTOTUNE_SUCCESS);
                    AP_Notify::events.autotune_complete = 1;
                } else {
                    AP_Notify::events.autotune_next_axis = 1;
                }
                break;
            }
        }

        // reverse direction
        autotune_state.positive_direction = !autotune_state.positive_direction;

        if (autotune_state.axis == AUTOTUNE_AXIS_YAW) {
            attitude_control.angle_ef_roll_pitch_yaw( 0.0f, 0.0f, ahrs.yaw_sensor, false);
        }

        // set gains to their intra-test values (which are very close to the original gains)
        autotune_load_intra_test_gains();

        // reset testing step
        autotune_state.step = AUTOTUNE_STEP_WAITING_FOR_LEVEL;
        autotune_step_start_time = millis();
        break;
    }
}

// autotune_backup_gains_and_initialise - store current gains as originals
//  called before tuning starts to backup original gains
static void autotune_backup_gains_and_initialise()
{
    // initialise state because this is our first time
    if (autotune_roll_enabled()) {
        autotune_state.axis = AUTOTUNE_AXIS_ROLL;
    } else if (autotune_pitch_enabled()) {
        autotune_state.axis = AUTOTUNE_AXIS_PITCH;
    } else if (autotune_yaw_enabled()) {
        autotune_state.axis = AUTOTUNE_AXIS_YAW;
    }
    autotune_state.positive_direction = false;
    autotune_state.step = AUTOTUNE_STEP_WAITING_FOR_LEVEL;
    autotune_step_start_time = millis();
    autotune_state.tune_type = AUTOTUNE_TYPE_RD_UP;

    autotune_desired_yaw = ahrs.yaw_sensor;

    g.autotune_aggressiveness = constrain_float(g.autotune_aggressiveness, 0.05, 0.1);

    // backup original pids and initialise tuned pid values
    if (autotune_roll_enabled()) {
        orig_roll_rp = g.pid_rate_roll.kP();
        orig_roll_ri = g.pid_rate_roll.kI();
        orig_roll_rd = g.pid_rate_roll.kD();
        orig_roll_sp = g.p_stabilize_roll.kP();
        tune_roll_rp = g.pid_rate_roll.kP();
        tune_roll_rd = g.pid_rate_roll.kD();
        tune_roll_sp = g.p_stabilize_roll.kP();
    }
    if (autotune_pitch_enabled()) {
        orig_pitch_rp = g.pid_rate_pitch.kP();
        orig_pitch_ri = g.pid_rate_pitch.kI();
        orig_pitch_rd = g.pid_rate_pitch.kD();
        orig_pitch_sp = g.p_stabilize_pitch.kP();
        tune_pitch_rp = g.pid_rate_pitch.kP();
        tune_pitch_rd = g.pid_rate_pitch.kD();
        tune_pitch_sp = g.p_stabilize_pitch.kP();
    }
    if (autotune_yaw_enabled()) {
        orig_yaw_rp = g.pid_rate_yaw.kP();
        orig_yaw_ri = g.pid_rate_yaw.kI();
        orig_yaw_rd = g.pid_rate_yaw.kD();
        orig_yaw_rLPF = g.pid_rate_yaw.filt_hz();
        orig_yaw_sp = g.p_stabilize_yaw.kP();
        tune_yaw_rp = g.pid_rate_yaw.kP();
        tune_yaw_rLPF = g.pid_rate_yaw.filt_hz();
        tune_yaw_sp = g.p_stabilize_yaw.kP();
    }

    Log_Write_Event(DATA_AUTOTUNE_INITIALISED);
}

// autotune_load_orig_gains - set gains to their original values
//  called by autotune_stop and autotune_failed functions
static void autotune_load_orig_gains()
{
    // sanity check the gains
    bool failed = false;
    if (autotune_roll_enabled()) {
        if ((orig_roll_rp != 0) || (orig_roll_sp != 0)) {
            g.pid_rate_roll.kP(orig_roll_rp);
            g.pid_rate_roll.kI(orig_roll_ri);
            g.pid_rate_roll.kD(orig_roll_rd);
            g.p_stabilize_roll.kP(orig_roll_sp);
        } else {
            failed = true;
        }
    }
    if (autotune_pitch_enabled()) {
        if ((orig_pitch_rp != 0) || (orig_pitch_sp != 0)) {
            g.pid_rate_pitch.kP(orig_pitch_rp);
            g.pid_rate_pitch.kI(orig_pitch_ri);
            g.pid_rate_pitch.kD(orig_pitch_rd);
            g.p_stabilize_pitch.kP(orig_pitch_sp);
        } else {
            failed = true;
        }
    }
    if (autotune_yaw_enabled()) {
        if ((orig_yaw_rp != 0) || (orig_yaw_sp != 0) || (orig_yaw_rLPF != 0)) {
            g.pid_rate_yaw.kP(orig_yaw_rp);
            g.pid_rate_yaw.kI(orig_yaw_ri);
            g.pid_rate_yaw.kD(orig_yaw_rd);
            g.pid_rate_yaw.filt_hz(orig_yaw_rLPF);
            g.p_stabilize_yaw.kP(orig_yaw_sp);
        } else {
            failed = true;
        }
    }
    if (failed) {
        // log an error message and fail the autotune
        Log_Write_Error(ERROR_SUBSYSTEM_AUTOTUNE,ERROR_CODE_AUTOTUNE_BAD_GAINS);
    }
}

// autotune_load_tuned_gains - load tuned gains
static void autotune_load_tuned_gains()
{
    // sanity check the gains
    bool failed = true;
    if (autotune_roll_enabled()) {
        if (tune_roll_rp != 0) {
            g.pid_rate_roll.kP(tune_roll_rp);
            g.pid_rate_roll.kI(tune_roll_rp*AUTOTUNE_PI_RATIO_FINAL);
            g.pid_rate_roll.kD(tune_roll_rd);
            g.p_stabilize_roll.kP(tune_roll_sp);
            failed = false;
        }
    }
    if (autotune_pitch_enabled()) {
        if (tune_pitch_rp != 0) {
            g.pid_rate_pitch.kP(tune_pitch_rp);
            g.pid_rate_pitch.kI(tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL);
            g.pid_rate_pitch.kD(tune_pitch_rd);
            g.p_stabilize_pitch.kP(tune_pitch_sp);
            failed = false;
        }
    }
    if (autotune_yaw_enabled()) {
        if (tune_yaw_rp != 0) {
            g.pid_rate_yaw.kP(tune_yaw_rp);
            g.pid_rate_yaw.kI(tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL);
            g.pid_rate_yaw.kD(0.0f);
            g.pid_rate_yaw.filt_hz(tune_yaw_rLPF);
            g.p_stabilize_yaw.kP(tune_yaw_sp);
            failed = false;
        }
    }
    if (failed) {
        // log an error message and fail the autotune
        Log_Write_Error(ERROR_SUBSYSTEM_AUTOTUNE,ERROR_CODE_AUTOTUNE_BAD_GAINS);
    }
}

// autotune_load_intra_test_gains - gains used between tests
//  called during testing mode's update-gains step to set gains ahead of return-to-level step
static void autotune_load_intra_test_gains()
{
    // we are restarting tuning so reset gains to tuning-start gains (i.e. low I term)
    // sanity check the gains
    bool failed = true;
    if (autotune_roll_enabled() && (orig_roll_rp != 0)) {
        g.pid_rate_roll.kP(orig_roll_rp);
        g.pid_rate_roll.kI(orig_roll_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        g.pid_rate_roll.kD(orig_roll_rd);
        g.p_stabilize_roll.kP(orig_roll_sp);
        failed = false;
    }
    if (autotune_pitch_enabled() && (orig_pitch_rp != 0)) {
        g.pid_rate_pitch.kP(orig_pitch_rp);
        g.pid_rate_pitch.kI(orig_pitch_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        g.pid_rate_pitch.kD(orig_pitch_rd);
        g.p_stabilize_pitch.kP(orig_pitch_sp);
        failed = false;
    }
    if (autotune_yaw_enabled() && (orig_yaw_rp != 0)) {
        g.pid_rate_yaw.kP(orig_yaw_rp);
        g.pid_rate_yaw.kI(orig_yaw_rp*AUTOTUNE_PI_RATIO_FOR_TESTING);
        g.pid_rate_yaw.kD(orig_yaw_rd);
        g.pid_rate_yaw.filt_hz(orig_yaw_rLPF);
        g.p_stabilize_yaw.kP(orig_yaw_sp);
        failed = false;
    }
    if (failed) {
        // log an error message and fail the autotune
        Log_Write_Error(ERROR_SUBSYSTEM_AUTOTUNE,ERROR_CODE_AUTOTUNE_BAD_GAINS);
    }
}

// autotune_load_twitch_gains - load the to-be-tested gains for a single axis
// called by autotune_attitude_control() just before it beings testing a gain (i.e. just before it twitches)
static void autotune_load_twitch_gains()
{
    bool failed = true;
    switch (autotune_state.axis) {
        case AUTOTUNE_AXIS_ROLL:
            if (tune_roll_rp != 0) {
                g.pid_rate_roll.kP(tune_roll_rp);
                g.pid_rate_roll.kI(tune_roll_rp*0.01f);
                g.pid_rate_roll.kD(tune_roll_rd);
                g.p_stabilize_roll.kP(tune_roll_sp);
                failed = false;
            }
            break;
        case AUTOTUNE_AXIS_PITCH:
            if (tune_pitch_rp != 0) {
                g.pid_rate_pitch.kP(tune_pitch_rp);
                g.pid_rate_pitch.kI(tune_pitch_rp*0.01f);
                g.pid_rate_pitch.kD(tune_pitch_rd);
                g.p_stabilize_pitch.kP(tune_pitch_sp);
                failed = false;
            }
            break;
        case AUTOTUNE_AXIS_YAW:
            if (tune_yaw_rp != 0) {
                g.pid_rate_yaw.kP(tune_yaw_rp);
                g.pid_rate_yaw.kI(tune_yaw_rp*0.01f);
                g.pid_rate_yaw.kD(0.0f);
                g.pid_rate_yaw.filt_hz(tune_yaw_rLPF);
                g.p_stabilize_yaw.kP(tune_yaw_sp);
                failed = false;
            }
            break;
    }
    if (failed) {
        // log an error message and fail the autotune
        Log_Write_Error(ERROR_SUBSYSTEM_AUTOTUNE,ERROR_CODE_AUTOTUNE_BAD_GAINS);
    }
}

// autotune_save_tuning_gains - save the final tuned gains for each axis
// save discovered gains to eeprom if autotuner is enabled (i.e. switch is in the high position)
static void autotune_save_tuning_gains()
{
    // if we successfully completed tuning
    if (autotune_state.mode == AUTOTUNE_MODE_SUCCESS) {
        // sanity check the rate P values
        if (autotune_roll_enabled() && (tune_roll_rp != 0)) {
            // rate roll gains
            g.pid_rate_roll.kP(tune_roll_rp);
            g.pid_rate_roll.kI(tune_roll_rp*AUTOTUNE_PI_RATIO_FINAL);
            g.pid_rate_roll.kD(tune_roll_rd);
            g.pid_rate_roll.save_gains();
            // stabilize roll
            g.p_stabilize_roll.kP(tune_roll_sp);
            g.p_stabilize_roll.save_gains();

            if (attitude_control.get_bf_feedforward()) {
                attitude_control.save_accel_roll_max(tune_roll_accel);
            }

            // resave pids to originals in case the autotune is run again
            orig_roll_rp = g.pid_rate_roll.kP();
            orig_roll_ri = g.pid_rate_roll.kI();
            orig_roll_rd = g.pid_rate_roll.kD();
            orig_roll_sp = g.p_stabilize_roll.kP();
        }

        if (autotune_pitch_enabled() && (tune_pitch_rp != 0)) {
            // rate pitch gains
            g.pid_rate_pitch.kP(tune_pitch_rp);
            g.pid_rate_pitch.kI(tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL);
            g.pid_rate_pitch.kD(tune_pitch_rd);
            g.pid_rate_pitch.save_gains();
            // stabilize pitch
            g.p_stabilize_pitch.kP(tune_pitch_sp);
            g.p_stabilize_pitch.save_gains();

            if (attitude_control.get_bf_feedforward()) {
                attitude_control.save_accel_pitch_max(tune_pitch_accel);
            }

            // resave pids to originals in case the autotune is run again
            orig_pitch_rp = g.pid_rate_pitch.kP();
            orig_pitch_ri = g.pid_rate_pitch.kI();
            orig_pitch_rd = g.pid_rate_pitch.kD();
            orig_pitch_sp = g.p_stabilize_pitch.kP();
        }

        if (autotune_yaw_enabled() && (tune_yaw_rp != 0)) {
            // rate yaw gains
            g.pid_rate_yaw.kP(tune_yaw_rp);
            g.pid_rate_yaw.kI(tune_yaw_rp*AUTOTUNE_YAW_PI_RATIO_FINAL);
            g.pid_rate_yaw.kD(0.0f);
            g.pid_rate_yaw.filt_hz(tune_yaw_rLPF);
            g.pid_rate_yaw.save_gains();
            // stabilize yaw
            g.p_stabilize_yaw.kP(tune_yaw_sp);
            g.p_stabilize_yaw.save_gains();

            if (attitude_control.get_bf_feedforward()) {
                attitude_control.save_accel_yaw_max(tune_yaw_accel);
            }

            // resave pids to originals in case the autotune is run again
            orig_yaw_rp = g.pid_rate_yaw.kP();
            orig_yaw_ri = g.pid_rate_yaw.kI();
            orig_yaw_rd = g.pid_rate_yaw.kD();
            orig_yaw_rLPF = g.pid_rate_yaw.filt_hz();
            orig_yaw_sp = g.p_stabilize_yaw.kP();
        }
        // update GCS and log save gains event
        autotune_update_gcs(AUTOTUNE_MESSAGE_SAVED_GAINS);
        Log_Write_Event(DATA_AUTOTUNE_SAVEDGAINS);
        // reset Autotune so that gains are not saved again and autotune can be run again.
        autotune_state.mode = AUTOTUNE_MODE_UNINITIALISED;
    }
}

// autotune_update_gcs - send message to ground station
void autotune_update_gcs(uint8_t message_id)
{
    switch (message_id) {
        case AUTOTUNE_MESSAGE_STARTED:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Started"));
            break;
        case AUTOTUNE_MESSAGE_STOPPED:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Stopped"));
            break;
        case AUTOTUNE_MESSAGE_SUCCESS:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Success"));
            break;
        case AUTOTUNE_MESSAGE_FAILED:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Failed"));
            break;
        case AUTOTUNE_MESSAGE_SAVED_GAINS:
            gcs_send_text_P(SEVERITY_HIGH,PSTR("AutoTune: Saved Gains"));
            break;
    }
}

// axis helper functions
inline bool autotune_roll_enabled() {
    return g.autotune_axis_bitmask & AUTOTUNE_AXIS_BITMASK_ROLL;
}

inline bool autotune_pitch_enabled() {
    return g.autotune_axis_bitmask & AUTOTUNE_AXIS_BITMASK_PITCH;
}

inline bool autotune_yaw_enabled() {
    return g.autotune_axis_bitmask & AUTOTUNE_AXIS_BITMASK_YAW;
}

// autotune_twitching_test - twitching tests
// update min and max and test for end conditions
void autotune_twitching_test(float measurement, float target, float &measurement_min, float &measurement_max)
{
    // capture maximum measurement
    if (measurement > measurement_max) {
        // the measurement is continuing to increase without stopping
        measurement_max = measurement;
        measurement_min = measurement;
    }

    // capture minimum measurement after the measurement has peaked (aka "bounce back")
    if ((measurement < measurement_min) && (measurement_max > target * 0.5f)) {
        // the measurement is bouncing back
        measurement_min = measurement;
    }

    // calculate early stopping time based on the time it takes to get to 90%
    if (measurement_max < target * 0.9f) {
        // the measurement not reached the 90% threshold yet
        autotune_step_stop_time = autotune_step_start_time + (millis() - autotune_step_start_time) * 3.0f;
        autotune_step_stop_time = min(autotune_step_stop_time, autotune_step_start_time + AUTOTUNE_TESTING_STEP_TIMEOUT_MS);
    }

    if (measurement_max > target) {
        // the measurement has passed the target
        autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
    }

    if (measurement_max-measurement_min > measurement_max*g.autotune_aggressiveness) {
        // the measurement has passed 50% of the target and bounce back is larger than the threshold
        autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
    }

    if (millis() >= autotune_step_stop_time) {
        // we have passed the maximum stop time
        autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
    }
}

// autotune_updating_d_up - increase D and adjust P to optimize the D term for a little bounce back
// optimize D term while keeping the maximum just below the target by adjusting P
void autotune_updating_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max)
{
    if (measurement_max > target) {
        // if maximum measurement was higher than target
        // reduce P gain (which should reduce maximum)
        tune_p -= tune_p*tune_p_step_ratio;
        if (tune_p < tune_p_min) {
            // P gain is at minimum so start reducing D
            tune_p = tune_p_min;
            tune_d -= tune_d*tune_d_step_ratio;
            if (tune_d <= tune_d_min) {
                // We have reached minimum D gain so stop tuning
                tune_d = tune_d_min;
                autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        }
    }else if ((measurement_max < target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (tune_p <= tune_p_max)) {
        // we have not achieved a high enough maximum to get a good measurement of bounce back.
        // increase P gain (which should increase maximum)
        tune_p += tune_p*tune_p_step_ratio;
        if (tune_p >= tune_p_max) {
            tune_p = tune_p_max;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
    }else{
        // we have a good measurement of bounce back
        if (measurement_max-measurement_min > measurement_max*g.autotune_aggressiveness) {
            // ignore the next result unless it is the same as this one
            autotune_state.ignore_next = 1;
            // bounce back is bigger than our threshold so increment the success counter
            autotune_counter++;
        }else{
            if (autotune_state.ignore_next == 0){
                // bounce back is smaller than our threshold so decrement the success counter
                if (autotune_counter > 0 ) {
                    autotune_counter--;
                }
                // increase D gain (which should increase bounce back)
                tune_d += tune_d*tune_d_step_ratio*2.0f;
                // stop tuning if we hit maximum D
                if (tune_d >= tune_d_max) {
                    tune_d = tune_d_max;
                    autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                    Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
                }
            } else {
                autotune_state.ignore_next = 0;
            }
        }
    }
}

// autotune_updating_d_down - decrease D and adjust P to optimize the D term for no bounce back
// optimize D term while keeping the maximum just below the target by adjusting P
void autotune_updating_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max)
{
    if (measurement_max > target) {
        // if maximum measurement was higher than target
        // reduce P gain (which should reduce maximum)
        tune_p -= tune_p*tune_p_step_ratio;
        if (tune_p < tune_p_min) {
            // P gain is at minimum so start reducing D gain
            tune_p = tune_p_min;
            tune_d -= tune_d*tune_d_step_ratio;
            if (tune_d <= tune_d_min) {
                // We have reached minimum D so stop tuning
                tune_d = tune_d_min;
                autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        }
    }else if ((measurement_max < target*(1.0f-AUTOTUNE_D_UP_DOWN_MARGIN)) && (tune_p <= tune_p_max)) {
        // we have not achieved a high enough maximum to get a good measurement of bounce back.
        // increase P gain (which should increase maximum)
        tune_p += tune_p*tune_p_step_ratio;
        if (tune_p >= tune_p_max) {
            tune_p = tune_p_max;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
    }else{
        // we have a good measurement of bounce back
        if (measurement_max-measurement_min < measurement_max*g.autotune_aggressiveness) {
            if (autotune_state.ignore_next == 0){
                // bounce back is less than our threshold so increment the success counter
                autotune_counter++;
            } else {
                autotune_state.ignore_next = 0;
            }
        }else{
            // ignore the next result unless it is the same as this one
            autotune_state.ignore_next = 1;
            // bounce back is larger than our threshold so decrement the success counter
            if (autotune_counter > 0 ) {
                autotune_counter--;
            }
            // decrease D gain (which should decrease bounce back)
            tune_d -= tune_d*tune_d_step_ratio;
            // stop tuning if we hit minimum D
            if (tune_d <= tune_d_min) {
                tune_d = tune_d_min;
                autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        }
    }
}

// autotune_updating_p_down - decrease P until we don't reach the target before time out
// P is decreased to ensure we are not overshooting the target
void autotune_updating_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio, float target, float measurement_max)
{
    if (measurement_max < target) {
        if (autotune_state.ignore_next == 0){
            // if maximum measurement was lower than target so increment the success counter
            autotune_counter++;
        } else {
            autotune_state.ignore_next = 0;
        }
    }else{
        // ignore the next result unless it is the same as this one
        autotune_state.ignore_next = 1;
        // if maximum measurement was higher than target so decrement the success counter
        if (autotune_counter > 0 ) {
            autotune_counter--;
        }
        // decrease P gain (which should decrease the maximum)
        tune_p -= tune_p*tune_p_step_ratio;
        // stop tuning if we hit maximum P
        if (tune_p <= tune_p_min) {
            tune_p = tune_p_min;
            autotune_counter = AUTOTUNE_SUCCESS_COUNT;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
    }
}

// autotune_updating_p_up - increase P to ensure the target is reached
// P is increased until we achieve our target within a reasonable time
void autotune_updating_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float target, float measurement_max)
{
    if (measurement_max > target) {
        // ignore the next result unless it is the same as this one
        autotune_state.ignore_next = 1;
        // if maximum measurement was greater than target so increment the success counter
        autotune_counter++;
    }else{
        if (autotune_state.ignore_next == 0){
            // if maximum measurement was lower than target so decrement the success counter
            if (autotune_counter > 0 ) {
                autotune_counter--;
            }
            // increase P gain (which should increase the maximum)
            tune_p += tune_p*tune_p_step_ratio;
            // stop tuning if we hit maximum P
            if (tune_p >= tune_p_max) {
                tune_p = tune_p_max;
                autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        } else {
            autotune_state.ignore_next = 0;
        }
    }
}

// autotune_updating_p_up - increase P to ensure the target is reached while checking bounce back isn't increasing
// P is increased until we achieve our target within a reasonable time while reducing D if bounce back increases above the threshold
void autotune_updating_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max)
{
    if (measurement_max > target) {
        // ignore the next result unless it is the same as this one
        autotune_state.ignore_next = 1;
        // if maximum measurement was greater than target so increment the success counter
        autotune_counter++;
    }else if ((measurement_max-measurement_min > measurement_max*g.autotune_aggressiveness) && (tune_d > tune_d_min)) {
        // if bounce back was larger than the threshold so decrement the success counter
        if (autotune_counter > 0 ) {
            autotune_counter--;
        }
        // decrease D gain (which should decrease bounce back)
        tune_d -= tune_d*tune_d_step_ratio;
        // stop tuning if we hit minimum D
        if (tune_d <= tune_d_min) {
            tune_d = tune_d_min;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
        // decrease P gain to match D gain reduction
        tune_p -= tune_p*tune_p_step_ratio;
        // stop tuning if we hit minimum P
        if (tune_p <= tune_p_min) {
            tune_p = tune_p_min;
            Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
        }
        // cancel change in direction
        autotune_state.positive_direction = !autotune_state.positive_direction;
    }else{
        if (autotune_state.ignore_next == 0){
            // if maximum measurement was lower than target so decrement the success counter
            if (autotune_counter > 0 ) {
                autotune_counter--;
            }
            // increase P gain (which should increase the maximum)
            tune_p += tune_p*tune_p_step_ratio;
            // stop tuning if we hit maximum P
            if (tune_p >= tune_p_max) {
                tune_p = tune_p_max;
                autotune_counter = AUTOTUNE_SUCCESS_COUNT;
                Log_Write_Event(DATA_AUTOTUNE_REACHED_LIMIT);
            }
        } else {
            autotune_state.ignore_next = 0;
        }
    }
}

// autotune_twitching_measure_acceleration - measure rate of change of measurement
void autotune_twitching_measure_acceleration(float &rate_of_change, float rate_measurement, float &rate_measurement_max)
{
    if (rate_measurement_max < rate_measurement) {
        rate_measurement_max = rate_measurement;
        rate_of_change = (1000.0f*rate_measurement_max)/(millis() - autotune_step_start_time);
    }
}

#endif  // AUTOTUNE_ENABLED == ENABLED
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/control_circle.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_circle.pde - init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
static bool circle_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {
        circle_pilot_yaw_override = false;

        // initialize speeds and accelerations
        pos_control.set_speed_xy(wp_nav.get_speed_xy());
        pos_control.set_accel_xy(wp_nav.get_wp_acceleration());
        pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control.set_accel_z(g.pilot_accel_z);

        // initialise circle controller including setting the circle center based on vehicle speed
        circle_nav.init();

        return true;
    }else{
        return false;
    }
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
static void circle_run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.land_complete) {
        // To-Do: add some initialisation of position controllers
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.set_alt_target_to_current_alt();
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
        if (target_yaw_rate != 0) {
            circle_pilot_yaw_override = true;
        }

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

        // check for pilot requested take-off
        if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }
    }

    // run circle controller
    circle_nav.update();

    // call attitude controller
    if (circle_pilot_yaw_override) {
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), target_yaw_rate);
    }else{
        attitude_control.angle_ef_roll_pitch_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), circle_nav.get_yaw(),true);
    }

    // run altitude controller
    if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
        // if sonar is ok, use surface tracking
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
    }
    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
    pos_control.update_z_controller();
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/control_drift.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_drift.pde - init and run calls for drift flight mode
 */

#ifndef DRIFT_SPEEDGAIN
 # define DRIFT_SPEEDGAIN 8.0f
#endif
#ifndef DRIFT_SPEEDLIMIT
 # define DRIFT_SPEEDLIMIT 560.0f
#endif

#ifndef DRIFT_THR_ASSIST_GAIN
 # define DRIFT_THR_ASSIST_GAIN 1.8f    // gain controlling amount of throttle assistance
#endif

#ifndef DRIFT_THR_ASSIST_MAX
 # define DRIFT_THR_ASSIST_MAX  300.0f  // maximum assistance throttle assist will provide
#endif

#ifndef DRIFT_THR_MIN
 # define DRIFT_THR_MIN         213     // throttle assist will be active when pilot's throttle is above this value
#endif
#ifndef DRIFT_THR_MAX
 # define DRIFT_THR_MAX         787     // throttle assist will be active when pilot's throttle is below this value
#endif

// drift_init - initialise drift controller
static bool drift_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {
        return true;
    }else{
        return false;
    }
}

// drift_run - runs the drift controller
// should be called at 100hz or more
static void drift_run()
{
    static float breaker = 0.0f;
    static float roll_input = 0.0f;
    float target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or landed and throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || (ap.land_complete && ap.throttle_zero)) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);

    // Grab inertial velocity
    const Vector3f& vel = inertial_nav.get_velocity();

    // rotate roll, pitch input from north facing to vehicle's perspective
    float roll_vel =  vel.y * ahrs.cos_yaw() - vel.x * ahrs.sin_yaw(); // body roll vel
    float pitch_vel = vel.y * ahrs.sin_yaw() + vel.x * ahrs.cos_yaw(); // body pitch vel

    // gain sceduling for Yaw
    float pitch_vel2 = min(fabs(pitch_vel), 2000);
    target_yaw_rate = ((float)target_roll/1.0f) * (1.0f - (pitch_vel2 / 5000.0f)) * g.acro_yaw_p;

    roll_vel = constrain_float(roll_vel, -DRIFT_SPEEDLIMIT, DRIFT_SPEEDLIMIT);
    pitch_vel = constrain_float(pitch_vel, -DRIFT_SPEEDLIMIT, DRIFT_SPEEDLIMIT);
    
    roll_input = roll_input * .96f + (float)g.rc_4.control_in * .04f;

    //convert user input into desired roll velocity
    float roll_vel_error = roll_vel - (roll_input / DRIFT_SPEEDGAIN);

    // Roll velocity is feed into roll acceleration to minimize slip
    target_roll = roll_vel_error * -DRIFT_SPEEDGAIN;
    target_roll = constrain_int16(target_roll, -4500, 4500);

    // If we let go of sticks, bring us to a stop
    if(target_pitch == 0){
        // .14/ (.03 * 100) = 4.6 seconds till full breaking
        breaker += .03f;
        breaker = min(breaker, DRIFT_SPEEDGAIN);
        target_pitch = pitch_vel * breaker;
    }else{
        breaker = 0.0f;
    }

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // output pilot's throttle with angle boost
    attitude_control.set_throttle_out(get_throttle_assist(vel.z, pilot_throttle_scaled), true, g.throttle_filt);
}

// get_throttle_assist - return throttle output (range 0 ~ 1000) based on pilot input and z-axis velocity
int16_t get_throttle_assist(float velz, int16_t pilot_throttle_scaled)
{
    // throttle assist - adjusts throttle to slow the vehicle's vertical velocity
    //      Only active when pilot's throttle is between 213 ~ 787
    //      Assistance is strongest when throttle is at mid, drops linearly to no assistance at 213 and 787
    float thr_assist = 0.0f;
    if (pilot_throttle_scaled > g.throttle_min && pilot_throttle_scaled < THR_MAX &&
        pilot_throttle_scaled > DRIFT_THR_MIN && pilot_throttle_scaled < DRIFT_THR_MAX) {
        // calculate throttle assist gain
        thr_assist = 1.2f - ((float)abs(pilot_throttle_scaled - 500) / 240.0f);
        thr_assist = constrain_float(thr_assist, 0.0f, 1.0f) * -DRIFT_THR_ASSIST_GAIN * velz;

        // ensure throttle assist never adjusts the throttle by more than 300 pwm
        thr_assist = constrain_float(thr_assist, -DRIFT_THR_ASSIST_MAX, DRIFT_THR_ASSIST_MAX);

        // ensure throttle assist never pushes throttle below throttle_min or above throttle_max
        thr_assist = constrain_float(thr_assist, g.throttle_min - pilot_throttle_scaled, THR_MAX - pilot_throttle_scaled);
    }
    
    return pilot_throttle_scaled + thr_assist;
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/control_flip.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_flip.pde - init and run calls for flip flight mode
 *      original implementation in 2010 by Jose Julio
 *      Adapted and updated for AC2 in 2011 by Jason Short
 *
 *      Controls:
 *          CH7_OPT - CH12_OPT parameter must be set to "Flip" (AUXSW_FLIP) which is "2"
 *          Pilot switches to Stabilize, Acro or AltHold flight mode and puts ch7/ch8 switch to ON position
 *          Vehicle will Roll right by default but if roll or pitch stick is held slightly left, forward or back it will flip in that direction
 *          Vehicle should complete the roll within 2.5sec and will then return to the original flight mode it was in before flip was triggered
 *          Pilot may manually exit flip by switching off ch7/ch8 or by moving roll stick to >40deg left or right
 *
 *      State machine approach:
 *          Flip_Start (while copter is leaning <45deg) : roll right at 400deg/sec, increase throttle
 *          Flip_Roll (while copter is between +45deg ~ -90) : roll right at 400deg/sec, reduce throttle
 *          Flip_Recover (while copter is between -90deg and original target angle) : use earth frame angle controller to return vehicle to original attitude
 */

#define FLIP_THR_INC        200     // throttle increase during Flip_Start stage (under 45deg lean angle)
#define FLIP_THR_DEC        240     // throttle decrease during Flip_Roll stage (between 45deg ~ -90deg roll)
#define FLIP_ROTATION_RATE  40000   // rotation rate request in centi-degrees / sec (i.e. 400 deg/sec)
#define FLIP_TIMEOUT_MS     2500    // timeout after 2.5sec.  Vehicle will switch back to original flight mode
#define FLIP_RECOVERY_ANGLE 500     // consider successful recovery when roll is back within 5 degrees of original

#define FLIP_ROLL_RIGHT      1      // used to set flip_dir
#define FLIP_ROLL_LEFT      -1      // used to set flip_dir

#define FLIP_PITCH_BACK      1      // used to set flip_dir
#define FLIP_PITCH_FORWARD  -1      // used to set flip_dir

FlipState flip_state;               // current state of flip
uint8_t   flip_orig_control_mode;   // flight mode when flip was initated
uint32_t  flip_start_time;          // time since flip began
int8_t    flip_roll_dir;            // roll direction (-1 = roll left, 1 = roll right)
int8_t    flip_pitch_dir;           // pitch direction (-1 = pitch forward, 1 = pitch back)

// flip_init - initialise flip controller
static bool flip_init(bool ignore_checks)
{
    // only allow flip from ACRO, Stabilize, AltHold or Drift flight modes
    if (control_mode != ACRO && control_mode != STABILIZE && control_mode != ALT_HOLD) {
        return false;
    }

    // if in acro or stabilize ensure throttle is above zero
    if ((g.rc_3.control_in <= 0) && (control_mode == ACRO || control_mode == STABILIZE)) {
        return false;
    }

    // ensure roll input is less than 40deg
    if (abs(g.rc_1.control_in) >= 4000) {
        return false;
    }

    // only allow flip when flying
    if (!motors.armed() || ap.land_complete) {
        return false;
    }

    // capture original flight mode so that we can return to it after completion
    flip_orig_control_mode = control_mode;

    // initialise state
    flip_state = Flip_Start;
    flip_start_time = millis();

    flip_roll_dir = flip_pitch_dir = 0;

    // choose direction based on pilot's roll and pitch sticks
    if (g.rc_2.control_in > 300) {
        flip_pitch_dir = FLIP_PITCH_BACK;
    }else if(g.rc_2.control_in < -300) {
        flip_pitch_dir = FLIP_PITCH_FORWARD;
    }else if (g.rc_1.control_in >= 0) {
        flip_roll_dir = FLIP_ROLL_RIGHT;
    }else{
        flip_roll_dir = FLIP_ROLL_LEFT;
    }

    // log start of flip
    Log_Write_Event(DATA_FLIP_START);

    // capture current attitude which will be used during the Flip_Recovery stage
    flip_orig_attitude.x = constrain_float(ahrs.roll_sensor, -aparm.angle_max, aparm.angle_max);
    flip_orig_attitude.y = constrain_float(ahrs.pitch_sensor, -aparm.angle_max, aparm.angle_max);
    flip_orig_attitude.z = ahrs.yaw_sensor;

    return true;
}

// flip_run - runs the flip controller
// should be called at 100hz or more
static void flip_run()
{
    int16_t throttle_out;
    float recovery_angle;

    // if pilot inputs roll > 40deg or timeout occurs abandon flip
    if (!motors.armed() || (abs(g.rc_1.control_in) >= 4000) || (abs(g.rc_2.control_in) >= 4000) || ((millis() - flip_start_time) > FLIP_TIMEOUT_MS)) {
        flip_state = Flip_Abandon;
    }

    // get pilot's desired throttle
    throttle_out = get_pilot_desired_throttle(g.rc_3.control_in);

    // get corrected angle based on direction and axis of rotation
    // we flip the sign of flip_angle to minimize the code repetition
    int32_t flip_angle;

    if (flip_roll_dir != 0) {
        flip_angle = ahrs.roll_sensor * flip_roll_dir;
    } else {
        flip_angle = ahrs.pitch_sensor * flip_pitch_dir;
    }

    // state machine
    switch (flip_state) {

    case Flip_Start:
        // under 45 degrees request 400deg/sec roll or pitch
        attitude_control.rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE * flip_roll_dir, FLIP_ROTATION_RATE * flip_pitch_dir, 0.0);

        // increase throttle
        throttle_out += FLIP_THR_INC;

        // beyond 45deg lean angle move to next stage
        if (flip_angle >= 4500) {
            if (flip_roll_dir != 0) {
                // we are rolling
            flip_state = Flip_Roll;
            } else {
                // we are pitching
                flip_state = Flip_Pitch_A;
        }
        }
        break;

    case Flip_Roll:
        // between 45deg ~ -90deg request 400deg/sec roll
        attitude_control.rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE * flip_roll_dir, 0.0, 0.0);
        // decrease throttle
        if (throttle_out >= g.throttle_min) {
            throttle_out = max(throttle_out - FLIP_THR_DEC, g.throttle_min);
        }

        // beyond -90deg move on to recovery
        if ((flip_angle < 4500) && (flip_angle > -9000)) {
            flip_state = Flip_Recover;
        }
        break;

    case Flip_Pitch_A:
        // between 45deg ~ -90deg request 400deg/sec pitch
        attitude_control.rate_bf_roll_pitch_yaw(0.0, FLIP_ROTATION_RATE * flip_pitch_dir, 0.0);
        // decrease throttle
        if (throttle_out >= g.throttle_min) {
            throttle_out = max(throttle_out - FLIP_THR_DEC, g.throttle_min);
        }

        // check roll for inversion
        if ((labs(ahrs.roll_sensor) > 9000) && (flip_angle > 4500)) {
            flip_state = Flip_Pitch_B;
        }
        break;

    case Flip_Pitch_B:
        // between 45deg ~ -90deg request 400deg/sec pitch
        attitude_control.rate_bf_roll_pitch_yaw(0.0, FLIP_ROTATION_RATE * flip_pitch_dir, 0.0);
        // decrease throttle
        if (throttle_out >= g.throttle_min) {
            throttle_out = max(throttle_out - FLIP_THR_DEC, g.throttle_min);
        }

        // check roll for inversion
        if ((labs(ahrs.roll_sensor) < 9000) && (flip_angle > -4500)) {
            flip_state = Flip_Recover;
        }
        break;

    case Flip_Recover:
        // use originally captured earth-frame angle targets to recover
        attitude_control.angle_ef_roll_pitch_yaw(flip_orig_attitude.x, flip_orig_attitude.y, flip_orig_attitude.z, false);

        // increase throttle to gain any lost altitude
        throttle_out += FLIP_THR_INC;

        if (flip_roll_dir != 0) {
            // we are rolling
            recovery_angle = fabs(flip_orig_attitude.x - (float)ahrs.roll_sensor);
        } else {
            // we are pitching
            recovery_angle = fabs(flip_orig_attitude.y - (float)ahrs.pitch_sensor);
        }

        // check for successful recovery
        if (fabs(recovery_angle) <= FLIP_RECOVERY_ANGLE) {
            // restore original flight mode
            if (!set_mode(flip_orig_control_mode)) {
                // this should never happen but just in case
                set_mode(STABILIZE);
            }
            // log successful completion
            Log_Write_Event(DATA_FLIP_END);
        }
        break;

    case Flip_Abandon:
        // restore original flight mode
        if (!set_mode(flip_orig_control_mode)) {
            // this should never happen but just in case
            set_mode(STABILIZE);
        }
        // log abandoning flip
        Log_Write_Error(ERROR_SUBSYSTEM_FLIP,ERROR_CODE_FLIP_ABANDONED);
        break;
    }

    // output pilot's throttle without angle boost
    if (throttle_out == 0.0f) {
        attitude_control.set_throttle_out_unstabilized(0,false,g.throttle_filt);
    } else {
        attitude_control.set_throttle_out(throttle_out, false, g.throttle_filt);
    }
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/control_guided.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_guided.pde - init and run calls for guided flight mode
 */

#ifndef GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM
 # define GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM     500     // point nose at target if it is more than 5m away
#endif

#define GUIDED_POSVEL_TIMEOUT_MS    3000    // guided mode's position-velocity controller times out after 3seconds with no new updates

static Vector3f posvel_pos_target_cm;
static Vector3f posvel_vel_target_cms;
static uint32_t posvel_update_time_ms;

struct Guided_Limit {
    uint32_t timeout_ms;  // timeout (in seconds) from the time that guided is invoked
    float alt_min_cm;   // lower altitude limit in cm above home (0 = no limit)
    float alt_max_cm;   // upper altitude limit in cm above home (0 = no limit)
    float horiz_max_cm; // horizontal position limit in cm from where guided mode was initiated (0 = no limit)
    uint32_t start_time;// system time in milliseconds that control was handed to the external computer
    Vector3f start_pos; // start position as a distance from home in cm.  used for checking horiz_max limit
} guided_limit;

// guided_init - initialise guided controller
static bool guided_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {
        // initialise yaw
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
        // start in position control mode
        guided_pos_control_start();
        return true;
    }else{
        return false;
    }
}


// guided_takeoff_start - initialises waypoint controller to implement take-off
static void guided_takeoff_start(float final_alt_above_home)
{
    guided_mode = Guided_TakeOff;
    
    // initialise wpnav destination
    Vector3f target_pos = inertial_nav.get_position();
    target_pos.z = pv_alt_above_origin(final_alt_above_home);
    wp_nav.set_wp_destination(target_pos);

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    // tell motors to do a slow start
    motors.slow_start(true);
}

// initialise guided mode's position controller
static void guided_pos_control_start()
{
    // set to position control mode
    guided_mode = Guided_WP;

    // initialise waypoint and spline controller
    wp_nav.wp_and_spline_init();

    // initialise wpnav to stopping point at current altitude
    // To-Do: set to current location if disarmed?
    // To-Do: set to stopping point altitude?
    Vector3f stopping_point;
    stopping_point.z = inertial_nav.get_altitude();
    wp_nav.get_wp_stopping_point_xy(stopping_point);
    wp_nav.set_wp_destination(stopping_point);

    // initialise yaw
    set_auto_yaw_mode(get_default_auto_yaw_mode(false));
}

// initialise guided mode's velocity controller
static void guided_vel_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_Velocity;

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise velocity controller
    pos_control.init_vel_controller_xyz();
}

// initialise guided mode's posvel controller
static void guided_posvel_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_PosVel;

    pos_control.init_xy_controller();

    // set speed and acceleration from wpnav's speed and acceleration
    pos_control.set_speed_xy(wp_nav.get_speed_xy());
    pos_control.set_accel_xy(wp_nav.get_wp_acceleration());

    const Vector3f& curr_pos = inertial_nav.get_position();
    const Vector3f& curr_vel = inertial_nav.get_velocity();

    // set target position and velocity to current position and velocity
    pos_control.set_xy_target(curr_pos.x, curr_pos.y);
    pos_control.set_desired_velocity_xy(curr_vel.x, curr_vel.y);

    // set vertical speed and acceleration
    pos_control.set_speed_z(wp_nav.get_speed_down(), wp_nav.get_speed_up());
    pos_control.set_accel_z(wp_nav.get_accel_z());

    // pilot always controls yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// guided_set_destination - sets guided mode's target destination
static void guided_set_destination(const Vector3f& destination)
{
    // ensure we are in position control mode
    if (guided_mode != Guided_WP) {
        guided_pos_control_start();
    }

    wp_nav.set_wp_destination(destination);
}

// guided_set_velocity - sets guided mode's target velocity
static void guided_set_velocity(const Vector3f& velocity)
{
    // check we are in velocity control mode
    if (guided_mode != Guided_Velocity) {
        guided_vel_control_start();
    }

    // set position controller velocity target
    pos_control.set_desired_velocity(velocity);
}

// set guided mode posvel target
static void guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity) {
    // check we are in velocity control mode
    if (guided_mode != Guided_PosVel) {
        guided_posvel_control_start();
    }

    posvel_update_time_ms = millis();
    posvel_pos_target_cm = destination;
    posvel_vel_target_cms = velocity;

    pos_control.set_pos_target(posvel_pos_target_cm);
}

// guided_run - runs the guided controller
// should be called at 100hz or more
static void guided_run()
{
    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        // To-Do: reset waypoint controller?
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        // To-Do: handle take-offs - these may not only be immediately after auto_armed becomes true
        return;
    }

    // call the correct auto controller
    switch (guided_mode) {

    case Guided_TakeOff:
        // run takeoff controller
        guided_takeoff_run();
        break;

    case Guided_WP:
        // run position controller
        guided_pos_control_run();
        break;

    case Guided_Velocity:
        // run velocity controller
        guided_vel_control_run();
        break;

    case Guided_PosVel:
        // run position-velocity controller
        guided_posvel_control_run();
        break;
    }
 }

// guided_takeoff_run - takeoff in guided mode
//      called by guided_run at 100hz or more
static void guided_takeoff_run()
{
    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        // initialise wpnav targets
        wp_nav.shift_wp_origin_to_current_pos();
        // reset attitude control targets
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        // tell motors to do a slow start
        motors.slow_start(true);
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
}

// guided_pos_control_run - runs the guided position controller
// called from guided_run
static void guided_pos_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
        if (target_yaw_rate != 0) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(), true);
    }
}

// guided_vel_control_run - runs the guided velocity controller
// called from guided_run
static void guided_vel_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
        if (target_yaw_rate != 0) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // calculate dt
    float dt = pos_control.time_since_last_xy_update();

    // update at poscontrol update rate
    if (dt >= pos_control.get_dt_xy()) {
        // sanity check dt
        if (dt >= 0.2f) {
            dt = 0.0f;
        }

        // call velocity controller which includes z axis controller
        pos_control.update_vel_controller_xyz(ekfNavVelGainScaler);
    }

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(pos_control.get_roll(), pos_control.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(pos_control.get_roll(), pos_control.get_pitch(), get_auto_heading(), true);
    }
}

// guided_posvel_control_run - runs the guided spline controller
// called from guided_run
static void guided_posvel_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;

    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
        if (target_yaw_rate != 0) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set velocity to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - posvel_update_time_ms > GUIDED_POSVEL_TIMEOUT_MS && !posvel_vel_target_cms.is_zero()) {
        posvel_vel_target_cms.zero();
    }

    // calculate dt
    float dt = pos_control.time_since_last_xy_update();

    // update at poscontrol update rate
    if (dt >= pos_control.get_dt_xy()) {
        // sanity check dt
        if (dt >= 0.2f) {
            dt = 0.0f;
        }

        // advance position target using velocity target
        posvel_pos_target_cm += posvel_vel_target_cms * dt;

        // send position and velocity targets to position controller
        pos_control.set_pos_target(posvel_pos_target_cm);
        pos_control.set_desired_velocity_xy(posvel_vel_target_cms.x, posvel_vel_target_cms.y);

        // run position controller
        pos_control.update_xy_controller(AC_PosControl::XY_MODE_POS_AND_VEL_FF, ekfNavVelGainScaler);
        pos_control.update_z_controller();
    }

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(pos_control.get_roll(), pos_control.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(pos_control.get_roll(), pos_control.get_pitch(), get_auto_heading(), true);
    }
}

// Guided Limit code

// guided_limit_clear - clear/turn off guided limits
static void guided_limit_clear()
{
    guided_limit.timeout_ms = 0;
    guided_limit.alt_min_cm = 0.0f;
    guided_limit.alt_max_cm = 0.0f;
    guided_limit.horiz_max_cm = 0.0f;
}

// guided_limit_set - set guided timeout and movement limits
static void guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm)
{
    guided_limit.timeout_ms = timeout_ms;
    guided_limit.alt_min_cm = alt_min_cm;
    guided_limit.alt_max_cm = alt_max_cm;
    guided_limit.horiz_max_cm = horiz_max_cm;
}

// guided_limit_init_time_and_pos - initialise guided start time and position as reference for limit checking
//  only called from AUTO mode's auto_nav_guided_start function
static void guided_limit_init_time_and_pos()
{
    // initialise start time
    guided_limit.start_time = hal.scheduler->millis();

    // initialise start position from current position
    guided_limit.start_pos = inertial_nav.get_position();
}

// guided_limit_check - returns true if guided mode has breached a limit
//  used when guided is invoked from the NAV_GUIDED_ENABLE mission command
static bool guided_limit_check()
{
    // check if we have passed the timeout
    if ((guided_limit.timeout_ms > 0) && (millis() - guided_limit.start_time >= guided_limit.timeout_ms)) {
        return true;
    }

    // get current location
    const Vector3f& curr_pos = inertial_nav.get_position();

    // check if we have gone below min alt
    if ((guided_limit.alt_min_cm != 0.0f) && (curr_pos.z < guided_limit.alt_min_cm)) {
        return true;
    }

    // check if we have gone above max alt
    if ((guided_limit.alt_max_cm != 0.0f) && (curr_pos.z > guided_limit.alt_max_cm)) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (guided_limit.horiz_max_cm > 0.0f) {
        float horiz_move = pv_get_horizontal_distance_cm(guided_limit.start_pos, curr_pos);
        if (horiz_move > guided_limit.horiz_max_cm) {
            return true;
        }
    }

    // if we got this far we must be within limits
    return false;
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/control_land.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static bool land_with_gps;

static uint32_t land_start_time;
static bool land_pause;

// land_init - initialise land controller
static bool land_init(bool ignore_checks)
{
    // check if we have GPS and decide which LAND we're going to do
    land_with_gps = position_ok();
    if (land_with_gps) {
        // set target to stopping point
        Vector3f stopping_point;
        wp_nav.get_loiter_stopping_point_xy(stopping_point);
        wp_nav.init_loiter_target(stopping_point);
    }

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(wp_nav.get_speed_down(), wp_nav.get_speed_up());
    pos_control.set_accel_z(wp_nav.get_accel_z());

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    land_start_time = millis();

    land_pause = false;

    return true;
}

// land_run - runs the land controller
// should be called at 100hz or more
static void land_run()
{
    if (land_with_gps) {
        land_gps_run();
    }else{
        land_nogps_run();
    }
}

// land_run - runs the land controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
static void land_gps_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;

    // if not auto armed or landed set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.land_complete) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        wp_nav.init_loiter_target();

#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (ap.land_complete && (ap.throttle_zero || failsafe.radio)) {
            init_disarm_motors();
        }
#else
        // disarm when the landing detector says we've landed
        if (ap.land_complete) {
            init_disarm_motors();
        }
#endif
        return;
    }

    // relax loiter target if we might be landed
    if (land_complete_maybe()) {
        wp_nav.loiter_soften_for_landing();
    }

    // process pilot inputs
    if (!failsafe.radio) {
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // process pilot's roll and pitch input
            roll_control = g.rc_1.control_in;
            pitch_control = g.rc_2.control_in;
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

    // process roll, pitch inputs
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

    // run loiter controller
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

    // pause 4 seconds before beginning land descent
    float cmb_rate;
    if(land_pause && millis()-land_start_time < 4000) {
        cmb_rate = 0;
    } else {
        land_pause = false;
        cmb_rate = get_land_descent_speed();
    }

    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();
}

// land_nogps_run - runs the land controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
static void land_nogps_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f;
    float target_yaw_rate = 0;

    // if not auto armed or landed set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.land_complete) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (ap.land_complete && (ap.throttle_zero || failsafe.radio)) {
            init_disarm_motors();
        }
#else
        // disarm when the landing detector says we've landed
        if (ap.land_complete) {
            init_disarm_motors();
        }
#endif
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // get pilot desired lean angles
            get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // pause 4 seconds before beginning land descent
    float cmb_rate;
    if(land_pause && millis()-land_start_time < LAND_WITH_DELAY_MS) {
        cmb_rate = 0;
    } else {
        land_pause = false;
        cmb_rate = get_land_descent_speed();
    }

    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // call position controller
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();
}

// get_land_descent_speed - high level landing logic
//      returns climb rate (in cm/s) which should be passed to the position controller
//      should be called at 100hz or higher
static float get_land_descent_speed()
{
#if CONFIG_SONAR == ENABLED
    bool sonar_ok = sonar_enabled && (sonar.status() == RangeFinder::RangeFinder_Good);
#else
    bool sonar_ok = false;
#endif
    // if we are above 10m and the sonar does not sense anything perform regular alt hold descent
    if (pos_control.get_pos_target().z >= pv_alt_above_origin(LAND_START_ALT) && !(sonar_ok && sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
        return pos_control.get_speed_down();
    }else{
        return -abs(g.land_speed);
    }
}

// land_do_not_use_GPS - forces land-mode to not use the GPS but instead rely on pilot input for roll and pitch
//  called during GPS failsafe to ensure that if we were already in LAND mode that we do not use the GPS
//  has no effect if we are not already in LAND mode
static void land_do_not_use_GPS()
{
    land_with_gps = false;
}

// set_mode_land_with_pause - sets mode to LAND and triggers 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
static void set_mode_land_with_pause()
{
    set_mode(LAND);
    land_pause = true;

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}

// landing_with_GPS - returns true if vehicle is landing using GPS
static bool landing_with_GPS() {
    return (control_mode == LAND && land_with_gps);
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/control_loiter.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_loiter.pde - init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
static bool loiter_init(bool ignore_checks)
{
    if (position_ok() || optflow_position_ok() || ignore_checks) {

        // set target to current position
        wp_nav.init_loiter_target();

        // initialize vertical speed and accelerationj
        pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control.set_accel_z(g.pilot_accel_z);

        // initialise altitude target to stopping point
        pos_control.set_target_to_stopping_point_z();

        return true;
    }else{
        return false;
    }
}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
static void loiter_run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        wp_nav.init_loiter_target();
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(g.rc_3.control_in)-throttle_average);
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // process pilot's roll and pitch input
        wp_nav.set_pilot_desired_acceleration(g.rc_1.control_in, g.rc_2.control_in);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

        // check for pilot requested take-off
        if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        wp_nav.clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (land_complete_maybe()) {
        wp_nav.loiter_soften_for_landing();
    }

    // when landed reset targets and output zero throttle
    if (ap.land_complete) {
        wp_nav.init_loiter_target();
        // move throttle to between minimum and non-takeoff-throttle to keep us on the ground
        attitude_control.set_throttle_out_unstabilized(get_throttle_pre_takeoff(g.rc_3.control_in),true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(g.rc_3.control_in)-throttle_average);
    }else{
        // run loiter controller
        wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

        // body-frame rate controller is run directly from 100hz loop

        // run altitude controller
        if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
            // if sonar is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // update altitude target and call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
    }
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/control_poshold.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if POSHOLD_ENABLED == ENABLED

/*
 * control_poshold.pde - init and run calls for PosHold flight mode
 *     PosHold tries to improve upon regular loiter by mixing the pilot input with the loiter controller
 */

#define POSHOLD_SPEED_0                          10      // speed below which it is always safe to switch to loiter

#if MAIN_LOOP_RATE == 100
 // definitions for 100hz loop update rate
 # define POSHOLD_BRAKE_TIME_ESTIMATE_MAX        600     // max number of cycles the brake will be applied before we switch to loiter
 # define POSHOLD_BRAKE_TO_LOITER_TIMER          150     // Number of cycles to transition from brake mode to loiter mode.
 # define POSHOLD_WIND_COMP_START_TIMER          150     // Number of cycles to start wind compensation update after loiter is engaged
 # define POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER  50      // Set it from 100 to 200, the number of centiseconds loiter and manual commands are mixed to make a smooth transition.
 # define POSHOLD_SMOOTH_RATE_FACTOR             0.05f   // filter applied to pilot's roll/pitch input as it returns to center.  A lower number will cause the roll/pitch to return to zero more slowly if the brake_rate is also low.
 # define POSHOLD_WIND_COMP_TIMER_10HZ           10      // counter value used to reduce wind compensation to 10hz
 # define LOOP_RATE_FACTOR                       1       // used to adapt PosHold params to loop_rate
 # define TC_WIND_COMP                          0.01f   // Time constant for poshold_update_wind_comp_estimate()
 #else
 // definitions for 400hz loop update rate
 # define POSHOLD_BRAKE_TIME_ESTIMATE_MAX        (600*4) // max number of cycles the brake will be applied before we switch to loiter
 # define POSHOLD_BRAKE_TO_LOITER_TIMER          (150*4) // Number of cycles to transition from brake mode to loiter mode.  Must be lower than POSHOLD_LOITER_STAB_TIMER
 # define POSHOLD_WIND_COMP_START_TIMER          (150*4) // Number of cycles to start wind compensation update after loiter is engaged
 # define POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER  (50*4)  // Set it from 100 to 200, the number of centiseconds loiter and manual commands are mixed to make a smooth transition.
 # define POSHOLD_SMOOTH_RATE_FACTOR             0.0125f // filter applied to pilot's roll/pitch input as it returns to center.  A lower number will cause the roll/pitch to return to zero more slowly if the brake_rate is also low.
 # define POSHOLD_WIND_COMP_TIMER_10HZ           40      // counter value used to reduce wind compensation to 10hz
 # define LOOP_RATE_FACTOR                       4       // used to adapt PosHold params to loop_rate
 # define TC_WIND_COMP                          0.0025f // Time constant for poshold_update_wind_comp_estimate()
 #endif

// definitions that are independent of main loop rate
#define POSHOLD_STICK_RELEASE_SMOOTH_ANGLE       1800    // max angle required (in centi-degrees) after which the smooth stick release effect is applied
#define POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX     10      // wind compensation estimates will only run when velocity is at or below this speed in cm/s

// declare some function to keep compiler happy
static void poshold_update_pilot_lean_angle(float &lean_angle_filtered, float &lean_angle_raw);
static int16_t poshold_mix_controls(float mix_ratio, int16_t first_control, int16_t second_control);
static void poshold_update_brake_angle_from_velocity(int16_t &brake_angle, float velocity);
static void poshold_update_wind_comp_estimate();
static void poshold_get_wind_comp_lean_angles(int16_t &roll_angle, int16_t &pitch_angle);
static void poshold_roll_controller_to_pilot_override();
static void poshold_pitch_controller_to_pilot_override();

// mission state enumeration
enum poshold_rp_mode {
    POSHOLD_PILOT_OVERRIDE=0,            // pilot is controlling this axis (i.e. roll or pitch)
    POSHOLD_BRAKE,                       // this axis is braking towards zero
    POSHOLD_BRAKE_READY_TO_LOITER,       // this axis has completed braking and is ready to enter loiter mode (both modes must be this value before moving to next stage)
    POSHOLD_BRAKE_TO_LOITER,             // both vehicle's axis (roll and pitch) are transitioning from braking to loiter mode (braking and loiter controls are mixed)
    POSHOLD_LOITER,                      // both vehicle axis are holding position
    POSHOLD_CONTROLLER_TO_PILOT_OVERRIDE // pilot has input controls on this axis and this axis is transitioning to pilot override (other axis will transition to brake if no pilot input)
};

static struct {
    poshold_rp_mode roll_mode            : 3;    // roll mode: pilot override, brake or loiter
    poshold_rp_mode pitch_mode           : 3;    // pitch mode: pilot override, brake or loiter
    uint8_t braking_time_updated_roll   : 1;    // true once we have re-estimated the braking time.  This is done once as the vehicle begins to flatten out after braking
    uint8_t braking_time_updated_pitch  : 1;    // true once we have re-estimated the braking time.  This is done once as the vehicle begins to flatten out after braking
    uint8_t loiter_reset_I              : 1;    // true the very first time PosHold enters loiter, thereafter we trust the i terms loiter has

    // pilot input related variables
    float pilot_roll;                         // pilot requested roll angle (filtered to slow returns to zero)
    float pilot_pitch;                        // pilot requested roll angle (filtered to slow returns to zero)

    // braking related variables
    float brake_gain;                           // gain used during conversion of vehicle's velocity to lean angle during braking (calculated from brake_rate)
    int16_t brake_roll;                         // target roll angle during braking periods
    int16_t brake_pitch;                        // target pitch angle during braking periods
    int16_t brake_timeout_roll;                 // number of cycles allowed for the braking to complete, this timeout will be updated at half-braking
    int16_t brake_timeout_pitch;                // number of cycles allowed for the braking to complete, this timeout will be updated at half-braking
    int16_t brake_angle_max_roll;               // maximum lean angle achieved during braking.  Used to determine when the vehicle has begun to flatten out so that we can re-estimate the braking time
    int16_t brake_angle_max_pitch;              // maximum lean angle achieved during braking  Used to determine when the vehicle has begun to flatten out so that we can re-estimate the braking time
    int16_t brake_to_loiter_timer;              // cycles to mix brake and loiter controls in POSHOLD_BRAKE_TO_LOITER

    // loiter related variables
    int16_t controller_to_pilot_timer_roll;     // cycles to mix controller and pilot controls in POSHOLD_CONTROLLER_TO_PILOT
    int16_t controller_to_pilot_timer_pitch;    // cycles to mix controller and pilot controls in POSHOLD_CONTROLLER_TO_PILOT
    int16_t controller_final_roll;              // final roll angle from controller as we exit brake or loiter mode (used for mixing with pilot input)
    int16_t controller_final_pitch;             // final pitch angle from controller as we exit brake or loiter mode (used for mixing with pilot input)

    // wind compensation related variables
    Vector2f wind_comp_ef;                      // wind compensation in earth frame, filtered lean angles from position controller
    int16_t wind_comp_roll;                     // roll angle to compensate for wind
    int16_t wind_comp_pitch;                    // pitch angle to compensate for wind
    uint16_t wind_comp_start_timer;             // counter to delay start of wind compensation for a short time after loiter is engaged
    int8_t  wind_comp_timer;                    // counter to reduce wind comp roll/pitch lean angle calcs to 10hz

    // final output
    int16_t roll;   // final roll angle sent to attitude controller
    int16_t pitch;  // final pitch angle sent to attitude controller
} poshold;

// poshold_init - initialise PosHold controller
static bool poshold_init(bool ignore_checks)
{
    // fail to initialise PosHold mode if no GPS lock
    if (!position_ok() && !ignore_checks) {
        return false;
    }
    
    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    // initialise lean angles to current attitude
    poshold.pilot_roll = 0;
    poshold.pilot_pitch = 0;

    // compute brake_gain
    poshold.brake_gain = (15.0f * (float)g.poshold_brake_rate + 95.0f) / 100.0f;

    if (ap.land_complete) {
        // if landed begin in loiter mode
        poshold.roll_mode = POSHOLD_LOITER;
        poshold.pitch_mode = POSHOLD_LOITER;
        // set target to current position
        // only init here as we can switch to PosHold in flight with a velocity <> 0 that will be used as _last_vel in PosControl and never updated again as we inhibit Reset_I
        wp_nav.init_loiter_target();
    }else{
        // if not landed start in pilot override to avoid hard twitch
        poshold.roll_mode = POSHOLD_PILOT_OVERRIDE;
        poshold.pitch_mode = POSHOLD_PILOT_OVERRIDE;
    }

    // loiter's I terms should be reset the first time only
    poshold.loiter_reset_I = true;

    // initialise wind_comp each time PosHold is switched on
    poshold.wind_comp_ef.zero();
    poshold.wind_comp_roll = 0;
    poshold.wind_comp_pitch = 0;
    poshold.wind_comp_timer = 0;

    return true;
}

// poshold_run - runs the PosHold controller
// should be called at 100hz or more
static void poshold_run()
{
    float target_roll, target_pitch;  // pilot's roll and pitch angle inputs
    float target_yaw_rate = 0;          // pilot desired yaw rate in centi-degrees/sec
    int16_t target_climb_rate = 0;      // pilot desired climb rate in centimeters/sec
    float brake_to_loiter_mix;          // mix of brake and loiter controls.  0 = fully brake controls, 1 = fully loiter controls
    float controller_to_pilot_roll_mix; // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
    float controller_to_pilot_pitch_mix;    // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
    float vel_fw, vel_right;            // vehicle's current velocity in body-frame forward and right directions
    const Vector3f& vel = inertial_nav.get_velocity();

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        wp_nav.init_loiter_target();
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(g.rc_3.control_in)-throttle_average);
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

        // get pilot desired climb rate (for alt-hold mode and take-off)
        target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

        // check for pilot requested take-off
        if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }
    }

    // relax loiter target if we might be landed
    if (land_complete_maybe()) {
        wp_nav.loiter_soften_for_landing();
    }

    // if landed initialise loiter targets, set throttle to zero and exit
    if (ap.land_complete) {
        wp_nav.init_loiter_target();
        // move throttle to between minimum and non-takeoff-throttle to keep us on the ground
        attitude_control.set_throttle_out_unstabilized(get_throttle_pre_takeoff(g.rc_3.control_in),true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(g.rc_3.control_in)-throttle_average);
        return;
    }else{
        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

        // convert inertial nav earth-frame velocities to body-frame
        // To-Do: move this to AP_Math (or perhaps we already have a function to do this)
        vel_fw = vel.x*ahrs.cos_yaw() + vel.y*ahrs.sin_yaw();
        vel_right = -vel.x*ahrs.sin_yaw() + vel.y*ahrs.cos_yaw();
        
        // If not in LOITER, retrieve latest wind compensation lean angles related to current yaw
        if (poshold.roll_mode != POSHOLD_LOITER || poshold.pitch_mode != POSHOLD_LOITER)
        poshold_get_wind_comp_lean_angles(poshold.wind_comp_roll, poshold.wind_comp_pitch);

        // Roll state machine
        //  Each state (aka mode) is responsible for:
        //      1. dealing with pilot input
        //      2. calculating the final roll output to the attitude controller
        //      3. checking if the state (aka mode) should be changed and if 'yes' perform any required initialisation for the new state
        switch (poshold.roll_mode) {

            case POSHOLD_PILOT_OVERRIDE:
                // update pilot desired roll angle using latest radio input
                //  this filters the input so that it returns to zero no faster than the brake-rate
                poshold_update_pilot_lean_angle(poshold.pilot_roll, target_roll);

                // switch to BRAKE mode for next iteration if no pilot input
                if ((target_roll == 0) && (abs(poshold.pilot_roll) < 2 * g.poshold_brake_rate)) {
                    // initialise BRAKE mode
                    poshold.roll_mode = POSHOLD_BRAKE;        // Set brake roll mode
                    poshold.brake_roll = 0;                  // initialise braking angle to zero
                    poshold.brake_angle_max_roll = 0;        // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                    poshold.brake_timeout_roll = POSHOLD_BRAKE_TIME_ESTIMATE_MAX; // number of cycles the brake will be applied, updated during braking mode.
                    poshold.braking_time_updated_roll = false;   // flag the braking time can be re-estimated
                }

                // final lean angle should be pilot input plus wind compensation
                poshold.roll = poshold.pilot_roll + poshold.wind_comp_roll;
                break;

            case POSHOLD_BRAKE:
            case POSHOLD_BRAKE_READY_TO_LOITER:
                // calculate brake_roll angle to counter-act velocity
                poshold_update_brake_angle_from_velocity(poshold.brake_roll, vel_right);

                // update braking time estimate
                if (!poshold.braking_time_updated_roll) {
                    // check if brake angle is increasing
                    if (abs(poshold.brake_roll) >= poshold.brake_angle_max_roll) {
                        poshold.brake_angle_max_roll = abs(poshold.brake_roll);
                    } else {
                        // braking angle has started decreasing so re-estimate braking time
                        poshold.brake_timeout_roll = 1+(uint16_t)(LOOP_RATE_FACTOR*15L*(int32_t)(abs(poshold.brake_roll))/(10L*(int32_t)g.poshold_brake_rate));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                        poshold.braking_time_updated_roll = true;
                    }
                }

                // if velocity is very low reduce braking time to 0.5seconds
                if ((fabs(vel_right) <= POSHOLD_SPEED_0) && (poshold.brake_timeout_roll > 50*LOOP_RATE_FACTOR)) {
                    poshold.brake_timeout_roll = 50*LOOP_RATE_FACTOR;
                }

                // reduce braking timer
                if (poshold.brake_timeout_roll > 0) {
                    poshold.brake_timeout_roll--;
                } else {
                    // indicate that we are ready to move to Loiter.
                    // Loiter will only actually be engaged once both roll_mode and pitch_mode are changed to POSHOLD_BRAKE_READY_TO_LOITER
                    //  logic for engaging loiter is handled below the roll and pitch mode switch statements
                    poshold.roll_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                }

                // final lean angle is braking angle + wind compensation angle
                poshold.roll = poshold.brake_roll + poshold.wind_comp_roll;

                // check for pilot input
                if (target_roll != 0) {
                    // init transition to pilot override
                    poshold_roll_controller_to_pilot_override();
                }
                break;

            case POSHOLD_BRAKE_TO_LOITER:
            case POSHOLD_LOITER:
                // these modes are combined roll-pitch modes and are handled below
                break;

            case POSHOLD_CONTROLLER_TO_PILOT_OVERRIDE:
                // update pilot desired roll angle using latest radio input
                //  this filters the input so that it returns to zero no faster than the brake-rate
                poshold_update_pilot_lean_angle(poshold.pilot_roll, target_roll);

                // count-down loiter to pilot timer
                if (poshold.controller_to_pilot_timer_roll > 0) {
                    poshold.controller_to_pilot_timer_roll--;
                } else {
                    // when timer runs out switch to full pilot override for next iteration
                    poshold.roll_mode = POSHOLD_PILOT_OVERRIDE;
                }

                // calculate controller_to_pilot mix ratio
                controller_to_pilot_roll_mix = (float)poshold.controller_to_pilot_timer_roll / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;

                // mix final loiter lean angle and pilot desired lean angles
                poshold.roll = poshold_mix_controls(controller_to_pilot_roll_mix, poshold.controller_final_roll, poshold.pilot_roll + poshold.wind_comp_roll);
                break;
        }

        // Pitch state machine
        //  Each state (aka mode) is responsible for:
        //      1. dealing with pilot input
        //      2. calculating the final pitch output to the attitude contpitcher
        //      3. checking if the state (aka mode) should be changed and if 'yes' perform any required initialisation for the new state
        switch (poshold.pitch_mode) {

            case POSHOLD_PILOT_OVERRIDE:
                // update pilot desired pitch angle using latest radio input
                //  this filters the input so that it returns to zero no faster than the brake-rate
                poshold_update_pilot_lean_angle(poshold.pilot_pitch, target_pitch);

                // switch to BRAKE mode for next iteration if no pilot input
                if ((target_pitch == 0) && (abs(poshold.pilot_pitch) < 2 * g.poshold_brake_rate)) {
                    // initialise BRAKE mode
                    poshold.pitch_mode = POSHOLD_BRAKE;       // set brake pitch mode
                    poshold.brake_pitch = 0;                 // initialise braking angle to zero
                    poshold.brake_angle_max_pitch = 0;       // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                    poshold.brake_timeout_pitch = POSHOLD_BRAKE_TIME_ESTIMATE_MAX; // number of cycles the brake will be applied, updated during braking mode.
                    poshold.braking_time_updated_pitch = false;   // flag the braking time can be re-estimated
                }

                // final lean angle should be pilot input plus wind compensation
                poshold.pitch = poshold.pilot_pitch + poshold.wind_comp_pitch;
                break;

            case POSHOLD_BRAKE:
            case POSHOLD_BRAKE_READY_TO_LOITER:
                // calculate brake_pitch angle to counter-act velocity
                poshold_update_brake_angle_from_velocity(poshold.brake_pitch, -vel_fw);

                // update braking time estimate
                if (!poshold.braking_time_updated_pitch) {
                    // check if brake angle is increasing
                    if (abs(poshold.brake_pitch) >= poshold.brake_angle_max_pitch) {
                        poshold.brake_angle_max_pitch = abs(poshold.brake_pitch);
                    } else {
                        // braking angle has started decreasing so re-estimate braking time
                        poshold.brake_timeout_pitch = 1+(uint16_t)(LOOP_RATE_FACTOR*15L*(int32_t)(abs(poshold.brake_pitch))/(10L*(int32_t)g.poshold_brake_rate));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                        poshold.braking_time_updated_pitch = true;
                    }
                }

                // if velocity is very low reduce braking time to 0.5seconds
                if ((fabs(vel_fw) <= POSHOLD_SPEED_0) && (poshold.brake_timeout_pitch > 50*LOOP_RATE_FACTOR)) {
                    poshold.brake_timeout_pitch = 50*LOOP_RATE_FACTOR;
                }

                // reduce braking timer
                if (poshold.brake_timeout_pitch > 0) {
                    poshold.brake_timeout_pitch--;
                } else {
                    // indicate that we are ready to move to Loiter.
                    // Loiter will only actually be engaged once both pitch_mode and pitch_mode are changed to POSHOLD_BRAKE_READY_TO_LOITER
                    //  logic for engaging loiter is handled below the pitch and pitch mode switch statements
                    poshold.pitch_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                }

                // final lean angle is braking angle + wind compensation angle
                poshold.pitch = poshold.brake_pitch + poshold.wind_comp_pitch;

                // check for pilot input
                if (target_pitch != 0) {
                    // init transition to pilot override
                    poshold_pitch_controller_to_pilot_override();
                }
                break;

            case POSHOLD_BRAKE_TO_LOITER:
            case POSHOLD_LOITER:
                // these modes are combined pitch-pitch modes and are handled below
                break;

            case POSHOLD_CONTROLLER_TO_PILOT_OVERRIDE:
                // update pilot desired pitch angle using latest radio input
                //  this filters the input so that it returns to zero no faster than the brake-rate
                poshold_update_pilot_lean_angle(poshold.pilot_pitch, target_pitch);

                // count-down loiter to pilot timer
                if (poshold.controller_to_pilot_timer_pitch > 0) {
                    poshold.controller_to_pilot_timer_pitch--;
                } else {
                    // when timer runs out switch to full pilot override for next iteration
                    poshold.pitch_mode = POSHOLD_PILOT_OVERRIDE;
                }

                // calculate controller_to_pilot mix ratio
                controller_to_pilot_pitch_mix = (float)poshold.controller_to_pilot_timer_pitch / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;

                // mix final loiter lean angle and pilot desired lean angles
                poshold.pitch = poshold_mix_controls(controller_to_pilot_pitch_mix, poshold.controller_final_pitch, poshold.pilot_pitch + poshold.wind_comp_pitch);
                break;
        }

        //
        // Shared roll & pitch states (POSHOLD_BRAKE_TO_LOITER and POSHOLD_LOITER)
        //

        // switch into LOITER mode when both roll and pitch are ready
        if (poshold.roll_mode == POSHOLD_BRAKE_READY_TO_LOITER && poshold.pitch_mode == POSHOLD_BRAKE_READY_TO_LOITER) {
            poshold.roll_mode = POSHOLD_BRAKE_TO_LOITER;
            poshold.pitch_mode = POSHOLD_BRAKE_TO_LOITER;
            poshold.brake_to_loiter_timer = POSHOLD_BRAKE_TO_LOITER_TIMER;
            // init loiter controller
            wp_nav.init_loiter_target(inertial_nav.get_position(), poshold.loiter_reset_I); // (false) to avoid I_term reset. In original code, velocity(0,0,0) was used instead of current velocity: wp_nav.init_loiter_target(inertial_nav.get_position(), Vector3f(0,0,0));
            // at this stage, we are going to run update_loiter that will reset I_term once. From now, we ensure next time that we will enter loiter and update it, I_term won't be reset anymore
            poshold.loiter_reset_I = false;
            // set delay to start of wind compensation estimate updates
            poshold.wind_comp_start_timer = POSHOLD_WIND_COMP_START_TIMER;
        }

        // roll-mode is used as the combined roll+pitch mode when in BRAKE_TO_LOITER or LOITER modes
        if (poshold.roll_mode == POSHOLD_BRAKE_TO_LOITER || poshold.roll_mode == POSHOLD_LOITER) {

            // force pitch mode to be same as roll_mode just to keep it consistent (it's not actually used in these states)
            poshold.pitch_mode = poshold.roll_mode;

            // handle combined roll+pitch mode
            switch (poshold.roll_mode) {
                case POSHOLD_BRAKE_TO_LOITER:
                    // reduce brake_to_loiter timer
                    if (poshold.brake_to_loiter_timer > 0) {
                        poshold.brake_to_loiter_timer--;
                    } else {
                        // progress to full loiter on next iteration
                        poshold.roll_mode = POSHOLD_LOITER;
                        poshold.pitch_mode = POSHOLD_LOITER;
                    }

                    // calculate percentage mix of loiter and brake control
                    brake_to_loiter_mix = (float)poshold.brake_to_loiter_timer / (float)POSHOLD_BRAKE_TO_LOITER_TIMER;

                    // calculate brake_roll and pitch angles to counter-act velocity
                    poshold_update_brake_angle_from_velocity(poshold.brake_roll, vel_right);
                    poshold_update_brake_angle_from_velocity(poshold.brake_pitch, -vel_fw);

                    // run loiter controller
                    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

                    // calculate final roll and pitch output by mixing loiter and brake controls
                    poshold.roll = poshold_mix_controls(brake_to_loiter_mix, poshold.brake_roll + poshold.wind_comp_roll, wp_nav.get_roll());
                    poshold.pitch = poshold_mix_controls(brake_to_loiter_mix, poshold.brake_pitch + poshold.wind_comp_pitch, wp_nav.get_pitch());

                    // check for pilot input
                    if (target_roll != 0 || target_pitch != 0) {
                        // if roll input switch to pilot override for roll
                        if (target_roll != 0) {
                            // init transition to pilot override
                            poshold_roll_controller_to_pilot_override();
                            // switch pitch-mode to brake (but ready to go back to loiter anytime)
                            // no need to reset poshold.brake_pitch here as wind comp has not been updated since last brake_pitch computation
                            poshold.pitch_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                        }
                        // if pitch input switch to pilot override for pitch
                        if (target_pitch != 0) {
                            // init transition to pilot override
                            poshold_pitch_controller_to_pilot_override();
                            if (target_roll == 0) {
                                // switch roll-mode to brake (but ready to go back to loiter anytime)
                                // no need to reset poshold.brake_roll here as wind comp has not been updated since last brake_roll computation
                                poshold.roll_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                            }
                        }
                    }
                    break;

                case POSHOLD_LOITER:
                    // run loiter controller
                    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

                    // set roll angle based on loiter controller outputs
                    poshold.roll = wp_nav.get_roll();
                    poshold.pitch = wp_nav.get_pitch();

                    // update wind compensation estimate
                    poshold_update_wind_comp_estimate();

                    // check for pilot input
                    if (target_roll != 0 || target_pitch != 0) {
                        // if roll input switch to pilot override for roll
                        if (target_roll != 0) {
                            // init transition to pilot override
                            poshold_roll_controller_to_pilot_override();
                            // switch pitch-mode to brake (but ready to go back to loiter anytime)
                            poshold.pitch_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                            // reset brake_pitch because wind_comp is now different and should give the compensation of the whole previous loiter angle
                            poshold.brake_pitch = 0;
                        }
                        // if pitch input switch to pilot override for pitch
                        if (target_pitch != 0) {
                            // init transition to pilot override
                            poshold_pitch_controller_to_pilot_override();
                            // if roll not overriden switch roll-mode to brake (but be ready to go back to loiter any time)
                            if (target_roll == 0) {
                                poshold.roll_mode = POSHOLD_BRAKE_READY_TO_LOITER;
                                poshold.brake_roll = 0;
                            }
                        }
                    }
                    break;

                default:
                    // do nothing for uncombined roll and pitch modes
                    break;
            }
        }
        
        // constrain target pitch/roll angles
        poshold.roll = constrain_int16(poshold.roll, -aparm.angle_max, aparm.angle_max);
        poshold.pitch = constrain_int16(poshold.pitch, -aparm.angle_max, aparm.angle_max);

        // update attitude controller targets
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(poshold.roll, poshold.pitch, target_yaw_rate);

        // throttle control
        if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
            // if sonar is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }
        // update altitude target and call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
    }
}

// poshold_update_pilot_lean_angle - update the pilot's filtered lean angle with the latest raw input received
static void poshold_update_pilot_lean_angle(float &lean_angle_filtered, float &lean_angle_raw)
{
    // if raw input is large or reversing the vehicle's lean angle immediately set the fitlered angle to the new raw angle
    if ((lean_angle_filtered > 0 && lean_angle_raw < 0) || (lean_angle_filtered < 0 && lean_angle_raw > 0) || (abs(lean_angle_raw) > POSHOLD_STICK_RELEASE_SMOOTH_ANGLE)) {
        lean_angle_filtered = lean_angle_raw;
    } else {
        // lean_angle_raw must be pulling lean_angle_filtered towards zero, smooth the decrease
        if (lean_angle_filtered > 0) {
            // reduce the filtered lean angle at 5% or the brake rate (whichever is faster).
            lean_angle_filtered -= max((float)lean_angle_filtered * POSHOLD_SMOOTH_RATE_FACTOR, max(1, g.poshold_brake_rate/LOOP_RATE_FACTOR));
            // do not let the filtered angle fall below the pilot's input lean angle.
            // the above line pulls the filtered angle down and the below line acts as a catch
            lean_angle_filtered = max(lean_angle_filtered, lean_angle_raw);
        }else{
            lean_angle_filtered += max(-(float)lean_angle_filtered * POSHOLD_SMOOTH_RATE_FACTOR, max(1, g.poshold_brake_rate/LOOP_RATE_FACTOR));
            lean_angle_filtered = min(lean_angle_filtered, lean_angle_raw);
        }
    }
}

// poshold_mix_controls - mixes two controls based on the mix_ratio
//  mix_ratio of 1 = use first_control completely, 0 = use second_control completely, 0.5 = mix evenly
static int16_t poshold_mix_controls(float mix_ratio, int16_t first_control, int16_t second_control)
{
    mix_ratio = constrain_float(mix_ratio, 0.0f, 1.0f);
    return (int16_t)((mix_ratio * first_control) + ((1.0f-mix_ratio)*second_control));
}

// poshold_update_brake_angle_from_velocity - updates the brake_angle based on the vehicle's velocity and brake_gain
//  brake_angle is slewed with the wpnav.poshold_brake_rate and constrained by the wpnav.poshold_braking_angle_max
//  velocity is assumed to be in the same direction as lean angle so for pitch you should provide the velocity backwards (i.e. -ve forward velocity)
static void poshold_update_brake_angle_from_velocity(int16_t &brake_angle, float velocity)
{
    float lean_angle;
    int16_t brake_rate = g.poshold_brake_rate;

#if MAIN_LOOP_RATE == 400
    brake_rate /= 4;
    if (brake_rate <= 0) {
        brake_rate = 1;
    }
#endif

    // calculate velocity-only based lean angle
    if (velocity >= 0) {
        lean_angle = -poshold.brake_gain * velocity * (1.0f+500.0f/(velocity+60.0f));
    } else {
        lean_angle = -poshold.brake_gain * velocity * (1.0f+500.0f/(-velocity+60.0f));
    }

    // do not let lean_angle be too far from brake_angle
    brake_angle = constrain_int16((int16_t)lean_angle, brake_angle - brake_rate, brake_angle + brake_rate);

    // constrain final brake_angle
    brake_angle = constrain_int16(brake_angle, -g.poshold_brake_angle_max, g.poshold_brake_angle_max);
}

// poshold_update_wind_comp_estimate - updates wind compensation estimate
//  should be called at the maximum loop rate when loiter is engaged
static void poshold_update_wind_comp_estimate()
{
    // check wind estimate start has not been delayed
    if (poshold.wind_comp_start_timer > 0) {
        poshold.wind_comp_start_timer--;
        return;
    }

    // check horizontal velocity is low
    if (inertial_nav.get_velocity_xy() > POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX) {
        return;
    }

    // get position controller accel target
    //  To-Do: clean this up by using accessor in loiter controller (or move entire PosHold controller to a library shared with loiter)
    const Vector3f& accel_target = pos_control.get_accel_target();

    // update wind compensation in earth-frame lean angles
    if (poshold.wind_comp_ef.x == 0) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        poshold.wind_comp_ef.x = accel_target.x;
    } else {
        // low pass filter the position controller's lean angle output
        poshold.wind_comp_ef.x = (1.0f-TC_WIND_COMP)*poshold.wind_comp_ef.x + TC_WIND_COMP*accel_target.x;
    }
    if (poshold.wind_comp_ef.y == 0) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        poshold.wind_comp_ef.y = accel_target.y;
    } else {
        // low pass filter the position controller's lean angle output
        poshold.wind_comp_ef.y = (1.0f-TC_WIND_COMP)*poshold.wind_comp_ef.y + TC_WIND_COMP*accel_target.y;
    }
}

// poshold_get_wind_comp_lean_angles - retrieve wind compensation angles in body frame roll and pitch angles
//  should be called at the maximum loop rate
static void poshold_get_wind_comp_lean_angles(int16_t &roll_angle, int16_t &pitch_angle)
{
    // reduce rate to 10hz
    poshold.wind_comp_timer++;
    if (poshold.wind_comp_timer < POSHOLD_WIND_COMP_TIMER_10HZ) {
        return;
    }
    poshold.wind_comp_timer = 0;

    // convert earth frame desired accelerations to body frame roll and pitch lean angles
    roll_angle = (float)fast_atan((-poshold.wind_comp_ef.x*ahrs.sin_yaw() + poshold.wind_comp_ef.y*ahrs.cos_yaw())/981)*(18000/M_PI);
    pitch_angle = (float)fast_atan(-(poshold.wind_comp_ef.x*ahrs.cos_yaw() + poshold.wind_comp_ef.y*ahrs.sin_yaw())/981)*(18000/M_PI);
}

// poshold_roll_controller_to_pilot_override - initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
static void poshold_roll_controller_to_pilot_override()
{
    poshold.roll_mode = POSHOLD_CONTROLLER_TO_PILOT_OVERRIDE;
    poshold.controller_to_pilot_timer_roll = POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;
    // initialise pilot_roll to 0, wind_comp will be updated to compensate and poshold_update_pilot_lean_angle function shall not smooth this transition at next iteration. so 0 is the right value
    poshold.pilot_roll = 0;
    // store final controller output for mixing with pilot input
    poshold.controller_final_roll = poshold.roll;
}

// poshold_pitch_controller_to_pilot_override - initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
static void poshold_pitch_controller_to_pilot_override()
{
    poshold.pitch_mode = POSHOLD_CONTROLLER_TO_PILOT_OVERRIDE;
    poshold.controller_to_pilot_timer_pitch = POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;
    // initialise pilot_pitch to 0, wind_comp will be updated to compensate and poshold_update_pilot_lean_angle function shall not smooth this transition at next iteration. so 0 is the right value
    poshold.pilot_pitch = 0;
    // store final loiter outputs for mixing with pilot input
    poshold.controller_final_pitch = poshold.pitch;
}

#endif  // POSHOLD_ENABLED == ENABLED
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/control_rtl.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_rtl.pde - init and run calls for RTL flight mode
 *
 * There are two parts to RTL, the high level decision making which controls which state we are in
 * and the lower implementation of the waypoint or landing controllers within those states
 */

// rtl_init - initialise rtl controller
static bool rtl_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {
        rtl_climb_start();
        return true;
    }else{
        return false;
    }
}

// rtl_run - runs the return-to-launch controller
// should be called at 100hz or more
static void rtl_run()
{
    // check if we need to move to next state
    if (rtl_state_complete) {
        switch (rtl_state) {
        case InitialClimb:
            rtl_return_start();
            break;
        case ReturnHome:
            rtl_loiterathome_start();
            break;
        case LoiterAtHome:
            if (g.rtl_alt_final > 0 && !failsafe.radio) {
                rtl_descent_start();
            }else{
                rtl_land_start();
            }
            break;
        case FinalDescent:
            // do nothing
            break;
        case Land:
            // do nothing - rtl_land_run will take care of disarming motors
            break;
        }
    }

    // call the correct run function
    switch (rtl_state) {

    case InitialClimb:
        rtl_climb_return_run();
        break;

    case ReturnHome:
        rtl_climb_return_run();
        break;

    case LoiterAtHome:
        rtl_loiterathome_run();
        break;

    case FinalDescent:
        rtl_descent_run();
        break;

    case Land:
        rtl_land_run();
        break;
    }
}

// rtl_climb_start - initialise climb to RTL altitude
static void rtl_climb_start()
{
    rtl_state = InitialClimb;
    rtl_state_complete = false;

    // initialise waypoint and spline controller
    wp_nav.wp_and_spline_init();

    // get horizontal stopping point
    Vector3f destination;
    wp_nav.get_wp_stopping_point_xy(destination);

#if AC_RALLY == ENABLED
    // rally_point.alt will be the altitude of the nearest rally point or the RTL_ALT. uses absolute altitudes
    Location rally_point = rally.calc_best_rally_or_home_location(current_loc, get_RTL_alt()+ahrs.get_home().alt);
    rally_point.alt -= ahrs.get_home().alt; // convert to altitude above home
    rally_point.alt = max(rally_point.alt, current_loc.alt);    // ensure we do not descend before reaching home
    destination.z = pv_alt_above_origin(rally_point.alt);
#else
    destination.z = pv_alt_above_origin(get_RTL_alt());
#endif

    // set the destination
    wp_nav.set_wp_destination(destination);
    wp_nav.set_fast_waypoint(true);

    // hold current yaw during initial climb
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// rtl_return_start - initialise return to home
static void rtl_return_start()
{
    rtl_state = ReturnHome;
    rtl_state_complete = false;

    // set target to above home/rally point
#if AC_RALLY == ENABLED
    // rally_point will be the nearest rally point or home.  uses absolute altitudes
    Location rally_point = rally.calc_best_rally_or_home_location(current_loc, get_RTL_alt()+ahrs.get_home().alt);
    rally_point.alt -= ahrs.get_home().alt; // convert to altitude above home
    rally_point.alt = max(rally_point.alt, current_loc.alt);    // ensure we do not descend before reaching home
    Vector3f destination = pv_location_to_vector(rally_point);
#else
    Vector3f destination = pv_location_to_vector(ahrs.get_home());
    destination.z = get_RTL_alt();
#endif

    wp_nav.set_wp_destination(destination);

    // initialise yaw to point home (maybe)
    set_auto_yaw_mode(get_default_auto_yaw_mode(true));
}

// rtl_climb_return_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by rtl_run at 100hz or more
static void rtl_climb_return_run()
{
    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        // reset attitude control targets
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        // To-Do: re-initialise wpnav targets
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
        if (target_yaw_rate != 0) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(),true);
    }

    // check if we've completed this stage of RTL
    rtl_state_complete = wp_nav.reached_wp_destination();
}

// rtl_return_start - initialise return to home
static void rtl_loiterathome_start()
{
    rtl_state = LoiterAtHome;
    rtl_state_complete = false;
    rtl_loiter_start_time = millis();

    // yaw back to initial take-off heading yaw unless pilot has already overridden yaw
    if(get_default_auto_yaw_mode(true) != AUTO_YAW_HOLD) {
        set_auto_yaw_mode(AUTO_YAW_RESETTOARMEDYAW);
    } else {
        set_auto_yaw_mode(AUTO_YAW_HOLD);
    }
}

// rtl_climb_return_descent_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by rtl_run at 100hz or more
static void rtl_loiterathome_run()
{
    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        // reset attitude control targets
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        // To-Do: re-initialise wpnav targets
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
        if (target_yaw_rate != 0) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(),true);
    }

    // check if we've completed this stage of RTL
    if ((millis() - rtl_loiter_start_time) >= (uint32_t)g.rtl_loiter_time.get()) {
        if (auto_yaw_mode == AUTO_YAW_RESETTOARMEDYAW) {
            // check if heading is within 2 degrees of heading when vehicle was armed
            if (labs(wrap_180_cd(ahrs.yaw_sensor-initial_armed_bearing)) <= 200) {
                rtl_state_complete = true;
            }
        } else {
            // we have loitered long enough
            rtl_state_complete = true;
        }
    }
}

// rtl_descent_start - initialise descent to final alt
static void rtl_descent_start()
{
    rtl_state = FinalDescent;
    rtl_state_complete = false;

    // Set wp navigation target to above home
    wp_nav.init_loiter_target(wp_nav.get_wp_destination());

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// rtl_descent_run - implements the final descent to the RTL_ALT
//      called by rtl_run at 100hz or more
static void rtl_descent_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        // set target to current position
        wp_nav.init_loiter_target();
        return;
    }

    // process pilot's input
    if (!failsafe.radio) {
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // process pilot's roll and pitch input
            roll_control = g.rc_1.control_in;
            pitch_control = g.rc_2.control_in;
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

    // process roll, pitch inputs
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

    // run loiter controller
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call z-axis position controller
    pos_control.set_alt_target_with_slew(pv_alt_above_origin(g.rtl_alt_final), G_Dt);
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

    // check if we've reached within 20cm of final altitude
    rtl_state_complete = fabs(pv_alt_above_origin(g.rtl_alt_final) - inertial_nav.get_altitude()) < 20.0f;
}

// rtl_loiterathome_start - initialise controllers to loiter over home
static void rtl_land_start()
{
    rtl_state = Land;
    rtl_state_complete = false;

    // Set wp navigation target to above home
    wp_nav.init_loiter_target(wp_nav.get_wp_destination());

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// rtl_returnhome_run - return home
//      called by rtl_run at 100hz or more
static void rtl_land_run()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;
    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.land_complete) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        // set target to current position
        wp_nav.init_loiter_target();

#if LAND_REQUIRE_MIN_THROTTLE_TO_DISARM == ENABLED
        // disarm when the landing detector says we've landed and throttle is at minimum
        if (ap.land_complete && (ap.throttle_zero || failsafe.radio)) {
            init_disarm_motors();
        }
#else
        // disarm when the landing detector says we've landed
        if (ap.land_complete) {
            init_disarm_motors();
        }
#endif

        // check if we've completed this stage of RTL
        rtl_state_complete = ap.land_complete;
        return;
    }

    // relax loiter target if we might be landed
    if (land_complete_maybe()) {
        wp_nav.loiter_soften_for_landing();
    }

    // process pilot's input
    if (!failsafe.radio) {
        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // process pilot's roll and pitch input
            roll_control = g.rc_1.control_in;
            pitch_control = g.rc_2.control_in;
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
    }

     // process pilot's roll and pitch input
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

    // run loiter controller
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call z-axis position controller
    float cmb_rate = get_land_descent_speed();
    pos_control.set_alt_target_from_climb_rate(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();

    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

    // check if we've completed this stage of RTL
    rtl_state_complete = ap.land_complete;
}

// get_RTL_alt - return altitude which vehicle should return home at
//      altitude is in cm above home
static float get_RTL_alt()
{
    // maximum of current altitude and rtl altitude
    float rtl_alt = max(current_loc.alt, g.rtl_altitude);

#if AC_FENCE == ENABLED
    // ensure not above fence altitude if alt fence is enabled
    if ((fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        rtl_alt = min(rtl_alt, pv_alt_above_origin(fence.get_safe_alt()*100.0f));
    }
#endif

    return rtl_alt;
}

#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/control_sport.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_sport.pde - init and run calls for sport flight mode
 */

// sport_init - initialise sport controller
static bool sport_init(bool ignore_checks)
{
    // initialize vertical speed and accelerationj
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    return true;
}

// sport_run - runs the sport controller
// should be called at 100hz or more
static void sport_run()
{
    float target_roll_rate, target_pitch_rate, target_yaw_rate;
    float target_climb_rate = 0;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || g.rc_3.control_in <= 0) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(g.rc_3.control_in)-throttle_average);
        return;
    }

    // apply SIMPLE mode transform
    update_simple_mode();

    // get pilot's desired roll and pitch rates

    // calculate rate requests
    target_roll_rate = g.rc_1.control_in * g.acro_rp_p;
    target_pitch_rate = g.rc_2.control_in * g.acro_rp_p;

    int32_t roll_angle = wrap_180_cd(ahrs.roll_sensor);
    target_roll_rate -= constrain_int32(roll_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_roll;

    // Calculate trainer mode earth frame rate command for pitch
    int32_t pitch_angle = wrap_180_cd(ahrs.pitch_sensor);
    target_pitch_rate -= constrain_int32(pitch_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_pitch;

    if (roll_angle > aparm.angle_max){
        target_roll_rate -=  g.acro_rp_p*(roll_angle-aparm.angle_max);
    }else if (roll_angle < -aparm.angle_max) {
        target_roll_rate -=  g.acro_rp_p*(roll_angle+aparm.angle_max);
    }

    if (pitch_angle > aparm.angle_max){
        target_pitch_rate -=  g.acro_rp_p*(pitch_angle-aparm.angle_max);
    }else if (pitch_angle < -aparm.angle_max) {
        target_pitch_rate -=  g.acro_rp_p*(pitch_angle+aparm.angle_max);
    }

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

    // get pilot desired climb rate
    target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

    // check for pilot requested take-off
    if (ap.land_complete && target_climb_rate > 0) {
        // indicate we are taking off
        set_land_complete(false);
        // clear i term when we're taking off
        set_throttle_takeoff();
    }

    // reset target lean angles and heading while landed
    if (ap.land_complete) {
        // move throttle to between minimum and non-takeoff-throttle to keep us on the ground
        attitude_control.set_throttle_out_unstabilized(get_throttle_pre_takeoff(g.rc_3.control_in),true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(g.rc_3.control_in)-throttle_average);
    }else{

        // call attitude controller
        attitude_control.rate_ef_roll_pitch_yaw(target_roll_rate, target_pitch_rate, target_yaw_rate);

        // call throttle controller
        if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
            // if sonar is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
    }
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/control_stabilize.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_stabilize.pde - init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
static bool stabilize_init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    // To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
    pos_control.set_alt_target(0);

    // stabilize should never be made to fail
    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
static void stabilize_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || g.rc_3.control_in <= 0) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/control_stop.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_stop.pde - init and run calls for stop flight mode
 */

// stop_init - initialise stop controller
static bool stop_init(bool ignore_checks)
{
    if (position_ok() || optflow_position_ok() || ignore_checks) {

        // set desired acceleration to zero
        wp_nav.clear_pilot_desired_acceleration();

        // set target to current position
        wp_nav.init_stop_target(STOP_MODE_DECEL_RATE);

        // initialize vertical speed and acceleration
        pos_control.set_speed_z(0, 0);
        pos_control.set_accel_z(STOP_MODE_DECEL_RATE);

        // initialise altitude target to stopping point
        pos_control.set_target_to_stopping_point_z();

        return true;
    }else{
        return false;
    }
}

// stop_run - runs the stop controller
// should be called at 100hz or more
static void stop_run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        wp_nav.init_stop_target(STOP_MODE_DECEL_RATE);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(0)-throttle_average);
        return;
    }

    if (ap.land_complete) {
        // ToDo: What do we do if we are landed?  Disarm?)
    }

    // relax stop target if we might be landed
    if (land_complete_maybe()) {
        // ToDo: What do we do here?
    }

    // run stop controller
    wp_nav.update_stop(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);

    // body-frame rate controller is run directly from 100hz loop

    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
    pos_control.update_z_controller();
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/crash_check.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Code to detect a crash main ArduCopter code
#ifndef CRASH_CHECK_ITERATIONS_MAX
 # define CRASH_CHECK_ITERATIONS_MAX        20      // 2 second (ie. 10 iterations at 10hz) inverted indicates a crash
#endif
#ifndef CRASH_CHECK_ANGLE_DEVIATION_CD
 # define CRASH_CHECK_ANGLE_DEVIATION_CD    2000    // 20 degrees beyond angle max is signal we are inverted
#endif
#ifndef CRASH_CHECK_ALT_CHANGE_LIMIT_CM
 # define CRASH_CHECK_ALT_CHANGE_LIMIT_CM   50      // baro altitude must not change by more than 50cm
#endif

// crash_check - disarms motors if a crash has been detected
// crashes are detected by the vehicle being more than 20 degrees beyond it's angle limits continuously for more than 1 second
// should be called at 10hz
void crash_check()
{
    static uint8_t inverted_count;  // number of iterations we have been inverted
    static int32_t baro_alt_prev;

#if PARACHUTE == ENABLED
    // check parachute
    parachute_check();
#endif

    // return immediately if motors are not armed or pilot's throttle is above zero
    if (!motors.armed() || (!ap.throttle_zero && !failsafe.radio)) {
        inverted_count = 0;
        return;
    }

    // return immediately if we are not in an angle stabilize flight mode or we are flipping
    if (control_mode == ACRO || control_mode == FLIP) {
        inverted_count = 0;
        return;
    }

    // check angles
    int32_t lean_max = aparm.angle_max + CRASH_CHECK_ANGLE_DEVIATION_CD;
    if (labs(ahrs.roll_sensor) > lean_max || labs(ahrs.pitch_sensor) > lean_max) {
        inverted_count++;

        // if we have just become inverted record the baro altitude
        if (inverted_count == 1) {
            baro_alt_prev = baro_alt;

        // exit if baro altitude change indicates we are moving (probably falling)
        }else if (labs(baro_alt - baro_alt_prev) > CRASH_CHECK_ALT_CHANGE_LIMIT_CM) {
            inverted_count = 0;
            return;

        // check if inverted for 2 seconds
        }else if (inverted_count >= CRASH_CHECK_ITERATIONS_MAX) {
            // log an error in the dataflash
            Log_Write_Error(ERROR_SUBSYSTEM_CRASH_CHECK, ERROR_CODE_CRASH_CHECK_CRASH);
            // send message to gcs
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Crash: Disarming"));
            // disarm motors
            init_disarm_motors();
        }
    }else{
        // we are not inverted so reset counter
        inverted_count = 0;
    }
}

#if PARACHUTE == ENABLED

// Code to detect a crash main ArduCopter code
#ifndef PARACHUTE_CHECK_ITERATIONS_MAX
 # define PARACHUTE_CHECK_ITERATIONS_MAX        10      // 1 second (ie. 10 iterations at 10hz) of loss of control triggers the parachute
#endif
#ifndef PARACHUTE_CHECK_ANGLE_DEVIATION_CD
 # define PARACHUTE_CHECK_ANGLE_DEVIATION_CD    3000    // 30 degrees off from target indicates a loss of control
#endif

// parachute_check - disarms motors and triggers the parachute if serious loss of control has been detected
// vehicle is considered to have a "serious loss of control" by the vehicle being more than 30 degrees off from the target roll and pitch angles continuously for 1 second
// should be called at 10hz
void parachute_check()
{
    static uint8_t control_loss_count;	// number of iterations we have been out of control
    static int32_t baro_alt_start;

    // exit immediately if parachute is not enabled
    if (!parachute.enabled()) {
        return;
    }

    // call update to give parachute a chance to move servo or relay back to off position
    parachute.update();

    // return immediately if motors are not armed or pilot's throttle is above zero
    if (!motors.armed()) {
        control_loss_count = 0;
        return;
    }

    // return immediately if we are not in an angle stabilize flight mode or we are flipping
    if (control_mode == ACRO || control_mode == FLIP) {
        control_loss_count = 0;
        return;
    }

    // ensure we are flying
    if (ap.land_complete) {
        control_loss_count = 0;
        return;
    }

    // ensure the first control_loss event is from above the min altitude
    if (control_loss_count == 0 && parachute.alt_min() != 0 && (baro_alt < (int32_t)parachute.alt_min() * 100)) {
        return;
    }

    // get desired lean angles
    const Vector3f& target_angle = attitude_control.angle_ef_targets();

    // check roll and pitch angles
    if (labs(ahrs.roll_sensor - target_angle.x) > CRASH_CHECK_ANGLE_DEVIATION_CD ||
        labs(ahrs.pitch_sensor - target_angle.y) > CRASH_CHECK_ANGLE_DEVIATION_CD) {
        control_loss_count++;

        // don't let control_loss_count get too high
        if (control_loss_count > PARACHUTE_CHECK_ITERATIONS_MAX) {
            control_loss_count = PARACHUTE_CHECK_ITERATIONS_MAX;
        }

        // record baro alt if we have just started losing control
        if (control_loss_count == 1) {
            baro_alt_start = baro_alt;

        // exit if baro altitude change indicates we are not falling
        }else if (baro_alt >= baro_alt_start) {
            control_loss_count = 0;
            return;

        // To-Do: add check that the vehicle is actually falling

        // check if loss of control for at least 1 second
        }else if (control_loss_count >= PARACHUTE_CHECK_ITERATIONS_MAX) {
            // reset control loss counter
            control_loss_count = 0;
            // log an error in the dataflash
            Log_Write_Error(ERROR_SUBSYSTEM_CRASH_CHECK, ERROR_CODE_CRASH_CHECK_LOSS_OF_CONTROL);
            // release parachute
            parachute_release();
        }
    }else{
        // we are not inverted so reset counter
        control_loss_count = 0;
    }
}

// parachute_release - trigger the release of the parachute, disarm the motors and notify the user
static void parachute_release()
{
    // send message to gcs and dataflash
    gcs_send_text_P(SEVERITY_HIGH,PSTR("Parachute: Released!"));
    Log_Write_Event(DATA_PARACHUTE_RELEASED);

    // disarm motors
    init_disarm_motors();

    // release parachute
    parachute.release();
}

// parachute_manual_release - trigger the release of the parachute, after performing some checks for pilot error
//   checks if the vehicle is landed 
static void parachute_manual_release()
{
    // exit immediately if parachute is not enabled
    if (!parachute.enabled()) {
        return;
    }

    // do not release if we are landed or below the minimum altitude above home
    if (ap.land_complete || (parachute.alt_min() != 0 && (baro_alt < (int32_t)parachute.alt_min() * 100))) {
        // warn user of reason for failure
        gcs_send_text_P(SEVERITY_HIGH,PSTR("Parachute: Too Low"));
        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_PARACHUTE, ERROR_CODE_PARACHUTE_TOO_LOW);
        return;
    }

    // if we get this far release parachute
    parachute_release();
}

#endif // PARACHUTE == ENABLED
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/ekf_check.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/**
 *
 * ekf_check.pde - detects failures of the ekf or inertial nav system
 *                 triggers an alert to the pilot and helps take countermeasures
 *
 */

#ifndef EKF_CHECK_ITERATIONS_MAX
 # define EKF_CHECK_ITERATIONS_MAX          10      // 1 second (ie. 10 iterations at 10hz) of bad variances signals a failure
#endif

#ifndef EKF_CHECK_WARNING_TIME
 # define EKF_CHECK_WARNING_TIME            (30*1000)   // warning text messages are sent to ground no more than every 30 seconds
#endif

////////////////////////////////////////////////////////////////////////////////
// EKF_check strucutre
////////////////////////////////////////////////////////////////////////////////
static struct {
    uint8_t fail_count;         // number of iterations ekf or dcm have been out of tolerances
    uint8_t bad_variance : 1;   // true if ekf should be considered untrusted (fail_count has exceeded EKF_CHECK_ITERATIONS_MAX)
    uint32_t last_warn_time;    // system time of last warning in milliseconds.  Used to throttle text warnings sent to GCS
} ekf_check_state;

// ekf_check - detects if ekf variance are out of tolerance and triggers failsafe
// should be called at 10hz
void ekf_check()
{
    // return immediately if motors are not armed, ekf check is disabled, not using ekf or usb is connected
    if (!motors.armed() || ap.usb_connected || (g.ekfcheck_thresh <= 0.0f)) {
        ekf_check_state.fail_count = 0;
        ekf_check_state.bad_variance = false;
        AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
        failsafe_ekf_off_event();   // clear failsafe
        return;
    }

    // compare compass and velocity variance vs threshold
    if (ekf_over_threshold()) {
        // if compass is not yet flagged as bad
        if (!ekf_check_state.bad_variance) {
            // increase counter
            ekf_check_state.fail_count++;
            // if counter above max then trigger failsafe
            if (ekf_check_state.fail_count >= EKF_CHECK_ITERATIONS_MAX) {
                // limit count from climbing too high
                ekf_check_state.fail_count = EKF_CHECK_ITERATIONS_MAX;
                ekf_check_state.bad_variance = true;
                // log an error in the dataflash
                Log_Write_Error(ERROR_SUBSYSTEM_EKFCHECK, ERROR_CODE_EKFCHECK_BAD_VARIANCE);
                // send message to gcs
                if ((hal.scheduler->millis() - ekf_check_state.last_warn_time) > EKF_CHECK_WARNING_TIME) {
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("EKF variance"));
                    ekf_check_state.last_warn_time = hal.scheduler->millis();
                }
                failsafe_ekf_event();
            }
        }
    } else {
        // reduce counter
        if (ekf_check_state.fail_count > 0) {
            ekf_check_state.fail_count--;

            // if compass is flagged as bad and the counter reaches zero then clear flag
            if (ekf_check_state.bad_variance && ekf_check_state.fail_count == 0) {
                ekf_check_state.bad_variance = false;
                // log recovery in the dataflash
                Log_Write_Error(ERROR_SUBSYSTEM_EKFCHECK, ERROR_CODE_EKFCHECK_VARIANCE_CLEARED);
                // clear failsafe
                failsafe_ekf_off_event();
            }
        }
    }

    // set AP_Notify flags
    AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;

    // To-Do: add ekf variances to extended status
}

// ekf_over_threshold - returns true if the ekf's variance are over the tolerance
static bool ekf_over_threshold()
{
#if AP_AHRS_NAVEKF_AVAILABLE
    // return false immediately if disabled
    if (g.ekfcheck_thresh <= 0.0f) {
        return false;
    }

    // return true immediately if position is bad
    if (!position_ok() && !optflow_position_ok()) {
        return true;
    }

    // use EKF to get variance
    float posVar, hgtVar, tasVar;
    Vector3f magVar;
    Vector2f offset;
    float compass_variance;
    float vel_variance;
    ahrs.get_NavEKF().getVariances(vel_variance, posVar, hgtVar, magVar, tasVar, offset);
    compass_variance = magVar.length();

    // return true if compass and velocity variance over the threshold
    return (compass_variance >= g.ekfcheck_thresh && vel_variance >= g.ekfcheck_thresh);
#else
    return false;
#endif
}


// failsafe_ekf_event - perform ekf failsafe
static void failsafe_ekf_event()
{
    // return immediately if ekf failsafe already triggered
    if (failsafe.ekf) {
        return;
    }

    // do nothing if motors disarmed or not in flight mode that requires GPS
    if (!motors.armed() || !mode_requires_GPS(control_mode)) {
        return;
    }

    // EKF failsafe event has occurred
    failsafe.ekf = true;
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_EKFINAV, ERROR_CODE_FAILSAFE_OCCURRED);

    // take action based on flight mode
    if (mode_requires_GPS(control_mode)) {
        set_mode_land_with_pause();
    }

    // if flight mode is LAND ensure it's not the GPS controlled LAND
    if (control_mode == LAND) {
        land_do_not_use_GPS();
    }
}

// failsafe_ekf_off_event - actions to take when EKF failsafe is cleared
static void failsafe_ekf_off_event(void)
{
    // return immediately if not in ekf failsafe
    if (!failsafe.ekf) {
        return;
    }

    // clear flag and log recovery
    failsafe.ekf = false;
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_EKFINAV, ERROR_CODE_FAILSAFE_RESOLVED);
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/esc_calibration.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*****************************************************************************
*  esc_calibration.pde : functions to check and perform ESC calibration
*****************************************************************************/

#define ESC_CALIBRATION_HIGH_THROTTLE   950

// enum for ESC CALIBRATION
enum ESCCalibrationModes {
    ESCCAL_NONE = 0,
    ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH = 1,
    ESCCAL_PASSTHROUGH_ALWAYS = 2,
    ESCCAL_AUTO = 3
};

// check if we should enter esc calibration mode
static void esc_calibration_startup_check()
{
    // exit immediately if pre-arm rc checks fail
    pre_arm_rc_checks();
    if (!ap.pre_arm_rc_check) {
        // clear esc flag for next time
        if (g.esc_calibrate != ESCCAL_NONE) {
            g.esc_calibrate.set_and_save(ESCCAL_NONE);
        }
        return;
    }

    // check ESC parameter
    switch (g.esc_calibrate) {
        case ESCCAL_NONE:
            // check if throttle is high
            if (g.rc_3.control_in >= ESC_CALIBRATION_HIGH_THROTTLE) {
                // we will enter esc_calibrate mode on next reboot
                g.esc_calibrate.set_and_save(ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH);
                // send message to gcs
                gcs_send_text_P(SEVERITY_HIGH,PSTR("ESC Calibration: restart board"));
                // turn on esc calibration notification
                AP_Notify::flags.esc_calibration = true;
                // block until we restart
                while(1) { delay(5); }
            }
            break;
        case ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH:
            // check if throttle is high
            if (g.rc_3.control_in >= ESC_CALIBRATION_HIGH_THROTTLE) {
                // pass through pilot throttle to escs
                esc_calibration_passthrough();
            }
            break;
        case ESCCAL_PASSTHROUGH_ALWAYS:
            // pass through pilot throttle to escs
            esc_calibration_passthrough();
            break;
        case ESCCAL_AUTO:
            // perform automatic ESC calibration
            esc_calibration_auto();
            break;
        default:
            // do nothing
            break;
    }
    // clear esc flag for next time
    g.esc_calibrate.set_and_save(ESCCAL_NONE);
}

// esc_calibration_passthrough - pass through pilot throttle to escs
static void esc_calibration_passthrough()
{
    // clear esc flag for next time
    g.esc_calibrate.set_and_save(ESCCAL_NONE);

    // reduce update rate to motors to 50Hz
    motors.set_update_rate(50);

    // send message to GCS
    gcs_send_text_P(SEVERITY_HIGH,PSTR("ESC Calibration: passing pilot throttle to ESCs"));

    while(1) {
        // arm motors
        motors.armed(true);
        motors.enable();

        // flash LEDS
        AP_Notify::flags.esc_calibration = true;

        // read pilot input
        read_radio();
        delay(10);

        // pass through to motors
        motors.throttle_pass_through(g.rc_3.radio_in);
    }
}

// esc_calibration_auto - calibrate the ESCs automatically using a timer and no pilot input
static void esc_calibration_auto()
{
    bool printed_msg = false;

    // reduce update rate to motors to 50Hz
    motors.set_update_rate(50);

    // send message to GCS
    gcs_send_text_P(SEVERITY_HIGH,PSTR("ESC Calibration: auto calibration"));

    // arm and enable motors
    motors.armed(true);
    motors.enable();

    // flash LEDS
    AP_Notify::flags.esc_calibration = true;

    // raise throttle to maximum
    delay(10);
    motors.throttle_pass_through(g.rc_3.radio_max);

    // wait for safety switch to be pressed
    while (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        if (!printed_msg) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("ESC Calibration: push safety switch"));
            printed_msg = true;
        }
        delay(10);
    }

    // delay for 5 seconds
    delay(5000);

    // reduce throttle to minimum
    motors.throttle_pass_through(g.rc_3.radio_min);

    // clear esc parameter
    g.esc_calibrate.set_and_save(ESCCAL_NONE);

    // block until we restart
    while(1) { delay(5); }
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/events.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */
static void failsafe_radio_on_event()
{
    // if motors are not armed there is nothing to do
    if( !motors.armed() ) {
        return;
    }

    // This is how to handle a failsafe.
    switch(control_mode) {
        case STABILIZE:
        case ACRO:
            // if throttle is zero OR vehicle is landed disarm motors
            if (ap.throttle_zero || ap.land_complete) {
                init_disarm_motors();

            // if failsafe_throttle is FS_THR_ENABLED_ALWAYS_LAND then land immediately
            }else if(g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                set_mode_land_with_pause();

            // if far from home then RTL
            }else if(home_distance > wp_nav.get_wp_radius()) {
                // switch to RTL or if that fails, LAND
                set_mode_RTL_or_land_with_pause();

            // We have no GPS or are very close to home so we will land
            }else{
                set_mode_land_with_pause();
            }
            break;

        case AUTO:
            // if mission has not started AND vehicle is landed, disarm motors
            if (!ap.auto_armed && ap.land_complete) {
                init_disarm_motors();

            // if failsafe_throttle is FS_THR_ENABLED_ALWAYS_LAND then land immediately
            } else if(g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                set_mode_land_with_pause();

            // if failsafe_throttle is FS_THR_ENABLED_ALWAYS_RTL do RTL
            } else if (g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_RTL) {
                if(home_distance > wp_nav.get_wp_radius()) {
                    // switch to RTL or if that fails, LAND
                    set_mode_RTL_or_land_with_pause();
                }else{
                    // We are very close to home so we will land
                    set_mode_land_with_pause();
                }
            }
            // failsafe_throttle must be FS_THR_ENABLED_CONTINUE_MISSION so no need to do anything
            break;

        case LAND:
            // continue to land if battery failsafe is also active otherwise fall through to default handling
            if (g.failsafe_battery_enabled == FS_BATT_LAND && failsafe.battery) {
                break;
            }
            // no break
        default:
            // used for AltHold, Guided, Loiter, RTL, Circle, Drift, Sport, Flip, Autotune, PosHold
            // if landed disarm
            if (ap.land_complete) {
                init_disarm_motors();

            // if failsafe_throttle is FS_THR_ENABLED_ALWAYS_LAND then land immediately
            } else if(g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                set_mode_land_with_pause();

            // if far from home then RTL
            }else if(home_distance > wp_nav.get_wp_radius()) {
                // switch to RTL or if that fails, LAND
                set_mode_RTL_or_land_with_pause();
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode_land_with_pause();
            }
            break;
    }

    // log the error to the dataflash
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_OCCURRED);

}

// failsafe_off_event - respond to radio contact being regained
// we must be in AUTO, LAND or RTL modes
// or Stabilize or ACRO mode but with motors disarmed
static void failsafe_radio_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_RESOLVED);
}

static void failsafe_battery_event(void)
{
    // return immediately if low battery event has already been triggered
    if (failsafe.battery) {
        return;
    }

    // failsafe check
    if (g.failsafe_battery_enabled != FS_BATT_DISABLED && motors.armed()) {
        switch(control_mode) {
            case STABILIZE:
            case ACRO:
                // if throttle is zero OR vehicle is landed disarm motors
                if (ap.throttle_zero || ap.land_complete) {
                    init_disarm_motors();
                }else{
                    // set mode to RTL or LAND
                    if (g.failsafe_battery_enabled == FS_BATT_RTL && home_distance > wp_nav.get_wp_radius()) {
                        // switch to RTL or if that fails, LAND
                        set_mode_RTL_or_land_with_pause();
                    }else{
                        set_mode_land_with_pause();
                    }
                }
                break;
            case AUTO:
                // if mission has not started AND vehicle is landed, disarm motors
                if (!ap.auto_armed && ap.land_complete) {
                    init_disarm_motors();

                // set mode to RTL or LAND
                } else if (home_distance > wp_nav.get_wp_radius()) {
                    // switch to RTL or if that fails, LAND
                    set_mode_RTL_or_land_with_pause();
                } else {
                    set_mode_land_with_pause();
                }
                break;
            default:
                // used for AltHold, Guided, Loiter, RTL, Circle, Drift, Sport, Flip, Autotune, PosHold
                // if landed disarm
                if (ap.land_complete) {
                    init_disarm_motors();

                // set mode to RTL or LAND
                } else if (g.failsafe_battery_enabled == FS_BATT_RTL && home_distance > wp_nav.get_wp_radius()) {
                    // switch to RTL or if that fails, LAND
                    set_mode_RTL_or_land_with_pause();
                } else {
                    set_mode_land_with_pause();
                }
                break;
        }
    }

    // set the low battery flag
    set_failsafe_battery(true);

    // warn the ground station and log to dataflash
    gcs_send_text_P(SEVERITY_HIGH,PSTR("Low Battery!"));
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_BATT, ERROR_CODE_FAILSAFE_OCCURRED);

}

// failsafe_gcs_check - check for ground station failsafe
static void failsafe_gcs_check()
{
    uint32_t last_gcs_update_ms;

    // return immediately if gcs failsafe is disabled, gcs has never been connected or we are not overriding rc controls from the gcs and we are not in guided mode
    // this also checks to see if we have a GCS failsafe active, if we do, then must continue to process the logic for recovery from this state.
    if ((!failsafe.gcs)&&(g.failsafe_gcs == FS_GCS_DISABLED || failsafe.last_heartbeat_ms == 0 || (!failsafe.rc_override_active && control_mode != GUIDED))) {
        return;
    }

    // calc time since last gcs update
    // note: this only looks at the heartbeat from the device id set by g.sysid_my_gcs
    last_gcs_update_ms = millis() - failsafe.last_heartbeat_ms;

    // check if all is well
    if (last_gcs_update_ms < FS_GCS_TIMEOUT_MS) {
        // check for recovery from gcs failsafe
        if (failsafe.gcs) {
            failsafe_gcs_off_event();
            set_failsafe_gcs(false);
        }
        return;
    }

    // do nothing if gcs failsafe already triggered or motors disarmed
    if (failsafe.gcs || !motors.armed()) {
        return;
    }

    // GCS failsafe event has occured
    // update state, log to dataflash
    set_failsafe_gcs(true);
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_OCCURRED);

    // clear overrides so that RC control can be regained with radio.
    hal.rcin->clear_overrides();
    failsafe.rc_override_active = false;

    // This is how to handle a failsafe.
    // use the throttle failsafe setting to decide what to do
    switch(control_mode) {
        case STABILIZE:
        case ACRO:
        case SPORT:
            // if throttle is zero disarm motors
            if (ap.throttle_zero) {
                init_disarm_motors();
            }else if(home_distance > wp_nav.get_wp_radius()) {
                // switch to RTL or if that fails, LAND
                set_mode_RTL_or_land_with_pause();
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode_land_with_pause();
            }
            break;
        case AUTO:
            // if g.failsafe_gcs is 1 do RTL, 2 means continue with the mission
            if (g.failsafe_gcs == FS_GCS_ENABLED_ALWAYS_RTL) {
                if (home_distance > wp_nav.get_wp_radius()) {
                    // switch to RTL or if that fails, LAND
                    set_mode_RTL_or_land_with_pause();
                }else{
                    // We are very close to home so we will land
                    set_mode_land_with_pause();
                }
            }
            // if failsafe_throttle is 2 (i.e. FS_THR_ENABLED_CONTINUE_MISSION) no need to do anything
            break;
        default:
            if(home_distance > wp_nav.get_wp_radius()) {
                // switch to RTL or if that fails, LAND
                set_mode_RTL_or_land_with_pause();
            }else{
                // We have no GPS or are very close to home so we will land
                set_mode_land_with_pause();
            }
            break;
    }
}

// failsafe_gcs_off_event - actions to take when GCS contact is restored
static void failsafe_gcs_off_event(void)
{
    // log recovery of GCS in logs?
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_RESOLVED);
}

// set_mode_RTL_or_land_with_pause - sets mode to RTL if possible or LAND with 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
static void set_mode_RTL_or_land_with_pause()
{
    // attempt to switch to RTL, if this fails then switch to Land
    if (!set_mode(RTL)) {
        // set mode to land will trigger mode change notification to pilot
        set_mode_land_with_pause();
    } else {
        // alert pilot to mode change
        AP_Notify::events.failsafe_mode_change = 1;
    }
}

static void update_events()
{
    ServoRelayEvents.update_events();
}

#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/failsafe.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
//  failsafe support
//  Andrew Tridgell, December 2011
//
//  our failsafe strategy is to detect main loop lockup and disarm the motors
//

static bool failsafe_enabled = false;
static uint16_t failsafe_last_mainLoop_count;
static uint32_t failsafe_last_timestamp;
static bool in_failsafe;

//
// failsafe_enable - enable failsafe
//
void failsafe_enable()
{
    failsafe_enabled = true;
    failsafe_last_timestamp = micros();
}

//
// failsafe_disable - used when we know we are going to delay the mainloop significantly
//
void failsafe_disable()
{
    failsafe_enabled = false;
}

//
//  failsafe_check - this function is called from the core timer interrupt at 1kHz.
//
void failsafe_check()
{
    uint32_t tnow = hal.scheduler->micros();

    if (mainLoop_count != failsafe_last_mainLoop_count) {
        // the main loop is running, all is OK
        failsafe_last_mainLoop_count = mainLoop_count;
        failsafe_last_timestamp = tnow;
        if (in_failsafe) {
            in_failsafe = false;
            Log_Write_Error(ERROR_SUBSYSTEM_CPU,ERROR_CODE_FAILSAFE_RESOLVED);
        }
        return;
    }

    if (!in_failsafe && failsafe_enabled && tnow - failsafe_last_timestamp > 2000000) {
        // motors are running but we have gone 2 second since the
        // main loop ran. That means we're in trouble and should
        // disarm the motors.
        in_failsafe = true;
        // reduce motors to minimum (we do not immediately disarm because we want to log the failure)
        if (motors.armed()) {
            motors.output_min();
        }
        // log an error
        Log_Write_Error(ERROR_SUBSYSTEM_CPU,ERROR_CODE_FAILSAFE_OCCURRED);
    }

    if (failsafe_enabled && in_failsafe && tnow - failsafe_last_timestamp > 1000000) {
        // disarm motors every second
        failsafe_last_timestamp = tnow;
        if(motors.armed()) {
            motors.armed(false);
            motors.output();
        }
    }
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/fence.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Code to integrate AC_Fence library with main ArduCopter code

#if AC_FENCE == ENABLED

// fence_check - ask fence library to check for breaches and initiate the response
// called at 1hz
void fence_check()
{
    uint8_t new_breaches; // the type of fence that has been breached
    uint8_t orig_breaches = fence.get_breaches();

    // return immediately if motors are not armed
    if(!motors.armed()) {
        return;
    }

    // give fence library our current distance from home in meters
    fence.set_home_distance(home_distance*0.01f);

    // check for a breach
    new_breaches = fence.check_fence(pv_alt_above_home(inertial_nav.get_altitude())/100.0f);

    // if there is a new breach take action
    if( new_breaches != AC_FENCE_TYPE_NONE ) {

        // if the user wants some kind of response and motors are armed
        if(fence.get_action() != AC_FENCE_ACTION_REPORT_ONLY ) {

            // disarm immediately if we think we are on the ground or in a manual flight mode with zero throttle
            // don't disarm if the high-altitude fence has been broken because it's likely the user has pulled their throttle to zero to bring it down
            if (ap.land_complete || (mode_has_manual_throttle(control_mode) && ap.throttle_zero && !failsafe.radio && ((fence.get_breaches() & AC_FENCE_TYPE_ALT_MAX)== 0))){
                init_disarm_motors();
            }else{
                // if we are within 100m of the fence, RTL
                if (fence.get_breach_distance(new_breaches) <= AC_FENCE_GIVE_UP_DISTANCE) {
                    if (!set_mode(RTL)) {
                        set_mode(LAND);
                    }
                }else{
                    // if more than 100m outside the fence just force a land
                    set_mode(LAND);
                }
            }
        }

        // log an error in the dataflash
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, new_breaches);
    }

    // record clearing of breach
    if(orig_breaches != AC_FENCE_TYPE_NONE && fence.get_breaches() == AC_FENCE_TYPE_NONE) {
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_FENCE, ERROR_CODE_ERROR_RESOLVED);
    }
}

// fence_send_mavlink_status - send fence status to ground station
static void fence_send_mavlink_status(mavlink_channel_t chan)
{   
    if (fence.enabled()) {
        // traslate fence library breach types to mavlink breach types
        uint8_t mavlink_breach_type = FENCE_BREACH_NONE;
        uint8_t breaches = fence.get_breaches();
        if ((breaches & AC_FENCE_TYPE_ALT_MAX) != 0) {
            mavlink_breach_type = FENCE_BREACH_MAXALT;
        }
        if ((breaches & AC_FENCE_TYPE_CIRCLE) != 0) {
            mavlink_breach_type = FENCE_BREACH_BOUNDARY;
        }

        // send status
        mavlink_msg_fence_status_send(chan,
                                      (int8_t)(fence.get_breaches()!=0),
                                      fence.get_breach_count(),
                                      mavlink_breach_type,
                                      fence.get_breach_time());
    }
}

#endif
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/flight_mode.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * flight.pde - high level calls to set and update flight modes
 *      logic for individual flight modes is in control_acro.pde, control_stabilize.pde, etc
 */

// set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was succesfully set
// ACRO, STABILIZE, ALTHOLD, LAND, DRIFT and SPORT can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
static bool set_mode(uint8_t mode)
{
    // boolean to record if flight mode could be set
    bool success = false;
    bool ignore_checks = !motors.armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

    // return immediately if we are already in the desired mode
    if (mode == control_mode) {
        return true;
    }

    switch(mode) {
        case ACRO:
            #if FRAME_CONFIG == HELI_FRAME
                success = heli_acro_init(ignore_checks);
            #else
                success = acro_init(ignore_checks);
            #endif
            break;

        case STABILIZE:
            #if FRAME_CONFIG == HELI_FRAME
                success = heli_stabilize_init(ignore_checks);
            #else
                success = stabilize_init(ignore_checks);
            #endif
            break;

        case ALT_HOLD:
            success = althold_init(ignore_checks);
            break;

        case AUTO:
            success = auto_init(ignore_checks);
            break;

        case CIRCLE:
            success = circle_init(ignore_checks);
            break;

        case LOITER:
            success = loiter_init(ignore_checks);
            break;

        case GUIDED:
            success = guided_init(ignore_checks);
            break;

        case LAND:
            success = land_init(ignore_checks);
            break;

        case RTL:
            success = rtl_init(ignore_checks);
            break;

        case DRIFT:
            success = drift_init(ignore_checks);
            break;

        case SPORT:
            success = sport_init(ignore_checks);
            break;

        case FLIP:
            success = flip_init(ignore_checks);
            break;

#if AUTOTUNE_ENABLED == ENABLED
        case AUTOTUNE:
            success = autotune_init(ignore_checks);
            break;
#endif

#if POSHOLD_ENABLED == ENABLED
        case POSHOLD:
            success = poshold_init(ignore_checks);
            break;
#endif

        case STOP:
            success = stop_init(ignore_checks);
            break;

        default:
            success = false;
            break;
    }

    // update flight mode
    if (success) {
        // perform any cleanup required by previous flight mode
        exit_mode(control_mode, mode);
        control_mode = mode;
        DataFlash.Log_Write_Mode(control_mode);

#if AC_FENCE == ENABLED
        // pilot requested flight mode change during a fence breach indicates pilot is attempting to manually recover
        // this flight mode change could be automatic (i.e. fence, battery, GPS or GCS failsafe)
        // but it should be harmless to disable the fence temporarily in these situations as well
        fence.manual_recovery_start();
#endif
    }else{
        // Log error that we failed to enter desired flight mode
        Log_Write_Error(ERROR_SUBSYSTEM_FLIGHT_MODE,mode);
    }

    // update notify object
    if (success) {
        notify_flight_mode(control_mode);
    }

    // return success or failure
    return success;
}

// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
static void update_flight_mode()
{
#if AP_AHRS_NAVEKF_AVAILABLE
    // Update EKF speed limit - used to limit speed when we are using optical flow
    ahrs.getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);
#endif
    switch (control_mode) {
        case ACRO:
            #if FRAME_CONFIG == HELI_FRAME
                heli_acro_run();
            #else
                acro_run();
            #endif
            break;

        case STABILIZE:
            #if FRAME_CONFIG == HELI_FRAME
                heli_stabilize_run();
            #else
                stabilize_run();
            #endif
            break;

        case ALT_HOLD:
            althold_run();
            break;

        case AUTO:
            auto_run();
            break;

        case CIRCLE:
            circle_run();
            break;

        case LOITER:
            loiter_run();
            break;

        case GUIDED:
            guided_run();
            break;

        case LAND:
            land_run();
            break;

        case RTL:
            rtl_run();
            break;

        case DRIFT:
            drift_run();
            break;

        case SPORT:
            sport_run();
            break;

        case FLIP:
            flip_run();
            break;

#if AUTOTUNE_ENABLED == ENABLED
        case AUTOTUNE:
            autotune_run();
            break;
#endif

#if POSHOLD_ENABLED == ENABLED
        case POSHOLD:
            poshold_run();
            break;
#endif

        case STOP:
            stop_run();
            break;
    }
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
static void exit_mode(uint8_t old_control_mode, uint8_t new_control_mode)
{
#if AUTOTUNE_ENABLED == ENABLED
    if (old_control_mode == AUTOTUNE) {
        autotune_stop();
    }
#endif

    // stop mission when we leave auto mode
    if (old_control_mode == AUTO) {
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();
        }
#if MOUNT == ENABLED
        camera_mount.set_mode_to_default();
#endif  // MOUNT == ENABLED
    }

    // smooth throttle transition when switching from manual to automatic flight modes
    if (mode_has_manual_throttle(old_control_mode) && !mode_has_manual_throttle(new_control_mode) && motors.armed() && !ap.land_complete) {
        // this assumes all manual flight modes use get_pilot_desired_throttle to translate pilot input to output throttle
        set_accel_throttle_I_from_pilot_throttle(get_pilot_desired_throttle(g.rc_3.control_in));
    }
    
#if FRAME_CONFIG == HELI_FRAME
    // firmly reset the flybar passthrough to false when exiting acro mode.
    if (old_control_mode == ACRO) {
        attitude_control.use_flybar_passthrough(false);
    }
#endif //HELI_FRAME
}

// returns true or false whether mode requires GPS
static bool mode_requires_GPS(uint8_t mode) {
    switch(mode) {
        case AUTO:
        case GUIDED:
        case LOITER:
        case RTL:
        case CIRCLE:
        case DRIFT:
        case POSHOLD:
        case STOP:
            return true;
        default:
            return false;
    }

    return false;
}

// mode_has_manual_throttle - returns true if the flight mode has a manual throttle (i.e. pilot directly controls throttle)
static bool mode_has_manual_throttle(uint8_t mode) {
    switch(mode) {
        case ACRO:
        case STABILIZE:
            return true;
        default:
            return false;
    }

    return false;
}

// mode_allows_arming - returns true if vehicle can be armed in the specified mode
//  arming_from_gcs should be set to true if the arming request comes from the ground station
static bool mode_allows_arming(uint8_t mode, bool arming_from_gcs) {
    if (mode_has_manual_throttle(mode) || mode == LOITER || mode == ALT_HOLD || mode == POSHOLD || (arming_from_gcs && mode == GUIDED)) {
        return true;
    }
    return false;
}

// notify_flight_mode - sets notify object based on flight mode.  Only used for OreoLED notify device
static void notify_flight_mode(uint8_t mode) {
    switch(mode) {
        case AUTO:
        case GUIDED:
        case RTL:
        case CIRCLE:
        case LAND:
            // autopilot modes
            AP_Notify::flags.autopilot_mode = true;
            break;
        default:
            // all other are manual flight modes
            AP_Notify::flags.autopilot_mode = false;
            break;
    }
}

//
// print_flight_mode - prints flight mode to serial port.
//
static void
print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    switch (mode) {
    case STABILIZE:
        port->print_P(PSTR("STABILIZE"));
        break;
    case ACRO:
        port->print_P(PSTR("ACRO"));
        break;
    case ALT_HOLD:
        port->print_P(PSTR("ALT_HOLD"));
        break;
    case AUTO:
        port->print_P(PSTR("AUTO"));
        break;
    case GUIDED:
        port->print_P(PSTR("GUIDED"));
        break;
    case LOITER:
        port->print_P(PSTR("LOITER"));
        break;
    case RTL:
        port->print_P(PSTR("RTL"));
        break;
    case CIRCLE:
        port->print_P(PSTR("CIRCLE"));
        break;
    case LAND:
        port->print_P(PSTR("LAND"));
        break;
    case OF_LOITER:
        port->print_P(PSTR("OF_LOITER"));
        break;
    case DRIFT:
        port->print_P(PSTR("DRIFT"));
        break;
    case SPORT:
        port->print_P(PSTR("SPORT"));
        break;
    case FLIP:
        port->print_P(PSTR("FLIP"));
        break;
    case AUTOTUNE:
        port->print_P(PSTR("AUTOTUNE"));
        break;
    case POSHOLD:
        port->print_P(PSTR("POSHOLD"));
        break;
    default:
        port->printf_P(PSTR("Mode(%u)"), (unsigned)mode);
        break;
    }
}

#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/heli.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Traditional helicopter variables and functions

#include "heli.h"

#if FRAME_CONFIG == HELI_FRAME

#ifndef HELI_DYNAMIC_FLIGHT_SPEED_MIN
 #define HELI_DYNAMIC_FLIGHT_SPEED_MIN      500     // we are in "dynamic flight" when the speed is over 1m/s for 2 seconds
#endif

// counter to control dynamic flight profile
static int8_t heli_dynamic_flight_counter;

// heli_init - perform any special initialisation required for the tradheli
static void heli_init()
{
    motors.set_dt(MAIN_LOOP_SECONDS);
    // force recalculation of RSC ramp rates after setting _dt
    motors.recalc_scalers();
}

// get_pilot_desired_collective - converts pilot input (from 0 ~ 1000) to a value that can be fed into the g.rc_3.servo_out function
static int16_t get_pilot_desired_collective(int16_t control_in)
{
    // return immediately if reduce collective range for manual flight has not been configured
    if (g.heli_stab_col_min == 0 && g.heli_stab_col_max == 1000) {
        return control_in;
    }

    // scale pilot input to reduced collective range
    float scalar = ((float)(g.heli_stab_col_max - g.heli_stab_col_min))/1000.0f;
    int16_t collective_out = g.heli_stab_col_min + control_in * scalar;
    collective_out = constrain_int16(collective_out, 0, 1000);
    return collective_out;
}

// heli_check_dynamic_flight - updates the dynamic_flight flag based on our horizontal velocity
// should be called at 50hz
static void check_dynamic_flight(void)
{
    if (!motors.armed() || !motors.motor_runup_complete() ||
        control_mode == LAND || (control_mode==RTL && rtl_state == Land) || (control_mode == AUTO && auto_mode == Auto_Land)) {
        heli_dynamic_flight_counter = 0;
        heli_flags.dynamic_flight = false;
        return;
    }

    bool moving = false;

    // with GPS lock use inertial nav to determine if we are moving
    if (position_ok()) {
        // get horizontal velocity
        float velocity = inertial_nav.get_velocity_xy();
        moving = (velocity >= HELI_DYNAMIC_FLIGHT_SPEED_MIN);
    }else{
        // with no GPS lock base it on throttle and forward lean angle
        moving = (g.rc_3.servo_out > 800 || ahrs.pitch_sensor < -1500);
    }

    if (moving) {
        // if moving for 2 seconds, set the dynamic flight flag
        if (!heli_flags.dynamic_flight) {
            heli_dynamic_flight_counter++;
            if (heli_dynamic_flight_counter >= 100) {
                heli_flags.dynamic_flight = true;
                heli_dynamic_flight_counter = 100;
            }
        }
    }else{
        // if not moving for 2 seconds, clear the dynamic flight flag
        if (heli_flags.dynamic_flight) {
            if (heli_dynamic_flight_counter > 0) {
                heli_dynamic_flight_counter--;
            }else{
                heli_flags.dynamic_flight = false;
            }
        }
    }
}

// update_heli_control_dynamics - pushes several important factors up into AP_MotorsHeli.
// should be run between the rate controller and the servo updates.
static void update_heli_control_dynamics(void)
{
    // Use Leaky_I if we are not moving fast
    attitude_control.use_leaky_i(!heli_flags.dynamic_flight);
    
    // To-Do: Update dynamic phase angle of swashplate
}

// heli_update_landing_swash - sets swash plate flag so higher minimum is used when landed or landing
// should be called soon after update_land_detector in main code
static void heli_update_landing_swash()
{
    switch(control_mode) {
        case ACRO:
        case STABILIZE:
        case DRIFT:
        case SPORT:
            // manual modes always uses full swash range
            motors.set_collective_for_landing(false);
            break;

        case LAND:
            // landing always uses limit swash range
            motors.set_collective_for_landing(true);
            break;

        case RTL:
            if (rtl_state == Land) {
                motors.set_collective_for_landing(true);
            }else{
                motors.set_collective_for_landing(!heli_flags.dynamic_flight || ap.land_complete || !ap.auto_armed);
            }
            break;

        case AUTO:
            if (auto_mode == Auto_Land) {
                motors.set_collective_for_landing(true);
            }else{
                motors.set_collective_for_landing(!heli_flags.dynamic_flight || ap.land_complete || !ap.auto_armed);
            }
            break;

        default:
            // auto and hold use limited swash when landed
            motors.set_collective_for_landing(!heli_flags.dynamic_flight || ap.land_complete || !ap.auto_armed);
            break;
    }
}

// heli_update_rotor_speed_targets - reads pilot input and passes new rotor speed targets to heli motors object
static void heli_update_rotor_speed_targets()
{
    // get rotor control method
    uint8_t rsc_control_mode = motors.get_rsc_mode();
    int16_t rsc_control_deglitched = rotor_speed_deglitch_filter.apply(g.rc_8.control_in);

    switch (rsc_control_mode) {
        case AP_MOTORS_HELI_RSC_MODE_NONE:
            // even though pilot passes rotors speed directly to rotor ESC via receiver, motor lib needs to know if
            // rotor is spinning in case we are using direct drive tailrotor which must be spun up at same time
        case AP_MOTORS_HELI_RSC_MODE_CH8_PASSTHROUGH:
            // pass through pilot desired rotor speed
            motors.set_desired_rotor_speed(rsc_control_deglitched);
            break;
        case AP_MOTORS_HELI_RSC_MODE_SETPOINT:
            // pass setpoint through as desired rotor speed
            if (rsc_control_deglitched > 0) {
                motors.set_desired_rotor_speed(motors.get_rsc_setpoint());
            }else{
                motors.set_desired_rotor_speed(0);
            }
            break;
    }
}

#endif  // FRAME_CONFIG == HELI_FRAME
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/heli_control_acro.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#if FRAME_CONFIG == HELI_FRAME
/*
 * heli_control_acro.pde - init and run calls for acro flight mode for trad heli
 */

// heli_acro_init - initialise acro controller
static bool heli_acro_init(bool ignore_checks)
{
    // if heli is equipped with a flybar, then tell the attitude controller to pass through controls directly to servos
    attitude_control.use_flybar_passthrough(motors.has_flybar());

    // always successfully enter acro
    return true;
}

// heli_acro_run - runs the acro controller
// should be called at 100hz or more
static void heli_acro_run()
{
    float target_roll, target_pitch, target_yaw;
    
    // Tradheli should not reset roll, pitch, yaw targets when motors are not runup, because
    // we may be in autorotation flight.  These should be reset only when transitioning from disarmed
    // to armed, because the pilot will have placed the helicopter down on the landing pad.  This is so
    // that the servos move in a realistic fashion while disarmed for operational checks.
    // Also, unlike multicopters we do not set throttle (i.e. collective pitch) to zero so the swash servos move
    
    if(!motors.armed()) {
        heli_flags.init_targets_on_arming=true;
        attitude_control.set_yaw_target_to_current_heading();
    }

    if(motors.armed() && heli_flags.init_targets_on_arming) {
        heli_flags.init_targets_on_arming=false;
        attitude_control.relax_bf_rate_controller();
    }   

    if (!motors.has_flybar()){
        // convert the input to the desired body frame rate
        get_pilot_desired_angle_rates(g.rc_1.control_in, g.rc_2.control_in, g.rc_4.control_in, target_roll, target_pitch, target_yaw);
        // run attitude controller
        attitude_control.rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);
    }else{
        // flybar helis only need yaw rate control
        get_pilot_desired_yaw_rate(g.rc_4.control_in, target_yaw);
        // run attitude controller
        attitude_control.passthrough_bf_roll_pitch_rate_yaw(g.rc_1.control_in, g.rc_2.control_in, target_yaw);
    }

    // output pilot's throttle without angle boost
    attitude_control.set_throttle_out(g.rc_3.control_in, false, g.throttle_filt);
}

// get_pilot_desired_yaw_rate - transform pilot's yaw input into a desired yaw angle rate
// returns desired yaw rate in centi-degrees-per-second
static void get_pilot_desired_yaw_rate(int16_t yaw_in, float &yaw_out)
{
    // calculate rate request
    float rate_bf_yaw_request = yaw_in * g.acro_yaw_p;

    // hand back rate request
    yaw_out = rate_bf_yaw_request;
}

#endif  //HELI_FRAME
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/heli_control_stabilize.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#if FRAME_CONFIG == HELI_FRAME
/*
 * heli_control_stabilize.pde - init and run calls for stabilize flight mode for trad heli
 */

// stabilize_init - initialise stabilize controller
static bool heli_stabilize_init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    // To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
    pos_control.set_alt_target(0);
    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
static void heli_stabilize_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // Tradheli should not reset roll, pitch, yaw targets when motors are not runup, because
    // we may be in autorotation flight.  These should be reset only when transitioning from disarmed
    // to armed, because the pilot will have placed the helicopter down on the landing pad.  This is so
    // that the servos move in a realistic fashion while disarmed for operational checks.
    // Also, unlike multicopters we do not set throttle (i.e. collective pitch) to zero so the swash servos move
    
    if(!motors.armed()) {
        heli_flags.init_targets_on_arming=true;
        attitude_control.set_yaw_target_to_current_heading();
    }
    
    if(motors.armed() && heli_flags.init_targets_on_arming) {
        heli_flags.init_targets_on_arming=false;
        attitude_control.relax_bf_rate_controller();
    }
    
    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_collective(g.rc_3.control_in);

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // output pilot's throttle - note that TradHeli does not used angle-boost
    attitude_control.set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);
}

#endif  //HELI_FRAME
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/inertia.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// read_inertia - read inertia in from accelerometers
static void read_inertia()
{
    // inertial altitude estimates
    inertial_nav.update(G_Dt);
}

// read_inertial_altitude - pull altitude and climb rate from inertial nav library
static void read_inertial_altitude()
{
    // exit immediately if we do not have an altitude estimate or home is not set
    if (!inertial_nav.get_filter_status().flags.vert_pos || (ap.home_state == HOME_UNSET)) {
        return;
    }

    // with inertial nav we can update the altitude and climb rate at 50hz
    current_loc.alt = pv_alt_above_home(inertial_nav.get_altitude());
    current_loc.flags.relative_alt = true;
    climb_rate = inertial_nav.get_velocity_z();
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/land_detector.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// counter to verify landings
static uint32_t land_detector = ((float)LAND_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE; // we assume we are landed

// land_complete_maybe - return true if we may have landed (used to reset loiter targets during landing)
static bool land_complete_maybe()
{
    return (ap.land_complete || ap.land_complete_maybe);
}

// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// called at MAIN_LOOP_RATE
static void update_land_detector()
{
    // land detector can not use the following sensors because they are unreliable during landing
    // barometer altitude :                 ground effect can cause errors larger than 4m
    // EKF vertical velocity or altitude :  poor barometer and large acceleration from ground impact
    // earth frame angle or angle error :   landing on an uneven surface will force the airframe to match the ground angle
    // gyro output :                        on uneven surface the airframe may rock back an forth after landing
    // range finder :                       tend to be problematic at very short distances
    // input throttle :                     in slow land the input throttle may be only slightly less than hover

    // check that the average throttle output is near minimum (less than 12.5% hover throttle)
    bool motor_at_lower_limit = motors.limit.throttle_lower && (motors.get_throttle_low_comp() < 0.125f);
    Vector3f accel_ef = ahrs.get_accel_ef_blended();
    accel_ef.z += GRAVITY_MSS;

    // lowpass filter on accel
    accel_ef = land_accel_ef_filter.apply(accel_ef, MAIN_LOOP_SECONDS);

    // check that the airframe is not accelerating (not falling or breaking after fast forward flight)
    bool accel_stationary = (accel_ef.length() < 1.0f);

    if ( motor_at_lower_limit && accel_stationary) {
        if (!ap.land_complete) {
            // increase counter until we hit the trigger then set land complete flag
            if( land_detector < ((float)LAND_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE) {
                land_detector++;
            }else{
                set_land_complete(true);
                land_detector = ((float)LAND_DETECTOR_TRIGGER_SEC)*MAIN_LOOP_RATE;
            }
        }
    } else {
        // we've sensed movement up or down so reset land_detector
        land_detector = 0;
        // if throttle output is high then clear landing flag
        if (motors.get_throttle_out() > get_non_takeoff_throttle()) {
            set_land_complete(false);
        }
    }

    // set land maybe flag
    set_land_complete_maybe(land_detector >= LAND_DETECTOR_MAYBE_TRIGGER_SEC*MAIN_LOOP_RATE);
}

// update_throttle_low_comp - sets motors throttle_low_comp value depending upon vehicle state
//  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
//  has no effect when throttle is above hover throttle
static void update_throttle_low_comp()
{
    if (mode_has_manual_throttle(control_mode)) {
        // manual throttle
        if(!motors.armed() || g.rc_3.control_in <= 0) {
            motors.set_throttle_low_comp(0.1f);
        } else {
            motors.set_throttle_low_comp(0.5f);
        }
    } else {
        // autopilot controlled throttle

        // check for aggressive flight requests
        const Vector3f angle_target = attitude_control.angle_ef_targets();
        bool large_angle_request = (pythagorous2(angle_target.x, angle_target.y) > 1500.0f);

        // check for large external disturbance
        const Vector3f angle_error = attitude_control.angle_bf_error();
        bool large_angle_error = (pythagorous2(angle_error.x, angle_error.y) > 3000.0f);

        // check for large acceleration ( falling )
        Vector3f accel_ef = ahrs.get_accel_ef_blended();
        accel_ef.z += GRAVITY_MSS;
        bool accel_moving = (accel_ef.length() > 3.0f);

        if ( large_angle_request || large_angle_error || accel_moving) {
            // if target lean angle is over 15 degrees set high
            motors.set_throttle_low_comp(0.9f);
        } else {
            motors.set_throttle_low_comp(0.1f);
        }
    }
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/landing_gear.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Run landing gear controller at 10Hz
static void landinggear_update(){

    // If landing gear control is active, run update function.
    if (check_if_auxsw_mode_used(AUXSW_LANDING_GEAR)){

        // last status (deployed or retracted) used to check for changes
        static bool last_deploy_status;

        // if we are doing an automatic landing procedure, force the landing gear to deploy.
        // To-Do: should we pause the auto-land procedure to give time for gear to come down?
        if (control_mode == LAND || (control_mode==RTL && rtl_state == Land) || (control_mode == AUTO && auto_mode == Auto_Land)){
            landinggear.force_deploy(true);
        }            

        landinggear.update();

        // send event message to datalog if status has changed
        if (landinggear.deployed() != last_deploy_status){
            if (landinggear.deployed()) {
                Log_Write_Event(DATA_LANDING_GEAR_DEPLOYED);
            } else {
                Log_Write_Event(DATA_LANDING_GEAR_RETRACTED);
            }
        }

        last_deploy_status = landinggear.deployed();        
    }
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/leds.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// updates the status of notify
// should be called at 50hz
static void update_notify()
{
    notify.update();
}

#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/motor_test.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  mavlink motor test - implements the MAV_CMD_DO_MOTOR_TEST mavlink command so that the GCS/pilot can test an individual motor or flaps
                       to ensure proper wiring, rotation.
 */

// motor test definitions
#define MOTOR_TEST_PWM_MIN              800     // min pwm value accepted by the test
#define MOTOR_TEST_PWM_MAX              2200    // max pwm value accepted by the test
#define MOTOR_TEST_TIMEOUT_MS_MAX       30000   // max timeout is 30 seconds

static uint32_t motor_test_start_ms = 0;        // system time the motor test began
static uint32_t motor_test_timeout_ms = 0;      // test will timeout this many milliseconds after the motor_test_start_ms
static uint8_t motor_test_seq = 0;              // motor sequence number of motor being tested
static uint8_t motor_test_throttle_type = 0;    // motor throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through)
static uint16_t motor_test_throttle_value = 0;  // throttle to be sent to motor, value depends upon it's type

// motor_test_output - checks for timeout and sends updates to motors objects
static void motor_test_output()
{
    // exit immediately if the motor test is not running
    if (!ap.motor_test) {
        return;
    }

    // check for test timeout
    if ((hal.scheduler->millis() - motor_test_start_ms) >= motor_test_timeout_ms) {
        // stop motor test
        motor_test_stop();
    } else {
        int16_t pwm = 0;   // pwm that will be output to the motors

        // calculate pwm based on throttle type
        switch (motor_test_throttle_type) {

            case MOTOR_TEST_THROTTLE_PERCENT:
                // sanity check motor_test_throttle value
                if (motor_test_throttle_value <= 100) {
                    pwm = g.rc_3.radio_min + (g.rc_3.radio_max - g.rc_3.radio_min) * (float)motor_test_throttle_value/100.0f;
                }
                break;

            case MOTOR_TEST_THROTTLE_PWM:
                pwm = motor_test_throttle_value;
                break;

            case MOTOR_TEST_THROTTLE_PILOT:
                pwm = g.rc_3.radio_in;
                break;

            default:
                motor_test_stop();
                return;
                break;
        }

        // sanity check throttle values
        if (pwm >= MOTOR_TEST_PWM_MIN && pwm <= MOTOR_TEST_PWM_MAX ) {
            // turn on motor to specified pwm vlaue
            motors.output_test(motor_test_seq, pwm);
        } else {
            motor_test_stop();
        }
    }
}

// mavlink_motor_test_check - perform checks before motor tests can begin
//  return true if tests can continue, false if not
static bool mavlink_motor_test_check(mavlink_channel_t chan, bool check_rc)
{
    // check rc has been calibrated
    pre_arm_rc_checks();
    if(check_rc && !ap.pre_arm_rc_check) {
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH,PSTR("Motor Test: RC not calibrated"));
        return false;
    }

    // ensure we are landed
    if (!ap.land_complete) {
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH,PSTR("Motor Test: vehicle not landed"));
        return false;
    }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        gcs[chan-MAVLINK_COMM_0].send_text_P(SEVERITY_HIGH,PSTR("Motor Test: Safety Switch"));
        return false;
    }

    // if we got this far the check was successful and the motor test can continue
    return true;
}

// mavlink_motor_test_start - start motor test - spin a single motor at a specified pwm
//  returns MAV_RESULT_ACCEPTED on success, MAV_RESULT_FAILED on failure
static uint8_t mavlink_motor_test_start(mavlink_channel_t chan, uint8_t motor_seq, uint8_t throttle_type, uint16_t throttle_value, float timeout_sec)
{
    // if test has not started try to start it
    if (!ap.motor_test) {
        /* perform checks that it is ok to start test
           The RC calibrated check can be skipped if direct pwm is
           supplied
        */
        if (!mavlink_motor_test_check(chan, throttle_type != 1)) {
            return MAV_RESULT_FAILED;
        } else {
            // start test
            ap.motor_test = true;

            // enable and arm motors
            if (!motors.armed()) {
                init_rc_out();
                output_min();
                motors.armed(true);
            }

            // disable throttle, battery and gps failsafe
            g.failsafe_throttle = FS_THR_DISABLED;
            g.failsafe_battery_enabled = FS_BATT_DISABLED;
            g.failsafe_gcs = FS_GCS_DISABLED;

            // turn on notify leds
            AP_Notify::flags.esc_calibration = true;
        }
    }

    // set timeout
    motor_test_start_ms = hal.scheduler->millis();
    motor_test_timeout_ms = min(timeout_sec * 1000, MOTOR_TEST_TIMEOUT_MS_MAX);

    // store required output
    motor_test_seq = motor_seq;
    motor_test_throttle_type = throttle_type;
    motor_test_throttle_value = throttle_value;

    // return success
    return MAV_RESULT_ACCEPTED;
}

// motor_test_stop - stops the motor test
static void motor_test_stop()
{
    // exit immediately if the test is not running
    if (!ap.motor_test) {
        return;
    }

    // flag test is complete
    ap.motor_test = false;

    // disarm motors
    motors.armed(false);

    // reset timeout
    motor_test_start_ms = 0;
    motor_test_timeout_ms = 0;

    // re-enable failsafes
    g.failsafe_throttle.load();
    g.failsafe_battery_enabled.load();
    g.failsafe_gcs.load();

    // turn off notify leds
    AP_Notify::flags.esc_calibration = false;
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/motors.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define AUTO_DISARMING_DELAY    15  // called at 1hz so 15 seconds

static uint8_t auto_disarming_counter;

// arm_motors_check - checks for pilot input to arm or disarm the copter
// called at 10hz
static void arm_motors_check()
{
    static int16_t arming_counter;

    // ensure throttle is down
    if (g.rc_3.control_in > 0) {
        arming_counter = 0;
        return;
    }

    int16_t tmp = g.rc_4.control_in;

    // full right
    if (tmp > 4000) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if( arming_counter <= AUTO_TRIM_DELAY ) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !motors.armed()) {
            // reset arming counter if arming fail
            if (!init_arm_motors(false)) {
                arming_counter = 0;
            }
        }

        // arm the motors and configure for flight
        if (arming_counter == AUTO_TRIM_DELAY && motors.armed() && control_mode == STABILIZE) {
            auto_trim_counter = 250;
            // ensure auto-disarm doesn't trigger immediately
            auto_disarming_counter = 0;
        }

    // full left
    }else if (tmp < -4000) {
        if (!mode_has_manual_throttle(control_mode) && !ap.land_complete) {
            arming_counter = 0;
            return;
        }

        // increase the counter to a maximum of 1 beyond the disarm delay
        if( arming_counter <= DISARM_DELAY ) {
            arming_counter++;
        }

        // disarm the motors
        if (arming_counter == DISARM_DELAY && motors.armed()) {
            init_disarm_motors();
        }

    // Yaw is centered so reset arming counter
    }else{
        arming_counter = 0;
    }
}

// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 15 seconds
// called at 1hz
static void auto_disarm_check()
{
    // exit immediately if we are already disarmed or throttle is not zero
    if (!motors.armed() || !ap.throttle_zero) {
        auto_disarming_counter = 0;
        return;
    }

    // allow auto disarm in manual flight modes or Loiter/AltHold if we're landed
    if (mode_has_manual_throttle(control_mode) || ap.land_complete) {
        auto_disarming_counter++;

        if(auto_disarming_counter >= AUTO_DISARMING_DELAY) {
            init_disarm_motors();
            auto_disarming_counter = 0;
        }
    }else{
        auto_disarming_counter = 0;
    }
}

// init_arm_motors - performs arming process including initialisation of barometer and gyros
//  returns false if arming failed because of pre-arm checks, arming checks or a gyro calibration failure
static bool init_arm_motors(bool arming_from_gcs)
{
	// arming marker
    // Flag used to track if we have armed the motors the first time.
    // This is used to decide if we should run the ground_start routine
    // which calibrates the IMU
    static bool did_ground_start = false;
    static bool in_arm_motors = false;

    // exit immediately if already in this function
    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    // run pre-arm-checks and display failures
    if(!pre_arm_checks(true) || !arm_checks(true, arming_from_gcs)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }

    // disable cpu failsafe because initialising everything takes a while
    failsafe_disable();

    // reset battery failsafe
    set_failsafe_battery(false);

    // notify that arming will occur (we do this early to give plenty of warning)
    AP_Notify::flags.armed = true;
    // call update_notify a few times to ensure the message gets out
    for (uint8_t i=0; i<=10; i++) {
        update_notify();
    }

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("ARMING MOTORS"));
#endif

    // Remember Orientation
    // --------------------
    init_simple_bearing();

    initial_armed_bearing = ahrs.yaw_sensor;

    // Reset home position if it has already been set before (but not locked)
    if (ap.home_state == HOME_SET_NOT_LOCKED) {
        set_home_to_current_location();
    }
    calc_distance_and_bearing();

    if(did_ground_start == false) {
        startup_ground(true);
        // final check that gyros calibrated successfully
        if (((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) && !ins.gyro_calibrated_ok_all()) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Gyro calibration failed"));
            AP_Notify::flags.armed = false;
            failsafe_enable();
            in_arm_motors = false;
            return false;
        }
        did_ground_start = true;
    }

    // go back to normal AHRS gains
    ahrs.set_fast_gains(false);

    // enable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(true);
    hal.util->set_soft_armed(true);

    // set hover throttle
    motors.set_hover_throttle(g.throttle_mid);

#if SPRAYER == ENABLED
    // turn off sprayer's test if on
    sprayer.test_pump(false);
#endif

    // short delay to allow reading of rc inputs
    delay(30);

    // enable output to motors
    output_min();

    // finally actually arm the motors
    motors.armed(true);

    // log arming to dataflash
    Log_Write_Event(DATA_ARMED);

    // log flight mode in case it was changed while vehicle was disarmed
    DataFlash.Log_Write_Mode(control_mode);

    // reenable failsafe
    failsafe_enable();

    // perf monitor ignores delay due to arming
    perf_ignore_this_loop();

    // flag exiting this function
    in_arm_motors = false;

    // return success
    return true;
}

// perform pre-arm checks and set ap.pre_arm_check flag
//  return true if the checks pass successfully
static bool pre_arm_checks(bool display_failure)
{
    // exit immediately if already armed
    if (motors.armed()) {
        return true;
    }

    // exit immediately if we've already successfully performed the pre-arm check
    if (ap.pre_arm_check) {
        // run gps checks because results may change and affect LED colour
        // no need to display failures because arm_checks will do that if the pilot tries to arm
        pre_arm_gps_checks(false);
        return true;
    }

    // succeed if pre arm checks are disabled
    if(g.arming_check == ARMING_CHECK_NONE) {
        set_pre_arm_check(true);
        set_pre_arm_rc_check(true);
        return true;
    }

    // pre-arm rc checks a prerequisite
    pre_arm_rc_checks();
    if(!ap.pre_arm_rc_check) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: RC not calibrated"));
        }
        return false;
    }
    // check Baro
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_BARO)) {
        // barometer health check
        if(!barometer.all_healthy()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Barometer not healthy"));
            }
            return false;
        }
        // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
        // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
        // that may differ from the baro height due to baro drift.
        nav_filter_status filt_status = inertial_nav.get_filter_status();
        bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
        if (using_baro_ref) {
            if (fabs(inertial_nav.get_altitude() - baro_alt) > PREARM_MAX_ALT_DISPARITY_CM) {
                if (display_failure) {
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Altitude disparity"));
                }
                return false;
            }
        }
    }

    // check Compass
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_COMPASS)) {
        // check the primary compass is healthy
        if(!compass.healthy()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not healthy"));
            }
            return false;
        }

        // check compass learning is on or offsets have been set
        if(!compass.configured()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not calibrated"));
            }
            return false;
        }

        // check for unreasonable compass offsets
        Vector3f offsets = compass.get_offsets();
        if(offsets.length() > COMPASS_OFFSETS_MAX) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass offsets too high"));
            }
            return false;
        }

        // check for unreasonable mag field length
        float mag_field = compass.get_field().length();
        if (mag_field > COMPASS_MAGFIELD_EXPECTED*1.65 || mag_field < COMPASS_MAGFIELD_EXPECTED*0.35) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check mag field"));
            }
            return false;
        }

#if COMPASS_MAX_INSTANCES > 1
        // check all compasses point in roughly same direction
        if (compass.get_count() > 1) {
            Vector3f prime_mag_vec = compass.get_field();
            prime_mag_vec.normalize();
            for(uint8_t i=0; i<compass.get_count(); i++) {
                // get next compass
                Vector3f mag_vec = compass.get_field(i);
                mag_vec.normalize();
                Vector3f vec_diff = mag_vec - prime_mag_vec;
                if (vec_diff.length() > COMPASS_ACCEPTABLE_VECTOR_DIFF) {
                    if (display_failure) {
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: inconsistent compasses"));
                    }
                    return false;
                }
            }
        }
#endif

    }

    // check GPS
    if (!pre_arm_gps_checks(display_failure)) {
        return false;
    }

#if AC_FENCE == ENABLED
    // check fence is initialised
    if(!fence.pre_arm_check()) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: check fence"));
        }
        return false;
    }
#endif

    // check INS
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) {
        // check accelerometers have been calibrated
        if(!ins.calibrated()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: INS not calibrated"));
            }
            return false;
        }

        // check accels are healthy
        if(!ins.get_accel_health_all()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Accelerometers not healthy"));
            }
            return false;
        }

#if INS_MAX_INSTANCES > 1
        // check all accelerometers point in roughly same direction
        if (ins.get_accel_count() > 1) {
            const Vector3f &prime_accel_vec = ins.get_accel();
            for(uint8_t i=0; i<ins.get_accel_count(); i++) {
                // get next accel vector
                const Vector3f &accel_vec = ins.get_accel(i);
                Vector3f vec_diff = accel_vec - prime_accel_vec;
                if (vec_diff.length() > PREARM_MAX_ACCEL_VECTOR_DIFF) {
                    if (display_failure) {
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: inconsistent Accelerometers"));
                    }
                    return false;
                }
            }
        }
#endif

        // check gyros are healthy
        if(!ins.get_gyro_health_all()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Gyros not healthy"));
            }
            return false;
        }

#if INS_MAX_INSTANCES > 1
        // check all gyros are consistent
        if (ins.get_gyro_count() > 1) {
            for(uint8_t i=0; i<ins.get_gyro_count(); i++) {
                // get rotation rate difference between gyro #i and primary gyro
                Vector3f vec_diff = ins.get_gyro(i) - ins.get_gyro();
                if (vec_diff.length() > PREARM_MAX_GYRO_VECTOR_DIFF) {
                    if (display_failure) {
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: inconsistent Gyros"));
                    }
                    return false;
                }
            }
        }
#endif
    }
#if CONFIG_HAL_BOARD != HAL_BOARD_VRBRAIN
#ifndef CONFIG_ARCH_BOARD_PX4FMU_V1
    // check board voltage
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_VOLTAGE)) {
        if(hal.analogin->board_voltage() < BOARD_VOLTAGE_MIN || hal.analogin->board_voltage() > BOARD_VOLTAGE_MAX) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check Board Voltage"));
            }
            return false;
        }
    }
#endif
#endif

    // check various parameter values
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_PARAMETERS)) {

        // ensure ch7 and ch8 have different functions
        if (check_duplicate_auxsw()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Duplicate Aux Switch Options"));
            }
            return false;
        }

        // failsafe parameter checks
        if (g.failsafe_throttle) {
            // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
            if (g.rc_3.radio_min <= g.failsafe_throttle_value+10 || g.failsafe_throttle_value < 910) {
                if (display_failure) {
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check FS_THR_VALUE"));
                }
                return false;
            }
        }

        // lean angle parameter check
        if (aparm.angle_max < 1000 || aparm.angle_max > 8000) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check ANGLE_MAX"));
            }
            return false;
        }

        // acro balance parameter check
        if ((g.acro_balance_roll > g.p_stabilize_roll.kP()) || (g.acro_balance_pitch > g.p_stabilize_pitch.kP())) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: ACRO_BAL_ROLL/PITCH"));
            }
            return false;
        }

#if CONFIG_SONAR == ENABLED
        // check range finder
        if (!sonar.pre_arm_check()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: check range finder"));
            }
            return false;
        }
#endif
    }

    // if we've gotten this far then pre arm checks have completed
    set_pre_arm_check(true);
    return true;
}

// perform pre_arm_rc_checks checks and set ap.pre_arm_rc_check flag
static void pre_arm_rc_checks()
{
    // exit immediately if we've already successfully performed the pre-arm rc check
    if( ap.pre_arm_rc_check ) {
        return;
    }

    // set rc-checks to success if RC checks are disabled
    if ((g.arming_check != ARMING_CHECK_ALL) && !(g.arming_check & ARMING_CHECK_RC)) {
        set_pre_arm_rc_check(true);
        return;
    }

    // check if radio has been calibrated
    if(!g.rc_3.radio_min.load() && !g.rc_3.radio_max.load()) {
        return;
    }

    // check channels 1 & 2 have min <= 1300 and max >= 1700
    if (g.rc_1.radio_min > 1300 || g.rc_1.radio_max < 1700 || g.rc_2.radio_min > 1300 || g.rc_2.radio_max < 1700) {
        return;
    }

    // check channels 3 & 4 have min <= 1300 and max >= 1700
    if (g.rc_3.radio_min > 1300 || g.rc_3.radio_max < 1700 || g.rc_4.radio_min > 1300 || g.rc_4.radio_max < 1700) {
        return;
    }

    // check channels 1 & 2 have trim >= 1300 and <= 1700
    if (g.rc_1.radio_trim < 1300 || g.rc_1.radio_trim > 1700 || g.rc_2.radio_trim < 1300 || g.rc_2.radio_trim > 1700) {
        return;
    }

    // check channel 4 has trim >= 1300 and <= 1700
    if (g.rc_4.radio_trim < 1300 || g.rc_4.radio_trim > 1700) {
        return;
    }

    // if we've gotten this far rc is ok
    set_pre_arm_rc_check(true);
}

// performs pre_arm gps related checks and returns true if passed
static bool pre_arm_gps_checks(bool display_failure)
{
    // always check if inertial nav has started and is ready
    if(!ahrs.healthy()) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Waiting for Nav Checks"));
        }
        return false;
    }

    // return true immediately if gps check is disabled
    if (!(g.arming_check == ARMING_CHECK_ALL || g.arming_check & ARMING_CHECK_GPS)) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // check if flight mode requires GPS
    bool gps_required = mode_requires_GPS(control_mode);

#if AC_FENCE == ENABLED
    // if circular fence is enabled we need GPS
    if ((fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) != 0) {
        gps_required = true;
    }
#endif

    // return true if GPS is not required
    if (!gps_required) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // ensure GPS is ok
    if (!position_ok()) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Need 3D Fix"));
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // check home and EKF origin are not too far
    if (far_from_EKF_origin(ahrs.get_home())) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: EKF-home variance"));
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // warn about hdop separately - to prevent user confusion with no gps lock
    if (gps.get_hdop() > g.gps_hdop_good) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: High GPS HDOP"));
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // if we got here all must be ok
    AP_Notify::flags.pre_arm_gps_check = true;
    return true;
}

// arm_checks - perform final checks before arming
//  always called just before arming.  Return true if ok to arm
//  has side-effect that logging is started
static bool arm_checks(bool display_failure, bool arming_from_gcs)
{
#if LOGGING_ENABLED == ENABLED
    // start dataflash
    start_logging();
#endif

    // always check if inertial nav has started and is ready
    if(!ahrs.healthy()) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Waiting for Nav Checks"));
        }
        return false;
    }

    // always check if the current mode allows arming
    if (!mode_allows_arming(control_mode, arming_from_gcs)) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Mode not armable"));
        }
        return false;
    }

    // always check if rotor is spinning on heli
    #if FRAME_CONFIG == HELI_FRAME
    // heli specific arming check
    if (!motors.allow_arming()){
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Rotor not spinning"));
        }
        return false;
    }
    #endif  // HELI_FRAME

    // succeed if arming checks are disabled
    if (g.arming_check == ARMING_CHECK_NONE) {
        return true;
    }

    // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
    // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
    // that may differ from the baro height due to baro drift.
    nav_filter_status filt_status = inertial_nav.get_filter_status();
    bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
    if (((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_BARO)) && using_baro_ref) {
        if (fabs(inertial_nav.get_altitude() - baro_alt) > PREARM_MAX_ALT_DISPARITY_CM) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Altitude disparity"));
            }
            return false;
        }
    }

    // check gps
    if (!pre_arm_gps_checks(display_failure)) {
        return false;
    }

    // check lean angle
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) {
        if (labs(ahrs.roll_sensor) > aparm.angle_max || labs(ahrs.pitch_sensor) > aparm.angle_max) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Leaning"));
            }
            return false;
        }
    }

    // check throttle
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_RC)) {
        // check throttle is not too low - must be above failsafe throttle
        if (g.failsafe_throttle != FS_THR_DISABLED && g.rc_3.radio_in < g.failsafe_throttle_value) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Throttle below Failsafe"));
            }
            return false;
        }

        // check throttle is not too high - skips checks if arming from GCS in Guided
        if (!(arming_from_gcs && control_mode == GUIDED)) {
            // above top of deadband is too always high
            if (g.rc_3.control_in > (g.rc_3.get_control_mid() + g.throttle_deadzone)) {
                if (display_failure) {
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Throttle too high"));
                }
                return false;
            }
            // in manual modes throttle must be at zero
            if ((mode_has_manual_throttle(control_mode) || control_mode == DRIFT) && g.rc_3.control_in > 0) {
                if (display_failure) {
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Throttle too high"));
                }
                return false;
            }
        }
    }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Safety Switch"));
        }
        return false;
    }

    // if we've gotten this far all is ok
    return true;
}

// init_disarm_motors - disarm motors
static void init_disarm_motors()
{
    // return immediately if we are already disarmed
    if (!motors.armed()) {
        return;
    }

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("DISARMING MOTORS"));
#endif

    motors.armed(false);

    // save compass offsets learned by the EKF
    Vector3f magOffsets;
    if (ahrs.use_compass() && ahrs.getMagOffsets(magOffsets)) {
        compass.set_and_save_offsets(compass.get_primary(), magOffsets);
    }

#if AUTOTUNE_ENABLED == ENABLED
    // save auto tuned parameters
    autotune_save_tuning_gains();
#endif

    // we are not in the air
    set_land_complete(true);
    set_land_complete_maybe(true);

    // reset the mission
    mission.reset();

    // setup fast AHRS gains to get right attitude
    ahrs.set_fast_gains(true);

    // log disarm to the dataflash
    Log_Write_Event(DATA_DISARMED);

    // suspend logging
    if (!(g.log_bitmask & MASK_LOG_WHEN_DISARMED)) {
        DataFlash.EnableWrites(false);
    }

    // disable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(false);
    hal.util->set_soft_armed(false);
}

// motors_output - send output to motors library which will adjust and send to ESCs and servos
static void motors_output()
{
    // check if we are performing the motor test
    if (ap.motor_test) {
        motor_test_output();
    } else {
        motors.output();
    }
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/navigation.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
static void run_nav_updates(void)
{
    // fetch position from inertial navigation
    calc_position();

    // calculate distance and bearing for reporting and autopilot decisions
    calc_distance_and_bearing();

    // run autopilot to make high level decisions about control modes
    run_autopilot();
}

// calc_position - get lat and lon positions from inertial nav library
static void calc_position()
{
    // pull position from interial nav library
    current_loc.lng = inertial_nav.get_longitude();
    current_loc.lat = inertial_nav.get_latitude();
}

// calc_distance_and_bearing - calculate distance and bearing to next waypoint and home
static void calc_distance_and_bearing()
{
    calc_wp_distance();
    calc_wp_bearing();
    calc_home_distance_and_bearing();
}

// calc_wp_distance - calculate distance to next waypoint for reporting and autopilot decisions
static void calc_wp_distance()
{
    // get target from loiter or wpinav controller
    if (control_mode == LOITER || control_mode == CIRCLE) {
        wp_distance = wp_nav.get_loiter_distance_to_target();
    }else if (control_mode == AUTO || control_mode == RTL || (control_mode == GUIDED && guided_mode == Guided_WP)) {
        wp_distance = wp_nav.get_wp_distance_to_destination();
    }else{
        wp_distance = 0;
    }
}

// calc_wp_bearing - calculate bearing to next waypoint for reporting and autopilot decisions
static void calc_wp_bearing()
{
    // get target from loiter or wpinav controller
    if (control_mode == LOITER || control_mode == CIRCLE) {
        wp_bearing = wp_nav.get_loiter_bearing_to_target();
    } else if (control_mode == AUTO || control_mode == RTL || (control_mode == GUIDED && guided_mode == Guided_WP)) {
        wp_bearing = wp_nav.get_wp_bearing_to_destination();
    } else {
        wp_bearing = 0;
    }
}

// calc_home_distance_and_bearing - calculate distance and bearing to home for reporting and autopilot decisions
static void calc_home_distance_and_bearing()
{
    Vector3f curr = inertial_nav.get_position();

    // calculate home distance and bearing
    if (position_ok()) {
        home_distance = pythagorous2(curr.x, curr.y);
        Vector3f home = pv_location_to_vector(ahrs.get_home());
        home_bearing = pv_get_bearing_cd(curr,home);

        // update super simple bearing (if required) because it relies on home_bearing
        update_super_simple_bearing(false);
    }
}

// run_autopilot - highest level call to process mission commands
static void run_autopilot()
{
    if (control_mode == AUTO) {
        // update state of mission
        // may call commands_process.pde's start_command and verify_command functions
        mission.update();
    }
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/perf_info.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
//  high level performance monitoring
//
//  we measure the main loop time
//

#if MAIN_LOOP_RATE == 400
 # define PERF_INFO_OVERTIME_THRESHOLD_MICROS 3000
#else
 # define PERF_INFO_OVERTIME_THRESHOLD_MICROS 10500
#endif

static uint16_t perf_info_loop_count;
static uint32_t perf_info_max_time;
static uint32_t perf_info_min_time;
static uint16_t perf_info_long_running;
static bool perf_ignore_loop = false;

// perf_info_reset - reset all records of loop time to zero
static void perf_info_reset()
{
    perf_info_loop_count = 0;
    perf_info_max_time = 0;
    perf_info_min_time = 0;
    perf_info_long_running = 0;
}

// perf_ignore_loop - ignore this loop from performance measurements (used to reduce false positive when arming)
static void perf_ignore_this_loop()
{
    perf_ignore_loop = true;
}

// perf_info_check_loop_time - check latest loop time vs min, max and overtime threshold
static void perf_info_check_loop_time(uint32_t time_in_micros)
{
    perf_info_loop_count++;

    // exit if this loop should be ignored
    if (perf_ignore_loop) {
        perf_ignore_loop = false;
        return;
    }

    if( time_in_micros > perf_info_max_time) {
        perf_info_max_time = time_in_micros;
    }
    if( perf_info_min_time == 0 || time_in_micros < perf_info_min_time) {
        perf_info_min_time = time_in_micros;
    }
    if( time_in_micros > PERF_INFO_OVERTIME_THRESHOLD_MICROS ) {
        perf_info_long_running++;
    }
}

// perf_info_get_long_running_percentage - get number of long running loops as a percentage of the total number of loops
static uint16_t perf_info_get_num_loops()
{
    return perf_info_loop_count;
}

// perf_info_get_max_time - return maximum loop time (in microseconds)
static uint32_t perf_info_get_max_time()
{
    return perf_info_max_time;
}

// perf_info_get_max_time - return maximum loop time (in microseconds)
static uint32_t perf_info_get_min_time()
{
    return perf_info_min_time;
}

// perf_info_get_num_long_running - get number of long running loops
static uint16_t perf_info_get_num_long_running()
{
    return perf_info_long_running;
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/position_vector.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// position_vector.pde related utility functions

// position vectors are Vector3f
//    .x = latitude from home in cm
//    .y = longitude from home in cm
//    .z = altitude above home in cm

// pv_location_to_vector - convert lat/lon coordinates to a position vector
Vector3f pv_location_to_vector(const Location& loc)
{
    const struct Location &origin = inertial_nav.get_origin();
    float alt_above_origin = pv_alt_above_origin(loc.alt);  // convert alt-relative-to-home to alt-relative-to-origin
    return Vector3f((loc.lat-origin.lat) * LATLON_TO_CM, (loc.lng-origin.lng) * LATLON_TO_CM * scaleLongDown, alt_above_origin);
}

// pv_location_to_vector_with_default - convert lat/lon coordinates to a position vector,
//     defaults to the default_posvec's lat/lon and alt if the provided lat/lon or alt are zero
Vector3f pv_location_to_vector_with_default(const Location& loc, const Vector3f& default_posvec)
{
    Vector3f pos = pv_location_to_vector(loc);

    // set target altitude to default if not provided
    if (loc.alt == 0) {
        pos.z = default_posvec.z;
    }

    // set target position to default if not provided
    if (loc.lat == 0 && loc.lng == 0) {
        pos.x = default_posvec.x;
        pos.y = default_posvec.y;
    }

    return pos;
}

// pv_alt_above_origin - convert altitude above home to altitude above EKF origin
float pv_alt_above_origin(float alt_above_home_cm)
{
    const struct Location &origin = inertial_nav.get_origin();
    return alt_above_home_cm + (ahrs.get_home().alt - origin.alt);
}

// pv_alt_above_home - convert altitude above EKF origin to altitude above home
float pv_alt_above_home(float alt_above_origin_cm)
{
    const struct Location &origin = inertial_nav.get_origin();
    return alt_above_origin_cm + (origin.alt - ahrs.get_home().alt);
}

// pv_get_bearing_cd - return bearing in centi-degrees between two positions
float pv_get_bearing_cd(const Vector3f &origin, const Vector3f &destination)
{
    float bearing = fast_atan2(destination.y-origin.y, destination.x-origin.x) * DEGX100;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}

// pv_get_horizontal_distance_cm - return distance between two positions in cm
float pv_get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination)
{
    return pythagorous2(destination.x-origin.x,destination.y-origin.y);
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/radio.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

static void default_dead_zones()
{
    g.rc_1.set_default_dead_zone(30);
    g.rc_2.set_default_dead_zone(30);
#if FRAME_CONFIG == HELI_FRAME
    g.rc_3.set_default_dead_zone(10);
    g.rc_4.set_default_dead_zone(15);
    g.rc_8.set_default_dead_zone(10);
#else
    g.rc_3.set_default_dead_zone(30);
    g.rc_4.set_default_dead_zone(40);
#endif
    g.rc_6.set_default_dead_zone(0);
}

static void init_rc_in()
{
    // set rc channel ranges
    g.rc_1.set_angle(ROLL_PITCH_INPUT_MAX);
    g.rc_2.set_angle(ROLL_PITCH_INPUT_MAX);
    g.rc_3.set_range(g.throttle_min, THR_MAX);
    g.rc_4.set_angle(4500);

    g.rc_1.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    g.rc_2.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    g.rc_4.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

    //set auxiliary servo ranges
    g.rc_5.set_range(0,1000);
    g.rc_6.set_range(0,1000);
    g.rc_7.set_range(0,1000);
    g.rc_8.set_range(0,1000);

    // set default dead zones
    default_dead_zones();

    // initialise throttle_zero flag
    ap.throttle_zero = true;
}

 // init_rc_out -- initialise motors and check if pilot wants to perform ESC calibration
static void init_rc_out()
{
    motors.set_update_rate(g.rc_speed);
    motors.set_frame_orientation(g.frame_orientation);
    motors.Init();                                              // motor initialisation
    motors.set_min_throttle(g.throttle_min);

    for(uint8_t i = 0; i < 5; i++) {
        delay(20);
        read_radio();
    }

    // we want the input to be scaled correctly
    g.rc_3.set_range_out(0,1000);

    // check if we should enter esc calibration mode
    esc_calibration_startup_check();

    // enable output to motors
    pre_arm_rc_checks();
    if (ap.pre_arm_rc_check) {
        output_min();
    }

    // setup correct scaling for ESCs like the UAVCAN PX4ESC which
    // take a proportion of speed. Note: this assumes rc_3 is
    // throttle and should really use rcmap.
    hal.rcout->set_esc_scaling(g.rc_3.radio_min, g.rc_3.radio_max);
}

// output_min - enable and output lowest possible value to motors
void output_min()
{
    // enable motors
    motors.enable();
    motors.output_min();
}

static void read_radio()
{
    static uint32_t last_update_ms = 0;
    uint32_t tnow_ms = millis();

    if (hal.rcin->new_input()) {
        last_update_ms = tnow_ms;
        ap.new_radio_frame = true;
        uint16_t periods[8];
        hal.rcin->read(periods,8);
        g.rc_1.set_pwm(periods[rcmap.roll()-1]);
        g.rc_2.set_pwm(periods[rcmap.pitch()-1]);

        set_throttle_and_failsafe(periods[rcmap.throttle()-1]);
        set_throttle_zero_flag(g.rc_3.control_in);

        g.rc_4.set_pwm(periods[rcmap.yaw()-1]);
        g.rc_5.set_pwm(periods[4]);
        g.rc_6.set_pwm(periods[5]);
        g.rc_7.set_pwm(periods[6]);
        g.rc_8.set_pwm(periods[7]);

        // read channels 9 ~ 14
        for (uint8_t i=8; i<RC_MAX_CHANNELS; i++) {
            if (RC_Channel::rc_channel(i) != NULL) {
                RC_Channel::rc_channel(i)->set_pwm(RC_Channel::rc_channel(i)->read());
            }
        }

        // flag we must have an rc receiver attached
        if (!failsafe.rc_override_active) {
            ap.rc_receiver_present = true;
        }

        // update output on any aux channels, for manual passthru
        RC_Channel_aux::output_ch_all();
    }else{
        uint32_t elapsed = tnow_ms - last_update_ms;
        // turn on throttle failsafe if no update from the RC Radio for 500ms or 2000ms if we are using RC_OVERRIDE
        if (((!failsafe.rc_override_active && (elapsed >= FS_RADIO_TIMEOUT_MS)) || (failsafe.rc_override_active && (elapsed >= FS_RADIO_RC_OVERRIDE_TIMEOUT_MS))) &&
            (g.failsafe_throttle && (ap.rc_receiver_present||motors.armed()) && !failsafe.radio)) {
            Log_Write_Error(ERROR_SUBSYSTEM_RADIO, ERROR_CODE_RADIO_LATE_FRAME);
            set_failsafe_radio(true);
        }
    }
}

#define FS_COUNTER 3        // radio failsafe kicks in after 3 consecutive throttle values below failsafe_throttle_value
static void set_throttle_and_failsafe(uint16_t throttle_pwm)
{
    // if failsafe not enabled pass through throttle and exit
    if(g.failsafe_throttle == FS_THR_DISABLED) {
        g.rc_3.set_pwm(throttle_pwm);
        return;
    }

    //check for low throttle value
    if (throttle_pwm < (uint16_t)g.failsafe_throttle_value) {

        // if we are already in failsafe or motors not armed pass through throttle and exit
        if (failsafe.radio || !(ap.rc_receiver_present || motors.armed())) {
            g.rc_3.set_pwm(throttle_pwm);
            return;
        }

        // check for 3 low throttle values
        // Note: we do not pass through the low throttle until 3 low throttle values are recieved
        failsafe.radio_counter++;
        if( failsafe.radio_counter >= FS_COUNTER ) {
            failsafe.radio_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
            set_failsafe_radio(true);
            g.rc_3.set_pwm(throttle_pwm);   // pass through failsafe throttle
        }
    }else{
        // we have a good throttle so reduce failsafe counter
        failsafe.radio_counter--;
        if( failsafe.radio_counter <= 0 ) {
            failsafe.radio_counter = 0;   // check to ensure we don't underflow the counter

            // disengage failsafe after three (nearly) consecutive valid throttle values
            if (failsafe.radio) {
                set_failsafe_radio(false);
            }
        }
        // pass through throttle
        g.rc_3.set_pwm(throttle_pwm);
    }
}

#define THROTTLE_ZERO_DEBOUNCE_TIME_MS 400
// set_throttle_zero_flag - set throttle_zero flag from debounced throttle control_in
static void set_throttle_zero_flag(int16_t throttle_control)
{
    static uint32_t last_nonzero_throttle_ms = 0;
    uint32_t tnow_ms = millis();

    // if non-zero throttle immediately set as non-zero
    if (throttle_control > 0) {
        last_nonzero_throttle_ms = tnow_ms;
        ap.throttle_zero = false;
    } else if (tnow_ms - last_nonzero_throttle_ms > THROTTLE_ZERO_DEBOUNCE_TIME_MS) {
        ap.throttle_zero = true;
    }
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/sensors.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

static void init_barometer(bool full_calibration)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));
    if (full_calibration) {
        barometer.calibrate();
    }else{
        barometer.update_calibration();
    }
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

// return barometric altitude in centimeters
static void read_barometer(void)
{
    barometer.update();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }
    baro_alt = barometer.get_altitude() * 100.0f;
    baro_climbrate = barometer.get_climb_rate() * 100.0f;
}

#if CONFIG_SONAR == ENABLED
static void init_sonar(void)
{
   sonar.init();
}
#endif

// return sonar altitude in centimeters
static int16_t read_sonar(void)
{
#if CONFIG_SONAR == ENABLED
    sonar.update();

    // exit immediately if sonar is disabled
    if (!sonar_enabled || (sonar.status() != RangeFinder::RangeFinder_Good)) {
        sonar_alt_health = 0;
        return 0;
    }

    int16_t temp_alt = sonar.distance_cm();

    if (temp_alt >= sonar.min_distance_cm() && 
        temp_alt <= sonar.max_distance_cm() * SONAR_RELIABLE_DISTANCE_PCT) {
        if ( sonar_alt_health < SONAR_ALT_HEALTH_MAX ) {
            sonar_alt_health++;
        }
    }else{
        sonar_alt_health = 0;
    }

 #if SONAR_TILT_CORRECTION == 1
    // correct alt for angle of the sonar
    float temp = ahrs.cos_pitch() * ahrs.cos_roll();
    temp = max(temp, 0.707f);
    temp_alt = (float)temp_alt * temp;
 #endif

    return temp_alt;
#else
    return 0;
#endif
}

// initialise compass
static void init_compass()
{
    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        cliSerial->println_P(PSTR("COMPASS INIT ERROR"));
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
        return;
    }
    ahrs.set_compass(&compass);
}

// initialise optical flow sensor
static void init_optflow()
{
#if OPTFLOW == ENABLED
    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }

    // initialise optical flow sensor
    optflow.init();
#endif      // OPTFLOW == ENABLED
}

// called at 200hz
#if OPTFLOW == ENABLED
static void update_optical_flow(void)
{
    static uint32_t last_of_update = 0;

    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }

    // read from sensor
    optflow.update();

    // write to log and send to EKF if new data has arrived
    if (optflow.last_update() != last_of_update) {
        last_of_update = optflow.last_update();
        uint8_t flowQuality = optflow.quality();
        Vector2f flowRate = optflow.flowRate();
        Vector2f bodyRate = optflow.bodyRate();
        ahrs.writeOptFlowMeas(flowQuality, flowRate, bodyRate, last_of_update);
        if (g.log_bitmask & MASK_LOG_OPTFLOW) {
            Log_Write_Optflow();
        }
    }
}
#endif  // OPTFLOW == ENABLED

// read_battery - check battery voltage and current and invoke failsafe if necessary
// called at 10hz
static void read_battery(void)
{
    battery.read();

    // update compass with current value
    if (battery.has_current()) {
        compass.set_current(battery.current_amps());
    }

    // update motors with voltage and current
    if (battery.get_type() != AP_BattMonitor::BattMonitor_TYPE_NONE) {
        motors.set_voltage(battery.voltage());
    }
    if (battery.has_current()) {
        motors.set_current(battery.current_amps());
    }

    // check for low voltage or current if the low voltage check hasn't already been triggered
    // we only check when we're not powered by USB to avoid false alarms during bench tests
    if (!ap.usb_connected && !failsafe.battery && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        failsafe_battery_event();
    }

    // log battery info to the dataflash
    if (should_log(MASK_LOG_CURRENT)) {
        Log_Write_Current();
    }
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    // avoid divide by zero
    if (g.rssi_range <= 0) {
        receiver_rssi = 0;
    }else{
        rssi_analog_source->set_pin(g.rssi_pin);
        float ret = rssi_analog_source->voltage_average() * 255 / g.rssi_range;
        receiver_rssi = constrain_int16(ret, 0, 255);
    }
}

#if EPM_ENABLED == ENABLED
// epm update - moves epm pwm output back to neutral after grab or release is completed
void epm_update()
{
    epm.update();
}
#endif
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/setup.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_75
#define WITH_ESC_CALIB
#endif

// Functions called from the setup menu
static int8_t   setup_factory           (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_show              (uint8_t argc, const Menu::arg *argv);
static int8_t   setup_set               (uint8_t argc, const Menu::arg *argv);
#ifdef WITH_ESC_CALIB
static int8_t   esc_calib               (uint8_t argc, const Menu::arg *argv);
#endif

// Command/function table for the setup menu
const struct Menu::command setup_menu_commands[] PROGMEM = {
    // command			function called
    // =======          ===============
    {"reset",                       setup_factory},
    {"show",                        setup_show},
    {"set",                         setup_set},
#ifdef WITH_ESC_CALIB
    {"esc_calib",                   esc_calib},
#endif
};

// Create the setup menu object.
MENU(setup_menu, "setup", setup_menu_commands);

// Called from the top-level menu to run the setup menu.
static int8_t
setup_mode(uint8_t argc, const Menu::arg *argv)
{
    // Give the user some guidance
    cliSerial->printf_P(PSTR("Setup Mode\n\n\n"));

    // Run the setup menu.  When the menu exits, we will return to the main menu.
    setup_menu.run();
    return 0;
}

// Initialise the EEPROM to 'factory' settings (mostly defined in APM_Config.h or via defaults).
// Called by the setup menu 'factoryreset' command.
static int8_t
setup_factory(uint8_t argc, const Menu::arg *argv)
{
    int16_t c;

    cliSerial->printf_P(PSTR("\n'Y' = factory reset, any other key to abort:\n"));

    do {
        c = cliSerial->read();
    } while (-1 == c);

    if (('y' != c) && ('Y' != c))
        return(-1);

    AP_Param::erase_all();
    cliSerial->printf_P(PSTR("\nReboot board"));

    delay(1000);

    for (;; ) {
    }
    // note, cannot actually return here
    return(0);
}

//Set a parameter to a specified value. It will cast the value to the current type of the
//parameter and make sure it fits in case of INT8 and INT16
static int8_t setup_set(uint8_t argc, const Menu::arg *argv)
{
    int8_t value_int8;
    int16_t value_int16;

    AP_Param *param;
    enum ap_var_type p_type;

    if(argc!=3)
    {
        cliSerial->printf_P(PSTR("Invalid command. Usage: set <name> <value>\n"));
        return 0;
    }

    param = AP_Param::find(argv[1].str, &p_type);
    if(!param)
    {
        cliSerial->printf_P(PSTR("Param not found: %s\n"), argv[1].str);
        return 0;
    }

    switch(p_type)
    {
        case AP_PARAM_INT8:
            value_int8 = (int8_t)(argv[2].i);
            if(argv[2].i!=value_int8)
            {
                cliSerial->printf_P(PSTR("Value out of range for type INT8\n"));
                return 0;
            }
            ((AP_Int8*)param)->set_and_save(value_int8);
            break;
        case AP_PARAM_INT16:
            value_int16 = (int16_t)(argv[2].i);
            if(argv[2].i!=value_int16)
            {
                cliSerial->printf_P(PSTR("Value out of range for type INT16\n"));
                return 0;
            }
            ((AP_Int16*)param)->set_and_save(value_int16);
            break;

        //int32 and float don't need bounds checking, just use the value provoded by Menu::arg
        case AP_PARAM_INT32:
            ((AP_Int32*)param)->set_and_save(argv[2].i);
            break;
        case AP_PARAM_FLOAT:
            ((AP_Float*)param)->set_and_save(argv[2].f);
            break;
        default:
            cliSerial->printf_P(PSTR("Cannot set parameter of type %d.\n"), p_type);
            break;
    }

    return 0;
}

// Print the current configuration.
// Called by the setup menu 'show' command.
static int8_t
setup_show(uint8_t argc, const Menu::arg *argv)
{
    AP_Param *param;
    ap_var_type type;

    //If a parameter name is given as an argument to show, print only that parameter
    if(argc>=2)
    {

        param=AP_Param::find(argv[1].str, &type);

        if(!param)
        {
            cliSerial->printf_P(PSTR("Parameter not found: '%s'\n"), argv[1]);
            return 0;
        }
        AP_Param::show(param, argv[1].str, type, cliSerial);
        return 0;
    }

    // clear the area
    print_blanks(8);

    report_version();
    report_radio();
    report_frame();
    report_batt_monitor();
    report_flight_modes();
    report_ins();
    report_compass();
    report_optflow();

    AP_Param::show_all(cliSerial);

    return(0);
}

#ifdef WITH_ESC_CALIB
#define PWM_CALIB_MIN 1000
#define PWM_CALIB_MAX 2000
#define PWM_HIGHEST_MAX 2200
#define PWM_LOWEST_MAX 1200
#define PWM_HIGHEST_MIN 1800
#define PWM_LOWEST_MIN 800

static int8_t
esc_calib(uint8_t argc,const Menu::arg *argv)
{


	char c;
	unsigned max_channels = 0;
	uint32_t set_mask = 0;

	uint16_t pwm_high = PWM_CALIB_MAX;
	uint16_t pwm_low = PWM_CALIB_MIN;


	if (argc < 2) {
		cliSerial->printf_P(PSTR("Pls provide Channel Mask\n"
                                    "\tusage: esc_calib 1010 - enables calibration for 2nd and 4th Motor\n"));
        return(0);
	}
    

	
    set_mask = strtol (argv[1].str, NULL, 2);
	if (set_mask == 0)
		cliSerial->printf_P(PSTR("no channels chosen"));
    //cliSerial->printf_P(PSTR("\n%d\n"),set_mask);
    set_mask<<=1;
	/* wait 50 ms */
	hal.scheduler->delay(50);


	cliSerial->printf_P(PSTR("\nATTENTION, please remove or fix propellers before starting calibration!\n"
	       "\n"
	       "Make sure\n"
	       "\t - that the ESCs are not powered\n"
	       "\t - that safety is off\n"
	       "\t - that the controllers are stopped\n"
	       "\n"
	       "Do you want to start calibration now: y or n?\n"));

	/* wait for user input */
	while (1) {
            c= cliSerial->read();
			if (c == 'y' || c == 'Y') {

				break;

			} else if (c == 0x03 || c == 0x63 || c == 'q') {
				cliSerial->printf_P(PSTR("ESC calibration exited\n"));
				return(0);

			} else if (c == 'n' || c == 'N') {
				cliSerial->printf_P(PSTR("ESC calibration aborted\n"));
				return(0);

			} 

		/* rate limit to ~ 20 Hz */
		hal.scheduler->delay(50);
	}


	/* get number of channels available on the device */
	max_channels = AP_MOTORS_MAX_NUM_MOTORS;

	/* tell IO/FMU that the system is armed (it will output values if safety is off) */
	motors.armed(true);


	cliSerial->printf_P(PSTR("Outputs armed\n"));


	/* wait for user confirmation */
	cliSerial->printf_P(PSTR("\nHigh PWM set: %d\n"
	       "\n"
	       "Connect battery now and hit c+ENTER after the ESCs confirm the first calibration step\n"
	       "\n"), pwm_high);

	while (1) {
		/* set max PWM */
		for (unsigned i = 0; i < max_channels; i++) {

			if (set_mask & 1<<i) {
				motors.output_test(i, pwm_high);
			}
		}
        c = cliSerial->read();
            
		if (c == 'c') {
            break;

		} else if (c == 0x03 || c == 0x63 || c == 'q') {
			cliSerial->printf_P(PSTR("ESC calibration exited\n"));
			return(0);
		}
        
		/* rate limit to ~ 20 Hz */
		hal.scheduler->delay(50);
	}

	cliSerial->printf_P(PSTR("Low PWM set: %d\n"
	       "\n"
	       "Hit c+Enter when finished\n"
	       "\n"), pwm_low);

	while (1) {

		/* set disarmed PWM */
		for (unsigned i = 0; i < max_channels; i++) {
			if (set_mask & 1<<i) {
				motors.output_test(i, pwm_low);
			}
		}
		c = cliSerial->read();

		if (c == 'c') {

			break;

		} else if (c == 0x03 || c == 0x63 || c == 'q') {
			cliSerial->printf_P(PSTR("ESC calibration exited\n"));
			return(0);
		}
		
		/* rate limit to ~ 20 Hz */
		hal.scheduler->delay(50);
	}

	/* disarm */
	motors.armed(false);
    
	cliSerial->printf_P(PSTR("Outputs disarmed\n"));

	cliSerial->printf_P(PSTR("ESC calibration finished\n"));

	return(0);
}
#endif // WITH_ESC_CALIB


/***************************************************************************/
// CLI reports
/***************************************************************************/

static void report_batt_monitor()
{
    cliSerial->printf_P(PSTR("\nBatt Mon:\n"));
    print_divider();
    if (battery.num_instances() == 0) {
        print_enabled(false);
    } else if (!battery.has_current()) {
        cliSerial->printf_P(PSTR("volts"));
    } else {
        cliSerial->printf_P(PSTR("volts and cur"));
    }
    print_blanks(2);
}

static void report_frame()
{
    cliSerial->printf_P(PSTR("Frame\n"));
    print_divider();

 #if FRAME_CONFIG == QUAD_FRAME
    cliSerial->printf_P(PSTR("Quad frame\n"));
 #elif FRAME_CONFIG == TRI_FRAME
    cliSerial->printf_P(PSTR("TRI frame\n"));
 #elif FRAME_CONFIG == HEXA_FRAME
    cliSerial->printf_P(PSTR("Hexa frame\n"));
 #elif FRAME_CONFIG == Y6_FRAME
    cliSerial->printf_P(PSTR("Y6 frame\n"));
 #elif FRAME_CONFIG == OCTA_FRAME
    cliSerial->printf_P(PSTR("Octa frame\n"));
 #elif FRAME_CONFIG == HELI_FRAME
    cliSerial->printf_P(PSTR("Heli frame\n"));
 #endif

    print_blanks(2);
}

static void report_radio()
{
    cliSerial->printf_P(PSTR("Radio\n"));
    print_divider();
    // radio
    print_radio_values();
    print_blanks(2);
}

static void report_ins()
{
    cliSerial->printf_P(PSTR("INS\n"));
    print_divider();

    print_gyro_offsets();
    print_accel_offsets_and_scaling();
    print_blanks(2);
}

static void report_flight_modes()
{
    cliSerial->printf_P(PSTR("Flight modes\n"));
    print_divider();

    for(int16_t i = 0; i < 6; i++ ) {
        print_switch(i, flight_modes[i], BIT_IS_SET(g.simple_modes, i));
    }
    print_blanks(2);
}

void report_optflow()
{
 #if OPTFLOW == ENABLED
    cliSerial->printf_P(PSTR("OptFlow\n"));
    print_divider();

    print_enabled(optflow.enabled());

    print_blanks(2);
 #endif     // OPTFLOW == ENABLED
}

/***************************************************************************/
// CLI utilities
/***************************************************************************/

static void
print_radio_values()
{
    cliSerial->printf_P(PSTR("CH1: %d | %d\n"), (int)g.rc_1.radio_min, (int)g.rc_1.radio_max);
    cliSerial->printf_P(PSTR("CH2: %d | %d\n"), (int)g.rc_2.radio_min, (int)g.rc_2.radio_max);
    cliSerial->printf_P(PSTR("CH3: %d | %d\n"), (int)g.rc_3.radio_min, (int)g.rc_3.radio_max);
    cliSerial->printf_P(PSTR("CH4: %d | %d\n"), (int)g.rc_4.radio_min, (int)g.rc_4.radio_max);
    cliSerial->printf_P(PSTR("CH5: %d | %d\n"), (int)g.rc_5.radio_min, (int)g.rc_5.radio_max);
    cliSerial->printf_P(PSTR("CH6: %d | %d\n"), (int)g.rc_6.radio_min, (int)g.rc_6.radio_max);
    cliSerial->printf_P(PSTR("CH7: %d | %d\n"), (int)g.rc_7.radio_min, (int)g.rc_7.radio_max);
    cliSerial->printf_P(PSTR("CH8: %d | %d\n"), (int)g.rc_8.radio_min, (int)g.rc_8.radio_max);
}

static void
print_switch(uint8_t p, uint8_t m, bool b)
{
    cliSerial->printf_P(PSTR("Pos %d:\t"),p);
    print_flight_mode(cliSerial, m);
    cliSerial->printf_P(PSTR(",\t\tSimple: "));
    if(b)
        cliSerial->printf_P(PSTR("ON\n"));
    else
        cliSerial->printf_P(PSTR("OFF\n"));
}

static void
print_accel_offsets_and_scaling(void)
{
    const Vector3f &accel_offsets = ins.get_accel_offsets();
    const Vector3f &accel_scale = ins.get_accel_scale();
    cliSerial->printf_P(PSTR("A_off: %4.2f, %4.2f, %4.2f\nA_scale: %4.2f, %4.2f, %4.2f\n"),
                    (float)accel_offsets.x,                           // Pitch
                    (float)accel_offsets.y,                           // Roll
                    (float)accel_offsets.z,                           // YAW
                    (float)accel_scale.x,                             // Pitch
                    (float)accel_scale.y,                             // Roll
                    (float)accel_scale.z);                            // YAW
}

static void
print_gyro_offsets(void)
{
    const Vector3f &gyro_offsets = ins.get_gyro_offsets();
    cliSerial->printf_P(PSTR("G_off: %4.2f, %4.2f, %4.2f\n"),
                    (float)gyro_offsets.x,
                    (float)gyro_offsets.y,
                    (float)gyro_offsets.z);
}

#endif // CLI_ENABLED

// report_compass - displays compass information.  Also called by compassmot.pde
static void report_compass()
{
    cliSerial->printf_P(PSTR("Compass\n"));
    print_divider();

    print_enabled(g.compass_enabled);

    // mag declination
    cliSerial->printf_P(PSTR("Mag Dec: %4.4f\n"),
                    degrees(compass.get_declination()));

    // mag offsets
    Vector3f offsets;
    for (uint8_t i=0; i<compass.get_count(); i++) {
        offsets = compass.get_offsets(i);
        // mag offsets
        cliSerial->printf_P(PSTR("Mag%d off: %4.4f, %4.4f, %4.4f\n"),
                        (int)i,
                        offsets.x,
                        offsets.y,
                        offsets.z);
    }

    // motor compensation
    cliSerial->print_P(PSTR("Motor Comp: "));
    if( compass.get_motor_compensation_type() == AP_COMPASS_MOT_COMP_DISABLED ) {
        cliSerial->print_P(PSTR("Off\n"));
    }else{
        if( compass.get_motor_compensation_type() == AP_COMPASS_MOT_COMP_THROTTLE ) {
            cliSerial->print_P(PSTR("Throttle"));
        }
        if( compass.get_motor_compensation_type() == AP_COMPASS_MOT_COMP_CURRENT ) {
            cliSerial->print_P(PSTR("Current"));
        }
        Vector3f motor_compensation;
        for (uint8_t i=0; i<compass.get_count(); i++) {
            motor_compensation = compass.get_motor_compensation(i);
            cliSerial->printf_P(PSTR("\nComMot%d: %4.2f, %4.2f, %4.2f\n"),
                        (int)i,
                        motor_compensation.x,
                        motor_compensation.y,
                        motor_compensation.z);
        }
    }
    print_blanks(1);
}

static void
print_blanks(int16_t num)
{
    while(num > 0) {
        num--;
        cliSerial->println("");
    }
}

static void
print_divider(void)
{
    for (int i = 0; i < 40; i++) {
        cliSerial->print_P(PSTR("-"));
    }
    cliSerial->println();
}

static void print_enabled(bool b)
{
    if(b)
        cliSerial->print_P(PSTR("en"));
    else
        cliSerial->print_P(PSTR("dis"));
    cliSerial->print_P(PSTR("abled\n"));
}

static void report_version()
{
    cliSerial->printf_P(PSTR("FW Ver: %d\n"),(int)g.k_format_version);
    print_divider();
    print_blanks(2);
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/switches.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define CONTROL_SWITCH_DEBOUNCE_TIME_MS  200

static void read_control_switch()
{
    uint32_t tnow_ms = millis();

    // calculate position of flight mode switch
    int8_t switch_position;
    if      (g.rc_5.radio_in < 1231) switch_position = 0;
    else if (g.rc_5.radio_in < 1361) switch_position = 1;
    else if (g.rc_5.radio_in < 1491) switch_position = 2;
    else if (g.rc_5.radio_in < 1621) switch_position = 3;
    else if (g.rc_5.radio_in < 1750) switch_position = 4;
    else switch_position = 5;

    // store time that switch last moved
    if(control_switch_state.last_switch_position != switch_position) {
        control_switch_state.last_edge_time_ms = tnow_ms;
    }

    // debounce switch
    bool control_switch_changed = control_switch_state.debounced_switch_position != switch_position;
    bool sufficient_time_elapsed = tnow_ms - control_switch_state.last_edge_time_ms > CONTROL_SWITCH_DEBOUNCE_TIME_MS;
    bool failsafe_disengaged = !failsafe.radio && failsafe.radio_counter == 0;

    if (control_switch_changed && sufficient_time_elapsed && failsafe_disengaged) {
        // set flight mode and simple mode setting
        if (set_mode(flight_modes[switch_position])) {
            // play a tone
            if (control_switch_state.debounced_switch_position != -1) {
                // alert user to mode change failure (except if autopilot is just starting up)
                if (ap.initialised) {
                    AP_Notify::events.user_mode_change = 1;
                }
            }

            if(!check_if_auxsw_mode_used(AUXSW_SIMPLE_MODE) && !check_if_auxsw_mode_used(AUXSW_SUPERSIMPLE_MODE)) {
                // if none of the Aux Switches are set to Simple or Super Simple Mode then
                // set Simple Mode using stored parameters from EEPROM
                if (BIT_IS_SET(g.super_simple, switch_position)) {
                    set_simple_mode(2);
                }else{
                    set_simple_mode(BIT_IS_SET(g.simple_modes, switch_position));
                }
            }

        } else if (control_switch_state.last_switch_position != -1) {
            // alert user to mode change failure
            AP_Notify::events.user_mode_change_failed = 1;
        }

        // set the debounced switch position
        control_switch_state.debounced_switch_position = switch_position;
    }

    control_switch_state.last_switch_position = switch_position;
}

// check_if_auxsw_mode_used - Check to see if any of the Aux Switches are set to a given mode.
static bool check_if_auxsw_mode_used(uint8_t auxsw_mode_check)
{
    bool ret = g.ch7_option == auxsw_mode_check || g.ch8_option == auxsw_mode_check || g.ch9_option == auxsw_mode_check 
                || g.ch10_option == auxsw_mode_check || g.ch11_option == auxsw_mode_check || g.ch12_option == auxsw_mode_check;

    return ret;
}

// check_duplicate_auxsw - Check to see if any Aux Switch Functions are duplicated
static bool check_duplicate_auxsw(void)
{
    bool ret = ((g.ch7_option != AUXSW_DO_NOTHING) && (g.ch7_option == g.ch8_option ||
                g.ch7_option == g.ch9_option || g.ch7_option == g.ch10_option ||
                g.ch7_option == g.ch11_option || g.ch7_option == g.ch12_option));

    ret = ret || ((g.ch8_option != AUXSW_DO_NOTHING) && (g.ch8_option == g.ch9_option ||
                    g.ch8_option == g.ch10_option || g.ch8_option == g.ch11_option ||
                    g.ch8_option == g.ch12_option));

    ret = ret || ((g.ch9_option != AUXSW_DO_NOTHING) && (g.ch9_option == g.ch10_option ||
                    g.ch9_option == g.ch11_option || g.ch9_option == g.ch12_option));

    ret = ret || ((g.ch10_option != AUXSW_DO_NOTHING) && (g.ch10_option == g.ch11_option ||
                    g.ch10_option == g.ch12_option));

    ret = ret || ((g.ch11_option != AUXSW_DO_NOTHING) && (g.ch11_option == g.ch12_option));

    return ret;
}

static void reset_control_switch()
{
    control_switch_state.last_switch_position = control_switch_state.debounced_switch_position = -1;
    read_control_switch();
}

// read_3pos_switch
static uint8_t read_3pos_switch(int16_t radio_in)
{
    if (radio_in < AUX_SWITCH_PWM_TRIGGER_LOW) return AUX_SWITCH_LOW;      // switch is in low position
    if (radio_in > AUX_SWITCH_PWM_TRIGGER_HIGH) return AUX_SWITCH_HIGH;    // switch is in high position
    return AUX_SWITCH_MIDDLE;                                       // switch is in middle position
}

// read_aux_switches - checks aux switch positions and invokes configured actions
static void read_aux_switches()
{
    uint8_t switch_position;

    // exit immediately during radio failsafe
    if (failsafe.radio || failsafe.radio_counter != 0) {
        return;
    }

    // check if ch7 switch has changed position
    switch_position = read_3pos_switch(g.rc_7.radio_in);
    if (ap.CH7_flag != switch_position) {
        // set the CH7 flag
        ap.CH7_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch7_option, ap.CH7_flag);
    }

    // check if Ch8 switch has changed position
    switch_position = read_3pos_switch(g.rc_8.radio_in);
    if (ap.CH8_flag != switch_position) {
        // set the CH8 flag
        ap.CH8_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch8_option, ap.CH8_flag);
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // check if Ch9 switch has changed position
    switch_position = read_3pos_switch(g.rc_9.radio_in);
    if (ap.CH9_flag != switch_position) {
        // set the CH9 flag
        ap.CH9_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch9_option, ap.CH9_flag);
    }
#endif

    // check if Ch10 switch has changed position
    switch_position = read_3pos_switch(g.rc_10.radio_in);
    if (ap.CH10_flag != switch_position) {
        // set the CH10 flag
        ap.CH10_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch10_option, ap.CH10_flag);
    }

    // check if Ch11 switch has changed position
    switch_position = read_3pos_switch(g.rc_11.radio_in);
    if (ap.CH11_flag != switch_position) {
        // set the CH11 flag
        ap.CH11_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch11_option, ap.CH11_flag);
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    // check if Ch12 switch has changed position
    switch_position = read_3pos_switch(g.rc_12.radio_in);
    if (ap.CH12_flag != switch_position) {
        // set the CH12 flag
        ap.CH12_flag = switch_position;

        // invoke the appropriate function
        do_aux_switch_function(g.ch12_option, ap.CH12_flag);
    }
#endif
}

// init_aux_switches - invoke configured actions at start-up for aux function where it is safe to do so
static void init_aux_switches()
{
    // set the CH7 ~ CH12 flags
    ap.CH7_flag = read_3pos_switch(g.rc_7.radio_in);
    ap.CH8_flag = read_3pos_switch(g.rc_8.radio_in);
    ap.CH10_flag = read_3pos_switch(g.rc_10.radio_in);
    ap.CH11_flag = read_3pos_switch(g.rc_11.radio_in);

    // ch9, ch12 only supported on some boards
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    ap.CH9_flag = read_3pos_switch(g.rc_9.radio_in);
    ap.CH12_flag = read_3pos_switch(g.rc_12.radio_in);
#endif

    // initialise functions assigned to switches
    init_aux_switch_function(g.ch7_option, ap.CH7_flag);
    init_aux_switch_function(g.ch8_option, ap.CH8_flag);
    init_aux_switch_function(g.ch10_option, ap.CH10_flag);
    init_aux_switch_function(g.ch11_option, ap.CH11_flag);

    // ch9, ch12 only supported on some boards
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    init_aux_switch_function(g.ch9_option, ap.CH9_flag);
    init_aux_switch_function(g.ch12_option, ap.CH12_flag);
#endif
}

// init_aux_switch_function - initialize aux functions
static void init_aux_switch_function(int8_t ch_option, uint8_t ch_flag)
{    
    // init channel options
    switch(ch_option) {
        case AUXSW_SIMPLE_MODE:
        case AUXSW_SONAR:
        case AUXSW_FENCE:
        case AUXSW_RESETTOARMEDYAW:
        case AUXSW_SUPERSIMPLE_MODE:
        case AUXSW_ACRO_TRAINER:
        case AUXSW_EPM:
        case AUXSW_SPRAYER:
        case AUXSW_PARACHUTE_ENABLE:
        case AUXSW_PARACHUTE_3POS:      // we trust the vehicle will be disarmed so even if switch is in release position the chute will not release
        case AUXSW_RETRACT_MOUNT:
        case AUXSW_MISSION_RESET:
        case AUXSW_ATTCON_FEEDFWD:
        case AUXSW_ATTCON_ACCEL_LIM:
        case AUXSW_RELAY:
        case AUXSW_LANDING_GEAR:
            do_aux_switch_function(ch_option, ch_flag);
            break;
    }
}

// do_aux_switch_function - implement the function invoked by the ch7 or ch8 switch
static void do_aux_switch_function(int8_t ch_function, uint8_t ch_flag)
{
    int8_t tmp_function = ch_function;

    // multi mode check
    if(ch_function == AUXSW_MULTI_MODE) {
        if (g.rc_6.radio_in < CH6_PWM_TRIGGER_LOW) {
            tmp_function = AUXSW_FLIP;
        }else if (g.rc_6.radio_in > CH6_PWM_TRIGGER_HIGH) {
            tmp_function = AUXSW_SAVE_WP;
        }else{
            tmp_function = AUXSW_RTL;
        }
    }

    switch(tmp_function) {
        case AUXSW_FLIP:
            // flip if switch is on, positive throttle and we're actually flying
            if(ch_flag == AUX_SWITCH_HIGH) {
                set_mode(FLIP);
            }
            break;

        case AUXSW_SIMPLE_MODE:
            // low = simple mode off, middle or high position turns simple mode on
            set_simple_mode(ch_flag == AUX_SWITCH_HIGH || ch_flag == AUX_SWITCH_MIDDLE);
            break;

        case AUXSW_SUPERSIMPLE_MODE:
            // low = simple mode off, middle = simple mode, high = super simple mode
            set_simple_mode(ch_flag);
            break;

        case AUXSW_RTL:
            if (ch_flag == AUX_SWITCH_HIGH) {
                // engage RTL (if not possible we remain in current flight mode)
                set_mode(RTL);
            }else{
                // return to flight mode switch's flight mode if we are currently in RTL
                if (control_mode == RTL) {
                    reset_control_switch();
                }
            }
            break;

        case AUXSW_SAVE_TRIM:
            if ((ch_flag == AUX_SWITCH_HIGH) && (control_mode <= ACRO) && (g.rc_3.control_in == 0)) {
                save_trim();
            }
            break;

        case AUXSW_SAVE_WP:
            // save waypoint when switch is brought high
            if (ch_flag == AUX_SWITCH_HIGH) {

                // do not allow saving new waypoints while we're in auto or disarmed
                if(control_mode == AUTO || !motors.armed()) {
                    return;
                }

				// do not allow saving the first waypoint with zero throttle
				if((mission.num_commands() == 0) && (g.rc_3.control_in == 0)){
					return;
				}

                // create new mission command
                AP_Mission::Mission_Command cmd  = {};

                // if the mission is empty save a takeoff command
                if(mission.num_commands() == 0) {
                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    cmd.id = MAV_CMD_NAV_TAKEOFF;
                    cmd.content.location.options = 0;
                    cmd.p1 = 0;
                    cmd.content.location.lat = 0;
                    cmd.content.location.lng = 0;
                    cmd.content.location.alt = max(current_loc.alt,100);

                    // use the current altitude for the target alt for takeoff.
                    // only altitude will matter to the AP mission script for takeoff.
                    if(mission.add_cmd(cmd)) {
                        // log event
                        Log_Write_Event(DATA_SAVEWP_ADD_WP);
                    }
                }

                // set new waypoint to current location
                cmd.content.location = current_loc;

                // if throttle is above zero, create waypoint command
                if(g.rc_3.control_in > 0) {
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                }else{
					// with zero throttle, create LAND command
                    cmd.id = MAV_CMD_NAV_LAND;
                }

                // save command
                if(mission.add_cmd(cmd)) {
                    // log event
                    Log_Write_Event(DATA_SAVEWP_ADD_WP);
                }
            }
            break;

#if CAMERA == ENABLED
        case AUXSW_CAMERA_TRIGGER:
            if (ch_flag == AUX_SWITCH_HIGH) {
                do_take_picture();
            }
            break;
#endif

        case AUXSW_SONAR:
            // enable or disable the sonar
#if CONFIG_SONAR == ENABLED
            if (ch_flag == AUX_SWITCH_HIGH) {
                sonar_enabled = true;
            }else{
                sonar_enabled = false;
            }
#endif
            break;

#if AC_FENCE == ENABLED
        case AUXSW_FENCE:
            // enable or disable the fence
            if (ch_flag == AUX_SWITCH_HIGH) {
                fence.enable(true);
                Log_Write_Event(DATA_FENCE_ENABLE);
            }else{
                fence.enable(false);
                Log_Write_Event(DATA_FENCE_DISABLE);
            }
            break;
#endif
        // To-Do: add back support for this feature
        //case AUXSW_RESETTOARMEDYAW:
        //    if (ch_flag == AUX_SWITCH_HIGH) {
        //        set_yaw_mode(YAW_RESETTOARMEDYAW);
        //    }else{
        //        set_yaw_mode(YAW_HOLD);
        //    }
        //    break;

        case AUXSW_ACRO_TRAINER:
            switch(ch_flag) {
                case AUX_SWITCH_LOW:
                    g.acro_trainer = ACRO_TRAINER_DISABLED;
                    Log_Write_Event(DATA_ACRO_TRAINER_DISABLED);
                    break;
                case AUX_SWITCH_MIDDLE:
                    g.acro_trainer = ACRO_TRAINER_LEVELING;
                    Log_Write_Event(DATA_ACRO_TRAINER_LEVELING);
                    break;
                case AUX_SWITCH_HIGH:
                    g.acro_trainer = ACRO_TRAINER_LIMITED;
                    Log_Write_Event(DATA_ACRO_TRAINER_LIMITED);
                    break;
            }
            break;
#if EPM_ENABLED == ENABLED
        case AUXSW_EPM:
            switch(ch_flag) {
                case AUX_SWITCH_LOW:
                    epm.release();
                    Log_Write_Event(DATA_EPM_RELEASE);
                    break;
                case AUX_SWITCH_HIGH:
                    epm.grab();
                    Log_Write_Event(DATA_EPM_GRAB);
                    break;
            }
            break;
#endif
#if SPRAYER == ENABLED
        case AUXSW_SPRAYER:
            sprayer.enable(ch_flag == AUX_SWITCH_HIGH);
            // if we are disarmed the pilot must want to test the pump
            sprayer.test_pump((ch_flag == AUX_SWITCH_HIGH) && !motors.armed());
            break;
#endif

        case AUXSW_AUTO:
            if (ch_flag == AUX_SWITCH_HIGH) {
                set_mode(AUTO);
            }else{
                // return to flight mode switch's flight mode if we are currently in AUTO
                if (control_mode == AUTO) {
                    reset_control_switch();
                }
            }
            break;

#if AUTOTUNE_ENABLED == ENABLED
        case AUXSW_AUTOTUNE:
            // turn on auto tuner
            switch(ch_flag) {
                case AUX_SWITCH_LOW:
                case AUX_SWITCH_MIDDLE:
                    // restore flight mode based on flight mode switch position
                    if (control_mode == AUTOTUNE) {
                        reset_control_switch();
                    }
                    break;
                case AUX_SWITCH_HIGH:
                    // start an autotuning session
                    set_mode(AUTOTUNE);
                    break;
            }
            break;
#endif

        case AUXSW_LAND:
            if (ch_flag == AUX_SWITCH_HIGH) {
                set_mode(LAND);
            }else{
                // return to flight mode switch's flight mode if we are currently in LAND
                if (control_mode == LAND) {
                    reset_control_switch();
                }
            }
            break;

#if PARACHUTE == ENABLED
    case AUXSW_PARACHUTE_ENABLE:
        // Parachute enable/disable
        parachute.enabled(ch_flag == AUX_SWITCH_HIGH);
        break;

    case AUXSW_PARACHUTE_RELEASE:
        if (ch_flag == AUX_SWITCH_HIGH) {
            parachute_manual_release();
        }
        break;

    case AUXSW_PARACHUTE_3POS:
        // Parachute disable, enable, release with 3 position switch
        switch (ch_flag) {
            case AUX_SWITCH_LOW:
                parachute.enabled(false);
                Log_Write_Event(DATA_PARACHUTE_DISABLED);
                break;
            case AUX_SWITCH_MIDDLE:
                parachute.enabled(true);
                Log_Write_Event(DATA_PARACHUTE_ENABLED);
                break;
            case AUX_SWITCH_HIGH:
                parachute.enabled(true);
                parachute_manual_release();
                break;
        }
        break;
#endif

    case AUXSW_MISSION_RESET:
        if (ch_flag == AUX_SWITCH_HIGH) {
            mission.reset();
        }
        break;

    case AUXSW_ATTCON_FEEDFWD:
        // enable or disable feed forward
        attitude_control.bf_feedforward(ch_flag == AUX_SWITCH_HIGH);
        break;

    case AUXSW_ATTCON_ACCEL_LIM:
        // enable or disable accel limiting by restoring defaults
        attitude_control.accel_limiting(ch_flag == AUX_SWITCH_HIGH);
        break;
        
#if MOUNT == ENABLE
    case AUXSW_RETRACT_MOUNT:
        switch (ch_flag) {
            case AUX_SWITCH_HIGH:
                camera_mount.set_mode(MAV_MOUNT_MODE_RETRACT);
                break;
            case AUX_SWITCH_LOW:
                camera_mount.set_mode_to_default();
                break;
        }
        break;
#endif

    case AUXSW_RELAY:
        ServoRelayEvents.do_set_relay(0, ch_flag == AUX_SWITCH_HIGH);
        break;

    case AUXSW_LANDING_GEAR:
        switch (ch_flag) {
            case AUX_SWITCH_LOW:
                landinggear.set_cmd_mode(LandingGear_Deploy);
                break;
            case AUX_SWITCH_MIDDLE:
                landinggear.set_cmd_mode(LandingGear_Auto);
                break;
            case AUX_SWITCH_HIGH:
                landinggear.set_cmd_mode(LandingGear_Retract);
                break;
        }
        break;    

    case AUXSW_LOST_COPTER_SOUND:
        switch (ch_flag) {
            case AUX_SWITCH_HIGH:
                AP_Notify::flags.vehicle_lost = true;
                break;
            case AUX_SWITCH_LOW:
                AP_Notify::flags.vehicle_lost = false;
                break;
        }
        break;
    }
}

// save_trim - adds roll and pitch trims from the radio to ahrs
static void save_trim()
{
    // save roll and pitch trim
    float roll_trim = ToRad((float)g.rc_1.control_in/100.0f);
    float pitch_trim = ToRad((float)g.rc_2.control_in/100.0f);
    ahrs.add_trim(roll_trim, pitch_trim);
    Log_Write_Event(DATA_SAVE_TRIM);
    gcs_send_text_P(SEVERITY_HIGH, PSTR("Trim saved"));
}

// auto_trim - slightly adjusts the ahrs.roll_trim and ahrs.pitch_trim towards the current stick positions
// meant to be called continuously while the pilot attempts to keep the copter level
static void auto_trim()
{
    if(auto_trim_counter > 0) {
        auto_trim_counter--;

        // flash the leds
        AP_Notify::flags.save_trim = true;

        // calculate roll trim adjustment
        float roll_trim_adjustment = ToRad((float)g.rc_1.control_in / 4000.0f);

        // calculate pitch trim adjustment
        float pitch_trim_adjustment = ToRad((float)g.rc_2.control_in / 4000.0f);

        // make sure accelerometer values impact attitude quickly
        ahrs.set_fast_gains(true);

        // add trim to ahrs object
        // save to eeprom on last iteration
        ahrs.add_trim(roll_trim_adjustment, pitch_trim_adjustment, (auto_trim_counter == 0));

        // on last iteration restore leds and accel gains to normal
        if(auto_trim_counter == 0) {
            ahrs.set_fast_gains(false);
            AP_Notify::flags.save_trim = false;
        }
    }
}

#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/system.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

#if CLI_ENABLED == ENABLED
// Functions called from the top-level menu
static int8_t   process_logs(uint8_t argc, const Menu::arg *argv);      // in Log.pde
static int8_t   setup_mode(uint8_t argc, const Menu::arg *argv);        // in setup.pde
static int8_t   test_mode(uint8_t argc, const Menu::arg *argv);         // in test.cpp
static int8_t   reboot_board(uint8_t argc, const Menu::arg *argv);

// This is the help function
static int8_t   main_menu_help(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf_P(PSTR("Commands:\n"
                         "  logs\n"
                         "  setup\n"
                         "  test\n"
                         "  reboot\n"
                         "\n"));
    return(0);
}

// Command/function table for the top-level menu.
const struct Menu::command main_menu_commands[] PROGMEM = {
//   command		function called
//   =======        ===============
    {"logs",                process_logs},
    {"setup",               setup_mode},
    {"test",                test_mode},
    {"reboot",              reboot_board},
    {"help",                main_menu_help},
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

static int8_t reboot_board(uint8_t argc, const Menu::arg *argv)
{
    hal.scheduler->reboot(false);
    return 0;
}

// the user wants the CLI. It never exits
static void run_cli(AP_HAL::UARTDriver *port)
{
    cliSerial = port;
    Menu::set_port(port);
    port->set_blocking_writes(true);

    // disable the mavlink delay callback
    hal.scheduler->register_delay_callback(NULL, 5);

    // disable main_loop failsafe
    failsafe_disable();

    // cut the engines
    if(motors.armed()) {
        motors.armed(false);
        motors.output();
    }

    while (1) {
        main_menu.run();
    }
}

#endif // CLI_ENABLED

static void init_ardupilot()
{
    if (!hal.gpio->usb_connected()) {
        // USB is not connected, this means UART0 may be a Xbee, with
        // its darned bricking problem. We can't write to it for at
        // least one second after powering up. Simplest solution for
        // now is to delay for 1 second. Something more elegant may be
        // added later
        delay(1000);
    }

    // initialise serial port
    serial_manager.init_console();

    cliSerial->printf_P(PSTR("\n\nInit " FIRMWARE_STRING
                         "\n\nFree RAM: %u\n"),
                        hal.util->available_memory());

    //
    // Report firmware version code expect on console (check of actual EEPROM format version is done in load_parameters function)
    //
    report_version();

    // load parameters from EEPROM
    load_parameters();

    BoardConfig.init();

    // initialise serial port
    serial_manager.init();

    // init EPM cargo gripper
#if EPM_ENABLED == ENABLED
    epm.init();
#endif

    // initialise notify system
    // disable external leds if epm is enabled because of pin conflict on the APM
    notify.init(true);

    // initialise battery monitor
    battery.init();
    
    rssi_analog_source      = hal.analogin->channel(g.rssi_pin);

    barometer.init();

    // Register the mavlink service callback. This will run
    // anytime there are more than 5ms remaining in a call to
    // hal.scheduler->delay.
    hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.
    ap.usb_connected = true;
    check_usb_mux();

    // init the GCS connected to the console
    gcs[0].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_Console, 0);

    // init telemetry port
    gcs[1].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 0);

#if MAVLINK_COMM_NUM_BUFFERS > 2
    // setup serial port for telem2
    gcs[2].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, 1);
#endif

#if FRSKY_TELEM_ENABLED == ENABLED
    // setup frsky
    frsky_telemetry.init(serial_manager);
#endif

    // identify ourselves correctly with the ground station
    mavlink_system.sysid = g.sysid_this_mav;

#if LOGGING_ENABLED == ENABLED
    DataFlash.Init(log_structure, sizeof(log_structure)/sizeof(log_structure[0]));
    if (!DataFlash.CardInserted()) {
        gcs_send_text_P(SEVERITY_HIGH, PSTR("No dataflash inserted"));
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedErase()) {
        gcs_send_text_P(SEVERITY_HIGH, PSTR("ERASING LOGS"));
        do_erase_logs();
        gcs[0].reset_cli_timeout();
    }
#endif

    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up motors and output to escs

    // initialise which outputs Servo and Relay events can use
    ServoRelayEvents.set_channel_mask(~motors.get_motor_mask());

    relay.init();

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

    // Do GPS init
    gps.init(&DataFlash, serial_manager);

    if(g.compass_enabled)
        init_compass();

#if OPTFLOW == ENABLED
    // make optflow available to AHRS
    ahrs.set_optflow(&optflow);
#endif

    // initialise attitude and position controllers
    attitude_control.set_dt(MAIN_LOOP_SECONDS);
    pos_control.set_dt(MAIN_LOOP_SECONDS);

    // init the optical flow sensor
    init_optflow();

#if MOUNT == ENABLED
    // initialise camera mount
    camera_mount.init(serial_manager);
#endif

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

#if CLI_ENABLED == ENABLED
    if (g.cli_enabled) {
        const prog_char_t *msg = PSTR("\nPress ENTER 3 times to start interactive setup\n");
        cliSerial->println_P(msg);
        if (gcs[1].initialised && (gcs[1].get_uart() != NULL)) {
            gcs[1].get_uart()->println_P(msg);
        }
        if (num_gcs > 2 && gcs[2].initialised && (gcs[2].get_uart() != NULL)) {
            gcs[2].get_uart()->println_P(msg);
        }
    }
#endif // CLI_ENABLED

#if HIL_MODE != HIL_MODE_DISABLED
    while (barometer.get_last_update() == 0) {
        // the barometer begins updating when we get the first
        // HIL_STATE message
        gcs_send_text_P(SEVERITY_LOW, PSTR("Waiting for first HIL_STATE message"));
        delay(1000);
    }

    // set INS to HIL mode
    ins.set_hil_mode();
#endif

    // read Baro pressure at ground
    //-----------------------------
    init_barometer(true);

    // initialise sonar
#if CONFIG_SONAR == ENABLED
    init_sonar();
#endif

    // initialise mission library
    mission.init();

    // initialise the flight mode and aux switch
    // ---------------------------
    reset_control_switch();
    init_aux_switches();

#if FRAME_CONFIG == HELI_FRAME
    // trad heli specific initialisation
    heli_init();
#endif

    startup_ground(true);

#if LOGGING_ENABLED == ENABLED
    Log_Write_Startup();
#endif

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    serial_manager.set_blocking_writes_all(false);

    // enable CPU failsafe
    failsafe_enable();

    cliSerial->print_P(PSTR("\nReady to FLY "));

    // flag that initialisation has completed
    ap.initialised = true;
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
static void startup_ground(bool force_gyro_cal)
{
    gcs_send_text_P(SEVERITY_LOW,PSTR("GROUND START"));

    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);

    // Warm up and read Gyro offsets
    // -----------------------------
    ins.init(force_gyro_cal?AP_InertialSensor::COLD_START:AP_InertialSensor::WARM_START,
             ins_sample_rate);
 #if CLI_ENABLED == ENABLED
    report_ins();
 #endif

    // reset ahrs gyro bias
    if (force_gyro_cal) {
        ahrs.reset_gyro_drift();
    }

    // setup fast AHRS gains to get right attitude
    ahrs.set_fast_gains(true);

    // set landed flag
    set_land_complete(true);
    set_land_complete_maybe(true);
}

// position_ok - returns true if the horizontal absolute position is ok and home position is set
static bool position_ok()
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // return false if ekf failsafe has triggered
    if (failsafe.ekf) {
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal position
    if (!motors.armed()) {
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs));
    } else {
        // once armed we require a good absolute position and EKF must not be in const_pos_mode
        return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
    }
}

// optflow_position_ok - returns true if optical flow based position estimate is ok
static bool optflow_position_ok()
{
#if OPTFLOW != ENABLED
    return false;
#else
    // return immediately if optflow is not enabled or EKF not used
    if (!optflow.enabled() || !ahrs.have_inertial_nav()) {
        return false;
    }

    // get filter status from EKF
    nav_filter_status filt_status = inertial_nav.get_filter_status();
    return (filt_status.flags.horiz_pos_rel || filt_status.flags.pred_horiz_pos_rel);
#endif
}

// update_auto_armed - update status of auto_armed flag
static void update_auto_armed()
{
    // disarm checks
    if(ap.auto_armed){
        // if motors are disarmed, auto_armed should also be false
        if(!motors.armed()) {
            set_auto_armed(false);
            return;
        }
        // if in stabilize or acro flight mode and throttle is zero, auto-armed should become false
        if(mode_has_manual_throttle(control_mode) && ap.throttle_zero && !failsafe.radio) {
            set_auto_armed(false);
        }
    }else{
        // arm checks
        
#if FRAME_CONFIG == HELI_FRAME
        // for tradheli if motors are armed and throttle is above zero and the motor is started, auto_armed should be true
        if(motors.armed() && !ap.throttle_zero && motors.motor_runup_complete()) {
            set_auto_armed(true);
        }
#else
        // if motors are armed and throttle is above zero auto_armed should be true
        if(motors.armed() && !ap.throttle_zero) {
            set_auto_armed(true);
        }
#endif // HELI_FRAME
    }
}

static void check_usb_mux(void)
{
    bool usb_check = hal.gpio->usb_connected();
    if (usb_check == ap.usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    ap.usb_connected = usb_check;
}

// frsky_telemetry_send - sends telemetry data using frsky telemetry
//  should be called at 5Hz by scheduler
#if FRSKY_TELEM_ENABLED == ENABLED
static void frsky_telemetry_send(void)
{
    frsky_telemetry.send_frames((uint8_t)control_mode);
}
#endif

/*
  should we log a message type now?
 */
static bool should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    if (!(mask & g.log_bitmask) || in_mavlink_delay) {
        return false;
    }
    bool ret = motors.armed() || (g.log_bitmask & MASK_LOG_WHEN_DISARMED) != 0;
    if (ret && !DataFlash.logging_started() && !in_log_download) {
        // we have to set in_mavlink_delay to prevent logging while
        // writing headers
        start_logging();
    }
    return ret;
#else
    return false;
#endif
}
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/test.pde"
// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if CLI_ENABLED == ENABLED

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
#if HIL_MODE == HIL_MODE_DISABLED
static int8_t   test_baro(uint8_t argc,                 const Menu::arg *argv);
#endif
static int8_t   test_compass(uint8_t argc,              const Menu::arg *argv);
static int8_t   test_ins(uint8_t argc,                  const Menu::arg *argv);
static int8_t   test_optflow(uint8_t argc,              const Menu::arg *argv);
static int8_t   test_relay(uint8_t argc,                const Menu::arg *argv);
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
static int8_t   test_shell(uint8_t argc,                const Menu::arg *argv);
#endif
#if HIL_MODE == HIL_MODE_DISABLED
static int8_t   test_sonar(uint8_t argc,                const Menu::arg *argv);
#endif

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
const struct Menu::command test_menu_commands[] PROGMEM = {
#if HIL_MODE == HIL_MODE_DISABLED
    {"baro",                test_baro},
#endif
    {"compass",             test_compass},
    {"ins",                 test_ins},
    {"optflow",             test_optflow},
    {"relay",               test_relay},
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    {"shell", 				test_shell},
#endif
#if HIL_MODE == HIL_MODE_DISABLED
    {"rangefinder",         test_sonar},
#endif
};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

static int8_t
test_mode(uint8_t argc, const Menu::arg *argv)
{
    test_menu.run();
    return 0;
}

#if HIL_MODE == HIL_MODE_DISABLED
static int8_t
test_baro(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    init_barometer(true);

    while(1) {
        delay(100);
        read_barometer();

        if (!barometer.healthy()) {
            cliSerial->println_P(PSTR("not healthy"));
        } else {
            cliSerial->printf_P(PSTR("Alt: %0.2fm, Raw: %f Temperature: %.1f\n"),
                                baro_alt / 100.0,
                                barometer.get_pressure(), 
                                barometer.get_temperature());
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
    return 0;
}
#endif

static int8_t
test_compass(uint8_t argc, const Menu::arg *argv)
{
    uint8_t delta_ms_fast_loop;
    uint8_t medium_loopCounter = 0;

    if (!g.compass_enabled) {
        cliSerial->printf_P(PSTR("Compass: "));
        print_enabled(false);
        return (0);
    }

    if (!compass.init()) {
        cliSerial->println_P(PSTR("Compass initialisation failed!"));
        return 0;
    }

    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_compass(&compass);
#if OPTFLOW == ENABLED
    ahrs.set_optflow(&optflow);
#endif
    report_compass();

    // we need the AHRS initialised for this test
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);
    ahrs.reset();
    int16_t counter = 0;
    float heading = 0;

    print_hit_enter();

    while(1) {
        delay(20);
        if (millis() - fast_loopTimer > 19) {
            delta_ms_fast_loop      = millis() - fast_loopTimer;
            G_Dt                    = (float)delta_ms_fast_loop / 1000.0f;                       // used by DCM integrator
            fast_loopTimer          = millis();

            // INS
            // ---
            ahrs.update();

            medium_loopCounter++;
            if(medium_loopCounter == 5) {
                if (compass.read()) {
                    // Calculate heading
                    const Matrix3f &m = ahrs.get_dcm_matrix();
                    heading = compass.calculate_heading(m);
                    compass.learn_offsets();
                }
                medium_loopCounter = 0;
            }

            counter++;
            if (counter>20) {
                if (compass.healthy()) {
                    const Vector3f &mag_ofs = compass.get_offsets();
                    const Vector3f &mag = compass.get_field();
                    cliSerial->printf_P(PSTR("Heading: %ld, XYZ: %.0f, %.0f, %.0f,\tXYZoff: %6.2f, %6.2f, %6.2f\n"),
                                        (wrap_360_cd(ToDeg(heading) * 100)) /100,
                                        mag.x,
                                        mag.y,
                                        mag.z,
                                        mag_ofs.x,
                                        mag_ofs.y,
                                        mag_ofs.z);
                } else {
                    cliSerial->println_P(PSTR("compass not healthy"));
                }
                counter=0;
            }
        }
        if (cliSerial->available() > 0) {
            break;
        }
    }

    // save offsets. This allows you to get sane offset values using
    // the CLI before you go flying.
    cliSerial->println_P(PSTR("saving offsets"));
    compass.save_offsets();
    return (0);
}

static int8_t
test_ins(uint8_t argc, const Menu::arg *argv)
{
    Vector3f gyro, accel;
    print_hit_enter();
    cliSerial->printf_P(PSTR("INS\n"));
    delay(1000);

    ahrs.init();
    ins.init(AP_InertialSensor::COLD_START, 
             ins_sample_rate);
    cliSerial->printf_P(PSTR("...done\n"));

    delay(50);

    while(1) {
        ins.update();
        gyro = ins.get_gyro();
        accel = ins.get_accel();

        float test = accel.length() / GRAVITY_MSS;

        cliSerial->printf_P(PSTR("a %7.4f %7.4f %7.4f g %7.4f %7.4f %7.4f t %7.4f \n"),
            accel.x, accel.y, accel.z,
            gyro.x, gyro.y, gyro.z,
            test);

        delay(40);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

static int8_t
test_optflow(uint8_t argc, const Menu::arg *argv)
{
#if OPTFLOW == ENABLED
    if(optflow.enabled()) {
        cliSerial->printf_P(PSTR("dev id: %d\t"),(int)optflow.device_id());
        print_hit_enter();

        while(1) {
            delay(200);
            optflow.update();
            const Vector2f& flowRate = optflow.flowRate();
            cliSerial->printf_P(PSTR("flowX : %7.4f\t flowY : %7.4f\t flow qual : %d\n"),
                            flowRate.x,
                            flowRate.y,
                            (int)optflow.quality());

            if(cliSerial->available() > 0) {
                return (0);
            }
        }
    } else {
        cliSerial->printf_P(PSTR("OptFlow: "));
        print_enabled(false);
    }
    return (0);
#else
    return (0);
#endif      // OPTFLOW == ENABLED
}

static int8_t test_relay(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    delay(1000);

    while(1) {
        cliSerial->printf_P(PSTR("Relay on\n"));
        relay.on(0);
        delay(3000);
        if(cliSerial->available() > 0) {
            return (0);
        }

        cliSerial->printf_P(PSTR("Relay off\n"));
        relay.off(0);
        delay(3000);
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
/*
 *  run a debug shell
 */
static int8_t
test_shell(uint8_t argc, const Menu::arg *argv)
{
    hal.util->run_debug_shell(cliSerial);
    return 0;
}
#endif

#if HIL_MODE == HIL_MODE_DISABLED
/*
 *  test the rangefinders
 */
static int8_t
test_sonar(uint8_t argc, const Menu::arg *argv)
{
#if CONFIG_SONAR == ENABLED
	sonar.init();

    cliSerial->printf_P(PSTR("RangeFinder: %d devices detected\n"), sonar.num_sensors());

    print_hit_enter();
    while(1) {
        delay(100);
        sonar.update();

        cliSerial->printf_P(PSTR("Primary: status %d distance_cm %d \n"), (int)sonar.status(), sonar.distance_cm());
        cliSerial->printf_P(PSTR("All: device_0 type %d status %d distance_cm %d, device_1 type %d status %d distance_cm %d\n"),
        (int)sonar._type[0], (int)sonar.status(0), sonar.distance_cm(0), (int)sonar._type[1], (int)sonar.status(1), sonar.distance_cm(1));

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
#endif
    return (0);
}
#endif

static void print_hit_enter()
{
    cliSerial->printf_P(PSTR("Hit Enter to exit.\n\n"));
}

#endif // CLI_ENABLED
#line 1 "/Users/yndal/Desktop/WorkingDroneCode/Drone-old_global-param/ardupilot/ArduCopter/tuning.pde"
/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * tuning.pde - function to update various parameters in flight using the ch6 tuning knob
 *      This should not be confused with the AutoTune feature which can bve found in control_autotune.pde
 */

// tuning - updates parameters based on the ch6 tuning knob's position
//  should be called at 3.3hz
static void tuning() {

    // exit immediately if not tuning of when radio failsafe is invoked so tuning values are not set to zero
    if ((g.radio_tuning <= 0) || failsafe.radio || failsafe.radio_counter != 0) {
        return;
    }

    float tuning_value = (float)g.rc_6.control_in / 1000.0f;
    g.rc_6.set_range(g.radio_tuning_low,g.radio_tuning_high);

    switch(g.radio_tuning) {

    // Roll, Pitch tuning
    case CH6_STABILIZE_ROLL_PITCH_KP:
        g.p_stabilize_roll.kP(tuning_value);
        g.p_stabilize_pitch.kP(tuning_value);
        break;

    case CH6_RATE_ROLL_PITCH_KP:
        g.pid_rate_roll.kP(tuning_value);
        g.pid_rate_pitch.kP(tuning_value);
        break;

    case CH6_RATE_ROLL_PITCH_KI:
        g.pid_rate_roll.kI(tuning_value);
        g.pid_rate_pitch.kI(tuning_value);
        break;

    case CH6_RATE_ROLL_PITCH_KD:
        g.pid_rate_roll.kD(tuning_value);
        g.pid_rate_pitch.kD(tuning_value);
        break;

    // Yaw tuning
    case CH6_STABILIZE_YAW_KP:
        g.p_stabilize_yaw.kP(tuning_value);
        break;

    case CH6_YAW_RATE_KP:
        g.pid_rate_yaw.kP(tuning_value);
        break;

    case CH6_YAW_RATE_KD:
        g.pid_rate_yaw.kD(tuning_value);
        break;

    // Altitude and throttle tuning
    case CH6_ALTITUDE_HOLD_KP:
        g.p_alt_hold.kP(tuning_value);
        break;

    case CH6_THROTTLE_RATE_KP:
        g.p_vel_z.kP(tuning_value);
        break;

    case CH6_ACCEL_Z_KP:
        g.pid_accel_z.kP(tuning_value);
        break;

    case CH6_ACCEL_Z_KI:
        g.pid_accel_z.kI(tuning_value);
        break;

    case CH6_ACCEL_Z_KD:
        g.pid_accel_z.kD(tuning_value);
        break;

    // Loiter and navigation tuning
    case CH6_LOITER_POSITION_KP:
        g.p_pos_xy.kP(tuning_value);
        break;

    case CH6_VEL_XY_KP:
        g.pi_vel_xy.kP(tuning_value);
        break;

    case CH6_VEL_XY_KI:
        g.pi_vel_xy.kI(tuning_value);
        break;

    case CH6_WP_SPEED:
        // set waypoint navigation horizontal speed to 0 ~ 1000 cm/s
        wp_nav.set_speed_xy(g.rc_6.control_in);
        break;

    // Acro roll pitch gain
    case CH6_ACRO_RP_KP:
        g.acro_rp_p = tuning_value;
        break;

    // Acro yaw gain
    case CH6_ACRO_YAW_KP:
        g.acro_yaw_p = tuning_value;
        break;

#if FRAME_CONFIG == HELI_FRAME
    case CH6_HELI_EXTERNAL_GYRO:
        motors.ext_gyro_gain(g.rc_6.control_in);
        break;

    case CH6_RATE_PITCH_FF:
        g.pid_rate_pitch.ff(tuning_value);
        break;

    case CH6_RATE_ROLL_FF:
        g.pid_rate_roll.ff(tuning_value);
        break;

    case CH6_RATE_YAW_FF:
        g.pid_rate_yaw.ff(tuning_value);
        break;
#endif

    case CH6_AHRS_YAW_KP:
        ahrs._kp_yaw.set(tuning_value);
        break;

    case CH6_AHRS_KP:
        ahrs._kp.set(tuning_value);
        break;

    case CH6_DECLINATION:
        // set declination to +-20degrees
        compass.set_declination(ToRad((2.0f * g.rc_6.control_in - g.radio_tuning_high)/100.0f), false);     // 2nd parameter is false because we do not want to save to eeprom because this would have a performance impact
        break;

    case CH6_CIRCLE_RATE:
        // set circle rate
        circle_nav.set_rate(g.rc_6.control_in/25-20);   // allow approximately 45 degree turn rate in either direction
        break;

    case CH6_SONAR_GAIN:
        // set sonar gain
        g.sonar_gain.set(tuning_value);
        break;

#if 0
        // disabled for now - we need accessor functions
    case CH6_EKF_VERTICAL_POS:
        // EKF's baro vs accel (higher rely on accels more, baro impact is reduced)
        ahrs.get_NavEKF()._gpsVertPosNoise = tuning_value;
        break;

    case CH6_EKF_HORIZONTAL_POS:
        // EKF's gps vs accel (higher rely on accels more, gps impact is reduced)
        ahrs.get_NavEKF()._gpsHorizPosNoise = tuning_value;
        break;

    case CH6_EKF_ACCEL_NOISE:
        // EKF's accel noise (lower means trust accels more, gps & baro less)
        ahrs.get_NavEKF()._accNoise = tuning_value;
        break;
#endif

    case CH6_RC_FEEL_RP:
        // roll-pitch input smoothing
        g.rc_feel_rp = g.rc_6.control_in / 10;
        break;

    case CH6_RATE_PITCH_KP:
        g.pid_rate_pitch.kP(tuning_value);
        break;

    case CH6_RATE_PITCH_KI:
        g.pid_rate_pitch.kI(tuning_value);
        break;

    case CH6_RATE_PITCH_KD:
        g.pid_rate_pitch.kD(tuning_value);
        break;

    case CH6_RATE_ROLL_KP:
        g.pid_rate_roll.kP(tuning_value);
        break;

    case CH6_RATE_ROLL_KI:
        g.pid_rate_roll.kI(tuning_value);
        break;

    case CH6_RATE_ROLL_KD:
        g.pid_rate_roll.kD(tuning_value);
        break;

    case CH6_RATE_MOT_YAW_HEADROOM:
        motors.set_yaw_headroom(tuning_value*1000);
        break;

     case CH6_RATE_YAW_FILT:
         g.pid_rate_yaw.filt_hz(tuning_value);
         break;
    }
}
