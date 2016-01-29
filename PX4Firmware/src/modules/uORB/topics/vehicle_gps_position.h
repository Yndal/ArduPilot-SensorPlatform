/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Thomas Gubler <thomasgubler@student.ethz.ch>
 *           @author Julian Oes <joes@student.ethz.ch>
 *           @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file vehicle_gps_position.h
 * Definition of the GPS WGS84 uORB topic.
 */

#ifndef TOPIC_VEHICLE_GPS_H_
#define TOPIC_VEHICLE_GPS_H_

#include <stdint.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * GPS position in WGS84 coordinates.
 */
struct vehicle_gps_position_s {
	uint64_t timestamp_position;			/**< Timestamp for position information */
	int32_t lat;					/**< Latitude in 1E-7 degrees */
	int32_t lon;					/**< Longitude in 1E-7 degrees */
	int32_t alt;					/**< Altitude in 1E-3 meters (millimeters) above MSL  */

	uint64_t timestamp_variance;
	float s_variance_m_s;				/**< speed accuracy estimate m/s */
	float c_variance_rad;				/**< course accuracy estimate rad */
	uint8_t fix_type; 				/**< 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.   */

	float eph;					/**< GPS HDOP horizontal dilution of position in m */
	float epv;					/**< GPS VDOP horizontal dilution of position in m */

	unsigned noise_per_ms;				/**< */
	unsigned jamming_indicator;			/**< */

	uint64_t timestamp_velocity;			/**< Timestamp for velocity informations */
	float vel_m_s;					/**< GPS ground speed (m/s) */
	float vel_n_m_s;				/**< GPS ground speed in m/s */
	float vel_e_m_s;				/**< GPS ground speed in m/s */
	float vel_d_m_s;				/**< GPS ground speed in m/s */
	float cog_rad;					/**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
	bool vel_ned_valid;				/**< Flag to indicate if NED speed is valid */

	uint64_t timestamp_time;			/**< Timestamp for time information */
	uint64_t time_utc_usec;				/**< Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0 */

	uint8_t satellites_used;			/**< Number of satellites used */
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_gps_position);

#endif
