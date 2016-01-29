/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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

/*
 * @file circuit_breaker.c
 *
 * Circuit breaker parameters.
 * Analog to real aviation circuit breakers these parameters
 * allow to disable subsystems. They are not supported as standard
 * operation procedure and are only provided for development purposes.
 * To ensure they are not activated accidentally, the associated
 * parameter needs to set to the key (magic).
 */

#include <px4.h>
#include <systemlib/circuit_breaker_params.h>

/**
 * Circuit breaker for power supply check
 *
 * Setting this parameter to 894281 will disable the power valid
 * checks in the commander.
 * WARNING: ENABLING THIS CIRCUIT BREAKER IS AT OWN RISK
 *
 * @min 0
 * @max 894281
 * @group Circuit Breaker
 */
PX4_PARAM_DEFINE_INT32(CBRK_SUPPLY_CHK);

/**
 * Circuit breaker for rate controller output
 *
 * Setting this parameter to 140253 will disable the rate
 * controller uORB publication.
 * WARNING: ENABLING THIS CIRCUIT BREAKER IS AT OWN RISK
 *
 * @min 0
 * @max 140253
 * @group Circuit Breaker
 */
PX4_PARAM_DEFINE_INT32(CBRK_RATE_CTRL);

/**
 * Circuit breaker for IO safety
 *
 * Setting this parameter to 894281 will disable IO safety.
 * WARNING: ENABLING THIS CIRCUIT BREAKER IS AT OWN RISK
 *
 * @min 0
 * @max 22027
 * @group Circuit Breaker
 */
PX4_PARAM_DEFINE_INT32(CBRK_IO_SAFETY);

/**
 * Circuit breaker for airspeed sensor
 *
 * Setting this parameter to 162128 will disable the check for an airspeed sensor.
 * WARNING: ENABLING THIS CIRCUIT BREAKER IS AT OWN RISK
 *
 * @min 0
 * @max 162128
 * @group Circuit Breaker
 */
PX4_PARAM_DEFINE_INT32(CBRK_AIRSPD_CHK);

/**
 * Circuit breaker for flight termination
 *
 * Setting this parameter to 121212 will disable the flight termination action.
 * --> The IO driver will not do flight termination if requested by the FMU
 * WARNING: ENABLING THIS CIRCUIT BREAKER IS AT OWN RISK
 *
 * @min 0
 * @max 121212
 * @group Circuit Breaker
 */
PX4_PARAM_DEFINE_INT32(CBRK_FLIGHTTERM);

/**
 * Circuit breaker for engine failure detection
 *
 * Setting this parameter to 284953 will disable the engine failure detection.
 * If the aircraft is in engine failure mode the enine failure flag will be
 * set to healthy
 * WARNING: ENABLING THIS CIRCUIT BREAKER IS AT OWN RISK
 *
 * @min 0
 * @max 284953
 * @group Circuit Breaker
 */
PX4_PARAM_DEFINE_INT32(CBRK_ENGINEFAIL);
