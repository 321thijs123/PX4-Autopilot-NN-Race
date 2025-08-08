/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file gate_publisher.cu
 * Parameters for the Gate Publisher module
 *
 * @author Thijs Hof <thijs.hof@home.nl>
 */

/**
 * X Position of gate 1 in meters
 *
 * @group Gate Publisher
 */
PARAM_DEFINE_FLOAT(GATE1_X, 5.0f);


/**
 * Y Position of gate 1 in meters
 *
 * @group Gate Publisher
 */
PARAM_DEFINE_FLOAT(GATE1_Y, 0.0f);


/**
 * Z Position of gate 1 in meters
 *
 * @group Gate Publisher
 */
PARAM_DEFINE_FLOAT(GATE1_Z, 2.0f);


/**
 * Yaw of gate 1 in radians
 *
 * @group Gate Publisher
 */
PARAM_DEFINE_FLOAT(GATE1_YAW, 1.57f);


/**
 * X Position of gate 2 in meters
 *
 * @group Gate Publisher
 */
PARAM_DEFINE_FLOAT(GATE2_X, 0.0f);


/**
 * Y Position of gate 2 in meters
 *
 * @group Gate Publisher
 */
PARAM_DEFINE_FLOAT(GATE2_Y, 5.0f);


/**
 * Z Position of gate 2 in meters
 *
 * @group Gate Publisher
 */
PARAM_DEFINE_FLOAT(GATE2_Z, 2.0f);


/**
 * Yaw of gate 2 in radians
 *
 * @group Gate Publisher
 */
PARAM_DEFINE_FLOAT(GATE2_YAW, 3.14f);


/**
 * X Position of gate 3 in meters
 *
 * @group Gate Publisher
 */
PARAM_DEFINE_FLOAT(GATE3_X, -5.0f);


/**
 * Y Position of gate 3 in meters
 *
 * @group Gate Publisher
 */
PARAM_DEFINE_FLOAT(GATE3_Y, 0.0f);


/**
 * Z Position of gate 3 in meters
 *
 * @group Gate Publisher
 */
PARAM_DEFINE_FLOAT(GATE3_Z, 2.0f);


/**
 * Yaw of gate 3 in radians
 *
 * @group Gate Publisher
 */
PARAM_DEFINE_FLOAT(GATE3_YAW, -1.57f);


/**
 * X Position of gate 4 in meters
 *
 * @group Gate Publisher
 */
PARAM_DEFINE_FLOAT(GATE4_X, 0.0f);


/**
 * Y Position of gate 4 in meters
 *
 * @group Gate Publisher
 */
PARAM_DEFINE_FLOAT(GATE4_Y, -5.0f);


/**
 * Z Position of gate 4 in meters
 *
 * @group Gate Publisher
 */
PARAM_DEFINE_FLOAT(GATE4_Z, 2.0f);


/**
 * Yaw of gate 4 in radians
 *
 * @group Gate Publisher
 */
PARAM_DEFINE_FLOAT(GATE4_YAW, 0.0f);
