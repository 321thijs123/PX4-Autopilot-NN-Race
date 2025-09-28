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
 * @file mc_nn_control_params.c
 * Parameters for the Multicopter Neural Network Control module
 *
 * @author Sindre Meyer Hegre <sindre.hegre@gmail.com>
 */

/**
 * If true the neural network control is automatically started on boot.
 *
 * @boolean
 * @group Neural Control
 */
PARAM_DEFINE_INT32(MC_NN_EN, 1);

/**
 * The maximum RPM of the motors. Used to normalize the output of the neural network.
 *
 * @min 0
 * @max 80000
 * @group Neural Control
 */
PARAM_DEFINE_INT32(MC_NN_MAX_RPM, 32945);

/**
 * The minimum RPM of the motors. Used to normalize the output of the neural network.
 *
 * @min 0
 * @max 80000
 * @group Neural Control
 */
PARAM_DEFINE_INT32(MC_NN_MIN_RPM, 0);

/**
 * Thrust coefficient of the motors. Used to normalize the output of the neural network. Divided by 100 000
 *
 * @min 0.0
 * @max 5.0
 * @group Neural Control
 */
PARAM_DEFINE_FLOAT(MC_NN_THRST_COEF, 4.772f);

/**
 * Non-linearity of thrust to throttle curve
 *
 * @min 0.0
 * @max 1.0
 * @group Neural Control
 */
PARAM_DEFINE_FLOAT(MC_NN_NON_LIN, 0.7f);

/**
 * Enable or disable setting the trajectory setpoint with manual control.
 *
 * @boolean
 * @reboot_required true
 * @group Neural Control
 */
PARAM_DEFINE_INT32(MC_NN_MANL_CTRL, 1);

/**
 * The outputs of the neural network are multiplied by this value.
 *
 * @min 0.01
 * @group Neural Control
 */
PARAM_DEFINE_FLOAT(MC_NN_OUT_SCALE, 8.0);

/**
 * Offset for altitude related observations.
 *
 * @group Neural Control
 */
PARAM_DEFINE_FLOAT(MC_NN_Z_OFFSET, 40.0);

/**
 * Minimum X Position for Neural Control expressed in local ENU frame.
 *
 * @group Neural Control
 */
PARAM_DEFINE_FLOAT(MC_NN_MIN_X, -20.0);

/**
 * Maximum X Position for Neural Control expressed in local ENU frame.
 *
 * @group Neural Control
 */
PARAM_DEFINE_FLOAT(MC_NN_MAX_X, 20.0);


/**
 * Minimum Y Position for Neural Control expressed in local ENU frame.
 *
 * @group Neural Control
 */
PARAM_DEFINE_FLOAT(MC_NN_MIN_Y, -20.0);

/**
 * Maximum Y Position for Neural Control expressed in local ENU frame.
 *
 * @group Neural Control
 */
PARAM_DEFINE_FLOAT(MC_NN_MAX_Y, 20.0);


/**
 * Minimum Z Position for Neural Control expressed in local ENU frame.
 *
 * @group Neural Control
 */
PARAM_DEFINE_FLOAT(MC_NN_MIN_Z, 39.0);

/**
 * Maximum Z Position for Neural Control expressed in local ENU frame.
 *
 * @group Neural Control
 */
PARAM_DEFINE_FLOAT(MC_NN_MAX_Z, 120.0);
