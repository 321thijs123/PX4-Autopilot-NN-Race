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
 * @file gate_publisher.h
 * Gate Publisher module, publishes position of gates for autonomous drone racing.
 *
 * @author Thijs Hof <thijs.hof@home.nl>
 * @author Tom Fransen <twjfransen@gmail.com>
 */
#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

// Subscriptions
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>

// Publications
#include <uORB/topics/gates.h>

using namespace time_literals; // For the 1_s in the subscription interval
class GatePublisher : public ModuleBase<GatePublisher>, public ModuleParams,
	public px4::WorkItem
{
public:

	GatePublisher();
	~GatePublisher() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	int print_status() override;

	bool init();

private:
	struct Pos {
		float x, y, z = 0;

		// Addition
		Pos operator+(const Pos& other) const {
			return {x + other.x, y + other.y, z + other.z};
		}

		// Subtraction
		Pos operator-(const Pos& other) const {
			return {x - other.x, y - other.y, z - other.z};
		}

		// Scalar multiplication
		Pos operator*(float scalar) const {
			return {x * scalar, y * scalar, z * scalar};
		}

		// Scalar division
		Pos operator/(float scalar) const {
			return {x / scalar, y / scalar, z / scalar};
		}
	};

	Pos prevPosition;
	int next_gate{0};
	gates_s gates;

	const int N_GATES = 4;

	void Run() override;
	void set_gates();
	bool get_gate_passing(Pos gatePos, float gateYaw, Pos curDronePos, Pos prevDronePos, float width, float height);

	// Publications
	uORB::Publication<gates_s> _gates_pub{ORB_ID(gates)};

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _position_sub{this, ORB_ID(vehicle_local_position)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::GATE1_X>) _param_gate1_x,
		(ParamFloat<px4::params::GATE1_Y>) _param_gate1_y,
		(ParamFloat<px4::params::GATE1_Z>) _param_gate1_z,
		(ParamFloat<px4::params::GATE1_YAW>) _param_gate1_yaw,
		(ParamFloat<px4::params::GATE2_X>) _param_gate2_x,
		(ParamFloat<px4::params::GATE2_Y>) _param_gate2_y,
		(ParamFloat<px4::params::GATE2_Z>) _param_gate2_z,
		(ParamFloat<px4::params::GATE2_YAW>) _param_gate2_yaw,
		(ParamFloat<px4::params::GATE3_X>) _param_gate3_x,
		(ParamFloat<px4::params::GATE3_Y>) _param_gate3_y,
		(ParamFloat<px4::params::GATE3_Z>) _param_gate3_z,
		(ParamFloat<px4::params::GATE3_YAW>) _param_gate3_yaw,
		(ParamFloat<px4::params::GATE4_X>) _param_gate4_x,
		(ParamFloat<px4::params::GATE4_Y>) _param_gate4_y,
		(ParamFloat<px4::params::GATE4_Z>) _param_gate4_z,
		(ParamFloat<px4::params::GATE4_YAW>) _param_gate4_yaw,
		(ParamFloat<px4::params::MC_NN_Z_OFFSET>) _param_z_offset
	)
};
