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
* @file gate_publisher.cpp
* Gate Publisher module, publishes position of gates for autonomous drone racing.
*
* @author Thijs Hof <thijs.hof@home.nl>
* @author Tom Fransen <twjfransen@gmail.com>
*/

#include "gate_publisher.hpp"

GatePublisher::GatePublisher() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	set_gates();
}

GatePublisher::~GatePublisher()
{

}


bool GatePublisher::init()
{
	if (!_position_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int GatePublisher::task_spawn(int argc, char *argv[])
{
	// This function loads the model, sets up the interpreter, allocates memory for the model's tensors, and prepares the input data.
	GatePublisher *instance = new GatePublisher();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;

		} else {
			PX4_ERR("init failed");
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

void GatePublisher::set_gates() {
	const int index_gate1 = (0 - next_gate + N_GATES) % N_GATES;
	const int index_gate2 = (1 - next_gate + N_GATES) % N_GATES;
	const int index_gate3 = (2 - next_gate + N_GATES) % N_GATES;
	const int index_gate4 = (3 - next_gate + N_GATES) % N_GATES;

	gates.timestamp = hrt_absolute_time();

	gates.x[index_gate1] = _param_gate1_x.get();
	gates.x[index_gate2] = _param_gate2_x.get();
	gates.x[index_gate3] = _param_gate3_x.get();
	gates.x[index_gate4] = _param_gate4_x.get();

	gates.y[index_gate1] = _param_gate1_y.get();
	gates.y[index_gate2] = _param_gate2_y.get();
	gates.y[index_gate3] = _param_gate3_y.get();
	gates.y[index_gate4] = _param_gate4_y.get();

	gates.z[index_gate1] = _param_gate1_z.get();
	gates.z[index_gate2] = _param_gate2_z.get();
	gates.z[index_gate3] = _param_gate3_z.get();
	gates.z[index_gate4] = _param_gate4_z.get();

	gates.yaw[index_gate1] = _param_gate1_yaw.get();
	gates.yaw[index_gate2] = _param_gate2_yaw.get();
	gates.yaw[index_gate3] = _param_gate3_yaw.get();
	gates.yaw[index_gate4] = _param_gate4_yaw.get();
}

bool GatePublisher::get_gate_passing(Pos gatePos, float gateYaw, Pos curDronePos, Pos prevDronePos, float width = 1.0f, float height = 1.0f) {
	Pos P0 = prevDronePos - gatePos;
	Pos P1 = curDronePos - gatePos;

	float sin_yaw = sinf(gateYaw);
	float cos_yaw = cosf(gateYaw);

	Pos P0_local;
	Pos P1_local;

	P0_local.x = P0.x * cos_yaw + P0.y * sin_yaw;
	P0_local.y = -P0.x * sin_yaw + P0.y * cos_yaw;
	P0_local.z = P0.z;

	P1_local.x = P1.x * cos_yaw + P1.y * sin_yaw;
	P1_local.y = -P1.x * sin_yaw + P1.y * cos_yaw;
	P1_local.z = P1.z;

	Pos P_diff = P1_local - P0_local;

	Pos P_hit = P1_local - (P_diff / P_diff.x * P1_local.x);

	bool in_bounds = (
		std::abs(P_hit.y) < width / 2 &&
		std::abs(P_hit.z) < height / 2
	);

	bool through_plane = std::signbit(P0_local.x) != std::signbit(P1_local.x);

	return in_bounds && through_plane;
}

void GatePublisher::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}


	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	vehicle_local_position_s position;

	if (_position_sub.update(&position)) {

		const float ground_offset = _param_z_offset.get();

		Pos curDronePos = {
			position.y,
			position.x,
			-position.z - ground_offset
		};

		Pos nextGatePos = {
			gates.x[0],
			gates.y[0],
			gates.z[0]
		};
		float nextGateYaw = gates.yaw[0];

		if (get_gate_passing(
				nextGatePos,
				nextGateYaw,
				curDronePos,
				prevPosition)) {
			PX4_INFO("Passed Gate %i", next_gate);

			next_gate = (next_gate + 1) % N_GATES;
		}

		set_gates();
		_gates_pub.publish(gates);
		prevPosition = curDronePos;
	}
}

int GatePublisher::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GatePublisher::print_status()
{
	PX4_INFO("Current gate: %i", next_gate);

	return 0;
}

int GatePublisher::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Gate Publisher module.
This module publishes the position of gates for drone racing.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gate_publisher", "dunno what to put here");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}



extern "C" __EXPORT int gate_publisher_main(int argc, char *argv[])
{
	return GatePublisher::main(argc, argv);
}
