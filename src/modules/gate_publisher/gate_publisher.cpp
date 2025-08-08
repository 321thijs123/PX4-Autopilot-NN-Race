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
*/

#include "gate_publisher.hpp"

GatePublisher::GatePublisher() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{

}

GatePublisher::~GatePublisher()
{

}


bool GatePublisher::init()
{
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

	gates_s gates;

	gates.timestamp = hrt_absolute_time();

	gates.x[0] = _param_gate1_x.get();
	gates.x[1] = _param_gate2_x.get();
	gates.x[2] = _param_gate3_x.get();
	gates.x[3] = _param_gate4_x.get();

	gates.y[0] = _param_gate1_y.get();
	gates.y[1] = _param_gate2_y.get();
	gates.y[2] = _param_gate3_y.get();
	gates.y[3] = _param_gate4_y.get();

	gates.z[0] = _param_gate1_z.get();
	gates.z[1] = _param_gate2_z.get();
	gates.z[2] = _param_gate3_z.get();
	gates.z[3] = _param_gate4_z.get();

	gates.yaw[0] = _param_gate1_yaw.get();
	gates.yaw[1] = _param_gate2_yaw.get();
	gates.yaw[2] = _param_gate3_yaw.get();
	gates.yaw[3] = _param_gate4_yaw.get();

	_gates_pub.publish(gates);
}

int GatePublisher::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GatePublisher::print_status()
{
	PX4_INFO("I'm alive");

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
