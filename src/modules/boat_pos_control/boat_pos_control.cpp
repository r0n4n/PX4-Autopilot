/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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


#include "boat_pos_control.hpp"

BoatPosControl::BoatPosControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

BoatPosControl::~BoatPosControl()
{

}

bool BoatPosControl::init()
{
	//Run on fixed interval
	ScheduleOnInterval(10_ms); // 2000 us interval, 200 Hz rate

	return true;
}

void BoatPosControl::parameters_update()
{
	if (_parameter_update_sub.updated()){
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
	// init velocity controller
	_velocity_pid.setGains(_param_usv_speed_p.get(),0,0);
	_velocity_pid.setOutputLimit(200);


	// init yaw rate controller
	_yaw_rate_pid.setGains(_param_usv_yaw_rate_p.get(),_param_usv_yaw_i.get(),0);
	_velocity_pid.setOutputLimit(200);
}

void BoatPosControl::vehicle_attitude_poll()
{
	if (_att_sub.updated()) {
		_att_sub.copy(&_vehicle_att);
	}
}

void BoatPosControl::Run()
{
	float dt = 0.01; // Using non zero value to a avoid division by zero

	/* advertise debug value */
	struct debug_key_value_s dbg;
	strncpy(dbg.key, "debug_val", sizeof(dbg.key));
	dbg.value = 0.0f;
	orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);

	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}
	parameters_update();
	vehicle_attitude_poll();

	vehicle_thrust_setpoint_s v_thrust_sp{};
	v_thrust_sp.timestamp = hrt_absolute_time();
	v_thrust_sp.xyz[0] = 0.0f;
	v_thrust_sp.xyz[1] = 0.0f;
	v_thrust_sp.xyz[2] = 0.0f;

	vehicle_torque_setpoint_s v_torque_sp{};
	v_torque_sp.timestamp = hrt_absolute_time();
	v_torque_sp.xyz[0] = 0.f;
	v_torque_sp.xyz[1] = 0.f;

	boat_position_setpoint_s boat_pos_sp{};
	boat_pos_sp.timestamp = hrt_absolute_time();
	boat_pos_sp.state = 0;
	boat_pos_sp.vx = 0.f ;

	const Quatf q{_vehicle_att.q};
	float yaw = Eulerf(q).psi();

	bool backward = false;




	if (_vehicle_control_mode_sub.updated()) {


		if (_vehicle_control_mode_sub.copy(&vehicle_control_mode)) {
			_armed = vehicle_control_mode.flag_armed;
			_position_ctrl_ena = vehicle_control_mode.flag_control_position_enabled; // change this when more modes are supported
		}
	}

	if (_position_setpoint_triplet_sub.updated()) {
		_position_setpoint_triplet_sub.copy(&_position_setpoint_triplet);
	}

	if (_vehicle_global_position_sub.updated()) {
		_vehicle_global_position_sub.copy(&_vehicle_global_position);
	}

	matrix::Vector2d global_position(_vehicle_global_position.lat, _vehicle_global_position.lon);
	matrix::Vector2d current_waypoint(_position_setpoint_triplet.current.lat, _position_setpoint_triplet.current.lon);
	matrix::Vector2d next_waypoint(_position_setpoint_triplet.next.lat, _position_setpoint_triplet.next.lon);

	float distance_to_next_wp = get_distance_to_next_waypoint(global_position(0), global_position(1),
					  current_waypoint(0),
					  current_waypoint(1));

	float desired_heading_hold = 0 ;

	//if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER  && _last_vehicle_status.nav_state !=vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER){
	//	desired_heading = yaw; // the yaw setpoint is the yaw measured when the vehicle enters in Loiter mode
	//}else {
	desired_heading = get_bearing_to_next_waypoint(global_position(0), global_position(1), current_waypoint(0),
				current_waypoint(1));
	//}

	// Velocity in body frame
	const Dcmf R_to_body(Quatf(_vehicle_att.q).inversed());
	const Vector3f vel = R_to_body * Vector3f(_local_pos.vx, _local_pos.vy, _local_pos.vz);
	float speed = vel(0);
	boat_pos_sp.vx = speed;
	boat_pos_sp.yaw_sp = desired_heading;
	boat_pos_sp.yaw = yaw;


	boat_pos_sp.distance_to_wp = distance_to_next_wp;



	if (_armed && _position_ctrl_ena){
		if (_local_pos_sub.update(&_local_pos)) {
			_manual_control_setpoint_sub.copy(&_manual_control_setpoint);
			_vehicle_status_sub.copy(&vehicle_status);



			if(vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION){
		            speed_sp = float(_position_setpoint_triplet.current.cruising_speed) ;
			}
			else{
			   speed_sp = distance_to_next_wp*_param_usv_pos_p.get() ;
			}

			// Adjust the setpoint to take the shortest path
			float heading_error = desired_heading - yaw;
			if (abs(heading_error)>M_PI_F){
				float new_heading_error = 2*M_PI_F-abs(heading_error);
				desired_heading = -sign(heading_error)*new_heading_error+yaw;
				heading_error = new_heading_error;
			}






			// state machine
			if (_current_waypoint != current_waypoint) {
				if (fabsf(heading_error) < _param_usv_yaw_epsi.get()) {
					_currentState = GuidanceState::DRIVING;
				}
				else{
					_currentState = GuidanceState::TURNING;
				}
			}


			// once position reached
			if (distance_to_next_wp<_param_usv_dist_epsi.get()) {
				_currentState = GuidanceState::HOLD;
				desired_heading_hold = yaw ;
			}

			// adjust setpoints according to the step ongoing
			switch (_currentState) {
			case GuidanceState::TURNING:
				speed_sp = 0.0f;

				if (fabsf(heading_error) < _param_usv_yaw_epsi.get()) {
					_currentState = GuidanceState::DRIVING;
				}
				break;

			case GuidanceState::DRIVING: {
				break;
			}

			case GuidanceState::HOLD:{
				desired_heading = desired_heading_hold; // keep the last direction
				speed_sp = 0.0f;
				if (distance_to_next_wp>_param_usv_dist_loiter.get()) { // if the boat is moving from the desired position
					_currentState = GuidanceState::STABILIZING; // we move to the stabilizing mode
				}
				break;
			}

			case GuidanceState::STABILIZING:{
				// select between forward and backward heading
				if (abs(heading_error)>(M_PI_F/2)){
					if (desired_heading>0){
						desired_heading = desired_heading - M_PI_F;
					}
					else{
						desired_heading = desired_heading + M_PI_F;
					}
					heading_error = desired_heading - yaw;
					backward = true;
				}
				if (backward){
					speed_sp = -speed_sp;
				}
				//once position reached
				if (distance_to_next_wp<_param_usv_dist_epsi.get()) {
					_currentState = GuidanceState::HOLD;
					desired_heading_hold = yaw ;
					backward = false;
				}
			}
			}






			// Speed control
			speed_sp = math::constrain(speed_sp, -_param_usv_speed_max.get(), _param_usv_speed_max.get());
			_velocity_pid.setSetpoint(speed_sp);
			float _thrust = _param_usv_speed_ffg.get()*speed + _velocity_pid.update(speed, dt);

			// direction control
			_yaw_rate_pid.setSetpoint(desired_heading);
			float _torque_sp = _yaw_rate_pid.update(yaw, dt);
			_torque_sp = math::constrain(_torque_sp, -1.0f, 1.0f);


			// publish torque and thurst commands
			//v_torque_sp.xyz[2] = -_torque_sp;
			v_torque_sp.xyz[2] = -_torque_sp - _manual_control_setpoint.roll; // disturbance for testing

			//_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
			//_vehicle_torque_setpoint_pub.publish(v_torque_sp);

			// manual command used for testing
			_manual_control_setpoint_sub.copy(&_manual_control_setpoint);
			//v_thrust_sp.xyz[0] = math::constrain(_manual_control_setpoint.throttle, -_param_usv_thr_max.get(), _param_usv_thr_max.get()) ; // adjusting manual setpoint until the range is properly defined in the Mavlink interface
			v_thrust_sp.xyz[0] = _thrust + _manual_control_setpoint.throttle;
			//v_thrust_sp.xyz[0] = 0;
			_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);

			//v_torque_sp.xyz[2] = math::constrain(-_manual_control_setpoint.roll,-_param_usv_trq_max.get(),_param_usv_trq_max.get());
			_vehicle_torque_setpoint_pub.publish(v_torque_sp);

			//_differential_drive_guidance.computeGuidance(yaw,vel(0),dt);
			_current_waypoint = current_waypoint;
		}
		boat_pos_sp.state = uint8_t(_currentState);
		boat_pos_sp.vx_sp = speed_sp;
		boat_pos_sp.yaw_sp = desired_heading;
		boat_pos_sp.yaw = yaw;
		dbg.value = float(boat_pos_sp.state);
	       	orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
	}
	else if (_armed && vehicle_control_mode.flag_control_manual_enabled) {
		_manual_control_setpoint_sub.copy(&_manual_control_setpoint);
		v_thrust_sp.xyz[0] = math::constrain(_manual_control_setpoint.throttle, -_param_usv_thr_max.get(), _param_usv_thr_max.get()) ; // adjusting manual setpoint until the range is properly defined in the Mavlink interface
		_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);

		v_torque_sp.xyz[2] = math::constrain(-_manual_control_setpoint.roll,-_param_usv_trq_max.get(),_param_usv_trq_max.get());
		_vehicle_torque_setpoint_pub.publish(v_torque_sp);


	}
	else {
		v_thrust_sp.xyz[0] = 0.0f;
		_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
		v_torque_sp.xyz[0] = 0.f;
		v_torque_sp.xyz[1] = 0.f;
		v_torque_sp.xyz[2] = 0.f;
		_vehicle_torque_setpoint_pub.publish(v_torque_sp);
	}

	_boat_position_setpoint_pub.publish(boat_pos_sp);
	_last_vehicle_status = vehicle_status;

}

int BoatPosControl::task_spawn(int argc, char *argv[])
{
	BoatPosControl *instance = new BoatPosControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int BoatPosControl::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int BoatPosControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BoatPosControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
boat controller

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("boat_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int boat_pos_control_main(int argc, char *argv[])
{
	return BoatPosControl::main(argc, argv);
}
