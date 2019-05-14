#include "Copter.h"

#if MODE_QU_NEW_ENABLED == ENABLED

/*
 * Init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
bool Copter::ModeQUNew::init(bool ignore_checks)
{
	qu_mission_start();
    return true;
	}

// QU New mode run
void Copter::ModeQUNew::run()
{
	// Altitude is 1 m	
	pos_control->set_alt_target(100);
	pos_control->init_takeoff();
	go_to(1);
	chThdSleepMicroseconds(3e6);
	go_to(0);
	chThdSleepMicroseconds(3e6);
	go_to(2);
	chThdSleepMicroseconds(3e6);
	go_to(0);
	chThdSleepMicroseconds(3e6);
	go_to(3);
	chThdSleepMicroseconds(3e6);
	go_to(0);
	chThdSleepMicroseconds(3e6);
	go_to(4);
	chThdSleepMicroseconds(3e6);
	go_to(0);
	chThdSleepMicroseconds(3e6);
	go_to(5);
	chThdSleepMicroseconds(3e6);
	go_to(0);
	chThdSleepMicroseconds(3e6);
    return;
}


// initialise mission
void Copter::ModeQUNew::qu_mission_start()
{
    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

// QU New mode run (Going to different payloads now)
void Copter::ModeQUNew::go_to(int payload_num)
{

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

	switch(payload_num)
	{
		case 0:
			target=home;
			break;
			
		case 1:
			target=payload_1;
			break;
			
		case 2:
			target=payload_2;
			break;
			
		case 3:
			target=payload_3;
			break;
			
		case 4:
			target=payload_4;
			break;
			
		case 5:
			target=payload_5;
			break;			
	}
	
    // Initialize controller and set target
	pos_control->init_xy_controller();
	pos_control->set_xy_target(target.x,target.y);
	pos_control->set_target_to_stopping_point_xy();
	
	// run position controller
    pos_control->update_xy_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), 0.0f);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), auto_yaw.yaw(), true);
    }
}
 
#endif