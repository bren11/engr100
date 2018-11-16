#include <iostream>
#include <cstdlib>
#include <fstream>
#include <ctime>
#include <sstream>
#include "Copter.h"

using namespace std;

/*
 * custom_param1 is distance around the drone to walls
 * custom_param2 is the time of overshoot
 * custom_param3 is the angle of attack
 */
const int xSize = 300;
const int ySize = 400;
int thresh = 5;
string map[xSize][ySize];
int dronePosX = 200;
int dronePosY = 30;
int prevL = 0;
int prevR = 0;
int prevF = 0;
bool init = false;

int mode = 0; // initialize going forward
int error = 5;

float moving_angle = 0;
bool hold_front = false;
bool hold_left = false;
bool hold_right = false;

time_t startTime = time(NULL);

void end() {
	ofstream myfile;
	time_t current_time;
	current_time = time(NULL);

	myfile.open("data" + std::to_string(current_time) + ".txt");
	for (auto& m : map) {
		for (auto& j : m) {
			myfile << j;
		}
		myfile << "\n";
	myfile.close();
	}
}

// custom_init - initialise custom controller
bool Copter::custom_init(bool ignore_checks)
{
    // initialize vertical speeds and leash lengths
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // stop takeoff if running
    takeoff_stop();

    // reset integrators for roll and pitch controllers
    g.pid_roll.reset_I();
    g.pid_pitch.reset_I();

	moving_angle = g.custom_param3;

    return true;
}

// custom_run - runs the custom controller
// should be called at 100hz or more
void Copter::custom_run()
{
    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // desired roll, pitch, and yaw_rate
    float target_roll = 0.0f, target_pitch = 0.0f, target_yaw_rate = 0.0f;

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors->rotor_runup_complete());
#else
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);
#endif
    target_climb_rate = 0.0f;

    // Alt Hold State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        althold_state = AltHold_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#if FRAME_CONFIG == HELI_FRAME    
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
        heli_flags.init_targets_on_arming=true;
#else
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:
#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            heli_flags.init_targets_on_arming=false;
        }
#endif
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case AltHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
            if (motors->get_interlock()) {
                heli_flags.init_targets_on_arming=false;
            }
        }
#else
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#endif
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case AltHold_Flying:
        // compute the target climb rate, roll, pitch and yaw rate
        // land if custom_controller returns false
        if (!custom_controller(target_climb_rate, target_roll, target_pitch, target_yaw_rate)) {
            // switch to land mode
			end();
            set_mode(LAND, MODE_REASON_MISSION_END);
            break;
        }

        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        avoid.adjust_roll_pitch(target_roll, target_pitch, aparm.angle_max);
#endif

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        break;
    }
}

void setPoint(int x,int y, string str) {
	if (x > 0 && x < xSize && y > 0 && y < ySize) {
		map[x][ySize - y - 1] = str;
	}
}

void setRelativePoint(int x, int y, string str) {
	setPoint(x + dronePosX, y + dronePosY, str);
}

void deltaDrone(int x, int y) {
	dronePosX += x;
	dronePosY += y;
}

void start(int dist_forward, int dist_left, int dist_right) {
	for (auto& m : map) {
		for (auto& j : m) {
			j = " ";
		}
	}
	for (int i = 0; i < 10; i++) {
		prevL = dist_left;
		prevF = dist_forward;
		prevR = dist_right;
	}
	prevF /= 10;
	prevL /= 10;
	prevR /= 10;
}

void run(int dist_forward, int dist_left, int dist_right) {
	if (abs(dist_forward - prevF) < thresh) {
		deltaDrone(0, prevF - dist_forward);
	} 

	if (abs(dist_left - prevL) < thresh || abs(dist_right - prevR) < thresh) {
		if (abs(dist_left - prevL) < abs(dist_right - prevR)) {
			deltaDrone(dist_left - prevL, 0);
		} else {
			deltaDrone(prevR - dist_right, 0);
		}
	}

	setRelativePoint(0, dist_forward, "X");
	setRelativePoint(-dist_left, 0, "X");
	setRelativePoint(dist_right, 0, "X");
	
	setRelativePoint(0,0, "O");

	prevL = dist_left;
	prevF = dist_forward;
	prevR = dist_right;
}

void Move_left(float &target_roll, float &target_pitch) {
	target_roll = -1*moving_angle;
	target_pitch = 0;
}

void Move_right(float &target_roll, float &target_pitch) {
	target_roll = moving_angle;
	target_pitch = 0;
}

void Move_forward(float &target_roll, float &target_pitch) {
	target_pitch = moving_angle;
	target_roll = 0;
}

void Move_back(float &target_roll, float &target_pitch) {
	target_pitch = -1*moving_angle;
	target_roll = 0;
}

void Stop(float &target_roll, float &target_pitch) {
	target_pitch = 0;
	target_roll = 0;
}

// custom_controller - computes target climb rate, roll, pitch, and yaw rate for custom flight mode
// returns true to continue flying, and returns false to land
bool Copter::custom_controller(float &target_climb_rate, float &target_roll, float &target_pitch, float &target_yaw_rate)
{
    // get downward facing sensor reading in meters
    float rangefinder_alt = (float)rangefinder_state.alt_cm / 100.0f;

    // get horizontal sensor readings in meters
    float dist_forward, dist_right, dist_backward, dist_left;
    g2.proximity.get_horizontal_distance(0, dist_forward);
    g2.proximity.get_horizontal_distance(90, dist_right);
    g2.proximity.get_horizontal_distance(180, dist_backward);
    g2.proximity.get_horizontal_distance(270, dist_left);

    // set desired climb rate in centimeters per second
    target_climb_rate = 0.0f;

    // set desired roll and pitch in centi-degrees
    target_pitch = 0.0f;
    target_roll = 0.0f;

    // set desired yaw rate in centi-degrees per second (set to zero to hold constant heading)
    target_yaw_rate = 0.0f;

	switch (mode) {
		//move forward towards wall
		time_t endTime;
		case 0:
			hold_front = false;
			Move_forward(target_roll, target_pitch);
			if (dist_forward - g.custom_param1 < -1 * error) {
				mode = 1;
			}
			break;
		//hit wall and start moving right
		case 1:
			hold_front = true;
			hold_left = false;
			hold_right = false;
			Move_right(target_roll, target_pitch);
			//hit open space in front
			if (dist_forward - g.custom_param1 > 3 * error) {
				startTime = time(NULL);
				mode = 2;
				hold_front = false;
			//hit other wall to go other way
			} else if (dist_right - g.custom_param1 < -1 * error) {
				mode = 3;
			}
			break;
		//wait to pass edge of wall then go back to straight
		case 2:
			Move_right(target_roll, target_pitch);
			endTime = time(NULL);
			if (endTime - startTime > g.custom_param2) {
				mode = 0;
				hold_right = true;
			}
			break;
		//hit wall and move left
		case 3:
			hold_front = true;
			hold_left = false;
			hold_right = false;
			Move_left(target_roll, target_pitch);
			if (dist_forward - g.custom_param1 > 3 * error) {
				startTime = time(NULL);
				mode = 4;
				hold_front = false;
			} else if (dist_right - g.custom_param1 < -1 * error) {
				mode = 5;
			}
			break;
		//move left a bit more to avoid edge of wall
		case 4:
			Move_left(target_roll, target_pitch);
			endTime = time(NULL);
			if (endTime - startTime > g.custom_param2) {
				mode = 0;
				hold_left = true;
			}
			break;
		//cant move, so land.
		case 5:
			Stop(target_roll, target_pitch);
			return true;
	}

	/*if (hold_front) {
		g.custom_pid_pitch.set_input_filter_all(dist_forward - g.custom_param1);
		target_pitch = g.custom_pid_pitch.get_pid(); 
	}
	if (hold_left) {
		g.custom_pid_roll.set_input_filter_all(g.custom_param1 - dist_left);
		target_roll = g.custom_pid_roll.get_pid(); 
	}
	if (hold_right) {
		g.custom_pid_roll.set_input_filter_all(dist_right - g.custom_param1);
		target_roll = g.custom_pid_roll.get_pid(); 
	}*/

	if (!init) {
		start(dist_forward, dist_left, dist_right);
	}

	run(dist_forward, dist_left, dist_right);

    return false;
}
