#include "Copter.h"

// wall_init - initialise wall controller
bool ModeWall::init(bool ignore_checks)
{
    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    //get the current x and y measurement for a certainly target
    const Vector3f &curr_pos = inertial_nav.get_position_neu_cm();
    //set as a constant position target
    float wall_pos_x = curr_pos.x;
    float wall_pos_y = curr_pos.y;
    //we set the position desire to the position controller
    pos_control->set_pos_target_xy_cm(wall_pos_x, wall_pos_y);

    return true;
}

void ModeWall::run(){

    //handle pilot input
    float target_climb_rate = 0.0f;
    //set vetical speed and acceleration limit
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();


    //                  //
    // position control //
    //                  //

    //perform the xy position controller
    //  It will set the target_pitch and target_roll
    //  Accel-to-lean do so
    pos_control->update_xy_controller();


    //get pilot desired climb rate
    target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    //safe check
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    //Send the command climb rate to the position controller
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);

    //                  //
    // attitude control //
    //                  //
    //get the target_pitch and target_roll with safty check
    //float target_roll = 0.0f, target_pitch = 0.0f;
    // get_pilot_desired_lean_angles(target_roll, target_pitch, MIN(attitude_control->lean_angle_max_cd(), pos_control->get_lean_angle_max_cd()) * (2.0f/3.0f), attitude_control->get_althold_lean_angle_max_cd());

    //Then we need to call the attitude controller
    //  A ten degrees feed back is require
    //target_pitch = MIN(-10.0f,target_pitch);
    // attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, 0);

    pos_control->update_z_controller();

}

