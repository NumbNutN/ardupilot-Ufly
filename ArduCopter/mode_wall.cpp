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
    
    //  we set the position desire to the position controller
    //  only the position that is parallel with the wall direction is valuable
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


    //  /   /   /   /   //
    // position control //
    //  /   /   /   /   //

    //  perform the xy position controller  It will output a disired thrust vector
    pos_control->update_xy_controller();


    //get pilot desired climb rate
    target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    //safe check
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    //Send the command climb rate to the position controller
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);


    //  /   /   /   /   //
    // attitude control //
    //  /   /   /   /   //

    Vector3f thrust_vec = pos_control->get_thrust_vector(); 
    // set the desired body frame acceleration on x axis to be constant

    // ahrs.get_yaw() in radian
    Vector3f pitch_desired = {0,0.174f,ahrs.get_yaw()};

    //convert to the accel desired in NEU frame
    Vector3f accel_y_des = pos_control->lean_angles_to_accel(pitch_desired);
    accel_y_des.z = 0;
    thrust_vec += accel_y_des;

    //now call the attitude controller
    attitude_control->input_thrust_vector_rate_heading(thrust_vec, 0);

    pos_control->update_z_controller();

}

