#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_ORCAMotor_Params
{
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_ORCAMotor_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_ORCAMotor_Params);

    AP_Int8 type;
    AP_Int32 force_saturation;
    AP_Int16 p_gain_pid;
    AP_Int16 i_gain_pid;
    AP_Int16 d_gain_pid;
    AP_Int16 speed_limit;
    AP_Int16 accel_limit;
    AP_Int16 decel_limit;
    AP_Int16 softstart_duration;
    AP_Int16 invert_position;
    AP_Int16 autozero_mode;
    AP_Int16 autozero_force;
    AP_Int16 autozero_speed;
    AP_Int16 autozero_exit_mode;
};