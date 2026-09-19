#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_ORCAMotor_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_ORCAMotor_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_ORCAMotor_Params);

    AP_Int8 enabled;
    AP_Int32 force_saturation;
    AP_Int32 p_gain_pid;
    AP_Int32 i_gain_pid;
    AP_Int32 d_gain_pid;
    AP_Int32 speed_limit;
    AP_Int32 accel_limit;
    AP_Int32 deccel_limit;
    AP_Int32 softstart_duration;
    AP_Int8 invert_position;
    AP_Int8 autozero_mode;
    AP_Int8 autozero_force;
    AP_Int16 autozero_speed;
    AP_Int8 autozero_exit_mode;
};