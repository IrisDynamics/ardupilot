#include "AP_ORCAMotor_Params.h"
#include <SRV_Channel/SRV_Channel.h>

// table of user settable parameters
const AP_Param::GroupInfo AP_ORCAMotor_Params::var_info[] = {
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_ORCAMotor_Params, enabled, 0, AP_PARAM_FLAG_ENABLE),
    AP_GROUPINFO("FORCE_SATURATION", 2, AP_ORCAMotor_Params, force_saturation, 300000),
    AP_GROUPINFO("P_GAIN_PID", 3, AP_ORCAMotor_Params, p_gain_pid, 1000),
    AP_GROUPINFO("I_GAIN_PID", 4, AP_ORCAMotor_Params, i_gain_pid, 2000),
    AP_GROUPINFO("D_GAIN_PID", 5, AP_ORCAMotor_Params, d_gain_pid, 100),
    AP_GROUPINFO("SPEED_LIMIT", 6, AP_ORCAMotor_Params, speed_limit, 1000),
    AP_GROUPINFO("ACCEL_LIMIT", 7, AP_ORCAMotor_Params, accel_limit, 1000),
    AP_GROUPINFO("DECCEL_LIMIT", 8, AP_ORCAMotor_Params, deccel_limit, 1000),
    AP_GROUPINFO("SOFTSTART_DURATION", 9, AP_ORCAMotor_Params, softstart_duration, 3000),
    AP_GROUPINFO("INVERT_POSITION", 10, AP_ORCAMotor_Params, invert_position, 0),
    AP_GROUPINFO("AUTOZERO_MODE", 11, AP_ORCAMotor_Params, autozero_mode, 0),
    AP_GROUPINFO("AUTOZERO_FORCE", 12, AP_ORCAMotor_Params, autozero_force, 30),
    AP_GROUPINFO("AUTOZERO_SPEED", 13, AP_ORCAMotor_Params, autozero_speed, 50),
    AP_GROUPINFO("AUTOZERO_EXIT_MODE", 13, AP_ORCAMotor_Params, autozero_exit_mode, 0),
    AP_GROUPEND
};

AP_ORCAMotor_Params::AP_ORCAMotor_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}