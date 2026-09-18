#include "AP_ORCAMotor_Params.h"
#include <SRV_Channel/SRV_Channel.h>

// table of user settable parameters
const AP_Param::GroupInfo AP_ORCAMotor_Params::var_info[] = {
    AP_GROUPINFO("FORCE_SATURATION", 1, AP_ORCAMotor_Params, force_saturation, 300000),
    AP_GROUPINFO("P_GAIN_PID", 2, AP_ORCAMotor_Params, p_gain_pid, 1000),
    AP_GROUPINFO("I_GAIN_PID", 3, AP_ORCAMotor_Params, i_gain_pid, 2000),
    AP_GROUPINFO("D_GAIN_PID", 4, AP_ORCAMotor_Params, d_gain_pid, 100),
    AP_GROUPINFO("SPEED_LIMIT", 5, AP_ORCAMotor_Params, speed_limit, 1000),
    AP_GROUPINFO("ACCEL_LIMIT", 6, AP_ORCAMotor_Params, accel_limit, 1000),
    AP_GROUPINFO("DECCEL_LIMIT", 7, AP_ORCAMotor_Params, deccel_limit, 1000),
    AP_GROUPINFO("SOFTSTART_DURATION", 7, AP_ORCAMotor_Params, softstart_duration, 3000),
    AP_GROUPEND
};

AP_ORCAMotor_Params::AP_ORCAMotor_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}