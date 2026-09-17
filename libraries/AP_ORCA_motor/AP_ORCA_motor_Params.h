#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_ORCA_Motor_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_ORCA_Motor_Params(void);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_ORCA_Motor_Params);

    // parameters
    //uint32_t baud_rate;
    //uint16_t communication_timeout;
    //uint32_t force_maximum_pid;
    AP_Int32 p_gain_pid;
    AP_Int32 i_gain_pid;
    AP_Int32 dv_gain_pid;
    AP_Int32 speed_limit;
};