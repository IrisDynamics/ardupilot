#pragma once

#include "AP_ORCAMotor_config.h"

#if HAL_ORCAMOTOR_ENABLED
#include "AP_ORCAMotor.h"

class AP_ORCAMotor_Backend {
public:
    AP_ORCAMotor_Backend(AP_ORCAMotor_Params &params, uint8_t instance);

    CLASS_NO_COPY(AP_ORCAMotor_Backend);

    virtual void init() = 0;
    virtual bool healthy() = 0;

    virtual void set_mode(MotorMode mode) = 0;
    inline void set_target_position_um(int32_t _target_position){
        this->target_position = _target_position;
    }

    uint32_t target_position;
    AP_HAL::UARTDriver *serial_display_uart;

protected:
    AP_ORCAMotor_Params &_params;    // parameters for this backend
    uint8_t _instance;              // this instance's number
};

#endif