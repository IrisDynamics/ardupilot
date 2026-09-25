#pragma once

#include "AP_ORCAMotor_config.h"

#if HAL_ORCAMOTOR_ENABLED
#include "AP_ORCAMotor.h"

class AP_ORCAMotor_Backend {
    friend class AP_ORCAMotor;
public:
    AP_ORCAMotor_Backend(AP_ORCAMotor_Params &params, uint8_t instance, ExtMotorData &state);

    CLASS_NO_COPY(AP_ORCAMotor_Backend);

    virtual void init() = 0;
    virtual bool healthy() = 0;
    virtual void update() = 0;
    virtual void clear_motor_errors() = 0;

protected:
    inline void set_mode(MotorMode _target_mode) {
        this->target_mode = _target_mode;
    }
    inline void set_target_position_um(int32_t _target_position){
        this->target_position = _target_position;
        this->target_mode = MODE_POSITION;
    }
    inline void set_target_force_mN(int32_t _target_force){
        this->target_force = _target_force;
        this->target_mode = MODE_FORCE;
    }

    int32_t target_position;
    int32_t target_force;
    MotorMode target_mode;
    AP_ORCAMotor_Params &_params;    // parameters for this backend
    uint8_t _instance;              // this instance's number
    ExtMotorData& _state;
};

#endif