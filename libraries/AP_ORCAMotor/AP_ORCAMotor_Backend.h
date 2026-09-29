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
    virtual void clear_motor_errors() = 0;
    virtual void startup_config() = 0;

protected:
    enum AutoZeroMode {
        ZERO_MODE_NEGATIVE_POS = 0,
        ZERO_MODE_MANUAL = 1,
        ZERO_MODE_AUTO_ENABLE = 2,
        ZERO_MODE_ON_BOOT = 3,
        ZERO_MODE_IOSH = 4
    };

    inline void set_mode(MotorMode _target_mode) {
        if (_target_mode == MODE_AUTOZERO) {
            return; // Use run_auto_zero() for param setup check
        }
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
    inline void run_auto_zero() {
        switch ((AutoZeroMode)_params.autozero_mode.get())
        {
        case ZERO_MODE_AUTO_ENABLE:
        case ZERO_MODE_ON_BOOT:
            this->target_mode = MODE_AUTOZERO;
            break;
        default:
            // Won't run if autozero isn't set up
            break;
        }
    }

    int32_t target_position;
    int32_t target_force;
    MotorMode target_mode;
    AP_ORCAMotor_Params &_params;    // parameters for this backend
    uint8_t _instance;              // this instance's number
    ExtMotorData& _state;
};

#endif