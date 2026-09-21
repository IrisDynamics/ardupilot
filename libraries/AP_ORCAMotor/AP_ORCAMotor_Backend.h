#pragma once

#include "AP_ORCAMotor_config.h"

#if HAL_ORCAMOTOR_ENABLED
#include "AP_ORCAMotor.h"

class AP_ORCAMotor_Backend {
public:
    AP_ORCAMotor_Backend(AP_ORCAMotor_Params &params, uint8_t instance);

    CLASS_NO_COPY(AP_ORCAMotor_Backend);

    enum MotorMode {
        MODE_SLEEP = 1,
        MODE_FORCE = 2,
        MODE_POSITION = 3,
        MODE_HAPTIC = 4,
        MODE_KINEMATIC = 5,
        MODE_PWM = 11,
        MODE_AUTOZERO = 55
    };

    struct MotorData{
        int32_t force_mN;
        int32_t position_um;
        int32_t speed_mm_s;
        int32_t acceleration_mm_s_2;
        int16_t board_temp_C;
        int16_t coil_temp_C;
        int16_t mode_of_operation;
        int16_t errors;
        int16_t power_W;
        int16_t voltage_V;

    };

    virtual void init() = 0;
    virtual bool healthy() = 0;

    virtual void set_mode(MotorMode mode) = 0;
    inline void set_target_position_um(int32_t target_position){
        this->target_position = target_position;
    }

    uint32_t target_position;
    AP_HAL::UARTDriver *serial_display_uart;

protected:
    AP_ORCAMotor_Params &_params;    // parameters for this backend
    uint8_t _instance;              // this instance's number
};

#endif