#pragma once

#include "AP_ORCAMotor_config.h"

#if HAL_ORCAMOTOR_ENABLED
#include "AP_ORCAMotor.h"

class AP_ORCAMotor_Backend {
public:
    AP_ORCAMotor_Backend(AP_ORCAMotor_Params &params, uint8_t instance);

    CLASS_NO_COPY(AP_ORCAMotor_Backend);

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

    enum ExtMotorCommandMode {
		NoModeChange = 0,
		SleepMode 	 = 1,
		ForceMode 	 = 2,
		PosMode 	 = 3,
		KinMode 	 = 5
	};

    inline void set_target_position(int32_t target_position){
        this->target_position = target_position;
    }

private:
    AP_ORCAMotor_Params &_params;    // parameters for this backend
    uint8_t _instance;              // this instance's number
    uint32_t target_position;
};

#endif