#include "AP_ORCAMotor_Backend.h"

#if HAL_ORCAMOTOR_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_ORCAMotor_Backend::AP_ORCAMotor_Backend(AP_ORCAMotor_Params &params, uint8_t instance) :
    _params(params),
    _instance(instance)
{}
#endif