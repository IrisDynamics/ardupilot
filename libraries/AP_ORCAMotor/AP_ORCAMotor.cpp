
#include <AP_SerialManager/AP_SerialManager.h>
#include "AP_ORCAMotor.h"

#if HAL_ORCAMOTOR_ENABLED

#include "AP_ORCAMotor_Backend.h"
#include "AP_ORCAMotor_Modbus.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_ORCAMotor::var_info[] = {
    // @Group: 1_
    // @Path: AP_ORCAMotor_Params.cpp
    AP_SUBGROUPINFO(_params[0], "1_", 14, AP_ORCAMotor, AP_ORCAMotor_Params),

#if AP_ORCAMOTOR_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_ORCAMotor_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2_", 15, AP_ORCAMotor, AP_ORCAMotor_Params),
#endif

    AP_GROUPEND
};

AP_ORCAMotor::AP_ORCAMotor(){
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_ORCAMotor::init(){
    _backends[0] = NEW_NOTHROW AP_ORCAMotor_Modbus(_params[0], 0, _state[0]);
    _backends[0]->init();
    // TODO: Implement intialization via params
    // for (uint8_t i = 1; i < AP_ORCAMOTOR_MAX_INSTANCES; i++) { // Not sure why Torqeedo used i = 1
    //     if (get_instance(i) != nullptr) {
    //         return;
    //     }
    // }
    // uint8_t instance;
    // for(instance = 0; instance < AP_ORCAMOTOR_MAX_INSTANCES; instance++) {
    //     if (_params[instance].type.get() != 0) {
    //         _backends[instance] = NEW_NOTHROW AP_ORCAMotor_Modbus(_params[instance], instance, _state[instance]);
    //     }
    // }

    // for(instance = 0; instance < AP_ORCAMOTOR_MAX_INSTANCES; instance++) {
    //     if (_backends[instance] != nullptr) {
    //         _backends[instance]->init();
    //     }
    // }
}

bool AP_ORCAMotor::enabled() {
    for (int instance = 0; instance < AP_ORCAMOTOR_MAX_INSTANCES; instance++) {
        if(enabled(instance)) {
            return true;
        }
    }
    return false;
}

bool AP_ORCAMotor::enabled(uint8_t instance) {
    if (instance < AP_ORCAMOTOR_MAX_INSTANCES) {
        return _params[instance].type.get() != 0;
    }
    return false;
}

bool AP_ORCAMotor::healthy() {
    uint8_t num_backends = 0;
    uint8_t num_healthy = 0;
    for (uint8_t instance = 0; instance < AP_ORCAMOTOR_MAX_INSTANCES; instance++) {
        auto *backend = get_instance(instance);
        if (backend != nullptr) {
            num_backends++;
            if (backend->healthy()) {
                num_healthy++;
            }
        }
    }

    return ((num_backends > 0) && (num_healthy == num_backends));
}

bool AP_ORCAMotor::healthy(uint8_t instance) {
    auto* backend = get_instance(instance);
    if(backend == nullptr) {
        return false;
    }
    return backend->healthy();
}

void AP_ORCAMotor::set_mode(MotorMode mode) {
    for (int instance = 0; instance < AP_ORCAMOTOR_MAX_INSTANCES; instance++) {
        set_mode(instance, mode);
    }
}

void AP_ORCAMotor::set_mode(uint8_t instance, MotorMode mode) {
    auto* backend = get_instance(instance);
    if(backend == nullptr) {
        return;
    }
    backend->set_mode(mode);
}

void AP_ORCAMotor::set_target_position_um(int32_t target_position) {
    for(int instance = 0; instance < AP_ORCAMOTOR_MAX_INSTANCES; instance++) {
        set_target_position_um(instance, target_position);
    }
}  

void AP_ORCAMotor::set_target_position_um(uint8_t instance, int32_t target_position) {
    auto* backend = get_instance(instance);
    if(backend == nullptr) {
        return;
    }
    backend->set_target_position_um(target_position);
}

void AP_ORCAMotor::set_target_force_mN(int32_t target_force) {
    for (int instance = 0; instance < AP_ORCAMOTOR_MAX_INSTANCES; instance++) {
        set_target_force_mN(instance, target_force);
    }
    
}

void AP_ORCAMotor::set_target_force_mN(uint8_t instance, int32_t target_force) {
    auto* backend = get_instance(instance);
    if(backend == nullptr) {
        return;
    }
    backend->set_target_force_mN(target_force);
}

void AP_ORCAMotor::run_auto_zero(uint8_t instance) {
    auto* backend = get_instance(instance);
    if(backend == nullptr) {
        return;
    }
    backend->run_auto_zero();
}

void AP_ORCAMotor::run_auto_zero() {
    for (int instance = 0; instance < AP_ORCAMOTOR_MAX_INSTANCES; instance++) {
        run_auto_zero(instance);
    }
}

AP_ORCAMotor_Backend* AP_ORCAMotor::get_instance(uint8_t instance) const {
    if(instance < AP_ORCAMOTOR_MAX_INSTANCES) {
        return _backends[instance];
    }
    return nullptr;
}

AP_ORCAMotor *AP_ORCAMotor::_singleton = nullptr;

AP_ORCAMotor* AP_ORCAMotor::get_singleton() {
    return _singleton;
}

namespace AP {
    AP_ORCAMotor* orcamotor() {
        return AP_ORCAMotor::get_singleton();
    }
}
#endif