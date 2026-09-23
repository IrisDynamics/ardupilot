
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
    // for (uint8_t i = 0; i < AP_ORCAMOTOR_MAX_INSTANCES; i++) {
    //     if (get_instance(i) != nullptr) {
    //         return;
    //     }
    // }
    // uint8_t instance;
    // for(instance = 0; instance < AP_ORCAMOTOR_MAX_INSTANCES; instance++) {
    //     if (_params[instance].type.get() != 0) {
    //         _backends[instance] = NEW_NOTHROW AP_ORCAMotor_Modbus(_params[instance], instance);
    //     }
    // }

    // for(instance = 0; instance < AP_ORCAMOTOR_MAX_INSTANCES; instance++) {
    //     if (_backends[instance] != nullptr) {
    //         _backends[instance]->init();
    //     }
    // }
}

void AP_ORCAMotor::update(){
    for(int instance = 0; instance < AP_ORCAMOTOR_MAX_INSTANCES; instance++) {
        auto* backend = get_instance(instance);
        if(backend == nullptr) {
            continue;
        }
        backend->update();
    }
    //uart_poll(motor_uart);
    // uint8_t tx[8];

    // tx[0] = 1;        // slave id
    // tx[1] = 0x03;     // function
    // tx[2] = 0x01;     // reg hi
    // tx[3] = 0x52;     // reg lo
    // tx[4] = 0x00;     // count hi
    // tx[5] = 0x01;     // count lo

    // uint16_t crc = generate_crc(tx, 6);
    // tx[6] = crc & 0xFF;
    // tx[7] = crc >> 8;

    // motor_uart->write(tx, sizeof(tx));
    // motor_uart->flush();
    // uint8_t rx[7];
    // while(motor_uart->available()){
    //     uint8_t byte = motor_uart->read();
    //     process_byte(byte);
    // }
    // serial_display_uart->write(rx, sizeof(rx));
    // serial_display_uart->flush();
    // uart_poll(motor_uart);
    // //uint32_t elapsed_ms = AP_HAL::millis() - state_start_time;

    // switch (current_state){
    //     case OrcaState::PINGING:
    //         enqueue_ping_message();
    //         // if (check_ping_response(rx_buffer, rx_head, rx_tail)) {
    //         //     printf("echo received");
    //         // }
    //         current_state = OrcaState::PING_WAIT;
    //         state_start_time = AP_HAL::millis();
    //         serial_display_uart->printf("ping sent");
    //         serial_display_uart->flush();
    //         break;

    //     case OrcaState::PING_WAIT:
    //         serial_display_uart->printf("ping wait");
    //         serial_display_uart->flush();
    //         if (check_ping_response()) {
    //             current_state = OrcaState::SENDING;
    //             serial_display_uart->printf("echo received");
    //         } else if ((AP_HAL::millis() - state_start_time) > 1000) { // 100 ms timeout
    //             current_state = OrcaState::PINGING; // retry ping
    //         }
    //     break;

    //     case OrcaState::SENDING:
    //         enqueue_extended_motor_frame(target_position,0);
    //         state_start_time = AP_HAL::millis();
    //         current_state = OrcaState::RECEIVING;
    //     break;
    //     case OrcaState::RECEIVING:
    //         if (check_motor_frame_response()) {
    //             current_state = OrcaState::SENDING;
    //             //serial_display_uart->printf("motor frame received");
    //         } else if ((AP_HAL::millis() - state_start_time) > 1000) { // 1000 ms timeout
    //             serial_display_uart->printf("timeout reconnect");
    //             current_state = OrcaState::PINGING; // retry ping
    //         }
    //     break;
    //     case OrcaState::CONFIG:
    //     break;
    //     case OrcaState::IDLE:
    //     break;
    // }
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