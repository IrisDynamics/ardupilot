
#include <AP_SerialManager/AP_SerialManager.h>
#include "AP_ORCAMotor.h"

#if HAL_ORCAMOTOR_ENABLED

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

    for (uint8_t i = 1; i < AP_ORCAMOTOR_MAX_INSTANCES; i++) {
        if (get_instance(i) != nullptr) {
            return;
        }
    }

    uint8_t instance;
    for(instance = 0; instance < AP_ORCAMOTOR_MAX_INSTANCES; instance++) {
        if (_params[instance].enabled.get() != 0) {
            _backends[instance] = NEW_NOTHROW AP_ORCAMotor_Modbus(_params[instance], instance);
        }
    }
    // motor_uart = uart;
    // if (uart == nullptr) {
    //     // that UART doesn't exist on this platform
    //     return;
    // }

    // motor_uart->begin(HIGH_SPEED_MOTOR_BAUD);
    // motor_uart->configure_parity(EVEN);

    // serial_display_uart = hal.serial(5);
    // serial_display_uart->begin(DEFAULT_MOTOR_BAUD);
    // serial_display_uart->configure_parity(EVEN);
    // current_state = OrcaState::PINGING;
    // state_start_time = AP_HAL::millis();
}

void AP_ORCAMotor::update(){
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

#endif