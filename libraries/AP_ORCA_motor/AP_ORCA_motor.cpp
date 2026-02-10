#include <AP_SerialManager/AP_SerialManager.h>
#include "AP_ORCA_motor.h"
extern const AP_HAL::HAL& hal;

AP_ORCA_motor *AP_ORCA_motor::_singleton;

AP_ORCA_motor::AP_ORCA_motor(){

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_FETtecOneWire must be singleton");
    }
#endif
    _singleton = this;
}

void AP_ORCA_motor::init_uart(AP_HAL::UARTDriver *uart, const char *name){
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }

    uart->begin(DEFAULT_MOTOR_BAUD);
    uart->configure_parity(EVEN);
}

void AP_ORCA_motor::update(){
    
}