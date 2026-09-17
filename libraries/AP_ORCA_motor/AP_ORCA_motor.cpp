
#include <AP_SerialManager/AP_SerialManager.h>
#include "AP_ORCA_motor.h"
extern const AP_HAL::HAL& hal;

//AP_ORCA_motor *AP_ORCA_motor::_singleton;

AP_ORCA_motor::AP_ORCA_motor(){

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // if (_singleton != nullptr) {
    //     AP_HAL::panic("AP_FETtecOneWire must be singleton");
    // }
#endif
//    _singleton = this;
}

void AP_ORCA_motor::init(AP_HAL::UARTDriver *uart){
    motor_uart = uart;
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }

    motor_uart->begin(HIGH_SPEED_MOTOR_BAUD);
    motor_uart->configure_parity(EVEN);

    serial_display_uart = hal.serial(5);
    serial_display_uart->begin(DEFAULT_MOTOR_BAUD);
    serial_display_uart->configure_parity(EVEN);
    current_state = OrcaState::PINGING;
    state_start_time = AP_HAL::millis();
}

void AP_ORCA_motor::update(){
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
    uart_poll(motor_uart);
    //uint32_t elapsed_ms = AP_HAL::millis() - state_start_time;

    switch (current_state){
        case OrcaState::PINGING:
            enqueue_ping_message();
            // if (check_ping_response(rx_buffer, rx_head, rx_tail)) {
            //     printf("echo received");
            // }
            current_state = OrcaState::PING_WAIT;
            state_start_time = AP_HAL::millis();
            serial_display_uart->printf("ping sent");
            serial_display_uart->flush();
            break;

        case OrcaState::PING_WAIT:
            serial_display_uart->printf("ping wait");
            serial_display_uart->flush();
            if (check_ping_response()) {
                current_state = OrcaState::SENDING;
                serial_display_uart->printf("echo received");
            } else if ((AP_HAL::millis() - state_start_time) > 1000) { // 100 ms timeout
                current_state = OrcaState::PINGING; // retry ping
            }
        break;

        case OrcaState::SENDING:
            enqueue_extended_motor_frame(target_position,0);
            state_start_time = AP_HAL::millis();
            current_state = OrcaState::RECEIVING;
        break;
        case OrcaState::RECEIVING:
            if (check_motor_frame_response()) {
                current_state = OrcaState::SENDING;
                //serial_display_uart->printf("motor frame received");
            } else if ((AP_HAL::millis() - state_start_time) > 1000) { // 1000 ms timeout
                serial_display_uart->printf("timeout reconnect");
                current_state = OrcaState::PINGING; // retry ping
            }
        break;
        case OrcaState::CONFIG:
        break;
        case OrcaState::IDLE:
        break;
    }
}

void AP_ORCA_motor::enqueue_ping_message(){
    // uint8_t tx[6] = {1, 0x08, 0, 0,0,0};
    // uint16_t crc = generate_crc(tx, 4);
    // tx[4] = crc & 0xFF;
    // tx[5] = crc >> 8;
    motor_uart->write(ping_message, sizeof(ping_message));
    motor_uart->flush();

}

void AP_ORCA_motor::enqueue_extended_motor_frame(uint32_t position_um, uint16_t read_address){
    position_mode = 3;
    uint8_t tx[extended_motor_frame_tx_length] = {
        slave_id, 
        extended_motor_frame_function_code, 
        position_mode, 
        (uint8_t)(position_um>>24),
        (uint8_t)(position_um>>16),
        (uint8_t)(position_um>>8),
        (uint8_t)(position_um),
        (uint8_t)(read_address>>8),
        (uint8_t)read_address,
        0,
        0
    };
    uint16_t crc = generate_crc(tx, extended_motor_frame_tx_length-2);
    tx[extended_motor_frame_tx_length-2] = crc & 0xFF;
    tx[extended_motor_frame_tx_length-1] = crc >> 8;
    motor_uart->write(tx, sizeof(tx));
    motor_uart->flush();
}

bool AP_ORCA_motor::check_ping_response(){
    if (rx_buffer_count() < ping_response_length){
        //serial_display_uart->printf("not enough bytes: %d", rx_buffer_count());
        return false;
    }
    uint8_t transaction_frame[ping_response_length];

    // copy bytes without disturbing order
    for (uint8_t i = 0; i < ping_response_length; i++) {
        transaction_frame[i] = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
        if (transaction_frame[i] != ping_message[i]){
            //serial_display_uart->printf("byte mismatch: %c, %c", transaction_frame[i], ping_message[i]);
            return false;
        }
    }
    return true;
}
bool AP_ORCA_motor::check_motor_frame_response()
{
    if (rx_buffer_count() < extended_motor_frame_rx_length){
        //serial_display_uart->printf("not enough bytes: %d", rx_buffer_count());
        return false;
    }
    //serial_display_uart->printf("got enough bytes: %d \n\n", rx_buffer_count());
    uint8_t transaction_frame[extended_motor_frame_rx_length];

    // copy bytes without disturbing order
    for (uint8_t i = 0; i < extended_motor_frame_rx_length; i++) {
        transaction_frame[i] = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    }

    //check slave address
    if (transaction_frame[0] != slave_id){
        return false;
    }

    //check function code
    if (transaction_frame[1] != extended_motor_frame_function_code){
        return false;
    }
   
	uint8_t* d = transaction_frame;

    int idx = 2;
	idx = parseint32(d, idx, (int32_t*)&motor_data.force_mN);
	idx = parseint32(d, idx, (int32_t*)&motor_data.position_um);
    idx = parseint32(d, idx, (int32_t*)&motor_data.speed_mm_s);
	idx = parseint32(d, idx, (int32_t*)&motor_data.acceleration_mm_s_2);
	idx = parseint16(d, idx, (int16_t*)&motor_data.board_temp_C);
	idx = parseint16(d, idx, (int16_t*)&motor_data.coil_temp_C);
	idx = parseint16(d, idx, &motor_data.voltage_V);
	idx = parseint16(d, idx, &motor_data.power_W);
	idx = parseint16(d, idx, &motor_data.mode_of_operation);
    int16_t kin_status, kin_complete_count, placeholder;
	idx = parseint16(d, idx, &kin_status);
	idx = parseint16(d, idx, &kin_complete_count);
	idx = parseint16(d, idx, &motor_data.errors);
	idx = parseint16(d, idx, &placeholder);

    if (bad_crc(transaction_frame, sizeof(transaction_frame))){
        return false;
    }

    return true;
}

bool AP_ORCA_motor::bad_crc(uint8_t* rx_data, uint16_t rx_message_length){
    uint16_t crc = generate_crc(rx_data,rx_message_length-2);
    if ((rx_data[rx_message_length-2] != (crc & 0xFF)) | (rx_data[rx_message_length-1] != (crc >> 8))){
        //serial_display_uart->printf("bad crc");
        return true;
    }
    return false;
}

uint16_t AP_ORCA_motor::rx_buffer_count()
{
    if (rx_head >= rx_tail)
        return rx_head - rx_tail;

    return RX_BUFFER_SIZE - rx_tail + rx_head;
}


#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;

uint32_t state_start_time = 0;