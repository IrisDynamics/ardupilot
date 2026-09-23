/* Table of CRC values for high–order byte */
#include "AP_ORCAMotor_Modbus.h"

#if HAL_ORCAMOTOR_ENABLED
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/AP_HAL.h> 

extern const AP_HAL::HAL& hal;

void AP_ORCAMotor_Modbus::init() {
    //const AP_SerialManager &serial_manager = AP::serialmanager();

    //motor_uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ORCAMotor, _instance);

    if (_initialised) {
        return;
    }

    // create background thread to process serial input and output
    char thread_name[15];
    hal.util->snprintf(thread_name, sizeof(thread_name), "orcamotor%u", (unsigned)_instance);
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ORCAMotor_Modbus::thread_main, void), thread_name, 2048, AP_HAL::Scheduler::PRIORITY_RCOUT, 1)) {
        return;
    }
}

bool AP_ORCAMotor_Modbus::init_internals() {
    motor_uart = AP_HAL::get_HAL().serial(4);

    if(motor_uart != nullptr) {
        motor_uart->begin(AP_SERIALMANAGER_ORCAMOTOR_BAUD, ORCAMOTOR_RX_BYTES, ORCAMOTOR_TX_BYTES);
        motor_uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        motor_uart->set_stop_bits(1);
        motor_uart->configure_parity(UART_PARITY_EVEN);
        return true;
    }
    return false;
    
}

void AP_ORCAMotor_Modbus::thread_main() {
    if(!init_internals()) {
        return;
    }
    _initialised = true;

    while(true) {
        switch (trans.state) {
            case IDLE:
                handle_stream();
                break;
            case SENDING:
                transmit();
                break;
            case RECEIVING:
                uart_poll();
                break;
            case PROCESSING:
                process_response();
                break;
            case ERROR:
                AP_HAL::get_HAL().console->printf("Entered error state\n");
                break;
            default:
                break;
        }
        hal.scheduler->delay(100);
    }
}

bool AP_ORCAMotor_Modbus::healthy() {
    enqueue_ping_message();
    return check_ping_response();
}

void AP_ORCAMotor_Modbus::update() {
    switch (trans.state)
    {
    case IDLE:
        handle_stream();
        break;
    case SENDING:
        transmit();
        break;
    case RECEIVING:
        uart_poll();
        break;
    case PROCESSING:
        process_response();
        break;
    case ERROR:
        AP_HAL::get_HAL().console->printf("Entered error state\n");
        break;
    default:
        break;
    }
}

// Sub-functions are specified for some ORCA specific packets
void AP_ORCAMotor_Modbus::write(const FunctionCode fn, const uint8_t* const data, const size_t data_len, const uint8_t* const sub_fn, const size_t sub_fn_len) {
    // Check message sizing
    size_t len = 2 + data_len + sub_fn_len + 2; //Slave ID, Fn code, and 2 bytes for CRC
    if(len > ORCAMOTOR_TX_BYTES) {
        AP_HAL::get_HAL().console->printf("Message too long\n");
        return;
    }
    // Allocate buffer and initial index
    uint8_t buf[len];
    size_t idx = 0;
    // Build message byte-by-byte 
    buf[idx++] = ORCA_SLAVE_ID;
    buf[idx++] = fn;
    if(sub_fn != nullptr && sub_fn_len) {
        memcpy(&buf[idx], sub_fn, sub_fn_len);
        idx += sub_fn_len;
    }
    if(data != nullptr && data_len) {
        memcpy(&buf[idx], data, data_len);
        idx += data_len;
    }
    // Calculate and add CRC
    uint16_t crc = generate_crc(buf, len-2);
    uint8_t crc_bytes[2];
    crc_bytes[0] = crc & 0xFF;
    crc_bytes[1] = crc >> 8;
    buf[idx++] = crc_bytes[0];
    buf[idx++] = crc_bytes[1];
    // Update the transaction item
    memcpy(trans.tx_buf, buf, len);
    trans.tx_len = len;
    trans.fn = fn;

    // Transaction t;
    // memcpy(&t.tx_buf, buf, len);
    // t.tx_len = len;
    // t.fn = fn;
    // _rb_write(&t);
    //AP_HAL::get_HAL().console->printf("Loaded transmit buffer\n");
}

void AP_ORCAMotor_Modbus::enqueue_ping_message(){
    uint8_t diag_fn[2] = {0, 0};
    write(MB_DIAG_QUERY_DATA, nullptr, 0, diag_fn, sizeof(diag_fn));
}

void AP_ORCAMotor_Modbus::enqueue_extended_motor_frame(ExtMtrCmdMode mode, uint32_t data, uint16_t read_address){
    uint8_t tx[EXT_MOTOR_FRAME_TX_LEN] = {
        mode, 
        (uint8_t)(data>>24),
        (uint8_t)(data>>16),
        (uint8_t)(data>>8),
        (uint8_t)(data),
        (uint8_t)(read_address>>8),
        (uint8_t)read_address
    };
    return write(ORCA_EXT_MTR_STREAM, tx, sizeof(tx));
}

bool AP_ORCAMotor_Modbus::check_ping_response(){
    if (trans.rx_len < PING_RESPONSE_RX_LEN) {
         AP_HAL::get_HAL().console->printf("Not enough bytes: %d\n", trans.rx_len);
        return false;
    }
    const uint8_t expected_response[6] = {0x01, 0x08, 0x00, 0x00, 0x80, 0x1A };                    

    for (uint8_t i = 0; i < PING_RESPONSE_RX_LEN; i++) {
        if (trans.rx_buf[i] != expected_response[i]){
            AP_HAL::get_HAL().console->printf("Byte mismatch: Expected %c, got %c\n", expected_response[i], trans.rx_buf[i]);
            return false;
        }
    }
    return true;
}

bool AP_ORCAMotor_Modbus::check_ext_motor_frame_response()
{
    if (trans.rx_len < EXT_MOTOR_FRAME_RX_LEN){
        AP_HAL::get_HAL().console->printf("Wrong byte count, got %d\n", trans.rx_len);
        return false;
    }

    if (trans.rx_buf[0] != ORCA_SLAVE_ID){
        AP_HAL::get_HAL().console->printf("Wrong slave ID count, got %d\n", trans.rx_buf[0]);
        return false;
    }
    if (trans.rx_buf[1] != ORCA_EXT_MTR_STREAM){
        AP_HAL::get_HAL().console->printf("Wrong function code, got %d\n", trans.rx_buf[1]);
        return false;
    }
   
	uint8_t* d = trans.rx_buf;

    int idx = 2;
	idx = parseint32(d, idx, (int32_t*)&_state.force_mN);
	idx = parseint32(d, idx, (int32_t*)&_state.position_um);
    idx = parseint32(d, idx, (int32_t*)&_state.speed_mm_s);
	idx = parseint32(d, idx, (int32_t*)&_state.acceleration_mm_s_2);
	idx = parseint16(d, idx, (int16_t*)&_state.board_temp_C);
	idx = parseint16(d, idx, (int16_t*)&_state.coil_temp_C);
	idx = parseint16(d, idx, (int16_t*)&_state.mode_of_operation);
	idx = parseint16(d, idx, (int16_t*)&_state.kin_status);
	idx = parseint16(d, idx, (int16_t*)&_state.errors);
	idx = parseint16(d, idx, (int16_t*)&_state.motor_status);
    idx = parseint16(d, idx, (int16_t*)&_state.register_read_1);
    idx = parseint16(d, idx, (int16_t*)&_state.register_read_2);

    return true;
}

void AP_ORCAMotor_Modbus::handle_stream() {

    TransactionState exit_state = SENDING;
    // if(!_rb_empty()) {
    //     _rb_read(&trans);
    //     trans.state = SENDING;
    //     return;
    // }
    
    switch (target_mode)
    {
    case MODE_SLEEP:
        //AP_HAL::get_HAL().console->printf("Stream target: SLEEP\n");
        enqueue_extended_motor_frame(EXT_MODE_SLEEP, 0, 0);
        break;
    case MODE_FORCE:
        enqueue_extended_motor_frame(EXT_MODE_FORCE, target_force, 0);
        break;
    case MODE_POSITION:
        //AP_HAL::get_HAL().console->printf("Stream target: POSITION\n");
        enqueue_extended_motor_frame(EXT_MODE_POSITION, target_position, 0);
        break;
    case MODE_HAPTIC:
    case MODE_KINEMATIC:
    case MODE_PWM:
    case MODE_AUTOZERO:
    default:
        target_mode = MODE_SLEEP;
        exit_state = IDLE;
        break;
    }

    trans.state = exit_state;
}

void AP_ORCAMotor_Modbus::transmit() {
    if(!motor_uart) return;
    // AP_HAL::get_HAL().console->printf("Transmitting:\n");
    // for(int i = 0; i < trans.tx_len; i++) {
    //     AP_HAL::get_HAL().console->printf("%x ", trans.tx_buf[i]);
    // }
    // AP_HAL::get_HAL().console->printf("\n");
    motor_uart->write(trans.tx_buf, trans.tx_len);
    motor_uart->flush();
    trans.state = RECEIVING;
    trans.rx_entry_ms = AP_HAL::millis();
}

void AP_ORCAMotor_Modbus::uart_poll() {
    trans.rx_len = 0;
    while (motor_uart->available() && ((AP_HAL::millis() - trans.rx_entry_ms) < ORCAMOTOR_RX_TIMEOUT_MS)) {
        uint8_t c;
        if(motor_uart->read(c)) {
            trans.rx_buf[trans.rx_len++] = c;
        }
    }
    // No response after 1s
    if((AP_HAL::millis() - trans.rx_entry_ms >= ORCAMOTOR_RX_TIMEOUT_MS) && trans.rx_len == 0) {
        //AP_HAL::get_HAL().console->printf("Receive timeout\n");
        trans.state = ERROR;
        return;
    }
    // Got response
    if(trans.rx_len > 0) {
        trans.state = PROCESSING;
        // AP_HAL::get_HAL().console->printf("Got %u bytes in response:\n", trans.rx_len);
        // for(int i = 0; i < trans.rx_len; i++) {
        //     AP_HAL::get_HAL().console->printf("%x ", trans.rx_buf[i]);
        // }
        // AP_HAL::get_HAL().console->printf("\n");
    }
    // Re-enters this function
}

void AP_ORCAMotor_Modbus::process_response() {
    if(bad_crc(trans.rx_buf, trans.rx_len)) {
        AP_HAL::get_HAL().console->printf("Received bad CRC!:\n");
        trans.state = ERROR;
        return;
    }

    TransactionState exit_state = IDLE;
    switch (trans.fn) {
        case MB_READ_SINGLE_REG:
            break;
        case MB_WRITE_SINGLE_REG:
            break;
        case MB_WRITE_MULTI_REG:
            break;
        case MB_DIAG_QUERY_DATA:
            if(!check_ping_response()) {
                exit_state = ERROR;
            }
            break;
        case ORCA_MNG_HS_STREAM:
            break;
        case ORCA_MTR_CMD_STREAM:
            break;
        case ORCA_EXT_MTR_STREAM:
            if(!check_ext_motor_frame_response()) {
                exit_state = ERROR;
            }
            break;
        case ORCA_MTR_READ_STREAM:
            break;
        case ORCA_MTR_WRITE_STREAM:
            break;    
        default:
            exit_state = ERROR;
            break;
    }
    trans.state = exit_state;
}

#endif