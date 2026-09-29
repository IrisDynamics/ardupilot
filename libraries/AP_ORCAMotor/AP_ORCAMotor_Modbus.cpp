/* Table of CRC values for high–order byte */
#include "AP_ORCAMotor_Modbus.h"

#if HAL_ORCAMOTOR_ENABLED
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/AP_HAL.h> 

extern const AP_HAL::HAL& hal;

void AP_ORCAMotor_Modbus::init() {
    if (_initialised) {
        return;
    }

    // create background thread to process serial input and output
    char thread_name[15];
    hal.util->snprintf(thread_name, sizeof(thread_name), "orcamotor%u", (unsigned)_instance);
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ORCAMotor_Modbus::thread_main, void), thread_name, 2048, AP_HAL::Scheduler::PRIORITY_RCOUT, 1)) {
        return;
    }

    clear_motor_errors();
    startup_config();
}

bool AP_ORCAMotor_Modbus::init_internals() {
    const AP_SerialManager &serial_manager = AP::serialmanager();
    motor_uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ORCAMotor, _instance);

    //motor_uart = AP_HAL::get_HAL().serial(4);

    if(motor_uart != nullptr) {
        motor_uart->begin(AP_SERIALMANAGER_ORCAMOTOR_BAUD, ORCAMOTOR_RX_BYTES, ORCAMOTOR_TX_BYTES);
        motor_uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        motor_uart->set_stop_bits(1);
        motor_uart->configure_parity(UART_PARITY_EVEN);
        return true;
    }
    return false;   
}

void AP_ORCAMotor_Modbus::clear_motor_errors() {
    writeSingleReg(CTRL_REG_0, CR0_CLEAR_ERR);
    set_mode(MODE_SLEEP);
}

void AP_ORCAMotor_Modbus::startup_config() {
    int16_t force[2];
    int32_to_arr_LE(_params.force_saturation.get(), force);
    writeMultiReg(USER_MAX_FORCE, force, 2);
    writeSingleReg(PC_PGAIN, _params.p_gain_pid.get());
    writeSingleReg(PC_IGAIN, _params.i_gain_pid.get());
    writeSingleReg(PC_DVGAIN, _params.d_gain_pid.get());
    writeSingleReg(POS_MAX_VEL, _params.speed_limit.get());
    writeSingleReg(POS_MAX_ACCEL, _params.accel_limit.get());
    writeSingleReg(POS_MAX_DECEL, _params.decel_limit.get());
    writeSingleReg(PC_SOFTSTART_PERIOD, _params.softstart_duration.get());
    //TODO: read CTRL-REG for INVERT POS status
    writeSingleReg(ZERO_MODE, _params.autozero_mode.get());
    writeSingleReg(AUTO_ZERO_FORCE_N, _params.autozero_force.get());
    writeSingleReg(AUTO_ZERO_SPEED_MMPS, _params.autozero_speed.get());
    writeSingleReg(AUTO_ZERO_EXIT_MODE, _params.autozero_exit_mode.get());    
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
                trans.state = IDLE;
                break;
            default:
                break;
        }
        hal.scheduler->delay(50);
    }
}

bool AP_ORCAMotor_Modbus::healthy() {
    if(!_initialised) {
        return false;
    }
    WITH_SEMAPHORE(_last_healthy_sem);
    const uint32_t now_ms = AP_HAL::millis();
    return ((now_ms - _last_received_ms < 3000) && (now_ms - _last_send_ms < 3000));
}

void AP_ORCAMotor_Modbus::writeQueued(const FunctionCode fn, const uint8_t* const data, const size_t data_len, const uint8_t* const sub_fn, const size_t sub_fn_len) {
    return write(fn, data, data_len, sub_fn, sub_fn_len, true);
}

void AP_ORCAMotor_Modbus::writeSingleReg(const RegisterMap reg, const int16_t val, bool queued) {
    uint8_t _reg[2] = {
        (uint8_t)(reg >> 8),
        (uint8_t)(reg & 0xFF)
    };
    uint8_t _val[2] = {
        (uint8_t)(val >> 8),
        (uint8_t)(val & 0xFF)
    };
    return queued ? writeQueued(MB_WRITE_SINGLE_REG, _val, 2, _reg, 2) : write(MB_WRITE_SINGLE_REG, _val, 2, _reg, 2);
}

void AP_ORCAMotor_Modbus::writeMultiReg(const RegisterMap reg, const int16_t* const val, const size_t len, bool queued) {
    //Convert 16 bit reg into array of {REG_HIGH, REG_LOW, NUM_REGS_HIGH, NUM_REGS_LOW, NUM_BYTES}
    const uint8_t header_len = 5;
    const uint8_t num_bytes = 2*len;
    uint8_t _reg[header_len] = { 
        (uint8_t)(reg >> 8),
        (uint8_t)(reg & 0xFF), 
        (uint8_t)(len >> 8),
        (uint8_t)(len & 0xFF),
        num_bytes
    };
    uint8_t _val[num_bytes];
    for(int i = 0; i < len; i++) {
        _val[2*i] = (uint8_t)((val[i] >> 8) & 0xFF);
        _val[2*i+1] = (uint8_t)(val[i] & 0xFF);
    }
    return queued ? writeQueued(MB_WRITE_MULTI_REG, _val, num_bytes, _reg, header_len) : write(MB_WRITE_MULTI_REG, _val, num_bytes, _reg, header_len);
}

void AP_ORCAMotor_Modbus::readRegister(const RegisterMap reg, const size_t len) {
    const uint8_t header_len = 4;
    uint8_t header[header_len] = {
        (uint8_t)(reg >> 8),
        (uint8_t)(reg & 0xFF),
        (uint8_t)(len >> 8),
        (uint8_t)(len & 0xFF)
    };
    return write(MB_READ_SINGLE_REG, header, header_len); 
}

// Sub-functions are specified for some ORCA specific packets
void AP_ORCAMotor_Modbus::write(const FunctionCode fn, const uint8_t* const data, const size_t data_len, const uint8_t* const sub_fn, const size_t sub_fn_len, bool queued) {
    // Check message sizing
    size_t len = 2 + data_len + sub_fn_len + 2; //Slave ID, Fn code, and 2 bytes for CRC
    if(len > ORCAMOTOR_TX_BYTES) {
        AP_HAL::get_HAL().console->printf("Message too long\n");
        return;
    }
    // Allocate buffer and initial index
    uint8_t* p_buf;
    size_t idx = 0;
    Transmission t;

    if(queued) {
        p_buf = t.tx_buf;
        t.tx_len = len;
        t.fn = fn;
    } else {
        p_buf = trans.tx_buf;
        trans.tx_len = len;
        trans.fn = fn;
    }
    
    // Build message byte-by-byte 
    p_buf[idx++] = ORCA_SLAVE_ID;
    p_buf[idx++] = fn;
    if(sub_fn != nullptr && sub_fn_len) {
        memcpy(&p_buf[idx], sub_fn, sub_fn_len);
        idx += sub_fn_len;
    }
    if(data != nullptr && data_len) {
        memcpy(&p_buf[idx], data, data_len);
        idx += data_len;
    }
    // Calculate and add CRC
    uint16_t crc = generate_crc(p_buf, len-2);
    uint8_t crc_bytes[2];
    crc_bytes[0] = crc & 0xFF;
    crc_bytes[1] = crc >> 8;
    p_buf[idx++] = crc_bytes[0];
    p_buf[idx++] = crc_bytes[1];

    if(queued) {
        if(!_rb_write(&t)) {
            AP_HAL::get_HAL().console->printf("Failed to write transmission to queue\n");
        }
    } 

    //AP_HAL::get_HAL().console->printf("Loaded transmit buffer\n");
}

void AP_ORCAMotor_Modbus::send_ping_message() {
    uint8_t diag_fn[2] = {0, 0};
    write(MB_DIAG_QUERY_DATA, nullptr, 0, diag_fn, sizeof(diag_fn));
}

void AP_ORCAMotor_Modbus::send_extended_motor_frame(ExtMtrCmdMode mode, uint32_t data, uint16_t read_address) {
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

void AP_ORCAMotor_Modbus::handle_auto_zero() {
    if(!auto_zero_running) {
        writeSingleReg(CTRL_REG_3, MODE_AUTOZERO, false);
        auto_zero_running = true;
        auto_zero_start_ms = AP_HAL::millis();
        return;
    }
    
    readRegister(MOTOR_STATUS, 1);
    if ((AP_HAL::millis() - auto_zero_start_ms) > AUTO_ZERO_TIMEOUT_MS) {
        AP_HAL::get_HAL().console->printf("Auto-Zero timed out\n");
        //TODO: GCS err msg
        target_mode = MODE_SLEEP;
        auto_zero_running = false;
        return;
    }
    if(!trans.rx_len) {
        return;
    }
    if((trans.rx_buf[4] & (uint8_t)AUTO_ZERO_COMPLETE) == (uint8_t)AUTO_ZERO_COMPLETE) {
        target_mode = (MotorMode)_params.autozero_exit_mode.get();
        auto_zero_running = false;
    } 
}

bool AP_ORCAMotor_Modbus::check_ping_response() {
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

bool AP_ORCAMotor_Modbus::check_ext_motor_frame_response() {
    if (bad_response_header(ORCA_EXT_MTR_STREAM, EXT_MOTOR_FRAME_RX_LEN)) {
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

//TODO: Combine these two fns
bool AP_ORCAMotor_Modbus::check_single_reg_write_response() {
    if (bad_response_header(MB_WRITE_SINGLE_REG, SINGLE_REG_WRITE_RX_LEN)) {
        return false;
    }
    if(memcmp(&trans.rx_buf[2], &trans.tx_buf[2], sizeof(uint16_t)) != 0) {
        AP_HAL::get_HAL().console->printf("Wrong initial address, got %2x%2x\n", trans.rx_buf[2], trans.rx_buf[3]);
        return false;
    }
    if(memcmp(&trans.rx_buf[4], &trans.tx_buf[4], sizeof(uint16_t)) != 0) {
        AP_HAL::get_HAL().console->printf("Wrong reg value, got %2x%2x\n", trans.rx_buf[4], trans.rx_buf[5]);
        return false;
    }
    return true;
}
// Device reponds with {slave_id, starting address, num_regs, crc}
bool AP_ORCAMotor_Modbus::check_multi_reg_write_response() {
    if (bad_response_header(MB_WRITE_MULTI_REG, MULTI_REG_WRITE_RX_LEN)) {
        return false;
    }
    if(memcmp(&trans.rx_buf[2], &trans.tx_buf[2], sizeof(uint16_t)) != 0) {
        AP_HAL::get_HAL().console->printf("Wrong initial address, got %2x%2x\n", trans.rx_buf[2], trans.rx_buf[3]);
        return false;
    }
    if(memcmp(&trans.rx_buf[4], &trans.tx_buf[4], sizeof(uint16_t)) != 0) {
        AP_HAL::get_HAL().console->printf("Wrong number of regs, got %2x%2x\n", trans.rx_buf[4], trans.rx_buf[5]);
        return false;
    }
    return true;
}

void AP_ORCAMotor_Modbus::handle_stream() {
    // Check if we have queued commands from outside the UART thread
    if(!_rb_empty()) {
        _rb_read(&trans);
        AP_HAL::get_HAL().console->printf("Transmitting queued message:\n");
        for(int i = 0; i < trans.tx_len; i++) {
            AP_HAL::get_HAL().console->printf("%02x ", trans.tx_buf[i]);
        }
        AP_HAL::get_HAL().console->printf("\n");
        //AP_HAL::get_HAL().console->printf("Read transaction\nLen: %u\n", trans.tx_len);
        trans.state = SENDING;
        return;
    }
    // Otherwise manage the motor based on target mode, force, position etc.
    switch (target_mode) {
        case MODE_SLEEP:
            //AP_HAL::get_HAL().console->printf("Stream target: SLEEP\n");
            send_extended_motor_frame(EXT_MODE_SLEEP, 0, 0);
            break;
        case MODE_FORCE:
            send_extended_motor_frame(EXT_MODE_FORCE, target_force, 0);
            break;
        case MODE_POSITION:
            //AP_HAL::get_HAL().console->printf("Stream target: POSITION\n");
            send_extended_motor_frame(EXT_MODE_POSITION, target_position, 0);
            break;
        case MODE_AUTOZERO:
            handle_auto_zero();
            break;
        case MODE_HAPTIC:
        case MODE_KINEMATIC:
        case MODE_PWM:
        default:
            target_mode = MODE_SLEEP;
            trans.state = IDLE;
            return;
    }

    trans.state = SENDING;
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
    WITH_SEMAPHORE(_last_healthy_sem);
    _last_send_ms = AP_HAL::millis();
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
        WITH_SEMAPHORE(_last_healthy_sem);
        _last_received_ms = AP_HAL::millis();
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
        AP_HAL::get_HAL().console->printf("Received bad CRC!\n");
        trans.state = ERROR;
        return;
    }

    TransactionState exit_state = IDLE;
    switch (trans.fn) {
        case MB_READ_SINGLE_REG:
            break;
        case MB_WRITE_SINGLE_REG:
            if(!check_single_reg_write_response()) {
                exit_state = ERROR;
            }
            break;
        case MB_WRITE_MULTI_REG:
            if(!check_multi_reg_write_response()) {
                exit_state = ERROR;
            }
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