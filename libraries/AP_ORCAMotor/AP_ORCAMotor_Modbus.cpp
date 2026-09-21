/* Table of CRC values for high–order byte */
#include "AP_ORCAMotor_Modbus.h"
#include <AP_SerialManager/AP_SerialManager.h>

#if HAL_ORCAMOTOR_ENABLED

void AP_ORCAMotor_Modbus::init() {
    const AP_SerialManager &serial_manager = AP::serialmanager();

    motor_uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ORCAMotor, _instance);

    if(motor_uart != nullptr) {
        motor_uart->begin(AP_SERIALMANAGER_ORCAMOTOR_BAUD, ORCAMOTOR_RX_BYTES, ORCAMOTOR_TX_BYTES);
        motor_uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        motor_uart->set_stop_bits(1);
        motor_uart->configure_parity(UART_PARITY_EVEN);
    }
}

bool AP_ORCAMotor_Modbus::healthy() {
    enqueue_ping_message();
    return check_ping_response();
}

void AP_ORCAMotor_Modbus::set_mode(MotorMode mode) {
    uint8_t tx[4] = {0x00, CTRL_REG_3, 0x00, mode};

    write(MB_WRITE_SINGLE_REG, tx, sizeof(tx));
}

void AP_ORCAMotor_Modbus::write(const uint8_t *message, const size_t len) {
    if(motor_uart == nullptr) {
        return;
    }

    uint16_t crc = generate_crc(message, len);
    uint8_t crc_bytes[2];
    crc_bytes[0] = crc & 0xFF;
    crc_bytes[1] = crc >> 8;

    motor_uart->write(message, len);
    motor_uart->write(crc_bytes, sizeof(crc_bytes));
    motor_uart->flush();
}

void AP_ORCAMotor_Modbus::write(const FunctionCode fn, const uint8_t const *data, const size_t data_len, const uint8_t const *sub_fn, const size_t sub_fn_len) {
    size_t len = 2 + data_len + sub_fn_len;
    uint8_t buf[len];
    size_t idx = 0;
    buf[idx++] = ORCA_SLAVE_ID;
    buf[idx++] = fn;
    if(data && data_len) {
        memcpy(&buf[idx], data, data_len);
        idx += data_len;
    }
    if(sub_fn && sub_fn_len) {
        memcpy(&buf[idx], sub_fn, sub_fn_len);
    }
    return write(buf, len);
}

void AP_ORCAMotor_Modbus::enqueue_ping_message(){
    uint8_t diag_fn[2] = {0, 0};
    write(MB_DIAG_QUERY_DATA, nullptr, 0, diag_fn, sizeof(diag_fn));
}

void AP_ORCAMotor_Modbus::enqueue_extended_motor_frame(uint32_t position_um, uint16_t read_address){
    uint8_t tx[EXT_MOTOR_FRAME_TX_LEN] = {
        EXT_MODE_POSITION, 
        (uint8_t)(position_um>>24),
        (uint8_t)(position_um>>16),
        (uint8_t)(position_um>>8),
        (uint8_t)(position_um),
        (uint8_t)(read_address>>8),
        (uint8_t)read_address
    };
    return write(ORCA_EXT_MTR_STREAM, tx, sizeof(tx));
}

bool AP_ORCAMotor_Modbus::check_ping_response(){
    if (rx_buffer_count() < PING_RESPONSE_RX_LEN) {
        //serial_display_uart->printf("not enough bytes: %d", rx_buffer_count());
        return false;
    }
    const uint8_t expected_response[6] = {0x01, 0x08, 0x00, 0x00, 0x80, 0x1A };                    
    uint8_t transaction_frame[PING_RESPONSE_RX_LEN];

    for (uint8_t i = 0; i < PING_RESPONSE_RX_LEN; i++) {
        transaction_frame[i] = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % ORCAMOTOR_RX_BYTES;
        if (transaction_frame[i] != expected_response[i]){
            //serial_display_uart->printf("byte mismatch: %c, %c", transaction_frame[i], ping_message[i]);
            return false;
        }
    }
    return true;
}

bool AP_ORCAMotor_Modbus::check_motor_frame_response()
{
    if (rx_buffer_count() < EXT_MOTOR_FRAME_RX_LEN){
        //serial_display_uart->printf("not enough bytes: %d", rx_buffer_count());
        return false;
    }

    uint8_t transaction_frame[EXT_MOTOR_FRAME_RX_LEN];
    for (uint8_t i = 0; i < EXT_MOTOR_FRAME_RX_LEN; i++) {
        transaction_frame[i] = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % ORCAMOTOR_RX_BYTES;
    }

    if (transaction_frame[0] != ORCA_SLAVE_ID){
        return false;
    }
    if (transaction_frame[1] != ORCA_EXT_MTR_STREAM){
        return false;
    }
    if (bad_crc(transaction_frame, sizeof(transaction_frame))){
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

    return true;
}

uint16_t AP_ORCAMotor_Modbus::rx_buffer_count()
{
    if (rx_head >= rx_tail)
        return rx_head - rx_tail;

    return ORCAMOTOR_RX_BYTES - rx_tail + rx_head;
}
#endif