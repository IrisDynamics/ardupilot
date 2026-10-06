#pragma once

#include "AP_ORCAMotor_config.h"

#if HAL_ORCAMOTOR_ENABLED
#include "AP_ORCAMotor_Backend.h"

#define ORCAMOTOR_BAUD 19200
#define ORCAMOTOR_RX_BYTES 256
#define ORCAMOTOR_TX_BYTES 32
#define ORCAMOTOR_RX_TIMEOUT_MS 1000
#define UART_PARITY_EVEN 2
#define ORCA_SLAVE_ID 0x01

// Specified TX lengths DO NOT include slave id, crc, or sub function codes
#define EXT_MOTOR_FRAME_TX_LEN 7
#define EXT_MOTOR_FRAME_RX_LEN 42
#define MULTI_REG_WRITE_RX_LEN 8
#define SINGLE_REG_WRITE_RX_LEN 8
#define PING_RESPONSE_RX_LEN 6
#define TRANS_QUEUE_SIZE 16

#define AUTO_ZERO_TIMEOUT_MS 3000 // This should be increased for high shaft lengths

/* Table of CRC values for high–order byte */
static constexpr uint8_t crc_hi_table[256] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
};

/* Table of CRC values for low–order byte */
static constexpr uint8_t crc_lo_table[256] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};

class AP_ORCAMotor_Modbus : public AP_ORCAMotor_Backend
{
public:
    using AP_ORCAMotor_Backend::AP_ORCAMotor_Backend;

    void init() override;
    bool healthy() override;
    void clear_motor_errors() override;
    void startup_config() override;

private:
    enum TransactionState {
        IDLE,
        SENDING,
        RECEIVING,
        PROCESSING,
        ERROR
    };

    enum FunctionCode {
        NONE = 0, // Don't use
        MB_READ_SINGLE_REG = 0x03,
        MB_WRITE_SINGLE_REG = 0x06,
        MB_WRITE_MULTI_REG = 0x10,
        MB_DIAG_QUERY_DATA = 0x08,
        ORCA_MNG_HS_STREAM = 0x41,
        ORCA_MTR_CMD_STREAM = 0x64,
        ORCA_EXT_MTR_STREAM = 0x66,
        ORCA_MTR_READ_STREAM = 0x68,
        ORCA_MTR_WRITE_STREAM = 0x69
    };

    enum ExtMtrCmdMode {
        EXT_MODE_NO_CHANGE = 0x00,
        EXT_MODE_SLEEP = 0x01,
        EXT_MODE_FORCE = 0x02,
        EXT_MODE_POSITION = 0x03,
        EXT_MODE_KINEMATIC = 0x05
    };

    enum RegisterMap {
        CTRL_REG_0 = 0,
        CTRL_REG_1 = 1,
        CTRL_REG_2 = 2,
        CTRL_REG_3 = 3,
        CTRL_REG_4 = 4,
        USER_MAX_FORCE = 140,
        USER_MAX_FORCE_H = 141,
        PC_PGAIN = 133,
        PC_IGAIN = 134,
        PC_DVGAIN = 135,
        PC_SOFTSTART_PERIOD = 150,
        POS_MAX_VEL = 153,
        POS_MAX_ACCEL = 154,
        POS_MAX_DECEL = 155,
        ZERO_MODE = 171,
        AUTO_ZERO_FORCE_N = 172,
        AUTO_ZERO_EXIT_MODE = 173,
        AUTO_ZERO_SPEED_MMPS = 177,
        MOTOR_STATUS = 321
    };

    enum CtrlReg0Fn {
        CR0_RESET = 1,
        CR0_CLEAR_ERR = 2,
        CR0_ZERO_POS = 4,
        CR0_INVERT_POS = 8
    };

    enum MotorStatusBits {
        AUTO_ZERO_COMPLETE = (1 << 0),
        AUTO_ZERO_RUNNING = (1 << 2),
        POSITION_MODE_MOVING = (1 << 3)
    };

    struct Transaction {
        uint8_t tx_buf[ORCAMOTOR_TX_BYTES];
        uint8_t tx_len;
        uint8_t rx_buf[ORCAMOTOR_RX_BYTES];
        uint8_t rx_len;
        uint32_t rx_entry_ms;
        TransactionState state;
        FunctionCode fn;
    };

    struct Transmission {
        uint8_t tx_buf[ORCAMOTOR_TX_BYTES];
        uint8_t tx_len;
        FunctionCode fn;
    };

    struct TransactionQueue {
        Transmission buffer[TRANS_QUEUE_SIZE];
        size_t head;
        size_t tail;
    };

    void thread_main();
    bool init_internals();

    // Functionality wrappers
    void send_ping_message();
    void send_extended_motor_frame(ExtMtrCmdMode mode, uint32_t data, uint16_t read_address);
    void handle_auto_zero();

    // Modbus response handlers
    bool check_ping_response();
    bool check_ext_motor_frame_response();
    bool check_multi_reg_write_response();
    bool check_single_reg_write_response();

    // Main transaction state machine
    void handle_stream();
    void uart_poll();
    void transmit();
    void process_response();

    // Builds modbus commands and places them in transaction buffer
    void write(const FunctionCode fn, const uint8_t* const data = nullptr, const size_t data_len = 0, const uint8_t* const sub_fn = nullptr, const size_t sub_fn_len = 0, bool queued = false);
    void writeQueued(const FunctionCode fn, const uint8_t* const data = nullptr, const size_t data_len = 0, const uint8_t* const sub_fn = nullptr, const size_t sub_fn_len = 0);
    void writeSingleReg(const RegisterMap reg, const int16_t val, bool queued = true);
    void writeMultiReg(const RegisterMap reg, const int16_t* const val, const size_t len, bool queued = true);
    void readRegister(const RegisterMap reg, const size_t len);

    // Helper functions for processing;
    inline int parseint32(uint8_t* data, int start_index, int32_t* value)
    {
        if(data == nullptr || value == nullptr){
            return start_index;
        }
        *value = (data[start_index]    << 24)
                 | (data[start_index + 1] << 16)
                 | (data[start_index + 2] << 8)
                 |  data[start_index + 3];
        return start_index + 4;
    }
    inline int parseint16(uint8_t* data, int start_index, int16_t* value)
    {
        if(data == nullptr || value == nullptr){
            return start_index;
        }
        *value = (data[start_index]     << 8)
                 | data[start_index + 1];
        return start_index + 2;
    }
    inline uint16_t generate_crc(const uint8_t *tx_message, size_t tx_message_len)
    {
        if(tx_message == nullptr){
            return 0;
        }
        uint8_t crc_hi_byte = 0xFF;	// initialize crc bytes
        uint8_t crc_lo_byte = 0xFF; //
        int index = 0; // for indexing the crc tables

        while (tx_message_len--) {
            index = crc_hi_byte ^ *tx_message++;
            crc_hi_byte = crc_lo_byte ^ crc_hi_table[index];
            crc_lo_byte = crc_lo_table[index];
        }
        return (crc_hi_byte << 8 | crc_lo_byte);	// return crc result, with bytes swapped for modbus message
    }
    inline bool bad_crc(uint8_t* rx_data, uint16_t rx_frame_len)
    {
        if(rx_data == nullptr){
            return true;
        }
        uint16_t crc = generate_crc(rx_data, rx_frame_len-2);
        if ((rx_data[rx_frame_len-2] != (crc & 0xFF)) | (rx_data[rx_frame_len-1] != (crc >> 8))) {
            return true;
        }
        return false;
    }
    inline bool bad_response_header(FunctionCode fn, uint8_t expected_len)
    {
        if (trans.rx_len < expected_len) {
            //AP_HAL::get_HAL().console->printf("Not enough bytes: %d\n", trans.rx_len);
            return true;
        }
        if (trans.rx_buf[0] != ORCA_SLAVE_ID) {
            //AP_HAL::get_HAL().console->printf("Wrong slave ID count, got %d\n", trans.rx_buf[0]);
            return true;
        }
        if (trans.rx_buf[1] != fn) {
            //AP_HAL::get_HAL().console->printf("Wrong function code, got %d\n", trans.rx_buf[1]);
            return true;
        }
        return false;
    }
    inline void int32_to_arr_BE(int32_t v, int16_t b[2])
    {
        if(b == nullptr) {
            return;
        }
        b[0] = (int16_t)(v >> 16);
        b[1] = (int16_t)(v & 0xFFFF);
    }
    inline void int32_to_arr_LE(int32_t v, int16_t b[2])
    {
        if(b == nullptr) {
            return;
        }
        b[0] = (int16_t)(v & 0xFFFF);
        b[1] = (int16_t)(v >> 16);
    }

    // Control functions for using transaction ring buffer
    inline void _rb_init()
    {
        _trans.head = 0;
        _trans.tail = 0;
    }
    inline bool _rb_empty()
    {
        return _trans.head == _trans.tail;
    }
    inline bool _rb_full()
    {
        return (_trans.head + 1) % TRANS_QUEUE_SIZE == _trans.tail;
    }
    inline bool _rb_write(Transmission* t)
    {
        if(t == nullptr) {
            return false;
        }
        if (!_queue_mutex.take_nonblocking()) {
            return false;
        }
        if (_rb_full()) {
            _queue_mutex.give();
            return false;
        }
        memcpy(&_trans.buffer[_trans.head], t, sizeof(Transmission));
        _trans.head = (_trans.head + 1) % TRANS_QUEUE_SIZE;
        _queue_mutex.give();
        return true;
    }
    inline bool _rb_read(Transaction* t)
    {
        if(t == nullptr) {
            return false;
        }
        _queue_mutex.take_blocking();
        if (_rb_empty()) {
            _queue_mutex.give();
            return false;
        }
        memcpy(t->tx_buf, &_trans.buffer[_trans.tail].tx_buf, sizeof(Transmission::tx_buf));
        t->tx_len = _trans.buffer[_trans.tail].tx_len;
        t->fn = _trans.buffer[_trans.tail].fn;
        _trans.tail = (_trans.tail + 1) % TRANS_QUEUE_SIZE;
        _queue_mutex.give();
        return true;
    }

    AP_HAL::UARTDriver *motor_uart;     // Pointer to the assigned UART driver for this instance
    Transaction trans = {0};            // The current transaction frame being used by the thread
    TransactionQueue _trans = {0};      // The queue of messages to be transmitted, coming from outside the thread
    bool auto_zero_running = false;     // Whether auto zero routine is currently running
    uint32_t auto_zero_start_ms = 0;    // Start timestamp of auto zero routine
    bool _initialised = false;          // Whether or not thread main has been intialized for this instance
    HAL_Semaphore _queue_mutex;         // Mutex for the transmission queue, so reading and writing do not overlap
    HAL_Semaphore _last_healthy_sem;    // Mutex for the health check, protecting _last_received_ms and _last_send_ms
    uint32_t _last_received_ms = 0;     // Last message reception from motor
    uint32_t _last_send_ms = 0;         // Last transmission to motor
};
#endif