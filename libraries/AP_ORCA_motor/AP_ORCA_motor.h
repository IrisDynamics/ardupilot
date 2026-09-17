
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
     Test for AP_ORCA_motor
*/
#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include "crc_table.h"

#define DEFAULT_MOTOR_BAUD 19200
#define HIGH_SPEED_MOTOR_BAUD 460800
#define EVEN 2


class AP_ORCA_motor{

public:
    AP_ORCA_motor();

    /// Do not allow copies
    CLASS_NO_COPY(AP_ORCA_motor);

   // static const struct AP_Param::GroupInfo var_info[];

    // static AP_ORCA_motor *get_singleton() {
    //     return _singleton;
    // }

    /// periodically called from SRV_Channels::push()
    struct MotorData{
        int32_t force_mN;
        int32_t position_um;
        int32_t speed_mm_s;
        int32_t acceleration_mm_s_2;
        int16_t board_temp_C;
        int16_t coil_temp_C;
        int16_t mode_of_operation;
        int16_t errors;
        int16_t power_W;
        int16_t voltage_V;

    };

    enum ExtMotorCommandMode {
		NoModeChange = 0,
		SleepMode 	 = 1,
		ForceMode 	 = 2,
		PosMode 	 = 3,
		KinMode 	 = 5
	};

    void update();


    void set_target_position(int32_t _target_position){
        target_position = _target_position;
    }

    void enqueue_ping_message();
    void enqueue_extended_motor_frame(uint32_t position_um, uint16_t read_address);
    
    /**
        initialize the serial port
    */
    void init(AP_HAL::UARTDriver *uart);

	/**
	 * @brief Generates and returns a 16 bit CRC for a given message.
	 * 	      The return CRC already has the byte order swapped and is ready to be placed in a Modbus message.
	 *
	 * @param	message			Pointer to the message buffer to be used for CRC generation.
	 * @param message_len	 	Number of bytes in the message buffer.
	 */
	static uint16_t generate_crc(uint8_t *message, int message_len) {

        uint16_t crc = 0xFFFF; // Initial value
        for (int i = 0; i < message_len; i++) {
            crc ^= (uint16_t)message[i]; // XOR byte into LSB of CRC
            for (int j = 0; j < 8; j++) {
                if (crc & 0x0001) { // If LSB is set
                    crc >>= 1;
                    crc ^= 0xA001; // Polynomial for Modbus
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
	}
    bool check_ping_response();
    bool check_motor_frame_response();


    //void AP_ORCA_motor::process_byte(uint8_t byte);

     private:
     enum class OrcaState : uint8_t{
        PINGING,
        PING_WAIT,
        CONFIG,
        SENDING,
        RECEIVING,
        IDLE
    };
    //static AP_ORCA_motor *_singleton;
    AP_HAL::UARTDriver *motor_uart;
    AP_HAL::UARTDriver *serial_display_uart;
    OrcaState current_state = OrcaState::IDLE;
    uint32_t state_start_time;
    uint8_t slave_id = 1;
    uint8_t ping_message[6] = {0x01, 0x08, 0x00,0x00,0x80, 0x1A}; //diagnostic queury expected echo response
    uint8_t extended_motor_frame_function_code = 0x66;           
    uint8_t position_mode = 3;
    int32_t target_position = 0;
    MotorData motor_data;

    #define RX_BUFFER_SIZE 256
    const uint8_t ping_response_length = 6;
    const uint8_t extended_motor_frame_tx_length = 11;
    const uint8_t extended_motor_frame_rx_length = 42;
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    volatile uint16_t rx_head = 0, rx_tail = 0;

    void uart_poll(AP_HAL::UARTDriver* uart) {
        while (uart->available()) {

            uint8_t b = uart->read();
            uint16_t next = (rx_head + 1) % RX_BUFFER_SIZE;

            if (next != rx_tail) {      // prevent overflow
                rx_buffer[rx_head] = b;
                rx_head = next;
            }
        }
    }
    uint16_t rx_buffer_count();

    bool bad_crc(uint8_t* rx_data, uint16_t rx_message_length);

    inline int parseint32(uint8_t* data, int start_index, int32_t* value)
	{
		*value = (data[start_index]    << 24)
			  | (data[start_index + 1] << 16)
			  | (data[start_index + 2] << 8)
			  |  data[start_index + 3];
		return start_index + 4;
	}

	inline int parseint16(uint8_t* data, int start_index, int16_t* value)
	{
		*value = (data[start_index]     << 8)
			   | data[start_index + 1];
		return start_index + 2;
	}

};

// uint8_t tx_buf[64];
// size_t tx_len = build_message(tx_buf, sizeof(tx_buf));  // fill buffer
// uart.write(tx_buf, tx_len);                             // send in one go

