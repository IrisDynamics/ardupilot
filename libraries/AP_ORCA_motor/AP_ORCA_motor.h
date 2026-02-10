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
#define HIGH_SPEED_MOTOR_BAUD 1000000
#define EVEN 2


class AP_ORCA_motor{

public:
    AP_ORCA_motor();

    /// Do not allow copies
    CLASS_NO_COPY(AP_ORCA_motor);

    static const struct AP_Param::GroupInfo var_info[];

    static AP_ORCA_motor *get_singleton() {
        return _singleton;
    }

    /// periodically called from SRV_Channels::push()
    void update();
        /**
        initialize the device driver: configure serial port, wake-up and configure ESCs
    */
    void init();

    
    /**
        initialize the serial port
    */
    void init_uart(AP_HAL::UARTDriver *uart, const char *name);

	/**
	 * @brief Generates and returns a 16 bit CRC for a given message.
	 * 	      The return CRC already has the byte order swapped and is ready to be placed in a Modbus message.
	 *
	 * @param	message			Pointer to the message buffer to be used for CRC generation.
	 * @param message_len	 	Number of bytes in the message buffer.
	 */
	static uint16_t generate_crc(uint8_t *message, int message_len) {

		uint8_t crc_hi_byte = 0xFF;	// initialize crc bytes
		uint8_t crc_lo_byte = 0xFF; //
		int index = 0; // for indexing the crc tables

		while(message_len--) {

			// calculate crc
			index = crc_hi_byte ^ *message++;
			crc_hi_byte = crc_lo_byte ^ crc_hi_table[index];
			crc_lo_byte = crc_lo_table[index];

		}

		return (crc_hi_byte << 8 | crc_lo_byte);	// return crc result, with bytes swapped for modbus message

	}
     private:
    static AP_ORCA_motor *_singleton;
    AP_HAL::UARTDriver *_uart;




    enum class OrcaState : uint8_t{
     PINGING,
     CONFIG,
     SENDING,
     RECEIVING,
     IDLE
    };


};