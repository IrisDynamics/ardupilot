/*
  simple test of UART interfaces
 */

#include "AP_ORCA_motor/AP_ORCA_motor.h"

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL(); 
AP_ORCA_motor motor; 

void setup(void)
{
    /*
      start all UARTs at orca default with default buffer sizes
    */

    hal.scheduler->delay(1000); //Ensure that hal.serial(n) can be initialized    
    motor.init_uart(hal.serial(4), "ORCA_MOTOR");
}


static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uint8_t tx[8];

    tx[0] = 1;        // slave id
    tx[1] = 0x03;     // function
    tx[2] = 0x00;     // reg hi
    tx[3] = 0x00;     // reg lo
    tx[4] = 0x00;     // count hi
    tx[5] = 0x02;     // count lo

    uint16_t crc = motor.generate_crc(tx, 6);
    tx[6] = crc & 0xFF;
    tx[7] = crc >> 8;

    uart->write(tx, sizeof(tx));
    uart->flush();
    //uart->printf("Hello on UART %s at %.3f seconds\n",
    //            name, (double)(AP_HAL::millis() * 0.001f));
    //uint8_t bytes[] = {0x01, 0x03, 0x01, 0x52, 0x00, 0x01, 0x00, 0x00};
    // uint8_t tx_buffer[] = {0x01, 0x06, 0x00, 0x03, 0x00, 0x05, 0xB9, 0xC9};
    // //uint16_t generated_crc_bytes = generate_crc(bytes, 6);
    // //bytes[6] = (uint8_t)(generated_crc_bytes>>8);
    // //bytes[7] = (uint8_t)generated_crc_bytes;

    // for (int i=0; i<8; i++){
    //     uart->write(tx_buffer[i]);
    //     hal.scheduler->delay_microseconds(833);
    // }
    
}



void loop(void)
{

    //test_uart(hal.serial(0), "SERIAL0");
    // test_uart(hal.serial(1), "SERIAL1");
    // test_uart(hal.serial(2), "SERIAL2");
    // test_uart(hal.serial(3), "SERIAL3");
    test_uart(hal.serial(4), "SERIAL4");
   //test_uart(hal.serial(5), "SERIAL5");

        // also do a raw printf() on some platforms, which prints to the
        // debug console
    // ::printf("Hello on debug console at %.3f seconds\n", (double)(AP_HAL::millis() * 0.001f));

    hal.scheduler->delay(100);

}

AP_HAL_MAIN();
