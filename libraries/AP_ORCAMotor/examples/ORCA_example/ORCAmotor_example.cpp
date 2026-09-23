/*
  simple test of UART interfaces
 */

#include <AP_HAL/AP_HAL.h>                                      //This is a common Hardware Abstraction Layer.
#include "AP_ORCAMotor/AP_ORCAMotor.h"
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Notify/AP_BoardLED.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <SITL/SITL.h>
#include <AP_Scheduler/AP_Scheduler.h>

#include <stdio.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL(); 

static AP_BoardConfig board_config;

#if AP_NOTIFY_GPIO_LED_3_ENABLED
// create board led object
AP_BoardLED board_led;
#endif

// create fake gcs object
GCS_Dummy _gcs;                                                 //gcs stands for Ground Control Station

#if AP_SIM_ENABLED
SITL::SIM sitl;
AP_Baro baro;
AP_Scheduler scheduler;
#endif

#if AP_RTC_ENABLED
AP_RTC rtc;
#endif

#if HAL_LOGGING_ENABLED
AP_Logger logger;
#endif


static AP_SerialManager serial_manager;
static AP_ORCAMotor motor;

static uint32_t last_log = 0;

void setup(void)
{
    /*
      start all UARTs at orca default with default buffer sizes
    */
    hal.console->printf("ORCAMotor Test\n");

#if AP_SIM_ENABLED
    sitl.init();
#endif  // AP_SIM_ENABLED

    board_config.init();

#if AP_NOTIFY_GPIO_LED_3_ENABLED
    // Initialise the leds
    board_led.init();
#endif

    hal.console->printf("Serial manager init\n");
    serial_manager.init();
    hal.console->printf("Motor init\n");
    motor.init();
    motor.set_mode(MODE_SLEEP);
}


// static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
// {
//     if (uart == nullptr) {
//         // that UART doesn't exist on this platform
//         return;
//     }
//     uint8_t tx[8];

//     tx[0] = 1;        // slave id
//     tx[1] = 0x03;     // function
//     tx[2] = 0x00;     // reg hi
//     tx[3] = 0x00;     // reg lo
//     tx[4] = 0x00;     // count hi
//     tx[5] = 0x02;     // count lo

//     uint16_t crc = motor.generate_crc(tx, 6);
//     tx[6] = crc & 0xFF;
//     tx[7] = crc >> 8;

//     uart->write(tx, sizeof(tx));
//     uart->flush();
//     //uart->printf("Hello on UART %s at %.3f seconds\n",
//     //            name, (double)(AP_HAL::millis() * 0.001f));
//     //uint8_t bytes[] = {0x01, 0x03, 0x01, 0x52, 0x00, 0x01, 0x00, 0x00};
//     // uint8_t tx_buffer[] = {0x01, 0x06, 0x00, 0x03, 0x00, 0x05, 0xB9, 0xC9};
//     // //uint16_t generated_crc_bytes = generate_crc(bytes, 6);
//     // //bytes[6] = (uint8_t)(generated_crc_bytes>>8);
//     // //bytes[7] = (uint8_t)generated_crc_bytes;

//     // for (int i=0; i<8; i++){
//     //     uart->write(tx_buffer[i]);
//     //     hal.scheduler->delay_microseconds(833);
//     // }
    
// }



void loop(void)
{
    motor.update();
    if(AP_HAL::millis() - last_log > 5000) {
        last_log = AP_HAL::millis();
        hal.console->printf("MotorData:\nPos: %ld um\nForce: %ld mN\n", motor._state[0].position_um, motor._state[0].force_mN);
    }
    hal.scheduler->delay(50);
}

AP_HAL_MAIN();
