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

char serial_buf[128];
enum SerialCommand {
    NONE,
    MODE,
    POSITION,
    FORCE
};

void read_input() {
    uint8_t i = 0;
    while(hal.console->available()) {
        uint8_t c;
        if(hal.console->read(c)) {
            serial_buf[i++] = (char)c;
        }
        if((char)c == '\n') {
            serial_buf[i] = '\0';
            break;
        }
    }
    
    if(i == 0) return;
    hal.console->printf("Received: %s", serial_buf);
    SerialCommand command = NONE;
    long value;

    if (strstr(serial_buf, "mode:")) {
        command = MODE;
    } else if (strstr(serial_buf, "force:")) {
        command = FORCE;
    } else if(strstr(serial_buf, "pos:")) {
        command = POSITION;
    }

    char* tok = strtok(serial_buf, ":");
    if(tok != NULL) {
        tok = strtok(NULL, ":");
    }
    if(tok != NULL) {
        value = atoi(tok);
    }
    
    switch (command)
    {
    case MODE:
        motor.set_mode((MotorMode)value);
        hal.console->printf("Setting mode: %ld\n", value);
        break;
    case FORCE:
        motor.set_target_force_mN(value);
        hal.console->printf("Setting force target: %ld mN\n", value);
        break;
    case POSITION:
        motor.set_target_position_um(value);
        hal.console->printf("Setting position target: %ld um\n", value);
        break;
    default:
        break;
    }
}

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
    motor.set_target_position_um(20000);
}

void loop(void)
{
    motor.update();
    if(AP_HAL::millis() - last_log > 5000) {
        last_log = AP_HAL::millis();
        ExtMotorData* m = &motor._state[0];
        hal.console->printf("MotorData:\nPos: %ld um\nForce: %ld mN\nSpd: %ld mm/s\nAccel: %ld mm/s2\n\n", m->position_um, m->force_mN, m->speed_mm_s, m->acceleration_mm_s_2);
    }
    read_input();
    hal.scheduler->delay(50);
}

AP_HAL_MAIN();
