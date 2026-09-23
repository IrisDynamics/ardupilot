
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

#include "AP_ORCAMotor_config.h"

#if HAL_ORCAMOTOR_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include "AP_ORCAMotor_Params.h"

#define AP_ORCAMOTOR_MAX_INSTANCES 2

enum MotorMode {
    MODE_SLEEP = 1,
    MODE_FORCE = 2,
    MODE_POSITION = 3,
    MODE_HAPTIC = 4,
    MODE_KINEMATIC = 5,
    MODE_PWM = 11,
    MODE_AUTOZERO = 55
};

struct ExtMotorData {
    int32_t force_mN;
    int32_t position_um;
    int32_t speed_mm_s;
    int32_t acceleration_mm_s_2;
    int16_t board_temp_C;
    int16_t coil_temp_C;
    int16_t mode_of_operation;
    int16_t kin_status;
    int16_t errors;
    int16_t motor_status;
    int16_t register_read_1;
    int16_t register_read_2;
};

class AP_ORCAMotor_Backend;
class AP_ORCAMotor_Modbus;

class AP_ORCAMotor{

    friend class AP_ORCAMotor_Backend;
    friend class AP_ORCAMotor_Modbus;
public:
    AP_ORCAMotor();

    /// Do not allow copies
    CLASS_NO_COPY(AP_ORCAMotor);
    static AP_ORCAMotor *get_singleton();

    void init();
    void update();

    bool enabled();
    bool enabled(uint8_t instance);

    bool healthy();
    bool healthy(uint8_t instance);

    void set_mode(MotorMode mode);
    void set_mode(uint8_t instance, MotorMode mode);

    void set_target_position_um(uint8_t instance, int32_t target_position);
    void set_target_position_um(int32_t target_position);

    void set_target_force_mN(uint8_t instance, int32_t target_force);
    void set_target_force_mN(int32_t target_force);

    static const struct AP_Param::GroupInfo var_info[];

    AP_ORCAMotor_Params _params[AP_ORCAMOTOR_MAX_INSTANCES];
    ExtMotorData _state[AP_ORCAMOTOR_MAX_INSTANCES];

private:
    AP_ORCAMotor_Backend *get_instance(uint8_t instance) const;

    static AP_ORCAMotor *_singleton;
    AP_ORCAMotor_Backend *_backends[AP_ORCAMOTOR_MAX_INSTANCES];  // pointers to instantiated backends
};

namespace AP {
    AP_ORCAMotor *orcamotor();
};

#endif
