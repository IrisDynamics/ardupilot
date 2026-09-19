
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

    static const struct AP_Param::GroupInfo var_info[];

    AP_ORCAMotor_Params _params[AP_ORCAMOTOR_MAX_INSTANCES];

private:
    AP_ORCAMotor_Backend *get_instance(uint8_t instance) const;

    static AP_ORCAMotor *_singleton;
    AP_ORCAMotor_Backend *_backends[AP_ORCAMOTOR_MAX_INSTANCES];  // pointers to instantiated backends
};

namespace AP {
    AP_ORCAMotor *orcamotor();
};

#endif
