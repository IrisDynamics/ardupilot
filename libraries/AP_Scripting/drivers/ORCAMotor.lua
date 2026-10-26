local ORCAMotor = {}
ORCAMotor.__index = ORCAMotor

-- ArduPilot specific
local MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
local PARAM_TABLE_KEY = 115
local ERROR_LOG_MIN_PERIOD_MS = 10000

-- Modbus specific
local MODBUS_PARTIY = 2
local MODBUS_BAUD = 19200
local MODBUS_STOP_BITS = 1
local MODBUS_FLOW_CTRL = 0
local MODBUS_FN_CODE = { ORCA_EXT_MTR = 0x66, WRITE_SINGLE_REG = 0x06, WRITE_MULTI_REG = 0x10 }
local MODBUS_RX_LEN = { ORCA_EXT_MTR = 42, WRITE_SINGLE_REG = 8, WRITE_MULTI_REG = 8 }
local MODBUS_TIMEOUT_MS = 1000

-- ORCA specific
local ORCA_SLAVE_ID = 1
local ORCA_EXT_CMD_MODE = { NO_CHANGE = 0x00, SLEEP = 0x01, FORCE = 0x02, POSITION = 0x03 }
local REGISTER = {
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
ORCAMotor.MODE = { SLEEP = 1, FORCE = 2, POSITION = 3 }
ORCAMotor.UPDATE_RATE_MS = 100
local ORCA_ERR_HANDLER = { AUTO = 0, RESET = 1, SLEEP = 3, EXTERNAL = 4, EXTERNAL_AND_SLEEP = 5 }
local ORCA_ERR = {
    CONFIG = { mask = 1, severity = MAV_SEVERITY.ERROR, resolve = ORCA_ERR_HANDLER.RESET, message = "verify params" },
    FORCE_CLIP = { mask = 32, severity = MAV_SEVERITY.NOTICE, resolve = ORCA_ERR_HANDLER.AUTO, message = "force reduced" },
    TEMP_EXCEEDED = { mask = 64, severity = MAV_SEVERITY.WARNING, resolve = ORCA_ERR_HANDLER.SLEEP, message = "check temp limits" },
    FORCE_EXCEEDED = { mask = 128, severity = MAV_SEVERITY.NOTICE, resolve = ORCA_ERR_HANDLER.AUTO, message = "check force limits" },
    POWER_EXCEEDED = { mask = 256, severity = MAV_SEVERITY.ERROR, resolve = ORCA_ERR_HANDLER.SLEEP, message = "check power limits" },
    SHAFT_FAILED = { mask = 512, severity = MAV_SEVERITY.ERROR, resolve = ORCA_ERR_HANDLER.EXTERNAL_AND_SLEEP, message = "re-insert shaft" },
    VOLTAGE_INVALID = { mask = 1024, severity = MAV_SEVERITY.ERROR, resolve = ORCA_ERR_HANDLER.EXTERNAL, message = "check power supply" },
    COMM_TIMEOUT = { mask = 2048, severity = MAV_SEVERITY.WARNING, resolve = ORCA_ERR_HANDLER.SLEEP, message = "increase update() frequency" },
    AUTO_ZERO_FAIL = { mask = 8192, severity = MAV_SEVERITY.WARNING, resolve = ORCA_ERR_HANDLER.SLEEP, message = "check auto zero settings" }
}
ORCAMotor.STATUS = { QUEUED = 0, PROCESSING = 1, OK = 2, ERROR = 3 }

--==========================================
-- Parameter Setup
--==========================================

function ORCAMotor:bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("could not add param %s", name))
    return Parameter(self.param_prefix .. name)
end

function ORCAMotor:_parameter_init()
    self.param_prefix = string.format("ORCA%d_", self.instance)
    assert(param:add_table(PARAM_TABLE_KEY, self.param_prefix, 15),
        string.format("could not add param table %s", self.param_prefix))

    --[[
    // @Param: ENABLE
    // @DisplayName: ORCA enable
    // @Description: ORCA enable
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    --]]
    self.params["enable"] = self:bind_add_param("ENABLE", 1, 0)

    --[[
    // @Param: FORCE_SAT
    // @DisplayName: Global force saturation
    // @Description: Limits the maximum force (mN) the position controller will generate in either direction
    // @Range: 0 300000
    // @User: Standard
    --]]
    self.params["force_saturation"] = self:bind_add_param("FORCE_SAT", 2, 300000)

    --[[
    // @Param: P_PID
    // @DisplayName: Proportional action gain
    // @Description: Sets proportional action gain (N/mm), increasing force based on current position error - scaled 64x
    // @Range: 0 32767
    // @User: Advanced
    --]]
    self.params["p_gain_pid"] = self:bind_add_param("P_PID", 3, 1000)

    --[[
    // @Param: I_PID
    // @DisplayName: Integral action gain
    // @Description: Sets integral action gain (N/mm/s), increasing force based on accumulated position error - scaled 64x
    // @Range: 0 32767
    // @User: Advanced
    --]]
    self.params["i_gain_pid"] = self:bind_add_param("I_PID", 4, 2000)

    --[[
    // @Param: D_PID
    // @DisplayName: Derivative action gain
    // @Description: Sets derivative action gain (N*s/mm), increasing force based on rate of change of position error - scaled 2x
    // @Range: 0 32767
    // @User: Advanced
    --]]
    self.params["d_gain_pid"] = self:bind_add_param("D_PID", 5, 100)

    --[[
    // @Param: SPD_LIM
    // @DisplayName: Max velocity
    // @Description: Sets the maximum shaft velocity (mm/s) in Position Mode
    // @Values: 0:Infinite
    // @Range: 0 32767
    // @User: Advanced
    --]]
    self.params["speed_limit"] = self:bind_add_param("SPD_LIM", 6, 1000)

    --[[
    // @Param: ACC_LIM
    // @DisplayName: Max acceleration
    // @Description: Sets the maximum shaft acceleration (mm/s2) in Position Mode
    // @Values: 0:Infinite
    // @Range: 0 32767
    // @User: Advanced
    --]]
    self.params["accel_limit"] = self:bind_add_param("ACC_LIM", 7, 1000)

    --[[
    // @Param: DEC_LIM
    // @DisplayName: Max deceleration
    // @Description: Sets the maximum shaft deceleration (mm/s2) in Position Mode
    // @Values: 0:Infinite
    // @Range: 0 32767
    // @User: Advanced
    --]]
    self.params["decel_limit"] = self:bind_add_param("DEC_LIM", 8, 1000)

    --[[
    // @Param: SOFT_MS
    // @DisplayName: Softstart duration
    // @Description: The duration that forces generated by the position controller will linearly ramp up over when entering Position Mode
    // @Units: ms
    // @Values: 0:Disabled
    // @Range: 0 32767
    // @User: Advanced
    --]]
    self.params["softstart_duration"] = self:bind_add_param("SOFT_MS", 9, 3000)

    --[[
    // @Param: INV_POS
    // @DisplayName: Invert positive direction
    // @Description: Invert the positive direction, so positive values increase in the direction of the cable harness
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    --]]
    self.params["invert_position"] = self:bind_add_param("INV_POS", 10, 0)

    --[[
    // @Param: AZ_MODE
    // @DisplayName: Auto zero mode
    // @Description: Sets the auto zero functionality mode
    // @Values: 0:Zero negative positions,1:Manual zeroing,2:Auto zero enabled,3:Auto zero on boot,4:IOSH auto zero
    // @User: Standard
    --]]
    self.params["autozero_mode"] = self:bind_add_param("AZ_MODE", 11, 0)

    --[[
    // @Param: AZ_FORCE
    // @DisplayName: Auto zero max force
    // @Description: The maximum force the auto-zeroing algorithm will use to move the motor
    // @Units: N
    // @Range: 0 800
    // @User: Advanced
    --]]
    self.params["autozero_force"] = self:bind_add_param("AZ_FORCE", 12, 30)

    --[[
    // @Param: AZ_SPD
    // @DisplayName: Auto zero speed
    // @Description: The shaft speed (mm/s) during the auto-zeroing routine
    // @Range: 0 500
    // @User: Advanced
    --]]
    self.params["autozero_speed"] = self:bind_add_param("AZ_SPD", 13, 50)

    --[[
    // @Param: AZ_EXIT
    // @DisplayName: Auto zero exit mode
    // @Description: The mode to enter after completing auto zero
    // @Values: 1:Sleep,2:Force,3:Position,4:Haptic,5:Kinematic,11:PWM
    // @User: Standard
    --]]
    self.params["autozero_exit_mode"] = self:bind_add_param("AZ_EXIT", 14, ORCAMotor.MODE.SLEEP)

    --[[
    // @Param: APPLY
    // @DisplayName: Apply params
    // @Description: Triggers the routine that loads all parameters to the motor
    // @Values: 0:None,1:Apply
    // @User: Standard
    ]]
    self.params["apply"] = self:bind_add_param("APPLY", 15, 0)
    self.params_loaded = true
end

--==========================================
-- Utility Functions
--==========================================

local parse = {}

-- 16-bit Unsigned Integer
function parse.u16(bytes, idx)
    return (bytes[idx] << 8) | bytes[idx + 1]
end

-- 16-bit Signed Integer
function parse.i16(bytes, idx)
    local u_val = parse.u16(bytes, idx)
    if u_val >= 0x8000 then
        return u_val - 0x10000
    end
    return u_val
end

-- 32-bit Unsigned Integer
function parse.u32(bytes, idx)
    return (bytes[idx] << 24) | (bytes[idx + 1] << 16) | (bytes[idx + 2] << 8) | bytes[idx + 3]
end

-- 32-bit Signed Integer (2's complement)
function parse.i32(bytes, idx)
    local u_val = parse.u32(bytes, idx)
    if u_val >= 0x80000000 then
        return u_val - 0x100000000
    end
    return u_val
end

--==========================================
-- Processing Tables
--==========================================

local crc_hi_lookup = {
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

local crc_lo_lookup = {
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

-- Used to map response payload to state table
local ext_motor_schema = {
    { name = "force_mN",     type = "i32", offset = 3 },
    { name = "position_um",  type = "i32", offset = 7 },
    { name = "speed_mm_s",   type = "i32", offset = 11 },
    { name = "accel_mm_s_2", type = "i32", offset = 15 },
    { name = "board_temp_C", type = "i16", offset = 19 },
    { name = "coil_temp_C",  type = "i16", offset = 21 },
    { name = "voltage_mV",   type = "u16", offset = 23 },
    { name = "power_W",      type = "u16", offset = 25 },
    { name = "mode",         type = "u16", offset = 27 },
    { name = "kin_status",   type = "u16", offset = 29 },
    { name = "kin_count",    type = "u16", offset = 31 },
    { name = "errors",       type = "u16", offset = 33 },
    { name = "motor_status", type = "u16", offset = 35 },
    { name = "read_reg_1",   type = "i16", offset = 37 },
    { name = "read_reg_2",   type = "i16", offset = 39 }
}

--==========================================
-- Modbus Transaction Functions
--==========================================

-- Generate Modbus CRC
local function generate_crc(byte_array, num_bytes)
    local crc_hi = 0xFF
    local crc_lo = 0xFF
    local len = num_bytes or #byte_array
    for i = 1, len do
        local idx = (crc_hi ~ byte_array[i]) + 1
        crc_hi = crc_lo ~ crc_hi_lookup[idx]
        crc_lo = crc_lo_lookup[idx]
    end
    return (crc_hi << 8 | crc_lo)
end

-- Append slave ID, function code, and CRC to TX payload
local function build_modbus_tx_frame(fn, payload)
    table.insert(payload, 1, ORCA_SLAVE_ID)
    table.insert(payload, 2, fn)
    local crc = generate_crc(payload)
    table.insert(payload, crc & 0xFF) -- Low byte
    table.insert(payload, crc >> 8)   -- High byte
    return payload
end

-- Verify slave ID, function code, and CRC in RX frame
function ORCAMotor:verify_modbus_rx_header(response)
    if response[1] ~= ORCA_SLAVE_ID or response[2] ~= self.in_flight.fn then
        gcs:send_text(MAV_SEVERITY.WARNING,
            string.format("ORCA%d: Invalid header or function code response", self.instance))
        return false
    end

    local calc_crc = generate_crc(response, #response - 2)
    local recv_crc = (response[#response] << 8) | response[#response - 1]

    if calc_crc ~= recv_crc then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("ORCA%d: CRC mismatch"))
        return false
    end
    return true
end

-- Build the TX payload for the Extended Motor Command
local function build_ext_tx_payload(mode, data, read_addr)
    mode = mode or ORCAMotor.MODE.SLEEP
    data = data or 0
    read_addr = read_addr or 0
    local payload = { -- Bitwise AND needs to be done to guarantee 8-bit length
        mode & 0xFF,
        (data >> 24) & 0xFF,
        (data >> 16) & 0xFF,
        (data >> 8) & 0xFF,
        data & 0xFF,
        (read_addr >> 8) & 0xFF,
        (read_addr) & 0xFF
    }
    return payload
end

-- Build the TX payload for a Modbus write command (single or multi-register)
local function build_write_tx_payload(fn, reg, data)
    local payload = {}
    if fn == MODBUS_FN_CODE.WRITE_SINGLE_REG then
        payload[1] = (reg >> 8) & 0xFF
        payload[2] = reg & 0xFF
        payload[3] = (data >> 8) & 0xFF
        payload[4] = data & 0xFF
    elseif fn == MODBUS_FN_CODE.WRITE_MULTI_REG then
        for _, val in ipairs(data) do
            table.insert(payload, (val >> 8) & 0xFF)
            table.insert(payload, val & 0xFF)
        end
        table.insert(payload, 1, (reg >> 8) & 0xFF)
        table.insert(payload, 2, reg & 0xFF)
        table.insert(payload, 3, (#data >> 8) & 0xFF)
        table.insert(payload, 4, #data & 0xFF)
        table.insert(payload, 5, (#data * 2) & 0xFF)
    end
    return payload
end

-- Parse the RX payload, returns a table based on provided schema
local function parse_modbus_rx_payload(data_bytes, schema)
    local result = {}
    for _, field in ipairs(schema) do                         -- Iterate through schema
        local decoder = parse[field.type]                     -- Define what parse function to use
        if decoder and (field.offset + 1) <= #data_bytes then --Verify that offsets and payload length works
            local raw_val = decoder(data_bytes, field.offset)
            result[field.name] = raw_val
        end
    end
    return result
end

-- Queue a Modbus register write
-- 'data' can be a single value or table of values for multiple write
-- set front to 'true' to push to front of queue
function ORCAMotor:_queue_write_register(reg, data, front)
    local fn = type(data) == "table" and MODBUS_FN_CODE.WRITE_MULTI_REG or MODBUS_FN_CODE.WRITE_SINGLE_REG
    local rx_len = type(data) == "table" and MODBUS_RX_LEN.WRITE_MULTI_REG or MODBUS_RX_LEN.WRITE_SINGLE_REG
    if front then
        table.insert(self.queue, 1, {
            fn = fn,
            rx_len = rx_len,
            reg = reg,
            data = data
        })
    else
        table.insert(self.queue, {
            fn = fn,
            rx_len = rx_len,
            reg = reg,
            data = data
        })
    end
end

function ORCAMotor:_verify_modbus_write_response(response)
    local valid = false
    local recv_reg = ((response[3] << 8) & 0xFF00) | (response[4] & 0xFF)
    local recv_data = ((response[5] << 8) & 0xFF00) | (response[6] & 0xFF)
    if self.in_flight.fn == MODBUS_FN_CODE.WRITE_SINGLE_REG then
        valid = recv_reg == self.in_flight.reg and recv_data == self.in_flight.data
    else
        valid = recv_reg == self.in_flight.reg and recv_data == #self.in_flight.data
    end
    if not valid then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("ORCA%d: Modbus write response mismatch", self.instance))
        return false
    end
    return true
end

--==========================================
-- Main Update Loop Functions
--==========================================

-- Accept a command item, build payload, finalise frame, and transmit
function ORCAMotor:_transmit_command(cmd)
    -- Flush read buffer
    while self.uart:available():toint() > 0 do
        self.uart:read()
    end

    local payload = {}
    if cmd.fn == MODBUS_FN_CODE.ORCA_EXT_MTR then
        payload = build_ext_tx_payload(cmd.mode, cmd.data, cmd.addr)
    elseif cmd.fn == MODBUS_FN_CODE.WRITE_SINGLE_REG or cmd.fn == MODBUS_FN_CODE.WRITE_MULTI_REG then
        payload = build_write_tx_payload(cmd.fn, cmd.reg, cmd.data)
    end
    build_modbus_tx_frame(cmd.fn, payload)
    local payload_string = string.char(table.unpack(payload))
    if not self.uart:writestring(payload_string) then  -- Faster than writing individual bytes in for loop
        gcs:send_text(MAV_SEVERITY.ERROR, string.format("ORCA%d: 'writestring' failed"))
        self:_update_cmd_status(ORCAMotor.STATUS.ERROR)
    end
end

-- Receive correct byte count, verify header, and parse response
function ORCAMotor:_check_response()
    local available = self.uart:available():toint()
    if available < self.in_flight.rx_len then
        return false
    end

    local response = {}
    for i = 1, self.in_flight.rx_len do
        response[i] = self.uart:read()
    end
    self.last_rx_byte_time = millis()

    if not self:verify_modbus_rx_header(response) then
        self:_update_cmd_status(ORCAMotor.STATUS.ERROR)
        return true
    end

    if self.in_flight.fn == MODBUS_FN_CODE.ORCA_EXT_MTR then
        self.state = parse_modbus_rx_payload(response, ext_motor_schema) -- Populates the current state
        self:_parse_reported_errors()
        self:_update_cmd_status(ORCAMotor.STATUS.OK)
    elseif self.in_flight.fn == MODBUS_FN_CODE.WRITE_SINGLE_REG or self.in_flight.fn == MODBUS_FN_CODE.WRITE_MULTI_REG then
        local status = self:_verify_modbus_write_response(response) and ORCAMotor.STATUS.OK or ORCAMotor.STATUS.ERROR
        self:_update_cmd_status(status)
    end
    return true
end

-- When no queued messages exist, build idle command based on current mode/position/force
function ORCAMotor:_build_idle_cmd()
    local cmd = {
        fn = MODBUS_FN_CODE.ORCA_EXT_MTR,
        rx_len = MODBUS_RX_LEN.ORCA_EXT_MTR,
        mode = nil,
        data = nil,
    }
    if self.target_mode == ORCAMotor.MODE.SLEEP then
        cmd.mode = ORCA_EXT_CMD_MODE.SLEEP
    elseif self.target_mode == ORCAMotor.MODE.POSITION then
        cmd.mode = ORCA_EXT_CMD_MODE.POSITION
        cmd.data = self.target_position
    elseif self.target_mode == ORCAMotor.MODE.FORCE then
        cmd.mode = ORCA_EXT_CMD_MODE.FORCE
        cmd.data = self.target_force
    end
    return cmd
end

--==========================================
-- Error Handling and Status Monitoring
--==========================================

-- Control rate of logging so duplicate errors do not spam Mavlink
function ORCAMotor:_log_error(err_key, err, msg)
    if self.state.errors == self.last_err and (millis() - self.last_err_log_ms) < ERROR_LOG_MIN_PERIOD_MS then return end
    gcs:send_text(err.severity, string.format("ORCA%d ERR:%s - %s", self.instance, err_key, err.message))
    gcs:send_text(err.severity, string.format("ORCA%d fix: %s", self.instance, msg))
    self.last_err_log_ms = millis()
end

-- Handle error based on error clear method <- taken from ORCA reference manual
function ORCAMotor:_handle_errror(err_key, err)
    if err.resolve == ORCA_ERR_HANDLER.AUTO then
        self:_log_error(err_key, err, "automatic")
    elseif err.resolve == ORCA_ERR_HANDLER.RESET then
        self:_log_error(err_key, err, "factory resetting...")
        self:_queue_write_register(REGISTER.CTRL_REG_4, 0xFE) -- Reset tuning values
        self:_queue_write_register(REGISTER.CTRL_REG_2, 0x3E0) -- Save to flash
        self:_queue_write_register(REGISTER.CTRL_REG_0, 0x01) -- Reset motor
    elseif err.resolve == ORCA_ERR_HANDLER.SLEEP then
        self:_log_error(err_key, err, "sleeping...")
        self:set_target_mode(ORCAMotor.MODE.SLEEP)
    elseif err.resolve == ORCA_ERR_HANDLER.EXTERNAL then
        self:_log_error(err_key, err, "external")
    elseif err.resolve == ORCA_ERR_HANDLER.EXTERNAL_AND_SLEEP then
        self:_log_error(err_key, err, "external + sleeping...")
        self:set_target_mode(ORCAMotor.MODE.SLEEP)
    end
end

-- Parse the error table populated by the idle command
function ORCAMotor:_parse_reported_errors()
    if self.state.errors == 0 then 
        if self.last_err ~= 0 then -- Active error was cleared in the motor
            gcs:send_text(MAV_SEVERITY.INFO, string.format("ORCA%d: Errors cleared", self.instance))
        end
        self.last_err = 0
        return
    end -- Early return if all clear

    for err_key, err in pairs(ORCA_ERR) do
        if self.state.errors & err.mask == err.mask then
            self:_handle_errror(err_key, err)
        end
    end

    self.last_err = self.state.errors
end

-- Gets called as transaction progresses
-- Only updates for user-called functions (except param load)
function ORCAMotor:_update_cmd_status(status)
    if self.in_flight.handle then
        self.in_flight.handle.status = status
    end
end

--==========================================
-- Private Setters
--==========================================

-- Private setter functions get called when a queued message is dequeued and 'task' is populated
function ORCAMotor:_set_position(pos)
    self.target_position = pos
    self.target_mode = ORCAMotor.MODE.POSITION
end

function ORCAMotor:_set_force(force)
    self.target_force = force
    self.target_mode = ORCAMotor.MODE.FORCE
end

function ORCAMotor:_set_mode(mode)
    self.target_mode = mode
end

--==========================================
-- Public Functions
--==========================================

-- Constructor
function ORCAMotor.new(port_num, instance_num)
    local self = setmetatable({}, ORCAMotor)

    self.instance = instance_num
    self.params = {}
    self.params_loaded = false
    self:_parameter_init()

    -- Initialisation
    self.uart = serial:find_serial(port_num)
    if self.uart then
        self.uart:begin(MODBUS_BAUD)
        self.uart:configure_parity(MODBUS_PARTIY)
        self.uart:set_flow_control(MODBUS_FLOW_CTRL)
        self.uart:set_stop_bits(MODBUS_STOP_BITS)
    else
        gcs:send_text(MAV_SEVERITY.ERROR, string.format("ORCA%d: Serial port not found", port_num))
    end


    -- Message variables
    self.queue = {}      -- Gets loaded when users call public member functions
    self.in_flight = nil -- Current transmission
    self.tx_time = 0
    self.last_rx_byte_time = 0

    -- Motor state information
    self.target_position = 0
    self.target_force = 0
    self.target_mode = ORCAMotor.MODE.SLEEP
    self.state = {
        force_mN = 0,
        position_um = 0,
        speed_mm_s = 0,
        accel_mm_s_2 = 0,
        board_temp_C = 0,
        coil_temp_C = 0,
        voltage_mV = 0,
        power_W = 0,
        mode = 0,
        kin_status = 0,
        kin_count = 0,
        errors = 0,
        motor_status = 0,
        read_reg_1 = 0,
        read_reg_2 = 0
    }
    self.last_err = 0
    self.last_err_log_ms = 0
    return self
end

-- Queue a mode command
function ORCAMotor:set_target_mode(target_mode)
    local handle = {
        id = millis(),
        status = ORCAMotor.STATUS.QUEUED
    }
    table.insert(self.queue, {
        fn = MODBUS_FN_CODE.ORCA_EXT_MTR,
        rx_len = MODBUS_RX_LEN.ORCA_EXT_MTR,
        mode = target_mode,
        task = function() self:_set_mode(target_mode) end,
        handle = handle
    })
    return handle
end

-- Queue a position command
function ORCAMotor:set_target_position_um(target_pos)
    local handle = {
        id = millis(),
        status = ORCAMotor.STATUS.QUEUED
    }
    table.insert(self.queue, {
        fn = MODBUS_FN_CODE.ORCA_EXT_MTR,
        rx_len = MODBUS_RX_LEN.ORCA_EXT_MTR,
        mode = ORCA_EXT_CMD_MODE.POSITION,
        data = target_pos,
        task = function() self:_set_position(target_pos) end,
        handle = handle
    })
    return handle
end

-- Queue a force command
function ORCAMotor:set_target_force_mN(target_force)
    local handle = {
        id = millis(),
        status = ORCAMotor.STATUS.QUEUED
    }
    table.insert(self.queue, {
        fn = MODBUS_FN_CODE.ORCA_EXT_MTR,
        rx_len = MODBUS_RX_LEN.ORCA_EXT_MTR,
        mode = ORCA_EXT_CMD_MODE.FORCE,
        data = target_force,
        task = function() self:_set_force(target_force) end,
        handle = handle
    })
    return handle
end

-- Update function must be called in user script
function ORCAMotor:update()
    if self.params.enable:get() == 0 then return end
    if self.params.apply:get() > 0 then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("ORCA%d: Applying params to motor", self.instance))
        self:load_params()
        self.params.apply:set_and_save(0)
    end

    if self.in_flight then
        if self:_check_response() then
            self.in_flight = nil
        elseif (millis() - self.tx_time) > MODBUS_TIMEOUT_MS then
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("ORCA%d: Command timeout", self.instance))
            self:_update_cmd_status(ORCAMotor.STATUS.ERROR)
            self.in_flight = nil
        else
            return
        end
    end

    local cmd = {}
    if #self.queue > 0 and not self.in_flight then
        cmd = table.remove(self.queue, 1)
        if cmd.task then cmd.task() end -- Run a function before transmission
    elseif not self.in_flight then
        cmd = self:_build_idle_cmd()
    end
    self.in_flight = cmd
    self:_update_cmd_status(ORCAMotor.STATUS.PROCESSING)
    self:_transmit_command(cmd)
    self.tx_time = millis()
end

-- Can be called by user to manually load params, or happens on `param set ORCAx_APPLY 1`
function ORCAMotor:load_params()
    local force = {
        self.params.force_saturation:get() & 0xFFFF,
        (self.params.force_saturation:get() >> 16) & 0xFFFF
    }
    self:_queue_write_register(REGISTER.USER_MAX_FORCE, force)
    self:_queue_write_register(REGISTER.PC_PGAIN, self.params.p_gain_pid:get())
    self:_queue_write_register(REGISTER.PC_IGAIN, self.params.i_gain_pid:get())
    self:_queue_write_register(REGISTER.PC_DVGAIN, self.params.d_gain_pid:get())
    self:_queue_write_register(REGISTER.PC_PGAIN, self.params.p_gain_pid:get())
    self:_queue_write_register(REGISTER.POS_MAX_VEL, self.params.speed_limit:get())
    self:_queue_write_register(REGISTER.POS_MAX_ACCEL, self.params.accel_limit:get())
    self:_queue_write_register(REGISTER.POS_MAX_DECEL, self.params.decel_limit:get())
    self:_queue_write_register(REGISTER.PC_SOFTSTART_PERIOD, self.params.softstart_duration:get())
    self:_queue_write_register(REGISTER.ZERO_MODE, self.params.autozero_mode:get())
    self:_queue_write_register(REGISTER.AUTO_ZERO_FORCE_N, self.params.autozero_force:get())
    self:_queue_write_register(REGISTER.AUTO_ZERO_SPEED_MMPS, self.params.autozero_speed:get())
    self:_queue_write_register(REGISTER.AUTO_ZERO_EXIT_MODE, self.params.autozero_exit_mode:get())
    self.params_loaded = true
end

-- Clears latched and active errors
function ORCAMotor:clear_motor_errors()
    self:_queue_write_register(REGISTER.CTRL_REG_0, 0x0002, true)
end

return ORCAMotor
