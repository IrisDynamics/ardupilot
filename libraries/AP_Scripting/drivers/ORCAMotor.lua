local ORCAMotor = {}
ORCAMotor.__index = ORCAMotor

-- ArduPilot specific
local MAV_SEVERITY = { EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7 }
ORCAMotor.UPDATE_RATE_MS = 50

-- Modbus specific
local MODBUS_PARTIY = 2
local MODBUS_BAUD = 19200
local MODBUS_STOP_BITS = 1
local MODBUS_FLOW_CTRL = 0
local MODBUS_FN_CODE = { ORCA_EXT_MTR = 0x66 }
local MODBUS_RX_LEN = { ORCA_EXT_MTR = 42 }
local MODBUS_TIMEOUT_MS = 1000

-- ORCA specific
local ORCA_SLAVE_ID = 1
local ORCA_EXT_CMD_MODE = { NO_CHANGE = 0x00, SLEEP = 0x01, FORCE = 0x02, POSITION = 0x03 }
ORCAMotor.MODE = { SLEEP = 1, FORCE = 2, POSITION = 3 }

-- Helper functions to parse values from Modbus response
local parse = {}

-- 16-bit Unsigned Integer
function parse.u16(bytes, idx)
    return (bytes[idx] << 8) | bytes[idx + 1]
end

-- 16-bit Signed Integer
function parse.i16(bytes, idx)
    local val = parse.u16(bytes, idx)
    return (val >= 0x8000) and (val - 0x10000) or val
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

-- Constructor
function ORCAMotor.new(port_num)
    local self = setmetatable({}, ORCAMotor)

    -- Initialisation
    self.uart = serial:find_serial(port_num)
    if self.uart then
        self.uart:begin(MODBUS_BAUD)
        self.uart:configure_parity(MODBUS_PARTIY)
        self.uart:set_flow_control(MODBUS_FLOW_CTRL)
        self.uart:set_stop_bits(MODBUS_STOP_BITS)
        self.uart:set_unbuffered_writes(true) --Enabling this fixed random occurences of write errors
    else
        gcs:send_text(MAV_SEVERITY.ERROR, string.format("ORCA%d: Serial port not found", port_num))
    end

    -- Message variables
    self.instance = port_num
    self.queue = {}      -- Gets loaded when users call member functions
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
        errors = 0,
        motor_status = 0,
        read_reg_1 = 0,
        read_reg_2 = 0
    }
    return self
end

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

-- Used when parsing Extended Motor Frame response
-- Names must exactly match names in 'state' member variable
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

-- Append slave ID, function code, and CRC to TX payload
local function build_modbus_tx_frame(fn, payload)
    table.insert(payload, 1, ORCA_SLAVE_ID)
    table.insert(payload, 2, fn)
    local crc = generate_crc(payload)
    table.insert(payload, crc & 0xFF)
    table.insert(payload, crc >> 8)
    return payload
end

-- Verify slave ID, function code, and CRC in RX frame
function ORCAMotor:verify_modbus_rx_header(response)
    if response[1] ~= ORCA_SLAVE_ID or response[2] ~= self.in_flight.fn then
        gcs:send_text(MAV_SEVERITY.WARNING, "Modbus Error: Invalid header or function code response")
        return false
    end

    local calc_crc = generate_crc(response, #response - 2)
    local recv_crc = (response[#response] << 8) | response[#response - 1]

    if calc_crc ~= recv_crc then
        gcs:send_text(MAV_SEVERITY.WARNING, "Modbus Error: CRC Mismatch!")
        return false
    end
    return true
end

-- Build the TX payload for the Extended Motor Command
local function build_ext_tx_payload(mode, data, read_addr)
    mode = mode or ORCAMotor.MODE.SLEEP
    data = data or 0
    read_addr = read_addr or 0
    local payload = {
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

-- Parse the RX payload, returns a table based on provided schema
local function parse_modbus_rx_payload(data_bytes, schema)
    local result = {}
    for _, field in ipairs(schema) do
        local decoder = parse[field.type]
        if decoder and (field.offset + 1) <= #data_bytes then
            local raw_val = decoder(data_bytes, field.offset)
            result[field.name] = raw_val
        end
    end
    return result
end

-- Ingest a command, build payload, finalise frame, and transmit
function ORCAMotor:_transmit_command(cmd)
    local payload = {}
    if cmd.fn == MODBUS_FN_CODE.ORCA_EXT_MTR then
        payload = build_ext_tx_payload(cmd.mode, cmd.data, cmd.addr)
    end

    build_modbus_tx_frame(cmd.fn, payload)
    if (#payload ~= 11) then
        gcs:send_text(3, "Malformed UART payload")
    end
    for i = 1, #payload do
        if payload[i] ~= nil then
            self.uart:write(payload[i])
        end
    end
end

-- Receive correct byte count, verify header, and parse response
function ORCAMotor:_check_response()
    local available = self.uart:available():toint()
    if available < self.in_flight.rx_len then
        return false
    end
    self.last_rx_byte_time = millis()
    local response = {}
    for i = 1, available do
        response[i] = self.uart:read()
    end

    if not self:verify_modbus_rx_header(response) then return false end

    if self.in_flight.fn == MODBUS_FN_CODE.ORCA_EXT_MTR then
        self.state = parse_modbus_rx_payload(response, ext_motor_schema) -- Populates the current state
    end
    return true
end

-- When no queued messages exist, build idle command based on current mode/position/force
function ORCAMotor:_build_idle_cmd()
    local cmd = {
        fn = MODBUS_FN_CODE.ORCA_EXT_MTR,
        rx_len = MODBUS_RX_LEN.ORCA_EXT_MTR,
        reg = nil,
        reg_count = nil,
        mode = nil,
        data = nil,
        addr = nil,
        task = nil
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

-- Update function gets called in user script
function ORCAMotor:update()
    if self.in_flight then
        if self:_check_response() then
            self.in_flight = nil
        elseif (millis() - self.tx_time) > MODBUS_TIMEOUT_MS then
            gcs:send_text(MAV_SEVERITY.ERROR, string.format("ORCA%d Error: Command Timeout", self.instance))
            self.in_flight = nil
        else
            return
        end
    end

    local cmd = {}
    if #self.queue > 0 and not self.in_flight then
        cmd = table.remove(self.queue, 1)
        if cmd.task then cmd.task() end -- Run a function before transmission
        self:_transmit_command(cmd)
    elseif not self.in_flight then
        cmd = self:_build_idle_cmd()
        self:_transmit_command(cmd)
    end
    self.in_flight = cmd
    self.tx_time = millis()
    local latency = (self.tx_time - self.last_rx_byte_time):toint()
    if latency > 500 then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format(" RX->TX Latency: %d ms", latency))
    end
end

-- Queue a mode command
function ORCAMotor:set_target_mode(target_mode)
    table.insert(self.queue, {
        fn = MODBUS_FN_CODE.ORCA_EXT_MTR,
        rx_len = MODBUS_RX_LEN.ORCA_EXT_MTR,
        reg = nil,
        reg_count = nil,
        mode = target_mode,
        data = nil,
        addr = nil,
        task = function() self:_set_mode(target_mode) end
    })
end

-- Queue a position command
function ORCAMotor:set_target_position_um(target_pos)
    table.insert(self.queue, {
        fn = MODBUS_FN_CODE.ORCA_EXT_MTR,
        rx_len = MODBUS_RX_LEN.ORCA_EXT_MTR,
        reg = nil,
        reg_count = nil,
        mode = ORCA_EXT_CMD_MODE.POSITION,
        data = target_pos,
        addr = nil,
        task = function() self:_set_position(target_pos) end
    })
end

-- Queue a force command
function ORCAMotor:set_target_force_mN(target_force)
    table.insert(self.queue, {
        fn = MODBUS_FN_CODE.ORCA_EXT_MTR,
        rx_len = MODBUS_RX_LEN.ORCA_EXT_MTR,
        reg = nil,
        reg_count = nil,
        mode = ORCA_EXT_CMD_MODE.FORCE,
        data = target_force,
        addr = nil,
        task = function() self:_set_force(target_force) end
    })
end

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

return ORCAMotor
