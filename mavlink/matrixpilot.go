package mavlink

import (
	"encoding/binary"
	"fmt"
	"math"
)

//////////////////////////////////////////////////
//
// NOTE: do not edit,
// this file created automatically by mavgen.go
//
//////////////////////////////////////////////////

// MavPreflightStorageAction: Action required when performing CMD_PREFLIGHT_STORAGE
const (
	MAV_PFS_CMD_READ_ALL       = 0 // Read all parameters from storage
	MAV_PFS_CMD_WRITE_ALL      = 1 // Write all parameters to storage
	MAV_PFS_CMD_CLEAR_ALL      = 2 // Clear all  parameters in storage
	MAV_PFS_CMD_READ_SPECIFIC  = 3 // Read specific parameters from storage
	MAV_PFS_CMD_WRITE_SPECIFIC = 4 // Write specific parameters to storage
	MAV_PFS_CMD_CLEAR_SPECIFIC = 5 // Clear specific parameters in storage
	MAV_PFS_CMD_DO_NOTHING     = 6 // do nothing
)

// MavCmd:
const (
	MAV_CMD_PREFLIGHT_STORAGE_ADVANCED = 0 // Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
)

// Depreciated but used as a compiler flag.  Do not remove
type FlexifunctionSet struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *FlexifunctionSet) MsgID() uint8 {
	return 150
}

func (self *FlexifunctionSet) MsgName() string {
	return "FlexifunctionSet"
}

func (self *FlexifunctionSet) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FlexifunctionSet) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	return nil
}

// Reqest reading of flexifunction data
type FlexifunctionReadReq struct {
	ReadReqType     int16 // Type of flexifunction data requested
	DataIndex       int16 // index into data where needed
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *FlexifunctionReadReq) MsgID() uint8 {
	return 151
}

func (self *FlexifunctionReadReq) MsgName() string {
	return "FlexifunctionReadReq"
}

func (self *FlexifunctionReadReq) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.ReadReqType))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.DataIndex))
	payload[4] = byte(self.TargetSystem)
	payload[5] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FlexifunctionReadReq) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	self.ReadReqType = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.DataIndex = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.TargetSystem = uint8(p.Payload[4])
	self.TargetComponent = uint8(p.Payload[5])
	return nil
}

// Flexifunction type and parameters for component at function index from buffer
type FlexifunctionBufferFunction struct {
	FuncIndex       uint16   // Function index
	FuncCount       uint16   // Total count of functions
	DataAddress     uint16   // Address in the flexifunction data, Set to 0xFFFF to use address in target memory
	DataSize        uint16   // Size of the
	TargetSystem    uint8    // System ID
	TargetComponent uint8    // Component ID
	Data            [48]int8 // Settings data
}

func (self *FlexifunctionBufferFunction) MsgID() uint8 {
	return 152
}

func (self *FlexifunctionBufferFunction) MsgName() string {
	return "FlexifunctionBufferFunction"
}

func (self *FlexifunctionBufferFunction) Pack(p *Packet) error {
	payload := make([]byte, 58)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.FuncIndex))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.FuncCount))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.DataAddress))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.DataSize))
	payload[8] = byte(self.TargetSystem)
	payload[9] = byte(self.TargetComponent)
	for i, v := range self.Data {
		payload[10+i*1] = byte(v)
	}

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FlexifunctionBufferFunction) Unpack(p *Packet) error {
	if len(p.Payload) < 58 {
		return fmt.Errorf("payload too small")
	}
	self.FuncIndex = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.FuncCount = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.DataAddress = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.DataSize = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.TargetSystem = uint8(p.Payload[8])
	self.TargetComponent = uint8(p.Payload[9])
	for i := 0; i < len(self.Data); i++ {
		self.Data[i] = int8(p.Payload[10+i*1])
	}
	return nil
}

// Flexifunction type and parameters for component at function index from buffer
type FlexifunctionBufferFunctionAck struct {
	FuncIndex       uint16 // Function index
	Result          uint16 // result of acknowledge, 0=fail, 1=good
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

func (self *FlexifunctionBufferFunctionAck) MsgID() uint8 {
	return 153
}

func (self *FlexifunctionBufferFunctionAck) MsgName() string {
	return "FlexifunctionBufferFunctionAck"
}

func (self *FlexifunctionBufferFunctionAck) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.FuncIndex))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.Result))
	payload[4] = byte(self.TargetSystem)
	payload[5] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FlexifunctionBufferFunctionAck) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	self.FuncIndex = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.Result = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.TargetSystem = uint8(p.Payload[4])
	self.TargetComponent = uint8(p.Payload[5])
	return nil
}

// Acknowldge sucess or failure of a flexifunction command
type FlexifunctionDirectory struct {
	TargetSystem    uint8    // System ID
	TargetComponent uint8    // Component ID
	DirectoryType   uint8    // 0=inputs, 1=outputs
	StartIndex      uint8    // index of first directory entry to write
	Count           uint8    // count of directory entries to write
	DirectoryData   [48]int8 // Settings data
}

func (self *FlexifunctionDirectory) MsgID() uint8 {
	return 155
}

func (self *FlexifunctionDirectory) MsgName() string {
	return "FlexifunctionDirectory"
}

func (self *FlexifunctionDirectory) Pack(p *Packet) error {
	payload := make([]byte, 53)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.DirectoryType)
	payload[3] = byte(self.StartIndex)
	payload[4] = byte(self.Count)
	for i, v := range self.DirectoryData {
		payload[5+i*1] = byte(v)
	}

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FlexifunctionDirectory) Unpack(p *Packet) error {
	if len(p.Payload) < 53 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.DirectoryType = uint8(p.Payload[2])
	self.StartIndex = uint8(p.Payload[3])
	self.Count = uint8(p.Payload[4])
	for i := 0; i < len(self.DirectoryData); i++ {
		self.DirectoryData[i] = int8(p.Payload[5+i*1])
	}
	return nil
}

// Acknowldge sucess or failure of a flexifunction command
type FlexifunctionDirectoryAck struct {
	Result          uint16 // result of acknowledge, 0=fail, 1=good
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
	DirectoryType   uint8  // 0=inputs, 1=outputs
	StartIndex      uint8  // index of first directory entry to write
	Count           uint8  // count of directory entries to write
}

func (self *FlexifunctionDirectoryAck) MsgID() uint8 {
	return 156
}

func (self *FlexifunctionDirectoryAck) MsgName() string {
	return "FlexifunctionDirectoryAck"
}

func (self *FlexifunctionDirectoryAck) Pack(p *Packet) error {
	payload := make([]byte, 7)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Result))
	payload[2] = byte(self.TargetSystem)
	payload[3] = byte(self.TargetComponent)
	payload[4] = byte(self.DirectoryType)
	payload[5] = byte(self.StartIndex)
	payload[6] = byte(self.Count)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FlexifunctionDirectoryAck) Unpack(p *Packet) error {
	if len(p.Payload) < 7 {
		return fmt.Errorf("payload too small")
	}
	self.Result = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.TargetSystem = uint8(p.Payload[2])
	self.TargetComponent = uint8(p.Payload[3])
	self.DirectoryType = uint8(p.Payload[4])
	self.StartIndex = uint8(p.Payload[5])
	self.Count = uint8(p.Payload[6])
	return nil
}

// Acknowldge sucess or failure of a flexifunction command
type FlexifunctionCommand struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	CommandType     uint8 // Flexifunction command type
}

func (self *FlexifunctionCommand) MsgID() uint8 {
	return 157
}

func (self *FlexifunctionCommand) MsgName() string {
	return "FlexifunctionCommand"
}

func (self *FlexifunctionCommand) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.CommandType)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FlexifunctionCommand) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.CommandType = uint8(p.Payload[2])
	return nil
}

// Acknowldge sucess or failure of a flexifunction command
type FlexifunctionCommandAck struct {
	CommandType uint16 // Command acknowledged
	Result      uint16 // result of acknowledge
}

func (self *FlexifunctionCommandAck) MsgID() uint8 {
	return 158
}

func (self *FlexifunctionCommandAck) MsgName() string {
	return "FlexifunctionCommandAck"
}

func (self *FlexifunctionCommandAck) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.CommandType))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.Result))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FlexifunctionCommandAck) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	self.CommandType = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.Result = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	return nil
}

// Backwards compatible MAVLink version of SERIAL_UDB_EXTRA - F2: Format Part A
type SerialUdbExtraF2A struct {
	SueTime           uint32 // Serial UDB Extra Time
	SueLatitude       int32  // Serial UDB Extra Latitude
	SueLongitude      int32  // Serial UDB Extra Longitude
	SueAltitude       int32  // Serial UDB Extra Altitude
	SueWaypointIndex  uint16 // Serial UDB Extra Waypoint Index
	SueRmat0          int16  // Serial UDB Extra Rmat 0
	SueRmat1          int16  // Serial UDB Extra Rmat 1
	SueRmat2          int16  // Serial UDB Extra Rmat 2
	SueRmat3          int16  // Serial UDB Extra Rmat 3
	SueRmat4          int16  // Serial UDB Extra Rmat 4
	SueRmat5          int16  // Serial UDB Extra Rmat 5
	SueRmat6          int16  // Serial UDB Extra Rmat 6
	SueRmat7          int16  // Serial UDB Extra Rmat 7
	SueRmat8          int16  // Serial UDB Extra Rmat 8
	SueCog            uint16 // Serial UDB Extra GPS Course Over Ground
	SueSog            int16  // Serial UDB Extra Speed Over Ground
	SueCpuLoad        uint16 // Serial UDB Extra CPU Load
	SueAirSpeed3dimu  uint16 // Serial UDB Extra 3D IMU Air Speed
	SueEstimatedWind0 int16  // Serial UDB Extra Estimated Wind 0
	SueEstimatedWind1 int16  // Serial UDB Extra Estimated Wind 1
	SueEstimatedWind2 int16  // Serial UDB Extra Estimated Wind 2
	SueMagfieldearth0 int16  // Serial UDB Extra Magnetic Field Earth 0
	SueMagfieldearth1 int16  // Serial UDB Extra Magnetic Field Earth 1
	SueMagfieldearth2 int16  // Serial UDB Extra Magnetic Field Earth 2
	SueSvs            int16  // Serial UDB Extra Number of Sattelites in View
	SueHdop           int16  // Serial UDB Extra GPS Horizontal Dilution of Precision
	SueStatus         uint8  // Serial UDB Extra Status
}

func (self *SerialUdbExtraF2A) MsgID() uint8 {
	return 170
}

func (self *SerialUdbExtraF2A) MsgName() string {
	return "SerialUdbExtraF2A"
}

func (self *SerialUdbExtraF2A) Pack(p *Packet) error {
	payload := make([]byte, 61)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.SueTime))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.SueLatitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.SueLongitude))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.SueAltitude))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.SueWaypointIndex))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.SueRmat0))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.SueRmat1))
	binary.LittleEndian.PutUint16(payload[22:], uint16(self.SueRmat2))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.SueRmat3))
	binary.LittleEndian.PutUint16(payload[26:], uint16(self.SueRmat4))
	binary.LittleEndian.PutUint16(payload[28:], uint16(self.SueRmat5))
	binary.LittleEndian.PutUint16(payload[30:], uint16(self.SueRmat6))
	binary.LittleEndian.PutUint16(payload[32:], uint16(self.SueRmat7))
	binary.LittleEndian.PutUint16(payload[34:], uint16(self.SueRmat8))
	binary.LittleEndian.PutUint16(payload[36:], uint16(self.SueCog))
	binary.LittleEndian.PutUint16(payload[38:], uint16(self.SueSog))
	binary.LittleEndian.PutUint16(payload[40:], uint16(self.SueCpuLoad))
	binary.LittleEndian.PutUint16(payload[42:], uint16(self.SueAirSpeed3dimu))
	binary.LittleEndian.PutUint16(payload[44:], uint16(self.SueEstimatedWind0))
	binary.LittleEndian.PutUint16(payload[46:], uint16(self.SueEstimatedWind1))
	binary.LittleEndian.PutUint16(payload[48:], uint16(self.SueEstimatedWind2))
	binary.LittleEndian.PutUint16(payload[50:], uint16(self.SueMagfieldearth0))
	binary.LittleEndian.PutUint16(payload[52:], uint16(self.SueMagfieldearth1))
	binary.LittleEndian.PutUint16(payload[54:], uint16(self.SueMagfieldearth2))
	binary.LittleEndian.PutUint16(payload[56:], uint16(self.SueSvs))
	binary.LittleEndian.PutUint16(payload[58:], uint16(self.SueHdop))
	payload[60] = byte(self.SueStatus)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialUdbExtraF2A) Unpack(p *Packet) error {
	if len(p.Payload) < 61 {
		return fmt.Errorf("payload too small")
	}
	self.SueTime = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.SueLatitude = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.SueLongitude = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.SueAltitude = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.SueWaypointIndex = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.SueRmat0 = int16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.SueRmat1 = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.SueRmat2 = int16(binary.LittleEndian.Uint16(p.Payload[22:]))
	self.SueRmat3 = int16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.SueRmat4 = int16(binary.LittleEndian.Uint16(p.Payload[26:]))
	self.SueRmat5 = int16(binary.LittleEndian.Uint16(p.Payload[28:]))
	self.SueRmat6 = int16(binary.LittleEndian.Uint16(p.Payload[30:]))
	self.SueRmat7 = int16(binary.LittleEndian.Uint16(p.Payload[32:]))
	self.SueRmat8 = int16(binary.LittleEndian.Uint16(p.Payload[34:]))
	self.SueCog = uint16(binary.LittleEndian.Uint16(p.Payload[36:]))
	self.SueSog = int16(binary.LittleEndian.Uint16(p.Payload[38:]))
	self.SueCpuLoad = uint16(binary.LittleEndian.Uint16(p.Payload[40:]))
	self.SueAirSpeed3dimu = uint16(binary.LittleEndian.Uint16(p.Payload[42:]))
	self.SueEstimatedWind0 = int16(binary.LittleEndian.Uint16(p.Payload[44:]))
	self.SueEstimatedWind1 = int16(binary.LittleEndian.Uint16(p.Payload[46:]))
	self.SueEstimatedWind2 = int16(binary.LittleEndian.Uint16(p.Payload[48:]))
	self.SueMagfieldearth0 = int16(binary.LittleEndian.Uint16(p.Payload[50:]))
	self.SueMagfieldearth1 = int16(binary.LittleEndian.Uint16(p.Payload[52:]))
	self.SueMagfieldearth2 = int16(binary.LittleEndian.Uint16(p.Payload[54:]))
	self.SueSvs = int16(binary.LittleEndian.Uint16(p.Payload[56:]))
	self.SueHdop = int16(binary.LittleEndian.Uint16(p.Payload[58:]))
	self.SueStatus = uint8(p.Payload[60])
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA - F2: Part B
type SerialUdbExtraF2B struct {
	SueTime                uint32 // Serial UDB Extra Time
	SueFlags               uint32 // Serial UDB Extra Status Flags
	SueBaromPress          int32  // SUE barometer pressure
	SueBaromAlt            int32  // SUE barometer altitude
	SuePwmInput1           int16  // Serial UDB Extra PWM Input Channel 1
	SuePwmInput2           int16  // Serial UDB Extra PWM Input Channel 2
	SuePwmInput3           int16  // Serial UDB Extra PWM Input Channel 3
	SuePwmInput4           int16  // Serial UDB Extra PWM Input Channel 4
	SuePwmInput5           int16  // Serial UDB Extra PWM Input Channel 5
	SuePwmInput6           int16  // Serial UDB Extra PWM Input Channel 6
	SuePwmInput7           int16  // Serial UDB Extra PWM Input Channel 7
	SuePwmInput8           int16  // Serial UDB Extra PWM Input Channel 8
	SuePwmInput9           int16  // Serial UDB Extra PWM Input Channel 9
	SuePwmInput10          int16  // Serial UDB Extra PWM Input Channel 10
	SuePwmInput11          int16  // Serial UDB Extra PWM Input Channel 11
	SuePwmInput12          int16  // Serial UDB Extra PWM Input Channel 12
	SuePwmOutput1          int16  // Serial UDB Extra PWM Output Channel 1
	SuePwmOutput2          int16  // Serial UDB Extra PWM Output Channel 2
	SuePwmOutput3          int16  // Serial UDB Extra PWM Output Channel 3
	SuePwmOutput4          int16  // Serial UDB Extra PWM Output Channel 4
	SuePwmOutput5          int16  // Serial UDB Extra PWM Output Channel 5
	SuePwmOutput6          int16  // Serial UDB Extra PWM Output Channel 6
	SuePwmOutput7          int16  // Serial UDB Extra PWM Output Channel 7
	SuePwmOutput8          int16  // Serial UDB Extra PWM Output Channel 8
	SuePwmOutput9          int16  // Serial UDB Extra PWM Output Channel 9
	SuePwmOutput10         int16  // Serial UDB Extra PWM Output Channel 10
	SuePwmOutput11         int16  // Serial UDB Extra PWM Output Channel 11
	SuePwmOutput12         int16  // Serial UDB Extra PWM Output Channel 12
	SueImuLocationX        int16  // Serial UDB Extra IMU Location X
	SueImuLocationY        int16  // Serial UDB Extra IMU Location Y
	SueImuLocationZ        int16  // Serial UDB Extra IMU Location Z
	SueLocationErrorEarthX int16  // Serial UDB Location Error Earth X
	SueLocationErrorEarthY int16  // Serial UDB Location Error Earth Y
	SueLocationErrorEarthZ int16  // Serial UDB Location Error Earth Z
	SueOscFails            int16  // Serial UDB Extra Oscillator Failure Count
	SueImuVelocityX        int16  // Serial UDB Extra IMU Velocity X
	SueImuVelocityY        int16  // Serial UDB Extra IMU Velocity Y
	SueImuVelocityZ        int16  // Serial UDB Extra IMU Velocity Z
	SueWaypointGoalX       int16  // Serial UDB Extra Current Waypoint Goal X
	SueWaypointGoalY       int16  // Serial UDB Extra Current Waypoint Goal Y
	SueWaypointGoalZ       int16  // Serial UDB Extra Current Waypoint Goal Z
	SueAeroX               int16  // Aeroforce in UDB X Axis
	SueAeroY               int16  // Aeroforce in UDB Y Axis
	SueAeroZ               int16  // Aeroforce in UDB Z axis
	SueBaromTemp           int16  // SUE barometer temperature
	SueBatVolt             int16  // SUE battery voltage
	SueBatAmp              int16  // SUE battery current
	SueBatAmpHours         int16  // SUE battery milli amp hours used
	SueDesiredHeight       int16  // Sue autopilot desired height
	SueMemoryStackFree     int16  // Serial UDB Extra Stack Memory Free
}

func (self *SerialUdbExtraF2B) MsgID() uint8 {
	return 171
}

func (self *SerialUdbExtraF2B) MsgName() string {
	return "SerialUdbExtraF2B"
}

func (self *SerialUdbExtraF2B) Pack(p *Packet) error {
	payload := make([]byte, 108)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.SueTime))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.SueFlags))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.SueBaromPress))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.SueBaromAlt))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.SuePwmInput1))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.SuePwmInput2))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.SuePwmInput3))
	binary.LittleEndian.PutUint16(payload[22:], uint16(self.SuePwmInput4))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.SuePwmInput5))
	binary.LittleEndian.PutUint16(payload[26:], uint16(self.SuePwmInput6))
	binary.LittleEndian.PutUint16(payload[28:], uint16(self.SuePwmInput7))
	binary.LittleEndian.PutUint16(payload[30:], uint16(self.SuePwmInput8))
	binary.LittleEndian.PutUint16(payload[32:], uint16(self.SuePwmInput9))
	binary.LittleEndian.PutUint16(payload[34:], uint16(self.SuePwmInput10))
	binary.LittleEndian.PutUint16(payload[36:], uint16(self.SuePwmInput11))
	binary.LittleEndian.PutUint16(payload[38:], uint16(self.SuePwmInput12))
	binary.LittleEndian.PutUint16(payload[40:], uint16(self.SuePwmOutput1))
	binary.LittleEndian.PutUint16(payload[42:], uint16(self.SuePwmOutput2))
	binary.LittleEndian.PutUint16(payload[44:], uint16(self.SuePwmOutput3))
	binary.LittleEndian.PutUint16(payload[46:], uint16(self.SuePwmOutput4))
	binary.LittleEndian.PutUint16(payload[48:], uint16(self.SuePwmOutput5))
	binary.LittleEndian.PutUint16(payload[50:], uint16(self.SuePwmOutput6))
	binary.LittleEndian.PutUint16(payload[52:], uint16(self.SuePwmOutput7))
	binary.LittleEndian.PutUint16(payload[54:], uint16(self.SuePwmOutput8))
	binary.LittleEndian.PutUint16(payload[56:], uint16(self.SuePwmOutput9))
	binary.LittleEndian.PutUint16(payload[58:], uint16(self.SuePwmOutput10))
	binary.LittleEndian.PutUint16(payload[60:], uint16(self.SuePwmOutput11))
	binary.LittleEndian.PutUint16(payload[62:], uint16(self.SuePwmOutput12))
	binary.LittleEndian.PutUint16(payload[64:], uint16(self.SueImuLocationX))
	binary.LittleEndian.PutUint16(payload[66:], uint16(self.SueImuLocationY))
	binary.LittleEndian.PutUint16(payload[68:], uint16(self.SueImuLocationZ))
	binary.LittleEndian.PutUint16(payload[70:], uint16(self.SueLocationErrorEarthX))
	binary.LittleEndian.PutUint16(payload[72:], uint16(self.SueLocationErrorEarthY))
	binary.LittleEndian.PutUint16(payload[74:], uint16(self.SueLocationErrorEarthZ))
	binary.LittleEndian.PutUint16(payload[76:], uint16(self.SueOscFails))
	binary.LittleEndian.PutUint16(payload[78:], uint16(self.SueImuVelocityX))
	binary.LittleEndian.PutUint16(payload[80:], uint16(self.SueImuVelocityY))
	binary.LittleEndian.PutUint16(payload[82:], uint16(self.SueImuVelocityZ))
	binary.LittleEndian.PutUint16(payload[84:], uint16(self.SueWaypointGoalX))
	binary.LittleEndian.PutUint16(payload[86:], uint16(self.SueWaypointGoalY))
	binary.LittleEndian.PutUint16(payload[88:], uint16(self.SueWaypointGoalZ))
	binary.LittleEndian.PutUint16(payload[90:], uint16(self.SueAeroX))
	binary.LittleEndian.PutUint16(payload[92:], uint16(self.SueAeroY))
	binary.LittleEndian.PutUint16(payload[94:], uint16(self.SueAeroZ))
	binary.LittleEndian.PutUint16(payload[96:], uint16(self.SueBaromTemp))
	binary.LittleEndian.PutUint16(payload[98:], uint16(self.SueBatVolt))
	binary.LittleEndian.PutUint16(payload[100:], uint16(self.SueBatAmp))
	binary.LittleEndian.PutUint16(payload[102:], uint16(self.SueBatAmpHours))
	binary.LittleEndian.PutUint16(payload[104:], uint16(self.SueDesiredHeight))
	binary.LittleEndian.PutUint16(payload[106:], uint16(self.SueMemoryStackFree))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialUdbExtraF2B) Unpack(p *Packet) error {
	if len(p.Payload) < 108 {
		return fmt.Errorf("payload too small")
	}
	self.SueTime = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.SueFlags = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.SueBaromPress = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.SueBaromAlt = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.SuePwmInput1 = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.SuePwmInput2 = int16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.SuePwmInput3 = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.SuePwmInput4 = int16(binary.LittleEndian.Uint16(p.Payload[22:]))
	self.SuePwmInput5 = int16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.SuePwmInput6 = int16(binary.LittleEndian.Uint16(p.Payload[26:]))
	self.SuePwmInput7 = int16(binary.LittleEndian.Uint16(p.Payload[28:]))
	self.SuePwmInput8 = int16(binary.LittleEndian.Uint16(p.Payload[30:]))
	self.SuePwmInput9 = int16(binary.LittleEndian.Uint16(p.Payload[32:]))
	self.SuePwmInput10 = int16(binary.LittleEndian.Uint16(p.Payload[34:]))
	self.SuePwmInput11 = int16(binary.LittleEndian.Uint16(p.Payload[36:]))
	self.SuePwmInput12 = int16(binary.LittleEndian.Uint16(p.Payload[38:]))
	self.SuePwmOutput1 = int16(binary.LittleEndian.Uint16(p.Payload[40:]))
	self.SuePwmOutput2 = int16(binary.LittleEndian.Uint16(p.Payload[42:]))
	self.SuePwmOutput3 = int16(binary.LittleEndian.Uint16(p.Payload[44:]))
	self.SuePwmOutput4 = int16(binary.LittleEndian.Uint16(p.Payload[46:]))
	self.SuePwmOutput5 = int16(binary.LittleEndian.Uint16(p.Payload[48:]))
	self.SuePwmOutput6 = int16(binary.LittleEndian.Uint16(p.Payload[50:]))
	self.SuePwmOutput7 = int16(binary.LittleEndian.Uint16(p.Payload[52:]))
	self.SuePwmOutput8 = int16(binary.LittleEndian.Uint16(p.Payload[54:]))
	self.SuePwmOutput9 = int16(binary.LittleEndian.Uint16(p.Payload[56:]))
	self.SuePwmOutput10 = int16(binary.LittleEndian.Uint16(p.Payload[58:]))
	self.SuePwmOutput11 = int16(binary.LittleEndian.Uint16(p.Payload[60:]))
	self.SuePwmOutput12 = int16(binary.LittleEndian.Uint16(p.Payload[62:]))
	self.SueImuLocationX = int16(binary.LittleEndian.Uint16(p.Payload[64:]))
	self.SueImuLocationY = int16(binary.LittleEndian.Uint16(p.Payload[66:]))
	self.SueImuLocationZ = int16(binary.LittleEndian.Uint16(p.Payload[68:]))
	self.SueLocationErrorEarthX = int16(binary.LittleEndian.Uint16(p.Payload[70:]))
	self.SueLocationErrorEarthY = int16(binary.LittleEndian.Uint16(p.Payload[72:]))
	self.SueLocationErrorEarthZ = int16(binary.LittleEndian.Uint16(p.Payload[74:]))
	self.SueOscFails = int16(binary.LittleEndian.Uint16(p.Payload[76:]))
	self.SueImuVelocityX = int16(binary.LittleEndian.Uint16(p.Payload[78:]))
	self.SueImuVelocityY = int16(binary.LittleEndian.Uint16(p.Payload[80:]))
	self.SueImuVelocityZ = int16(binary.LittleEndian.Uint16(p.Payload[82:]))
	self.SueWaypointGoalX = int16(binary.LittleEndian.Uint16(p.Payload[84:]))
	self.SueWaypointGoalY = int16(binary.LittleEndian.Uint16(p.Payload[86:]))
	self.SueWaypointGoalZ = int16(binary.LittleEndian.Uint16(p.Payload[88:]))
	self.SueAeroX = int16(binary.LittleEndian.Uint16(p.Payload[90:]))
	self.SueAeroY = int16(binary.LittleEndian.Uint16(p.Payload[92:]))
	self.SueAeroZ = int16(binary.LittleEndian.Uint16(p.Payload[94:]))
	self.SueBaromTemp = int16(binary.LittleEndian.Uint16(p.Payload[96:]))
	self.SueBatVolt = int16(binary.LittleEndian.Uint16(p.Payload[98:]))
	self.SueBatAmp = int16(binary.LittleEndian.Uint16(p.Payload[100:]))
	self.SueBatAmpHours = int16(binary.LittleEndian.Uint16(p.Payload[102:]))
	self.SueDesiredHeight = int16(binary.LittleEndian.Uint16(p.Payload[104:]))
	self.SueMemoryStackFree = int16(binary.LittleEndian.Uint16(p.Payload[106:]))
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F4: format
type SerialUdbExtraF4 struct {
	SueRollStabilizationAilerons uint8 // Serial UDB Extra Roll Stabilization with Ailerons Enabled
	SueRollStabilizationRudder   uint8 // Serial UDB Extra Roll Stabilization with Rudder Enabled
	SuePitchStabilization        uint8 // Serial UDB Extra Pitch Stabilization Enabled
	SueYawStabilizationRudder    uint8 // Serial UDB Extra Yaw Stabilization using Rudder Enabled
	SueYawStabilizationAileron   uint8 // Serial UDB Extra Yaw Stabilization using Ailerons Enabled
	SueAileronNavigation         uint8 // Serial UDB Extra Navigation with Ailerons Enabled
	SueRudderNavigation          uint8 // Serial UDB Extra Navigation with Rudder Enabled
	SueAltitudeholdStabilized    uint8 // Serial UDB Extra Type of Alitude Hold when in Stabilized Mode
	SueAltitudeholdWaypoint      uint8 // Serial UDB Extra Type of Alitude Hold when in Waypoint Mode
	SueRacingMode                uint8 // Serial UDB Extra Firmware racing mode enabled
}

func (self *SerialUdbExtraF4) MsgID() uint8 {
	return 172
}

func (self *SerialUdbExtraF4) MsgName() string {
	return "SerialUdbExtraF4"
}

func (self *SerialUdbExtraF4) Pack(p *Packet) error {
	payload := make([]byte, 10)
	payload[0] = byte(self.SueRollStabilizationAilerons)
	payload[1] = byte(self.SueRollStabilizationRudder)
	payload[2] = byte(self.SuePitchStabilization)
	payload[3] = byte(self.SueYawStabilizationRudder)
	payload[4] = byte(self.SueYawStabilizationAileron)
	payload[5] = byte(self.SueAileronNavigation)
	payload[6] = byte(self.SueRudderNavigation)
	payload[7] = byte(self.SueAltitudeholdStabilized)
	payload[8] = byte(self.SueAltitudeholdWaypoint)
	payload[9] = byte(self.SueRacingMode)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialUdbExtraF4) Unpack(p *Packet) error {
	if len(p.Payload) < 10 {
		return fmt.Errorf("payload too small")
	}
	self.SueRollStabilizationAilerons = uint8(p.Payload[0])
	self.SueRollStabilizationRudder = uint8(p.Payload[1])
	self.SuePitchStabilization = uint8(p.Payload[2])
	self.SueYawStabilizationRudder = uint8(p.Payload[3])
	self.SueYawStabilizationAileron = uint8(p.Payload[4])
	self.SueAileronNavigation = uint8(p.Payload[5])
	self.SueRudderNavigation = uint8(p.Payload[6])
	self.SueAltitudeholdStabilized = uint8(p.Payload[7])
	self.SueAltitudeholdWaypoint = uint8(p.Payload[8])
	self.SueRacingMode = uint8(p.Payload[9])
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F5: format
type SerialUdbExtraF5 struct {
	SueYawkpAileron float32 // Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
	SueYawkdAileron float32 // Serial UDB YAWKD_AILERON Gain for Rate control of navigation
	SueRollkp       float32 // Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
	SueRollkd       float32 // Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
}

func (self *SerialUdbExtraF5) MsgID() uint8 {
	return 173
}

func (self *SerialUdbExtraF5) MsgName() string {
	return "SerialUdbExtraF5"
}

func (self *SerialUdbExtraF5) Pack(p *Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.SueYawkpAileron))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.SueYawkdAileron))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.SueRollkp))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.SueRollkd))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialUdbExtraF5) Unpack(p *Packet) error {
	if len(p.Payload) < 16 {
		return fmt.Errorf("payload too small")
	}
	self.SueYawkpAileron = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.SueYawkdAileron = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.SueRollkp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.SueRollkd = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F6: format
type SerialUdbExtraF6 struct {
	SuePitchgain     float32 // Serial UDB Extra PITCHGAIN Proportional Control
	SuePitchkd       float32 // Serial UDB Extra Pitch Rate Control
	SueRudderElevMix float32 // Serial UDB Extra Rudder to Elevator Mix
	SueRollElevMix   float32 // Serial UDB Extra Roll to Elevator Mix
	SueElevatorBoost float32 // Gain For Boosting Manual Elevator control When Plane Stabilized
}

func (self *SerialUdbExtraF6) MsgID() uint8 {
	return 174
}

func (self *SerialUdbExtraF6) MsgName() string {
	return "SerialUdbExtraF6"
}

func (self *SerialUdbExtraF6) Pack(p *Packet) error {
	payload := make([]byte, 20)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.SuePitchgain))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.SuePitchkd))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.SueRudderElevMix))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.SueRollElevMix))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.SueElevatorBoost))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialUdbExtraF6) Unpack(p *Packet) error {
	if len(p.Payload) < 20 {
		return fmt.Errorf("payload too small")
	}
	self.SuePitchgain = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.SuePitchkd = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.SueRudderElevMix = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.SueRollElevMix = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.SueElevatorBoost = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F7: format
type SerialUdbExtraF7 struct {
	SueYawkpRudder  float32 // Serial UDB YAWKP_RUDDER Gain for Proporional control of navigation
	SueYawkdRudder  float32 // Serial UDB YAWKD_RUDDER Gain for Rate control of navigation
	SueRollkpRudder float32 // Serial UDB Extra ROLLKP_RUDDER Gain for Proportional control of roll stabilization
	SueRollkdRudder float32 // Serial UDB Extra ROLLKD_RUDDER Gain for Rate control of roll stabilization
	SueRudderBoost  float32 // SERIAL UDB EXTRA Rudder Boost Gain to Manual Control when stabilized
	SueRtlPitchDown float32 // Serial UDB Extra Return To Landing - Angle to Pitch Plane Down
}

func (self *SerialUdbExtraF7) MsgID() uint8 {
	return 175
}

func (self *SerialUdbExtraF7) MsgName() string {
	return "SerialUdbExtraF7"
}

func (self *SerialUdbExtraF7) Pack(p *Packet) error {
	payload := make([]byte, 24)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.SueYawkpRudder))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.SueYawkdRudder))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.SueRollkpRudder))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.SueRollkdRudder))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.SueRudderBoost))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.SueRtlPitchDown))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialUdbExtraF7) Unpack(p *Packet) error {
	if len(p.Payload) < 24 {
		return fmt.Errorf("payload too small")
	}
	self.SueYawkpRudder = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.SueYawkdRudder = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.SueRollkpRudder = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.SueRollkdRudder = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.SueRudderBoost = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.SueRtlPitchDown = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F8: format
type SerialUdbExtraF8 struct {
	SueHeightTargetMax    float32 // Serial UDB Extra HEIGHT_TARGET_MAX
	SueHeightTargetMin    float32 // Serial UDB Extra HEIGHT_TARGET_MIN
	SueAltHoldThrottleMin float32 // Serial UDB Extra ALT_HOLD_THROTTLE_MIN
	SueAltHoldThrottleMax float32 // Serial UDB Extra ALT_HOLD_THROTTLE_MAX
	SueAltHoldPitchMin    float32 // Serial UDB Extra ALT_HOLD_PITCH_MIN
	SueAltHoldPitchMax    float32 // Serial UDB Extra ALT_HOLD_PITCH_MAX
	SueAltHoldPitchHigh   float32 // Serial UDB Extra ALT_HOLD_PITCH_HIGH
}

func (self *SerialUdbExtraF8) MsgID() uint8 {
	return 176
}

func (self *SerialUdbExtraF8) MsgName() string {
	return "SerialUdbExtraF8"
}

func (self *SerialUdbExtraF8) Pack(p *Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.SueHeightTargetMax))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.SueHeightTargetMin))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.SueAltHoldThrottleMin))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.SueAltHoldThrottleMax))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.SueAltHoldPitchMin))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.SueAltHoldPitchMax))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.SueAltHoldPitchHigh))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialUdbExtraF8) Unpack(p *Packet) error {
	if len(p.Payload) < 28 {
		return fmt.Errorf("payload too small")
	}
	self.SueHeightTargetMax = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.SueHeightTargetMin = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.SueAltHoldThrottleMin = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.SueAltHoldThrottleMax = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.SueAltHoldPitchMin = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.SueAltHoldPitchMax = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.SueAltHoldPitchHigh = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F13: format
type SerialUdbExtraF13 struct {
	SueLatOrigin int32 // Serial UDB Extra MP Origin Latitude
	SueLonOrigin int32 // Serial UDB Extra MP Origin Longitude
	SueAltOrigin int32 // Serial UDB Extra MP Origin Altitude Above Sea Level
	SueWeekNo    int16 // Serial UDB Extra GPS Week Number
}

func (self *SerialUdbExtraF13) MsgID() uint8 {
	return 177
}

func (self *SerialUdbExtraF13) MsgName() string {
	return "SerialUdbExtraF13"
}

func (self *SerialUdbExtraF13) Pack(p *Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.SueLatOrigin))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.SueLonOrigin))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.SueAltOrigin))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.SueWeekNo))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialUdbExtraF13) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	self.SueLatOrigin = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.SueLonOrigin = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.SueAltOrigin = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.SueWeekNo = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F14: format
type SerialUdbExtraF14 struct {
	SueTrapSource     uint32 // Serial UDB Extra Type Program Address of Last Trap
	SueRcon           int16  // Serial UDB Extra Reboot Register of DSPIC
	SueTrapFlags      int16  // Serial UDB Extra  Last dspic Trap Flags
	SueOscFailCount   int16  // Serial UDB Extra Number of Ocillator Failures
	SueWindEstimation uint8  // Serial UDB Extra Wind Estimation Enabled
	SueGpsType        uint8  // Serial UDB Extra Type of GPS Unit
	SueDr             uint8  // Serial UDB Extra Dead Reckoning Enabled
	SueBoardType      uint8  // Serial UDB Extra Type of UDB Hardware
	SueAirframe       uint8  // Serial UDB Extra Type of Airframe
	SueClockConfig    uint8  // Serial UDB Extra UDB Internal Clock Configuration
	SueFlightPlanType uint8  // Serial UDB Extra Type of Flight Plan
}

func (self *SerialUdbExtraF14) MsgID() uint8 {
	return 178
}

func (self *SerialUdbExtraF14) MsgName() string {
	return "SerialUdbExtraF14"
}

func (self *SerialUdbExtraF14) Pack(p *Packet) error {
	payload := make([]byte, 17)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.SueTrapSource))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.SueRcon))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.SueTrapFlags))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.SueOscFailCount))
	payload[10] = byte(self.SueWindEstimation)
	payload[11] = byte(self.SueGpsType)
	payload[12] = byte(self.SueDr)
	payload[13] = byte(self.SueBoardType)
	payload[14] = byte(self.SueAirframe)
	payload[15] = byte(self.SueClockConfig)
	payload[16] = byte(self.SueFlightPlanType)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialUdbExtraF14) Unpack(p *Packet) error {
	if len(p.Payload) < 17 {
		return fmt.Errorf("payload too small")
	}
	self.SueTrapSource = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.SueRcon = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.SueTrapFlags = int16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.SueOscFailCount = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.SueWindEstimation = uint8(p.Payload[10])
	self.SueGpsType = uint8(p.Payload[11])
	self.SueDr = uint8(p.Payload[12])
	self.SueBoardType = uint8(p.Payload[13])
	self.SueAirframe = uint8(p.Payload[14])
	self.SueClockConfig = uint8(p.Payload[15])
	self.SueFlightPlanType = uint8(p.Payload[16])
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F15 format
type SerialUdbExtraF15 struct {
	SueIdVehicleModelName    [40]uint8 // Serial UDB Extra Model Name Of Vehicle
	SueIdVehicleRegistration [20]uint8 // Serial UDB Extra Registraton Number of Vehicle
}

func (self *SerialUdbExtraF15) MsgID() uint8 {
	return 179
}

func (self *SerialUdbExtraF15) MsgName() string {
	return "SerialUdbExtraF15"
}

func (self *SerialUdbExtraF15) Pack(p *Packet) error {
	payload := make([]byte, 60)
	copy(payload[0:], self.SueIdVehicleModelName[:])
	copy(payload[40:], self.SueIdVehicleRegistration[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialUdbExtraF15) Unpack(p *Packet) error {
	if len(p.Payload) < 60 {
		return fmt.Errorf("payload too small")
	}
	copy(self.SueIdVehicleModelName[:], p.Payload[0:40])
	copy(self.SueIdVehicleRegistration[:], p.Payload[40:60])
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F16 format
type SerialUdbExtraF16 struct {
	SueIdLeadPilot    [40]uint8 // Serial UDB Extra Name of Expected Lead Pilot
	SueIdDiyDronesUrl [70]uint8 // Serial UDB Extra URL of Lead Pilot or Team
}

func (self *SerialUdbExtraF16) MsgID() uint8 {
	return 180
}

func (self *SerialUdbExtraF16) MsgName() string {
	return "SerialUdbExtraF16"
}

func (self *SerialUdbExtraF16) Pack(p *Packet) error {
	payload := make([]byte, 110)
	copy(payload[0:], self.SueIdLeadPilot[:])
	copy(payload[40:], self.SueIdDiyDronesUrl[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialUdbExtraF16) Unpack(p *Packet) error {
	if len(p.Payload) < 110 {
		return fmt.Errorf("payload too small")
	}
	copy(self.SueIdLeadPilot[:], p.Payload[0:40])
	copy(self.SueIdDiyDronesUrl[:], p.Payload[40:110])
	return nil
}

// The altitude measured by sensors and IMU
type Altitudes struct {
	TimeBootMs     uint32 // Timestamp (milliseconds since system boot)
	AltGps         int32  // GPS altitude in meters, expressed as * 1000 (millimeters), above MSL
	AltImu         int32  // IMU altitude above ground in meters, expressed as * 1000 (millimeters)
	AltBarometric  int32  // barometeric altitude above ground in meters, expressed as * 1000 (millimeters)
	AltOpticalFlow int32  // Optical flow altitude above ground in meters, expressed as * 1000 (millimeters)
	AltRangeFinder int32  // Rangefinder Altitude above ground in meters, expressed as * 1000 (millimeters)
	AltExtra       int32  // Extra altitude above ground in meters, expressed as * 1000 (millimeters)
}

func (self *Altitudes) MsgID() uint8 {
	return 181
}

func (self *Altitudes) MsgName() string {
	return "Altitudes"
}

func (self *Altitudes) Pack(p *Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.AltGps))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.AltImu))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.AltBarometric))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.AltOpticalFlow))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.AltRangeFinder))
	binary.LittleEndian.PutUint32(payload[24:], uint32(self.AltExtra))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Altitudes) Unpack(p *Packet) error {
	if len(p.Payload) < 28 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.AltGps = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.AltImu = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.AltBarometric = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.AltOpticalFlow = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.AltRangeFinder = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.AltExtra = int32(binary.LittleEndian.Uint32(p.Payload[24:]))
	return nil
}

// The airspeed measured by sensors and IMU
type Airspeeds struct {
	TimeBootMs         uint32 // Timestamp (milliseconds since system boot)
	AirspeedImu        int16  // Airspeed estimate from IMU, cm/s
	AirspeedPitot      int16  // Pitot measured forward airpseed, cm/s
	AirspeedHotWire    int16  // Hot wire anenometer measured airspeed, cm/s
	AirspeedUltrasonic int16  // Ultrasonic measured airspeed, cm/s
	Aoa                int16  // Angle of attack sensor, degrees * 10
	Aoy                int16  // Yaw angle sensor, degrees * 10
}

func (self *Airspeeds) MsgID() uint8 {
	return 182
}

func (self *Airspeeds) MsgName() string {
	return "Airspeeds"
}

func (self *Airspeeds) Pack(p *Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.AirspeedImu))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.AirspeedPitot))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.AirspeedHotWire))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.AirspeedUltrasonic))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Aoa))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.Aoy))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Airspeeds) Unpack(p *Packet) error {
	if len(p.Payload) < 16 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.AirspeedImu = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.AirspeedPitot = int16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.AirspeedHotWire = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.AirspeedUltrasonic = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.Aoa = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Aoy = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F17 format
type SerialUdbExtraF17 struct {
	SueFeedForward float32 // SUE Feed Forward Gain
	SueTurnRateNav float32 // SUE Max Turn Rate when Navigating
	SueTurnRateFbw float32 // SUE Max Turn Rate in Fly By Wire Mode
}

func (self *SerialUdbExtraF17) MsgID() uint8 {
	return 183
}

func (self *SerialUdbExtraF17) MsgName() string {
	return "SerialUdbExtraF17"
}

func (self *SerialUdbExtraF17) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.SueFeedForward))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.SueTurnRateNav))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.SueTurnRateFbw))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialUdbExtraF17) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	self.SueFeedForward = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.SueTurnRateNav = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.SueTurnRateFbw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F18 format
type SerialUdbExtraF18 struct {
	AngleOfAttackNormal   float32 // SUE Angle of Attack Normal
	AngleOfAttackInverted float32 // SUE Angle of Attack Inverted
	ElevatorTrimNormal    float32 // SUE Elevator Trim Normal
	ElevatorTrimInverted  float32 // SUE Elevator Trim Inverted
	ReferenceSpeed        float32 // SUE reference_speed
}

func (self *SerialUdbExtraF18) MsgID() uint8 {
	return 184
}

func (self *SerialUdbExtraF18) MsgName() string {
	return "SerialUdbExtraF18"
}

func (self *SerialUdbExtraF18) Pack(p *Packet) error {
	payload := make([]byte, 20)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.AngleOfAttackNormal))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.AngleOfAttackInverted))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.ElevatorTrimNormal))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.ElevatorTrimInverted))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.ReferenceSpeed))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialUdbExtraF18) Unpack(p *Packet) error {
	if len(p.Payload) < 20 {
		return fmt.Errorf("payload too small")
	}
	self.AngleOfAttackNormal = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.AngleOfAttackInverted = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.ElevatorTrimNormal = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.ElevatorTrimInverted = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.ReferenceSpeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F19 format
type SerialUdbExtraF19 struct {
	SueAileronOutputChannel  uint8 // SUE aileron output channel
	SueAileronReversed       uint8 // SUE aileron reversed
	SueElevatorOutputChannel uint8 // SUE elevator output channel
	SueElevatorReversed      uint8 // SUE elevator reversed
	SueThrottleOutputChannel uint8 // SUE throttle output channel
	SueThrottleReversed      uint8 // SUE throttle reversed
	SueRudderOutputChannel   uint8 // SUE rudder output channel
	SueRudderReversed        uint8 // SUE rudder reversed
}

func (self *SerialUdbExtraF19) MsgID() uint8 {
	return 185
}

func (self *SerialUdbExtraF19) MsgName() string {
	return "SerialUdbExtraF19"
}

func (self *SerialUdbExtraF19) Pack(p *Packet) error {
	payload := make([]byte, 8)
	payload[0] = byte(self.SueAileronOutputChannel)
	payload[1] = byte(self.SueAileronReversed)
	payload[2] = byte(self.SueElevatorOutputChannel)
	payload[3] = byte(self.SueElevatorReversed)
	payload[4] = byte(self.SueThrottleOutputChannel)
	payload[5] = byte(self.SueThrottleReversed)
	payload[6] = byte(self.SueRudderOutputChannel)
	payload[7] = byte(self.SueRudderReversed)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialUdbExtraF19) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	self.SueAileronOutputChannel = uint8(p.Payload[0])
	self.SueAileronReversed = uint8(p.Payload[1])
	self.SueElevatorOutputChannel = uint8(p.Payload[2])
	self.SueElevatorReversed = uint8(p.Payload[3])
	self.SueThrottleOutputChannel = uint8(p.Payload[4])
	self.SueThrottleReversed = uint8(p.Payload[5])
	self.SueRudderOutputChannel = uint8(p.Payload[6])
	self.SueRudderReversed = uint8(p.Payload[7])
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F20 format
type SerialUdbExtraF20 struct {
	SueTrimValueInput1  int16 // SUE UDB PWM Trim Value on Input 1
	SueTrimValueInput2  int16 // SUE UDB PWM Trim Value on Input 2
	SueTrimValueInput3  int16 // SUE UDB PWM Trim Value on Input 3
	SueTrimValueInput4  int16 // SUE UDB PWM Trim Value on Input 4
	SueTrimValueInput5  int16 // SUE UDB PWM Trim Value on Input 5
	SueTrimValueInput6  int16 // SUE UDB PWM Trim Value on Input 6
	SueTrimValueInput7  int16 // SUE UDB PWM Trim Value on Input 7
	SueTrimValueInput8  int16 // SUE UDB PWM Trim Value on Input 8
	SueTrimValueInput9  int16 // SUE UDB PWM Trim Value on Input 9
	SueTrimValueInput10 int16 // SUE UDB PWM Trim Value on Input 10
	SueTrimValueInput11 int16 // SUE UDB PWM Trim Value on Input 11
	SueTrimValueInput12 int16 // SUE UDB PWM Trim Value on Input 12
	SueNumberOfInputs   uint8 // SUE Number of Input Channels
}

func (self *SerialUdbExtraF20) MsgID() uint8 {
	return 186
}

func (self *SerialUdbExtraF20) MsgName() string {
	return "SerialUdbExtraF20"
}

func (self *SerialUdbExtraF20) Pack(p *Packet) error {
	payload := make([]byte, 25)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.SueTrimValueInput1))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.SueTrimValueInput2))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.SueTrimValueInput3))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.SueTrimValueInput4))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.SueTrimValueInput5))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.SueTrimValueInput6))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.SueTrimValueInput7))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.SueTrimValueInput8))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.SueTrimValueInput9))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.SueTrimValueInput10))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.SueTrimValueInput11))
	binary.LittleEndian.PutUint16(payload[22:], uint16(self.SueTrimValueInput12))
	payload[24] = byte(self.SueNumberOfInputs)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialUdbExtraF20) Unpack(p *Packet) error {
	if len(p.Payload) < 25 {
		return fmt.Errorf("payload too small")
	}
	self.SueTrimValueInput1 = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.SueTrimValueInput2 = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.SueTrimValueInput3 = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.SueTrimValueInput4 = int16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.SueTrimValueInput5 = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.SueTrimValueInput6 = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.SueTrimValueInput7 = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.SueTrimValueInput8 = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	self.SueTrimValueInput9 = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.SueTrimValueInput10 = int16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.SueTrimValueInput11 = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.SueTrimValueInput12 = int16(binary.LittleEndian.Uint16(p.Payload[22:]))
	self.SueNumberOfInputs = uint8(p.Payload[24])
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F21 format
type SerialUdbExtraF21 struct {
	SueAccelXOffset int16 // SUE X accelerometer offset
	SueAccelYOffset int16 // SUE Y accelerometer offset
	SueAccelZOffset int16 // SUE Z accelerometer offset
	SueGyroXOffset  int16 // SUE X gyro offset
	SueGyroYOffset  int16 // SUE Y gyro offset
	SueGyroZOffset  int16 // SUE Z gyro offset
}

func (self *SerialUdbExtraF21) MsgID() uint8 {
	return 187
}

func (self *SerialUdbExtraF21) MsgName() string {
	return "SerialUdbExtraF21"
}

func (self *SerialUdbExtraF21) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.SueAccelXOffset))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.SueAccelYOffset))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.SueAccelZOffset))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.SueGyroXOffset))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.SueGyroYOffset))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.SueGyroZOffset))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialUdbExtraF21) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	self.SueAccelXOffset = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.SueAccelYOffset = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.SueAccelZOffset = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.SueGyroXOffset = int16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.SueGyroYOffset = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.SueGyroZOffset = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F22 format
type SerialUdbExtraF22 struct {
	SueAccelXAtCalibration int16 // SUE X accelerometer at calibration time
	SueAccelYAtCalibration int16 // SUE Y accelerometer at calibration time
	SueAccelZAtCalibration int16 // SUE Z accelerometer at calibration time
	SueGyroXAtCalibration  int16 // SUE X gyro at calibration time
	SueGyroYAtCalibration  int16 // SUE Y gyro at calibration time
	SueGyroZAtCalibration  int16 // SUE Z gyro at calibration time
}

func (self *SerialUdbExtraF22) MsgID() uint8 {
	return 188
}

func (self *SerialUdbExtraF22) MsgName() string {
	return "SerialUdbExtraF22"
}

func (self *SerialUdbExtraF22) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.SueAccelXAtCalibration))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.SueAccelYAtCalibration))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.SueAccelZAtCalibration))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.SueGyroXAtCalibration))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.SueGyroYAtCalibration))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.SueGyroZAtCalibration))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialUdbExtraF22) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	self.SueAccelXAtCalibration = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.SueAccelYAtCalibration = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.SueAccelZAtCalibration = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.SueGyroXAtCalibration = int16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.SueGyroYAtCalibration = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.SueGyroZAtCalibration = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	return nil
}

// Message IDs
const (
	MSG_ID_FLEXIFUNCTION_SET                 = 150
	MSG_ID_FLEXIFUNCTION_READ_REQ            = 151
	MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION     = 152
	MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK = 153
	MSG_ID_FLEXIFUNCTION_DIRECTORY           = 155
	MSG_ID_FLEXIFUNCTION_DIRECTORY_ACK       = 156
	MSG_ID_FLEXIFUNCTION_COMMAND             = 157
	MSG_ID_FLEXIFUNCTION_COMMAND_ACK         = 158
	MSG_ID_SERIAL_UDB_EXTRA_F2_A             = 170
	MSG_ID_SERIAL_UDB_EXTRA_F2_B             = 171
	MSG_ID_SERIAL_UDB_EXTRA_F4               = 172
	MSG_ID_SERIAL_UDB_EXTRA_F5               = 173
	MSG_ID_SERIAL_UDB_EXTRA_F6               = 174
	MSG_ID_SERIAL_UDB_EXTRA_F7               = 175
	MSG_ID_SERIAL_UDB_EXTRA_F8               = 176
	MSG_ID_SERIAL_UDB_EXTRA_F13              = 177
	MSG_ID_SERIAL_UDB_EXTRA_F14              = 178
	MSG_ID_SERIAL_UDB_EXTRA_F15              = 179
	MSG_ID_SERIAL_UDB_EXTRA_F16              = 180
	MSG_ID_ALTITUDES                         = 181
	MSG_ID_AIRSPEEDS                         = 182
	MSG_ID_SERIAL_UDB_EXTRA_F17              = 183
	MSG_ID_SERIAL_UDB_EXTRA_F18              = 184
	MSG_ID_SERIAL_UDB_EXTRA_F19              = 185
	MSG_ID_SERIAL_UDB_EXTRA_F20              = 186
	MSG_ID_SERIAL_UDB_EXTRA_F21              = 187
	MSG_ID_SERIAL_UDB_EXTRA_F22              = 188
)

// DialectMatrixpilot is the dialect represented by matrixpilot.xml
var DialectMatrixpilot *Dialect = &Dialect{
	Name: "matrixpilot",
	crcExtras: map[uint8]uint8{
		150: 181, // MSG_ID_FLEXIFUNCTION_SET
		151: 26,  // MSG_ID_FLEXIFUNCTION_READ_REQ
		152: 101, // MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION
		153: 109, // MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK
		155: 12,  // MSG_ID_FLEXIFUNCTION_DIRECTORY
		156: 218, // MSG_ID_FLEXIFUNCTION_DIRECTORY_ACK
		157: 133, // MSG_ID_FLEXIFUNCTION_COMMAND
		158: 208, // MSG_ID_FLEXIFUNCTION_COMMAND_ACK
		170: 103, // MSG_ID_SERIAL_UDB_EXTRA_F2_A
		171: 245, // MSG_ID_SERIAL_UDB_EXTRA_F2_B
		172: 191, // MSG_ID_SERIAL_UDB_EXTRA_F4
		173: 54,  // MSG_ID_SERIAL_UDB_EXTRA_F5
		174: 54,  // MSG_ID_SERIAL_UDB_EXTRA_F6
		175: 171, // MSG_ID_SERIAL_UDB_EXTRA_F7
		176: 142, // MSG_ID_SERIAL_UDB_EXTRA_F8
		177: 249, // MSG_ID_SERIAL_UDB_EXTRA_F13
		178: 123, // MSG_ID_SERIAL_UDB_EXTRA_F14
		179: 7,   // MSG_ID_SERIAL_UDB_EXTRA_F15
		180: 222, // MSG_ID_SERIAL_UDB_EXTRA_F16
		181: 55,  // MSG_ID_ALTITUDES
		182: 154, // MSG_ID_AIRSPEEDS
		183: 175, // MSG_ID_SERIAL_UDB_EXTRA_F17
		184: 41,  // MSG_ID_SERIAL_UDB_EXTRA_F18
		185: 87,  // MSG_ID_SERIAL_UDB_EXTRA_F19
		186: 144, // MSG_ID_SERIAL_UDB_EXTRA_F20
		187: 134, // MSG_ID_SERIAL_UDB_EXTRA_F21
		188: 91,  // MSG_ID_SERIAL_UDB_EXTRA_F22
	},
}
