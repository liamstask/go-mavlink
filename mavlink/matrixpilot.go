package mavlink

import (
	"bytes"
	"encoding/binary"
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *FlexifunctionSet) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.ReadReqType,
		&self.DataIndex,
		&self.TargetSystem,
		&self.TargetComponent,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *FlexifunctionReadReq) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.ReadReqType,
		&self.DataIndex,
		&self.TargetSystem,
		&self.TargetComponent,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.FuncIndex,
		&self.FuncCount,
		&self.DataAddress,
		&self.DataSize,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Data,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *FlexifunctionBufferFunction) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.FuncIndex,
		&self.FuncCount,
		&self.DataAddress,
		&self.DataSize,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Data,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.FuncIndex,
		&self.Result,
		&self.TargetSystem,
		&self.TargetComponent,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *FlexifunctionBufferFunctionAck) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.FuncIndex,
		&self.Result,
		&self.TargetSystem,
		&self.TargetComponent,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.DirectoryType,
		&self.StartIndex,
		&self.Count,
		&self.DirectoryData,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *FlexifunctionDirectory) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.DirectoryType,
		&self.StartIndex,
		&self.Count,
		&self.DirectoryData,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Result,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.DirectoryType,
		&self.StartIndex,
		&self.Count,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *FlexifunctionDirectoryAck) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Result,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.DirectoryType,
		&self.StartIndex,
		&self.Count,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.CommandType,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *FlexifunctionCommand) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.CommandType,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.CommandType,
		&self.Result,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *FlexifunctionCommandAck) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.CommandType,
		&self.Result,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	SueVoltageMilis   int16  // Serial UDB Extra Voltage in MilliVolts
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.SueTime,
		&self.SueLatitude,
		&self.SueLongitude,
		&self.SueAltitude,
		&self.SueWaypointIndex,
		&self.SueRmat0,
		&self.SueRmat1,
		&self.SueRmat2,
		&self.SueRmat3,
		&self.SueRmat4,
		&self.SueRmat5,
		&self.SueRmat6,
		&self.SueRmat7,
		&self.SueRmat8,
		&self.SueCog,
		&self.SueSog,
		&self.SueCpuLoad,
		&self.SueVoltageMilis,
		&self.SueAirSpeed3dimu,
		&self.SueEstimatedWind0,
		&self.SueEstimatedWind1,
		&self.SueEstimatedWind2,
		&self.SueMagfieldearth0,
		&self.SueMagfieldearth1,
		&self.SueMagfieldearth2,
		&self.SueSvs,
		&self.SueHdop,
		&self.SueStatus,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SerialUdbExtraF2A) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.SueTime,
		&self.SueLatitude,
		&self.SueLongitude,
		&self.SueAltitude,
		&self.SueWaypointIndex,
		&self.SueRmat0,
		&self.SueRmat1,
		&self.SueRmat2,
		&self.SueRmat3,
		&self.SueRmat4,
		&self.SueRmat5,
		&self.SueRmat6,
		&self.SueRmat7,
		&self.SueRmat8,
		&self.SueCog,
		&self.SueSog,
		&self.SueCpuLoad,
		&self.SueVoltageMilis,
		&self.SueAirSpeed3dimu,
		&self.SueEstimatedWind0,
		&self.SueEstimatedWind1,
		&self.SueEstimatedWind2,
		&self.SueMagfieldearth0,
		&self.SueMagfieldearth1,
		&self.SueMagfieldearth2,
		&self.SueSvs,
		&self.SueHdop,
		&self.SueStatus,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA - F2: Part B
type SerialUdbExtraF2B struct {
	SueTime            uint32 // Serial UDB Extra Time
	SueFlags           uint32 // Serial UDB Extra Status Flags
	SuePwmInput1       int16  // Serial UDB Extra PWM Input Channel 1
	SuePwmInput2       int16  // Serial UDB Extra PWM Input Channel 2
	SuePwmInput3       int16  // Serial UDB Extra PWM Input Channel 3
	SuePwmInput4       int16  // Serial UDB Extra PWM Input Channel 4
	SuePwmInput5       int16  // Serial UDB Extra PWM Input Channel 5
	SuePwmInput6       int16  // Serial UDB Extra PWM Input Channel 6
	SuePwmInput7       int16  // Serial UDB Extra PWM Input Channel 7
	SuePwmInput8       int16  // Serial UDB Extra PWM Input Channel 8
	SuePwmInput9       int16  // Serial UDB Extra PWM Input Channel 9
	SuePwmInput10      int16  // Serial UDB Extra PWM Input Channel 10
	SuePwmOutput1      int16  // Serial UDB Extra PWM Output Channel 1
	SuePwmOutput2      int16  // Serial UDB Extra PWM Output Channel 2
	SuePwmOutput3      int16  // Serial UDB Extra PWM Output Channel 3
	SuePwmOutput4      int16  // Serial UDB Extra PWM Output Channel 4
	SuePwmOutput5      int16  // Serial UDB Extra PWM Output Channel 5
	SuePwmOutput6      int16  // Serial UDB Extra PWM Output Channel 6
	SuePwmOutput7      int16  // Serial UDB Extra PWM Output Channel 7
	SuePwmOutput8      int16  // Serial UDB Extra PWM Output Channel 8
	SuePwmOutput9      int16  // Serial UDB Extra PWM Output Channel 9
	SuePwmOutput10     int16  // Serial UDB Extra PWM Output Channel 10
	SueImuLocationX    int16  // Serial UDB Extra IMU Location X
	SueImuLocationY    int16  // Serial UDB Extra IMU Location Y
	SueImuLocationZ    int16  // Serial UDB Extra IMU Location Z
	SueOscFails        int16  // Serial UDB Extra Oscillator Failure Count
	SueImuVelocityX    int16  // Serial UDB Extra IMU Velocity X
	SueImuVelocityY    int16  // Serial UDB Extra IMU Velocity Y
	SueImuVelocityZ    int16  // Serial UDB Extra IMU Velocity Z
	SueWaypointGoalX   int16  // Serial UDB Extra Current Waypoint Goal X
	SueWaypointGoalY   int16  // Serial UDB Extra Current Waypoint Goal Y
	SueWaypointGoalZ   int16  // Serial UDB Extra Current Waypoint Goal Z
	SueMemoryStackFree int16  // Serial UDB Extra Stack Memory Free
}

func (self *SerialUdbExtraF2B) MsgID() uint8 {
	return 171
}

func (self *SerialUdbExtraF2B) MsgName() string {
	return "SerialUdbExtraF2B"
}

func (self *SerialUdbExtraF2B) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.SueTime,
		&self.SueFlags,
		&self.SuePwmInput1,
		&self.SuePwmInput2,
		&self.SuePwmInput3,
		&self.SuePwmInput4,
		&self.SuePwmInput5,
		&self.SuePwmInput6,
		&self.SuePwmInput7,
		&self.SuePwmInput8,
		&self.SuePwmInput9,
		&self.SuePwmInput10,
		&self.SuePwmOutput1,
		&self.SuePwmOutput2,
		&self.SuePwmOutput3,
		&self.SuePwmOutput4,
		&self.SuePwmOutput5,
		&self.SuePwmOutput6,
		&self.SuePwmOutput7,
		&self.SuePwmOutput8,
		&self.SuePwmOutput9,
		&self.SuePwmOutput10,
		&self.SueImuLocationX,
		&self.SueImuLocationY,
		&self.SueImuLocationZ,
		&self.SueOscFails,
		&self.SueImuVelocityX,
		&self.SueImuVelocityY,
		&self.SueImuVelocityZ,
		&self.SueWaypointGoalX,
		&self.SueWaypointGoalY,
		&self.SueWaypointGoalZ,
		&self.SueMemoryStackFree,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SerialUdbExtraF2B) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.SueTime,
		&self.SueFlags,
		&self.SuePwmInput1,
		&self.SuePwmInput2,
		&self.SuePwmInput3,
		&self.SuePwmInput4,
		&self.SuePwmInput5,
		&self.SuePwmInput6,
		&self.SuePwmInput7,
		&self.SuePwmInput8,
		&self.SuePwmInput9,
		&self.SuePwmInput10,
		&self.SuePwmOutput1,
		&self.SuePwmOutput2,
		&self.SuePwmOutput3,
		&self.SuePwmOutput4,
		&self.SuePwmOutput5,
		&self.SuePwmOutput6,
		&self.SuePwmOutput7,
		&self.SuePwmOutput8,
		&self.SuePwmOutput9,
		&self.SuePwmOutput10,
		&self.SueImuLocationX,
		&self.SueImuLocationY,
		&self.SueImuLocationZ,
		&self.SueOscFails,
		&self.SueImuVelocityX,
		&self.SueImuVelocityY,
		&self.SueImuVelocityZ,
		&self.SueWaypointGoalX,
		&self.SueWaypointGoalY,
		&self.SueWaypointGoalZ,
		&self.SueMemoryStackFree,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.SueRollStabilizationAilerons,
		&self.SueRollStabilizationRudder,
		&self.SuePitchStabilization,
		&self.SueYawStabilizationRudder,
		&self.SueYawStabilizationAileron,
		&self.SueAileronNavigation,
		&self.SueRudderNavigation,
		&self.SueAltitudeholdStabilized,
		&self.SueAltitudeholdWaypoint,
		&self.SueRacingMode,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SerialUdbExtraF4) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.SueRollStabilizationAilerons,
		&self.SueRollStabilizationRudder,
		&self.SuePitchStabilization,
		&self.SueYawStabilizationRudder,
		&self.SueYawStabilizationAileron,
		&self.SueAileronNavigation,
		&self.SueRudderNavigation,
		&self.SueAltitudeholdStabilized,
		&self.SueAltitudeholdWaypoint,
		&self.SueRacingMode,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F5: format
type SerialUdbExtraF5 struct {
	SueYawkpAileron            float32 // Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
	SueYawkdAileron            float32 // Serial UDB YAWKD_AILERON Gain for Rate control of navigation
	SueRollkp                  float32 // Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
	SueRollkd                  float32 // Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
	SueYawStabilizationAileron float32 // YAW_STABILIZATION_AILERON Proportional control
	SueAileronBoost            float32 // Gain For Boosting Manual Aileron control When Plane Stabilized
}

func (self *SerialUdbExtraF5) MsgID() uint8 {
	return 173
}

func (self *SerialUdbExtraF5) MsgName() string {
	return "SerialUdbExtraF5"
}

func (self *SerialUdbExtraF5) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.SueYawkpAileron,
		&self.SueYawkdAileron,
		&self.SueRollkp,
		&self.SueRollkd,
		&self.SueYawStabilizationAileron,
		&self.SueAileronBoost,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SerialUdbExtraF5) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.SueYawkpAileron,
		&self.SueYawkdAileron,
		&self.SueRollkp,
		&self.SueRollkd,
		&self.SueYawStabilizationAileron,
		&self.SueAileronBoost,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.SuePitchgain,
		&self.SuePitchkd,
		&self.SueRudderElevMix,
		&self.SueRollElevMix,
		&self.SueElevatorBoost,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SerialUdbExtraF6) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.SuePitchgain,
		&self.SuePitchkd,
		&self.SueRudderElevMix,
		&self.SueRollElevMix,
		&self.SueElevatorBoost,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.SueYawkpRudder,
		&self.SueYawkdRudder,
		&self.SueRollkpRudder,
		&self.SueRollkdRudder,
		&self.SueRudderBoost,
		&self.SueRtlPitchDown,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SerialUdbExtraF7) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.SueYawkpRudder,
		&self.SueYawkdRudder,
		&self.SueRollkpRudder,
		&self.SueRollkdRudder,
		&self.SueRudderBoost,
		&self.SueRtlPitchDown,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.SueHeightTargetMax,
		&self.SueHeightTargetMin,
		&self.SueAltHoldThrottleMin,
		&self.SueAltHoldThrottleMax,
		&self.SueAltHoldPitchMin,
		&self.SueAltHoldPitchMax,
		&self.SueAltHoldPitchHigh,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SerialUdbExtraF8) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.SueHeightTargetMax,
		&self.SueHeightTargetMin,
		&self.SueAltHoldThrottleMin,
		&self.SueAltHoldThrottleMax,
		&self.SueAltHoldPitchMin,
		&self.SueAltHoldPitchMax,
		&self.SueAltHoldPitchHigh,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.SueLatOrigin,
		&self.SueLonOrigin,
		&self.SueAltOrigin,
		&self.SueWeekNo,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SerialUdbExtraF13) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.SueLatOrigin,
		&self.SueLonOrigin,
		&self.SueAltOrigin,
		&self.SueWeekNo,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F14: format
type SerialUdbExtraF14 struct {
	SueTrapSource     uint32 // Serial UDB Extra Type Program Address of Last Trap
	SueRcon           int16  // Serial UDB Extra Reboot Regitster of DSPIC
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.SueTrapSource,
		&self.SueRcon,
		&self.SueTrapFlags,
		&self.SueOscFailCount,
		&self.SueWindEstimation,
		&self.SueGpsType,
		&self.SueDr,
		&self.SueBoardType,
		&self.SueAirframe,
		&self.SueClockConfig,
		&self.SueFlightPlanType,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SerialUdbExtraF14) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.SueTrapSource,
		&self.SueRcon,
		&self.SueTrapFlags,
		&self.SueOscFailCount,
		&self.SueWindEstimation,
		&self.SueGpsType,
		&self.SueDr,
		&self.SueBoardType,
		&self.SueAirframe,
		&self.SueClockConfig,
		&self.SueFlightPlanType,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Backwards compatible version of SERIAL_UDB_EXTRA F15 and F16: format
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.SueIdVehicleModelName,
		&self.SueIdVehicleRegistration,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SerialUdbExtraF15) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.SueIdVehicleModelName,
		&self.SueIdVehicleRegistration,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

//
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.SueIdLeadPilot,
		&self.SueIdDiyDronesUrl,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SerialUdbExtraF16) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.SueIdLeadPilot,
		&self.SueIdDiyDronesUrl,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.AltGps,
		&self.AltImu,
		&self.AltBarometric,
		&self.AltOpticalFlow,
		&self.AltRangeFinder,
		&self.AltExtra,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Altitudes) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.AltGps,
		&self.AltImu,
		&self.AltBarometric,
		&self.AltOpticalFlow,
		&self.AltRangeFinder,
		&self.AltExtra,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.AirspeedImu,
		&self.AirspeedPitot,
		&self.AirspeedHotWire,
		&self.AirspeedUltrasonic,
		&self.Aoa,
		&self.Aoy,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Airspeeds) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.AirspeedImu,
		&self.AirspeedPitot,
		&self.AirspeedHotWire,
		&self.AirspeedUltrasonic,
		&self.Aoa,
		&self.Aoy,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
		170: 150, // MSG_ID_SERIAL_UDB_EXTRA_F2_A
		171: 169, // MSG_ID_SERIAL_UDB_EXTRA_F2_B
		172: 191, // MSG_ID_SERIAL_UDB_EXTRA_F4
		173: 121, // MSG_ID_SERIAL_UDB_EXTRA_F5
		174: 54,  // MSG_ID_SERIAL_UDB_EXTRA_F6
		175: 171, // MSG_ID_SERIAL_UDB_EXTRA_F7
		176: 142, // MSG_ID_SERIAL_UDB_EXTRA_F8
		177: 249, // MSG_ID_SERIAL_UDB_EXTRA_F13
		178: 123, // MSG_ID_SERIAL_UDB_EXTRA_F14
		179: 7,   // MSG_ID_SERIAL_UDB_EXTRA_F15
		180: 222, // MSG_ID_SERIAL_UDB_EXTRA_F16
		181: 55,  // MSG_ID_ALTITUDES
		182: 154, // MSG_ID_AIRSPEEDS
	},
}
