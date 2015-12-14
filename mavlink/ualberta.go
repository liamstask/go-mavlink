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

// UalbertaAutopilotMode: Available autopilot modes for ualberta uav
const (
	MODE_MANUAL_DIRECT = 0 // Raw input pulse widts sent to output
	MODE_MANUAL_SCALED = 1 // Inputs are normalized using calibration, the converted back to raw pulse widths for output
	MODE_AUTO_PID_ATT  = 2 //  dfsdfs
	MODE_AUTO_PID_VEL  = 3 //  dfsfds
	MODE_AUTO_PID_POS  = 4 //  dfsdfsdfs
)

// UalbertaNavMode: Navigation filter mode
const (
	NAV_AHRS_INIT    = 0 //
	NAV_AHRS         = 1 // AHRS mode
	NAV_INS_GPS_INIT = 2 // INS/GPS initialization mode
	NAV_INS_GPS      = 3 // INS/GPS mode
)

// UalbertaPilotMode: Mode currently commanded by pilot
const (
	PILOT_MANUAL = 0 //  sdf
	PILOT_AUTO   = 1 //  dfs
	PILOT_ROTO   = 2 //  Rotomotion mode
)

// Accelerometer and Gyro biases from the navigation filter
type NavFilterBias struct {
	Usec   uint64  // Timestamp (microseconds)
	Accel0 float32 // b_f[0]
	Accel1 float32 // b_f[1]
	Accel2 float32 // b_f[2]
	Gyro0  float32 // b_f[0]
	Gyro1  float32 // b_f[1]
	Gyro2  float32 // b_f[2]
}

func (self *NavFilterBias) MsgID() uint8 {
	return 220
}

func (self *NavFilterBias) MsgName() string {
	return "NavFilterBias"
}

func (self *NavFilterBias) Pack(p *Packet) error {
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.Usec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Accel0))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Accel1))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Accel2))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Gyro0))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Gyro1))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Gyro2))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *NavFilterBias) Unpack(p *Packet) error {
	if len(p.Payload) < 32 {
		return fmt.Errorf("payload too small")
	}
	self.Usec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Accel0 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Accel1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Accel2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Gyro0 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Gyro1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Gyro2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	return nil
}

// Complete set of calibration parameters for the radio
type RadioCalibration struct {
	Aileron  [3]uint16 // Aileron setpoints: left, center, right
	Elevator [3]uint16 // Elevator setpoints: nose down, center, nose up
	Rudder   [3]uint16 // Rudder setpoints: nose left, center, nose right
	Gyro     [2]uint16 // Tail gyro mode/gain setpoints: heading hold, rate mode
	Pitch    [5]uint16 // Pitch curve setpoints (every 25%)
	Throttle [5]uint16 // Throttle curve setpoints (every 25%)
}

func (self *RadioCalibration) MsgID() uint8 {
	return 221
}

func (self *RadioCalibration) MsgName() string {
	return "RadioCalibration"
}

func (self *RadioCalibration) Pack(p *Packet) error {
	payload := make([]byte, 42)
	for i, v := range self.Aileron {
		binary.LittleEndian.PutUint16(payload[0+i*2:], uint16(v))
	}
	for i, v := range self.Elevator {
		binary.LittleEndian.PutUint16(payload[6+i*2:], uint16(v))
	}
	for i, v := range self.Rudder {
		binary.LittleEndian.PutUint16(payload[12+i*2:], uint16(v))
	}
	for i, v := range self.Gyro {
		binary.LittleEndian.PutUint16(payload[18+i*2:], uint16(v))
	}
	for i, v := range self.Pitch {
		binary.LittleEndian.PutUint16(payload[22+i*2:], uint16(v))
	}
	for i, v := range self.Throttle {
		binary.LittleEndian.PutUint16(payload[32+i*2:], uint16(v))
	}

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RadioCalibration) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}
	for i := 0; i < len(self.Aileron); i++ {
		self.Aileron[i] = uint16(binary.LittleEndian.Uint16(p.Payload[0+i*2:]))
	}
	for i := 0; i < len(self.Elevator); i++ {
		self.Elevator[i] = uint16(binary.LittleEndian.Uint16(p.Payload[6+i*2:]))
	}
	for i := 0; i < len(self.Rudder); i++ {
		self.Rudder[i] = uint16(binary.LittleEndian.Uint16(p.Payload[12+i*2:]))
	}
	for i := 0; i < len(self.Gyro); i++ {
		self.Gyro[i] = uint16(binary.LittleEndian.Uint16(p.Payload[18+i*2:]))
	}
	for i := 0; i < len(self.Pitch); i++ {
		self.Pitch[i] = uint16(binary.LittleEndian.Uint16(p.Payload[22+i*2:]))
	}
	for i := 0; i < len(self.Throttle); i++ {
		self.Throttle[i] = uint16(binary.LittleEndian.Uint16(p.Payload[32+i*2:]))
	}
	return nil
}

// System status specific to ualberta uav
type UalbertaSysStatus struct {
	Mode    uint8 // System mode, see UALBERTA_AUTOPILOT_MODE ENUM
	NavMode uint8 // Navigation mode, see UALBERTA_NAV_MODE ENUM
	Pilot   uint8 // Pilot mode, see UALBERTA_PILOT_MODE
}

func (self *UalbertaSysStatus) MsgID() uint8 {
	return 222
}

func (self *UalbertaSysStatus) MsgName() string {
	return "UalbertaSysStatus"
}

func (self *UalbertaSysStatus) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(self.Mode)
	payload[1] = byte(self.NavMode)
	payload[2] = byte(self.Pilot)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *UalbertaSysStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.Mode = uint8(p.Payload[0])
	self.NavMode = uint8(p.Payload[1])
	self.Pilot = uint8(p.Payload[2])
	return nil
}

// Message IDs
const (
	MSG_ID_NAV_FILTER_BIAS     = 220
	MSG_ID_RADIO_CALIBRATION   = 221
	MSG_ID_UALBERTA_SYS_STATUS = 222
)

// DialectUalberta is the dialect represented by ualberta.xml
var DialectUalberta *Dialect = &Dialect{
	Name: "ualberta",
	crcExtras: map[uint8]uint8{
		220: 34, // MSG_ID_NAV_FILTER_BIAS
		221: 71, // MSG_ID_RADIO_CALIBRATION
		222: 15, // MSG_ID_UALBERTA_SYS_STATUS
	},
}
