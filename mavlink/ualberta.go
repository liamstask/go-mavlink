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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Usec,
		&self.Accel0,
		&self.Accel1,
		&self.Accel2,
		&self.Gyro0,
		&self.Gyro1,
		&self.Gyro2,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *NavFilterBias) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Usec,
		&self.Accel0,
		&self.Accel1,
		&self.Accel2,
		&self.Gyro0,
		&self.Gyro1,
		&self.Gyro2,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Aileron,
		&self.Elevator,
		&self.Rudder,
		&self.Gyro,
		&self.Pitch,
		&self.Throttle,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *RadioCalibration) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Aileron,
		&self.Elevator,
		&self.Rudder,
		&self.Gyro,
		&self.Pitch,
		&self.Throttle,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Mode,
		&self.NavMode,
		&self.Pilot,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *UalbertaSysStatus) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Mode,
		&self.NavMode,
		&self.Pilot,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
		220: 34,  // MSG_ID_NAV_FILTER_BIAS
		221: 230, // MSG_ID_RADIO_CALIBRATION
		222: 15,  // MSG_ID_UALBERTA_SYS_STATUS
	},
}
