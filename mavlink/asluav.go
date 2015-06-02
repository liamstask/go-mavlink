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

// Voltage and current sensor data
type SensPower struct {
	Adc121VspbVolt float32 //  Power board voltage sensor reading in volts
	Adc121CspbAmp  float32 //  Power board current sensor reading in amps
	Adc121Cs1Amp   float32 //  Board current sensor 1 reading in amps
	Adc121Cs2Amp   float32 //  Board current sensor 2 reading in amps
}

func (self *SensPower) MsgID() uint8 {
	return 201
}

func (self *SensPower) MsgName() string {
	return "SensPower"
}

func (self *SensPower) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Adc121VspbVolt,
		&self.Adc121CspbAmp,
		&self.Adc121Cs1Amp,
		&self.Adc121Cs2Amp,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SensPower) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Adc121VspbVolt,
		&self.Adc121CspbAmp,
		&self.Adc121Cs1Amp,
		&self.Adc121Cs2Amp,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Maximum Power Point Tracker (MPPT) sensor data for solar module power performance tracking
type SensMppt struct {
	MpptTimestamp uint64  //  MPPT last timestamp
	Mppt1Volt     float32 //  MPPT1 voltage
	Mppt1Amp      float32 //  MPPT1 current
	Mppt2Volt     float32 //  MPPT2 voltage
	Mppt2Amp      float32 //  MPPT2 current
	Mppt3Volt     float32 //  MPPT3 voltage
	Mppt3Amp      float32 //  MPPT3 current
	Mppt1Pwm      uint16  //  MPPT1 pwm
	Mppt2Pwm      uint16  //  MPPT2 pwm
	Mppt3Pwm      uint16  //  MPPT3 pwm
	Mppt1Status   uint8   //  MPPT1 status
	Mppt2Status   uint8   //  MPPT2 status
	Mppt3Status   uint8   //  MPPT3 status
}

func (self *SensMppt) MsgID() uint8 {
	return 202
}

func (self *SensMppt) MsgName() string {
	return "SensMppt"
}

func (self *SensMppt) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.MpptTimestamp,
		&self.Mppt1Volt,
		&self.Mppt1Amp,
		&self.Mppt2Volt,
		&self.Mppt2Amp,
		&self.Mppt3Volt,
		&self.Mppt3Amp,
		&self.Mppt1Pwm,
		&self.Mppt2Pwm,
		&self.Mppt3Pwm,
		&self.Mppt1Status,
		&self.Mppt2Status,
		&self.Mppt3Status,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SensMppt) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.MpptTimestamp,
		&self.Mppt1Volt,
		&self.Mppt1Amp,
		&self.Mppt2Volt,
		&self.Mppt2Amp,
		&self.Mppt3Volt,
		&self.Mppt3Amp,
		&self.Mppt1Pwm,
		&self.Mppt2Pwm,
		&self.Mppt3Pwm,
		&self.Mppt1Status,
		&self.Mppt2Status,
		&self.Mppt3Status,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// ASL-fixed-wing controller data
type AslctrlData struct {
	Timestamp       uint64  //  Timestamp
	H               float32 //  See sourcecode for a description of these values...
	Href            float32 //
	HrefT           float32 //
	Pitchangle      float32 // Pitch angle [deg]
	Pitchangleref   float32 // Pitch angle reference[deg]
	Q               float32 //
	Qref            float32 //
	Uelev           float32 //
	Uthrot          float32 //
	Uthrot2         float32 //
	Az              float32 //
	Airspeedref     float32 // Airspeed reference [m/s]
	Yawangle        float32 // Yaw angle [deg]
	Yawangleref     float32 // Yaw angle reference[deg]
	Rollangle       float32 // Roll angle [deg]
	Rollangleref    float32 // Roll angle reference[deg]
	P               float32 //
	Pref            float32 //
	R               float32 //
	Rref            float32 //
	Uail            float32 //
	Urud            float32 //
	AslctrlMode     uint8   //  ASLCTRL control-mode (manual, stabilized, auto, etc...)
	Spoilersengaged uint8   //
}

func (self *AslctrlData) MsgID() uint8 {
	return 203
}

func (self *AslctrlData) MsgName() string {
	return "AslctrlData"
}

func (self *AslctrlData) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Timestamp,
		&self.H,
		&self.Href,
		&self.HrefT,
		&self.Pitchangle,
		&self.Pitchangleref,
		&self.Q,
		&self.Qref,
		&self.Uelev,
		&self.Uthrot,
		&self.Uthrot2,
		&self.Az,
		&self.Airspeedref,
		&self.Yawangle,
		&self.Yawangleref,
		&self.Rollangle,
		&self.Rollangleref,
		&self.P,
		&self.Pref,
		&self.R,
		&self.Rref,
		&self.Uail,
		&self.Urud,
		&self.AslctrlMode,
		&self.Spoilersengaged,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *AslctrlData) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Timestamp,
		&self.H,
		&self.Href,
		&self.HrefT,
		&self.Pitchangle,
		&self.Pitchangleref,
		&self.Q,
		&self.Qref,
		&self.Uelev,
		&self.Uthrot,
		&self.Uthrot2,
		&self.Az,
		&self.Airspeedref,
		&self.Yawangle,
		&self.Yawangleref,
		&self.Rollangle,
		&self.Rollangleref,
		&self.P,
		&self.Pref,
		&self.R,
		&self.Rref,
		&self.Uail,
		&self.Urud,
		&self.AslctrlMode,
		&self.Spoilersengaged,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// ASL-fixed-wing controller debug data
type AslctrlDebug struct {
	I321 uint32  //  Debug data
	F1   float32 //  Debug data
	F2   float32 //  Debug data
	F3   float32 //  Debug data
	F4   float32 //  Debug data
	F5   float32 //  Debug data
	F6   float32 //  Debug data
	F7   float32 //  Debug data
	F8   float32 //  Debug data
	I81  uint8   //  Debug data
	I82  uint8   //  Debug data
}

func (self *AslctrlDebug) MsgID() uint8 {
	return 204
}

func (self *AslctrlDebug) MsgName() string {
	return "AslctrlDebug"
}

func (self *AslctrlDebug) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.I321,
		&self.F1,
		&self.F2,
		&self.F3,
		&self.F4,
		&self.F5,
		&self.F6,
		&self.F7,
		&self.F8,
		&self.I81,
		&self.I82,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *AslctrlDebug) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.I321,
		&self.F1,
		&self.F2,
		&self.F3,
		&self.F4,
		&self.F5,
		&self.F6,
		&self.F7,
		&self.F8,
		&self.I81,
		&self.I82,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Extended state information for ASLUAVs
type AsluavStatus struct {
	MotorRpm     float32  //  Motor RPM
	LedStatus    uint8    //  Status of the position-indicator LEDs
	SatcomStatus uint8    //  Status of the IRIDIUM satellite communication system
	ServoStatus  [8]uint8 //  Status vector for up to 8 servos
}

func (self *AsluavStatus) MsgID() uint8 {
	return 205
}

func (self *AsluavStatus) MsgName() string {
	return "AsluavStatus"
}

func (self *AsluavStatus) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.MotorRpm,
		&self.LedStatus,
		&self.SatcomStatus,
		&self.ServoStatus,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *AsluavStatus) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.MotorRpm,
		&self.LedStatus,
		&self.SatcomStatus,
		&self.ServoStatus,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Extended EKF state estimates for ASLUAVs
type EkfExt struct {
	Timestamp uint64  //  Time since system start [us]
	Windspeed float32 //  Magnitude of wind velocity (in lateral inertial plane) [m/s]
	Winddir   float32 //  Wind heading angle from North [rad]
	Windz     float32 //  Z (Down) component of inertial wind velocity [m/s]
	Airspeed  float32 //  Magnitude of air velocity [m/s]
	Beta      float32 //  Sideslip angle [rad]
	Alpha     float32 //  Angle of attack [rad]
}

func (self *EkfExt) MsgID() uint8 {
	return 206
}

func (self *EkfExt) MsgName() string {
	return "EkfExt"
}

func (self *EkfExt) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Timestamp,
		&self.Windspeed,
		&self.Winddir,
		&self.Windz,
		&self.Airspeed,
		&self.Beta,
		&self.Alpha,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *EkfExt) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Timestamp,
		&self.Windspeed,
		&self.Winddir,
		&self.Windz,
		&self.Airspeed,
		&self.Beta,
		&self.Alpha,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Off-board controls/commands for ASLUAVs
type AslObctrl struct {
	Timestamp    uint64  //  Time since system start [us]
	Uelev        float32 //  Elevator command [~]
	Uthrot       float32 //  Throttle command [~]
	Uthrot2      float32 //  Throttle 2 command [~]
	Uaill        float32 //  Left aileron command [~]
	Uailr        float32 //  Right aileron command [~]
	Urud         float32 //  Rudder command [~]
	ObctrlStatus uint8   //  Off-board computer status
}

func (self *AslObctrl) MsgID() uint8 {
	return 207
}

func (self *AslObctrl) MsgName() string {
	return "AslObctrl"
}

func (self *AslObctrl) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Timestamp,
		&self.Uelev,
		&self.Uthrot,
		&self.Uthrot2,
		&self.Uaill,
		&self.Uailr,
		&self.Urud,
		&self.ObctrlStatus,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *AslObctrl) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Timestamp,
		&self.Uelev,
		&self.Uthrot,
		&self.Uthrot2,
		&self.Uaill,
		&self.Uailr,
		&self.Urud,
		&self.ObctrlStatus,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Atmospheric sensors (temperature, humidity, ...)
type SensAtmos struct {
	Tempambient float32 //  Ambient temperature [degrees Celsius]
	Humidity    float32 //  Relative humidity [%]
}

func (self *SensAtmos) MsgID() uint8 {
	return 208
}

func (self *SensAtmos) MsgName() string {
	return "SensAtmos"
}

func (self *SensAtmos) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Tempambient,
		&self.Humidity,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SensAtmos) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Tempambient,
		&self.Humidity,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Battery pack monitoring data for Li-Ion batteries
type SensBatmon struct {
	Temperature    float32 // Battery pack temperature in [deg C]
	Voltage        uint16  // Battery pack voltage in [mV]
	Current        int16   // Battery pack current in [mA]
	Batterystatus  uint16  // Battery monitor status report bits in Hex
	Serialnumber   uint16  // Battery monitor serial number in Hex
	Hostfetcontrol uint16  // Battery monitor sensor host FET control in Hex
	Cellvoltage1   uint16  // Battery pack cell 1 voltage in [mV]
	Cellvoltage2   uint16  // Battery pack cell 2 voltage in [mV]
	Cellvoltage3   uint16  // Battery pack cell 3 voltage in [mV]
	Cellvoltage4   uint16  // Battery pack cell 4 voltage in [mV]
	Cellvoltage5   uint16  // Battery pack cell 5 voltage in [mV]
	Cellvoltage6   uint16  // Battery pack cell 6 voltage in [mV]
	Soc            uint8   // Battery pack state-of-charge
}

func (self *SensBatmon) MsgID() uint8 {
	return 209
}

func (self *SensBatmon) MsgName() string {
	return "SensBatmon"
}

func (self *SensBatmon) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Temperature,
		&self.Voltage,
		&self.Current,
		&self.Batterystatus,
		&self.Serialnumber,
		&self.Hostfetcontrol,
		&self.Cellvoltage1,
		&self.Cellvoltage2,
		&self.Cellvoltage3,
		&self.Cellvoltage4,
		&self.Cellvoltage5,
		&self.Cellvoltage6,
		&self.Soc,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SensBatmon) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Temperature,
		&self.Voltage,
		&self.Current,
		&self.Batterystatus,
		&self.Serialnumber,
		&self.Hostfetcontrol,
		&self.Cellvoltage1,
		&self.Cellvoltage2,
		&self.Cellvoltage3,
		&self.Cellvoltage4,
		&self.Cellvoltage5,
		&self.Cellvoltage6,
		&self.Soc,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Message IDs
const (
	MSG_ID_SENS_POWER    = 201
	MSG_ID_SENS_MPPT     = 202
	MSG_ID_ASLCTRL_DATA  = 203
	MSG_ID_ASLCTRL_DEBUG = 204
	MSG_ID_ASLUAV_STATUS = 205
	MSG_ID_EKF_EXT       = 206
	MSG_ID_ASL_OBCTRL    = 207
	MSG_ID_SENS_ATMOS    = 208
	MSG_ID_SENS_BATMON   = 209
)

// DialectAsluav is the dialect represented by ASLUAV.xml
var DialectAsluav *Dialect = &Dialect{
	Name: "ASLUAV",
	crcExtras: map[uint8]uint8{
		201: 218, // MSG_ID_SENS_POWER
		202: 231, // MSG_ID_SENS_MPPT
		203: 0,   // MSG_ID_ASLCTRL_DATA
		204: 251, // MSG_ID_ASLCTRL_DEBUG
		205: 165, // MSG_ID_ASLUAV_STATUS
		206: 64,  // MSG_ID_EKF_EXT
		207: 234, // MSG_ID_ASL_OBCTRL
		208: 175, // MSG_ID_SENS_ATMOS
		209: 62,  // MSG_ID_SENS_BATMON
	},
}
