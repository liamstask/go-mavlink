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

// MavCmd:
const (
	MAV_CMD_RESET_MPPT      = 0 // Mission command to reset Maximum Power Point Tracker (MPPT)
	MAV_CMD_PAYLOAD_CONTROL = 1 // Mission command to perform a power cycle on payload
)

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
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Adc121VspbVolt))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Adc121CspbAmp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Adc121Cs1Amp))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Adc121Cs2Amp))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SensPower) Unpack(p *Packet) error {
	if len(p.Payload) < 16 {
		return fmt.Errorf("payload too small")
	}
	self.Adc121VspbVolt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Adc121CspbAmp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Adc121Cs1Amp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Adc121Cs2Amp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
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
	payload := make([]byte, 41)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.MpptTimestamp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Mppt1Volt))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Mppt1Amp))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Mppt2Volt))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Mppt2Amp))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Mppt3Volt))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Mppt3Amp))
	binary.LittleEndian.PutUint16(payload[32:], uint16(self.Mppt1Pwm))
	binary.LittleEndian.PutUint16(payload[34:], uint16(self.Mppt2Pwm))
	binary.LittleEndian.PutUint16(payload[36:], uint16(self.Mppt3Pwm))
	payload[38] = byte(self.Mppt1Status)
	payload[39] = byte(self.Mppt2Status)
	payload[40] = byte(self.Mppt3Status)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SensMppt) Unpack(p *Packet) error {
	if len(p.Payload) < 41 {
		return fmt.Errorf("payload too small")
	}
	self.MpptTimestamp = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Mppt1Volt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Mppt1Amp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Mppt2Volt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Mppt2Amp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Mppt3Volt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Mppt3Amp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Mppt1Pwm = uint16(binary.LittleEndian.Uint16(p.Payload[32:]))
	self.Mppt2Pwm = uint16(binary.LittleEndian.Uint16(p.Payload[34:]))
	self.Mppt3Pwm = uint16(binary.LittleEndian.Uint16(p.Payload[36:]))
	self.Mppt1Status = uint8(p.Payload[38])
	self.Mppt2Status = uint8(p.Payload[39])
	self.Mppt3Status = uint8(p.Payload[40])
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
	Nz              float32 //
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
	payload := make([]byte, 98)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.Timestamp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.H))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Href))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.HrefT))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Pitchangle))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Pitchangleref))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Q))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Qref))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Uelev))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.Uthrot))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.Uthrot2))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(self.Nz))
	binary.LittleEndian.PutUint32(payload[52:], math.Float32bits(self.Airspeedref))
	binary.LittleEndian.PutUint32(payload[56:], math.Float32bits(self.Yawangle))
	binary.LittleEndian.PutUint32(payload[60:], math.Float32bits(self.Yawangleref))
	binary.LittleEndian.PutUint32(payload[64:], math.Float32bits(self.Rollangle))
	binary.LittleEndian.PutUint32(payload[68:], math.Float32bits(self.Rollangleref))
	binary.LittleEndian.PutUint32(payload[72:], math.Float32bits(self.P))
	binary.LittleEndian.PutUint32(payload[76:], math.Float32bits(self.Pref))
	binary.LittleEndian.PutUint32(payload[80:], math.Float32bits(self.R))
	binary.LittleEndian.PutUint32(payload[84:], math.Float32bits(self.Rref))
	binary.LittleEndian.PutUint32(payload[88:], math.Float32bits(self.Uail))
	binary.LittleEndian.PutUint32(payload[92:], math.Float32bits(self.Urud))
	payload[96] = byte(self.AslctrlMode)
	payload[97] = byte(self.Spoilersengaged)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *AslctrlData) Unpack(p *Packet) error {
	if len(p.Payload) < 98 {
		return fmt.Errorf("payload too small")
	}
	self.Timestamp = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.H = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Href = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.HrefT = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Pitchangle = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Pitchangleref = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Q = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Qref = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Uelev = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Uthrot = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.Uthrot2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	self.Nz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48:]))
	self.Airspeedref = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[52:]))
	self.Yawangle = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[56:]))
	self.Yawangleref = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[60:]))
	self.Rollangle = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[64:]))
	self.Rollangleref = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[68:]))
	self.P = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[72:]))
	self.Pref = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[76:]))
	self.R = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[80:]))
	self.Rref = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[84:]))
	self.Uail = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[88:]))
	self.Urud = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[92:]))
	self.AslctrlMode = uint8(p.Payload[96])
	self.Spoilersengaged = uint8(p.Payload[97])
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
	payload := make([]byte, 38)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.I321))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.F1))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.F2))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.F3))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.F4))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.F5))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.F6))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.F7))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.F8))
	payload[36] = byte(self.I81)
	payload[37] = byte(self.I82)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *AslctrlDebug) Unpack(p *Packet) error {
	if len(p.Payload) < 38 {
		return fmt.Errorf("payload too small")
	}
	self.I321 = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.F1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.F2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.F3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.F4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.F5 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.F6 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.F7 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.F8 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.I81 = uint8(p.Payload[36])
	self.I82 = uint8(p.Payload[37])
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
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.MotorRpm))
	payload[4] = byte(self.LedStatus)
	payload[5] = byte(self.SatcomStatus)
	copy(payload[6:], self.ServoStatus[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *AsluavStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	self.MotorRpm = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.LedStatus = uint8(p.Payload[4])
	self.SatcomStatus = uint8(p.Payload[5])
	copy(self.ServoStatus[:], p.Payload[6:14])
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
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.Timestamp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Windspeed))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Winddir))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Windz))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Airspeed))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Beta))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Alpha))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *EkfExt) Unpack(p *Packet) error {
	if len(p.Payload) < 32 {
		return fmt.Errorf("payload too small")
	}
	self.Timestamp = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Windspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Winddir = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Windz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Airspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Beta = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Alpha = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
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
	payload := make([]byte, 33)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.Timestamp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Uelev))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Uthrot))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Uthrot2))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Uaill))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Uailr))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Urud))
	payload[32] = byte(self.ObctrlStatus)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *AslObctrl) Unpack(p *Packet) error {
	if len(p.Payload) < 33 {
		return fmt.Errorf("payload too small")
	}
	self.Timestamp = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Uelev = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Uthrot = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Uthrot2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Uaill = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Uailr = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Urud = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.ObctrlStatus = uint8(p.Payload[32])
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
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Tempambient))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Humidity))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SensAtmos) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	self.Tempambient = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Humidity = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
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
	payload := make([]byte, 27)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Temperature))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Voltage))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.Current))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Batterystatus))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.Serialnumber))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Hostfetcontrol))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.Cellvoltage1))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Cellvoltage2))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.Cellvoltage3))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.Cellvoltage4))
	binary.LittleEndian.PutUint16(payload[22:], uint16(self.Cellvoltage5))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.Cellvoltage6))
	payload[26] = byte(self.Soc)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SensBatmon) Unpack(p *Packet) error {
	if len(p.Payload) < 27 {
		return fmt.Errorf("payload too small")
	}
	self.Temperature = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Voltage = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.Current = int16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.Batterystatus = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Serialnumber = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.Hostfetcontrol = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Cellvoltage1 = uint16(binary.LittleEndian.Uint16(p.Payload[14:]))
	self.Cellvoltage2 = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.Cellvoltage3 = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.Cellvoltage4 = uint16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.Cellvoltage5 = uint16(binary.LittleEndian.Uint16(p.Payload[22:]))
	self.Cellvoltage6 = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.Soc = uint8(p.Payload[26])
	return nil
}

// Fixed-wing soaring (i.e. thermal seeking) data
type FwSoaringData struct {
	Timestamp            uint64  // Timestamp [ms]
	Timestampmodechanged uint64  // Timestamp since last mode change[ms]
	Xw                   float32 // Thermal core updraft strength [m/s]
	Xr                   float32 // Thermal radius [m]
	Xlat                 float32 // Thermal center latitude [deg]
	Xlon                 float32 // Thermal center longitude [deg]
	Varw                 float32 // Variance W
	Varr                 float32 // Variance R
	Varlat               float32 // Variance Lat
	Varlon               float32 // Variance Lon
	Loiterradius         float32 // Suggested loiter radius [m]
	Loiterdirection      float32 // Suggested loiter direction
	Disttosoarpoint      float32 // Distance to soar point [m]
	Vsinkexp             float32 // Expected sink rate at current airspeed, roll and throttle [m/s]
	Z1Localupdraftspeed  float32 // Measurement / updraft speed at current/local airplane position [m/s]
	Z2Deltaroll          float32 // Measurement / roll angle tracking error [deg]
	Z1Exp                float32 // Expected measurement 1
	Z2Exp                float32 // Expected measurement 2
	Thermalgsnorth       float32 // Thermal drift (from estimator prediction step only) [m/s]
	Thermalgseast        float32 // Thermal drift (from estimator prediction step only) [m/s]
	TseDot               float32 //  Total specific energy change (filtered) [m/s]
	Debugvar1            float32 //  Debug variable 1
	Debugvar2            float32 //  Debug variable 2
	Controlmode          uint8   // Control Mode [-]
	Valid                uint8   // Data valid [-]
}

func (self *FwSoaringData) MsgID() uint8 {
	return 210
}

func (self *FwSoaringData) MsgName() string {
	return "FwSoaringData"
}

func (self *FwSoaringData) Pack(p *Packet) error {
	payload := make([]byte, 102)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.Timestamp))
	binary.LittleEndian.PutUint64(payload[8:], uint64(self.Timestampmodechanged))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Xw))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Xr))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Xlat))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Xlon))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Varw))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Varr))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.Varlat))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.Varlon))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(self.Loiterradius))
	binary.LittleEndian.PutUint32(payload[52:], math.Float32bits(self.Loiterdirection))
	binary.LittleEndian.PutUint32(payload[56:], math.Float32bits(self.Disttosoarpoint))
	binary.LittleEndian.PutUint32(payload[60:], math.Float32bits(self.Vsinkexp))
	binary.LittleEndian.PutUint32(payload[64:], math.Float32bits(self.Z1Localupdraftspeed))
	binary.LittleEndian.PutUint32(payload[68:], math.Float32bits(self.Z2Deltaroll))
	binary.LittleEndian.PutUint32(payload[72:], math.Float32bits(self.Z1Exp))
	binary.LittleEndian.PutUint32(payload[76:], math.Float32bits(self.Z2Exp))
	binary.LittleEndian.PutUint32(payload[80:], math.Float32bits(self.Thermalgsnorth))
	binary.LittleEndian.PutUint32(payload[84:], math.Float32bits(self.Thermalgseast))
	binary.LittleEndian.PutUint32(payload[88:], math.Float32bits(self.TseDot))
	binary.LittleEndian.PutUint32(payload[92:], math.Float32bits(self.Debugvar1))
	binary.LittleEndian.PutUint32(payload[96:], math.Float32bits(self.Debugvar2))
	payload[100] = byte(self.Controlmode)
	payload[101] = byte(self.Valid)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FwSoaringData) Unpack(p *Packet) error {
	if len(p.Payload) < 102 {
		return fmt.Errorf("payload too small")
	}
	self.Timestamp = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Timestampmodechanged = uint64(binary.LittleEndian.Uint64(p.Payload[8:]))
	self.Xw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Xr = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Xlat = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Xlon = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Varw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Varr = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Varlat = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.Varlon = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	self.Loiterradius = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48:]))
	self.Loiterdirection = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[52:]))
	self.Disttosoarpoint = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[56:]))
	self.Vsinkexp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[60:]))
	self.Z1Localupdraftspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[64:]))
	self.Z2Deltaroll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[68:]))
	self.Z1Exp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[72:]))
	self.Z2Exp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[76:]))
	self.Thermalgsnorth = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[80:]))
	self.Thermalgseast = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[84:]))
	self.TseDot = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[88:]))
	self.Debugvar1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[92:]))
	self.Debugvar2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[96:]))
	self.Controlmode = uint8(p.Payload[100])
	self.Valid = uint8(p.Payload[101])
	return nil
}

// Monitoring of sensorpod status
type SensorpodStatus struct {
	Timestamp           uint64 // Timestamp in linuxtime [ms] (since 1.1.1970)
	FreeSpace           uint16 // Free space available in recordings directory in [Gb] * 1e2
	VisensorRate1       uint8  // Rate of ROS topic 1
	VisensorRate2       uint8  // Rate of ROS topic 2
	VisensorRate3       uint8  // Rate of ROS topic 3
	VisensorRate4       uint8  // Rate of ROS topic 4
	RecordingNodesCount uint8  // Number of recording nodes
	CpuTemp             uint8  // Temperature of sensorpod CPU in [deg C]
}

func (self *SensorpodStatus) MsgID() uint8 {
	return 211
}

func (self *SensorpodStatus) MsgName() string {
	return "SensorpodStatus"
}

func (self *SensorpodStatus) Pack(p *Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.Timestamp))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.FreeSpace))
	payload[10] = byte(self.VisensorRate1)
	payload[11] = byte(self.VisensorRate2)
	payload[12] = byte(self.VisensorRate3)
	payload[13] = byte(self.VisensorRate4)
	payload[14] = byte(self.RecordingNodesCount)
	payload[15] = byte(self.CpuTemp)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SensorpodStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 16 {
		return fmt.Errorf("payload too small")
	}
	self.Timestamp = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.FreeSpace = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.VisensorRate1 = uint8(p.Payload[10])
	self.VisensorRate2 = uint8(p.Payload[11])
	self.VisensorRate3 = uint8(p.Payload[12])
	self.VisensorRate4 = uint8(p.Payload[13])
	self.RecordingNodesCount = uint8(p.Payload[14])
	self.CpuTemp = uint8(p.Payload[15])
	return nil
}

// Monitoring of power board status
type SensPowerBoard struct {
	Timestamp        uint64  // Timestamp
	PwrBrdSystemVolt float32 // Power board system voltage
	PwrBrdServoVolt  float32 // Power board servo voltage
	PwrBrdMotLAmp    float32 // Power board left motor current sensor
	PwrBrdMotRAmp    float32 // Power board right motor current sensor
	PwrBrdServo1Amp  float32 // Power board servo1 current sensor
	PwrBrdServo2Amp  float32 // Power board servo1 current sensor
	PwrBrdServo3Amp  float32 // Power board servo1 current sensor
	PwrBrdServo4Amp  float32 // Power board servo1 current sensor
	PwrBrdAuxAmp     float32 // Power board aux current sensor
	PwrBrdStatus     uint8   // Power board status register
	PwrBrdLedStatus  uint8   // Power board leds status
}

func (self *SensPowerBoard) MsgID() uint8 {
	return 212
}

func (self *SensPowerBoard) MsgName() string {
	return "SensPowerBoard"
}

func (self *SensPowerBoard) Pack(p *Packet) error {
	payload := make([]byte, 46)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.Timestamp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.PwrBrdSystemVolt))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.PwrBrdServoVolt))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.PwrBrdMotLAmp))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.PwrBrdMotRAmp))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.PwrBrdServo1Amp))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.PwrBrdServo2Amp))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.PwrBrdServo3Amp))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.PwrBrdServo4Amp))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.PwrBrdAuxAmp))
	payload[44] = byte(self.PwrBrdStatus)
	payload[45] = byte(self.PwrBrdLedStatus)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SensPowerBoard) Unpack(p *Packet) error {
	if len(p.Payload) < 46 {
		return fmt.Errorf("payload too small")
	}
	self.Timestamp = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.PwrBrdSystemVolt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.PwrBrdServoVolt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.PwrBrdMotLAmp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.PwrBrdMotRAmp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.PwrBrdServo1Amp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.PwrBrdServo2Amp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.PwrBrdServo3Amp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.PwrBrdServo4Amp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.PwrBrdAuxAmp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.PwrBrdStatus = uint8(p.Payload[44])
	self.PwrBrdLedStatus = uint8(p.Payload[45])
	return nil
}

// Message IDs
const (
	MSG_ID_SENS_POWER       = 201
	MSG_ID_SENS_MPPT        = 202
	MSG_ID_ASLCTRL_DATA     = 203
	MSG_ID_ASLCTRL_DEBUG    = 204
	MSG_ID_ASLUAV_STATUS    = 205
	MSG_ID_EKF_EXT          = 206
	MSG_ID_ASL_OBCTRL       = 207
	MSG_ID_SENS_ATMOS       = 208
	MSG_ID_SENS_BATMON      = 209
	MSG_ID_FW_SOARING_DATA  = 210
	MSG_ID_SENSORPOD_STATUS = 211
	MSG_ID_SENS_POWER_BOARD = 212
)

// DialectAsluav is the dialect represented by ASLUAV.xml
var DialectAsluav *Dialect = &Dialect{
	Name: "ASLUAV",
	crcExtras: map[uint8]uint8{
		201: 218, // MSG_ID_SENS_POWER
		202: 231, // MSG_ID_SENS_MPPT
		203: 172, // MSG_ID_ASLCTRL_DATA
		204: 251, // MSG_ID_ASLCTRL_DEBUG
		205: 97,  // MSG_ID_ASLUAV_STATUS
		206: 64,  // MSG_ID_EKF_EXT
		207: 234, // MSG_ID_ASL_OBCTRL
		208: 175, // MSG_ID_SENS_ATMOS
		209: 62,  // MSG_ID_SENS_BATMON
		210: 20,  // MSG_ID_FW_SOARING_DATA
		211: 54,  // MSG_ID_SENSORPOD_STATUS
		212: 242, // MSG_ID_SENS_POWER_BOARD
	},
}
