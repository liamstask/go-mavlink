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

// DataTypes: Content Types for data transmission handshake
const (
	DATA_TYPE_JPEG_IMAGE = 1 //
	DATA_TYPE_RAW_IMAGE  = 2 //
	DATA_TYPE_KINECT     = 3 //
)

// MavCmd:
const (
	MAV_CMD_DO_START_SEARCH  = 0 // Starts a search
	MAV_CMD_DO_FINISH_SEARCH = 1 // Starts a search
	MAV_CMD_NAV_SWEEP        = 2 // Starts a search
)

//
type SetCamShutter struct {
	Gain       float32 // Camera gain
	Interval   uint16  // Shutter interval, in microseconds
	Exposure   uint16  // Exposure time, in microseconds
	CamNo      uint8   // Camera id
	CamMode    uint8   // Camera mode: 0 = auto, 1 = manual
	TriggerPin uint8   // Trigger pin, 0-3 for PtGrey FireFly
}

func (self *SetCamShutter) MsgID() uint8 {
	return 151
}

func (self *SetCamShutter) MsgName() string {
	return "SetCamShutter"
}

func (self *SetCamShutter) Pack(p *Packet) error {
	payload := make([]byte, 11)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Gain))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Interval))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.Exposure))
	payload[8] = byte(self.CamNo)
	payload[9] = byte(self.CamMode)
	payload[10] = byte(self.TriggerPin)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SetCamShutter) Unpack(p *Packet) error {
	if len(p.Payload) < 11 {
		return fmt.Errorf("payload too small")
	}
	self.Gain = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Interval = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.Exposure = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.CamNo = uint8(p.Payload[8])
	self.CamMode = uint8(p.Payload[9])
	self.TriggerPin = uint8(p.Payload[10])
	return nil
}

//
type ImageTriggered struct {
	Timestamp uint64  // Timestamp
	Seq       uint32  // IMU seq
	Roll      float32 // Roll angle in rad
	Pitch     float32 // Pitch angle in rad
	Yaw       float32 // Yaw angle in rad
	LocalZ    float32 // Local frame Z coordinate (height over ground)
	Lat       float32 // GPS X coordinate
	Lon       float32 // GPS Y coordinate
	Alt       float32 // Global frame altitude
	GroundX   float32 // Ground truth X
	GroundY   float32 // Ground truth Y
	GroundZ   float32 // Ground truth Z
}

func (self *ImageTriggered) MsgID() uint8 {
	return 152
}

func (self *ImageTriggered) MsgName() string {
	return "ImageTriggered"
}

func (self *ImageTriggered) Pack(p *Packet) error {
	payload := make([]byte, 52)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.Timestamp))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Seq))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.LocalZ))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Lat))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Lon))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Alt))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.GroundX))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.GroundY))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(self.GroundZ))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ImageTriggered) Unpack(p *Packet) error {
	if len(p.Payload) < 52 {
		return fmt.Errorf("payload too small")
	}
	self.Timestamp = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Seq = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.LocalZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Lat = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Lon = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Alt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.GroundX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.GroundY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	self.GroundZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48:]))
	return nil
}

//
type ImageTriggerControl struct {
	Enable uint8 // 0 to disable, 1 to enable
}

func (self *ImageTriggerControl) MsgID() uint8 {
	return 153
}

func (self *ImageTriggerControl) MsgName() string {
	return "ImageTriggerControl"
}

func (self *ImageTriggerControl) Pack(p *Packet) error {
	payload := make([]byte, 1)
	payload[0] = byte(self.Enable)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ImageTriggerControl) Unpack(p *Packet) error {
	if len(p.Payload) < 1 {
		return fmt.Errorf("payload too small")
	}
	self.Enable = uint8(p.Payload[0])
	return nil
}

//
type ImageAvailable struct {
	CamId       uint64  // Camera id
	Timestamp   uint64  // Timestamp
	ValidUntil  uint64  // Until which timestamp this buffer will stay valid
	ImgSeq      uint32  // The image sequence number
	ImgBufIndex uint32  // Position of the image in the buffer, starts with 0
	Key         uint32  // Shared memory area key
	Exposure    uint32  // Exposure time, in microseconds
	Gain        float32 // Camera gain
	Roll        float32 // Roll angle in rad
	Pitch       float32 // Pitch angle in rad
	Yaw         float32 // Yaw angle in rad
	LocalZ      float32 // Local frame Z coordinate (height over ground)
	Lat         float32 // GPS X coordinate
	Lon         float32 // GPS Y coordinate
	Alt         float32 // Global frame altitude
	GroundX     float32 // Ground truth X
	GroundY     float32 // Ground truth Y
	GroundZ     float32 // Ground truth Z
	Width       uint16  // Image width
	Height      uint16  // Image height
	Depth       uint16  // Image depth
	CamNo       uint8   // Camera # (starts with 0)
	Channels    uint8   // Image channels
}

func (self *ImageAvailable) MsgID() uint8 {
	return 154
}

func (self *ImageAvailable) MsgName() string {
	return "ImageAvailable"
}

func (self *ImageAvailable) Pack(p *Packet) error {
	payload := make([]byte, 92)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.CamId))
	binary.LittleEndian.PutUint64(payload[8:], uint64(self.Timestamp))
	binary.LittleEndian.PutUint64(payload[16:], uint64(self.ValidUntil))
	binary.LittleEndian.PutUint32(payload[24:], uint32(self.ImgSeq))
	binary.LittleEndian.PutUint32(payload[28:], uint32(self.ImgBufIndex))
	binary.LittleEndian.PutUint32(payload[32:], uint32(self.Key))
	binary.LittleEndian.PutUint32(payload[36:], uint32(self.Exposure))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.Gain))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[52:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[56:], math.Float32bits(self.LocalZ))
	binary.LittleEndian.PutUint32(payload[60:], math.Float32bits(self.Lat))
	binary.LittleEndian.PutUint32(payload[64:], math.Float32bits(self.Lon))
	binary.LittleEndian.PutUint32(payload[68:], math.Float32bits(self.Alt))
	binary.LittleEndian.PutUint32(payload[72:], math.Float32bits(self.GroundX))
	binary.LittleEndian.PutUint32(payload[76:], math.Float32bits(self.GroundY))
	binary.LittleEndian.PutUint32(payload[80:], math.Float32bits(self.GroundZ))
	binary.LittleEndian.PutUint16(payload[84:], uint16(self.Width))
	binary.LittleEndian.PutUint16(payload[86:], uint16(self.Height))
	binary.LittleEndian.PutUint16(payload[88:], uint16(self.Depth))
	payload[90] = byte(self.CamNo)
	payload[91] = byte(self.Channels)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ImageAvailable) Unpack(p *Packet) error {
	if len(p.Payload) < 92 {
		return fmt.Errorf("payload too small")
	}
	self.CamId = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Timestamp = uint64(binary.LittleEndian.Uint64(p.Payload[8:]))
	self.ValidUntil = uint64(binary.LittleEndian.Uint64(p.Payload[16:]))
	self.ImgSeq = uint32(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.ImgBufIndex = uint32(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Key = uint32(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Exposure = uint32(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Gain = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[52:]))
	self.LocalZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[56:]))
	self.Lat = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[60:]))
	self.Lon = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[64:]))
	self.Alt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[68:]))
	self.GroundX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[72:]))
	self.GroundY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[76:]))
	self.GroundZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[80:]))
	self.Width = uint16(binary.LittleEndian.Uint16(p.Payload[84:]))
	self.Height = uint16(binary.LittleEndian.Uint16(p.Payload[86:]))
	self.Depth = uint16(binary.LittleEndian.Uint16(p.Payload[88:]))
	self.CamNo = uint8(p.Payload[90])
	self.Channels = uint8(p.Payload[91])
	return nil
}

// Message sent to the MAV to set a new offset from the currently controlled position
type SetPositionControlOffset struct {
	X               float32 // x position offset
	Y               float32 // y position offset
	Z               float32 // z position offset
	Yaw             float32 // yaw orientation offset in radians, 0 = NORTH
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
}

func (self *SetPositionControlOffset) MsgID() uint8 {
	return 160
}

func (self *SetPositionControlOffset) MsgName() string {
	return "SetPositionControlOffset"
}

func (self *SetPositionControlOffset) Pack(p *Packet) error {
	payload := make([]byte, 18)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Z))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Yaw))
	payload[16] = byte(self.TargetSystem)
	payload[17] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SetPositionControlOffset) Unpack(p *Packet) error {
	if len(p.Payload) < 18 {
		return fmt.Errorf("payload too small")
	}
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.TargetSystem = uint8(p.Payload[16])
	self.TargetComponent = uint8(p.Payload[17])
	return nil
}

//
type PositionControlSetpoint struct {
	X   float32 // x position
	Y   float32 // y position
	Z   float32 // z position
	Yaw float32 // yaw orientation in radians, 0 = NORTH
	Id  uint16  // ID of waypoint, 0 for plain position
}

func (self *PositionControlSetpoint) MsgID() uint8 {
	return 170
}

func (self *PositionControlSetpoint) MsgName() string {
	return "PositionControlSetpoint"
}

func (self *PositionControlSetpoint) Pack(p *Packet) error {
	payload := make([]byte, 18)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Z))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Id))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *PositionControlSetpoint) Unpack(p *Packet) error {
	if len(p.Payload) < 18 {
		return fmt.Errorf("payload too small")
	}
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Id = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	return nil
}

//
type Marker struct {
	X     float32 // x position
	Y     float32 // y position
	Z     float32 // z position
	Roll  float32 // roll orientation
	Pitch float32 // pitch orientation
	Yaw   float32 // yaw orientation
	Id    uint16  // ID
}

func (self *Marker) MsgID() uint8 {
	return 171
}

func (self *Marker) MsgName() string {
	return "Marker"
}

func (self *Marker) Pack(p *Packet) error {
	payload := make([]byte, 26)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Z))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.Id))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Marker) Unpack(p *Packet) error {
	if len(p.Payload) < 26 {
		return fmt.Errorf("payload too small")
	}
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Id = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	return nil
}

//
type RawAux struct {
	Baro int32  // Barometric pressure (hecto Pascal)
	Adc1 uint16 // ADC1 (J405 ADC3, LPC2148 AD0.6)
	Adc2 uint16 // ADC2 (J405 ADC5, LPC2148 AD0.2)
	Adc3 uint16 // ADC3 (J405 ADC6, LPC2148 AD0.1)
	Adc4 uint16 // ADC4 (J405 ADC7, LPC2148 AD1.3)
	Vbat uint16 // Battery voltage
	Temp int16  // Temperature (degrees celcius)
}

func (self *RawAux) MsgID() uint8 {
	return 172
}

func (self *RawAux) MsgName() string {
	return "RawAux"
}

func (self *RawAux) Pack(p *Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Baro))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Adc1))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.Adc2))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Adc3))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.Adc4))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Vbat))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.Temp))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RawAux) Unpack(p *Packet) error {
	if len(p.Payload) < 16 {
		return fmt.Errorf("payload too small")
	}
	self.Baro = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Adc1 = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.Adc2 = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.Adc3 = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Adc4 = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.Vbat = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Temp = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	return nil
}

//
type WatchdogHeartbeat struct {
	WatchdogId   uint16 // Watchdog ID
	ProcessCount uint16 // Number of processes
}

func (self *WatchdogHeartbeat) MsgID() uint8 {
	return 180
}

func (self *WatchdogHeartbeat) MsgName() string {
	return "WatchdogHeartbeat"
}

func (self *WatchdogHeartbeat) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.WatchdogId))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.ProcessCount))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *WatchdogHeartbeat) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	self.WatchdogId = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.ProcessCount = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	return nil
}

//
type WatchdogProcessInfo struct {
	Timeout    int32     // Timeout (seconds)
	WatchdogId uint16    // Watchdog ID
	ProcessId  uint16    // Process ID
	Name       [100]byte // Process name
	Arguments  [147]byte // Process arguments
}

func (self *WatchdogProcessInfo) MsgID() uint8 {
	return 181
}

func (self *WatchdogProcessInfo) MsgName() string {
	return "WatchdogProcessInfo"
}

func (self *WatchdogProcessInfo) Pack(p *Packet) error {
	payload := make([]byte, 255)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Timeout))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.WatchdogId))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.ProcessId))
	copy(payload[8:], self.Name[:])
	copy(payload[108:], self.Arguments[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *WatchdogProcessInfo) Unpack(p *Packet) error {
	if len(p.Payload) < 255 {
		return fmt.Errorf("payload too small")
	}
	self.Timeout = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.WatchdogId = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.ProcessId = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	copy(self.Name[:], p.Payload[8:108])
	copy(self.Arguments[:], p.Payload[108:255])
	return nil
}

//
type WatchdogProcessStatus struct {
	Pid        int32  // PID
	WatchdogId uint16 // Watchdog ID
	ProcessId  uint16 // Process ID
	Crashes    uint16 // Number of crashes
	State      uint8  // Is running / finished / suspended / crashed
	Muted      uint8  // Is muted
}

func (self *WatchdogProcessStatus) MsgID() uint8 {
	return 182
}

func (self *WatchdogProcessStatus) MsgName() string {
	return "WatchdogProcessStatus"
}

func (self *WatchdogProcessStatus) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Pid))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.WatchdogId))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.ProcessId))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Crashes))
	payload[10] = byte(self.State)
	payload[11] = byte(self.Muted)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *WatchdogProcessStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	self.Pid = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.WatchdogId = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.ProcessId = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.Crashes = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.State = uint8(p.Payload[10])
	self.Muted = uint8(p.Payload[11])
	return nil
}

//
type WatchdogCommand struct {
	WatchdogId     uint16 // Watchdog ID
	ProcessId      uint16 // Process ID
	TargetSystemId uint8  // Target system ID
	CommandId      uint8  // Command ID
}

func (self *WatchdogCommand) MsgID() uint8 {
	return 183
}

func (self *WatchdogCommand) MsgName() string {
	return "WatchdogCommand"
}

func (self *WatchdogCommand) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.WatchdogId))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.ProcessId))
	payload[4] = byte(self.TargetSystemId)
	payload[5] = byte(self.CommandId)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *WatchdogCommand) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	self.WatchdogId = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.ProcessId = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.TargetSystemId = uint8(p.Payload[4])
	self.CommandId = uint8(p.Payload[5])
	return nil
}

//
type PatternDetected struct {
	Confidence float32   // Confidence of detection
	Type       uint8     // 0: Pattern, 1: Letter
	File       [100]byte // Pattern file name
	Detected   uint8     // Accepted as true detection, 0 no, 1 yes
}

func (self *PatternDetected) MsgID() uint8 {
	return 190
}

func (self *PatternDetected) MsgName() string {
	return "PatternDetected"
}

func (self *PatternDetected) Pack(p *Packet) error {
	payload := make([]byte, 106)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Confidence))
	payload[4] = byte(self.Type)
	copy(payload[5:], self.File[:])
	payload[105] = byte(self.Detected)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *PatternDetected) Unpack(p *Packet) error {
	if len(p.Payload) < 106 {
		return fmt.Errorf("payload too small")
	}
	self.Confidence = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Type = uint8(p.Payload[4])
	copy(self.File[:], p.Payload[5:105])
	self.Detected = uint8(p.Payload[105])
	return nil
}

// Notifies the operator about a point of interest (POI). This can be anything detected by the
//                 system. This generic message is intented to help interfacing to generic visualizations and to display
//                 the POI on a map.
//
type PointOfInterest struct {
	X                float32  // X Position
	Y                float32  // Y Position
	Z                float32  // Z Position
	Timeout          uint16   // 0: no timeout, >1: timeout in seconds
	Type             uint8    // 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
	Color            uint8    // 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
	CoordinateSystem uint8    // 0: global, 1:local
	Name             [26]byte // POI name
}

func (self *PointOfInterest) MsgID() uint8 {
	return 191
}

func (self *PointOfInterest) MsgName() string {
	return "PointOfInterest"
}

func (self *PointOfInterest) Pack(p *Packet) error {
	payload := make([]byte, 43)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Z))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Timeout))
	payload[14] = byte(self.Type)
	payload[15] = byte(self.Color)
	payload[16] = byte(self.CoordinateSystem)
	copy(payload[17:], self.Name[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *PointOfInterest) Unpack(p *Packet) error {
	if len(p.Payload) < 43 {
		return fmt.Errorf("payload too small")
	}
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Timeout = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Type = uint8(p.Payload[14])
	self.Color = uint8(p.Payload[15])
	self.CoordinateSystem = uint8(p.Payload[16])
	copy(self.Name[:], p.Payload[17:43])
	return nil
}

// Notifies the operator about the connection of two point of interests (POI). This can be anything detected by the
//                 system. This generic message is intented to help interfacing to generic visualizations and to display
//                 the POI on a map.
//
type PointOfInterestConnection struct {
	Xp1              float32  // X1 Position
	Yp1              float32  // Y1 Position
	Zp1              float32  // Z1 Position
	Xp2              float32  // X2 Position
	Yp2              float32  // Y2 Position
	Zp2              float32  // Z2 Position
	Timeout          uint16   // 0: no timeout, >1: timeout in seconds
	Type             uint8    // 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
	Color            uint8    // 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
	CoordinateSystem uint8    // 0: global, 1:local
	Name             [26]byte // POI connection name
}

func (self *PointOfInterestConnection) MsgID() uint8 {
	return 192
}

func (self *PointOfInterestConnection) MsgName() string {
	return "PointOfInterestConnection"
}

func (self *PointOfInterestConnection) Pack(p *Packet) error {
	payload := make([]byte, 55)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Xp1))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Yp1))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Zp1))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Xp2))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Yp2))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Zp2))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.Timeout))
	payload[26] = byte(self.Type)
	payload[27] = byte(self.Color)
	payload[28] = byte(self.CoordinateSystem)
	copy(payload[29:], self.Name[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *PointOfInterestConnection) Unpack(p *Packet) error {
	if len(p.Payload) < 55 {
		return fmt.Errorf("payload too small")
	}
	self.Xp1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Yp1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Zp1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Xp2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Yp2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Zp2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Timeout = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.Type = uint8(p.Payload[26])
	self.Color = uint8(p.Payload[27])
	self.CoordinateSystem = uint8(p.Payload[28])
	copy(self.Name[:], p.Payload[29:55])
	return nil
}

//
type BriefFeature struct {
	X                     float32   // x position in m
	Y                     float32   // y position in m
	Z                     float32   // z position in m
	Response              float32   // Harris operator response at this location
	Size                  uint16    // Size in pixels
	Orientation           uint16    // Orientation
	OrientationAssignment uint8     // Orientation assignment 0: false, 1:true
	Descriptor            [32]uint8 // Descriptor
}

func (self *BriefFeature) MsgID() uint8 {
	return 195
}

func (self *BriefFeature) MsgName() string {
	return "BriefFeature"
}

func (self *BriefFeature) Pack(p *Packet) error {
	payload := make([]byte, 53)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Z))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Response))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Size))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.Orientation))
	payload[20] = byte(self.OrientationAssignment)
	copy(payload[21:], self.Descriptor[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *BriefFeature) Unpack(p *Packet) error {
	if len(p.Payload) < 53 {
		return fmt.Errorf("payload too small")
	}
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Response = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Size = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.Orientation = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.OrientationAssignment = uint8(p.Payload[20])
	copy(self.Descriptor[:], p.Payload[21:53])
	return nil
}

//
type AttitudeControl struct {
	Roll         float32 // roll
	Pitch        float32 // pitch
	Yaw          float32 // yaw
	Thrust       float32 // thrust
	Target       uint8   // The system to be controlled
	RollManual   uint8   // roll control enabled auto:0, manual:1
	PitchManual  uint8   // pitch auto:0, manual:1
	YawManual    uint8   // yaw auto:0, manual:1
	ThrustManual uint8   // thrust auto:0, manual:1
}

func (self *AttitudeControl) MsgID() uint8 {
	return 200
}

func (self *AttitudeControl) MsgName() string {
	return "AttitudeControl"
}

func (self *AttitudeControl) Pack(p *Packet) error {
	payload := make([]byte, 21)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Thrust))
	payload[16] = byte(self.Target)
	payload[17] = byte(self.RollManual)
	payload[18] = byte(self.PitchManual)
	payload[19] = byte(self.YawManual)
	payload[20] = byte(self.ThrustManual)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *AttitudeControl) Unpack(p *Packet) error {
	if len(p.Payload) < 21 {
		return fmt.Errorf("payload too small")
	}
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Thrust = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Target = uint8(p.Payload[16])
	self.RollManual = uint8(p.Payload[17])
	self.PitchManual = uint8(p.Payload[18])
	self.YawManual = uint8(p.Payload[19])
	self.ThrustManual = uint8(p.Payload[20])
	return nil
}

//
type DetectionStats struct {
	Detections        uint32  // Number of detections
	ClusterIters      uint32  // Number of cluster iterations
	BestScore         float32 // Best score
	BestLat           int32   // Latitude of the best detection * 1E7
	BestLon           int32   // Longitude of the best detection * 1E7
	BestAlt           int32   // Altitude of the best detection * 1E3
	BestDetectionId   uint32  // Best detection ID
	BestClusterId     uint32  // Best cluster ID
	BestClusterIterId uint32  // Best cluster ID
	ImagesDone        uint32  // Number of images already processed
	ImagesTodo        uint32  // Number of images still to process
	Fps               float32 // Average images per seconds processed
}

func (self *DetectionStats) MsgID() uint8 {
	return 205
}

func (self *DetectionStats) MsgName() string {
	return "DetectionStats"
}

func (self *DetectionStats) Pack(p *Packet) error {
	payload := make([]byte, 48)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Detections))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.ClusterIters))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.BestScore))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.BestLat))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.BestLon))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.BestAlt))
	binary.LittleEndian.PutUint32(payload[24:], uint32(self.BestDetectionId))
	binary.LittleEndian.PutUint32(payload[28:], uint32(self.BestClusterId))
	binary.LittleEndian.PutUint32(payload[32:], uint32(self.BestClusterIterId))
	binary.LittleEndian.PutUint32(payload[36:], uint32(self.ImagesDone))
	binary.LittleEndian.PutUint32(payload[40:], uint32(self.ImagesTodo))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.Fps))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *DetectionStats) Unpack(p *Packet) error {
	if len(p.Payload) < 48 {
		return fmt.Errorf("payload too small")
	}
	self.Detections = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.ClusterIters = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.BestScore = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.BestLat = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.BestLon = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.BestAlt = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.BestDetectionId = uint32(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.BestClusterId = uint32(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.BestClusterIterId = uint32(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.ImagesDone = uint32(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.ImagesTodo = uint32(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.Fps = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	return nil
}

//
type OnboardHealth struct {
	Uptime         uint32  // Uptime of system
	RamTotal       float32 // RAM size in GiB
	SwapTotal      float32 // Swap size in GiB
	DiskTotal      float32 // Disk total in GiB
	Temp           float32 // Temperature
	Voltage        float32 // Supply voltage V
	NetworkLoadIn  float32 // Network load inbound KiB/s
	NetworkLoadOut float32 // Network load outbound in KiB/s
	CpuFreq        uint16  // CPU frequency
	CpuLoad        uint8   // CPU load in percent
	RamUsage       uint8   // RAM usage in percent
	SwapUsage      uint8   // Swap usage in percent
	DiskHealth     int8    // Disk health (-1: N/A, 0: ERR, 1: RO, 2: RW)
	DiskUsage      uint8   // Disk usage in percent
}

func (self *OnboardHealth) MsgID() uint8 {
	return 206
}

func (self *OnboardHealth) MsgName() string {
	return "OnboardHealth"
}

func (self *OnboardHealth) Pack(p *Packet) error {
	payload := make([]byte, 39)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Uptime))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.RamTotal))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.SwapTotal))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.DiskTotal))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Temp))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Voltage))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.NetworkLoadIn))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.NetworkLoadOut))
	binary.LittleEndian.PutUint16(payload[32:], uint16(self.CpuFreq))
	payload[34] = byte(self.CpuLoad)
	payload[35] = byte(self.RamUsage)
	payload[36] = byte(self.SwapUsage)
	payload[37] = byte(self.DiskHealth)
	payload[38] = byte(self.DiskUsage)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *OnboardHealth) Unpack(p *Packet) error {
	if len(p.Payload) < 39 {
		return fmt.Errorf("payload too small")
	}
	self.Uptime = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.RamTotal = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.SwapTotal = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.DiskTotal = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Temp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Voltage = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.NetworkLoadIn = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.NetworkLoadOut = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.CpuFreq = uint16(binary.LittleEndian.Uint16(p.Payload[32:]))
	self.CpuLoad = uint8(p.Payload[34])
	self.RamUsage = uint8(p.Payload[35])
	self.SwapUsage = uint8(p.Payload[36])
	self.DiskHealth = int8(p.Payload[37])
	self.DiskUsage = uint8(p.Payload[38])
	return nil
}

// Message IDs
const (
	MSG_ID_SET_CAM_SHUTTER              = 151
	MSG_ID_IMAGE_TRIGGERED              = 152
	MSG_ID_IMAGE_TRIGGER_CONTROL        = 153
	MSG_ID_IMAGE_AVAILABLE              = 154
	MSG_ID_SET_POSITION_CONTROL_OFFSET  = 160
	MSG_ID_POSITION_CONTROL_SETPOINT    = 170
	MSG_ID_MARKER                       = 171
	MSG_ID_RAW_AUX                      = 172
	MSG_ID_WATCHDOG_HEARTBEAT           = 180
	MSG_ID_WATCHDOG_PROCESS_INFO        = 181
	MSG_ID_WATCHDOG_PROCESS_STATUS      = 182
	MSG_ID_WATCHDOG_COMMAND             = 183
	MSG_ID_PATTERN_DETECTED             = 190
	MSG_ID_POINT_OF_INTEREST            = 191
	MSG_ID_POINT_OF_INTEREST_CONNECTION = 192
	MSG_ID_BRIEF_FEATURE                = 195
	MSG_ID_ATTITUDE_CONTROL             = 200
	MSG_ID_DETECTION_STATS              = 205
	MSG_ID_ONBOARD_HEALTH               = 206
)

// DialectPixhawk is the dialect represented by pixhawk.xml
var DialectPixhawk *Dialect = &Dialect{
	Name: "pixhawk",
	crcExtras: map[uint8]uint8{
		151: 108, // MSG_ID_SET_CAM_SHUTTER
		152: 86,  // MSG_ID_IMAGE_TRIGGERED
		153: 95,  // MSG_ID_IMAGE_TRIGGER_CONTROL
		154: 224, // MSG_ID_IMAGE_AVAILABLE
		160: 22,  // MSG_ID_SET_POSITION_CONTROL_OFFSET
		170: 28,  // MSG_ID_POSITION_CONTROL_SETPOINT
		171: 249, // MSG_ID_MARKER
		172: 182, // MSG_ID_RAW_AUX
		180: 153, // MSG_ID_WATCHDOG_HEARTBEAT
		181: 16,  // MSG_ID_WATCHDOG_PROCESS_INFO
		182: 29,  // MSG_ID_WATCHDOG_PROCESS_STATUS
		183: 162, // MSG_ID_WATCHDOG_COMMAND
		190: 90,  // MSG_ID_PATTERN_DETECTED
		191: 95,  // MSG_ID_POINT_OF_INTEREST
		192: 36,  // MSG_ID_POINT_OF_INTEREST_CONNECTION
		195: 88,  // MSG_ID_BRIEF_FEATURE
		200: 254, // MSG_ID_ATTITUDE_CONTROL
		205: 87,  // MSG_ID_DETECTION_STATS
		206: 19,  // MSG_ID_ONBOARD_HEALTH
	},
}
