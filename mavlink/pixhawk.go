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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Gain,
		&self.Interval,
		&self.Exposure,
		&self.CamNo,
		&self.CamMode,
		&self.TriggerPin,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SetCamShutter) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Gain,
		&self.Interval,
		&self.Exposure,
		&self.CamNo,
		&self.CamMode,
		&self.TriggerPin,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Timestamp,
		&self.Seq,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.LocalZ,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.GroundX,
		&self.GroundY,
		&self.GroundZ,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ImageTriggered) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Timestamp,
		&self.Seq,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.LocalZ,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.GroundX,
		&self.GroundY,
		&self.GroundZ,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Enable,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ImageTriggerControl) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Enable,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.CamId,
		&self.Timestamp,
		&self.ValidUntil,
		&self.ImgSeq,
		&self.ImgBufIndex,
		&self.Key,
		&self.Exposure,
		&self.Gain,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.LocalZ,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.GroundX,
		&self.GroundY,
		&self.GroundZ,
		&self.Width,
		&self.Height,
		&self.Depth,
		&self.CamNo,
		&self.Channels,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ImageAvailable) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.CamId,
		&self.Timestamp,
		&self.ValidUntil,
		&self.ImgSeq,
		&self.ImgBufIndex,
		&self.Key,
		&self.Exposure,
		&self.Gain,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.LocalZ,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.GroundX,
		&self.GroundY,
		&self.GroundZ,
		&self.Width,
		&self.Height,
		&self.Depth,
		&self.CamNo,
		&self.Channels,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.X,
		&self.Y,
		&self.Z,
		&self.Yaw,
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

func (self *SetPositionControlOffset) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.X,
		&self.Y,
		&self.Z,
		&self.Yaw,
		&self.TargetSystem,
		&self.TargetComponent,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.X,
		&self.Y,
		&self.Z,
		&self.Yaw,
		&self.Id,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *PositionControlSetpoint) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.X,
		&self.Y,
		&self.Z,
		&self.Yaw,
		&self.Id,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.X,
		&self.Y,
		&self.Z,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.Id,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Marker) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.X,
		&self.Y,
		&self.Z,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.Id,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Baro,
		&self.Adc1,
		&self.Adc2,
		&self.Adc3,
		&self.Adc4,
		&self.Vbat,
		&self.Temp,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *RawAux) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Baro,
		&self.Adc1,
		&self.Adc2,
		&self.Adc3,
		&self.Adc4,
		&self.Vbat,
		&self.Temp,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.WatchdogId,
		&self.ProcessCount,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *WatchdogHeartbeat) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.WatchdogId,
		&self.ProcessCount,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Timeout,
		&self.WatchdogId,
		&self.ProcessId,
		&self.Name,
		&self.Arguments,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *WatchdogProcessInfo) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Timeout,
		&self.WatchdogId,
		&self.ProcessId,
		&self.Name,
		&self.Arguments,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Pid,
		&self.WatchdogId,
		&self.ProcessId,
		&self.Crashes,
		&self.State,
		&self.Muted,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *WatchdogProcessStatus) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Pid,
		&self.WatchdogId,
		&self.ProcessId,
		&self.Crashes,
		&self.State,
		&self.Muted,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.WatchdogId,
		&self.ProcessId,
		&self.TargetSystemId,
		&self.CommandId,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *WatchdogCommand) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.WatchdogId,
		&self.ProcessId,
		&self.TargetSystemId,
		&self.CommandId,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Confidence,
		&self.Type,
		&self.File,
		&self.Detected,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *PatternDetected) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Confidence,
		&self.Type,
		&self.File,
		&self.Detected,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.X,
		&self.Y,
		&self.Z,
		&self.Timeout,
		&self.Type,
		&self.Color,
		&self.CoordinateSystem,
		&self.Name,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *PointOfInterest) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.X,
		&self.Y,
		&self.Z,
		&self.Timeout,
		&self.Type,
		&self.Color,
		&self.CoordinateSystem,
		&self.Name,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Xp1,
		&self.Yp1,
		&self.Zp1,
		&self.Xp2,
		&self.Yp2,
		&self.Zp2,
		&self.Timeout,
		&self.Type,
		&self.Color,
		&self.CoordinateSystem,
		&self.Name,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *PointOfInterestConnection) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Xp1,
		&self.Yp1,
		&self.Zp1,
		&self.Xp2,
		&self.Yp2,
		&self.Zp2,
		&self.Timeout,
		&self.Type,
		&self.Color,
		&self.CoordinateSystem,
		&self.Name,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.X,
		&self.Y,
		&self.Z,
		&self.Response,
		&self.Size,
		&self.Orientation,
		&self.OrientationAssignment,
		&self.Descriptor,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *BriefFeature) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.X,
		&self.Y,
		&self.Z,
		&self.Response,
		&self.Size,
		&self.Orientation,
		&self.OrientationAssignment,
		&self.Descriptor,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.Thrust,
		&self.Target,
		&self.RollManual,
		&self.PitchManual,
		&self.YawManual,
		&self.ThrustManual,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *AttitudeControl) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.Thrust,
		&self.Target,
		&self.RollManual,
		&self.PitchManual,
		&self.YawManual,
		&self.ThrustManual,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Detections,
		&self.ClusterIters,
		&self.BestScore,
		&self.BestLat,
		&self.BestLon,
		&self.BestAlt,
		&self.BestDetectionId,
		&self.BestClusterId,
		&self.BestClusterIterId,
		&self.ImagesDone,
		&self.ImagesTodo,
		&self.Fps,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *DetectionStats) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Detections,
		&self.ClusterIters,
		&self.BestScore,
		&self.BestLat,
		&self.BestLon,
		&self.BestAlt,
		&self.BestDetectionId,
		&self.BestClusterId,
		&self.BestClusterIterId,
		&self.ImagesDone,
		&self.ImagesTodo,
		&self.Fps,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Uptime,
		&self.RamTotal,
		&self.SwapTotal,
		&self.DiskTotal,
		&self.Temp,
		&self.Voltage,
		&self.NetworkLoadIn,
		&self.NetworkLoadOut,
		&self.CpuFreq,
		&self.CpuLoad,
		&self.RamUsage,
		&self.SwapUsage,
		&self.DiskHealth,
		&self.DiskUsage,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *OnboardHealth) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Uptime,
		&self.RamTotal,
		&self.SwapTotal,
		&self.DiskTotal,
		&self.Temp,
		&self.Voltage,
		&self.NetworkLoadIn,
		&self.NetworkLoadOut,
		&self.CpuFreq,
		&self.CpuLoad,
		&self.RamUsage,
		&self.SwapUsage,
		&self.DiskHealth,
		&self.DiskUsage,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
		181: 42,  // MSG_ID_WATCHDOG_PROCESS_INFO
		182: 29,  // MSG_ID_WATCHDOG_PROCESS_STATUS
		183: 162, // MSG_ID_WATCHDOG_COMMAND
		190: 74,  // MSG_ID_PATTERN_DETECTED
		191: 19,  // MSG_ID_POINT_OF_INTEREST
		192: 236, // MSG_ID_POINT_OF_INTEREST_CONNECTION
		195: 232, // MSG_ID_BRIEF_FEATURE
		200: 254, // MSG_ID_ATTITUDE_CONTROL
		205: 87,  // MSG_ID_DETECTION_STATS
		206: 19,  // MSG_ID_ONBOARD_HEALTH
	},
}
