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
	MAV_CMD_DO_MOTOR_TEST      = 209 // Mission command to perform motor test
	MAV_CMD_DO_GRIPPER         = 211 // Mission command to operate EPM gripper
	MAV_CMD_DO_AUTOTUNE_ENABLE = 212 // Enable/disable autotune
	MAV_CMD_NAV_ALTITUDE_WAIT  = 83  // Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control surfaces to prevent them seizing up.
	MAV_CMD_DO_START_MAG_CAL   = 4   // Initiate a magnetometer calibration
	MAV_CMD_DO_ACCEPT_MAG_CAL  = 5   // Initiate a magnetometer calibration
	MAV_CMD_DO_CANCEL_MAG_CAL  = 6   // Cancel a running magnetometer calibration
)

// LimitsState:
const (
	LIMITS_INIT       = 0 //  pre-initialization
	LIMITS_DISABLED   = 1 //  disabled
	LIMITS_ENABLED    = 2 //  checking limits
	LIMITS_TRIGGERED  = 3 //  a limit has been breached
	LIMITS_RECOVERING = 4 //  taking action eg. RTL
	LIMITS_RECOVERED  = 5 //  we're no longer in breach of a limit
)

// LimitModule:
const (
	LIMIT_GPSLOCK  = 1 //  pre-initialization
	LIMIT_GEOFENCE = 2 //  disabled
	LIMIT_ALTITUDE = 4 //  checking limits
)

// RallyFlags: Flags in RALLY_POINT message
const (
	FAVORABLE_WIND   = 1 // Flag set when requiring favorable winds for landing.
	LAND_IMMEDIATELY = 2 // Flag set when plane is to immediately descend to break altitude and land without GCS intervention.  Flag not set when plane is to loiter at Rally point until commanded to land.
)

// ParachuteAction:
const (
	PARACHUTE_DISABLE = 0 // Disable parachute release
	PARACHUTE_ENABLE  = 1 // Enable parachute release
	PARACHUTE_RELEASE = 2 // Release parachute
)

// MotorTestThrottleType:
const (
	MOTOR_TEST_THROTTLE_PERCENT = 0 // throttle as a percentage from 0 ~ 100
	MOTOR_TEST_THROTTLE_PWM     = 1 // throttle as an absolute PWM value (normally in range of 1000~2000)
	MOTOR_TEST_THROTTLE_PILOT   = 2 // throttle pass-through from pilot's transmitter
)

// GripperActions: Gripper actions.
const (
	GRIPPER_ACTION_RELEASE = 0 // gripper release of cargo
	GRIPPER_ACTION_GRAB    = 1 // gripper grabs onto cargo
)

// CameraStatusTypes:
const (
	CAMERA_STATUS_TYPE_HEARTBEAT  = 0 // Camera heartbeat, announce camera component ID at 1hz
	CAMERA_STATUS_TYPE_TRIGGER    = 1 // Camera image triggered
	CAMERA_STATUS_TYPE_DISCONNECT = 2 // Camera connection lost
	CAMERA_STATUS_TYPE_ERROR      = 3 // Camera unknown error
	CAMERA_STATUS_TYPE_LOWBATT    = 4 // Camera battery low. Parameter p1 shows reported voltage
	CAMERA_STATUS_TYPE_LOWSTORE   = 5 // Camera storage low. Parameter p1 shows reported shots remaining
	CAMERA_STATUS_TYPE_LOWSTOREV  = 6 // Camera storage low. Parameter p1 shows reported video minutes remaining
)

// CameraFeedbackFlags:
const (
	CAMERA_FEEDBACK_PHOTO       = 0 // Shooting photos, not video
	CAMERA_FEEDBACK_VIDEO       = 1 // Shooting video, not stills
	CAMERA_FEEDBACK_BADEXPOSURE = 2 // Unable to achieve requested exposure (e.g. shutter speed too low)
	CAMERA_FEEDBACK_CLOSEDLOOP  = 3 // Closed loop feedback from camera, we know for sure it has successfully taken a picture
	CAMERA_FEEDBACK_OPENLOOP    = 4 // Open loop camera, an image trigger has been requested but we can't know for sure it has successfully taken a picture
)

// MavModeGimbal:
const (
	MAV_MODE_GIMBAL_UNINITIALIZED     = 0 // Gimbal is powered on but has not started initializing yet
	MAV_MODE_GIMBAL_CALIBRATING_PITCH = 1 // Gimbal is currently running calibration on the pitch axis
	MAV_MODE_GIMBAL_CALIBRATING_ROLL  = 2 // Gimbal is currently running calibration on the roll axis
	MAV_MODE_GIMBAL_CALIBRATING_YAW   = 3 // Gimbal is currently running calibration on the yaw axis
	MAV_MODE_GIMBAL_INITIALIZED       = 4 // Gimbal has finished calibrating and initializing, but is relaxed pending reception of first rate command from copter
	MAV_MODE_GIMBAL_ACTIVE            = 5 // Gimbal is actively stabilizing
	MAV_MODE_GIMBAL_RATE_CMD_TIMEOUT  = 6 // Gimbal is relaxed because it missed more than 10 expected rate command messages in a row.  Gimbal will move back to active mode when it receives a new rate command
)

// GimbalAxis:
const (
	GIMBAL_AXIS_YAW   = 0 // Gimbal yaw axis
	GIMBAL_AXIS_PITCH = 1 // Gimbal pitch axis
	GIMBAL_AXIS_ROLL  = 2 // Gimbal roll axis
)

// GimbalAxisCalibrationStatus:
const (
	GIMBAL_AXIS_CALIBRATION_STATUS_IN_PROGRESS = 0 // Axis calibration is in progress
	GIMBAL_AXIS_CALIBRATION_STATUS_SUCCEEDED   = 1 // Axis calibration succeeded
	GIMBAL_AXIS_CALIBRATION_STATUS_FAILED      = 2 // Axis calibration failed
)

// FactoryTest:
const (
	FACTORY_TEST_AXIS_RANGE_LIMITS = 0 // Tests to make sure each axis can move to its mechanical limits
)

// GoproCmdResult:
const (
	GOPRO_CMD_RESULT_UNKNOWN                        = 0  // The result of the command is unknown
	GOPRO_CMD_RESULT_SUCCESSFUL                     = 1  // The command was successfully sent, and a response was successfully received
	GOPRO_CMD_RESULT_SEND_CMD_START_TIMEOUT         = 2  // Timed out waiting for the GoPro to acknowledge our request to send a command
	GOPRO_CMD_RESULT_SEND_CMD_COMPLETE_TIMEOUT      = 3  // Timed out waiting for the GoPro to read the command
	GOPRO_CMD_RESULT_GET_RESPONSE_START_TIMEOUT     = 4  // Timed out waiting for the GoPro to begin transmitting a response to the command
	GOPRO_CMD_RESULT_GET_RESPONSE_COMPLETE_TIMEOUT  = 5  // Timed out waiting for the GoPro to finish transmitting a response to the command
	GOPRO_CMD_RESULT_GET_CMD_COMPLETE_TIMEOUT       = 6  // Timed out waiting for the GoPro to finish transmitting its own command
	GOPRO_CMD_RESULT_SEND_RESPONSE_START_TIMEOUT    = 7  // Timed out waiting for the GoPro to start reading a response to its own command
	GOPRO_CMD_RESULT_SEND_RESPONSE_COMPLETE_TIMEOUT = 8  // Timed out waiting for the GoPro to finish reading a response to its own command
	GOPRO_CMD_RESULT_PREEMPTED                      = 9  // Command to the GoPro was preempted by the GoPro sending its own command
	GOPRO_CMD_RECEIVED_DATA_OVERFLOW                = 10 // More data than expected received in response to the command
	GOPRO_CMD_RECEIVED_DATA_UNDERFLOW               = 11 // Less data than expected received in response to the command
)

// LedControlPattern:
const (
	LED_CONTROL_PATTERN_OFF            = 0   // LED patterns off (return control to regular vehicle control)
	LED_CONTROL_PATTERN_FIRMWAREUPDATE = 1   // LEDs show pattern during firmware update
	LED_CONTROL_PATTERN_CUSTOM         = 255 // Custom Pattern using custom bytes fields
)

// EkfStatusFlags: Flags in EKF_STATUS message
const (
	EKF_ATTITUDE           = 1   // set if EKF's attitude estimate is good
	EKF_VELOCITY_HORIZ     = 2   // set if EKF's horizontal velocity estimate is good
	EKF_VELOCITY_VERT      = 4   // set if EKF's vertical velocity estimate is good
	EKF_POS_HORIZ_REL      = 8   // set if EKF's horizontal position (relative) estimate is good
	EKF_POS_HORIZ_ABS      = 16  // set if EKF's horizontal position (absolute) estimate is good
	EKF_POS_VERT_ABS       = 32  // set if EKF's vertical position (absolute) estimate is good
	EKF_POS_VERT_AGL       = 64  // set if EKF's vertical position (above ground) estimate is good
	EKF_CONST_POS_MODE     = 128 // EKF is in constant position mode and does not know it's absolute or relative position
	EKF_PRED_POS_HORIZ_REL = 8   // set if EKF's predicted horizontal position (relative) estimate is good
	EKF_PRED_POS_HORIZ_ABS = 9   // set if EKF's predicted horizontal position (absolute) estimate is good
)

// MagCalStatus:
const (
	MAG_CAL_NOT_STARTED      = 0 //
	MAG_CAL_WAITING_TO_START = 1 //
	MAG_CAL_RUNNING_STEP_ONE = 2 //
	MAG_CAL_RUNNING_STEP_TWO = 3 //
	MAG_CAL_SUCCESS          = 4 //
	MAG_CAL_FAILED           = 5 //
)

// PidTuningAxis:
const (
	PID_TUNING_ROLL  = 1 //
	PID_TUNING_PITCH = 2 //
	PID_TUNING_YAW   = 3 //
	PID_TUNING_ACCZ  = 4 //
	PID_TUNING_STEER = 5 //
)

// Offsets and calibrations values for hardware
//         sensors. This makes it easier to debug the calibration process.
type SensorOffsets struct {
	MagDeclination float32 // magnetic declination (radians)
	RawPress       int32   // raw pressure from barometer
	RawTemp        int32   // raw temperature from barometer
	GyroCalX       float32 // gyro X calibration
	GyroCalY       float32 // gyro Y calibration
	GyroCalZ       float32 // gyro Z calibration
	AccelCalX      float32 // accel X calibration
	AccelCalY      float32 // accel Y calibration
	AccelCalZ      float32 // accel Z calibration
	MagOfsX        int16   // magnetometer X offset
	MagOfsY        int16   // magnetometer Y offset
	MagOfsZ        int16   // magnetometer Z offset
}

func (self *SensorOffsets) MsgID() uint8 {
	return 150
}

func (self *SensorOffsets) MsgName() string {
	return "SensorOffsets"
}

func (self *SensorOffsets) Pack(p *Packet) error {
	payload := make([]byte, 42)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.MagDeclination))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.RawPress))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.RawTemp))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.GyroCalX))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.GyroCalY))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.GyroCalZ))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.AccelCalX))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.AccelCalY))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.AccelCalZ))
	binary.LittleEndian.PutUint16(payload[36:], uint16(self.MagOfsX))
	binary.LittleEndian.PutUint16(payload[38:], uint16(self.MagOfsY))
	binary.LittleEndian.PutUint16(payload[40:], uint16(self.MagOfsZ))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SensorOffsets) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}
	self.MagDeclination = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.RawPress = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.RawTemp = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.GyroCalX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.GyroCalY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.GyroCalZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.AccelCalX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.AccelCalY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.AccelCalZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.MagOfsX = int16(binary.LittleEndian.Uint16(p.Payload[36:]))
	self.MagOfsY = int16(binary.LittleEndian.Uint16(p.Payload[38:]))
	self.MagOfsZ = int16(binary.LittleEndian.Uint16(p.Payload[40:]))
	return nil
}

// Deprecated. Use MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS instead. Set the magnetometer offsets
type SetMagOffsets struct {
	MagOfsX         int16 // magnetometer X offset
	MagOfsY         int16 // magnetometer Y offset
	MagOfsZ         int16 // magnetometer Z offset
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *SetMagOffsets) MsgID() uint8 {
	return 151
}

func (self *SetMagOffsets) MsgName() string {
	return "SetMagOffsets"
}

func (self *SetMagOffsets) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.MagOfsX))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.MagOfsY))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.MagOfsZ))
	payload[6] = byte(self.TargetSystem)
	payload[7] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SetMagOffsets) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	self.MagOfsX = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.MagOfsY = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.MagOfsZ = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.TargetSystem = uint8(p.Payload[6])
	self.TargetComponent = uint8(p.Payload[7])
	return nil
}

// state of APM memory
type Meminfo struct {
	Brkval  uint16 // heap top
	Freemem uint16 // free memory
}

func (self *Meminfo) MsgID() uint8 {
	return 152
}

func (self *Meminfo) MsgName() string {
	return "Meminfo"
}

func (self *Meminfo) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Brkval))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.Freemem))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Meminfo) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	self.Brkval = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.Freemem = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	return nil
}

// raw ADC output
type ApAdc struct {
	Adc1 uint16 // ADC output 1
	Adc2 uint16 // ADC output 2
	Adc3 uint16 // ADC output 3
	Adc4 uint16 // ADC output 4
	Adc5 uint16 // ADC output 5
	Adc6 uint16 // ADC output 6
}

func (self *ApAdc) MsgID() uint8 {
	return 153
}

func (self *ApAdc) MsgName() string {
	return "ApAdc"
}

func (self *ApAdc) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Adc1))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.Adc2))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Adc3))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.Adc4))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Adc5))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.Adc6))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ApAdc) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	self.Adc1 = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.Adc2 = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.Adc3 = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.Adc4 = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.Adc5 = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Adc6 = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	return nil
}

// Configure on-board Camera Control System.
type DigicamConfigure struct {
	ExtraValue      float32 // Correspondent value to given extra_param
	ShutterSpeed    uint16  // Divisor number //e.g. 1000 means 1/1000 (0 means ignore)
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Mode            uint8   // Mode enumeration from 1 to N //P, TV, AV, M, Etc (0 means ignore)
	Aperture        uint8   // F stop number x 10 //e.g. 28 means 2.8 (0 means ignore)
	Iso             uint8   // ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore)
	ExposureType    uint8   // Exposure type enumeration from 1 to N (0 means ignore)
	CommandId       uint8   // Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once
	EngineCutOff    uint8   // Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
	ExtraParam      uint8   // Extra parameters enumeration (0 means ignore)
}

func (self *DigicamConfigure) MsgID() uint8 {
	return 154
}

func (self *DigicamConfigure) MsgName() string {
	return "DigicamConfigure"
}

func (self *DigicamConfigure) Pack(p *Packet) error {
	payload := make([]byte, 15)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.ExtraValue))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.ShutterSpeed))
	payload[6] = byte(self.TargetSystem)
	payload[7] = byte(self.TargetComponent)
	payload[8] = byte(self.Mode)
	payload[9] = byte(self.Aperture)
	payload[10] = byte(self.Iso)
	payload[11] = byte(self.ExposureType)
	payload[12] = byte(self.CommandId)
	payload[13] = byte(self.EngineCutOff)
	payload[14] = byte(self.ExtraParam)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *DigicamConfigure) Unpack(p *Packet) error {
	if len(p.Payload) < 15 {
		return fmt.Errorf("payload too small")
	}
	self.ExtraValue = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.ShutterSpeed = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.TargetSystem = uint8(p.Payload[6])
	self.TargetComponent = uint8(p.Payload[7])
	self.Mode = uint8(p.Payload[8])
	self.Aperture = uint8(p.Payload[9])
	self.Iso = uint8(p.Payload[10])
	self.ExposureType = uint8(p.Payload[11])
	self.CommandId = uint8(p.Payload[12])
	self.EngineCutOff = uint8(p.Payload[13])
	self.ExtraParam = uint8(p.Payload[14])
	return nil
}

// Control on-board Camera Control System to take shots.
type DigicamControl struct {
	ExtraValue      float32 // Correspondent value to given extra_param
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Session         uint8   // 0: stop, 1: start or keep it up //Session control e.g. show/hide lens
	ZoomPos         uint8   // 1 to N //Zoom's absolute position (0 means ignore)
	ZoomStep        int8    // -100 to 100 //Zooming step value to offset zoom from the current position
	FocusLock       uint8   // 0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus
	Shot            uint8   // 0: ignore, 1: shot or start filming
	CommandId       uint8   // Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once
	ExtraParam      uint8   // Extra parameters enumeration (0 means ignore)
}

func (self *DigicamControl) MsgID() uint8 {
	return 155
}

func (self *DigicamControl) MsgName() string {
	return "DigicamControl"
}

func (self *DigicamControl) Pack(p *Packet) error {
	payload := make([]byte, 13)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.ExtraValue))
	payload[4] = byte(self.TargetSystem)
	payload[5] = byte(self.TargetComponent)
	payload[6] = byte(self.Session)
	payload[7] = byte(self.ZoomPos)
	payload[8] = byte(self.ZoomStep)
	payload[9] = byte(self.FocusLock)
	payload[10] = byte(self.Shot)
	payload[11] = byte(self.CommandId)
	payload[12] = byte(self.ExtraParam)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *DigicamControl) Unpack(p *Packet) error {
	if len(p.Payload) < 13 {
		return fmt.Errorf("payload too small")
	}
	self.ExtraValue = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.TargetSystem = uint8(p.Payload[4])
	self.TargetComponent = uint8(p.Payload[5])
	self.Session = uint8(p.Payload[6])
	self.ZoomPos = uint8(p.Payload[7])
	self.ZoomStep = int8(p.Payload[8])
	self.FocusLock = uint8(p.Payload[9])
	self.Shot = uint8(p.Payload[10])
	self.CommandId = uint8(p.Payload[11])
	self.ExtraParam = uint8(p.Payload[12])
	return nil
}

// Message to configure a camera mount, directional antenna, etc.
type MountConfigure struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	MountMode       uint8 // mount operating mode (see MAV_MOUNT_MODE enum)
	StabRoll        uint8 // (1 = yes, 0 = no)
	StabPitch       uint8 // (1 = yes, 0 = no)
	StabYaw         uint8 // (1 = yes, 0 = no)
}

func (self *MountConfigure) MsgID() uint8 {
	return 156
}

func (self *MountConfigure) MsgName() string {
	return "MountConfigure"
}

func (self *MountConfigure) Pack(p *Packet) error {
	payload := make([]byte, 6)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.MountMode)
	payload[3] = byte(self.StabRoll)
	payload[4] = byte(self.StabPitch)
	payload[5] = byte(self.StabYaw)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MountConfigure) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.MountMode = uint8(p.Payload[2])
	self.StabRoll = uint8(p.Payload[3])
	self.StabPitch = uint8(p.Payload[4])
	self.StabYaw = uint8(p.Payload[5])
	return nil
}

// Message to control a camera mount, directional antenna, etc.
type MountControl struct {
	InputA          int32 // pitch(deg*100) or lat, depending on mount mode
	InputB          int32 // roll(deg*100) or lon depending on mount mode
	InputC          int32 // yaw(deg*100) or alt (in cm) depending on mount mode
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	SavePosition    uint8 // if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
}

func (self *MountControl) MsgID() uint8 {
	return 157
}

func (self *MountControl) MsgName() string {
	return "MountControl"
}

func (self *MountControl) Pack(p *Packet) error {
	payload := make([]byte, 15)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.InputA))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.InputB))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.InputC))
	payload[12] = byte(self.TargetSystem)
	payload[13] = byte(self.TargetComponent)
	payload[14] = byte(self.SavePosition)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MountControl) Unpack(p *Packet) error {
	if len(p.Payload) < 15 {
		return fmt.Errorf("payload too small")
	}
	self.InputA = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.InputB = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.InputC = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.TargetSystem = uint8(p.Payload[12])
	self.TargetComponent = uint8(p.Payload[13])
	self.SavePosition = uint8(p.Payload[14])
	return nil
}

// Message with some status from APM to GCS about camera or antenna mount
type MountStatus struct {
	PointingA       int32 // pitch(deg*100)
	PointingB       int32 // roll(deg*100)
	PointingC       int32 // yaw(deg*100)
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *MountStatus) MsgID() uint8 {
	return 158
}

func (self *MountStatus) MsgName() string {
	return "MountStatus"
}

func (self *MountStatus) Pack(p *Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.PointingA))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.PointingB))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.PointingC))
	payload[12] = byte(self.TargetSystem)
	payload[13] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MountStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	self.PointingA = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.PointingB = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.PointingC = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.TargetSystem = uint8(p.Payload[12])
	self.TargetComponent = uint8(p.Payload[13])
	return nil
}

// A fence point. Used to set a point when from
// 	      GCS -> MAV. Also used to return a point from MAV -> GCS
type FencePoint struct {
	Lat             float32 // Latitude of point
	Lng             float32 // Longitude of point
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Idx             uint8   // point index (first point is 1, 0 is for return point)
	Count           uint8   // total number of points (for sanity checking)
}

func (self *FencePoint) MsgID() uint8 {
	return 160
}

func (self *FencePoint) MsgName() string {
	return "FencePoint"
}

func (self *FencePoint) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Lat))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Lng))
	payload[8] = byte(self.TargetSystem)
	payload[9] = byte(self.TargetComponent)
	payload[10] = byte(self.Idx)
	payload[11] = byte(self.Count)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FencePoint) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	self.Lat = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Lng = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.TargetSystem = uint8(p.Payload[8])
	self.TargetComponent = uint8(p.Payload[9])
	self.Idx = uint8(p.Payload[10])
	self.Count = uint8(p.Payload[11])
	return nil
}

// Request a current fence point from MAV
type FenceFetchPoint struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	Idx             uint8 // point index (first point is 1, 0 is for return point)
}

func (self *FenceFetchPoint) MsgID() uint8 {
	return 161
}

func (self *FenceFetchPoint) MsgName() string {
	return "FenceFetchPoint"
}

func (self *FenceFetchPoint) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.Idx)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FenceFetchPoint) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.Idx = uint8(p.Payload[2])
	return nil
}

// Status of geo-fencing. Sent in extended
// 	    status stream when fencing enabled
type FenceStatus struct {
	BreachTime   uint32 // time of last breach in milliseconds since boot
	BreachCount  uint16 // number of fence breaches
	BreachStatus uint8  // 0 if currently inside fence, 1 if outside
	BreachType   uint8  // last breach type (see FENCE_BREACH_* enum)
}

func (self *FenceStatus) MsgID() uint8 {
	return 162
}

func (self *FenceStatus) MsgName() string {
	return "FenceStatus"
}

func (self *FenceStatus) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.BreachTime))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.BreachCount))
	payload[6] = byte(self.BreachStatus)
	payload[7] = byte(self.BreachType)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FenceStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	self.BreachTime = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.BreachCount = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.BreachStatus = uint8(p.Payload[6])
	self.BreachType = uint8(p.Payload[7])
	return nil
}

// Status of DCM attitude estimator
type Ahrs struct {
	Omegaix     float32 // X gyro drift estimate rad/s
	Omegaiy     float32 // Y gyro drift estimate rad/s
	Omegaiz     float32 // Z gyro drift estimate rad/s
	AccelWeight float32 // average accel_weight
	RenormVal   float32 // average renormalisation value
	ErrorRp     float32 // average error_roll_pitch value
	ErrorYaw    float32 // average error_yaw value
}

func (self *Ahrs) MsgID() uint8 {
	return 163
}

func (self *Ahrs) MsgName() string {
	return "Ahrs"
}

func (self *Ahrs) Pack(p *Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Omegaix))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Omegaiy))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Omegaiz))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.AccelWeight))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.RenormVal))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.ErrorRp))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.ErrorYaw))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Ahrs) Unpack(p *Packet) error {
	if len(p.Payload) < 28 {
		return fmt.Errorf("payload too small")
	}
	self.Omegaix = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Omegaiy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Omegaiz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.AccelWeight = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.RenormVal = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.ErrorRp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.ErrorYaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	return nil
}

// Status of simulation environment, if used
type Simstate struct {
	Roll  float32 // Roll angle (rad)
	Pitch float32 // Pitch angle (rad)
	Yaw   float32 // Yaw angle (rad)
	Xacc  float32 // X acceleration m/s/s
	Yacc  float32 // Y acceleration m/s/s
	Zacc  float32 // Z acceleration m/s/s
	Xgyro float32 // Angular speed around X axis rad/s
	Ygyro float32 // Angular speed around Y axis rad/s
	Zgyro float32 // Angular speed around Z axis rad/s
	Lat   int32   // Latitude in degrees * 1E7
	Lng   int32   // Longitude in degrees * 1E7
}

func (self *Simstate) MsgID() uint8 {
	return 164
}

func (self *Simstate) MsgName() string {
	return "Simstate"
}

func (self *Simstate) Pack(p *Packet) error {
	payload := make([]byte, 44)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Xacc))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Yacc))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Zacc))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Xgyro))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Ygyro))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Zgyro))
	binary.LittleEndian.PutUint32(payload[36:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[40:], uint32(self.Lng))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Simstate) Unpack(p *Packet) error {
	if len(p.Payload) < 44 {
		return fmt.Errorf("payload too small")
	}
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Xacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Yacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Zacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Xgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Ygyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Zgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Lng = int32(binary.LittleEndian.Uint32(p.Payload[40:]))
	return nil
}

// Status of key hardware
type Hwstatus struct {
	Vcc    uint16 // board voltage (mV)
	I2cerr uint8  // I2C error count
}

func (self *Hwstatus) MsgID() uint8 {
	return 165
}

func (self *Hwstatus) MsgName() string {
	return "Hwstatus"
}

func (self *Hwstatus) Pack(p *Packet) error {
	payload := make([]byte, 3)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Vcc))
	payload[2] = byte(self.I2cerr)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Hwstatus) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.Vcc = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.I2cerr = uint8(p.Payload[2])
	return nil
}

// Status generated by radio
type Radio struct {
	Rxerrors uint16 // receive errors
	Fixed    uint16 // count of error corrected packets
	Rssi     uint8  // local signal strength
	Remrssi  uint8  // remote signal strength
	Txbuf    uint8  // how full the tx buffer is as a percentage
	Noise    uint8  // background noise level
	Remnoise uint8  // remote background noise level
}

func (self *Radio) MsgID() uint8 {
	return 166
}

func (self *Radio) MsgName() string {
	return "Radio"
}

func (self *Radio) Pack(p *Packet) error {
	payload := make([]byte, 9)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Rxerrors))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.Fixed))
	payload[4] = byte(self.Rssi)
	payload[5] = byte(self.Remrssi)
	payload[6] = byte(self.Txbuf)
	payload[7] = byte(self.Noise)
	payload[8] = byte(self.Remnoise)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Radio) Unpack(p *Packet) error {
	if len(p.Payload) < 9 {
		return fmt.Errorf("payload too small")
	}
	self.Rxerrors = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.Fixed = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.Rssi = uint8(p.Payload[4])
	self.Remrssi = uint8(p.Payload[5])
	self.Txbuf = uint8(p.Payload[6])
	self.Noise = uint8(p.Payload[7])
	self.Remnoise = uint8(p.Payload[8])
	return nil
}

// Status of AP_Limits. Sent in extended
// 	    status stream when AP_Limits is enabled
type LimitsStatus struct {
	LastTrigger   uint32 // time of last breach in milliseconds since boot
	LastAction    uint32 // time of last recovery action in milliseconds since boot
	LastRecovery  uint32 // time of last successful recovery in milliseconds since boot
	LastClear     uint32 // time of last all-clear in milliseconds since boot
	BreachCount   uint16 // number of fence breaches
	LimitsState   uint8  // state of AP_Limits, (see enum LimitState, LIMITS_STATE)
	ModsEnabled   uint8  // AP_Limit_Module bitfield of enabled modules, (see enum moduleid or LIMIT_MODULE)
	ModsRequired  uint8  // AP_Limit_Module bitfield of required modules, (see enum moduleid or LIMIT_MODULE)
	ModsTriggered uint8  // AP_Limit_Module bitfield of triggered modules, (see enum moduleid or LIMIT_MODULE)
}

func (self *LimitsStatus) MsgID() uint8 {
	return 167
}

func (self *LimitsStatus) MsgName() string {
	return "LimitsStatus"
}

func (self *LimitsStatus) Pack(p *Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.LastTrigger))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.LastAction))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.LastRecovery))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.LastClear))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.BreachCount))
	payload[18] = byte(self.LimitsState)
	payload[19] = byte(self.ModsEnabled)
	payload[20] = byte(self.ModsRequired)
	payload[21] = byte(self.ModsTriggered)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *LimitsStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	self.LastTrigger = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.LastAction = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.LastRecovery = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.LastClear = uint32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.BreachCount = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.LimitsState = uint8(p.Payload[18])
	self.ModsEnabled = uint8(p.Payload[19])
	self.ModsRequired = uint8(p.Payload[20])
	self.ModsTriggered = uint8(p.Payload[21])
	return nil
}

// Wind estimation
type Wind struct {
	Direction float32 // wind direction that wind is coming from (degrees)
	Speed     float32 // wind speed in ground plane (m/s)
	SpeedZ    float32 // vertical wind speed (m/s)
}

func (self *Wind) MsgID() uint8 {
	return 168
}

func (self *Wind) MsgName() string {
	return "Wind"
}

func (self *Wind) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Direction))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Speed))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.SpeedZ))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Wind) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	self.Direction = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Speed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.SpeedZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	return nil
}

// Data packet, size 16
type Data16 struct {
	Type uint8     // data type
	Len  uint8     // data length
	Data [16]uint8 // raw data
}

func (self *Data16) MsgID() uint8 {
	return 169
}

func (self *Data16) MsgName() string {
	return "Data16"
}

func (self *Data16) Pack(p *Packet) error {
	payload := make([]byte, 18)
	payload[0] = byte(self.Type)
	payload[1] = byte(self.Len)
	copy(payload[2:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Data16) Unpack(p *Packet) error {
	if len(p.Payload) < 18 {
		return fmt.Errorf("payload too small")
	}
	self.Type = uint8(p.Payload[0])
	self.Len = uint8(p.Payload[1])
	copy(self.Data[:], p.Payload[2:18])
	return nil
}

// Data packet, size 32
type Data32 struct {
	Type uint8     // data type
	Len  uint8     // data length
	Data [32]uint8 // raw data
}

func (self *Data32) MsgID() uint8 {
	return 170
}

func (self *Data32) MsgName() string {
	return "Data32"
}

func (self *Data32) Pack(p *Packet) error {
	payload := make([]byte, 34)
	payload[0] = byte(self.Type)
	payload[1] = byte(self.Len)
	copy(payload[2:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Data32) Unpack(p *Packet) error {
	if len(p.Payload) < 34 {
		return fmt.Errorf("payload too small")
	}
	self.Type = uint8(p.Payload[0])
	self.Len = uint8(p.Payload[1])
	copy(self.Data[:], p.Payload[2:34])
	return nil
}

// Data packet, size 64
type Data64 struct {
	Type uint8     // data type
	Len  uint8     // data length
	Data [64]uint8 // raw data
}

func (self *Data64) MsgID() uint8 {
	return 171
}

func (self *Data64) MsgName() string {
	return "Data64"
}

func (self *Data64) Pack(p *Packet) error {
	payload := make([]byte, 66)
	payload[0] = byte(self.Type)
	payload[1] = byte(self.Len)
	copy(payload[2:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Data64) Unpack(p *Packet) error {
	if len(p.Payload) < 66 {
		return fmt.Errorf("payload too small")
	}
	self.Type = uint8(p.Payload[0])
	self.Len = uint8(p.Payload[1])
	copy(self.Data[:], p.Payload[2:66])
	return nil
}

// Data packet, size 96
type Data96 struct {
	Type uint8     // data type
	Len  uint8     // data length
	Data [96]uint8 // raw data
}

func (self *Data96) MsgID() uint8 {
	return 172
}

func (self *Data96) MsgName() string {
	return "Data96"
}

func (self *Data96) Pack(p *Packet) error {
	payload := make([]byte, 98)
	payload[0] = byte(self.Type)
	payload[1] = byte(self.Len)
	copy(payload[2:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Data96) Unpack(p *Packet) error {
	if len(p.Payload) < 98 {
		return fmt.Errorf("payload too small")
	}
	self.Type = uint8(p.Payload[0])
	self.Len = uint8(p.Payload[1])
	copy(self.Data[:], p.Payload[2:98])
	return nil
}

// Rangefinder reporting
type Rangefinder struct {
	Distance float32 // distance in meters
	Voltage  float32 // raw voltage if available, zero otherwise
}

func (self *Rangefinder) MsgID() uint8 {
	return 173
}

func (self *Rangefinder) MsgName() string {
	return "Rangefinder"
}

func (self *Rangefinder) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Distance))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Voltage))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Rangefinder) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	self.Distance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Voltage = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	return nil
}

// Airspeed auto-calibration
type AirspeedAutocal struct {
	Vx           float32 // GPS velocity north m/s
	Vy           float32 // GPS velocity east m/s
	Vz           float32 // GPS velocity down m/s
	DiffPressure float32 // Differential pressure pascals
	Eas2tas      float32 // Estimated to true airspeed ratio
	Ratio        float32 // Airspeed ratio
	StateX       float32 // EKF state x
	StateY       float32 // EKF state y
	StateZ       float32 // EKF state z
	Pax          float32 // EKF Pax
	Pby          float32 // EKF Pby
	Pcz          float32 // EKF Pcz
}

func (self *AirspeedAutocal) MsgID() uint8 {
	return 174
}

func (self *AirspeedAutocal) MsgName() string {
	return "AirspeedAutocal"
}

func (self *AirspeedAutocal) Pack(p *Packet) error {
	payload := make([]byte, 48)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Vx))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Vy))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Vz))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.DiffPressure))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Eas2tas))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Ratio))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.StateX))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.StateY))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.StateZ))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Pax))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.Pby))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.Pcz))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *AirspeedAutocal) Unpack(p *Packet) error {
	if len(p.Payload) < 48 {
		return fmt.Errorf("payload too small")
	}
	self.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.DiffPressure = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Eas2tas = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Ratio = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.StateX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.StateY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.StateZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Pax = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Pby = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.Pcz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	return nil
}

// A rally point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS
type RallyPoint struct {
	Lat             int32  // Latitude of point in degrees * 1E7
	Lng             int32  // Longitude of point in degrees * 1E7
	Alt             int16  // Transit / loiter altitude in meters relative to home
	BreakAlt        int16  // Break altitude in meters relative to home
	LandDir         uint16 // Heading to aim for when landing. In centi-degrees.
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
	Idx             uint8  // point index (first point is 0)
	Count           uint8  // total number of points (for sanity checking)
	Flags           uint8  // See RALLY_FLAGS enum for definition of the bitmask.
}

func (self *RallyPoint) MsgID() uint8 {
	return 175
}

func (self *RallyPoint) MsgName() string {
	return "RallyPoint"
}

func (self *RallyPoint) Pack(p *Packet) error {
	payload := make([]byte, 19)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Lng))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Alt))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.BreakAlt))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.LandDir))
	payload[14] = byte(self.TargetSystem)
	payload[15] = byte(self.TargetComponent)
	payload[16] = byte(self.Idx)
	payload[17] = byte(self.Count)
	payload[18] = byte(self.Flags)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RallyPoint) Unpack(p *Packet) error {
	if len(p.Payload) < 19 {
		return fmt.Errorf("payload too small")
	}
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Lng = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Alt = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.BreakAlt = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.LandDir = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.TargetSystem = uint8(p.Payload[14])
	self.TargetComponent = uint8(p.Payload[15])
	self.Idx = uint8(p.Payload[16])
	self.Count = uint8(p.Payload[17])
	self.Flags = uint8(p.Payload[18])
	return nil
}

// Request a current rally point from MAV. MAV should respond with a RALLY_POINT message. MAV should not respond if the request is invalid.
type RallyFetchPoint struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	Idx             uint8 // point index (first point is 0)
}

func (self *RallyFetchPoint) MsgID() uint8 {
	return 176
}

func (self *RallyFetchPoint) MsgName() string {
	return "RallyFetchPoint"
}

func (self *RallyFetchPoint) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.Idx)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RallyFetchPoint) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.Idx = uint8(p.Payload[2])
	return nil
}

// Status of compassmot calibration
type CompassmotStatus struct {
	Current       float32 // current (amps)
	Compensationx float32 // Motor Compensation X
	Compensationy float32 // Motor Compensation Y
	Compensationz float32 // Motor Compensation Z
	Throttle      uint16  // throttle (percent*10)
	Interference  uint16  // interference (percent)
}

func (self *CompassmotStatus) MsgID() uint8 {
	return 177
}

func (self *CompassmotStatus) MsgName() string {
	return "CompassmotStatus"
}

func (self *CompassmotStatus) Pack(p *Packet) error {
	payload := make([]byte, 20)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Current))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Compensationx))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Compensationy))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Compensationz))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Throttle))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.Interference))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *CompassmotStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 20 {
		return fmt.Errorf("payload too small")
	}
	self.Current = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Compensationx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Compensationy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Compensationz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Throttle = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.Interference = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	return nil
}

// Status of secondary AHRS filter if available
type Ahrs2 struct {
	Roll     float32 // Roll angle (rad)
	Pitch    float32 // Pitch angle (rad)
	Yaw      float32 // Yaw angle (rad)
	Altitude float32 // Altitude (MSL)
	Lat      int32   // Latitude in degrees * 1E7
	Lng      int32   // Longitude in degrees * 1E7
}

func (self *Ahrs2) MsgID() uint8 {
	return 178
}

func (self *Ahrs2) MsgName() string {
	return "Ahrs2"
}

func (self *Ahrs2) Pack(p *Packet) error {
	payload := make([]byte, 24)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Altitude))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.Lng))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Ahrs2) Unpack(p *Packet) error {
	if len(p.Payload) < 24 {
		return fmt.Errorf("payload too small")
	}
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Altitude = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Lng = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	return nil
}

// Camera Event
type CameraStatus struct {
	TimeUsec     uint64  // Image timestamp (microseconds since UNIX epoch, according to camera clock)
	P1           float32 // Parameter 1 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	P2           float32 // Parameter 2 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	P3           float32 // Parameter 3 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	P4           float32 // Parameter 4 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	ImgIdx       uint16  // Image index
	TargetSystem uint8   // System ID
	CamIdx       uint8   // Camera ID
	EventId      uint8   // See CAMERA_STATUS_TYPES enum for definition of the bitmask
}

func (self *CameraStatus) MsgID() uint8 {
	return 179
}

func (self *CameraStatus) MsgName() string {
	return "CameraStatus"
}

func (self *CameraStatus) Pack(p *Packet) error {
	payload := make([]byte, 29)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.P1))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.P2))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.P3))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.P4))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.ImgIdx))
	payload[26] = byte(self.TargetSystem)
	payload[27] = byte(self.CamIdx)
	payload[28] = byte(self.EventId)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *CameraStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 29 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.P1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.P2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.P3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.P4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.ImgIdx = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.TargetSystem = uint8(p.Payload[26])
	self.CamIdx = uint8(p.Payload[27])
	self.EventId = uint8(p.Payload[28])
	return nil
}

// Camera Capture Feedback
type CameraFeedback struct {
	TimeUsec     uint64  // Image timestamp (microseconds since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB)
	Lat          int32   // Latitude in (deg * 1E7)
	Lng          int32   // Longitude in (deg * 1E7)
	AltMsl       float32 // Altitude Absolute (meters AMSL)
	AltRel       float32 // Altitude Relative (meters above HOME location)
	Roll         float32 // Camera Roll angle (earth frame, degrees, +-180)
	Pitch        float32 // Camera Pitch angle (earth frame, degrees, +-180)
	Yaw          float32 // Camera Yaw (earth frame, degrees, 0-360, true)
	FocLen       float32 // Focal Length (mm)
	ImgIdx       uint16  // Image index
	TargetSystem uint8   // System ID
	CamIdx       uint8   // Camera ID
	Flags        uint8   // See CAMERA_FEEDBACK_FLAGS enum for definition of the bitmask
}

func (self *CameraFeedback) MsgID() uint8 {
	return 180
}

func (self *CameraFeedback) MsgName() string {
	return "CameraFeedback"
}

func (self *CameraFeedback) Pack(p *Packet) error {
	payload := make([]byte, 45)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.Lng))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.AltMsl))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.AltRel))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.FocLen))
	binary.LittleEndian.PutUint16(payload[40:], uint16(self.ImgIdx))
	payload[42] = byte(self.TargetSystem)
	payload[43] = byte(self.CamIdx)
	payload[44] = byte(self.Flags)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *CameraFeedback) Unpack(p *Packet) error {
	if len(p.Payload) < 45 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Lng = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.AltMsl = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.AltRel = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.FocLen = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.ImgIdx = uint16(binary.LittleEndian.Uint16(p.Payload[40:]))
	self.TargetSystem = uint8(p.Payload[42])
	self.CamIdx = uint8(p.Payload[43])
	self.Flags = uint8(p.Payload[44])
	return nil
}

// 2nd Battery status
type Battery2 struct {
	Voltage        uint16 // voltage in millivolts
	CurrentBattery int16  // Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
}

func (self *Battery2) MsgID() uint8 {
	return 181
}

func (self *Battery2) MsgName() string {
	return "Battery2"
}

func (self *Battery2) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Voltage))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.CurrentBattery))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Battery2) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	self.Voltage = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.CurrentBattery = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	return nil
}

// Status of third AHRS filter if available. This is for ANU research group (Ali and Sean)
type Ahrs3 struct {
	Roll     float32 // Roll angle (rad)
	Pitch    float32 // Pitch angle (rad)
	Yaw      float32 // Yaw angle (rad)
	Altitude float32 // Altitude (MSL)
	Lat      int32   // Latitude in degrees * 1E7
	Lng      int32   // Longitude in degrees * 1E7
	V1       float32 // test variable1
	V2       float32 // test variable2
	V3       float32 // test variable3
	V4       float32 // test variable4
}

func (self *Ahrs3) MsgID() uint8 {
	return 182
}

func (self *Ahrs3) MsgName() string {
	return "Ahrs3"
}

func (self *Ahrs3) Pack(p *Packet) error {
	payload := make([]byte, 40)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Altitude))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.Lng))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.V1))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.V2))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.V3))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.V4))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Ahrs3) Unpack(p *Packet) error {
	if len(p.Payload) < 40 {
		return fmt.Errorf("payload too small")
	}
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Altitude = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Lng = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.V1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.V2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.V3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.V4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	return nil
}

// Request the autopilot version from the system/component.
type AutopilotVersionRequest struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *AutopilotVersionRequest) MsgID() uint8 {
	return 183
}

func (self *AutopilotVersionRequest) MsgName() string {
	return "AutopilotVersionRequest"
}

func (self *AutopilotVersionRequest) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *AutopilotVersionRequest) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	return nil
}

// Control vehicle LEDs
type LedControl struct {
	TargetSystem    uint8     // System ID
	TargetComponent uint8     // Component ID
	Instance        uint8     // Instance (LED instance to control or 255 for all LEDs)
	Pattern         uint8     // Pattern (see LED_PATTERN_ENUM)
	CustomLen       uint8     // Custom Byte Length
	CustomBytes     [24]uint8 // Custom Bytes
}

func (self *LedControl) MsgID() uint8 {
	return 186
}

func (self *LedControl) MsgName() string {
	return "LedControl"
}

func (self *LedControl) Pack(p *Packet) error {
	payload := make([]byte, 29)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.Instance)
	payload[3] = byte(self.Pattern)
	payload[4] = byte(self.CustomLen)
	copy(payload[5:], self.CustomBytes[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *LedControl) Unpack(p *Packet) error {
	if len(p.Payload) < 29 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.Instance = uint8(p.Payload[2])
	self.Pattern = uint8(p.Payload[3])
	self.CustomLen = uint8(p.Payload[4])
	copy(self.CustomBytes[:], p.Payload[5:29])
	return nil
}

// Reports progress of compass calibration.
type MagCalProgress struct {
	DirectionX     float32   // Body frame direction vector for display
	DirectionY     float32   // Body frame direction vector for display
	DirectionZ     float32   // Body frame direction vector for display
	CompassId      uint8     // Compass being calibrated
	CalMask        uint8     // Bitmask of compasses being calibrated
	CalStatus      uint8     // Status (see MAG_CAL_STATUS enum)
	Attempt        uint8     // Attempt number
	CompletionPct  uint8     // Completion percentage
	CompletionMask [10]uint8 // Bitmask of sphere sections (see http://en.wikipedia.org/wiki/Geodesic_grid)
}

func (self *MagCalProgress) MsgID() uint8 {
	return 191
}

func (self *MagCalProgress) MsgName() string {
	return "MagCalProgress"
}

func (self *MagCalProgress) Pack(p *Packet) error {
	payload := make([]byte, 27)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.DirectionX))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.DirectionY))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.DirectionZ))
	payload[12] = byte(self.CompassId)
	payload[13] = byte(self.CalMask)
	payload[14] = byte(self.CalStatus)
	payload[15] = byte(self.Attempt)
	payload[16] = byte(self.CompletionPct)
	copy(payload[17:], self.CompletionMask[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MagCalProgress) Unpack(p *Packet) error {
	if len(p.Payload) < 27 {
		return fmt.Errorf("payload too small")
	}
	self.DirectionX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.DirectionY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.DirectionZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.CompassId = uint8(p.Payload[12])
	self.CalMask = uint8(p.Payload[13])
	self.CalStatus = uint8(p.Payload[14])
	self.Attempt = uint8(p.Payload[15])
	self.CompletionPct = uint8(p.Payload[16])
	copy(self.CompletionMask[:], p.Payload[17:27])
	return nil
}

// Reports results of completed compass calibration. Sent until MAG_CAL_ACK received.
type MagCalReport struct {
	Fitness   float32 // RMS milligauss residuals
	OfsX      float32 // X offset
	OfsY      float32 // Y offset
	OfsZ      float32 // Z offset
	DiagX     float32 // X diagonal (matrix 11)
	DiagY     float32 // Y diagonal (matrix 22)
	DiagZ     float32 // Z diagonal (matrix 33)
	OffdiagX  float32 // X off-diagonal (matrix 12 and 21)
	OffdiagY  float32 // Y off-diagonal (matrix 13 and 31)
	OffdiagZ  float32 // Z off-diagonal (matrix 32 and 23)
	CompassId uint8   // Compass being calibrated
	CalMask   uint8   // Bitmask of compasses being calibrated
	CalStatus uint8   // Status (see MAG_CAL_STATUS enum)
	Autosaved uint8   // 0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters
}

func (self *MagCalReport) MsgID() uint8 {
	return 192
}

func (self *MagCalReport) MsgName() string {
	return "MagCalReport"
}

func (self *MagCalReport) Pack(p *Packet) error {
	payload := make([]byte, 44)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Fitness))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.OfsX))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.OfsY))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.OfsZ))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.DiagX))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.DiagY))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.DiagZ))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.OffdiagX))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.OffdiagY))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.OffdiagZ))
	payload[40] = byte(self.CompassId)
	payload[41] = byte(self.CalMask)
	payload[42] = byte(self.CalStatus)
	payload[43] = byte(self.Autosaved)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MagCalReport) Unpack(p *Packet) error {
	if len(p.Payload) < 44 {
		return fmt.Errorf("payload too small")
	}
	self.Fitness = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.OfsX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.OfsY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.OfsZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.DiagX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.DiagY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.DiagZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.OffdiagX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.OffdiagY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.OffdiagZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.CompassId = uint8(p.Payload[40])
	self.CalMask = uint8(p.Payload[41])
	self.CalStatus = uint8(p.Payload[42])
	self.Autosaved = uint8(p.Payload[43])
	return nil
}

// EKF Status message including flags and variances
type EkfStatusReport struct {
	VelocityVariance   float32 // Velocity variance
	PosHorizVariance   float32 // Horizontal Position variance
	PosVertVariance    float32 // Vertical Position variance
	CompassVariance    float32 // Compass variance
	TerrainAltVariance float32 // Terrain Altitude variance
	Flags              uint16  // Flags
}

func (self *EkfStatusReport) MsgID() uint8 {
	return 193
}

func (self *EkfStatusReport) MsgName() string {
	return "EkfStatusReport"
}

func (self *EkfStatusReport) Pack(p *Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.VelocityVariance))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.PosHorizVariance))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.PosVertVariance))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.CompassVariance))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.TerrainAltVariance))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.Flags))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *EkfStatusReport) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	self.VelocityVariance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.PosHorizVariance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.PosVertVariance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.CompassVariance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.TerrainAltVariance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Flags = uint16(binary.LittleEndian.Uint16(p.Payload[20:]))
	return nil
}

// PID tuning information
type PidTuning struct {
	Desired  float32 // desired rate (degrees/s)
	Achieved float32 // achieved rate (degrees/s)
	Ff       float32 // FF component
	P        float32 // P component
	I        float32 // I component
	D        float32 // D component
	Axis     uint8   // axis
}

func (self *PidTuning) MsgID() uint8 {
	return 194
}

func (self *PidTuning) MsgName() string {
	return "PidTuning"
}

func (self *PidTuning) Pack(p *Packet) error {
	payload := make([]byte, 25)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Desired))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Achieved))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Ff))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.P))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.I))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.D))
	payload[24] = byte(self.Axis)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *PidTuning) Unpack(p *Packet) error {
	if len(p.Payload) < 25 {
		return fmt.Errorf("payload too small")
	}
	self.Desired = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Achieved = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Ff = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.P = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.I = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.D = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Axis = uint8(p.Payload[24])
	return nil
}

// 3 axis gimbal measurements
type GimbalReport struct {
	DeltaTime       float32 // Time since last update (seconds)
	DeltaAngleX     float32 // Delta angle X (radians)
	DeltaAngleY     float32 // Delta angle Y (radians)
	DeltaAngleZ     float32 // Delta angle X (radians)
	DeltaVelocityX  float32 // Delta velocity X (m/s)
	DeltaVelocityY  float32 // Delta velocity Y (m/s)
	DeltaVelocityZ  float32 // Delta velocity Z (m/s)
	JointRoll       float32 //  Joint ROLL (radians)
	JointEl         float32 //  Joint EL (radians)
	JointAz         float32 //  Joint AZ (radians)
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
}

func (self *GimbalReport) MsgID() uint8 {
	return 200
}

func (self *GimbalReport) MsgName() string {
	return "GimbalReport"
}

func (self *GimbalReport) Pack(p *Packet) error {
	payload := make([]byte, 42)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.DeltaTime))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.DeltaAngleX))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.DeltaAngleY))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.DeltaAngleZ))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.DeltaVelocityX))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.DeltaVelocityY))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.DeltaVelocityZ))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.JointRoll))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.JointEl))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.JointAz))
	payload[40] = byte(self.TargetSystem)
	payload[41] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GimbalReport) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}
	self.DeltaTime = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.DeltaAngleX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.DeltaAngleY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.DeltaAngleZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.DeltaVelocityX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.DeltaVelocityY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.DeltaVelocityZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.JointRoll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.JointEl = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.JointAz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.TargetSystem = uint8(p.Payload[40])
	self.TargetComponent = uint8(p.Payload[41])
	return nil
}

// Control message for rate gimbal
type GimbalControl struct {
	DemandedRateX   float32 // Demanded angular rate X (rad/s)
	DemandedRateY   float32 // Demanded angular rate Y (rad/s)
	DemandedRateZ   float32 // Demanded angular rate Z (rad/s)
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
}

func (self *GimbalControl) MsgID() uint8 {
	return 201
}

func (self *GimbalControl) MsgName() string {
	return "GimbalControl"
}

func (self *GimbalControl) Pack(p *Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.DemandedRateX))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.DemandedRateY))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.DemandedRateZ))
	payload[12] = byte(self.TargetSystem)
	payload[13] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GimbalControl) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	self.DemandedRateX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.DemandedRateY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.DemandedRateZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.TargetSystem = uint8(p.Payload[12])
	self.TargetComponent = uint8(p.Payload[13])
	return nil
}

//
//             Causes the gimbal to reset and boot as if it was just powered on
//
type GimbalReset struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *GimbalReset) MsgID() uint8 {
	return 202
}

func (self *GimbalReset) MsgName() string {
	return "GimbalReset"
}

func (self *GimbalReset) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GimbalReset) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	return nil
}

//
//             Reports progress and success or failure of gimbal axis calibration procedure
//
type GimbalAxisCalibrationProgress struct {
	CalibrationAxis     uint8 // Which gimbal axis we're reporting calibration progress for
	CalibrationProgress uint8 // The current calibration progress for this axis, 0x64=100%
	CalibrationStatus   uint8 // The status of the running calibration
}

func (self *GimbalAxisCalibrationProgress) MsgID() uint8 {
	return 203
}

func (self *GimbalAxisCalibrationProgress) MsgName() string {
	return "GimbalAxisCalibrationProgress"
}

func (self *GimbalAxisCalibrationProgress) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(self.CalibrationAxis)
	payload[1] = byte(self.CalibrationProgress)
	payload[2] = byte(self.CalibrationStatus)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GimbalAxisCalibrationProgress) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.CalibrationAxis = uint8(p.Payload[0])
	self.CalibrationProgress = uint8(p.Payload[1])
	self.CalibrationStatus = uint8(p.Payload[2])
	return nil
}

//
//             Instructs the gimbal to set its current position as its new home position.  Will primarily be used for factory calibration
//
type GimbalSetHomeOffsets struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *GimbalSetHomeOffsets) MsgID() uint8 {
	return 204
}

func (self *GimbalSetHomeOffsets) MsgName() string {
	return "GimbalSetHomeOffsets"
}

func (self *GimbalSetHomeOffsets) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GimbalSetHomeOffsets) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	return nil
}

//
//             Sent by the gimbal after it receives a SET_HOME_OFFSETS message to indicate the result of the home offset calibration
//
type GimbalHomeOffsetCalibrationResult struct {
	CalibrationResult uint8 // The result of the home offset calibration
}

func (self *GimbalHomeOffsetCalibrationResult) MsgID() uint8 {
	return 205
}

func (self *GimbalHomeOffsetCalibrationResult) MsgName() string {
	return "GimbalHomeOffsetCalibrationResult"
}

func (self *GimbalHomeOffsetCalibrationResult) Pack(p *Packet) error {
	payload := make([]byte, 1)
	payload[0] = byte(self.CalibrationResult)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GimbalHomeOffsetCalibrationResult) Unpack(p *Packet) error {
	if len(p.Payload) < 1 {
		return fmt.Errorf("payload too small")
	}
	self.CalibrationResult = uint8(p.Payload[0])
	return nil
}

//
//             Set factory configuration parameters (such as assembly date and time, and serial number).  This is only intended to be used
//             during manufacture, not by end users, so it is protected by a simple checksum of sorts (this won't stop anybody determined,
//             it's mostly just to keep the average user from trying to modify these values.  This will need to be revisited if that isn't
//             adequate.
//
type GimbalSetFactoryParameters struct {
	Magic1          uint32 // Magic number 1 for validation
	Magic2          uint32 // Magic number 2 for validation
	Magic3          uint32 // Magic number 3 for validation
	SerialNumberPt1 uint32 // Unit Serial Number Part 1 (part code, design, language/country)
	SerialNumberPt2 uint32 // Unit Serial Number Part 2 (option, year, month)
	SerialNumberPt3 uint32 // Unit Serial Number Part 3 (incrementing serial number per month)
	AssemblyYear    uint16 // Assembly Date Year
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
	AssemblyMonth   uint8  // Assembly Date Month
	AssemblyDay     uint8  // Assembly Date Day
	AssemblyHour    uint8  // Assembly Time Hour
	AssemblyMinute  uint8  // Assembly Time Minute
	AssemblySecond  uint8  // Assembly Time Second
}

func (self *GimbalSetFactoryParameters) MsgID() uint8 {
	return 206
}

func (self *GimbalSetFactoryParameters) MsgName() string {
	return "GimbalSetFactoryParameters"
}

func (self *GimbalSetFactoryParameters) Pack(p *Packet) error {
	payload := make([]byte, 33)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Magic1))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Magic2))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Magic3))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.SerialNumberPt1))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.SerialNumberPt2))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.SerialNumberPt3))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.AssemblyYear))
	payload[26] = byte(self.TargetSystem)
	payload[27] = byte(self.TargetComponent)
	payload[28] = byte(self.AssemblyMonth)
	payload[29] = byte(self.AssemblyDay)
	payload[30] = byte(self.AssemblyHour)
	payload[31] = byte(self.AssemblyMinute)
	payload[32] = byte(self.AssemblySecond)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GimbalSetFactoryParameters) Unpack(p *Packet) error {
	if len(p.Payload) < 33 {
		return fmt.Errorf("payload too small")
	}
	self.Magic1 = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Magic2 = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Magic3 = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.SerialNumberPt1 = uint32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.SerialNumberPt2 = uint32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.SerialNumberPt3 = uint32(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.AssemblyYear = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.TargetSystem = uint8(p.Payload[26])
	self.TargetComponent = uint8(p.Payload[27])
	self.AssemblyMonth = uint8(p.Payload[28])
	self.AssemblyDay = uint8(p.Payload[29])
	self.AssemblyHour = uint8(p.Payload[30])
	self.AssemblyMinute = uint8(p.Payload[31])
	self.AssemblySecond = uint8(p.Payload[32])
	return nil
}

//
//             Sent by the gimbal after the factory parameters are successfully loaded, to inform the factory software that the load is complete
//
type GimbalFactoryParametersLoaded struct {
	Dummy uint8 // Dummy field because mavgen doesn't allow messages with no fields
}

func (self *GimbalFactoryParametersLoaded) MsgID() uint8 {
	return 207
}

func (self *GimbalFactoryParametersLoaded) MsgName() string {
	return "GimbalFactoryParametersLoaded"
}

func (self *GimbalFactoryParametersLoaded) Pack(p *Packet) error {
	payload := make([]byte, 1)
	payload[0] = byte(self.Dummy)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GimbalFactoryParametersLoaded) Unpack(p *Packet) error {
	if len(p.Payload) < 1 {
		return fmt.Errorf("payload too small")
	}
	self.Dummy = uint8(p.Payload[0])
	return nil
}

//
//             Commands the gimbal to erase its firmware image and flash configuration, leaving only the bootloader.  The gimbal will then reboot into the bootloader,
//             ready for the load of a new application firmware image.  Erasing the flash configuration will cause the gimbal to re-perform axis calibration when a
//             new firmware image is loaded, and will cause all tuning parameters to return to their factory defaults.  WARNING: sending this command will render a
//             gimbal inoperable until a new firmware image is loaded onto it.  For this reason, a particular "knock" value must be sent for the command to take effect.
//             Use this command at your own risk
//
type GimbalEraseFirmwareAndConfig struct {
	Knock           uint32 // Knock value to confirm this is a valid request
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

func (self *GimbalEraseFirmwareAndConfig) MsgID() uint8 {
	return 208
}

func (self *GimbalEraseFirmwareAndConfig) MsgName() string {
	return "GimbalEraseFirmwareAndConfig"
}

func (self *GimbalEraseFirmwareAndConfig) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Knock))
	payload[4] = byte(self.TargetSystem)
	payload[5] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GimbalEraseFirmwareAndConfig) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	self.Knock = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.TargetSystem = uint8(p.Payload[4])
	self.TargetComponent = uint8(p.Payload[5])
	return nil
}

//
//             Command the gimbal to perform a series of factory tests.  Should not be needed by end users
//
type GimbalPerformFactoryTests struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *GimbalPerformFactoryTests) MsgID() uint8 {
	return 209
}

func (self *GimbalPerformFactoryTests) MsgName() string {
	return "GimbalPerformFactoryTests"
}

func (self *GimbalPerformFactoryTests) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GimbalPerformFactoryTests) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	return nil
}

//
//             Reports the current status of a section of a running factory test
//
type GimbalReportFactoryTestsProgress struct {
	Test                uint8 // Which factory test is currently running
	TestSection         uint8 // Which section of the test is currently running.  The meaning of this is test-dependent
	TestSectionProgress uint8 // The progress of the current test section, 0x64=100%
	TestStatus          uint8 // The status of the currently executing test section.  The meaning of this is test and section-dependent
}

func (self *GimbalReportFactoryTestsProgress) MsgID() uint8 {
	return 210
}

func (self *GimbalReportFactoryTestsProgress) MsgName() string {
	return "GimbalReportFactoryTestsProgress"
}

func (self *GimbalReportFactoryTestsProgress) Pack(p *Packet) error {
	payload := make([]byte, 4)
	payload[0] = byte(self.Test)
	payload[1] = byte(self.TestSection)
	payload[2] = byte(self.TestSectionProgress)
	payload[3] = byte(self.TestStatus)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GimbalReportFactoryTestsProgress) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	self.Test = uint8(p.Payload[0])
	self.TestSection = uint8(p.Payload[1])
	self.TestSectionProgress = uint8(p.Payload[2])
	self.TestStatus = uint8(p.Payload[3])
	return nil
}

// Instruct a HeroBus attached GoPro to power on
type GoproPowerOn struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *GoproPowerOn) MsgID() uint8 {
	return 215
}

func (self *GoproPowerOn) MsgName() string {
	return "GoproPowerOn"
}

func (self *GoproPowerOn) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GoproPowerOn) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	return nil
}

// Instruct a HeroBus attached GoPro to power off
type GoproPowerOff struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *GoproPowerOff) MsgID() uint8 {
	return 216
}

func (self *GoproPowerOff) MsgName() string {
	return "GoproPowerOff"
}

func (self *GoproPowerOff) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GoproPowerOff) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	return nil
}

// Send a command to a HeroBus attached GoPro.  Will generate a GOPRO_RESPONSE message with results of the command
type GoproCommand struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	GpCmdName1      uint8 // First character of the 2 character GoPro command
	GpCmdName2      uint8 // Second character of the 2 character GoPro command
	GpCmdParm       uint8 // Parameter for the command
}

func (self *GoproCommand) MsgID() uint8 {
	return 217
}

func (self *GoproCommand) MsgName() string {
	return "GoproCommand"
}

func (self *GoproCommand) Pack(p *Packet) error {
	payload := make([]byte, 5)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.GpCmdName1)
	payload[3] = byte(self.GpCmdName2)
	payload[4] = byte(self.GpCmdParm)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GoproCommand) Unpack(p *Packet) error {
	if len(p.Payload) < 5 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.GpCmdName1 = uint8(p.Payload[2])
	self.GpCmdName2 = uint8(p.Payload[3])
	self.GpCmdParm = uint8(p.Payload[4])
	return nil
}

//
//             Response to a command sent to a HeroBus attached GoPro with a GOPRO_COMMAND message.  Contains response from the camera as well as information about any errors encountered while attempting to communicate with the camera
//
type GoproResponse struct {
	GpCmdResult           uint16 // Result of the command attempt to the GoPro, as defined by GOPRO_CMD_RESULT enum.
	GpCmdName1            uint8  // First character of the 2 character GoPro command that generated this response
	GpCmdName2            uint8  // Second character of the 2 character GoPro command that generated this response
	GpCmdResponseStatus   uint8  // Response byte from the GoPro's response to the command.  0 = Success, 1 = Failure
	GpCmdResponseArgument uint8  // Response argument from the GoPro's response to the command
}

func (self *GoproResponse) MsgID() uint8 {
	return 218
}

func (self *GoproResponse) MsgName() string {
	return "GoproResponse"
}

func (self *GoproResponse) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.GpCmdResult))
	payload[2] = byte(self.GpCmdName1)
	payload[3] = byte(self.GpCmdName2)
	payload[4] = byte(self.GpCmdResponseStatus)
	payload[5] = byte(self.GpCmdResponseArgument)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GoproResponse) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	self.GpCmdResult = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.GpCmdName1 = uint8(p.Payload[2])
	self.GpCmdName2 = uint8(p.Payload[3])
	self.GpCmdResponseStatus = uint8(p.Payload[4])
	self.GpCmdResponseArgument = uint8(p.Payload[5])
	return nil
}

// RPM sensor output
type Rpm struct {
	Rpm1 float32 // RPM Sensor1
	Rpm2 float32 // RPM Sensor2
}

func (self *Rpm) MsgID() uint8 {
	return 226
}

func (self *Rpm) MsgName() string {
	return "Rpm"
}

func (self *Rpm) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Rpm1))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Rpm2))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Rpm) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	self.Rpm1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Rpm2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	return nil
}

// Message IDs
const (
	MSG_ID_SENSOR_OFFSETS                        = 150
	MSG_ID_SET_MAG_OFFSETS                       = 151
	MSG_ID_MEMINFO                               = 152
	MSG_ID_AP_ADC                                = 153
	MSG_ID_DIGICAM_CONFIGURE                     = 154
	MSG_ID_DIGICAM_CONTROL                       = 155
	MSG_ID_MOUNT_CONFIGURE                       = 156
	MSG_ID_MOUNT_CONTROL                         = 157
	MSG_ID_MOUNT_STATUS                          = 158
	MSG_ID_FENCE_POINT                           = 160
	MSG_ID_FENCE_FETCH_POINT                     = 161
	MSG_ID_FENCE_STATUS                          = 162
	MSG_ID_AHRS                                  = 163
	MSG_ID_SIMSTATE                              = 164
	MSG_ID_HWSTATUS                              = 165
	MSG_ID_RADIO                                 = 166
	MSG_ID_LIMITS_STATUS                         = 167
	MSG_ID_WIND                                  = 168
	MSG_ID_DATA16                                = 169
	MSG_ID_DATA32                                = 170
	MSG_ID_DATA64                                = 171
	MSG_ID_DATA96                                = 172
	MSG_ID_RANGEFINDER                           = 173
	MSG_ID_AIRSPEED_AUTOCAL                      = 174
	MSG_ID_RALLY_POINT                           = 175
	MSG_ID_RALLY_FETCH_POINT                     = 176
	MSG_ID_COMPASSMOT_STATUS                     = 177
	MSG_ID_AHRS2                                 = 178
	MSG_ID_CAMERA_STATUS                         = 179
	MSG_ID_CAMERA_FEEDBACK                       = 180
	MSG_ID_BATTERY2                              = 181
	MSG_ID_AHRS3                                 = 182
	MSG_ID_AUTOPILOT_VERSION_REQUEST             = 183
	MSG_ID_LED_CONTROL                           = 186
	MSG_ID_MAG_CAL_PROGRESS                      = 191
	MSG_ID_MAG_CAL_REPORT                        = 192
	MSG_ID_EKF_STATUS_REPORT                     = 193
	MSG_ID_PID_TUNING                            = 194
	MSG_ID_GIMBAL_REPORT                         = 200
	MSG_ID_GIMBAL_CONTROL                        = 201
	MSG_ID_GIMBAL_RESET                          = 202
	MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS      = 203
	MSG_ID_GIMBAL_SET_HOME_OFFSETS               = 204
	MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT = 205
	MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS         = 206
	MSG_ID_GIMBAL_FACTORY_PARAMETERS_LOADED      = 207
	MSG_ID_GIMBAL_ERASE_FIRMWARE_AND_CONFIG      = 208
	MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS          = 209
	MSG_ID_GIMBAL_REPORT_FACTORY_TESTS_PROGRESS  = 210
	MSG_ID_GOPRO_POWER_ON                        = 215
	MSG_ID_GOPRO_POWER_OFF                       = 216
	MSG_ID_GOPRO_COMMAND                         = 217
	MSG_ID_GOPRO_RESPONSE                        = 218
	MSG_ID_RPM                                   = 226
)

// DialectArdupilotmega is the dialect represented by ardupilotmega.xml
var DialectArdupilotmega *Dialect = &Dialect{
	Name: "ardupilotmega",
	crcExtras: map[uint8]uint8{
		150: 134, // MSG_ID_SENSOR_OFFSETS
		151: 219, // MSG_ID_SET_MAG_OFFSETS
		152: 208, // MSG_ID_MEMINFO
		153: 188, // MSG_ID_AP_ADC
		154: 84,  // MSG_ID_DIGICAM_CONFIGURE
		155: 22,  // MSG_ID_DIGICAM_CONTROL
		156: 19,  // MSG_ID_MOUNT_CONFIGURE
		157: 21,  // MSG_ID_MOUNT_CONTROL
		158: 134, // MSG_ID_MOUNT_STATUS
		160: 78,  // MSG_ID_FENCE_POINT
		161: 68,  // MSG_ID_FENCE_FETCH_POINT
		162: 189, // MSG_ID_FENCE_STATUS
		163: 127, // MSG_ID_AHRS
		164: 154, // MSG_ID_SIMSTATE
		165: 21,  // MSG_ID_HWSTATUS
		166: 21,  // MSG_ID_RADIO
		167: 144, // MSG_ID_LIMITS_STATUS
		168: 1,   // MSG_ID_WIND
		169: 234, // MSG_ID_DATA16
		170: 73,  // MSG_ID_DATA32
		171: 181, // MSG_ID_DATA64
		172: 22,  // MSG_ID_DATA96
		173: 83,  // MSG_ID_RANGEFINDER
		174: 167, // MSG_ID_AIRSPEED_AUTOCAL
		175: 138, // MSG_ID_RALLY_POINT
		176: 234, // MSG_ID_RALLY_FETCH_POINT
		177: 240, // MSG_ID_COMPASSMOT_STATUS
		178: 47,  // MSG_ID_AHRS2
		179: 189, // MSG_ID_CAMERA_STATUS
		180: 52,  // MSG_ID_CAMERA_FEEDBACK
		181: 174, // MSG_ID_BATTERY2
		182: 229, // MSG_ID_AHRS3
		183: 85,  // MSG_ID_AUTOPILOT_VERSION_REQUEST
		186: 72,  // MSG_ID_LED_CONTROL
		191: 92,  // MSG_ID_MAG_CAL_PROGRESS
		192: 36,  // MSG_ID_MAG_CAL_REPORT
		193: 71,  // MSG_ID_EKF_STATUS_REPORT
		194: 98,  // MSG_ID_PID_TUNING
		200: 134, // MSG_ID_GIMBAL_REPORT
		201: 205, // MSG_ID_GIMBAL_CONTROL
		202: 94,  // MSG_ID_GIMBAL_RESET
		203: 128, // MSG_ID_GIMBAL_AXIS_CALIBRATION_PROGRESS
		204: 54,  // MSG_ID_GIMBAL_SET_HOME_OFFSETS
		205: 63,  // MSG_ID_GIMBAL_HOME_OFFSET_CALIBRATION_RESULT
		206: 112, // MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS
		207: 201, // MSG_ID_GIMBAL_FACTORY_PARAMETERS_LOADED
		208: 221, // MSG_ID_GIMBAL_ERASE_FIRMWARE_AND_CONFIG
		209: 226, // MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS
		210: 238, // MSG_ID_GIMBAL_REPORT_FACTORY_TESTS_PROGRESS
		215: 241, // MSG_ID_GOPRO_POWER_ON
		216: 155, // MSG_ID_GOPRO_POWER_OFF
		217: 43,  // MSG_ID_GOPRO_COMMAND
		218: 149, // MSG_ID_GOPRO_RESPONSE
		226: 207, // MSG_ID_RPM
	},
}
