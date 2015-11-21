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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.MagDeclination,
		&self.RawPress,
		&self.RawTemp,
		&self.GyroCalX,
		&self.GyroCalY,
		&self.GyroCalZ,
		&self.AccelCalX,
		&self.AccelCalY,
		&self.AccelCalZ,
		&self.MagOfsX,
		&self.MagOfsY,
		&self.MagOfsZ,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SensorOffsets) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.MagDeclination,
		&self.RawPress,
		&self.RawTemp,
		&self.GyroCalX,
		&self.GyroCalY,
		&self.GyroCalZ,
		&self.AccelCalX,
		&self.AccelCalY,
		&self.AccelCalZ,
		&self.MagOfsX,
		&self.MagOfsY,
		&self.MagOfsZ,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.MagOfsX,
		&self.MagOfsY,
		&self.MagOfsZ,
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

func (self *SetMagOffsets) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.MagOfsX,
		&self.MagOfsY,
		&self.MagOfsZ,
		&self.TargetSystem,
		&self.TargetComponent,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Brkval,
		&self.Freemem,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Meminfo) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Brkval,
		&self.Freemem,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Adc1,
		&self.Adc2,
		&self.Adc3,
		&self.Adc4,
		&self.Adc5,
		&self.Adc6,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ApAdc) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Adc1,
		&self.Adc2,
		&self.Adc3,
		&self.Adc4,
		&self.Adc5,
		&self.Adc6,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.ExtraValue,
		&self.ShutterSpeed,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Mode,
		&self.Aperture,
		&self.Iso,
		&self.ExposureType,
		&self.CommandId,
		&self.EngineCutOff,
		&self.ExtraParam,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *DigicamConfigure) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.ExtraValue,
		&self.ShutterSpeed,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Mode,
		&self.Aperture,
		&self.Iso,
		&self.ExposureType,
		&self.CommandId,
		&self.EngineCutOff,
		&self.ExtraParam,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.ExtraValue,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Session,
		&self.ZoomPos,
		&self.ZoomStep,
		&self.FocusLock,
		&self.Shot,
		&self.CommandId,
		&self.ExtraParam,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *DigicamControl) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.ExtraValue,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Session,
		&self.ZoomPos,
		&self.ZoomStep,
		&self.FocusLock,
		&self.Shot,
		&self.CommandId,
		&self.ExtraParam,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.MountMode,
		&self.StabRoll,
		&self.StabPitch,
		&self.StabYaw,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *MountConfigure) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.MountMode,
		&self.StabRoll,
		&self.StabPitch,
		&self.StabYaw,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.InputA,
		&self.InputB,
		&self.InputC,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.SavePosition,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *MountControl) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.InputA,
		&self.InputB,
		&self.InputC,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.SavePosition,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.PointingA,
		&self.PointingB,
		&self.PointingC,
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

func (self *MountStatus) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.PointingA,
		&self.PointingB,
		&self.PointingC,
		&self.TargetSystem,
		&self.TargetComponent,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Lat,
		&self.Lng,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Idx,
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

func (self *FencePoint) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Lat,
		&self.Lng,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Idx,
		&self.Count,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Idx,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *FenceFetchPoint) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Idx,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.BreachTime,
		&self.BreachCount,
		&self.BreachStatus,
		&self.BreachType,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *FenceStatus) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.BreachTime,
		&self.BreachCount,
		&self.BreachStatus,
		&self.BreachType,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Omegaix,
		&self.Omegaiy,
		&self.Omegaiz,
		&self.AccelWeight,
		&self.RenormVal,
		&self.ErrorRp,
		&self.ErrorYaw,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Ahrs) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Omegaix,
		&self.Omegaiy,
		&self.Omegaiz,
		&self.AccelWeight,
		&self.RenormVal,
		&self.ErrorRp,
		&self.ErrorYaw,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
		&self.Xgyro,
		&self.Ygyro,
		&self.Zgyro,
		&self.Lat,
		&self.Lng,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Simstate) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
		&self.Xgyro,
		&self.Ygyro,
		&self.Zgyro,
		&self.Lat,
		&self.Lng,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Vcc,
		&self.I2cerr,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Hwstatus) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Vcc,
		&self.I2cerr,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Rxerrors,
		&self.Fixed,
		&self.Rssi,
		&self.Remrssi,
		&self.Txbuf,
		&self.Noise,
		&self.Remnoise,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Radio) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Rxerrors,
		&self.Fixed,
		&self.Rssi,
		&self.Remrssi,
		&self.Txbuf,
		&self.Noise,
		&self.Remnoise,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.LastTrigger,
		&self.LastAction,
		&self.LastRecovery,
		&self.LastClear,
		&self.BreachCount,
		&self.LimitsState,
		&self.ModsEnabled,
		&self.ModsRequired,
		&self.ModsTriggered,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *LimitsStatus) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.LastTrigger,
		&self.LastAction,
		&self.LastRecovery,
		&self.LastClear,
		&self.BreachCount,
		&self.LimitsState,
		&self.ModsEnabled,
		&self.ModsRequired,
		&self.ModsTriggered,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Direction,
		&self.Speed,
		&self.SpeedZ,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Wind) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Direction,
		&self.Speed,
		&self.SpeedZ,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Type,
		&self.Len,
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

func (self *Data16) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Type,
		&self.Len,
		&self.Data,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Type,
		&self.Len,
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

func (self *Data32) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Type,
		&self.Len,
		&self.Data,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Type,
		&self.Len,
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

func (self *Data64) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Type,
		&self.Len,
		&self.Data,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Type,
		&self.Len,
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

func (self *Data96) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Type,
		&self.Len,
		&self.Data,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Distance,
		&self.Voltage,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Rangefinder) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Distance,
		&self.Voltage,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.DiffPressure,
		&self.Eas2tas,
		&self.Ratio,
		&self.StateX,
		&self.StateY,
		&self.StateZ,
		&self.Pax,
		&self.Pby,
		&self.Pcz,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *AirspeedAutocal) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.DiffPressure,
		&self.Eas2tas,
		&self.Ratio,
		&self.StateX,
		&self.StateY,
		&self.StateZ,
		&self.Pax,
		&self.Pby,
		&self.Pcz,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Lat,
		&self.Lng,
		&self.Alt,
		&self.BreakAlt,
		&self.LandDir,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Idx,
		&self.Count,
		&self.Flags,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *RallyPoint) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Lat,
		&self.Lng,
		&self.Alt,
		&self.BreakAlt,
		&self.LandDir,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Idx,
		&self.Count,
		&self.Flags,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Idx,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *RallyFetchPoint) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Idx,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Current,
		&self.Compensationx,
		&self.Compensationy,
		&self.Compensationz,
		&self.Throttle,
		&self.Interference,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *CompassmotStatus) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Current,
		&self.Compensationx,
		&self.Compensationy,
		&self.Compensationz,
		&self.Throttle,
		&self.Interference,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.Altitude,
		&self.Lat,
		&self.Lng,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Ahrs2) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.Altitude,
		&self.Lat,
		&self.Lng,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.P1,
		&self.P2,
		&self.P3,
		&self.P4,
		&self.ImgIdx,
		&self.TargetSystem,
		&self.CamIdx,
		&self.EventId,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *CameraStatus) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.P1,
		&self.P2,
		&self.P3,
		&self.P4,
		&self.ImgIdx,
		&self.TargetSystem,
		&self.CamIdx,
		&self.EventId,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Lat,
		&self.Lng,
		&self.AltMsl,
		&self.AltRel,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.FocLen,
		&self.ImgIdx,
		&self.TargetSystem,
		&self.CamIdx,
		&self.Flags,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *CameraFeedback) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Lat,
		&self.Lng,
		&self.AltMsl,
		&self.AltRel,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.FocLen,
		&self.ImgIdx,
		&self.TargetSystem,
		&self.CamIdx,
		&self.Flags,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Voltage,
		&self.CurrentBattery,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Battery2) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Voltage,
		&self.CurrentBattery,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.Altitude,
		&self.Lat,
		&self.Lng,
		&self.V1,
		&self.V2,
		&self.V3,
		&self.V4,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Ahrs3) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.Altitude,
		&self.Lat,
		&self.Lng,
		&self.V1,
		&self.V2,
		&self.V3,
		&self.V4,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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

func (self *AutopilotVersionRequest) Unpack(p *Packet) error {
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Instance,
		&self.Pattern,
		&self.CustomLen,
		&self.CustomBytes,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *LedControl) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Instance,
		&self.Pattern,
		&self.CustomLen,
		&self.CustomBytes,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.DirectionX,
		&self.DirectionY,
		&self.DirectionZ,
		&self.CompassId,
		&self.CalMask,
		&self.CalStatus,
		&self.Attempt,
		&self.CompletionPct,
		&self.CompletionMask,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *MagCalProgress) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.DirectionX,
		&self.DirectionY,
		&self.DirectionZ,
		&self.CompassId,
		&self.CalMask,
		&self.CalStatus,
		&self.Attempt,
		&self.CompletionPct,
		&self.CompletionMask,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Fitness,
		&self.OfsX,
		&self.OfsY,
		&self.OfsZ,
		&self.DiagX,
		&self.DiagY,
		&self.DiagZ,
		&self.OffdiagX,
		&self.OffdiagY,
		&self.OffdiagZ,
		&self.CompassId,
		&self.CalMask,
		&self.CalStatus,
		&self.Autosaved,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *MagCalReport) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Fitness,
		&self.OfsX,
		&self.OfsY,
		&self.OfsZ,
		&self.DiagX,
		&self.DiagY,
		&self.DiagZ,
		&self.OffdiagX,
		&self.OffdiagY,
		&self.OffdiagZ,
		&self.CompassId,
		&self.CalMask,
		&self.CalStatus,
		&self.Autosaved,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.VelocityVariance,
		&self.PosHorizVariance,
		&self.PosVertVariance,
		&self.CompassVariance,
		&self.TerrainAltVariance,
		&self.Flags,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *EkfStatusReport) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.VelocityVariance,
		&self.PosHorizVariance,
		&self.PosVertVariance,
		&self.CompassVariance,
		&self.TerrainAltVariance,
		&self.Flags,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Desired,
		&self.Achieved,
		&self.Ff,
		&self.P,
		&self.I,
		&self.D,
		&self.Axis,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *PidTuning) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Desired,
		&self.Achieved,
		&self.Ff,
		&self.P,
		&self.I,
		&self.D,
		&self.Axis,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.DeltaTime,
		&self.DeltaAngleX,
		&self.DeltaAngleY,
		&self.DeltaAngleZ,
		&self.DeltaVelocityX,
		&self.DeltaVelocityY,
		&self.DeltaVelocityZ,
		&self.JointRoll,
		&self.JointEl,
		&self.JointAz,
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

func (self *GimbalReport) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.DeltaTime,
		&self.DeltaAngleX,
		&self.DeltaAngleY,
		&self.DeltaAngleZ,
		&self.DeltaVelocityX,
		&self.DeltaVelocityY,
		&self.DeltaVelocityZ,
		&self.JointRoll,
		&self.JointEl,
		&self.JointAz,
		&self.TargetSystem,
		&self.TargetComponent,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.DemandedRateX,
		&self.DemandedRateY,
		&self.DemandedRateZ,
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

func (self *GimbalControl) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.DemandedRateX,
		&self.DemandedRateY,
		&self.DemandedRateZ,
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

func (self *GimbalReset) Unpack(p *Packet) error {
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.CalibrationAxis,
		&self.CalibrationProgress,
		&self.CalibrationStatus,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *GimbalAxisCalibrationProgress) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.CalibrationAxis,
		&self.CalibrationProgress,
		&self.CalibrationStatus,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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

func (self *GimbalSetHomeOffsets) Unpack(p *Packet) error {
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.CalibrationResult,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *GimbalHomeOffsetCalibrationResult) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.CalibrationResult,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Magic1,
		&self.Magic2,
		&self.Magic3,
		&self.SerialNumberPt1,
		&self.SerialNumberPt2,
		&self.SerialNumberPt3,
		&self.AssemblyYear,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.AssemblyMonth,
		&self.AssemblyDay,
		&self.AssemblyHour,
		&self.AssemblyMinute,
		&self.AssemblySecond,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *GimbalSetFactoryParameters) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Magic1,
		&self.Magic2,
		&self.Magic3,
		&self.SerialNumberPt1,
		&self.SerialNumberPt2,
		&self.SerialNumberPt3,
		&self.AssemblyYear,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.AssemblyMonth,
		&self.AssemblyDay,
		&self.AssemblyHour,
		&self.AssemblyMinute,
		&self.AssemblySecond,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Dummy,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *GimbalFactoryParametersLoaded) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Dummy,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Knock,
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

func (self *GimbalEraseFirmwareAndConfig) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Knock,
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

func (self *GimbalPerformFactoryTests) Unpack(p *Packet) error {
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Test,
		&self.TestSection,
		&self.TestSectionProgress,
		&self.TestStatus,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *GimbalReportFactoryTestsProgress) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Test,
		&self.TestSection,
		&self.TestSectionProgress,
		&self.TestStatus,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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

func (self *GoproPowerOn) Unpack(p *Packet) error {
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

func (self *GoproPowerOff) Unpack(p *Packet) error {
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.GpCmdName1,
		&self.GpCmdName2,
		&self.GpCmdParm,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *GoproCommand) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.GpCmdName1,
		&self.GpCmdName2,
		&self.GpCmdParm,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.GpCmdResult,
		&self.GpCmdName1,
		&self.GpCmdName2,
		&self.GpCmdResponseStatus,
		&self.GpCmdResponseArgument,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *GoproResponse) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.GpCmdResult,
		&self.GpCmdName1,
		&self.GpCmdName2,
		&self.GpCmdResponseStatus,
		&self.GpCmdResponseArgument,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Rpm1,
		&self.Rpm2,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Rpm) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Rpm1,
		&self.Rpm2,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
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
