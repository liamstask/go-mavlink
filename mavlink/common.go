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

// MavAutopilot: Micro air vehicle / autopilot classes. This identifies the individual model.
const (
	MAV_AUTOPILOT_GENERIC                                      = 0  // Generic autopilot, full support for everything
	MAV_AUTOPILOT_PIXHAWK                                      = 1  // PIXHAWK autopilot, http://pixhawk.ethz.ch
	MAV_AUTOPILOT_SLUGS                                        = 2  // SLUGS autopilot, http://slugsuav.soe.ucsc.edu
	MAV_AUTOPILOT_ARDUPILOTMEGA                                = 3  // ArduPilotMega / ArduCopter, http://diydrones.com
	MAV_AUTOPILOT_OPENPILOT                                    = 4  // OpenPilot, http://openpilot.org
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY                       = 5  // Generic autopilot only supporting simple waypoints
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6  // Generic autopilot supporting waypoints and other simple navigation commands
	MAV_AUTOPILOT_GENERIC_MISSION_FULL                         = 7  // Generic autopilot supporting the full mission command set
	MAV_AUTOPILOT_INVALID                                      = 8  // No valid autopilot, e.g. a GCS or other MAVLink component
	MAV_AUTOPILOT_PPZ                                          = 9  // PPZ UAV - http://nongnu.org/paparazzi
	MAV_AUTOPILOT_UDB                                          = 10 // UAV Dev Board
	MAV_AUTOPILOT_FP                                           = 11 // FlexiPilot
	MAV_AUTOPILOT_PX4                                          = 12 // PX4 Autopilot - http://pixhawk.ethz.ch/px4/
	MAV_AUTOPILOT_SMACCMPILOT                                  = 13 // SMACCMPilot - http://smaccmpilot.org
	MAV_AUTOPILOT_AUTOQUAD                                     = 14 // AutoQuad -- http://autoquad.org
	MAV_AUTOPILOT_ARMAZILA                                     = 15 // Armazila -- http://armazila.com
	MAV_AUTOPILOT_AEROB                                        = 16 // Aerob -- http://aerob.ru
	MAV_AUTOPILOT_ASLUAV                                       = 17 // ASLUAV autopilot -- http://www.asl.ethz.ch
)

// MavType:
const (
	MAV_TYPE_GENERIC            = 0  // Generic micro air vehicle.
	MAV_TYPE_FIXED_WING         = 1  // Fixed wing aircraft.
	MAV_TYPE_QUADROTOR          = 2  // Quadrotor
	MAV_TYPE_COAXIAL            = 3  // Coaxial helicopter
	MAV_TYPE_HELICOPTER         = 4  // Normal helicopter with tail rotor.
	MAV_TYPE_ANTENNA_TRACKER    = 5  // Ground installation
	MAV_TYPE_GCS                = 6  // Operator control unit / ground control station
	MAV_TYPE_AIRSHIP            = 7  // Airship, controlled
	MAV_TYPE_FREE_BALLOON       = 8  // Free balloon, uncontrolled
	MAV_TYPE_ROCKET             = 9  // Rocket
	MAV_TYPE_GROUND_ROVER       = 10 // Ground rover
	MAV_TYPE_SURFACE_BOAT       = 11 // Surface vessel, boat, ship
	MAV_TYPE_SUBMARINE          = 12 // Submarine
	MAV_TYPE_HEXAROTOR          = 13 // Hexarotor
	MAV_TYPE_OCTOROTOR          = 14 // Octorotor
	MAV_TYPE_TRICOPTER          = 15 // Octorotor
	MAV_TYPE_FLAPPING_WING      = 16 // Flapping wing
	MAV_TYPE_KITE               = 17 // Flapping wing
	MAV_TYPE_ONBOARD_CONTROLLER = 18 // Onboard companion controller
	MAV_TYPE_VTOL_DUOROTOR      = 19 // Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
	MAV_TYPE_VTOL_QUADROTOR     = 20 // Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
	MAV_TYPE_VTOL_RESERVED1     = 21 // VTOL reserved 1
	MAV_TYPE_VTOL_RESERVED2     = 22 // VTOL reserved 2
	MAV_TYPE_VTOL_RESERVED3     = 23 // VTOL reserved 3
	MAV_TYPE_VTOL_RESERVED4     = 24 // VTOL reserved 4
	MAV_TYPE_VTOL_RESERVED5     = 25 // VTOL reserved 5
	MAV_TYPE_GIMBAL             = 26 // Onboard gimbal
)

// MavModeFlag: These flags encode the MAV mode.
const (
	MAV_MODE_FLAG_SAFETY_ARMED         = 128 // 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly.
	MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64  // 0b01000000 remote control input is enabled.
	MAV_MODE_FLAG_HIL_ENABLED          = 32  // 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational.
	MAV_MODE_FLAG_STABILIZE_ENABLED    = 16  // 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.
	MAV_MODE_FLAG_GUIDED_ENABLED       = 8   // 0b00001000 guided mode enabled, system flies MISSIONs / mission items.
	MAV_MODE_FLAG_AUTO_ENABLED         = 4   // 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.
	MAV_MODE_FLAG_TEST_ENABLED         = 2   // 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations.
	MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  = 1   // 0b00000001 Reserved for future use.
)

// MavModeFlagDecodePosition: These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not.
const (
	MAV_MODE_FLAG_DECODE_POSITION_SAFETY      = 128 // First bit:  10000000
	MAV_MODE_FLAG_DECODE_POSITION_MANUAL      = 64  // Second bit: 01000000
	MAV_MODE_FLAG_DECODE_POSITION_HIL         = 32  // Third bit:  00100000
	MAV_MODE_FLAG_DECODE_POSITION_STABILIZE   = 16  // Fourth bit: 00010000
	MAV_MODE_FLAG_DECODE_POSITION_GUIDED      = 8   // Fifth bit:  00001000
	MAV_MODE_FLAG_DECODE_POSITION_AUTO        = 4   // Sixt bit:   00000100
	MAV_MODE_FLAG_DECODE_POSITION_TEST        = 2   // Seventh bit: 00000010
	MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE = 1   // Eighth bit: 00000001
)

// MavGoto: Override command, pauses current mission execution and moves immediately to a position
const (
	MAV_GOTO_DO_HOLD                    = 0 // Hold at the current position.
	MAV_GOTO_DO_CONTINUE                = 1 // Continue with the next item in mission execution.
	MAV_GOTO_HOLD_AT_CURRENT_POSITION   = 2 // Hold at the current position of the system
	MAV_GOTO_HOLD_AT_SPECIFIED_POSITION = 3 // Hold at the position specified in the parameters of the DO_HOLD action
)

// MavMode: These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it                simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.
const (
	MAV_MODE_PREFLIGHT          = 0   // System is not ready to fly, booting, calibrating, etc. No flag is set.
	MAV_MODE_STABILIZE_DISARMED = 80  // System is allowed to be active, under assisted RC control.
	MAV_MODE_STABILIZE_ARMED    = 208 // System is allowed to be active, under assisted RC control.
	MAV_MODE_MANUAL_DISARMED    = 64  // System is allowed to be active, under manual (RC) control, no stabilization
	MAV_MODE_MANUAL_ARMED       = 192 // System is allowed to be active, under manual (RC) control, no stabilization
	MAV_MODE_GUIDED_DISARMED    = 88  // System is allowed to be active, under autonomous control, manual setpoint
	MAV_MODE_GUIDED_ARMED       = 216 // System is allowed to be active, under autonomous control, manual setpoint
	MAV_MODE_AUTO_DISARMED      = 92  // System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs)
	MAV_MODE_AUTO_ARMED         = 220 // System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs)
	MAV_MODE_TEST_DISARMED      = 66  // UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
	MAV_MODE_TEST_ARMED         = 194 // UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
)

// MavState:
const (
	MAV_STATE_UNINIT      = 0 // Uninitialized system, state is unknown.
	MAV_STATE_BOOT        = 1 // System is booting up.
	MAV_STATE_CALIBRATING = 2 // System is calibrating and not flight-ready.
	MAV_STATE_STANDBY     = 3 // System is grounded and on standby. It can be launched any time.
	MAV_STATE_ACTIVE      = 4 // System is active and might be already airborne. Motors are engaged.
	MAV_STATE_CRITICAL    = 5 // System is in a non-normal flight mode. It can however still navigate.
	MAV_STATE_EMERGENCY   = 6 // System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
	MAV_STATE_POWEROFF    = 7 // System just initialized its power-down sequence, will shut down now.
)

// MavComponent:
const (
	MAV_COMP_ID_ALL            = 0   //
	MAV_COMP_ID_GPS            = 220 //
	MAV_COMP_ID_MISSIONPLANNER = 190 //
	MAV_COMP_ID_PATHPLANNER    = 195 //
	MAV_COMP_ID_MAPPER         = 180 //
	MAV_COMP_ID_CAMERA         = 100 //
	MAV_COMP_ID_IMU            = 200 //
	MAV_COMP_ID_IMU_2          = 201 //
	MAV_COMP_ID_IMU_3          = 202 //
	MAV_COMP_ID_UDP_BRIDGE     = 240 //
	MAV_COMP_ID_UART_BRIDGE    = 241 //
	MAV_COMP_ID_SYSTEM_CONTROL = 250 //
	MAV_COMP_ID_SERVO1         = 140 //
	MAV_COMP_ID_SERVO2         = 141 //
	MAV_COMP_ID_SERVO3         = 142 //
	MAV_COMP_ID_SERVO4         = 143 //
	MAV_COMP_ID_SERVO5         = 144 //
	MAV_COMP_ID_SERVO6         = 145 //
	MAV_COMP_ID_SERVO7         = 146 //
	MAV_COMP_ID_SERVO8         = 147 //
	MAV_COMP_ID_SERVO9         = 148 //
	MAV_COMP_ID_SERVO10        = 149 //
	MAV_COMP_ID_SERVO11        = 150 //
	MAV_COMP_ID_SERVO12        = 151 //
	MAV_COMP_ID_SERVO13        = 152 //
	MAV_COMP_ID_SERVO14        = 153 //
	MAV_COMP_ID_GIMBAL         = 154 //
)

// MavSysStatusSensor: These encode the sensors whose status is sent as part of the SYS_STATUS message.
const (
	MAV_SYS_STATUS_SENSOR_3D_GYRO                = 1   // 0x01 3D gyro
	MAV_SYS_STATUS_SENSOR_3D_ACCEL               = 2   // 0x02 3D accelerometer
	MAV_SYS_STATUS_SENSOR_3D_MAG                 = 4   // 0x04 3D magnetometer
	MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE      = 8   // 0x08 absolute pressure
	MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE  = 16  // 0x10 differential pressure
	MAV_SYS_STATUS_SENSOR_GPS                    = 32  // 0x20 GPS
	MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW           = 64  // 0x40 optical flow
	MAV_SYS_STATUS_SENSOR_VISION_POSITION        = 128 // 0x80 computer vision position
	MAV_SYS_STATUS_SENSOR_LASER_POSITION         = 8   // 0x100 laser based position
	MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH  = 9   // 0x200 external ground truth (Vicon or Leica)
	MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL   = 10  // 0x400 3D angular rate control
	MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 11  // 0x800 attitude stabilization
	MAV_SYS_STATUS_SENSOR_YAW_POSITION           = 12  // 0x1000 yaw position
	MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL     = 13  // 0x2000 z/altitude control
	MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL    = 14  // 0x4000 x/y position control
	MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS          = 15  // 0x8000 motor outputs / control
	MAV_SYS_STATUS_SENSOR_RC_RECEIVER            = 16  // 0x10000 rc receiver
	MAV_SYS_STATUS_SENSOR_3D_GYRO2               = 17  // 0x20000 2nd 3D gyro
	MAV_SYS_STATUS_SENSOR_3D_ACCEL2              = 18  // 0x40000 2nd 3D accelerometer
	MAV_SYS_STATUS_SENSOR_3D_MAG2                = 19  // 0x80000 2nd 3D magnetometer
	MAV_SYS_STATUS_GEOFENCE                      = 20  // 0x100000 geofence
	MAV_SYS_STATUS_AHRS                          = 21  // 0x200000 AHRS subsystem health
	MAV_SYS_STATUS_TERRAIN                       = 22  // 0x400000 Terrain subsystem health
)

// MavFrame:
const (
	MAV_FRAME_GLOBAL                  = 0  // Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL)
	MAV_FRAME_LOCAL_NED               = 1  // Local coordinate frame, Z-up (x: north, y: east, z: down).
	MAV_FRAME_MISSION                 = 2  // NOT a coordinate frame, indicates a mission command.
	MAV_FRAME_GLOBAL_RELATIVE_ALT     = 3  // Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
	MAV_FRAME_LOCAL_ENU               = 4  // Local coordinate frame, Z-down (x: east, y: north, z: up)
	MAV_FRAME_GLOBAL_INT              = 5  // Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL)
	MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6  // Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location.
	MAV_FRAME_LOCAL_OFFSET_NED        = 7  // Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position.
	MAV_FRAME_BODY_NED                = 8  // Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.
	MAV_FRAME_BODY_OFFSET_NED         = 9  // Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east.
	MAV_FRAME_GLOBAL_TERRAIN_ALT      = 10 // Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
	MAV_FRAME_GLOBAL_TERRAIN_ALT_INT  = 11 // Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
)

// MavlinkDataStreamType:
const (
	MAVLINK_DATA_STREAM_IMG_JPEG   = 0 //
	MAVLINK_DATA_STREAM_IMG_BMP    = 1 //
	MAVLINK_DATA_STREAM_IMG_RAW8U  = 2 //
	MAVLINK_DATA_STREAM_IMG_RAW32U = 3 //
	MAVLINK_DATA_STREAM_IMG_PGM    = 4 //
	MAVLINK_DATA_STREAM_IMG_PNG    = 5 //
)

// FenceAction:
const (
	FENCE_ACTION_NONE            = 0 // Disable fenced mode
	FENCE_ACTION_GUIDED          = 1 // Switched to guided mode to return point (fence point 0)
	FENCE_ACTION_REPORT          = 2 // Report fence breach, but don't take action
	FENCE_ACTION_GUIDED_THR_PASS = 3 // Switched to guided mode to return point (fence point 0) with manual throttle control
)

// FenceBreach:
const (
	FENCE_BREACH_NONE     = 0 // No last fence breach
	FENCE_BREACH_MINALT   = 1 // Breached minimum altitude
	FENCE_BREACH_MAXALT   = 2 // Breached maximum altitude
	FENCE_BREACH_BOUNDARY = 3 // Breached fence boundary
)

// MavMountMode: Enumeration of possible mount operation modes
const (
	MAV_MOUNT_MODE_RETRACT           = 0 // Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
	MAV_MOUNT_MODE_NEUTRAL           = 1 // Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
	MAV_MOUNT_MODE_MAVLINK_TARGETING = 2 // Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
	MAV_MOUNT_MODE_RC_TARGETING      = 3 // Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
	MAV_MOUNT_MODE_GPS_POINT         = 4 // Load neutral position and start to point to Lat,Lon,Alt
)

// MavCmd: Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data.
const (
	MAV_CMD_NAV_WAYPOINT                   = 16  // Navigate to MISSION.
	MAV_CMD_NAV_LOITER_UNLIM               = 17  // Loiter around this MISSION an unlimited amount of time
	MAV_CMD_NAV_LOITER_TURNS               = 18  // Loiter around this MISSION for X turns
	MAV_CMD_NAV_LOITER_TIME                = 19  // Loiter around this MISSION for X seconds
	MAV_CMD_NAV_RETURN_TO_LAUNCH           = 20  // Return to launch location
	MAV_CMD_NAV_LAND                       = 21  // Land at location
	MAV_CMD_NAV_TAKEOFF                    = 22  // Takeoff from ground / hand
	MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT    = 30  // Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached.
	MAV_CMD_NAV_LOITER_TO_ALT              = 31  // Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.  Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter until heading toward the next waypoint.
	MAV_CMD_NAV_ROI                        = 80  // Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
	MAV_CMD_NAV_PATHPLANNING               = 81  // Control autonomous path planning on the MAV.
	MAV_CMD_NAV_SPLINE_WAYPOINT            = 82  // Navigate to MISSION using a spline path.
	MAV_CMD_NAV_GUIDED_ENABLE              = 92  // hand control over to an external controller
	MAV_CMD_NAV_LAST                       = 95  // NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration
	MAV_CMD_CONDITION_DELAY                = 112 // Delay mission state machine.
	MAV_CMD_CONDITION_CHANGE_ALT           = 113 // Ascend/descend at rate.  Delay mission state machine until desired altitude reached.
	MAV_CMD_CONDITION_DISTANCE             = 114 // Delay mission state machine until within desired distance of next NAV point.
	MAV_CMD_CONDITION_YAW                  = 115 // Reach a certain target angle.
	MAV_CMD_CONDITION_LAST                 = 159 // NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration
	MAV_CMD_DO_SET_MODE                    = 176 // Set system mode.
	MAV_CMD_DO_JUMP                        = 177 // Jump to the desired command in the mission list.  Repeat this action only the specified number of times
	MAV_CMD_DO_CHANGE_SPEED                = 178 // Change speed and/or throttle set points.
	MAV_CMD_DO_SET_HOME                    = 179 // Changes the home location either to the current location or a specified location.
	MAV_CMD_DO_SET_PARAMETER               = 180 // Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter.
	MAV_CMD_DO_SET_RELAY                   = 181 // Set a relay to a condition.
	MAV_CMD_DO_REPEAT_RELAY                = 182 // Cycle a relay on and off for a desired number of cyles with a desired period.
	MAV_CMD_DO_SET_SERVO                   = 183 // Set a servo to a desired PWM value.
	MAV_CMD_DO_REPEAT_SERVO                = 184 // Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.
	MAV_CMD_DO_FLIGHTTERMINATION           = 185 // Terminate flight immediately
	MAV_CMD_DO_LAND_START                  = 189 // Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0/0 if not needed. If specified then it will be used to help find the closest landing sequence.
	MAV_CMD_DO_RALLY_LAND                  = 190 // Mission command to perform a landing from a rally point.
	MAV_CMD_DO_GO_AROUND                   = 191 // Mission command to safely abort an autonmous landing.
	MAV_CMD_DO_CONTROL_VIDEO               = 200 // Control onboard camera system.
	MAV_CMD_DO_SET_ROI                     = 201 // Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
	MAV_CMD_DO_DIGICAM_CONFIGURE           = 202 // Mission command to configure an on-board camera controller system.
	MAV_CMD_DO_DIGICAM_CONTROL             = 203 // Mission command to control an on-board camera controller system.
	MAV_CMD_DO_MOUNT_CONFIGURE             = 204 // Mission command to configure a camera or antenna mount
	MAV_CMD_DO_MOUNT_CONTROL               = 205 // Mission command to control a camera or antenna mount
	MAV_CMD_DO_SET_CAM_TRIGG_DIST          = 206 // Mission command to set CAM_TRIGG_DIST for this flight
	MAV_CMD_DO_FENCE_ENABLE                = 207 // Mission command to enable the geofence
	MAV_CMD_DO_PARACHUTE                   = 208 // Mission command to trigger a parachute
	MAV_CMD_DO_INVERTED_FLIGHT             = 210 // Change to/from inverted flight
	MAV_CMD_DO_MOUNT_CONTROL_QUAT          = 220 // Mission command to control a camera or antenna mount, using a quaternion as reference.
	MAV_CMD_DO_GUIDED_MASTER               = 221 // set id of master controller
	MAV_CMD_DO_GUIDED_LIMITS               = 222 // set limits for external control
	MAV_CMD_DO_LAST                        = 240 // NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
	MAV_CMD_PREFLIGHT_CALIBRATION          = 241 // Trigger calibration. This command will be only accepted if in pre-flight mode.
	MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS   = 242 // Set sensor offsets. This command will be only accepted if in pre-flight mode.
	MAV_CMD_PREFLIGHT_STORAGE              = 245 // Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
	MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN      = 246 // Request the reboot or shutdown of system components.
	MAV_CMD_OVERRIDE_GOTO                  = 252 // Hold / continue the current action
	MAV_CMD_MISSION_START                  = 51  // start running a mission
	MAV_CMD_COMPONENT_ARM_DISARM           = 52  // Arms / Disarms a component
	MAV_CMD_START_RX_PAIR                  = 53  // Starts receiver pairing
	MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 54  // Request autopilot capabilities
	MAV_CMD_IMAGE_START_CAPTURE            = 55  // Start image capture sequence
	MAV_CMD_IMAGE_STOP_CAPTURE             = 56  // Stop image capture sequence
	MAV_CMD_DO_TRIGGER_CONTROL             = 57  // Enable or disable on-board camera triggering system.
	MAV_CMD_VIDEO_START_CAPTURE            = 58  // Starts video capture
	MAV_CMD_VIDEO_STOP_CAPTURE             = 59  // Stop the current video capture
	MAV_CMD_PANORAMA_CREATE                = 60  // Create a panorama at the current position
	MAV_CMD_PAYLOAD_PREPARE_DEPLOY         = 61  // Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity.
	MAV_CMD_PAYLOAD_CONTROL_DEPLOY         = 62  // Control the payload deployment.
)

// MavDataStream: Data stream IDs. A data stream is not a fixed set of messages, but rather a      recommendation to the autopilot software. Individual autopilots may or may not obey      the recommended messages.
const (
	MAV_DATA_STREAM_ALL             = 0  // Enable all data streams
	MAV_DATA_STREAM_RAW_SENSORS     = 1  // Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
	MAV_DATA_STREAM_EXTENDED_STATUS = 2  // Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
	MAV_DATA_STREAM_RC_CHANNELS     = 3  // Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
	MAV_DATA_STREAM_RAW_CONTROLLER  = 4  // Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
	MAV_DATA_STREAM_POSITION        = 6  // Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
	MAV_DATA_STREAM_EXTRA1          = 10 // Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA2          = 11 // Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA3          = 12 // Dependent on the autopilot
)

// MavRoi:  The ROI (region of interest) for the vehicle. This can be                 be used by the vehicle for camera/vehicle attitude alignment (see                 MAV_CMD_NAV_ROI).
const (
	MAV_ROI_NONE     = 0 // No region of interest.
	MAV_ROI_WPNEXT   = 1 // Point toward next MISSION.
	MAV_ROI_WPINDEX  = 2 // Point toward given MISSION.
	MAV_ROI_LOCATION = 3 // Point toward fixed location.
	MAV_ROI_TARGET   = 4 // Point toward of given id.
)

// MavCmdAck: ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.
const (
	MAV_CMD_ACK_OK                                 = 0 // Command / mission item is ok.
	MAV_CMD_ACK_ERR_FAIL                           = 1 // Generic error message if none of the other reasons fails or if no detailed error reporting is implemented.
	MAV_CMD_ACK_ERR_ACCESS_DENIED                  = 2 // The system is refusing to accept this command from this source / communication partner.
	MAV_CMD_ACK_ERR_NOT_SUPPORTED                  = 3 // Command or mission item is not supported, other commands would be accepted.
	MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED = 4 // The coordinate frame of this command / mission item is not supported.
	MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE       = 5 // The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible.
	MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE             = 6 // The X or latitude value is out of range.
	MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE             = 7 // The Y or longitude value is out of range.
	MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE             = 8 // The Z or altitude value is out of range.
)

// MavParamType: Specifies the datatype of a MAVLink parameter.
const (
	MAV_PARAM_TYPE_UINT8  = 1  // 8-bit unsigned integer
	MAV_PARAM_TYPE_INT8   = 2  // 8-bit signed integer
	MAV_PARAM_TYPE_UINT16 = 3  // 16-bit unsigned integer
	MAV_PARAM_TYPE_INT16  = 4  // 16-bit signed integer
	MAV_PARAM_TYPE_UINT32 = 5  // 32-bit unsigned integer
	MAV_PARAM_TYPE_INT32  = 6  // 32-bit signed integer
	MAV_PARAM_TYPE_UINT64 = 7  // 64-bit unsigned integer
	MAV_PARAM_TYPE_INT64  = 8  // 64-bit signed integer
	MAV_PARAM_TYPE_REAL32 = 9  // 32-bit floating-point
	MAV_PARAM_TYPE_REAL64 = 10 // 64-bit floating-point
)

// MavResult: result from a mavlink command
const (
	MAV_RESULT_ACCEPTED             = 0 // Command ACCEPTED and EXECUTED
	MAV_RESULT_TEMPORARILY_REJECTED = 1 // Command TEMPORARY REJECTED/DENIED
	MAV_RESULT_DENIED               = 2 // Command PERMANENTLY DENIED
	MAV_RESULT_UNSUPPORTED          = 3 // Command UNKNOWN/UNSUPPORTED
	MAV_RESULT_FAILED               = 4 // Command executed, but failed
)

// MavMissionResult: result in a mavlink mission ack
const (
	MAV_MISSION_ACCEPTED          = 0  // mission accepted OK
	MAV_MISSION_ERROR             = 1  // generic error / not accepting mission commands at all right now
	MAV_MISSION_UNSUPPORTED_FRAME = 2  // coordinate frame is not supported
	MAV_MISSION_UNSUPPORTED       = 3  // command is not supported
	MAV_MISSION_NO_SPACE          = 4  // mission item exceeds storage space
	MAV_MISSION_INVALID           = 5  // one of the parameters has an invalid value
	MAV_MISSION_INVALID_PARAM1    = 6  // param1 has an invalid value
	MAV_MISSION_INVALID_PARAM2    = 7  // param2 has an invalid value
	MAV_MISSION_INVALID_PARAM3    = 8  // param3 has an invalid value
	MAV_MISSION_INVALID_PARAM4    = 9  // param4 has an invalid value
	MAV_MISSION_INVALID_PARAM5_X  = 10 // x/param5 has an invalid value
	MAV_MISSION_INVALID_PARAM6_Y  = 11 // y/param6 has an invalid value
	MAV_MISSION_INVALID_PARAM7    = 12 // param7 has an invalid value
	MAV_MISSION_INVALID_SEQUENCE  = 13 // received waypoint out of sequence
	MAV_MISSION_DENIED            = 14 // not accepting any mission commands from this communication partner
)

// MavSeverity: Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
const (
	MAV_SEVERITY_EMERGENCY = 0 // System is unusable. This is a "panic" condition.
	MAV_SEVERITY_ALERT     = 1 // Action should be taken immediately. Indicates error in non-critical systems.
	MAV_SEVERITY_CRITICAL  = 2 // Action must be taken immediately. Indicates failure in a primary system.
	MAV_SEVERITY_ERROR     = 3 // Indicates an error in secondary/redundant systems.
	MAV_SEVERITY_WARNING   = 4 // Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
	MAV_SEVERITY_NOTICE    = 5 // An unusual event has occured, though not an error condition. This should be investigated for the root cause.
	MAV_SEVERITY_INFO      = 6 // Normal operational messages. Useful for logging. No action is required for these messages.
	MAV_SEVERITY_DEBUG     = 7 // Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
)

// MavPowerStatus: Power supply status flags (bitmask)
const (
	MAV_POWER_STATUS_BRICK_VALID                = 1  // main brick power supply valid
	MAV_POWER_STATUS_SERVO_VALID                = 2  // main servo power supply valid for FMU
	MAV_POWER_STATUS_USB_CONNECTED              = 4  // USB power is connected
	MAV_POWER_STATUS_PERIPH_OVERCURRENT         = 8  // peripheral supply is in over-current state
	MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = 16 // hi-power peripheral supply is in over-current state
	MAV_POWER_STATUS_CHANGED                    = 32 // Power status has changed since boot
)

// SerialControlDev: SERIAL_CONTROL device types
const (
	SERIAL_CONTROL_DEV_TELEM1 = 0 // First telemetry port
	SERIAL_CONTROL_DEV_TELEM2 = 1 // Second telemetry port
	SERIAL_CONTROL_DEV_GPS1   = 2 // First GPS port
	SERIAL_CONTROL_DEV_GPS2   = 3 // Second GPS port
)

// SerialControlFlag: SERIAL_CONTROL flags (bitmask)
const (
	SERIAL_CONTROL_FLAG_REPLY     = 1  // Set if this is a reply
	SERIAL_CONTROL_FLAG_RESPOND   = 2  // Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message
	SERIAL_CONTROL_FLAG_EXCLUSIVE = 4  // Set if access to the serial port should be removed from whatever driver is currently using it, giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without this flag set
	SERIAL_CONTROL_FLAG_BLOCKING  = 8  // Block on writes to the serial port
	SERIAL_CONTROL_FLAG_MULTI     = 16 // Send multiple replies until port is drained
)

// MavDistanceSensor: Enumeration of distance sensor types
const (
	MAV_DISTANCE_SENSOR_LASER      = 0 // Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
	MAV_DISTANCE_SENSOR_ULTRASOUND = 1 // Ultrasound rangefinder, e.g. MaxBotix units
	MAV_DISTANCE_SENSOR_INFRARED   = 2 // Infrared rangefinder, e.g. Sharp units
)

// MavSensorOrientation: Enumeration of sensor orientation, according to its rotations
const (
	MAV_SENSOR_ROTATION_NONE                       = 0  // Roll: 0, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_YAW_45                     = 1  // Roll: 0, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_YAW_90                     = 2  // Roll: 0, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_YAW_135                    = 3  // Roll: 0, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_YAW_180                    = 4  // Roll: 0, Pitch: 0, Yaw: 180
	MAV_SENSOR_ROTATION_YAW_225                    = 5  // Roll: 0, Pitch: 0, Yaw: 225
	MAV_SENSOR_ROTATION_YAW_270                    = 6  // Roll: 0, Pitch: 0, Yaw: 270
	MAV_SENSOR_ROTATION_YAW_315                    = 7  // Roll: 0, Pitch: 0, Yaw: 315
	MAV_SENSOR_ROTATION_ROLL_180                   = 8  // Roll: 180, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_YAW_45            = 9  // Roll: 180, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_ROLL_180_YAW_90            = 10 // Roll: 180, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_180_YAW_135           = 11 // Roll: 180, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_PITCH_180                  = 12 // Roll: 0, Pitch: 180, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_YAW_225           = 13 // Roll: 180, Pitch: 0, Yaw: 225
	MAV_SENSOR_ROTATION_ROLL_180_YAW_270           = 14 // Roll: 180, Pitch: 0, Yaw: 270
	MAV_SENSOR_ROTATION_ROLL_180_YAW_315           = 15 // Roll: 180, Pitch: 0, Yaw: 315
	MAV_SENSOR_ROTATION_ROLL_90                    = 16 // Roll: 90, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_YAW_45             = 17 // Roll: 90, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_ROLL_90_YAW_90             = 18 // Roll: 90, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_90_YAW_135            = 19 // Roll: 90, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_ROLL_270                   = 20 // Roll: 270, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_YAW_45            = 21 // Roll: 270, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_ROLL_270_YAW_90            = 22 // Roll: 270, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_270_YAW_135           = 23 // Roll: 270, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_PITCH_90                   = 24 // Roll: 0, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_PITCH_270                  = 25 // Roll: 0, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_PITCH_180_YAW_90           = 26 // Roll: 0, Pitch: 180, Yaw: 90
	MAV_SENSOR_ROTATION_PITCH_180_YAW_270          = 27 // Roll: 0, Pitch: 180, Yaw: 270
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_90           = 28 // Roll: 90, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_PITCH_90          = 29 // Roll: 180, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_PITCH_90          = 30 // Roll: 270, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_180          = 31 // Roll: 90, Pitch: 180, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_PITCH_180         = 32 // Roll: 270, Pitch: 180, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_270          = 33 // Roll: 90, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_PITCH_270         = 34 // Roll: 180, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_PITCH_270         = 35 // Roll: 270, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90   = 36 // Roll: 90, Pitch: 180, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_90_YAW_270            = 37 // Roll: 90, Pitch: 0, Yaw: 270
	MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315 = 38 // Roll: 315, Pitch: 315, Yaw: 315
)

// MavProtocolCapability: Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability.
const (
	MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT                  = 1   // Autopilot supports MISSION float message type.
	MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT                    = 2   // Autopilot supports the new param float message type.
	MAV_PROTOCOL_CAPABILITY_MISSION_INT                    = 4   // Autopilot supports MISSION_INT scaled integer message type.
	MAV_PROTOCOL_CAPABILITY_COMMAND_INT                    = 8   // Autopilot supports COMMAND_INT scaled integer message type.
	MAV_PROTOCOL_CAPABILITY_PARAM_UNION                    = 16  // Autopilot supports the new param union message type.
	MAV_PROTOCOL_CAPABILITY_FTP                            = 32  // Autopilot supports the new param union message type.
	MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET            = 64  // Autopilot supports commanding attitude offboard.
	MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED  = 128 // Autopilot supports commanding position and velocity targets in local NED frame.
	MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 8   // Autopilot supports commanding position and velocity targets in global scaled integers.
	MAV_PROTOCOL_CAPABILITY_TERRAIN                        = 9   // Autopilot supports terrain protocol / data handling.
	MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET            = 10  // Autopilot supports direct actuator control.
)

// MavEstimatorType: Enumeration of estimator types
const (
	MAV_ESTIMATOR_TYPE_NAIVE   = 1 // This is a naive estimator without any real covariance feedback.
	MAV_ESTIMATOR_TYPE_VISION  = 2 // Computer vision based estimate. Might be up to scale.
	MAV_ESTIMATOR_TYPE_VIO     = 3 // Visual-inertial estimate.
	MAV_ESTIMATOR_TYPE_GPS     = 4 // Plain GPS estimate.
	MAV_ESTIMATOR_TYPE_GPS_INS = 5 // Estimator integrating GPS and inertial sensing.
)

// MavBatteryType: Enumeration of battery types
const (
	MAV_BATTERY_TYPE_UNKNOWN = 0 // Not specified.
	MAV_BATTERY_TYPE_LIPO    = 1 // Lithium polymere battery
	MAV_BATTERY_TYPE_LIFE    = 2 // Lithium ferrite battery
	MAV_BATTERY_TYPE_LION    = 3 // Lithium-ION battery
	MAV_BATTERY_TYPE_NIMH    = 4 // Nickel metal hydride battery
)

// MavBatteryFunction: Enumeration of battery functions
const (
	MAV_BATTERY_FUNCTION_UNKNOWN    = 0 // Lithium polymere battery
	MAV_BATTERY_FUNCTION_ALL        = 1 // Battery supports all flight systems
	MAV_BATTERY_FUNCTION_PROPULSION = 2 // Battery for the propulsion system
	MAV_BATTERY_FUNCTION_AVIONICS   = 3 // Avionics battery
	MAV_BATTERY_TYPE_PAYLOAD        = 4 // Payload battery
)

// Message IDs
const (
	MSG_ID_HEARTBEAT                               = 0
	MSG_ID_SYS_STATUS                              = 1
	MSG_ID_SYSTEM_TIME                             = 2
	MSG_ID_PING                                    = 4
	MSG_ID_CHANGE_OPERATOR_CONTROL                 = 5
	MSG_ID_CHANGE_OPERATOR_CONTROL_ACK             = 6
	MSG_ID_AUTH_KEY                                = 7
	MSG_ID_SET_MODE                                = 11
	MSG_ID_PARAM_REQUEST_READ                      = 20
	MSG_ID_PARAM_REQUEST_LIST                      = 21
	MSG_ID_PARAM_VALUE                             = 22
	MSG_ID_PARAM_SET                               = 23
	MSG_ID_GPS_RAW_INT                             = 24
	MSG_ID_GPS_STATUS                              = 25
	MSG_ID_SCALED_IMU                              = 26
	MSG_ID_RAW_IMU                                 = 27
	MSG_ID_RAW_PRESSURE                            = 28
	MSG_ID_SCALED_PRESSURE                         = 29
	MSG_ID_ATTITUDE                                = 30
	MSG_ID_ATTITUDE_QUATERNION                     = 31
	MSG_ID_LOCAL_POSITION_NED                      = 32
	MSG_ID_GLOBAL_POSITION_INT                     = 33
	MSG_ID_RC_CHANNELS_SCALED                      = 34
	MSG_ID_RC_CHANNELS_RAW                         = 35
	MSG_ID_SERVO_OUTPUT_RAW                        = 36
	MSG_ID_MISSION_REQUEST_PARTIAL_LIST            = 37
	MSG_ID_MISSION_WRITE_PARTIAL_LIST              = 38
	MSG_ID_MISSION_ITEM                            = 39
	MSG_ID_MISSION_REQUEST                         = 40
	MSG_ID_MISSION_SET_CURRENT                     = 41
	MSG_ID_MISSION_CURRENT                         = 42
	MSG_ID_MISSION_REQUEST_LIST                    = 43
	MSG_ID_MISSION_COUNT                           = 44
	MSG_ID_MISSION_CLEAR_ALL                       = 45
	MSG_ID_MISSION_ITEM_REACHED                    = 46
	MSG_ID_MISSION_ACK                             = 47
	MSG_ID_SET_GPS_GLOBAL_ORIGIN                   = 48
	MSG_ID_GPS_GLOBAL_ORIGIN                       = 49
	MSG_ID_PARAM_MAP_RC                            = 50
	MSG_ID_SAFETY_SET_ALLOWED_AREA                 = 54
	MSG_ID_SAFETY_ALLOWED_AREA                     = 55
	MSG_ID_ATTITUDE_QUATERNION_COV                 = 61
	MSG_ID_NAV_CONTROLLER_OUTPUT                   = 62
	MSG_ID_GLOBAL_POSITION_INT_COV                 = 63
	MSG_ID_LOCAL_POSITION_NED_COV                  = 64
	MSG_ID_RC_CHANNELS                             = 65
	MSG_ID_REQUEST_DATA_STREAM                     = 66
	MSG_ID_DATA_STREAM                             = 67
	MSG_ID_MANUAL_CONTROL                          = 69
	MSG_ID_RC_CHANNELS_OVERRIDE                    = 70
	MSG_ID_MISSION_ITEM_INT                        = 73
	MSG_ID_VFR_HUD                                 = 74
	MSG_ID_COMMAND_INT                             = 75
	MSG_ID_COMMAND_LONG                            = 76
	MSG_ID_COMMAND_ACK                             = 77
	MSG_ID_MANUAL_SETPOINT                         = 81
	MSG_ID_SET_ATTITUDE_TARGET                     = 82
	MSG_ID_ATTITUDE_TARGET                         = 83
	MSG_ID_SET_POSITION_TARGET_LOCAL_NED           = 84
	MSG_ID_POSITION_TARGET_LOCAL_NED               = 85
	MSG_ID_SET_POSITION_TARGET_GLOBAL_INT          = 86
	MSG_ID_POSITION_TARGET_GLOBAL_INT              = 87
	MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET = 89
	MSG_ID_HIL_STATE                               = 90
	MSG_ID_HIL_CONTROLS                            = 91
	MSG_ID_HIL_RC_INPUTS_RAW                       = 92
	MSG_ID_OPTICAL_FLOW                            = 100
	MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE         = 101
	MSG_ID_VISION_POSITION_ESTIMATE                = 102
	MSG_ID_VISION_SPEED_ESTIMATE                   = 103
	MSG_ID_VICON_POSITION_ESTIMATE                 = 104
	MSG_ID_HIGHRES_IMU                             = 105
	MSG_ID_OPTICAL_FLOW_RAD                        = 106
	MSG_ID_HIL_SENSOR                              = 107
	MSG_ID_SIM_STATE                               = 108
	MSG_ID_RADIO_STATUS                            = 109
	MSG_ID_FILE_TRANSFER_PROTOCOL                  = 110
	MSG_ID_TIMESYNC                                = 111
	MSG_ID_CAMERA_TRIGGER                          = 112
	MSG_ID_HIL_GPS                                 = 113
	MSG_ID_HIL_OPTICAL_FLOW                        = 114
	MSG_ID_HIL_STATE_QUATERNION                    = 115
	MSG_ID_SCALED_IMU2                             = 116
	MSG_ID_LOG_REQUEST_LIST                        = 117
	MSG_ID_LOG_ENTRY                               = 118
	MSG_ID_LOG_REQUEST_DATA                        = 119
	MSG_ID_LOG_DATA                                = 120
	MSG_ID_LOG_ERASE                               = 121
	MSG_ID_LOG_REQUEST_END                         = 122
	MSG_ID_GPS_INJECT_DATA                         = 123
	MSG_ID_GPS2_RAW                                = 124
	MSG_ID_POWER_STATUS                            = 125
	MSG_ID_SERIAL_CONTROL                          = 126
	MSG_ID_GPS_RTK                                 = 127
	MSG_ID_GPS2_RTK                                = 128
	MSG_ID_SCALED_IMU3                             = 129
	MSG_ID_DATA_TRANSMISSION_HANDSHAKE             = 130
	MSG_ID_ENCAPSULATED_DATA                       = 131
	MSG_ID_DISTANCE_SENSOR                         = 132
	MSG_ID_TERRAIN_REQUEST                         = 133
	MSG_ID_TERRAIN_DATA                            = 134
	MSG_ID_TERRAIN_CHECK                           = 135
	MSG_ID_TERRAIN_REPORT                          = 136
	MSG_ID_SCALED_PRESSURE2                        = 137
	MSG_ID_ATT_POS_MOCAP                           = 138
	MSG_ID_SET_ACTUATOR_CONTROL_TARGET             = 139
	MSG_ID_ACTUATOR_CONTROL_TARGET                 = 140
	MSG_ID_BATTERY_STATUS                          = 147
	MSG_ID_AUTOPILOT_VERSION                       = 148
	MSG_ID_LANDING_TARGET                          = 149
	MSG_ID_V2_EXTENSION                            = 248
	MSG_ID_MEMORY_VECT                             = 249
	MSG_ID_DEBUG_VECT                              = 250
	MSG_ID_NAMED_VALUE_FLOAT                       = 251
	MSG_ID_NAMED_VALUE_INT                         = 252
	MSG_ID_STATUSTEXT                              = 253
	MSG_ID_DEBUG                                   = 254
)

// The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying out the user interface based on the autopilot).
type Heartbeat struct {
	Type           uint8  // Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
	Autopilot      uint8  // Autopilot type / class. defined in MAV_AUTOPILOT ENUM
	BaseMode       uint8  // System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
	CustomMode     uint32 // A bitfield for use for autopilot-specific flags.
	SystemStatus   uint8  // System status flag, see MAV_STATE ENUM
	MavlinkVersion uint8  // MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
}

func (self *Heartbeat) MsgID() uint8 {
	return 0
}

func (self *Heartbeat) MsgName() string {
	return "Heartbeat"
}

func (self *Heartbeat) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Type,
		&self.Autopilot,
		&self.BaseMode,
		&self.CustomMode,
		&self.SystemStatus,
		&self.MavlinkVersion,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Heartbeat) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Type,
		&self.Autopilot,
		&self.BaseMode,
		&self.CustomMode,
		&self.SystemStatus,
		&self.MavlinkVersion,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows wether the system is currently active or not and if an emergency occured. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occured it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
type SysStatus struct {
	OnboardControlSensorsPresent uint32 // Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
	OnboardControlSensorsEnabled uint32 // Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
	OnboardControlSensorsHealth  uint32 // Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
	Load                         uint16 // Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
	VoltageBattery               uint16 // Battery voltage, in millivolts (1 = 1 millivolt)
	CurrentBattery               int16  // Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
	BatteryRemaining             int8   // Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
	DropRateComm                 uint16 // Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	ErrorsComm                   uint16 // Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	ErrorsCount1                 uint16 // Autopilot-specific errors
	ErrorsCount2                 uint16 // Autopilot-specific errors
	ErrorsCount3                 uint16 // Autopilot-specific errors
	ErrorsCount4                 uint16 // Autopilot-specific errors
}

func (self *SysStatus) MsgID() uint8 {
	return 1
}

func (self *SysStatus) MsgName() string {
	return "SysStatus"
}

func (self *SysStatus) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.OnboardControlSensorsPresent,
		&self.OnboardControlSensorsEnabled,
		&self.OnboardControlSensorsHealth,
		&self.Load,
		&self.VoltageBattery,
		&self.CurrentBattery,
		&self.BatteryRemaining,
		&self.DropRateComm,
		&self.ErrorsComm,
		&self.ErrorsCount1,
		&self.ErrorsCount2,
		&self.ErrorsCount3,
		&self.ErrorsCount4,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SysStatus) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.OnboardControlSensorsPresent,
		&self.OnboardControlSensorsEnabled,
		&self.OnboardControlSensorsHealth,
		&self.Load,
		&self.VoltageBattery,
		&self.CurrentBattery,
		&self.BatteryRemaining,
		&self.DropRateComm,
		&self.ErrorsComm,
		&self.ErrorsCount1,
		&self.ErrorsCount2,
		&self.ErrorsCount3,
		&self.ErrorsCount4,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The system time is the time of the master clock, typically the computer clock of the main onboard computer.
type SystemTime struct {
	TimeUnixUsec uint64 // Timestamp of the master clock in microseconds since UNIX epoch.
	TimeBootMs   uint32 // Timestamp of the component clock since boot time in milliseconds.
}

func (self *SystemTime) MsgID() uint8 {
	return 2
}

func (self *SystemTime) MsgName() string {
	return "SystemTime"
}

func (self *SystemTime) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUnixUsec,
		&self.TimeBootMs,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SystemTime) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUnixUsec,
		&self.TimeBootMs,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections.
type Ping struct {
	TimeUsec        uint64 // Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
	Seq             uint32 // PING sequence
	TargetSystem    uint8  // 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
	TargetComponent uint8  // 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
}

func (self *Ping) MsgID() uint8 {
	return 4
}

func (self *Ping) MsgName() string {
	return "Ping"
}

func (self *Ping) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Seq,
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

func (self *Ping) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Seq,
		&self.TargetSystem,
		&self.TargetComponent,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Request to control this MAV
type ChangeOperatorControl struct {
	TargetSystem   uint8    // System the GCS requests control for
	ControlRequest uint8    // 0: request control of this MAV, 1: Release control of this MAV
	Version        uint8    // 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
	Passkey        [25]byte // Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
}

func (self *ChangeOperatorControl) MsgID() uint8 {
	return 5
}

func (self *ChangeOperatorControl) MsgName() string {
	return "ChangeOperatorControl"
}

func (self *ChangeOperatorControl) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.ControlRequest,
		&self.Version,
		&self.Passkey,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ChangeOperatorControl) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.ControlRequest,
		&self.Version,
		&self.Passkey,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Accept / deny control of this MAV
type ChangeOperatorControlAck struct {
	GcsSystemId    uint8 // ID of the GCS this message
	ControlRequest uint8 // 0: request control of this MAV, 1: Release control of this MAV
	Ack            uint8 // 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control
}

func (self *ChangeOperatorControlAck) MsgID() uint8 {
	return 6
}

func (self *ChangeOperatorControlAck) MsgName() string {
	return "ChangeOperatorControlAck"
}

func (self *ChangeOperatorControlAck) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.GcsSystemId,
		&self.ControlRequest,
		&self.Ack,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ChangeOperatorControlAck) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.GcsSystemId,
		&self.ControlRequest,
		&self.Ack,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple, so transmitting the key requires an encrypted channel for true safety.
type AuthKey struct {
	Key [32]byte // key
}

func (self *AuthKey) MsgID() uint8 {
	return 7
}

func (self *AuthKey) MsgName() string {
	return "AuthKey"
}

func (self *AuthKey) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Key,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *AuthKey) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Key,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall aircraft, not only for one component.
type SetMode struct {
	TargetSystem uint8  // The system setting the mode
	BaseMode     uint8  // The new base mode
	CustomMode   uint32 // The new autopilot-specific mode. This field can be ignored by an autopilot.
}

func (self *SetMode) MsgID() uint8 {
	return 11
}

func (self *SetMode) MsgName() string {
	return "SetMode"
}

func (self *SetMode) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.BaseMode,
		&self.CustomMode,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SetMode) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.BaseMode,
		&self.CustomMode,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also http://qgroundcontrol.org/parameter_interface for a full documentation of QGroundControl and IMU code.
type ParamRequestRead struct {
	TargetSystem    uint8    // System ID
	TargetComponent uint8    // Component ID
	ParamId         [16]byte // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamIndex      int16    // Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
}

func (self *ParamRequestRead) MsgID() uint8 {
	return 20
}

func (self *ParamRequestRead) MsgName() string {
	return "ParamRequestRead"
}

func (self *ParamRequestRead) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.ParamId,
		&self.ParamIndex,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ParamRequestRead) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.ParamId,
		&self.ParamIndex,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Request all parameters of this component. After his request, all parameters are emitted.
type ParamRequestList struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *ParamRequestList) MsgID() uint8 {
	return 21
}

func (self *ParamRequestList) MsgName() string {
	return "ParamRequestList"
}

func (self *ParamRequestList) Pack(p *Packet) error {
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

func (self *ParamRequestList) Unpack(p *Packet) error {
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

// Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout.
type ParamValue struct {
	ParamId    [16]byte // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamValue float32  // Onboard parameter value
	ParamType  uint8    // Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
	ParamCount uint16   // Total number of onboard parameters
	ParamIndex uint16   // Index of this onboard parameter
}

func (self *ParamValue) MsgID() uint8 {
	return 22
}

func (self *ParamValue) MsgName() string {
	return "ParamValue"
}

func (self *ParamValue) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.ParamId,
		&self.ParamValue,
		&self.ParamType,
		&self.ParamCount,
		&self.ParamIndex,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ParamValue) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.ParamId,
		&self.ParamValue,
		&self.ParamType,
		&self.ParamCount,
		&self.ParamIndex,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component should acknowledge the new parameter value by sending a param_value message to all communication partners. This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message.
type ParamSet struct {
	TargetSystem    uint8    // System ID
	TargetComponent uint8    // Component ID
	ParamId         [16]byte // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamValue      float32  // Onboard parameter value
	ParamType       uint8    // Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
}

func (self *ParamSet) MsgID() uint8 {
	return 23
}

func (self *ParamSet) MsgName() string {
	return "ParamSet"
}

func (self *ParamSet) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.ParamId,
		&self.ParamValue,
		&self.ParamType,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ParamSet) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.ParamId,
		&self.ParamValue,
		&self.ParamType,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The global position, as returned by the Global Positioning System (GPS). This is
//                 NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
type GpsRawInt struct {
	TimeUsec          uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	FixType           uint8  // 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
	Lat               int32  // Latitude (WGS84), in degrees * 1E7
	Lon               int32  // Longitude (WGS84), in degrees * 1E7
	Alt               int32  // Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
	Eph               uint16 // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	Epv               uint16 // GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	Vel               uint16 // GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
	Cog               uint16 // Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	SatellitesVisible uint8  // Number of satellites visible. If unknown, set to 255
}

func (self *GpsRawInt) MsgID() uint8 {
	return 24
}

func (self *GpsRawInt) MsgName() string {
	return "GpsRawInt"
}

func (self *GpsRawInt) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.FixType,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.Eph,
		&self.Epv,
		&self.Vel,
		&self.Cog,
		&self.SatellitesVisible,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *GpsRawInt) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.FixType,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.Eph,
		&self.Epv,
		&self.Vel,
		&self.Cog,
		&self.SatellitesVisible,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
type GpsStatus struct {
	SatellitesVisible  uint8     // Number of satellites visible
	SatellitePrn       [20]uint8 // Global satellite ID
	SatelliteUsed      [20]uint8 // 0: Satellite not used, 1: used for localization
	SatelliteElevation [20]uint8 // Elevation (0: right on top of receiver, 90: on the horizon) of satellite
	SatelliteAzimuth   [20]uint8 // Direction of satellite, 0: 0 deg, 255: 360 deg.
	SatelliteSnr       [20]uint8 // Signal to noise ratio of satellite
}

func (self *GpsStatus) MsgID() uint8 {
	return 25
}

func (self *GpsStatus) MsgName() string {
	return "GpsStatus"
}

func (self *GpsStatus) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.SatellitesVisible,
		&self.SatellitePrn,
		&self.SatelliteUsed,
		&self.SatelliteElevation,
		&self.SatelliteAzimuth,
		&self.SatelliteSnr,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *GpsStatus) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.SatellitesVisible,
		&self.SatellitePrn,
		&self.SatelliteUsed,
		&self.SatelliteElevation,
		&self.SatelliteAzimuth,
		&self.SatelliteSnr,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
type ScaledImu struct {
	TimeBootMs uint32 // Timestamp (milliseconds since system boot)
	Xacc       int16  // X acceleration (mg)
	Yacc       int16  // Y acceleration (mg)
	Zacc       int16  // Z acceleration (mg)
	Xgyro      int16  // Angular speed around X axis (millirad /sec)
	Ygyro      int16  // Angular speed around Y axis (millirad /sec)
	Zgyro      int16  // Angular speed around Z axis (millirad /sec)
	Xmag       int16  // X Magnetic field (milli tesla)
	Ymag       int16  // Y Magnetic field (milli tesla)
	Zmag       int16  // Z Magnetic field (milli tesla)
}

func (self *ScaledImu) MsgID() uint8 {
	return 26
}

func (self *ScaledImu) MsgName() string {
	return "ScaledImu"
}

func (self *ScaledImu) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
		&self.Xgyro,
		&self.Ygyro,
		&self.Zgyro,
		&self.Xmag,
		&self.Ymag,
		&self.Zmag,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ScaledImu) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
		&self.Xgyro,
		&self.Ygyro,
		&self.Zgyro,
		&self.Xmag,
		&self.Ymag,
		&self.Zmag,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw values without any scaling to allow data capture and system debugging.
type RawImu struct {
	TimeUsec uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Xacc     int16  // X acceleration (raw)
	Yacc     int16  // Y acceleration (raw)
	Zacc     int16  // Z acceleration (raw)
	Xgyro    int16  // Angular speed around X axis (raw)
	Ygyro    int16  // Angular speed around Y axis (raw)
	Zgyro    int16  // Angular speed around Z axis (raw)
	Xmag     int16  // X Magnetic field (raw)
	Ymag     int16  // Y Magnetic field (raw)
	Zmag     int16  // Z Magnetic field (raw)
}

func (self *RawImu) MsgID() uint8 {
	return 27
}

func (self *RawImu) MsgName() string {
	return "RawImu"
}

func (self *RawImu) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
		&self.Xgyro,
		&self.Ygyro,
		&self.Zgyro,
		&self.Xmag,
		&self.Ymag,
		&self.Zmag,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *RawImu) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
		&self.Xgyro,
		&self.Ygyro,
		&self.Zgyro,
		&self.Xmag,
		&self.Ymag,
		&self.Zmag,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
type RawPressure struct {
	TimeUsec    uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	PressAbs    int16  // Absolute pressure (raw)
	PressDiff1  int16  // Differential pressure 1 (raw)
	PressDiff2  int16  // Differential pressure 2 (raw)
	Temperature int16  // Raw Temperature measurement (raw)
}

func (self *RawPressure) MsgID() uint8 {
	return 28
}

func (self *RawPressure) MsgName() string {
	return "RawPressure"
}

func (self *RawPressure) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.PressAbs,
		&self.PressDiff1,
		&self.PressDiff2,
		&self.Temperature,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *RawPressure) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.PressAbs,
		&self.PressDiff1,
		&self.PressDiff2,
		&self.Temperature,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The pressure readings for the typical setup of one absolute and differential pressure sensor. The units are as specified in each field.
type ScaledPressure struct {
	TimeBootMs  uint32  // Timestamp (milliseconds since system boot)
	PressAbs    float32 // Absolute pressure (hectopascal)
	PressDiff   float32 // Differential pressure 1 (hectopascal)
	Temperature int16   // Temperature measurement (0.01 degrees celsius)
}

func (self *ScaledPressure) MsgID() uint8 {
	return 29
}

func (self *ScaledPressure) MsgName() string {
	return "ScaledPressure"
}

func (self *ScaledPressure) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.PressAbs,
		&self.PressDiff,
		&self.Temperature,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ScaledPressure) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.PressAbs,
		&self.PressDiff,
		&self.Temperature,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
type Attitude struct {
	TimeBootMs uint32  // Timestamp (milliseconds since system boot)
	Roll       float32 // Roll angle (rad, -pi..+pi)
	Pitch      float32 // Pitch angle (rad, -pi..+pi)
	Yaw        float32 // Yaw angle (rad, -pi..+pi)
	Rollspeed  float32 // Roll angular speed (rad/s)
	Pitchspeed float32 // Pitch angular speed (rad/s)
	Yawspeed   float32 // Yaw angular speed (rad/s)
}

func (self *Attitude) MsgID() uint8 {
	return 30
}

func (self *Attitude) MsgName() string {
	return "Attitude"
}

func (self *Attitude) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.Rollspeed,
		&self.Pitchspeed,
		&self.Yawspeed,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Attitude) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.Rollspeed,
		&self.Pitchspeed,
		&self.Yawspeed,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
type AttitudeQuaternion struct {
	TimeBootMs uint32  // Timestamp (milliseconds since system boot)
	Q1         float32 // Quaternion component 1, w (1 in null-rotation)
	Q2         float32 // Quaternion component 2, x (0 in null-rotation)
	Q3         float32 // Quaternion component 3, y (0 in null-rotation)
	Q4         float32 // Quaternion component 4, z (0 in null-rotation)
	Rollspeed  float32 // Roll angular speed (rad/s)
	Pitchspeed float32 // Pitch angular speed (rad/s)
	Yawspeed   float32 // Yaw angular speed (rad/s)
}

func (self *AttitudeQuaternion) MsgID() uint8 {
	return 31
}

func (self *AttitudeQuaternion) MsgName() string {
	return "AttitudeQuaternion"
}

func (self *AttitudeQuaternion) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Q1,
		&self.Q2,
		&self.Q3,
		&self.Q4,
		&self.Rollspeed,
		&self.Pitchspeed,
		&self.Yawspeed,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *AttitudeQuaternion) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Q1,
		&self.Q2,
		&self.Q3,
		&self.Q4,
		&self.Rollspeed,
		&self.Pitchspeed,
		&self.Yawspeed,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNed struct {
	TimeBootMs uint32  // Timestamp (milliseconds since system boot)
	X          float32 // X Position
	Y          float32 // Y Position
	Z          float32 // Z Position
	Vx         float32 // X Speed
	Vy         float32 // Y Speed
	Vz         float32 // Z Speed
}

func (self *LocalPositionNed) MsgID() uint8 {
	return 32
}

func (self *LocalPositionNed) MsgName() string {
	return "LocalPositionNed"
}

func (self *LocalPositionNed) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.X,
		&self.Y,
		&self.Z,
		&self.Vx,
		&self.Vy,
		&self.Vz,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *LocalPositionNed) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.X,
		&self.Y,
		&self.Z,
		&self.Vx,
		&self.Vy,
		&self.Vz,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It
//                is designed as scaled integer message since the resolution of float is not sufficient.
type GlobalPositionInt struct {
	TimeBootMs  uint32 // Timestamp (milliseconds since system boot)
	Lat         int32  // Latitude, expressed as * 1E7
	Lon         int32  // Longitude, expressed as * 1E7
	Alt         int32  // Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
	RelativeAlt int32  // Altitude above ground in meters, expressed as * 1000 (millimeters)
	Vx          int16  // Ground X Speed (Latitude), expressed as m/s * 100
	Vy          int16  // Ground Y Speed (Longitude), expressed as m/s * 100
	Vz          int16  // Ground Z Speed (Altitude), expressed as m/s * 100
	Hdg         uint16 // Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
}

func (self *GlobalPositionInt) MsgID() uint8 {
	return 33
}

func (self *GlobalPositionInt) MsgName() string {
	return "GlobalPositionInt"
}

func (self *GlobalPositionInt) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.RelativeAlt,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.Hdg,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *GlobalPositionInt) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.RelativeAlt,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.Hdg,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are inactive should be set to UINT16_MAX.
type RcChannelsScaled struct {
	TimeBootMs  uint32 // Timestamp (milliseconds since system boot)
	Port        uint8  // Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
	Chan1Scaled int16  // RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan2Scaled int16  // RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan3Scaled int16  // RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan4Scaled int16  // RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan5Scaled int16  // RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan6Scaled int16  // RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan7Scaled int16  // RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan8Scaled int16  // RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Rssi        uint8  // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
}

func (self *RcChannelsScaled) MsgID() uint8 {
	return 34
}

func (self *RcChannelsScaled) MsgName() string {
	return "RcChannelsScaled"
}

func (self *RcChannelsScaled) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Port,
		&self.Chan1Scaled,
		&self.Chan2Scaled,
		&self.Chan3Scaled,
		&self.Chan4Scaled,
		&self.Chan5Scaled,
		&self.Chan6Scaled,
		&self.Chan7Scaled,
		&self.Chan8Scaled,
		&self.Rssi,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *RcChannelsScaled) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Port,
		&self.Chan1Scaled,
		&self.Chan2Scaled,
		&self.Chan3Scaled,
		&self.Chan4Scaled,
		&self.Chan5Scaled,
		&self.Chan6Scaled,
		&self.Chan7Scaled,
		&self.Chan8Scaled,
		&self.Rssi,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type RcChannelsRaw struct {
	TimeBootMs uint32 // Timestamp (milliseconds since system boot)
	Port       uint8  // Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
	Chan1Raw   uint16 // RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan2Raw   uint16 // RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan3Raw   uint16 // RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan4Raw   uint16 // RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan5Raw   uint16 // RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan6Raw   uint16 // RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan7Raw   uint16 // RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan8Raw   uint16 // RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Rssi       uint8  // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
}

func (self *RcChannelsRaw) MsgID() uint8 {
	return 35
}

func (self *RcChannelsRaw) MsgName() string {
	return "RcChannelsRaw"
}

func (self *RcChannelsRaw) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Port,
		&self.Chan1Raw,
		&self.Chan2Raw,
		&self.Chan3Raw,
		&self.Chan4Raw,
		&self.Chan5Raw,
		&self.Chan6Raw,
		&self.Chan7Raw,
		&self.Chan8Raw,
		&self.Rssi,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *RcChannelsRaw) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Port,
		&self.Chan1Raw,
		&self.Chan2Raw,
		&self.Chan3Raw,
		&self.Chan4Raw,
		&self.Chan5Raw,
		&self.Chan6Raw,
		&self.Chan7Raw,
		&self.Chan8Raw,
		&self.Rssi,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
type ServoOutputRaw struct {
	TimeUsec  uint32 // Timestamp (microseconds since system boot)
	Port      uint8  // Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
	Servo1Raw uint16 // Servo output 1 value, in microseconds
	Servo2Raw uint16 // Servo output 2 value, in microseconds
	Servo3Raw uint16 // Servo output 3 value, in microseconds
	Servo4Raw uint16 // Servo output 4 value, in microseconds
	Servo5Raw uint16 // Servo output 5 value, in microseconds
	Servo6Raw uint16 // Servo output 6 value, in microseconds
	Servo7Raw uint16 // Servo output 7 value, in microseconds
	Servo8Raw uint16 // Servo output 8 value, in microseconds
}

func (self *ServoOutputRaw) MsgID() uint8 {
	return 36
}

func (self *ServoOutputRaw) MsgName() string {
	return "ServoOutputRaw"
}

func (self *ServoOutputRaw) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Port,
		&self.Servo1Raw,
		&self.Servo2Raw,
		&self.Servo3Raw,
		&self.Servo4Raw,
		&self.Servo5Raw,
		&self.Servo6Raw,
		&self.Servo7Raw,
		&self.Servo8Raw,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ServoOutputRaw) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Port,
		&self.Servo1Raw,
		&self.Servo2Raw,
		&self.Servo3Raw,
		&self.Servo4Raw,
		&self.Servo5Raw,
		&self.Servo6Raw,
		&self.Servo7Raw,
		&self.Servo8Raw,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Request a partial list of mission items from the system/component. http://qgroundcontrol.org/mavlink/waypoint_protocol. If start and end index are the same, just send one waypoint.
type MissionRequestPartialList struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	StartIndex      int16 // Start index, 0 by default
	EndIndex        int16 // End index, -1 by default (-1: send list to end). Else a valid index of the list
}

func (self *MissionRequestPartialList) MsgID() uint8 {
	return 37
}

func (self *MissionRequestPartialList) MsgName() string {
	return "MissionRequestPartialList"
}

func (self *MissionRequestPartialList) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.StartIndex,
		&self.EndIndex,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *MissionRequestPartialList) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.StartIndex,
		&self.EndIndex,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// This message is sent to the MAV to write a partial list. If start index == end index, only one item will be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should be REJECTED!
type MissionWritePartialList struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	StartIndex      int16 // Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
	EndIndex        int16 // End index, equal or greater than start index.
}

func (self *MissionWritePartialList) MsgID() uint8 {
	return 38
}

func (self *MissionWritePartialList) MsgName() string {
	return "MissionWritePartialList"
}

func (self *MissionWritePartialList) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.StartIndex,
		&self.EndIndex,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *MissionWritePartialList) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.StartIndex,
		&self.EndIndex,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Message encoding a mission item. This message is emitted to announce
//                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
type MissionItem struct {
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Seq             uint16  // Sequence
	Frame           uint8   // The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
	Command         uint16  // The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
	Current         uint8   // false:0, true:1
	Autocontinue    uint8   // autocontinue to next wp
	Param1          float32 // PARAM1, see MAV_CMD enum
	Param2          float32 // PARAM2, see MAV_CMD enum
	Param3          float32 // PARAM3, see MAV_CMD enum
	Param4          float32 // PARAM4, see MAV_CMD enum
	X               float32 // PARAM5 / local: x position, global: latitude
	Y               float32 // PARAM6 / y position: global: longitude
	Z               float32 // PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
}

func (self *MissionItem) MsgID() uint8 {
	return 39
}

func (self *MissionItem) MsgName() string {
	return "MissionItem"
}

func (self *MissionItem) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Seq,
		&self.Frame,
		&self.Command,
		&self.Current,
		&self.Autocontinue,
		&self.Param1,
		&self.Param2,
		&self.Param3,
		&self.Param4,
		&self.X,
		&self.Y,
		&self.Z,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *MissionItem) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Seq,
		&self.Frame,
		&self.Command,
		&self.Current,
		&self.Autocontinue,
		&self.Param1,
		&self.Param2,
		&self.Param3,
		&self.Param4,
		&self.X,
		&self.Y,
		&self.Z,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. http://qgroundcontrol.org/mavlink/waypoint_protocol
type MissionRequest struct {
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
	Seq             uint16 // Sequence
}

func (self *MissionRequest) MsgID() uint8 {
	return 40
}

func (self *MissionRequest) MsgName() string {
	return "MissionRequest"
}

func (self *MissionRequest) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Seq,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *MissionRequest) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Seq,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
type MissionSetCurrent struct {
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
	Seq             uint16 // Sequence
}

func (self *MissionSetCurrent) MsgID() uint8 {
	return 41
}

func (self *MissionSetCurrent) MsgName() string {
	return "MissionSetCurrent"
}

func (self *MissionSetCurrent) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Seq,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *MissionSetCurrent) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Seq,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
type MissionCurrent struct {
	Seq uint16 // Sequence
}

func (self *MissionCurrent) MsgID() uint8 {
	return 42
}

func (self *MissionCurrent) MsgName() string {
	return "MissionCurrent"
}

func (self *MissionCurrent) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Seq,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *MissionCurrent) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Seq,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Request the overall list of mission items from the system/component.
type MissionRequestList struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *MissionRequestList) MsgID() uint8 {
	return 43
}

func (self *MissionRequestList) MsgName() string {
	return "MissionRequestList"
}

func (self *MissionRequestList) Pack(p *Packet) error {
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

func (self *MissionRequestList) Unpack(p *Packet) error {
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

// This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of MISSIONs.
type MissionCount struct {
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
	Count           uint16 // Number of mission items in the sequence
}

func (self *MissionCount) MsgID() uint8 {
	return 44
}

func (self *MissionCount) MsgName() string {
	return "MissionCount"
}

func (self *MissionCount) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
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

func (self *MissionCount) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Count,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Delete all mission items at once.
type MissionClearAll struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *MissionClearAll) MsgID() uint8 {
	return 45
}

func (self *MissionClearAll) MsgName() string {
	return "MissionClearAll"
}

func (self *MissionClearAll) Pack(p *Packet) error {
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

func (self *MissionClearAll) Unpack(p *Packet) error {
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

// A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next MISSION.
type MissionItemReached struct {
	Seq uint16 // Sequence
}

func (self *MissionItemReached) MsgID() uint8 {
	return 46
}

func (self *MissionItemReached) MsgName() string {
	return "MissionItemReached"
}

func (self *MissionItemReached) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Seq,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *MissionItemReached) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Seq,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Ack message during MISSION handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
type MissionAck struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	Type            uint8 // See MAV_MISSION_RESULT enum
}

func (self *MissionAck) MsgID() uint8 {
	return 47
}

func (self *MissionAck) MsgName() string {
	return "MissionAck"
}

func (self *MissionAck) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Type,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *MissionAck) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Type,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// As local waypoints exist, the global MISSION reference allows to transform between the local coordinate frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings are connected and the MAV should move from in- to outdoor.
type SetGpsGlobalOrigin struct {
	TargetSystem uint8 // System ID
	Latitude     int32 // Latitude (WGS84), in degrees * 1E7
	Longitude    int32 // Longitude (WGS84, in degrees * 1E7
	Altitude     int32 // Altitude (AMSL), in meters * 1000 (positive for up)
}

func (self *SetGpsGlobalOrigin) MsgID() uint8 {
	return 48
}

func (self *SetGpsGlobalOrigin) MsgName() string {
	return "SetGpsGlobalOrigin"
}

func (self *SetGpsGlobalOrigin) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.Latitude,
		&self.Longitude,
		&self.Altitude,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SetGpsGlobalOrigin) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.Latitude,
		&self.Longitude,
		&self.Altitude,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) position
type GpsGlobalOrigin struct {
	Latitude  int32 // Latitude (WGS84), in degrees * 1E7
	Longitude int32 // Longitude (WGS84), in degrees * 1E7
	Altitude  int32 // Altitude (AMSL), in meters * 1000 (positive for up)
}

func (self *GpsGlobalOrigin) MsgID() uint8 {
	return 49
}

func (self *GpsGlobalOrigin) MsgName() string {
	return "GpsGlobalOrigin"
}

func (self *GpsGlobalOrigin) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Latitude,
		&self.Longitude,
		&self.Altitude,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *GpsGlobalOrigin) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Latitude,
		&self.Longitude,
		&self.Altitude,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Bind a RC channel to a parameter. The parameter should change accoding to the RC channel value.
type ParamMapRc struct {
	TargetSystem            uint8    // System ID
	TargetComponent         uint8    // Component ID
	ParamId                 [16]byte // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamIndex              int16    // Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored), send -2 to disable any existing map for this rc_channel_index.
	ParameterRcChannelIndex uint8    // Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob on the RC.
	ParamValue0             float32  // Initial parameter value
	Scale                   float32  // Scale, maps the RC range [-1, 1] to a parameter value
	ParamValueMin           float32  // Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends on implementation)
	ParamValueMax           float32  // Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends on implementation)
}

func (self *ParamMapRc) MsgID() uint8 {
	return 50
}

func (self *ParamMapRc) MsgName() string {
	return "ParamMapRc"
}

func (self *ParamMapRc) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.ParamId,
		&self.ParamIndex,
		&self.ParameterRcChannelIndex,
		&self.ParamValue0,
		&self.Scale,
		&self.ParamValueMin,
		&self.ParamValueMax,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ParamMapRc) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.ParamId,
		&self.ParamIndex,
		&self.ParameterRcChannelIndex,
		&self.ParamValue0,
		&self.Scale,
		&self.ParamValueMin,
		&self.ParamValueMax,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/MISSIONs to accept and which to reject. Safety areas are often enforced by national or competition regulations.
type SafetySetAllowedArea struct {
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Frame           uint8   // Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
	P1x             float32 // x position 1 / Latitude 1
	P1y             float32 // y position 1 / Longitude 1
	P1z             float32 // z position 1 / Altitude 1
	P2x             float32 // x position 2 / Latitude 2
	P2y             float32 // y position 2 / Longitude 2
	P2z             float32 // z position 2 / Altitude 2
}

func (self *SafetySetAllowedArea) MsgID() uint8 {
	return 54
}

func (self *SafetySetAllowedArea) MsgName() string {
	return "SafetySetAllowedArea"
}

func (self *SafetySetAllowedArea) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Frame,
		&self.P1x,
		&self.P1y,
		&self.P1z,
		&self.P2x,
		&self.P2y,
		&self.P2z,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SafetySetAllowedArea) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Frame,
		&self.P1x,
		&self.P1y,
		&self.P1z,
		&self.P2x,
		&self.P2y,
		&self.P2z,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Read out the safety zone the MAV currently assumes.
type SafetyAllowedArea struct {
	Frame uint8   // Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
	P1x   float32 // x position 1 / Latitude 1
	P1y   float32 // y position 1 / Longitude 1
	P1z   float32 // z position 1 / Altitude 1
	P2x   float32 // x position 2 / Latitude 2
	P2y   float32 // y position 2 / Longitude 2
	P2z   float32 // z position 2 / Altitude 2
}

func (self *SafetyAllowedArea) MsgID() uint8 {
	return 55
}

func (self *SafetyAllowedArea) MsgName() string {
	return "SafetyAllowedArea"
}

func (self *SafetyAllowedArea) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Frame,
		&self.P1x,
		&self.P1y,
		&self.P1z,
		&self.P2x,
		&self.P2y,
		&self.P2z,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SafetyAllowedArea) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Frame,
		&self.P1x,
		&self.P1y,
		&self.P1z,
		&self.P2x,
		&self.P2y,
		&self.P2z,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
type AttitudeQuaternionCov struct {
	TimeBootMs uint32     // Timestamp (milliseconds since system boot)
	Q          [4]float32 // Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
	Rollspeed  float32    // Roll angular speed (rad/s)
	Pitchspeed float32    // Pitch angular speed (rad/s)
	Yawspeed   float32    // Yaw angular speed (rad/s)
	Covariance [9]float32 // Attitude covariance
}

func (self *AttitudeQuaternionCov) MsgID() uint8 {
	return 61
}

func (self *AttitudeQuaternionCov) MsgName() string {
	return "AttitudeQuaternionCov"
}

func (self *AttitudeQuaternionCov) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Q,
		&self.Rollspeed,
		&self.Pitchspeed,
		&self.Yawspeed,
		&self.Covariance,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *AttitudeQuaternionCov) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Q,
		&self.Rollspeed,
		&self.Pitchspeed,
		&self.Yawspeed,
		&self.Covariance,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Outputs of the APM navigation controller. The primary use of this message is to check the response and signs of the controller before actual flight and to assist with tuning controller parameters.
type NavControllerOutput struct {
	NavRoll       float32 // Current desired roll in degrees
	NavPitch      float32 // Current desired pitch in degrees
	NavBearing    int16   // Current desired heading in degrees
	TargetBearing int16   // Bearing to current MISSION/target in degrees
	WpDist        uint16  // Distance to active MISSION in meters
	AltError      float32 // Current altitude error in meters
	AspdError     float32 // Current airspeed error in meters/second
	XtrackError   float32 // Current crosstrack error on x-y plane in meters
}

func (self *NavControllerOutput) MsgID() uint8 {
	return 62
}

func (self *NavControllerOutput) MsgName() string {
	return "NavControllerOutput"
}

func (self *NavControllerOutput) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.NavRoll,
		&self.NavPitch,
		&self.NavBearing,
		&self.TargetBearing,
		&self.WpDist,
		&self.AltError,
		&self.AspdError,
		&self.XtrackError,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *NavControllerOutput) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.NavRoll,
		&self.NavPitch,
		&self.NavBearing,
		&self.TargetBearing,
		&self.WpDist,
		&self.AltError,
		&self.AspdError,
		&self.XtrackError,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE: This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset.
type GlobalPositionIntCov struct {
	TimeBootMs    uint32      // Timestamp (milliseconds since system boot)
	TimeUtc       uint64      // Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown. Commonly filled by the precision time source of a GPS receiver.
	EstimatorType uint8       // Class id of the estimator this estimate originated from.
	Lat           int32       // Latitude, expressed as degrees * 1E7
	Lon           int32       // Longitude, expressed as degrees * 1E7
	Alt           int32       // Altitude in meters, expressed as * 1000 (millimeters), above MSL
	RelativeAlt   int32       // Altitude above ground in meters, expressed as * 1000 (millimeters)
	Vx            float32     // Ground X Speed (Latitude), expressed as m/s
	Vy            float32     // Ground Y Speed (Longitude), expressed as m/s
	Vz            float32     // Ground Z Speed (Altitude), expressed as m/s
	Covariance    [36]float32 // Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
}

func (self *GlobalPositionIntCov) MsgID() uint8 {
	return 63
}

func (self *GlobalPositionIntCov) MsgName() string {
	return "GlobalPositionIntCov"
}

func (self *GlobalPositionIntCov) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.TimeUtc,
		&self.EstimatorType,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.RelativeAlt,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.Covariance,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *GlobalPositionIntCov) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.TimeUtc,
		&self.EstimatorType,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.RelativeAlt,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.Covariance,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNedCov struct {
	TimeBootMs    uint32      // Timestamp (milliseconds since system boot). 0 for system without monotonic timestamp
	TimeUtc       uint64      // Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown. Commonly filled by the precision time source of a GPS receiver.
	EstimatorType uint8       // Class id of the estimator this estimate originated from.
	X             float32     // X Position
	Y             float32     // Y Position
	Z             float32     // Z Position
	Vx            float32     // X Speed (m/s)
	Vy            float32     // Y Speed (m/s)
	Vz            float32     // Z Speed (m/s)
	Ax            float32     // X Acceleration (m/s^2)
	Ay            float32     // Y Acceleration (m/s^2)
	Az            float32     // Z Acceleration (m/s^2)
	Covariance    [45]float32 // Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are the second row, etc.)
}

func (self *LocalPositionNedCov) MsgID() uint8 {
	return 64
}

func (self *LocalPositionNedCov) MsgName() string {
	return "LocalPositionNedCov"
}

func (self *LocalPositionNedCov) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.TimeUtc,
		&self.EstimatorType,
		&self.X,
		&self.Y,
		&self.Z,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.Ax,
		&self.Ay,
		&self.Az,
		&self.Covariance,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *LocalPositionNedCov) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.TimeUtc,
		&self.EstimatorType,
		&self.X,
		&self.Y,
		&self.Z,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.Ax,
		&self.Ay,
		&self.Az,
		&self.Covariance,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type RcChannels struct {
	TimeBootMs uint32 // Timestamp (milliseconds since system boot)
	Chancount  uint8  // Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.
	Chan1Raw   uint16 // RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan2Raw   uint16 // RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan3Raw   uint16 // RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan4Raw   uint16 // RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan5Raw   uint16 // RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan6Raw   uint16 // RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan7Raw   uint16 // RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan8Raw   uint16 // RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan9Raw   uint16 // RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan10Raw  uint16 // RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan11Raw  uint16 // RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan12Raw  uint16 // RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan13Raw  uint16 // RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan14Raw  uint16 // RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan15Raw  uint16 // RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan16Raw  uint16 // RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan17Raw  uint16 // RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan18Raw  uint16 // RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Rssi       uint8  // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
}

func (self *RcChannels) MsgID() uint8 {
	return 65
}

func (self *RcChannels) MsgName() string {
	return "RcChannels"
}

func (self *RcChannels) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Chancount,
		&self.Chan1Raw,
		&self.Chan2Raw,
		&self.Chan3Raw,
		&self.Chan4Raw,
		&self.Chan5Raw,
		&self.Chan6Raw,
		&self.Chan7Raw,
		&self.Chan8Raw,
		&self.Chan9Raw,
		&self.Chan10Raw,
		&self.Chan11Raw,
		&self.Chan12Raw,
		&self.Chan13Raw,
		&self.Chan14Raw,
		&self.Chan15Raw,
		&self.Chan16Raw,
		&self.Chan17Raw,
		&self.Chan18Raw,
		&self.Rssi,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *RcChannels) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Chancount,
		&self.Chan1Raw,
		&self.Chan2Raw,
		&self.Chan3Raw,
		&self.Chan4Raw,
		&self.Chan5Raw,
		&self.Chan6Raw,
		&self.Chan7Raw,
		&self.Chan8Raw,
		&self.Chan9Raw,
		&self.Chan10Raw,
		&self.Chan11Raw,
		&self.Chan12Raw,
		&self.Chan13Raw,
		&self.Chan14Raw,
		&self.Chan15Raw,
		&self.Chan16Raw,
		&self.Chan17Raw,
		&self.Chan18Raw,
		&self.Rssi,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

//
type RequestDataStream struct {
	TargetSystem    uint8  // The target requested to send the message stream.
	TargetComponent uint8  // The target requested to send the message stream.
	ReqStreamId     uint8  // The ID of the requested data stream
	ReqMessageRate  uint16 // The requested interval between two messages of this type
	StartStop       uint8  // 1 to start sending, 0 to stop sending.
}

func (self *RequestDataStream) MsgID() uint8 {
	return 66
}

func (self *RequestDataStream) MsgName() string {
	return "RequestDataStream"
}

func (self *RequestDataStream) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.ReqStreamId,
		&self.ReqMessageRate,
		&self.StartStop,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *RequestDataStream) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.ReqStreamId,
		&self.ReqMessageRate,
		&self.StartStop,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

//
type DataStream struct {
	StreamId    uint8  // The ID of the requested data stream
	MessageRate uint16 // The requested interval between two messages of this type
	OnOff       uint8  // 1 stream is enabled, 0 stream is stopped.
}

func (self *DataStream) MsgID() uint8 {
	return 67
}

func (self *DataStream) MsgName() string {
	return "DataStream"
}

func (self *DataStream) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.StreamId,
		&self.MessageRate,
		&self.OnOff,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *DataStream) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.StreamId,
		&self.MessageRate,
		&self.OnOff,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as boolean values of their
type ManualControl struct {
	Target  uint8  // The system to be controlled.
	X       int16  // X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
	Y       int16  // Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
	Z       int16  // Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle.
	R       int16  // R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
	Buttons uint16 // A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
}

func (self *ManualControl) MsgID() uint8 {
	return 69
}

func (self *ManualControl) MsgName() string {
	return "ManualControl"
}

func (self *ManualControl) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Target,
		&self.X,
		&self.Y,
		&self.Z,
		&self.R,
		&self.Buttons,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ManualControl) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Target,
		&self.X,
		&self.Y,
		&self.Z,
		&self.R,
		&self.Buttons,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type RcChannelsOverride struct {
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
	Chan1Raw        uint16 // RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan2Raw        uint16 // RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan3Raw        uint16 // RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan4Raw        uint16 // RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan5Raw        uint16 // RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan6Raw        uint16 // RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan7Raw        uint16 // RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan8Raw        uint16 // RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
}

func (self *RcChannelsOverride) MsgID() uint8 {
	return 70
}

func (self *RcChannelsOverride) MsgName() string {
	return "RcChannelsOverride"
}

func (self *RcChannelsOverride) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Chan1Raw,
		&self.Chan2Raw,
		&self.Chan3Raw,
		&self.Chan4Raw,
		&self.Chan5Raw,
		&self.Chan6Raw,
		&self.Chan7Raw,
		&self.Chan8Raw,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *RcChannelsOverride) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Chan1Raw,
		&self.Chan2Raw,
		&self.Chan3Raw,
		&self.Chan4Raw,
		&self.Chan5Raw,
		&self.Chan6Raw,
		&self.Chan7Raw,
		&self.Chan8Raw,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Message encoding a mission item. This message is emitted to announce
//                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See alsohttp://qgroundcontrol.org/mavlink/waypoint_protocol.
type MissionItemInt struct {
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Seq             uint16  // Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).
	Frame           uint8   // The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
	Command         uint16  // The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
	Current         uint8   // false:0, true:1
	Autocontinue    uint8   // autocontinue to next wp
	Param1          float32 // PARAM1, see MAV_CMD enum
	Param2          float32 // PARAM2, see MAV_CMD enum
	Param3          float32 // PARAM3, see MAV_CMD enum
	Param4          float32 // PARAM4, see MAV_CMD enum
	X               int32   // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
	Y               int32   // PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
	Z               float32 // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
}

func (self *MissionItemInt) MsgID() uint8 {
	return 73
}

func (self *MissionItemInt) MsgName() string {
	return "MissionItemInt"
}

func (self *MissionItemInt) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Seq,
		&self.Frame,
		&self.Command,
		&self.Current,
		&self.Autocontinue,
		&self.Param1,
		&self.Param2,
		&self.Param3,
		&self.Param4,
		&self.X,
		&self.Y,
		&self.Z,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *MissionItemInt) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Seq,
		&self.Frame,
		&self.Command,
		&self.Current,
		&self.Autocontinue,
		&self.Param1,
		&self.Param2,
		&self.Param3,
		&self.Param4,
		&self.X,
		&self.Y,
		&self.Z,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Metrics typically displayed on a HUD for fixed wing aircraft
type VfrHud struct {
	Airspeed    float32 // Current airspeed in m/s
	Groundspeed float32 // Current ground speed in m/s
	Heading     int16   // Current heading in degrees, in compass units (0..360, 0=north)
	Throttle    uint16  // Current throttle setting in integer percent, 0 to 100
	Alt         float32 // Current altitude (MSL), in meters
	Climb       float32 // Current climb rate in meters/second
}

func (self *VfrHud) MsgID() uint8 {
	return 74
}

func (self *VfrHud) MsgName() string {
	return "VfrHud"
}

func (self *VfrHud) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Airspeed,
		&self.Groundspeed,
		&self.Heading,
		&self.Throttle,
		&self.Alt,
		&self.Climb,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *VfrHud) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Airspeed,
		&self.Groundspeed,
		&self.Heading,
		&self.Throttle,
		&self.Alt,
		&self.Climb,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value.
type CommandInt struct {
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Frame           uint8   // The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
	Command         uint16  // The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
	Current         uint8   // false:0, true:1
	Autocontinue    uint8   // autocontinue to next wp
	Param1          float32 // PARAM1, see MAV_CMD enum
	Param2          float32 // PARAM2, see MAV_CMD enum
	Param3          float32 // PARAM3, see MAV_CMD enum
	Param4          float32 // PARAM4, see MAV_CMD enum
	X               int32   // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
	Y               int32   // PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
	Z               float32 // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
}

func (self *CommandInt) MsgID() uint8 {
	return 75
}

func (self *CommandInt) MsgName() string {
	return "CommandInt"
}

func (self *CommandInt) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Frame,
		&self.Command,
		&self.Current,
		&self.Autocontinue,
		&self.Param1,
		&self.Param2,
		&self.Param3,
		&self.Param4,
		&self.X,
		&self.Y,
		&self.Z,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *CommandInt) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Frame,
		&self.Command,
		&self.Current,
		&self.Autocontinue,
		&self.Param1,
		&self.Param2,
		&self.Param3,
		&self.Param4,
		&self.X,
		&self.Y,
		&self.Z,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Send a command with up to seven parameters to the MAV
type CommandLong struct {
	TargetSystem    uint8   // System which should execute the command
	TargetComponent uint8   // Component which should execute the command, 0 for all components
	Command         uint16  // Command ID, as defined by MAV_CMD enum.
	Confirmation    uint8   // 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
	Param1          float32 // Parameter 1, as defined by MAV_CMD enum.
	Param2          float32 // Parameter 2, as defined by MAV_CMD enum.
	Param3          float32 // Parameter 3, as defined by MAV_CMD enum.
	Param4          float32 // Parameter 4, as defined by MAV_CMD enum.
	Param5          float32 // Parameter 5, as defined by MAV_CMD enum.
	Param6          float32 // Parameter 6, as defined by MAV_CMD enum.
	Param7          float32 // Parameter 7, as defined by MAV_CMD enum.
}

func (self *CommandLong) MsgID() uint8 {
	return 76
}

func (self *CommandLong) MsgName() string {
	return "CommandLong"
}

func (self *CommandLong) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Command,
		&self.Confirmation,
		&self.Param1,
		&self.Param2,
		&self.Param3,
		&self.Param4,
		&self.Param5,
		&self.Param6,
		&self.Param7,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *CommandLong) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Command,
		&self.Confirmation,
		&self.Param1,
		&self.Param2,
		&self.Param3,
		&self.Param4,
		&self.Param5,
		&self.Param6,
		&self.Param7,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Report status of a command. Includes feedback wether the command was executed.
type CommandAck struct {
	Command uint16 // Command ID, as defined by MAV_CMD enum.
	Result  uint8  // See MAV_RESULT enum
}

func (self *CommandAck) MsgID() uint8 {
	return 77
}

func (self *CommandAck) MsgName() string {
	return "CommandAck"
}

func (self *CommandAck) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Command,
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

func (self *CommandAck) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Command,
		&self.Result,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Setpoint in roll, pitch, yaw and thrust from the operator
type ManualSetpoint struct {
	TimeBootMs           uint32  // Timestamp in milliseconds since system boot
	Roll                 float32 // Desired roll rate in radians per second
	Pitch                float32 // Desired pitch rate in radians per second
	Yaw                  float32 // Desired yaw rate in radians per second
	Thrust               float32 // Collective thrust, normalized to 0 .. 1
	ModeSwitch           uint8   // Flight mode switch position, 0.. 255
	ManualOverrideSwitch uint8   // Override mode switch position, 0.. 255
}

func (self *ManualSetpoint) MsgID() uint8 {
	return 81
}

func (self *ManualSetpoint) MsgName() string {
	return "ManualSetpoint"
}

func (self *ManualSetpoint) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.Thrust,
		&self.ModeSwitch,
		&self.ManualOverrideSwitch,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ManualSetpoint) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.Thrust,
		&self.ModeSwitch,
		&self.ManualOverrideSwitch,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Set the vehicle attitude and body angular rates.
type SetAttitudeTarget struct {
	TimeBootMs      uint32     // Timestamp in milliseconds since system boot
	TargetSystem    uint8      // System ID
	TargetComponent uint8      // Component ID
	TypeMask        uint8      // Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude
	Q               [4]float32 // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	BodyRollRate    float32    // Body roll rate in radians per second
	BodyPitchRate   float32    // Body roll rate in radians per second
	BodyYawRate     float32    // Body roll rate in radians per second
	Thrust          float32    // Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
}

func (self *SetAttitudeTarget) MsgID() uint8 {
	return 82
}

func (self *SetAttitudeTarget) MsgName() string {
	return "SetAttitudeTarget"
}

func (self *SetAttitudeTarget) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.TypeMask,
		&self.Q,
		&self.BodyRollRate,
		&self.BodyPitchRate,
		&self.BodyYawRate,
		&self.Thrust,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SetAttitudeTarget) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.TypeMask,
		&self.Q,
		&self.BodyRollRate,
		&self.BodyPitchRate,
		&self.BodyYawRate,
		&self.Thrust,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Set the vehicle attitude and body angular rates.
type AttitudeTarget struct {
	TimeBootMs    uint32     // Timestamp in milliseconds since system boot
	TypeMask      uint8      // Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitude
	Q             [4]float32 // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	BodyRollRate  float32    // Body roll rate in radians per second
	BodyPitchRate float32    // Body roll rate in radians per second
	BodyYawRate   float32    // Body roll rate in radians per second
	Thrust        float32    // Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
}

func (self *AttitudeTarget) MsgID() uint8 {
	return 83
}

func (self *AttitudeTarget) MsgName() string {
	return "AttitudeTarget"
}

func (self *AttitudeTarget) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.TypeMask,
		&self.Q,
		&self.BodyRollRate,
		&self.BodyPitchRate,
		&self.BodyYawRate,
		&self.Thrust,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *AttitudeTarget) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.TypeMask,
		&self.Q,
		&self.BodyRollRate,
		&self.BodyPitchRate,
		&self.BodyYawRate,
		&self.Thrust,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Set vehicle position, velocity and acceleration setpoint in local frame.
type SetPositionTargetLocalNed struct {
	TimeBootMs      uint32  // Timestamp in milliseconds since system boot
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	CoordinateFrame uint8   // Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
	TypeMask        uint16  // Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
	X               float32 // X Position in NED frame in meters
	Y               float32 // Y Position in NED frame in meters
	Z               float32 // Z Position in NED frame in meters (note, altitude is negative in NED)
	Vx              float32 // X velocity in NED frame in meter / s
	Vy              float32 // Y velocity in NED frame in meter / s
	Vz              float32 // Z velocity in NED frame in meter / s
	Afx             float32 // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32 // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32 // yaw setpoint in rad
	YawRate         float32 // yaw rate setpoint in rad/s
}

func (self *SetPositionTargetLocalNed) MsgID() uint8 {
	return 84
}

func (self *SetPositionTargetLocalNed) MsgName() string {
	return "SetPositionTargetLocalNed"
}

func (self *SetPositionTargetLocalNed) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.CoordinateFrame,
		&self.TypeMask,
		&self.X,
		&self.Y,
		&self.Z,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.Afx,
		&self.Afy,
		&self.Afz,
		&self.Yaw,
		&self.YawRate,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SetPositionTargetLocalNed) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.CoordinateFrame,
		&self.TypeMask,
		&self.X,
		&self.Y,
		&self.Z,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.Afx,
		&self.Afy,
		&self.Afz,
		&self.Yaw,
		&self.YawRate,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Set vehicle position, velocity and acceleration setpoint in local frame.
type PositionTargetLocalNed struct {
	TimeBootMs      uint32  // Timestamp in milliseconds since system boot
	CoordinateFrame uint8   // Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
	TypeMask        uint16  // Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
	X               float32 // X Position in NED frame in meters
	Y               float32 // Y Position in NED frame in meters
	Z               float32 // Z Position in NED frame in meters (note, altitude is negative in NED)
	Vx              float32 // X velocity in NED frame in meter / s
	Vy              float32 // Y velocity in NED frame in meter / s
	Vz              float32 // Z velocity in NED frame in meter / s
	Afx             float32 // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32 // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32 // yaw setpoint in rad
	YawRate         float32 // yaw rate setpoint in rad/s
}

func (self *PositionTargetLocalNed) MsgID() uint8 {
	return 85
}

func (self *PositionTargetLocalNed) MsgName() string {
	return "PositionTargetLocalNed"
}

func (self *PositionTargetLocalNed) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.CoordinateFrame,
		&self.TypeMask,
		&self.X,
		&self.Y,
		&self.Z,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.Afx,
		&self.Afy,
		&self.Afz,
		&self.Yaw,
		&self.YawRate,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *PositionTargetLocalNed) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.CoordinateFrame,
		&self.TypeMask,
		&self.X,
		&self.Y,
		&self.Z,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.Afx,
		&self.Afy,
		&self.Afz,
		&self.Yaw,
		&self.YawRate,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Set vehicle position, velocity and acceleration setpoint in the WGS84 coordinate system.
type SetPositionTargetGlobalInt struct {
	TimeBootMs      uint32  // Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	CoordinateFrame uint8   // Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
	TypeMask        uint16  // Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
	LatInt          int32   // X Position in WGS84 frame in 1e7 * meters
	LonInt          int32   // Y Position in WGS84 frame in 1e7 * meters
	Alt             float32 // Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
	Vx              float32 // X velocity in NED frame in meter / s
	Vy              float32 // Y velocity in NED frame in meter / s
	Vz              float32 // Z velocity in NED frame in meter / s
	Afx             float32 // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32 // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32 // yaw setpoint in rad
	YawRate         float32 // yaw rate setpoint in rad/s
}

func (self *SetPositionTargetGlobalInt) MsgID() uint8 {
	return 86
}

func (self *SetPositionTargetGlobalInt) MsgName() string {
	return "SetPositionTargetGlobalInt"
}

func (self *SetPositionTargetGlobalInt) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.CoordinateFrame,
		&self.TypeMask,
		&self.LatInt,
		&self.LonInt,
		&self.Alt,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.Afx,
		&self.Afy,
		&self.Afz,
		&self.Yaw,
		&self.YawRate,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SetPositionTargetGlobalInt) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.CoordinateFrame,
		&self.TypeMask,
		&self.LatInt,
		&self.LonInt,
		&self.Alt,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.Afx,
		&self.Afy,
		&self.Afz,
		&self.Yaw,
		&self.YawRate,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Set vehicle position, velocity and acceleration setpoint in the WGS84 coordinate system.
type PositionTargetGlobalInt struct {
	TimeBootMs      uint32  // Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
	CoordinateFrame uint8   // Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
	TypeMask        uint16  // Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
	LatInt          int32   // X Position in WGS84 frame in 1e7 * meters
	LonInt          int32   // Y Position in WGS84 frame in 1e7 * meters
	Alt             float32 // Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
	Vx              float32 // X velocity in NED frame in meter / s
	Vy              float32 // Y velocity in NED frame in meter / s
	Vz              float32 // Z velocity in NED frame in meter / s
	Afx             float32 // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32 // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32 // yaw setpoint in rad
	YawRate         float32 // yaw rate setpoint in rad/s
}

func (self *PositionTargetGlobalInt) MsgID() uint8 {
	return 87
}

func (self *PositionTargetGlobalInt) MsgName() string {
	return "PositionTargetGlobalInt"
}

func (self *PositionTargetGlobalInt) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.CoordinateFrame,
		&self.TypeMask,
		&self.LatInt,
		&self.LonInt,
		&self.Alt,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.Afx,
		&self.Afy,
		&self.Afz,
		&self.Yaw,
		&self.YawRate,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *PositionTargetGlobalInt) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.CoordinateFrame,
		&self.TypeMask,
		&self.LatInt,
		&self.LonInt,
		&self.Alt,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.Afx,
		&self.Afy,
		&self.Afz,
		&self.Yaw,
		&self.YawRate,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNedSystemGlobalOffset struct {
	TimeBootMs uint32  // Timestamp (milliseconds since system boot)
	X          float32 // X Position
	Y          float32 // Y Position
	Z          float32 // Z Position
	Roll       float32 // Roll
	Pitch      float32 // Pitch
	Yaw        float32 // Yaw
}

func (self *LocalPositionNedSystemGlobalOffset) MsgID() uint8 {
	return 89
}

func (self *LocalPositionNedSystemGlobalOffset) MsgName() string {
	return "LocalPositionNedSystemGlobalOffset"
}

func (self *LocalPositionNedSystemGlobalOffset) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.X,
		&self.Y,
		&self.Z,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *LocalPositionNedSystemGlobalOffset) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.X,
		&self.Y,
		&self.Z,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware in the loop simulations.
type HilState struct {
	TimeUsec   uint64  // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Roll       float32 // Roll angle (rad)
	Pitch      float32 // Pitch angle (rad)
	Yaw        float32 // Yaw angle (rad)
	Rollspeed  float32 // Body frame roll / phi angular speed (rad/s)
	Pitchspeed float32 // Body frame pitch / theta angular speed (rad/s)
	Yawspeed   float32 // Body frame yaw / psi angular speed (rad/s)
	Lat        int32   // Latitude, expressed as * 1E7
	Lon        int32   // Longitude, expressed as * 1E7
	Alt        int32   // Altitude in meters, expressed as * 1000 (millimeters)
	Vx         int16   // Ground X Speed (Latitude), expressed as m/s * 100
	Vy         int16   // Ground Y Speed (Longitude), expressed as m/s * 100
	Vz         int16   // Ground Z Speed (Altitude), expressed as m/s * 100
	Xacc       int16   // X acceleration (mg)
	Yacc       int16   // Y acceleration (mg)
	Zacc       int16   // Z acceleration (mg)
}

func (self *HilState) MsgID() uint8 {
	return 90
}

func (self *HilState) MsgName() string {
	return "HilState"
}

func (self *HilState) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.Rollspeed,
		&self.Pitchspeed,
		&self.Yawspeed,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *HilState) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
		&self.Rollspeed,
		&self.Pitchspeed,
		&self.Yawspeed,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Sent from autopilot to simulation. Hardware in the loop control outputs
type HilControls struct {
	TimeUsec      uint64  // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	RollAilerons  float32 // Control output -1 .. 1
	PitchElevator float32 // Control output -1 .. 1
	YawRudder     float32 // Control output -1 .. 1
	Throttle      float32 // Throttle 0 .. 1
	Aux1          float32 // Aux 1, -1 .. 1
	Aux2          float32 // Aux 2, -1 .. 1
	Aux3          float32 // Aux 3, -1 .. 1
	Aux4          float32 // Aux 4, -1 .. 1
	Mode          uint8   // System mode (MAV_MODE)
	NavMode       uint8   // Navigation mode (MAV_NAV_MODE)
}

func (self *HilControls) MsgID() uint8 {
	return 91
}

func (self *HilControls) MsgName() string {
	return "HilControls"
}

func (self *HilControls) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.RollAilerons,
		&self.PitchElevator,
		&self.YawRudder,
		&self.Throttle,
		&self.Aux1,
		&self.Aux2,
		&self.Aux3,
		&self.Aux4,
		&self.Mode,
		&self.NavMode,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *HilControls) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.RollAilerons,
		&self.PitchElevator,
		&self.YawRudder,
		&self.Throttle,
		&self.Aux1,
		&self.Aux2,
		&self.Aux3,
		&self.Aux4,
		&self.Mode,
		&self.NavMode,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type HilRcInputsRaw struct {
	TimeUsec  uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Chan1Raw  uint16 // RC channel 1 value, in microseconds
	Chan2Raw  uint16 // RC channel 2 value, in microseconds
	Chan3Raw  uint16 // RC channel 3 value, in microseconds
	Chan4Raw  uint16 // RC channel 4 value, in microseconds
	Chan5Raw  uint16 // RC channel 5 value, in microseconds
	Chan6Raw  uint16 // RC channel 6 value, in microseconds
	Chan7Raw  uint16 // RC channel 7 value, in microseconds
	Chan8Raw  uint16 // RC channel 8 value, in microseconds
	Chan9Raw  uint16 // RC channel 9 value, in microseconds
	Chan10Raw uint16 // RC channel 10 value, in microseconds
	Chan11Raw uint16 // RC channel 11 value, in microseconds
	Chan12Raw uint16 // RC channel 12 value, in microseconds
	Rssi      uint8  // Receive signal strength indicator, 0: 0%, 255: 100%
}

func (self *HilRcInputsRaw) MsgID() uint8 {
	return 92
}

func (self *HilRcInputsRaw) MsgName() string {
	return "HilRcInputsRaw"
}

func (self *HilRcInputsRaw) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Chan1Raw,
		&self.Chan2Raw,
		&self.Chan3Raw,
		&self.Chan4Raw,
		&self.Chan5Raw,
		&self.Chan6Raw,
		&self.Chan7Raw,
		&self.Chan8Raw,
		&self.Chan9Raw,
		&self.Chan10Raw,
		&self.Chan11Raw,
		&self.Chan12Raw,
		&self.Rssi,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *HilRcInputsRaw) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Chan1Raw,
		&self.Chan2Raw,
		&self.Chan3Raw,
		&self.Chan4Raw,
		&self.Chan5Raw,
		&self.Chan6Raw,
		&self.Chan7Raw,
		&self.Chan8Raw,
		&self.Chan9Raw,
		&self.Chan10Raw,
		&self.Chan11Raw,
		&self.Chan12Raw,
		&self.Rssi,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Optical flow from a flow sensor (e.g. optical mouse sensor)
type OpticalFlow struct {
	TimeUsec       uint64  // Timestamp (UNIX)
	SensorId       uint8   // Sensor ID
	FlowX          int16   // Flow in pixels * 10 in x-sensor direction (dezi-pixels)
	FlowY          int16   // Flow in pixels * 10 in y-sensor direction (dezi-pixels)
	FlowCompMX     float32 // Flow in meters in x-sensor direction, angular-speed compensated
	FlowCompMY     float32 // Flow in meters in y-sensor direction, angular-speed compensated
	Quality        uint8   // Optical flow quality / confidence. 0: bad, 255: maximum quality
	GroundDistance float32 // Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
}

func (self *OpticalFlow) MsgID() uint8 {
	return 100
}

func (self *OpticalFlow) MsgName() string {
	return "OpticalFlow"
}

func (self *OpticalFlow) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.SensorId,
		&self.FlowX,
		&self.FlowY,
		&self.FlowCompMX,
		&self.FlowCompMY,
		&self.Quality,
		&self.GroundDistance,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *OpticalFlow) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.SensorId,
		&self.FlowX,
		&self.FlowY,
		&self.FlowCompMX,
		&self.FlowCompMY,
		&self.Quality,
		&self.GroundDistance,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

//
type GlobalVisionPositionEstimate struct {
	Usec  uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	X     float32 // Global X position
	Y     float32 // Global Y position
	Z     float32 // Global Z position
	Roll  float32 // Roll angle in rad
	Pitch float32 // Pitch angle in rad
	Yaw   float32 // Yaw angle in rad
}

func (self *GlobalVisionPositionEstimate) MsgID() uint8 {
	return 101
}

func (self *GlobalVisionPositionEstimate) MsgName() string {
	return "GlobalVisionPositionEstimate"
}

func (self *GlobalVisionPositionEstimate) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Usec,
		&self.X,
		&self.Y,
		&self.Z,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *GlobalVisionPositionEstimate) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Usec,
		&self.X,
		&self.Y,
		&self.Z,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

//
type VisionPositionEstimate struct {
	Usec  uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	X     float32 // Global X position
	Y     float32 // Global Y position
	Z     float32 // Global Z position
	Roll  float32 // Roll angle in rad
	Pitch float32 // Pitch angle in rad
	Yaw   float32 // Yaw angle in rad
}

func (self *VisionPositionEstimate) MsgID() uint8 {
	return 102
}

func (self *VisionPositionEstimate) MsgName() string {
	return "VisionPositionEstimate"
}

func (self *VisionPositionEstimate) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Usec,
		&self.X,
		&self.Y,
		&self.Z,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *VisionPositionEstimate) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Usec,
		&self.X,
		&self.Y,
		&self.Z,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

//
type VisionSpeedEstimate struct {
	Usec uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	X    float32 // Global X speed
	Y    float32 // Global Y speed
	Z    float32 // Global Z speed
}

func (self *VisionSpeedEstimate) MsgID() uint8 {
	return 103
}

func (self *VisionSpeedEstimate) MsgName() string {
	return "VisionSpeedEstimate"
}

func (self *VisionSpeedEstimate) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Usec,
		&self.X,
		&self.Y,
		&self.Z,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *VisionSpeedEstimate) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Usec,
		&self.X,
		&self.Y,
		&self.Z,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

//
type ViconPositionEstimate struct {
	Usec  uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	X     float32 // Global X position
	Y     float32 // Global Y position
	Z     float32 // Global Z position
	Roll  float32 // Roll angle in rad
	Pitch float32 // Pitch angle in rad
	Yaw   float32 // Yaw angle in rad
}

func (self *ViconPositionEstimate) MsgID() uint8 {
	return 104
}

func (self *ViconPositionEstimate) MsgName() string {
	return "ViconPositionEstimate"
}

func (self *ViconPositionEstimate) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Usec,
		&self.X,
		&self.Y,
		&self.Z,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ViconPositionEstimate) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Usec,
		&self.X,
		&self.Y,
		&self.Z,
		&self.Roll,
		&self.Pitch,
		&self.Yaw,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The IMU readings in SI units in NED body frame
type HighresImu struct {
	TimeUsec      uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	Xacc          float32 // X acceleration (m/s^2)
	Yacc          float32 // Y acceleration (m/s^2)
	Zacc          float32 // Z acceleration (m/s^2)
	Xgyro         float32 // Angular speed around X axis (rad / sec)
	Ygyro         float32 // Angular speed around Y axis (rad / sec)
	Zgyro         float32 // Angular speed around Z axis (rad / sec)
	Xmag          float32 // X Magnetic field (Gauss)
	Ymag          float32 // Y Magnetic field (Gauss)
	Zmag          float32 // Z Magnetic field (Gauss)
	AbsPressure   float32 // Absolute pressure in millibar
	DiffPressure  float32 // Differential pressure in millibar
	PressureAlt   float32 // Altitude calculated from pressure
	Temperature   float32 // Temperature in degrees celsius
	FieldsUpdated uint16  // Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
}

func (self *HighresImu) MsgID() uint8 {
	return 105
}

func (self *HighresImu) MsgName() string {
	return "HighresImu"
}

func (self *HighresImu) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
		&self.Xgyro,
		&self.Ygyro,
		&self.Zgyro,
		&self.Xmag,
		&self.Ymag,
		&self.Zmag,
		&self.AbsPressure,
		&self.DiffPressure,
		&self.PressureAlt,
		&self.Temperature,
		&self.FieldsUpdated,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *HighresImu) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
		&self.Xgyro,
		&self.Ygyro,
		&self.Zgyro,
		&self.Xmag,
		&self.Ymag,
		&self.Zmag,
		&self.AbsPressure,
		&self.DiffPressure,
		&self.PressureAlt,
		&self.Temperature,
		&self.FieldsUpdated,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
type OpticalFlowRad struct {
	TimeUsec            uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	SensorId            uint8   // Sensor ID
	IntegrationTimeUs   uint32  // Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
	IntegratedX         float32 // Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
	IntegratedY         float32 // Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
	IntegratedXgyro     float32 // RH rotation around X axis (rad)
	IntegratedYgyro     float32 // RH rotation around Y axis (rad)
	IntegratedZgyro     float32 // RH rotation around Z axis (rad)
	Temperature         int16   // Temperature * 100 in centi-degrees Celsius
	Quality             uint8   // Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
	TimeDeltaDistanceUs uint32  // Time in microseconds since the distance was sampled.
	Distance            float32 // Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
}

func (self *OpticalFlowRad) MsgID() uint8 {
	return 106
}

func (self *OpticalFlowRad) MsgName() string {
	return "OpticalFlowRad"
}

func (self *OpticalFlowRad) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.SensorId,
		&self.IntegrationTimeUs,
		&self.IntegratedX,
		&self.IntegratedY,
		&self.IntegratedXgyro,
		&self.IntegratedYgyro,
		&self.IntegratedZgyro,
		&self.Temperature,
		&self.Quality,
		&self.TimeDeltaDistanceUs,
		&self.Distance,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *OpticalFlowRad) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.SensorId,
		&self.IntegrationTimeUs,
		&self.IntegratedX,
		&self.IntegratedY,
		&self.IntegratedXgyro,
		&self.IntegratedYgyro,
		&self.IntegratedZgyro,
		&self.Temperature,
		&self.Quality,
		&self.TimeDeltaDistanceUs,
		&self.Distance,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The IMU readings in SI units in NED body frame
type HilSensor struct {
	TimeUsec      uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	Xacc          float32 // X acceleration (m/s^2)
	Yacc          float32 // Y acceleration (m/s^2)
	Zacc          float32 // Z acceleration (m/s^2)
	Xgyro         float32 // Angular speed around X axis in body frame (rad / sec)
	Ygyro         float32 // Angular speed around Y axis in body frame (rad / sec)
	Zgyro         float32 // Angular speed around Z axis in body frame (rad / sec)
	Xmag          float32 // X Magnetic field (Gauss)
	Ymag          float32 // Y Magnetic field (Gauss)
	Zmag          float32 // Z Magnetic field (Gauss)
	AbsPressure   float32 // Absolute pressure in millibar
	DiffPressure  float32 // Differential pressure (airspeed) in millibar
	PressureAlt   float32 // Altitude calculated from pressure
	Temperature   float32 // Temperature in degrees celsius
	FieldsUpdated uint32  // Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
}

func (self *HilSensor) MsgID() uint8 {
	return 107
}

func (self *HilSensor) MsgName() string {
	return "HilSensor"
}

func (self *HilSensor) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
		&self.Xgyro,
		&self.Ygyro,
		&self.Zgyro,
		&self.Xmag,
		&self.Ymag,
		&self.Zmag,
		&self.AbsPressure,
		&self.DiffPressure,
		&self.PressureAlt,
		&self.Temperature,
		&self.FieldsUpdated,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *HilSensor) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
		&self.Xgyro,
		&self.Ygyro,
		&self.Zgyro,
		&self.Xmag,
		&self.Ymag,
		&self.Zmag,
		&self.AbsPressure,
		&self.DiffPressure,
		&self.PressureAlt,
		&self.Temperature,
		&self.FieldsUpdated,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Status of simulation environment, if used
type SimState struct {
	Q1         float32 // True attitude quaternion component 1, w (1 in null-rotation)
	Q2         float32 // True attitude quaternion component 2, x (0 in null-rotation)
	Q3         float32 // True attitude quaternion component 3, y (0 in null-rotation)
	Q4         float32 // True attitude quaternion component 4, z (0 in null-rotation)
	Roll       float32 // Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
	Pitch      float32 // Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
	Yaw        float32 // Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
	Xacc       float32 // X acceleration m/s/s
	Yacc       float32 // Y acceleration m/s/s
	Zacc       float32 // Z acceleration m/s/s
	Xgyro      float32 // Angular speed around X axis rad/s
	Ygyro      float32 // Angular speed around Y axis rad/s
	Zgyro      float32 // Angular speed around Z axis rad/s
	Lat        float32 // Latitude in degrees
	Lon        float32 // Longitude in degrees
	Alt        float32 // Altitude in meters
	StdDevHorz float32 // Horizontal position standard deviation
	StdDevVert float32 // Vertical position standard deviation
	Vn         float32 // True velocity in m/s in NORTH direction in earth-fixed NED frame
	Ve         float32 // True velocity in m/s in EAST direction in earth-fixed NED frame
	Vd         float32 // True velocity in m/s in DOWN direction in earth-fixed NED frame
}

func (self *SimState) MsgID() uint8 {
	return 108
}

func (self *SimState) MsgName() string {
	return "SimState"
}

func (self *SimState) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Q1,
		&self.Q2,
		&self.Q3,
		&self.Q4,
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
		&self.Lon,
		&self.Alt,
		&self.StdDevHorz,
		&self.StdDevVert,
		&self.Vn,
		&self.Ve,
		&self.Vd,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SimState) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Q1,
		&self.Q2,
		&self.Q3,
		&self.Q4,
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
		&self.Lon,
		&self.Alt,
		&self.StdDevHorz,
		&self.StdDevVert,
		&self.Vn,
		&self.Ve,
		&self.Vd,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Status generated by radio and injected into MAVLink stream.
type RadioStatus struct {
	Rssi     uint8  // Local signal strength
	Remrssi  uint8  // Remote signal strength
	Txbuf    uint8  // Remaining free buffer space in percent.
	Noise    uint8  // Background noise level
	Remnoise uint8  // Remote background noise level
	Rxerrors uint16 // Receive errors
	Fixed    uint16 // Count of error corrected packets
}

func (self *RadioStatus) MsgID() uint8 {
	return 109
}

func (self *RadioStatus) MsgName() string {
	return "RadioStatus"
}

func (self *RadioStatus) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Rssi,
		&self.Remrssi,
		&self.Txbuf,
		&self.Noise,
		&self.Remnoise,
		&self.Rxerrors,
		&self.Fixed,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *RadioStatus) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Rssi,
		&self.Remrssi,
		&self.Txbuf,
		&self.Noise,
		&self.Remnoise,
		&self.Rxerrors,
		&self.Fixed,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// File transfer message
type FileTransferProtocol struct {
	TargetNetwork   uint8      // Network ID (0 for broadcast)
	TargetSystem    uint8      // System ID (0 for broadcast)
	TargetComponent uint8      // Component ID (0 for broadcast)
	Payload         [251]uint8 // Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
}

func (self *FileTransferProtocol) MsgID() uint8 {
	return 110
}

func (self *FileTransferProtocol) MsgName() string {
	return "FileTransferProtocol"
}

func (self *FileTransferProtocol) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetNetwork,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Payload,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *FileTransferProtocol) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetNetwork,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Payload,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Time synchronization message.
type Timesync struct {
	Tc1 int64 // Time sync timestamp 1
	Ts1 int64 // Time sync timestamp 2
}

func (self *Timesync) MsgID() uint8 {
	return 111
}

func (self *Timesync) MsgName() string {
	return "Timesync"
}

func (self *Timesync) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Tc1,
		&self.Ts1,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Timesync) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Tc1,
		&self.Ts1,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Camera-IMU triggering and synchronisation message.
type CameraTrigger struct {
	TimeUsec uint64 // Timestamp for the image frame in microseconds
	Seq      uint32 // Image frame sequence
}

func (self *CameraTrigger) MsgID() uint8 {
	return 112
}

func (self *CameraTrigger) MsgName() string {
	return "CameraTrigger"
}

func (self *CameraTrigger) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Seq,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *CameraTrigger) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Seq,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The global position, as returned by the Global Positioning System (GPS). This is
//                  NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
type HilGps struct {
	TimeUsec          uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	FixType           uint8  // 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
	Lat               int32  // Latitude (WGS84), in degrees * 1E7
	Lon               int32  // Longitude (WGS84), in degrees * 1E7
	Alt               int32  // Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
	Eph               uint16 // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
	Epv               uint16 // GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
	Vel               uint16 // GPS ground speed (m/s * 100). If unknown, set to: 65535
	Vn                int16  // GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
	Ve                int16  // GPS velocity in cm/s in EAST direction in earth-fixed NED frame
	Vd                int16  // GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
	Cog               uint16 // Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
	SatellitesVisible uint8  // Number of satellites visible. If unknown, set to 255
}

func (self *HilGps) MsgID() uint8 {
	return 113
}

func (self *HilGps) MsgName() string {
	return "HilGps"
}

func (self *HilGps) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.FixType,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.Eph,
		&self.Epv,
		&self.Vel,
		&self.Vn,
		&self.Ve,
		&self.Vd,
		&self.Cog,
		&self.SatellitesVisible,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *HilGps) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.FixType,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.Eph,
		&self.Epv,
		&self.Vel,
		&self.Vn,
		&self.Ve,
		&self.Vd,
		&self.Cog,
		&self.SatellitesVisible,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
type HilOpticalFlow struct {
	TimeUsec            uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	SensorId            uint8   // Sensor ID
	IntegrationTimeUs   uint32  // Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
	IntegratedX         float32 // Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
	IntegratedY         float32 // Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
	IntegratedXgyro     float32 // RH rotation around X axis (rad)
	IntegratedYgyro     float32 // RH rotation around Y axis (rad)
	IntegratedZgyro     float32 // RH rotation around Z axis (rad)
	Temperature         int16   // Temperature * 100 in centi-degrees Celsius
	Quality             uint8   // Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
	TimeDeltaDistanceUs uint32  // Time in microseconds since the distance was sampled.
	Distance            float32 // Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
}

func (self *HilOpticalFlow) MsgID() uint8 {
	return 114
}

func (self *HilOpticalFlow) MsgName() string {
	return "HilOpticalFlow"
}

func (self *HilOpticalFlow) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.SensorId,
		&self.IntegrationTimeUs,
		&self.IntegratedX,
		&self.IntegratedY,
		&self.IntegratedXgyro,
		&self.IntegratedYgyro,
		&self.IntegratedZgyro,
		&self.Temperature,
		&self.Quality,
		&self.TimeDeltaDistanceUs,
		&self.Distance,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *HilOpticalFlow) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.SensorId,
		&self.IntegrationTimeUs,
		&self.IntegratedX,
		&self.IntegratedY,
		&self.IntegratedXgyro,
		&self.IntegratedYgyro,
		&self.IntegratedZgyro,
		&self.Temperature,
		&self.Quality,
		&self.TimeDeltaDistanceUs,
		&self.Distance,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful for high throughput applications such as hardware in the loop simulations.
type HilStateQuaternion struct {
	TimeUsec           uint64     // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	AttitudeQuaternion [4]float32 // Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
	Rollspeed          float32    // Body frame roll / phi angular speed (rad/s)
	Pitchspeed         float32    // Body frame pitch / theta angular speed (rad/s)
	Yawspeed           float32    // Body frame yaw / psi angular speed (rad/s)
	Lat                int32      // Latitude, expressed as * 1E7
	Lon                int32      // Longitude, expressed as * 1E7
	Alt                int32      // Altitude in meters, expressed as * 1000 (millimeters)
	Vx                 int16      // Ground X Speed (Latitude), expressed as m/s * 100
	Vy                 int16      // Ground Y Speed (Longitude), expressed as m/s * 100
	Vz                 int16      // Ground Z Speed (Altitude), expressed as m/s * 100
	IndAirspeed        uint16     // Indicated airspeed, expressed as m/s * 100
	TrueAirspeed       uint16     // True airspeed, expressed as m/s * 100
	Xacc               int16      // X acceleration (mg)
	Yacc               int16      // Y acceleration (mg)
	Zacc               int16      // Z acceleration (mg)
}

func (self *HilStateQuaternion) MsgID() uint8 {
	return 115
}

func (self *HilStateQuaternion) MsgName() string {
	return "HilStateQuaternion"
}

func (self *HilStateQuaternion) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.AttitudeQuaternion,
		&self.Rollspeed,
		&self.Pitchspeed,
		&self.Yawspeed,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.IndAirspeed,
		&self.TrueAirspeed,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *HilStateQuaternion) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.AttitudeQuaternion,
		&self.Rollspeed,
		&self.Pitchspeed,
		&self.Yawspeed,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.Vx,
		&self.Vy,
		&self.Vz,
		&self.IndAirspeed,
		&self.TrueAirspeed,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to the described units
type ScaledImu2 struct {
	TimeBootMs uint32 // Timestamp (milliseconds since system boot)
	Xacc       int16  // X acceleration (mg)
	Yacc       int16  // Y acceleration (mg)
	Zacc       int16  // Z acceleration (mg)
	Xgyro      int16  // Angular speed around X axis (millirad /sec)
	Ygyro      int16  // Angular speed around Y axis (millirad /sec)
	Zgyro      int16  // Angular speed around Z axis (millirad /sec)
	Xmag       int16  // X Magnetic field (milli tesla)
	Ymag       int16  // Y Magnetic field (milli tesla)
	Zmag       int16  // Z Magnetic field (milli tesla)
}

func (self *ScaledImu2) MsgID() uint8 {
	return 116
}

func (self *ScaledImu2) MsgName() string {
	return "ScaledImu2"
}

func (self *ScaledImu2) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
		&self.Xgyro,
		&self.Ygyro,
		&self.Zgyro,
		&self.Xmag,
		&self.Ymag,
		&self.Zmag,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ScaledImu2) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
		&self.Xgyro,
		&self.Ygyro,
		&self.Zgyro,
		&self.Xmag,
		&self.Ymag,
		&self.Zmag,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END is called.
type LogRequestList struct {
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
	Start           uint16 // First log id (0 for first available)
	End             uint16 // Last log id (0xffff for last available)
}

func (self *LogRequestList) MsgID() uint8 {
	return 117
}

func (self *LogRequestList) MsgName() string {
	return "LogRequestList"
}

func (self *LogRequestList) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Start,
		&self.End,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *LogRequestList) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Start,
		&self.End,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Reply to LOG_REQUEST_LIST
type LogEntry struct {
	Id         uint16 // Log id
	NumLogs    uint16 // Total number of logs
	LastLogNum uint16 // High log number
	TimeUtc    uint32 // UTC timestamp of log in seconds since 1970, or 0 if not available
	Size       uint32 // Size of the log (may be approximate) in bytes
}

func (self *LogEntry) MsgID() uint8 {
	return 118
}

func (self *LogEntry) MsgName() string {
	return "LogEntry"
}

func (self *LogEntry) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Id,
		&self.NumLogs,
		&self.LastLogNum,
		&self.TimeUtc,
		&self.Size,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *LogEntry) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Id,
		&self.NumLogs,
		&self.LastLogNum,
		&self.TimeUtc,
		&self.Size,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Request a chunk of a log
type LogRequestData struct {
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
	Id              uint16 // Log id (from LOG_ENTRY reply)
	Ofs             uint32 // Offset into the log
	Count           uint32 // Number of bytes
}

func (self *LogRequestData) MsgID() uint8 {
	return 119
}

func (self *LogRequestData) MsgName() string {
	return "LogRequestData"
}

func (self *LogRequestData) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Id,
		&self.Ofs,
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

func (self *LogRequestData) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Id,
		&self.Ofs,
		&self.Count,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Reply to LOG_REQUEST_DATA
type LogData struct {
	Id    uint16    // Log id (from LOG_ENTRY reply)
	Ofs   uint32    // Offset into the log
	Count uint8     // Number of bytes (zero for end of log)
	Data  [90]uint8 // log data
}

func (self *LogData) MsgID() uint8 {
	return 120
}

func (self *LogData) MsgName() string {
	return "LogData"
}

func (self *LogData) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Id,
		&self.Ofs,
		&self.Count,
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

func (self *LogData) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Id,
		&self.Ofs,
		&self.Count,
		&self.Data,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Erase all logs
type LogErase struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *LogErase) MsgID() uint8 {
	return 121
}

func (self *LogErase) MsgName() string {
	return "LogErase"
}

func (self *LogErase) Pack(p *Packet) error {
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

func (self *LogErase) Unpack(p *Packet) error {
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

// Stop log transfer and resume normal logging
type LogRequestEnd struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

func (self *LogRequestEnd) MsgID() uint8 {
	return 122
}

func (self *LogRequestEnd) MsgName() string {
	return "LogRequestEnd"
}

func (self *LogRequestEnd) Pack(p *Packet) error {
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

func (self *LogRequestEnd) Unpack(p *Packet) error {
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

// data for injecting into the onboard GPS (used for DGPS)
type GpsInjectData struct {
	TargetSystem    uint8      // System ID
	TargetComponent uint8      // Component ID
	Len             uint8      // data length
	Data            [110]uint8 // raw data (110 is enough for 12 satellites of RTCMv2)
}

func (self *GpsInjectData) MsgID() uint8 {
	return 123
}

func (self *GpsInjectData) MsgName() string {
	return "GpsInjectData"
}

func (self *GpsInjectData) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
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

func (self *GpsInjectData) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Len,
		&self.Data,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame).
type Gps2Raw struct {
	TimeUsec          uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	FixType           uint8  // 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS fix, 5: RTK Fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
	Lat               int32  // Latitude (WGS84), in degrees * 1E7
	Lon               int32  // Longitude (WGS84), in degrees * 1E7
	Alt               int32  // Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
	Eph               uint16 // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	Epv               uint16 // GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	Vel               uint16 // GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
	Cog               uint16 // Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	SatellitesVisible uint8  // Number of satellites visible. If unknown, set to 255
	DgpsNumch         uint8  // Number of DGPS satellites
	DgpsAge           uint32 // Age of DGPS info
}

func (self *Gps2Raw) MsgID() uint8 {
	return 124
}

func (self *Gps2Raw) MsgName() string {
	return "Gps2Raw"
}

func (self *Gps2Raw) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.FixType,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.Eph,
		&self.Epv,
		&self.Vel,
		&self.Cog,
		&self.SatellitesVisible,
		&self.DgpsNumch,
		&self.DgpsAge,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Gps2Raw) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.FixType,
		&self.Lat,
		&self.Lon,
		&self.Alt,
		&self.Eph,
		&self.Epv,
		&self.Vel,
		&self.Cog,
		&self.SatellitesVisible,
		&self.DgpsNumch,
		&self.DgpsAge,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Power supply status
type PowerStatus struct {
	Vcc    uint16 // 5V rail voltage in millivolts
	Vservo uint16 // servo rail voltage in millivolts
	Flags  uint16 // power supply status flags (see MAV_POWER_STATUS enum)
}

func (self *PowerStatus) MsgID() uint8 {
	return 125
}

func (self *PowerStatus) MsgName() string {
	return "PowerStatus"
}

func (self *PowerStatus) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Vcc,
		&self.Vservo,
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

func (self *PowerStatus) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Vcc,
		&self.Vservo,
		&self.Flags,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages or change the devices settings. A message with zero bytes can be used to change just the baudrate.
type SerialControl struct {
	Device   uint8     // See SERIAL_CONTROL_DEV enum
	Flags    uint8     // See SERIAL_CONTROL_FLAG enum
	Timeout  uint16    // Timeout for reply data in milliseconds
	Baudrate uint32    // Baudrate of transfer. Zero means no change.
	Count    uint8     // how many bytes in this transfer
	Data     [70]uint8 // serial data
}

func (self *SerialControl) MsgID() uint8 {
	return 126
}

func (self *SerialControl) MsgName() string {
	return "SerialControl"
}

func (self *SerialControl) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Device,
		&self.Flags,
		&self.Timeout,
		&self.Baudrate,
		&self.Count,
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

func (self *SerialControl) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Device,
		&self.Flags,
		&self.Timeout,
		&self.Baudrate,
		&self.Count,
		&self.Data,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
type GpsRtk struct {
	TimeLastBaselineMs uint32 // Time since boot of last baseline message received in ms.
	RtkReceiverId      uint8  // Identification of connected RTK receiver.
	Wn                 uint16 // GPS Week Number of last baseline
	Tow                uint32 // GPS Time of Week of last baseline
	RtkHealth          uint8  // GPS-specific health report for RTK data.
	RtkRate            uint8  // Rate of baseline messages being received by GPS, in HZ
	Nsats              uint8  // Current number of sats used for RTK calculation.
	BaselineCoordsType uint8  // Coordinate system of baseline. 0 == ECEF, 1 == NED
	BaselineAMm        int32  // Current baseline in ECEF x or NED north component in mm.
	BaselineBMm        int32  // Current baseline in ECEF y or NED east component in mm.
	BaselineCMm        int32  // Current baseline in ECEF z or NED down component in mm.
	Accuracy           uint32 // Current estimate of baseline accuracy.
	IarNumHypotheses   int32  // Current number of integer ambiguity hypotheses.
}

func (self *GpsRtk) MsgID() uint8 {
	return 127
}

func (self *GpsRtk) MsgName() string {
	return "GpsRtk"
}

func (self *GpsRtk) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeLastBaselineMs,
		&self.RtkReceiverId,
		&self.Wn,
		&self.Tow,
		&self.RtkHealth,
		&self.RtkRate,
		&self.Nsats,
		&self.BaselineCoordsType,
		&self.BaselineAMm,
		&self.BaselineBMm,
		&self.BaselineCMm,
		&self.Accuracy,
		&self.IarNumHypotheses,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *GpsRtk) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeLastBaselineMs,
		&self.RtkReceiverId,
		&self.Wn,
		&self.Tow,
		&self.RtkHealth,
		&self.RtkRate,
		&self.Nsats,
		&self.BaselineCoordsType,
		&self.BaselineAMm,
		&self.BaselineBMm,
		&self.BaselineCMm,
		&self.Accuracy,
		&self.IarNumHypotheses,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
type Gps2Rtk struct {
	TimeLastBaselineMs uint32 // Time since boot of last baseline message received in ms.
	RtkReceiverId      uint8  // Identification of connected RTK receiver.
	Wn                 uint16 // GPS Week Number of last baseline
	Tow                uint32 // GPS Time of Week of last baseline
	RtkHealth          uint8  // GPS-specific health report for RTK data.
	RtkRate            uint8  // Rate of baseline messages being received by GPS, in HZ
	Nsats              uint8  // Current number of sats used for RTK calculation.
	BaselineCoordsType uint8  // Coordinate system of baseline. 0 == ECEF, 1 == NED
	BaselineAMm        int32  // Current baseline in ECEF x or NED north component in mm.
	BaselineBMm        int32  // Current baseline in ECEF y or NED east component in mm.
	BaselineCMm        int32  // Current baseline in ECEF z or NED down component in mm.
	Accuracy           uint32 // Current estimate of baseline accuracy.
	IarNumHypotheses   int32  // Current number of integer ambiguity hypotheses.
}

func (self *Gps2Rtk) MsgID() uint8 {
	return 128
}

func (self *Gps2Rtk) MsgName() string {
	return "Gps2Rtk"
}

func (self *Gps2Rtk) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeLastBaselineMs,
		&self.RtkReceiverId,
		&self.Wn,
		&self.Tow,
		&self.RtkHealth,
		&self.RtkRate,
		&self.Nsats,
		&self.BaselineCoordsType,
		&self.BaselineAMm,
		&self.BaselineBMm,
		&self.BaselineCMm,
		&self.Accuracy,
		&self.IarNumHypotheses,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Gps2Rtk) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeLastBaselineMs,
		&self.RtkReceiverId,
		&self.Wn,
		&self.Tow,
		&self.RtkHealth,
		&self.RtkRate,
		&self.Nsats,
		&self.BaselineCoordsType,
		&self.BaselineAMm,
		&self.BaselineBMm,
		&self.BaselineCMm,
		&self.Accuracy,
		&self.IarNumHypotheses,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described units
type ScaledImu3 struct {
	TimeBootMs uint32 // Timestamp (milliseconds since system boot)
	Xacc       int16  // X acceleration (mg)
	Yacc       int16  // Y acceleration (mg)
	Zacc       int16  // Z acceleration (mg)
	Xgyro      int16  // Angular speed around X axis (millirad /sec)
	Ygyro      int16  // Angular speed around Y axis (millirad /sec)
	Zgyro      int16  // Angular speed around Z axis (millirad /sec)
	Xmag       int16  // X Magnetic field (milli tesla)
	Ymag       int16  // Y Magnetic field (milli tesla)
	Zmag       int16  // Z Magnetic field (milli tesla)
}

func (self *ScaledImu3) MsgID() uint8 {
	return 129
}

func (self *ScaledImu3) MsgName() string {
	return "ScaledImu3"
}

func (self *ScaledImu3) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
		&self.Xgyro,
		&self.Ygyro,
		&self.Zgyro,
		&self.Xmag,
		&self.Ymag,
		&self.Zmag,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ScaledImu3) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Xacc,
		&self.Yacc,
		&self.Zacc,
		&self.Xgyro,
		&self.Ygyro,
		&self.Zgyro,
		&self.Xmag,
		&self.Ymag,
		&self.Zmag,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

//
type DataTransmissionHandshake struct {
	Type       uint8  // type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
	Size       uint32 // total data size in bytes (set on ACK only)
	Width      uint16 // Width of a matrix or image
	Height     uint16 // Height of a matrix or image
	Packets    uint16 // number of packets beeing sent (set on ACK only)
	Payload    uint8  // payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
	JpgQuality uint8  // JPEG quality out of [1,100]
}

func (self *DataTransmissionHandshake) MsgID() uint8 {
	return 130
}

func (self *DataTransmissionHandshake) MsgName() string {
	return "DataTransmissionHandshake"
}

func (self *DataTransmissionHandshake) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Type,
		&self.Size,
		&self.Width,
		&self.Height,
		&self.Packets,
		&self.Payload,
		&self.JpgQuality,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *DataTransmissionHandshake) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Type,
		&self.Size,
		&self.Width,
		&self.Height,
		&self.Packets,
		&self.Payload,
		&self.JpgQuality,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

//
type EncapsulatedData struct {
	Seqnr uint16     // sequence number (starting with 0 on every transmission)
	Data  [253]uint8 // image data bytes
}

func (self *EncapsulatedData) MsgID() uint8 {
	return 131
}

func (self *EncapsulatedData) MsgName() string {
	return "EncapsulatedData"
}

func (self *EncapsulatedData) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Seqnr,
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

func (self *EncapsulatedData) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Seqnr,
		&self.Data,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

//
type DistanceSensor struct {
	TimeBootMs      uint32 // Time since system boot
	MinDistance     uint16 // Minimum distance the sensor can measure in centimeters
	MaxDistance     uint16 // Maximum distance the sensor can measure in centimeters
	CurrentDistance uint16 // Current distance reading
	Type            uint8  // Type from MAV_DISTANCE_SENSOR enum.
	Id              uint8  // Onboard ID of the sensor
	Orientation     uint8  // Direction the sensor faces from MAV_SENSOR_ORIENTATION enum.
	Covariance      uint8  // Measurement covariance in centimeters, 0 for unknown / invalid readings
}

func (self *DistanceSensor) MsgID() uint8 {
	return 132
}

func (self *DistanceSensor) MsgName() string {
	return "DistanceSensor"
}

func (self *DistanceSensor) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.MinDistance,
		&self.MaxDistance,
		&self.CurrentDistance,
		&self.Type,
		&self.Id,
		&self.Orientation,
		&self.Covariance,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *DistanceSensor) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.MinDistance,
		&self.MaxDistance,
		&self.CurrentDistance,
		&self.Type,
		&self.Id,
		&self.Orientation,
		&self.Covariance,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Request for terrain data and terrain status
type TerrainRequest struct {
	Lat         int32  // Latitude of SW corner of first grid (degrees *10^7)
	Lon         int32  // Longitude of SW corner of first grid (in degrees *10^7)
	GridSpacing uint16 // Grid spacing in meters
	Mask        uint64 // Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
}

func (self *TerrainRequest) MsgID() uint8 {
	return 133
}

func (self *TerrainRequest) MsgName() string {
	return "TerrainRequest"
}

func (self *TerrainRequest) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Lat,
		&self.Lon,
		&self.GridSpacing,
		&self.Mask,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *TerrainRequest) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Lat,
		&self.Lon,
		&self.GridSpacing,
		&self.Mask,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUEST
type TerrainData struct {
	Lat         int32     // Latitude of SW corner of first grid (degrees *10^7)
	Lon         int32     // Longitude of SW corner of first grid (in degrees *10^7)
	GridSpacing uint16    // Grid spacing in meters
	Gridbit     uint8     // bit within the terrain request mask
	Data        [16]int16 // Terrain data in meters AMSL
}

func (self *TerrainData) MsgID() uint8 {
	return 134
}

func (self *TerrainData) MsgName() string {
	return "TerrainData"
}

func (self *TerrainData) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Lat,
		&self.Lon,
		&self.GridSpacing,
		&self.Gridbit,
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

func (self *TerrainData) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Lat,
		&self.Lon,
		&self.GridSpacing,
		&self.Gridbit,
		&self.Data,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle has all terrain data needed for a mission.
type TerrainCheck struct {
	Lat int32 // Latitude (degrees *10^7)
	Lon int32 // Longitude (degrees *10^7)
}

func (self *TerrainCheck) MsgID() uint8 {
	return 135
}

func (self *TerrainCheck) MsgName() string {
	return "TerrainCheck"
}

func (self *TerrainCheck) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Lat,
		&self.Lon,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *TerrainCheck) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Lat,
		&self.Lon,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Response from a TERRAIN_CHECK request
type TerrainReport struct {
	Lat           int32   // Latitude (degrees *10^7)
	Lon           int32   // Longitude (degrees *10^7)
	Spacing       uint16  // grid spacing (zero if terrain at this location unavailable)
	TerrainHeight float32 // Terrain height in meters AMSL
	CurrentHeight float32 // Current vehicle height above lat/lon terrain height (meters)
	Pending       uint16  // Number of 4x4 terrain blocks waiting to be received or read from disk
	Loaded        uint16  // Number of 4x4 terrain blocks in memory
}

func (self *TerrainReport) MsgID() uint8 {
	return 136
}

func (self *TerrainReport) MsgName() string {
	return "TerrainReport"
}

func (self *TerrainReport) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Lat,
		&self.Lon,
		&self.Spacing,
		&self.TerrainHeight,
		&self.CurrentHeight,
		&self.Pending,
		&self.Loaded,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *TerrainReport) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Lat,
		&self.Lon,
		&self.Spacing,
		&self.TerrainHeight,
		&self.CurrentHeight,
		&self.Pending,
		&self.Loaded,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Barometer readings for 2nd barometer
type ScaledPressure2 struct {
	TimeBootMs  uint32  // Timestamp (milliseconds since system boot)
	PressAbs    float32 // Absolute pressure (hectopascal)
	PressDiff   float32 // Differential pressure 1 (hectopascal)
	Temperature int16   // Temperature measurement (0.01 degrees celsius)
}

func (self *ScaledPressure2) MsgID() uint8 {
	return 137
}

func (self *ScaledPressure2) MsgName() string {
	return "ScaledPressure2"
}

func (self *ScaledPressure2) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.PressAbs,
		&self.PressDiff,
		&self.Temperature,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ScaledPressure2) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.PressAbs,
		&self.PressDiff,
		&self.Temperature,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Motion capture attitude and position
type AttPosMocap struct {
	TimeUsec uint64     // Timestamp (micros since boot or Unix epoch)
	Q        [4]float32 // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	X        float32    // X position in meters (NED)
	Y        float32    // Y position in meters (NED)
	Z        float32    // Z position in meters (NED)
}

func (self *AttPosMocap) MsgID() uint8 {
	return 138
}

func (self *AttPosMocap) MsgName() string {
	return "AttPosMocap"
}

func (self *AttPosMocap) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Q,
		&self.X,
		&self.Y,
		&self.Z,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *AttPosMocap) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.Q,
		&self.X,
		&self.Y,
		&self.Z,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Set the vehicle attitude and body angular rates.
type SetActuatorControlTarget struct {
	TimeUsec        uint64     // Timestamp (micros since boot or Unix epoch)
	GroupMlx        uint8      // Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
	TargetSystem    uint8      // System ID
	TargetComponent uint8      // Component ID
	Controls        [8]float32 // Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
}

func (self *SetActuatorControlTarget) MsgID() uint8 {
	return 139
}

func (self *SetActuatorControlTarget) MsgName() string {
	return "SetActuatorControlTarget"
}

func (self *SetActuatorControlTarget) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.GroupMlx,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Controls,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *SetActuatorControlTarget) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.GroupMlx,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.Controls,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Set the vehicle attitude and body angular rates.
type ActuatorControlTarget struct {
	TimeUsec uint64     // Timestamp (micros since boot or Unix epoch)
	GroupMlx uint8      // Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
	Controls [8]float32 // Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
}

func (self *ActuatorControlTarget) MsgID() uint8 {
	return 140
}

func (self *ActuatorControlTarget) MsgName() string {
	return "ActuatorControlTarget"
}

func (self *ActuatorControlTarget) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.GroupMlx,
		&self.Controls,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *ActuatorControlTarget) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeUsec,
		&self.GroupMlx,
		&self.Controls,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Battery information
type BatteryStatus struct {
	Id               uint8      // Battery ID
	BatteryFunction  uint8      // Function of the battery
	Type             uint8      // Type (chemistry) of the battery
	Temperature      int16      // Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
	Voltages         [10]uint16 // Battery voltage of cells, in millivolts (1 = 1 millivolt)
	CurrentBattery   int16      // Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
	CurrentConsumed  int32      // Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
	EnergyConsumed   int32      // Consumed energy, in 100*Joules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
	BatteryRemaining int8       // Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
}

func (self *BatteryStatus) MsgID() uint8 {
	return 147
}

func (self *BatteryStatus) MsgName() string {
	return "BatteryStatus"
}

func (self *BatteryStatus) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Id,
		&self.BatteryFunction,
		&self.Type,
		&self.Temperature,
		&self.Voltages,
		&self.CurrentBattery,
		&self.CurrentConsumed,
		&self.EnergyConsumed,
		&self.BatteryRemaining,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *BatteryStatus) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Id,
		&self.BatteryFunction,
		&self.Type,
		&self.Temperature,
		&self.Voltages,
		&self.CurrentBattery,
		&self.CurrentConsumed,
		&self.EnergyConsumed,
		&self.BatteryRemaining,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Version and capability of autopilot software
type AutopilotVersion struct {
	Capabilities            uint64   // bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
	FlightSwVersion         uint32   // Firmware version number
	MiddlewareSwVersion     uint32   // Middleware version number
	OsSwVersion             uint32   // Operating system version number
	BoardVersion            uint32   // HW / board version (last 8 bytes should be silicon ID, if any)
	FlightCustomVersion     [8]uint8 // Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
	MiddlewareCustomVersion [8]uint8 // Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
	OsCustomVersion         [8]uint8 // Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
	VendorId                uint16   // ID of the board vendor
	ProductId               uint16   // ID of the product
	Uid                     uint64   // UID if provided by hardware
}

func (self *AutopilotVersion) MsgID() uint8 {
	return 148
}

func (self *AutopilotVersion) MsgName() string {
	return "AutopilotVersion"
}

func (self *AutopilotVersion) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Capabilities,
		&self.FlightSwVersion,
		&self.MiddlewareSwVersion,
		&self.OsSwVersion,
		&self.BoardVersion,
		&self.FlightCustomVersion,
		&self.MiddlewareCustomVersion,
		&self.OsCustomVersion,
		&self.VendorId,
		&self.ProductId,
		&self.Uid,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *AutopilotVersion) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Capabilities,
		&self.FlightSwVersion,
		&self.MiddlewareSwVersion,
		&self.OsSwVersion,
		&self.BoardVersion,
		&self.FlightCustomVersion,
		&self.MiddlewareCustomVersion,
		&self.OsCustomVersion,
		&self.VendorId,
		&self.ProductId,
		&self.Uid,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// The location of a landing area captured from a downward facing camera
type LandingTarget struct {
	TargetNum uint8   // The ID of the target if multiple targets are present
	Frame     uint8   // MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
	AngleX    float32 // X-axis angular offset (in radians) of the target from the center of the image
	AngleY    float32 // Y-axis angular offset (in radians) of the target from the center of the image
	Distance  float32 // Distance to the target from the vehicle in meters
}

func (self *LandingTarget) MsgID() uint8 {
	return 149
}

func (self *LandingTarget) MsgName() string {
	return "LandingTarget"
}

func (self *LandingTarget) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetNum,
		&self.Frame,
		&self.AngleX,
		&self.AngleY,
		&self.Distance,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *LandingTarget) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetNum,
		&self.Frame,
		&self.AngleX,
		&self.AngleY,
		&self.Distance,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Message implementing parts of the V2 payload specs in V1 frames for transitional support.
type V2Extension struct {
	TargetNetwork   uint8      // Network ID (0 for broadcast)
	TargetSystem    uint8      // System ID (0 for broadcast)
	TargetComponent uint8      // Component ID (0 for broadcast)
	MessageType     uint16     // A code that identifies the software component that understands this message (analogous to usb device classes or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/extension-message-ids.xml.  Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
	Payload         [249]uint8 // Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
}

func (self *V2Extension) MsgID() uint8 {
	return 248
}

func (self *V2Extension) MsgName() string {
	return "V2Extension"
}

func (self *V2Extension) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TargetNetwork,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.MessageType,
		&self.Payload,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *V2Extension) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TargetNetwork,
		&self.TargetSystem,
		&self.TargetComponent,
		&self.MessageType,
		&self.Payload,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type MemoryVect struct {
	Address uint16   // Starting address of the debug variables
	Ver     uint8    // Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
	Type    uint8    // Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
	Value   [32]int8 // Memory contents at specified address
}

func (self *MemoryVect) MsgID() uint8 {
	return 249
}

func (self *MemoryVect) MsgName() string {
	return "MemoryVect"
}

func (self *MemoryVect) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Address,
		&self.Ver,
		&self.Type,
		&self.Value,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *MemoryVect) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Address,
		&self.Ver,
		&self.Type,
		&self.Value,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

//
type DebugVect struct {
	Name     [10]byte // Name
	TimeUsec uint64   // Timestamp
	X        float32  // x
	Y        float32  // y
	Z        float32  // z
}

func (self *DebugVect) MsgID() uint8 {
	return 250
}

func (self *DebugVect) MsgName() string {
	return "DebugVect"
}

func (self *DebugVect) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Name,
		&self.TimeUsec,
		&self.X,
		&self.Y,
		&self.Z,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *DebugVect) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Name,
		&self.TimeUsec,
		&self.X,
		&self.Y,
		&self.Z,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type NamedValueFloat struct {
	TimeBootMs uint32   // Timestamp (milliseconds since system boot)
	Name       [10]byte // Name of the debug variable
	Value      float32  // Floating point value
}

func (self *NamedValueFloat) MsgID() uint8 {
	return 251
}

func (self *NamedValueFloat) MsgName() string {
	return "NamedValueFloat"
}

func (self *NamedValueFloat) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Name,
		&self.Value,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *NamedValueFloat) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Name,
		&self.Value,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type NamedValueInt struct {
	TimeBootMs uint32   // Timestamp (milliseconds since system boot)
	Name       [10]byte // Name of the debug variable
	Value      int32    // Signed integer value
}

func (self *NamedValueInt) MsgID() uint8 {
	return 252
}

func (self *NamedValueInt) MsgName() string {
	return "NamedValueInt"
}

func (self *NamedValueInt) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Name,
		&self.Value,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *NamedValueInt) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Name,
		&self.Value,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
type Statustext struct {
	Severity uint8    // Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
	Text     [50]byte // Status text message, without null termination character
}

func (self *Statustext) MsgID() uint8 {
	return 253
}

func (self *Statustext) MsgName() string {
	return "Statustext"
}

func (self *Statustext) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.Severity,
		&self.Text,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Statustext) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.Severity,
		&self.Text,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}

// Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N.
type Debug struct {
	TimeBootMs uint32  // Timestamp (milliseconds since system boot)
	Ind        uint8   // index of debug variable
	Value      float32 // DEBUG value
}

func (self *Debug) MsgID() uint8 {
	return 254
}

func (self *Debug) MsgName() string {
	return "Debug"
}

func (self *Debug) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Ind,
		&self.Value,
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *Debug) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{}{
		&self.TimeBootMs,
		&self.Ind,
		&self.Value,
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}
