import time
from enum import Enum

#TODO:This time needs to come from fsw, or RFMController...
def PX4LIB_GetPX4TimeUs():
    nanos = time.monotonic_ns() 
    return nanos / 1000


class PX4_VehicleCmd_t(Enum):
    PX4_VEHICLE_CMD_NONE                         =     0
    PX4_VEHICLE_CMD_CUSTOM_0                     =     1
    PX4_VEHICLE_CMD_CUSTOM_1                     =     2
    PX4_VEHICLE_CMD_CUSTOM_2                     =     3
    PX4_VEHICLE_CMD_SET_ATTITUDE                 =     4
    PX4_VEHICLE_CMD_WAIT_FOR_ATP                 =     5
    PX4_VEHICLE_CMD_NAV_WAYPOINT                 =    16
    PX4_VEHICLE_CMD_NAV_LOITER_UNLIM             =    17
    PX4_VEHICLE_CMD_NAV_LOITER_TURNS             =    18
    PX4_VEHICLE_CMD_NAV_LOITER_TIME              =    19
    PX4_VEHICLE_CMD_NAV_RETURN_TO_LAUNCH         =    20
    PX4_VEHICLE_CMD_NAV_LAND                     =    21
    PX4_VEHICLE_CMD_NAV_TAKEOFF                  =    22
    PX4_VEHICLE_CMD_NAV_LOITER_TO_ALT            =    31
    PX4_VEHICLE_CMD_NAV_ROI                      =    80
    PX4_VEHICLE_CMD_NAV_PATHPLANNING             =    81
    PX4_VEHICLE_CMD_NAV_VTOL_TAKEOFF             =    84
    PX4_VEHICLE_CMD_NAV_VTOL_LAND                =    85
    PX4_VEHICLE_CMD_NAV_GUIDED_LIMITS            =    90
    PX4_VEHICLE_CMD_NAV_GUIDED_MASTER            =    91
    PX4_VEHICLE_CMD_NAV_GUIDED_ENABLE            =    92
    PX4_VEHICLE_CMD_NAV_LAST                     =    95
    PX4_VEHICLE_CMD_CONDITION_DELAY              =   112
    PX4_VEHICLE_CMD_CONDITION_CHANGE_ALT         =   113
    PX4_VEHICLE_CMD_CONDITION_DISTANCE           =   114
    PX4_VEHICLE_CMD_CONDITION_YAW                =   115
    PX4_VEHICLE_CMD_CONDITION_LAST               =   159
    PX4_VEHICLE_CMD_DO_SET_MODE                  =   176
    PX4_VEHICLE_CMD_DO_JUMP                      =   177
    PX4_VEHICLE_CMD_DO_CHANGE_SPEED              =   178
    PX4_VEHICLE_CMD_DO_SET_HOME                  =   179
    PX4_VEHICLE_CMD_DO_SET_PARAMETER             =   180
    PX4_VEHICLE_CMD_DO_SET_RELAY                 =   181
    PX4_VEHICLE_CMD_DO_REPEAT_RELAY              =   182
    PX4_VEHICLE_CMD_DO_SET_SERVO                 =   183
    PX4_VEHICLE_CMD_DO_REPEAT_SERVO              =   184
    PX4_VEHICLE_CMD_DO_FLIGHTTERMINATION         =   185
    PX4_VEHICLE_CMD_DO_GO_AROUND                 =   191
    PX4_VEHICLE_CMD_DO_REPOSITION                =   192
    PX4_VEHICLE_CMD_DO_PAUSE_CONTINUE            =   193
    PX4_VEHICLE_CMD_DO_CONTROL_VIDEO             =   200
    PX4_VEHICLE_CMD_DO_DIGICAM_CONTROL           =   203
    PX4_VEHICLE_CMD_DO_MOUNT_CONFIGURE           =   204
    PX4_VEHICLE_CMD_DO_MOUNT_CONTROL             =   205
    PX4_VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST        =   206
    PX4_VEHICLE_CMD_DO_FENCE_ENABLE              =   207
    PX4_VEHICLE_CMD_DO_PARACHUTE                 =   208
    PX4_VEHICLE_CMD_DO_INVERTED_FLIGHT           =   210
    PX4_VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT        =   220
    PX4_VEHICLE_CMD_DO_GUIDED_MASTER             =   221
    PX4_VEHICLE_CMD_DO_GUIDED_LIMITS             =   222
    PX4_VEHICLE_CMD_DO_LAST                      =   240
    PX4_VEHICLE_CMD_PREFLIGHT_CALIBRATION        =   241
    PX4_VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS =   242
    PX4_VEHICLE_CMD_PREFLIGHT_UAVCAN             =   243
    PX4_VEHICLE_CMD_PREFLIGHT_STORAGE            =   245
    PX4_VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN    =   246
    PX4_VEHICLE_CMD_OVERRIDE_GOTO                =   252
    PX4_VEHICLE_CMD_MISSION_START                =   300
    PX4_VEHICLE_CMD_COMPONENT_ARM_DISARM         =   400
    PX4_VEHICLE_CMD_START_RX_PAIR                =   500
    PX4_VEHICLE_CMD_DO_TRIGGER_CONTROL           =  2003
    PX4_VEHICLE_CMD_DO_VTOL_TRANSITION           =  3000
    PX4_VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY       = 30001
    PX4_VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY       = 30002


class PX4_NavigationState_t(Enum):
    PX4_NAVIGATION_STATE_MANUAL                  =  0
    PX4_NAVIGATION_STATE_ALTCTL                  =  1
    PX4_NAVIGATION_STATE_POSCTL                  =  2
    PX4_NAVIGATION_STATE_AUTO_MISSION            =  3
    PX4_NAVIGATION_STATE_AUTO_LOITER             =  4
    PX4_NAVIGATION_STATE_AUTO_RTL                =  5
    PX4_NAVIGATION_STATE_AUTO_RCRECOVER          =  6
    PX4_NAVIGATION_STATE_AUTO_RTGS               =  7
    PX4_NAVIGATION_STATE_AUTO_LANDENGFAIL        =  8
    PX4_NAVIGATION_STATE_AUTO_LANDGPSFAIL        =  9
    PX4_NAVIGATION_STATE_ACRO                    = 10
    PX4_NAVIGATION_STATE_UNUSED                  = 11
    PX4_NAVIGATION_STATE_DESCEND                 = 12
    PX4_NAVIGATION_STATE_TERMINATION             = 13
    PX4_NAVIGATION_STATE_OFFBOARD                = 14
    PX4_NAVIGATION_STATE_STAB                    = 15
    PX4_NAVIGATION_STATE_RATTITUDE               = 16
    PX4_NAVIGATION_STATE_AUTO_TAKEOFF            = 17
    PX4_NAVIGATION_STATE_AUTO_LAND               = 18
    PX4_NAVIGATION_STATE_AUTO_FOLLOW_TARGET      = 19
    PX4_NAVIGATION_STATE_AUTO_PRECLAND           = 20
    PX4_NAVIGATION_STATE_MAX                     = 21



# #*
#  * \brief Return to Launch states
#  */
class RTLState(Enum):
    #! The vehicle is in idle state during RTL navigation */
    RTL_STATE_NONE              = 0
    #! The vehicle is in climb state during RTL navigation */
    RTL_STATE_CLIMB             = 1
    #! The vehicle is in Return to home latitude and longitude state during RTL navigation */
    RTL_STATE_RETURN            = 2
    #!  VTOL to MC transition state during RTL navigation */
    RTL_STATE_TRANSITION_TO_MC  = 3
    #! The vehicle is in descend state during RTL navigation */
    RTL_STATE_DESCEND           = 4
    #! The vehicle is in loiter state during RTL navigation */
    RTL_STATE_LOITER            = 5
    #! The vehicle is in land state during RTL navigation */
    RTL_STATE_LAND              = 6
    #! The vehicle is in landed state during RTL navigation */
    RTL_STATE_LANDED            = 7



class PX4_SetpointType_t(Enum):
    PX4_SETPOINT_TYPE_POSITION                   = 0,
    PX4_SETPOINT_TYPE_VELOCITY                   = 1,
    PX4_SETPOINT_TYPE_LOITER                     = 2,
    PX4_SETPOINT_TYPE_TAKEOFF                    = 3,
    PX4_SETPOINT_TYPE_LAND                       = 4,
    PX4_SETPOINT_TYPE_IDLE                       = 5,
    PX4_SETPOINT_TYPE_OFFBOARD                   = 6,
    PX4_SETPOINT_TYPE_FOLLOW_TARGET              = 7,
    PX4_SETPOINT_TYPE_ATTITUDE                   = 8



class PX4_PositionSetpoint_t():
    def __init__(self):
        self.Timestamp: int = 0
        self.Lat: float = 0
        self.Lon: float  = 0
        self.X: float = 0
        self.Y: float = 0
        self.Z: float = 0
        self.VX: float = 0
        self.VY: float = 0
        self.VZ: float = 0
        self.Alt: float = 0
        self.Yaw: float = 0
        self.Yawspeed: float = 0
        self.YawBody: float = 0
        self.PitchBody: float = 0
        self.RollBody: float = 0
        self.LoiterRadius: float = 0
        self.PitchMin: float = 0
        self.AX: float = 0
        self.AY: float = 0
        self.AZ: float = 0
        self.AcceptanceRadius: float = 0
        self.CruisingSpeed: float = 0
        self.CruisingThrottle: float = 0
        self.Valid: bool = 0
        self.Type: PX4_SetpointType_t  = 0
        self.PositionValid: bool = False
        self.VelocityValid: bool = False
        self.VelocityFrame: int = 0
        self.AltValid: bool = False
        self.YawValid: bool = False
        self.DisableMcYawControl: bool = False
        self.YawspeedValid: bool = False
        self.LoiterDirection: int = 0
        self.AccelerationValid: bool = False
        self.AccelerationIsForce: bool = False


class PX4_MissionResultMsg_t():
    def __init__(self):
        self.Timestamp: int = 0
        self.InstanceCount: int = 0
        self.SeqReached: int = 0
        self.SeqCurrent: int = 0
        self.SeqTotal: int = 0
        self.ItemChangedIndex: int = 0
        self.ItemDoJumpRemaining: int = 0
        self.Valid: bool = False
        self.Warning: bool = False
        self.Reached: bool = False
        self.Finished: bool = False
        self.Failure: bool = False
        self.StayInFailsafe: bool = False
        self.FlightTermination: bool = False
        self.ItemDoJumpChanged: bool = False


class PX4_PositionSetpointTripletMsg_t():
    def __init__(self) -> None:
        self.Timestamp: int = 0
        self.Previous: PX4_PositionSetpoint_t = PX4_PositionSetpoint_t()
        self.Current: PX4_PositionSetpoint_t = PX4_PositionSetpoint_t()
        self.Next: PX4_PositionSetpoint_t = PX4_PositionSetpoint_t()
