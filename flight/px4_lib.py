import time
from enum import Enum


# TODO:This time needs to come from fsw, or RFMController...
def PX4LIB_GetPX4TimeUs():
    nanos = time.monotonic_ns()
    return nanos / 1000


class PX4_MissionMsg_t():
    def __init__(self) -> None:
        self.Timestamp: int = 0
        self.DatamanID: int = 0
        self.Count: int = 0
        self.CurrentSeq: int = 0


class PX4_HomePositionMsg_t:
    def __init__(self) -> None:
        self.Timestamp: int = 0
        self.Lat: float = 0
        self.Lon: float = 0
        self.Alt: float = 0
        self.X: float = 0
        self.Y: float = 0
        self.Z: float = 0
        self.Yaw: float = 0
        self.DirectionX: float = 0
        self.DirectionY: float = 0
        self.DirectionZ: float = 0


class PX4_GpsFixType_t(Enum):
    PX4_GPS_NONE0_FIX = 0,
    PX4_GPS_NONE1_FIX = 1,
    PX4_GPS_2D_FIX = 2,
    PX4_GPS_3D_FIX = 3,
    PX4_GPS_DGPS_FIX = 4,
    PX4_GPS_RTK_FIX = 5


class PX4_GpsStatus_t(Enum):
    PX4_GPS_STATUS_VALID = 1,
    PX4_GPS_STATUS_INVALID = 2,


class PX4_UtcClockStatus_t(Enum):
    PX4_UTC_STATUS_OK = 1,
    PX4_UTC_STATUS_NOT_OK = 2,


class PX4_VehicleGpsPositionMsg_t():
    Timestamp: int = 0
    TimeUtcUsec: int = 0
    Lat: int = 0
    Lon: int = 0
    Alt: int = 0
    AltEllipsoid: int = 0
    SVariance: float = 0
    CVariance: float = 0
    EpH: float = 0
    EpV: float = 0
    HDOP: float = 0
    VDOP: float = 0
    NoisePerMs: int = 0
    JammingIndicator: int = 0
    Vel_m_s: float = 0
    Vel_n_m_s: float = 0
    Vel_e_m_s: float = 0
    Vel_d_m_s: float = 0
    COG: float = 0
    TimestampTimeRelative: int = 0
    FixType: PX4_GpsFixType_t = 0
    VelNedValid: bool = False
    SatellitesUsed: int = 0
    Status: PX4_GpsStatus_t = 1
    UtcClockStatus: PX4_UtcClockStatus_t = 1


class PX4_VehicleGlobalPositionMsg_t():
    Timestamp: int = 0
    TimeUtcUsec: int = 0
    Lat: float = 0
    Lon: float = 0
    Alt: float = 0
    DeltaLatLon: list = [0, 0]
    DeltaAlt: float = 0
    LatLonResetCounter: int = 0
    AltResetCounter: int = 0
    VelN: float = 0
    VelE: float = 0
    VelD: float = 0
    Yaw: float = 0
    EpH: float = 0
    EpV: float = 0
    EvH: float = 0
    EvV: float = 0
    TerrainAlt: float = 0
    PressureAlt: float = 0
    TerrainAltValid: bool = False
    DeadReckoning: bool = False
    HorizontalVel: float = 0
    GroundTrack: float = 0


class PX4_ArmingState_t(Enum):
    PX4_ARMING_STATE_INIT = 0,
    PX4_ARMING_STATE_STANDBY = 1,
    PX4_ARMING_STATE_ARMED = 2,
    PX4_ARMING_STATE_ARMED_ERROR = 3,
    PX4_ARMING_STATE_STANDBY_ERROR = 4,
    PX4_ARMING_STATE_REBOOT = 5,
    PX4_ARMING_STATE_IN_AIR_RESTORE = 6,
    PX4_ARMING_STATE_MAX = 7


class PX4_HilState_t(Enum):
    PX4_HIL_STATE_OFF = 0,
    PX4_HIL_STATE_ON = 1


class PX4_SystemType_t(Enum):
    PX4_SYSTEM_TYPE_GENERIC = 0,
    PX4_SYSTEM_TYPE_FIXED_WING = 1,
    PX4_SYSTEM_TYPE_QUADROTOR = 2,
    PX4_SYSTEM_TYPE_COAXIAL = 3,
    PX4_SYSTEM_TYPE_HELICOPTER = 4,
    PX4_SYSTEM_TYPE_ANTENNA_TRACKER = 5,
    PX4_SYSTEM_TYPE_GCS = 6,
    PX4_SYSTEM_TYPE_AIRSHIP = 7,
    PX4_SYSTEM_TYPE_FREE_BALLOON = 8,
    PX4_SYSTEM_TYPE_ROCKET = 9,
    PX4_SYSTEM_TYPE_GROUND_ROVER = 10,
    PX4_SYSTEM_TYPE_SURFACE_BOAT = 11,
    PX4_SYSTEM_TYPE_SUBMARINE = 12,
    PX4_SYSTEM_TYPE_HEXAROTOR = 13,
    PX4_SYSTEM_TYPE_OCTOROTOR = 14,
    PX4_SYSTEM_TYPE_TRICOPTER = 15,
    PX4_SYSTEM_TYPE_FLAPPING_WING = 16,
    PX4_SYSTEM_TYPE_KITE = 17,
    PX4_SYSTEM_TYPE_ONBOARD_CONTROLLER = 18,
    PX4_SYSTEM_TYPE_VTOL_DUOROTOR = 19,
    PX4_SYSTEM_TYPE_VTOL_QUADROTOR = 20,
    PX4_SYSTEM_TYPE_VTOL_TILTROTOR = 21,
    PX4_SYSTEM_TYPE_VTOL_RESERVED2 = 22,
    PX4_SYSTEM_TYPE_VTOL_RESERVED3 = 23,
    PX4_SYSTEM_TYPE_VTOL_RESERVED4 = 24,
    PX4_SYSTEM_TYPE_VTOL_RESERVED5 = 25,
    PX4_SYSTEM_TYPE_GIMBAL = 26,
    PX4_SYSTEM_TYPE_ADSB = 27


class PX4_RcInMode_t(Enum):
    PX4_RC_IN_MODE_DEFAULT = 0,
    PX4_RC_IN_MODE_OFF = 1,
    PX4_RC_IN_MODE_GENERATED = 2


class PX4_NavigationState_t(Enum):
    PX4_NAVIGATION_STATE_MANUAL = 0
    PX4_NAVIGATION_STATE_ALTCTL = 1
    PX4_NAVIGATION_STATE_POSCTL = 2
    PX4_NAVIGATION_STATE_AUTO_MISSION = 3
    PX4_NAVIGATION_STATE_AUTO_LOITER = 4
    PX4_NAVIGATION_STATE_AUTO_RTL = 5
    PX4_NAVIGATION_STATE_AUTO_RCRECOVER = 6
    PX4_NAVIGATION_STATE_AUTO_RTGS = 7
    PX4_NAVIGATION_STATE_AUTO_LANDENGFAIL = 8
    PX4_NAVIGATION_STATE_AUTO_LANDGPSFAIL = 9
    PX4_NAVIGATION_STATE_ACRO = 10
    PX4_NAVIGATION_STATE_UNUSED = 11
    PX4_NAVIGATION_STATE_DESCEND = 12
    PX4_NAVIGATION_STATE_TERMINATION = 13
    PX4_NAVIGATION_STATE_OFFBOARD = 14
    PX4_NAVIGATION_STATE_STAB = 15
    PX4_NAVIGATION_STATE_RATTITUDE = 16
    PX4_NAVIGATION_STATE_AUTO_TAKEOFF = 17
    PX4_NAVIGATION_STATE_AUTO_LAND = 18
    PX4_NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19
    PX4_NAVIGATION_STATE_AUTO_PRECLAND = 20
    PX4_NAVIGATION_STATE_MAX = 21


class PX4_VehicleStatusMsg_t():
    Timestamp: int = 0
    SystemID: int = 0
    ComponentID: int = 0
    OnboardControlSensorsPresent: int = 0
    OnboardControlSensorsEnabled: int = 0
    OnboardControlSensorsHealth: int = 0
    NavState: PX4_NavigationState_t = PX4_NavigationState_t.PX4_NAVIGATION_STATE_MANUAL.name
    ArmingState: PX4_ArmingState_t = 0
    HilState: PX4_HilState_t = 0
    Failsafe: bool = True
    SystemType: PX4_SystemType_t = 0
    IsRotaryWing: bool = True
    IsVtol: bool = True
    VtolFwPermanentStab: bool = True
    InTransitionMode: bool = True
    RcSignalLost: bool = True
    RcInputMode: PX4_RcInMode_t = 0
    DataLinkLost: bool = True
    DataLinkLostCounter: int = 0
    EngineFailure: bool = True
    EngineFailureCmd: bool = True
    MissionFailure: bool = True


class PX4_VehicleLandDetectedMsg_t():
    Timestamp: int = 0
    AltMax: float = 0
    Landed: bool = False
    Freefall: bool = False
    GroundContact: bool = False


class PX4_VehicleCmd_t(Enum):
    PX4_VEHICLE_CMD_NONE = 0
    PX4_VEHICLE_CMD_CUSTOM_0 = 1
    PX4_VEHICLE_CMD_CUSTOM_1 = 2
    PX4_VEHICLE_CMD_CUSTOM_2 = 3
    PX4_VEHICLE_CMD_SET_ATTITUDE = 4
    PX4_VEHICLE_CMD_WAIT_FOR_ATP = 5
    PX4_VEHICLE_CMD_NAV_WAYPOINT = 16
    PX4_VEHICLE_CMD_NAV_LOITER_UNLIM = 17
    PX4_VEHICLE_CMD_NAV_LOITER_TURNS = 18
    PX4_VEHICLE_CMD_NAV_LOITER_TIME = 19
    PX4_VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20
    PX4_VEHICLE_CMD_NAV_LAND = 21
    PX4_VEHICLE_CMD_NAV_TAKEOFF = 22
    PX4_VEHICLE_CMD_NAV_LOITER_TO_ALT = 31
    PX4_VEHICLE_CMD_NAV_ROI = 80
    PX4_VEHICLE_CMD_NAV_PATHPLANNING = 81
    PX4_VEHICLE_CMD_NAV_VTOL_TAKEOFF = 84
    PX4_VEHICLE_CMD_NAV_VTOL_LAND = 85
    PX4_VEHICLE_CMD_NAV_GUIDED_LIMITS = 90
    PX4_VEHICLE_CMD_NAV_GUIDED_MASTER = 91
    PX4_VEHICLE_CMD_NAV_GUIDED_ENABLE = 92
    PX4_VEHICLE_CMD_NAV_LAST = 95
    PX4_VEHICLE_CMD_CONDITION_DELAY = 112
    PX4_VEHICLE_CMD_CONDITION_CHANGE_ALT = 113
    PX4_VEHICLE_CMD_CONDITION_DISTANCE = 114
    PX4_VEHICLE_CMD_CONDITION_YAW = 115
    PX4_VEHICLE_CMD_CONDITION_LAST = 159
    PX4_VEHICLE_CMD_DO_SET_MODE = 176
    PX4_VEHICLE_CMD_DO_JUMP = 177
    PX4_VEHICLE_CMD_DO_CHANGE_SPEED = 178
    PX4_VEHICLE_CMD_DO_SET_HOME = 179
    PX4_VEHICLE_CMD_DO_SET_PARAMETER = 180
    PX4_VEHICLE_CMD_DO_SET_RELAY = 181
    PX4_VEHICLE_CMD_DO_REPEAT_RELAY = 182
    PX4_VEHICLE_CMD_DO_SET_SERVO = 183
    PX4_VEHICLE_CMD_DO_REPEAT_SERVO = 184
    PX4_VEHICLE_CMD_DO_FLIGHTTERMINATION = 185
    PX4_VEHICLE_CMD_DO_GO_AROUND = 191
    PX4_VEHICLE_CMD_DO_REPOSITION = 192
    PX4_VEHICLE_CMD_DO_PAUSE_CONTINUE = 193
    PX4_VEHICLE_CMD_DO_CONTROL_VIDEO = 200
    PX4_VEHICLE_CMD_DO_DIGICAM_CONTROL = 203
    PX4_VEHICLE_CMD_DO_MOUNT_CONFIGURE = 204
    PX4_VEHICLE_CMD_DO_MOUNT_CONTROL = 205
    PX4_VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST = 206
    PX4_VEHICLE_CMD_DO_FENCE_ENABLE = 207
    PX4_VEHICLE_CMD_DO_PARACHUTE = 208
    PX4_VEHICLE_CMD_DO_INVERTED_FLIGHT = 210
    PX4_VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT = 220
    PX4_VEHICLE_CMD_DO_GUIDED_MASTER = 221
    PX4_VEHICLE_CMD_DO_GUIDED_LIMITS = 222
    PX4_VEHICLE_CMD_DO_LAST = 240
    PX4_VEHICLE_CMD_PREFLIGHT_CALIBRATION = 241
    PX4_VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242
    PX4_VEHICLE_CMD_PREFLIGHT_UAVCAN = 243
    PX4_VEHICLE_CMD_PREFLIGHT_STORAGE = 245
    PX4_VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
    PX4_VEHICLE_CMD_OVERRIDE_GOTO = 252
    PX4_VEHICLE_CMD_MISSION_START = 300
    PX4_VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
    PX4_VEHICLE_CMD_START_RX_PAIR = 500
    PX4_VEHICLE_CMD_DO_TRIGGER_CONTROL = 2003
    PX4_VEHICLE_CMD_DO_VTOL_TRANSITION = 3000
    PX4_VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY = 30001
    PX4_VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY = 30002


class PX4_VehicleCommandMsg_t():
    Timestamp: int = 0
    Param5: float = 0
    Param6: float = 0
    Param1: float = 0
    Param2: float = 0
    Param3: float = 0
    Param4: float = 0
    Param7: float = 0
    Command: PX4_VehicleCmd_t = 0
    TargetSystem: int = 0
    TargetComponent: int = 0
    SourceSystem: int = 0
    SourceComponent: int = 0
    Confirmation: int = 0


# #*
#  * \brief Return to Launch states
#  */
class RTLState(Enum):
    # ! The vehicle is in idle state during RTL navigation */
    RTL_STATE_NONE = 0
    # ! The vehicle is in climb state during RTL navigation */
    RTL_STATE_CLIMB = 1
    # ! The vehicle is in Return to home latitude and longitude state during RTL navigation */
    RTL_STATE_RETURN = 2
    # !  VTOL to MC transition state during RTL navigation */
    RTL_STATE_TRANSITION_TO_MC = 3
    # ! The vehicle is in descend state during RTL navigation */
    RTL_STATE_DESCEND = 4
    # ! The vehicle is in loiter state during RTL navigation */
    RTL_STATE_LOITER = 5
    # ! The vehicle is in land state during RTL navigation */
    RTL_STATE_LAND = 6
    # ! The vehicle is in landed state during RTL navigation */
    RTL_STATE_LANDED = 7


class PX4_SetpointType_t(Enum):
    PX4_SETPOINT_TYPE_POSITION = 0
    PX4_SETPOINT_TYPE_VELOCITY = 1
    PX4_SETPOINT_TYPE_LOITER = 2
    PX4_SETPOINT_TYPE_TAKEOFF = 3
    PX4_SETPOINT_TYPE_LAND = 4
    PX4_SETPOINT_TYPE_IDLE = 5
    PX4_SETPOINT_TYPE_OFFBOARD = 6
    PX4_SETPOINT_TYPE_FOLLOW_TARGET = 7
    PX4_SETPOINT_TYPE_ATTITUDE = 8


class PX4_PositionSetpoint_t():
    def __init__(self):
        self.Timestamp: int = 0
        self.Lat: float = 0
        self.Lon: float = 0
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
        self.Type: PX4_SetpointType_t = 0
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


class PX4_VehicleLocalPositionMsg_t():
    def __init__(self):
        self.Timestamp: int = 0
        self.RefTimestamp: int = 0
        self.RefLat: float = 0
        self.RefLon: float = 0
        self.SurfaceBottomTimestamp: int = 0
        self.X: float = 0
        self.Y: float = 0
        self.Z: float = 0
        self.Delta_XY = [0, 0]
        self.Delta_Z: float = 0
        self.VX: float = 0
        self.VY: float = 0
        self.VZ: float = 0
        self.Delta_VXY = [0, 0]
        self.Delta_VZ: float = 0
        self.AX: float = 0
        self.AY: float = 0
        self.AZ: float = 0
        self.Yaw: float = 0
        self.RefAlt: float = 0
        self.DistBottom: float = 0
        self.DistBottomRate: float = 0
        self.EpH: float = 0
        self.EpV: float = 0
        self.EvH: float = 0
        self.EvV: float = 0
        self.EstimatorType: int = 0
        self.XY_Valid: bool = True
        self.Z_Valid: bool = True
        self.V_XY_Valid: bool = True
        self.V_Z_Valid: bool = True
        self.XY_ResetCounter: int = 0
        self.Z_ResetCounter: int = 0
        self.VXY_ResetCounter: int = 0
        self.VZ_ResetCounter: int = 0
        self.XY_Global: bool = True
        self.Z_Global: bool = True
        self.DistBottomValid: bool = True
