from enum import Enum
from flight.px4_lib import PX4_ArmingState_t, PX4_HomePositionMsg_t, PX4_MissionMsg_t, PX4_MissionResultMsg_t, \
    PX4_NavigationState_t, PX4_PositionSetpoint_t, PX4_PositionSetpointTripletMsg_t, PX4_SetpointType_t, \
    PX4_VehicleCmd_t, PX4_VehicleCommandMsg_t, PX4_VehicleGlobalPositionMsg_t, PX4_VehicleGpsPositionMsg_t, \
    PX4_VehicleLandDetectedMsg_t, PX4_VehicleLocalPositionMsg_t, PX4_VehicleStatusMsg_t, PX4LIB_GetPX4TimeUs, RTLState
import sys
import math

import json

from pyliner.apps.communication import Communication
from pyliner.telemetry import SetpointTriplet, Telemetry


# /**
#  * \brief mission origin
#  */
class Origin(Enum):
    # /**! Mavlink originated mission item */
    ORIGIN_MAVLINK = 0,
    # /*! Onboard originated mission item */
    ORIGIN_ONBOARD = 1


class NAV_CurrentValueTable_t():
    def __init__(self) -> None:
        # /** \brief The home position message */
        self.HomePositionMsg: PX4_HomePositionMsg_t = PX4_HomePositionMsg_t()
        # /** \brief The mission message */
        self.MissionMsg: PX4_HomePositionMsg_t = PX4_HomePositionMsg_t()
        # /** \brief The position message from GPS */
        self.VehicleGpsPositionMsg: PX4_VehicleGpsPositionMsg_t = PX4_VehicleGpsPositionMsg_t()
        # /** \brief The global position message */
        self.VehicleGlobalPosition: PX4_VehicleGlobalPositionMsg_t = PX4_VehicleGlobalPositionMsg_t()
        # /** \brief The vehicle status message */
        self.VehicleStatusMsg: PX4_VehicleStatusMsg_t = PX4_VehicleStatusMsg_t()
        # /** \brief The land detection message */
        self.VehicleLandDetectedMsg: PX4_VehicleLandDetectedMsg_t = PX4_VehicleLandDetectedMsg_t()
        # /** \brief The vehicle local position message */
        self.VehicleLocalPositionMsg: PX4_VehicleLocalPositionMsg_t = PX4_VehicleLocalPositionMsg_t()
        # /** \brief The vehicle command message */
        self.VehicleCommandMsg: PX4_VehicleCommandMsg_t = PX4_VehicleCommandMsg_t()


class MissionItem:
    def __init__(self):
        # \brief The latitude in degrees #
        self.Lat = 0
        # \brief The longitude in degrees #
        self.Lon = 0
        # \brief The time that the MAV should stay inside the radius before advancing in seconds #
        self.TimeInside = 0
        # \brief The minimal pitch angle for fixed wing takeoff waypoints #
        self.PitchMin = 0
        # \brief The default radius in which the mission is accepted as reached in meters #
        self.AcceptanceRadius = 0
        # \brief lThe oiter radius in meters, 0 for a VTOL to hover, negative for counter-clockwise #
        self.LoiterRadius = 0
        # \brief The in radians NED -PI..+PI, NAN means don't change yaw. By "yaw" we mean heading here. #
        self.Yaw = 0
        self.YawBody = 0
        self.PitchBody = 0  # Body pitch in Theta mode. Alpha in Alpha Mode. Alpha/Theta mode can be set in FAC table.
        self.RollBody = 0
        # \brief The latitude padding #
        self.LatPadding = 0
        # \brief The longitude padding #
        self.LonPadding = 0
        # \brief The altitude in meters    (AMSL) #
        self.Altitude = 0
        self.CruisingSpeed = 0
        # \brief The array to store mission command values for MAV_FRAME_MISSION #
        self.Params = []
        # \brief The navigation command #
        self.NavCmd = 0
        # \brief The index where the do jump will go to #
        self.DoJumpMissionIndex = 0
        # \brief The how many times do jump needs to be done #
        self.DoJumpRepeatCount = 0
        # \brief The count how many times the jump has been done #
        self.DoJumpCurrentCount = 0
        # \brief The mission frame #
        self.Frame = 0
        # \brief How the mission item was generated #
        self.Origin = 0
        # \brief The exit xtrack location: 0 for center of loiter wp, 1 for exit location #
        self.LoiterExitXTrack = 0
        # \brief The heading needs to be reached #
        self.ForceHeading = 0
        # \brief True if altitude is relative from start point #
        self.AltitudeIsRelative = 0
        # \brief True if next waypoint should follow after this one #
        self.AutoContinue = 0
        # \brief Disables multi-copter yaw with this flag #
        self.DisableMcYaw = 0
        # \brief Number of milliseconds to delay until proceeding to the next mission item #
        self.DelayTime = 0


class NAV_HkTlm_t():
    def __init__(self):
        # * \navtlmmnemonic \NAV_CMDACPTCNT
        #  \brief Count of accepted commands #
        self.usCmdCnt: int = 0

        # * \navtlmmnemonic \NAV_CMDRJCTCNT
        # \brief Count of failed commands #
        self.usCmdErrCnt: int = 0

        # \brief Current nav state #
        self.NavState: PX4_NavigationState_t = 0

        # \brief RTL state  #
        self.RtlState: RTLState = 0

        # \brief Flag for if current mission waypoint yaw is reached #
        self.WaypointYawReached: bool = False

        # \brief Force descent flag #
        self.RtlForceDescentExecuting: bool = False

        # \brief Force descent completed flag #
        self.RtlForceDescentCompleted: bool = False

        # \brief Flag for current mission item reached #
        self.MissionItemReached: bool

        # * \brief True if we're waiting for an Authorization to Proceed command #
        self.WaitingForATP: bool = False

        # * \brief The time a Mission Item action started. #
        self.ActionStart: int = 0

        # * \brief The time a Mission Item action started. #
        self.ReleaseDelayAt: int = 0

        # * \brief The number of milliseconds NAV is waiting to proceed to the next Mission Item. #
        self.RemainingDelay: int = 0

        # \brief Flag for if current mission waypoint position is reached #
        self.WaypointPositionReached: bool = False

        self.AutoMissionItemIndex: int = 0

        # \brief Id for current mission #
        self.MissionID: str = ""


class Nav():
    def __init__(self, new_comms: Communication) -> None:
        self.NAV_NO_JUMP = -1
        self.NAV_MISSION_ITEM_MAX = 512
        self.NAV_EPSILON_POSITION = 0.001
        self.M_PI_2_F = 1.57079632
        self.DELAY_SIGMA = 0.01
        self.NAV_LAT_SHORT_FORM = 1000
        self.NAV_LON_SHORT_FORM = 1000
        self.CONVERT_DECIMAL_DEGREES = 1e7

        # Should be move to JSON config
        self.ConfigTblPtr = {"NAV_LOITER_RAD": 100.0}

        self.comms: Communication = new_comms

        self.HkTlm: NAV_HkTlm_t = NAV_HkTlm_t()
        # # \brief Output Data published at the end of cycle */
        # # \brief The mission result message */
        self.MissionResultMsg: PX4_MissionResultMsg_t = PX4_MissionResultMsg_t()

        self.PreviousState: PX4_VehicleStatusMsg_t = PX4_VehicleStatusMsg_t()

        # /** \brief The position set point triplet message */
        self.PositionSetpointTripletMsg: PX4_PositionSetpointTripletMsg_t = PX4_PositionSetpointTripletMsg_t()

        self.MissionItem: MissionItem = MissionItem()
        self.MissionItems = []
        # //osalbool missionStarted
        # //osalbool waypointStarted
        # \brief Flag is set to true if a previously unseen command is encountered */
        self.NewCommandArrived: bool = False
        # \brief Will allow to loiter at setpoint */
        self.CanLoiterAtSetpoint: bool = False
        # \brief True if loiter position is set */
        self.LoiterPositionSet: bool = False
        self.RtlState: RTLState = RTLState(0)
        # \brief Default time first time inside orbit in mission item */
        self.TimeFirstInsideOrbit: int = 0
        # \brief Default time for waypoint reached in mission item */
        self.TimeWpReached: int = 0
        # \brief Default mission cruising speed in mission item */
        self.MissionCruisingSpeed: float = 0
        # \brief Default mission throttle in mission item */
        self.MissionThrottle: float = 0
        # \brief True if vehicle status message is updated once by navigation states */
        self.VehicleStatusUpdateOnce: bool = False
        # \brief True if mission result message is updated by navigation states */
        self.MissionResultUpdated: bool = False
        # \brief True if position setpoint triplet message is updated by navigation states */
        self.PositionSetpointTripletUpdated: bool = False
        self.ForceDescentTarget: float = 0

        self.JsonLoaded = False  # This replaces "MissionTblPtr" check in original NAV code
        self.CVT: NAV_CurrentValueTable_t = NAV_CurrentValueTable_t()

        # /** \brief A temporary triplet type message to store reposition triplet  */
        self.RepositionTripletMsg: PX4_PositionSetpointTripletMsg_t = PX4_PositionSetpointTripletMsg_t()

    def NewMissionItem(self, JSONItem: dict) -> MissionItem:
        NewItem: MissionItem = MissionItem()
        # \brief The latitude in degrees #
        NewItem.Lat = float(JSONItem["Lat"])
        # \brief The longitude in degrees #
        NewItem.Lon = float(JSONItem["Lon"])
        # \brief The time that the MAV should stay inside the radius before advancing in seconds #
        NewItem.TimeInside = float(JSONItem["TimeInside"])
        # \brief The minimal pitch angle for fixed wing takeoff waypoints #
        NewItem.PitchMin = float(JSONItem["PitchMin"])
        # \brief The default radius in which the mission is accepted as reached in meters #
        NewItem.AcceptanceRadius = float(JSONItem["AcceptanceRadius"])
        # \brief lThe oiter radius in meters, 0 for a VTOL to hover, negative for counter-clockwise #
        NewItem.LoiterRadius = float(JSONItem["LoiterRadius"])
        # \brief The in radians NED -PI..+PI, NAN means don't change yaw. By "yaw" we mean heading here. #
        NewItem.Yaw = float(JSONItem["Yaw"])
        NewItem.YawBody = float(JSONItem["YawBody"])
        NewItem.PitchBody = float(JSONItem[
                                      "PitchBody"])  # Body pitch in Theta mode. Alpha in Alpha Mode. Alpha/Theta mode can be set in FAC table.
        NewItem.RollBody = float(JSONItem["RollBody"])
        # \brief The latitude padding #
        NewItem.LatPadding = float(JSONItem["LatPadding"])
        # \brief The longitude padding #
        NewItem.LonPadding = float(JSONItem["LonPadding"])
        # \brief The altitude in meters    (AMSL) #
        NewItem.Altitude = float(JSONItem["Altitude"])
        NewItem.CruisingSpeed = float(JSONItem["CruisingSpeed"])
        # \brief The array to store mission command values for MAV_FRAME_MISSION #
        NewItem.Params = JSONItem["Params"]
        # \brief The navigation command #
        # PX4_NavigationState_t[self.CVT.VehicleStatusMsg.NavState]
        NewItem.NavCmd = PX4_VehicleCmd_t[JSONItem["NavCmd"]]
        # \brief The index where the do jump will go to #
        NewItem.DoJumpMissionIndex = int(JSONItem["DoJumpMissionIndex"])
        # \brief The how many times do jump needs to be done #
        NewItem.DoJumpRepeatCount = float(JSONItem["DoJumpRepeatCount"])
        # \brief The count how many times the jump has been done #
        NewItem.DoJumpCurrentCount = float(JSONItem["DoJumpCurrentCount"])
        # \brief The mission frame #
        NewItem.Frame = float(JSONItem["Frame"])
        # \brief How the mission item was generated #
        NewItem.Origin = JSONItem["Origin"]
        # \brief The exit xtrack location: 0 for center of loiter wp, 1 for exit location #
        NewItem.LoiterExitXTrack = float(JSONItem["LoiterExitXTrack"])
        # \brief The heading needs to be reached #
        NewItem.ForceHeading = bool(JSONItem["ForceHeading"])
        # \brief True if altitude is relative from start point #
        NewItem.AltitudeIsRelative = bool(JSONItem["AltitudeIsRelative"])
        # \brief True if next waypoint should follow after this one #
        NewItem.AutoContinue = bool(JSONItem["AutoContinue"])
        # \brief Disables multi-copter yaw with this flag #
        NewItem.DisableMcYaw = bool(JSONItem["DisableMcYaw"])
        # \brief Number of milliseconds to delay until proceeding to the next mission item #
        NewItem.DelayTime = float(JSONItem["DelayTime"])

        return NewItem

    def NewEmptyMissionItem(self) -> MissionItem:
        NewItem: MissionItem = MissionItem()
        # \brief The latitude in degrees #
        NewItem.Lat = float(0.0)
        # \brief The longitude in degrees #
        NewItem.Lon = float(0.0)
        # \brief The time that the MAV should stay inside the radius before advancing in seconds #
        NewItem.TimeInside = float(0.0)
        # \brief The minimal pitch angle for fixed wing takeoff waypoints #
        NewItem.PitchMin = float(0.0)
        # \brief The default radius in which the mission is accepted as reached in meters #
        NewItem.AcceptanceRadius = float(0.0)
        # \brief lThe oiter radius in meters, 0 for a VTOL to hover, negative for counter-clockwise #
        NewItem.LoiterRadius = float(0.0)
        # \brief The in radians NED -PI..+PI, NAN means don't change yaw. By "yaw" we mean heading here. #
        NewItem.Yaw = float(0.0)
        NewItem.YawBody = float(0.0)
        NewItem.PitchBody = float(
            0.0)  # Body pitch in Theta mode. Alpha in Alpha Mode. Alpha/Theta mode can be set in FAC table.
        NewItem.RollBody = float(0.0)
        # \brief The latitude padding #
        NewItem.LatPadding = float(0.0)
        # \brief The longitude padding #
        NewItem.LonPadding = float(0.0)
        # \brief The altitude in meters    (AMSL) #
        NewItem.Altitude = float(0.0)
        NewItem.CruisingSpeed = float(0.0)
        # \brief The array to store mission command values for MAV_FRAME_MISSION #
        NewItem.Params = []
        # \brief The navigation command #
        # PX4_NavigationState_t[self.CVT.VehicleStatusMsg.NavState]
        NewItem.NavCmd = PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NONE
        # \brief The index where the do jump will go to #
        NewItem.DoJumpMissionIndex = int(0)
        # \brief The how many times do jump needs to be done #
        NewItem.DoJumpRepeatCount = float(0)
        # \brief The count how many times the jump has been done #
        NewItem.DoJumpCurrentCount = float(0)
        # \brief The mission frame #
        NewItem.Frame = float(0)
        # \brief How the mission item was generated #
        NewItem.Origin = Origin.ORIGIN_ONBOARD
        # \brief The exit xtrack location: 0 for center of loiter wp, 1 for exit location #
        NewItem.LoiterExitXTrack = float(0)
        # \brief The heading needs to be reached #
        NewItem.ForceHeading = bool(False)
        # \brief True if altitude is relative from start point #
        NewItem.AltitudeIsRelative = bool(False)
        # \brief True if next waypoint should follow after this one #
        NewItem.AutoContinue = bool(False)
        # \brief Disables multi-copter yaw with this flag #
        NewItem.DisableMcYaw = bool(False)
        # \brief Number of milliseconds to delay until proceeding to the next mission item #
        NewItem.DelayTime = float(0)

        return NewItem

    def LoadJSON(self, FilePath: str):
        with open(FilePath, "r") as read_file:
            self.JSONData = json.load(read_file)

        for item in self.JSONData["items"]:
            self.MissionItems.append(self.NewMissionItem(item))

        MissionItemsCount = len(self.JSONData["items"])
        # Doing it this way since NAV is written the "C" way and assumes that the mission array statically pre-allocated at compile time
        for item in range(self.NAV_MISSION_ITEM_MAX - MissionItemsCount):
            self.MissionItems.append(self.NewEmptyMissionItem())

        if self.JSONData and (MissionItemsCount > 0):
            self.JsonLoaded = True
            self.HkTlm.MissionID = self.JSONData['MissionName']
            print(f"Loaded '{self.HkTlm.MissionID}' Mission")
            print(f"MissionItems Count:{MissionItemsCount}")

    def StateChangeDetect(self):

        # /* When there is a change in state reset fail safe flag */
        CurrentState: PX4_VehicleStatusMsg_t = self.CVT.VehicleStatusMsg
        StateChange: bool = False

        if self.PreviousState is None:
            self.MissionResultMsg.StayInFailsafe = False
            self.PreviousState = CurrentState
            StateChange = True
        elif not (self.PreviousState.NavState == CurrentState.NavState):
            self.MissionResultMsg.StayInFailsafe = False
            self.PreviousState = CurrentState
            StateChange = True

        return StateChange

    def Takeoff(self):
        # TODO:Implement
        pass

    def Loiter(self):
        # TODO:Implement
        pass

    def Land(self):
        # TODO:Implement
        pass

    def Rtl(self):
        # TODO:Implement
        pass

    def TakeoffActive(self):
        # TODO:Implement
        pass

    def LoiterActive(self):
        # TODO:Implement
        pass

    def LandActive(self):
        # TODO:Implement
        pass

    def RtlActive(self):
        # TODO:Implement
        pass

    def SendPositionSetpointTripletMsg(self):
        # TODO:Add timestamp to prev, curr, next
        # find_aggregate_param_type(self, type_name: str, namespace: str) -> Union[xtce.AggregateParameterType, None]:
        # print(f"self.PositionSetpointTripletMsg.Previous.Type:{type(self.PositionSetpointTripletMsg.Previous.Type)}")
        current_timestamp = self.PositionSetpointTripletMsg.Timestamp

        SetPoint = Telemetry(
            # Previous_Timestamp = self.PositionSetpointTripletMsg.Previous.Timestamp,
            '/cfs/cpd/apps/px4lib/PX4_POSITION_SETPOINT_TRIPLET_MID',

            Timestamp=self.PositionSetpointTripletMsg.Timestamp,
            Previous_Timestamp=self.PositionSetpointTripletMsg.Previous.Timestamp,
            Previous_Lat=self.PositionSetpointTripletMsg.Previous.Lat,
            Previous_Lon=self.PositionSetpointTripletMsg.Previous.Lon,
            Previous_X=self.PositionSetpointTripletMsg.Previous.X,
            Previous_Y=self.PositionSetpointTripletMsg.Previous.Y,
            Previous_Z=self.PositionSetpointTripletMsg.Previous.Z,
            Previous_VX=self.PositionSetpointTripletMsg.Previous.VX,
            Previous_VY=self.PositionSetpointTripletMsg.Previous.VY,
            Previous_VZ=self.PositionSetpointTripletMsg.Previous.VZ,
            Previous_Alt=self.PositionSetpointTripletMsg.Previous.Alt,
            Previous_Yaw=math.radians(self.PositionSetpointTripletMsg.Previous.Yaw),
            Previous_Yawspeed=self.PositionSetpointTripletMsg.Previous.Yawspeed,
            Previous_YawBody=math.radians(self.PositionSetpointTripletMsg.Previous.YawBody),
            Previous_PitchBody=math.radians(self.PositionSetpointTripletMsg.Previous.PitchBody),
            Previous_RollBody=math.radians(self.PositionSetpointTripletMsg.Previous.RollBody),
            Previous_LoiterRadius=self.PositionSetpointTripletMsg.Previous.LoiterRadius,
            Previous_PitchMin=self.PositionSetpointTripletMsg.Previous.PitchMin,
            Previous_AX=self.PositionSetpointTripletMsg.Previous.AX,
            Previous_AY=self.PositionSetpointTripletMsg.Previous.AY,
            Previous_AZ=self.PositionSetpointTripletMsg.Previous.AZ,
            Previous_AcceptanceRadius=self.PositionSetpointTripletMsg.Previous.AcceptanceRadius,
            Previous_CruisingSpeed=self.PositionSetpointTripletMsg.Previous.CruisingSpeed,
            Previous_CruisingThrottle=self.PositionSetpointTripletMsg.Previous.CruisingThrottle,
            Previous_Valid=self.PositionSetpointTripletMsg.Previous.Valid,
            Previous_Type=self.PositionSetpointTripletMsg.Previous.Type.value,
            Previous_PositionValid=self.PositionSetpointTripletMsg.Previous.PositionValid,
            Previous_VelocityValid=self.PositionSetpointTripletMsg.Previous.VelocityValid,
            Previous_VelocityFrame=self.PositionSetpointTripletMsg.Previous.VelocityFrame,
            Previous_AltValid=self.PositionSetpointTripletMsg.Previous.AltValid,
            Previous_YawValid=self.PositionSetpointTripletMsg.Previous.YawValid,
            Previous_DisableMcYawControl=self.PositionSetpointTripletMsg.Previous.DisableMcYawControl,
            Previous_YawspeedValid=self.PositionSetpointTripletMsg.Previous.YawspeedValid,
            Previous_LoiterDirection=self.PositionSetpointTripletMsg.Previous.LoiterDirection,
            Previous_AccelerationValid=self.PositionSetpointTripletMsg.Previous.AccelerationValid,
            Previous_AccelerationIsForce=self.PositionSetpointTripletMsg.Previous.AccelerationIsForce,

            Current_Timestamp=self.PositionSetpointTripletMsg.Current.Timestamp,
            Current_Lat=self.PositionSetpointTripletMsg.Current.Lat,
            Current_Lon=self.PositionSetpointTripletMsg.Current.Lon,
            Current_X=self.PositionSetpointTripletMsg.Current.X,
            Current_Y=self.PositionSetpointTripletMsg.Current.Y,
            Current_Z=self.PositionSetpointTripletMsg.Current.Z,
            Current_VX=self.PositionSetpointTripletMsg.Current.VX,
            Current_VY=self.PositionSetpointTripletMsg.Current.VY,
            Current_VZ=self.PositionSetpointTripletMsg.Current.VZ,
            Current_Alt=self.PositionSetpointTripletMsg.Current.Alt,
            Current_Yaw=math.radians(self.PositionSetpointTripletMsg.Current.Yaw),
            Current_Yawspeed=self.PositionSetpointTripletMsg.Current.Yawspeed,
            Current_YawBody=math.radians(self.PositionSetpointTripletMsg.Current.YawBody),
            Current_PitchBody=math.radians(self.PositionSetpointTripletMsg.Current.PitchBody),
            Current_RollBody=math.radians(self.PositionSetpointTripletMsg.Current.RollBody),
            Current_LoiterRadius=self.PositionSetpointTripletMsg.Current.LoiterRadius,
            Current_PitchMin=self.PositionSetpointTripletMsg.Current.PitchMin,
            Current_AX=self.PositionSetpointTripletMsg.Current.AX,
            Current_AY=self.PositionSetpointTripletMsg.Current.AY,
            Current_AZ=self.PositionSetpointTripletMsg.Current.AZ,
            Current_AcceptanceRadius=self.PositionSetpointTripletMsg.Current.AcceptanceRadius,
            Current_CruisingSpeed=self.PositionSetpointTripletMsg.Current.CruisingSpeed,
            Current_CruisingThrottle=self.PositionSetpointTripletMsg.Current.CruisingThrottle,
            Current_Valid=self.PositionSetpointTripletMsg.Current.Valid,
            Current_Type=self.PositionSetpointTripletMsg.Current.Type.value,
            Current_PositionValid=self.PositionSetpointTripletMsg.Current.PositionValid,
            Current_VelocityValid=self.PositionSetpointTripletMsg.Current.VelocityValid,
            Current_VelocityFrame=self.PositionSetpointTripletMsg.Current.VelocityFrame,
            Current_AltValid=self.PositionSetpointTripletMsg.Current.AltValid,
            Current_YawValid=self.PositionSetpointTripletMsg.Current.YawValid,
            Current_DisableMcYawControl=self.PositionSetpointTripletMsg.Current.DisableMcYawControl,
            Current_YawspeedValid=self.PositionSetpointTripletMsg.Current.YawspeedValid,
            Current_LoiterDirection=self.PositionSetpointTripletMsg.Current.LoiterDirection,
            Current_AccelerationValid=self.PositionSetpointTripletMsg.Current.AccelerationValid,
            Current_AccelerationIsForce=self.PositionSetpointTripletMsg.Current.AccelerationIsForce,

            Next_Timestamp=self.PositionSetpointTripletMsg.Next.Timestamp,
            Next_Lat=self.PositionSetpointTripletMsg.Next.Lat,
            Next_Lon=self.PositionSetpointTripletMsg.Next.Lon,
            Next_X=self.PositionSetpointTripletMsg.Next.X,
            Next_Y=self.PositionSetpointTripletMsg.Next.Y,
            Next_Z=self.PositionSetpointTripletMsg.Next.Z,
            Next_VX=self.PositionSetpointTripletMsg.Next.VX,
            Next_VY=self.PositionSetpointTripletMsg.Next.VY,
            Next_VZ=self.PositionSetpointTripletMsg.Next.VZ,
            Next_Alt=self.PositionSetpointTripletMsg.Next.Alt,
            Next_Yaw=math.radians(self.PositionSetpointTripletMsg.Next.Yaw),
            Next_Yawspeed=self.PositionSetpointTripletMsg.Next.Yawspeed,
            Next_YawBody=math.radians(self.PositionSetpointTripletMsg.Next.YawBody),
            Next_PitchBody=math.radians(self.PositionSetpointTripletMsg.Next.PitchBody),
            Next_RollBody=math.radians(self.PositionSetpointTripletMsg.Next.RollBody),
            Next_LoiterRadius=self.PositionSetpointTripletMsg.Next.LoiterRadius,
            Next_PitchMin=self.PositionSetpointTripletMsg.Next.PitchMin,
            Next_AX=self.PositionSetpointTripletMsg.Next.AX,
            Next_AY=self.PositionSetpointTripletMsg.Next.AY,
            Next_AZ=self.PositionSetpointTripletMsg.Next.AZ,
            Next_AcceptanceRadius=self.PositionSetpointTripletMsg.Next.AcceptanceRadius,
            Next_CruisingSpeed=self.PositionSetpointTripletMsg.Next.CruisingSpeed,
            Next_CruisingThrottle=self.PositionSetpointTripletMsg.Next.CruisingThrottle,
            Next_Valid=self.PositionSetpointTripletMsg.Next.Valid,
            Next_Type=self.PositionSetpointTripletMsg.Next.Type.value,
            Next_PositionValid=self.PositionSetpointTripletMsg.Next.PositionValid,
            Next_VelocityValid=self.PositionSetpointTripletMsg.Next.VelocityValid,
            Next_VelocityFrame=self.PositionSetpointTripletMsg.Next.VelocityFrame,
            Next_AltValid=self.PositionSetpointTripletMsg.Next.AltValid,
            Next_YawValid=self.PositionSetpointTripletMsg.Next.YawValid,
            Next_DisableMcYawControl=self.PositionSetpointTripletMsg.Next.DisableMcYawControl,
            Next_YawspeedValid=self.PositionSetpointTripletMsg.Next.YawspeedValid,
            Next_LoiterDirection=self.PositionSetpointTripletMsg.Next.LoiterDirection,
            Next_AccelerationValid=self.PositionSetpointTripletMsg.Next.AccelerationValid,
            Next_AccelerationIsForce=self.PositionSetpointTripletMsg.Next.AccelerationIsForce,
        )
        self.comms.send_message(SetPoint)
        pass

    def Execute(self):
        Now: int = 0
        # /* Set vehicle arming state */
        if self.CVT.VehicleStatusMsg.Timestamp != 0 and not self.VehicleStatusUpdateOnce:
            self.CVT.VehicleStatusMsg.ArmingState = PX4_ArmingState_t.PX4_ARMING_STATE_STANDBY
            self.VehicleStatusUpdateOnce = True

        # # /* Execute only on command event*/
        # if self.NewCommandArrived:
        #     # /* Reset new command flag*/
        #     self.NewCommandArrived = False
        #     # /* Configure messages on command receipt */
        #     if self.CVT.VehicleCommandMsg.Command == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_DO_REPOSITION:
        #         # /* EVENT: DO REPOSITION
        #         # * store current position as previous and goal as next */
        #         self.RepositionTripletMsg.Previous.Yaw = self.CVT.VehicleGlobalPosition.Yaw
        #         self.RepositionTripletMsg.Previous.Lat = self.CVT.VehicleGlobalPosition.Lat
        #         self.RepositionTripletMsg.Previous.Lon = self.CVT.VehicleGlobalPosition.Lon
        #         self.RepositionTripletMsg.Previous.Alt = self.CVT.VehicleGlobalPosition.Alt

        #         # /* Store new current position */
        #         self.RepositionTripletMsg.Current.LoiterRadius = self.ConfigTblPtr["NAV_LOITER_RAD"] 
        #         # /* Not clear as to what this 1 signifies, but PX4 still has it 
        #         # * it there as of 1.9 */
        #         self.RepositionTripletMsg.Current.LoiterDirection = 1
        #         self.RepositionTripletMsg.Current.Type = PX4_SetpointType_t.PX4_SETPOINT_TYPE_LOITER

        #         # /* Assign yaw to current position set point */
        #         if math.isfinite(self.CVT.VehicleCommandMsg.Param4):
        #             self.RepositionTripletMsg.Current.Yaw = self.CVT.VehicleCommandMsg.Param4
        #         else:
        #             self.RepositionTripletMsg.Current.Yaw = math.nan

        #         # /* Assign latitude and longitude to current set point */
        #         if math.isfinite(self.CVT.VehicleCommandMsg.Param5) and math.isfinite(self.CVT.VehicleCommandMsg.Param6):
        #             if self.CVT.VehicleCommandMsg.Param5 < self.NAV_LAT_SHORT_FORM:
        #                 self.RepositionTripletMsg.Current.Lat = self.CVT.VehicleCommandMsg.Param5 
        #             else:
        #                 self.RepositionTripletMsg.Current.Lat = self.CVT.VehicleCommandMsg.Param5 / self.CONVERT_DECIMAL_DEGREES
        #             if self.CVT.VehicleCommandMsg.Param6 < self.NAV_LON_SHORT_FORM:
        #                 self.RepositionTripletMsg.Current.Lon = self.CVT.VehicleCommandMsg.Param6 
        #             else:
        #                 self.RepositionTripletMsg.Current.Lon = self.CVT.VehicleCommandMsg.Param6 / self.CONVERT_DECIMAL_DEGREES

        #         else:
        #             self.RepositionTripletMsg.Current.Lat = self.CVT.VehicleGlobalPosition.Lat
        #             self.RepositionTripletMsg.Current.Lon = self.CVT.VehicleGlobalPosition.Lon

        #         # /* Assign altitude to current set point */
        #         if math.isfinite(self.CVT.VehicleCommandMsg.Param7):
        #             self.RepositionTripletMsg.Current.Alt = self.CVT.VehicleCommandMsg.Param7
        #         else:
        #             self.RepositionTripletMsg.Current.Alt = self.CVT.VehicleGlobalPosition.Alt
        #         # /* Assign set point triplet validity */
        #         self.RepositionTripletMsg.Previous.Valid = True
        #         self.RepositionTripletMsg.Current.Valid = True
        #         self.RepositionTripletMsg.Next.Valid = False

        #     elif self.CVT.VehicleCommandMsg.Command == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_TAKEOFF:
        #         # /* EVENT: TAKEOFF
        #         # * store of command hist */
        #         CacheCommandEventHist()

        #         /* Store current position as previous and goal as next */
        #         TakeoffTripletMsg.Previous.Yaw =
        #                 CVT.VehicleGlobalPosition.Yaw
        #         TakeoffTripletMsg.Previous.Lat =
        #                 CVT.VehicleGlobalPosition.Lat
        #         TakeoffTripletMsg.Previous.Lon =
        #                 CVT.VehicleGlobalPosition.Lon
        #         TakeoffTripletMsg.Previous.Alt =
        #                 CVT.VehicleGlobalPosition.Alt

        #         /* Store new current position */
        #         TakeoffTripletMsg.Current.LoiterRadius =
        #                 ConfigTblPtr->NAV_LOITER_RAD
        #         TakeoffTripletMsg.Current.LoiterDirection = 1
        #         TakeoffTripletMsg.Current.Type =
        #                 PX4_SetpointType_t.PX4_SETPOINT_TYPE_TAKEOFF

        #         /* Check if home position is valid, set current yaw and previous valid accordingly */
        #         if (CVT.HomePositionMsg.Timestamp > 0)
        #         {
        #             TakeoffTripletMsg.Current.Yaw =
        #                     CVT.VehicleCommandMsg.Param4
        #             TakeoffTripletMsg.Previous.Valid = TRUE
        #         }
        #         else
        #         {
        #             TakeoffTripletMsg.Current.Yaw =
        #                     CVT.VehicleLocalPositionMsg.Yaw
        #             TakeoffTripletMsg.Previous.Valid = FALSE
        #         }

        #         /*  Check if param5 and param6 is finite, set Latitude, Longitude, Altitude and current and next position set point validity */
        #         if (math.isfinite(CVT.VehicleCommandMsg.Param5) &&
        #             math.isfinite(CVT.VehicleCommandMsg.Param6))
        #         {
        #             TakeoffTripletMsg.Current.Lat =
        #                     (CVT.VehicleCommandMsg.Param5 < 1000) ?
        #                             CVT.VehicleCommandMsg.Param5 :
        #                             CVT.VehicleCommandMsg.Param5 / (double) 1e7
        #             TakeoffTripletMsg.Current.Lon =
        #                     (CVT.VehicleCommandMsg.Param6 < 1000) ?
        #                             CVT.VehicleCommandMsg.Param6 :
        #                             CVT.VehicleCommandMsg.Param6 / (double) 1e7
        #         }
        #         else
        #         {
        #             TakeoffTripletMsg.Current.Lat = NAN
        #             TakeoffTripletMsg.Current.Lon = NAN
        #         }

        #         /* Assign set point triplet validity */
        #         TakeoffTripletMsg.Current.Alt = CVT.VehicleCommandMsg.Param7
        #         TakeoffTripletMsg.Current.Valid = TRUE
        #         TakeoffTripletMsg.Next.Valid = FALSE
        #     }
        # }

        # /* Detect events for navigation actions. Find if a state is seen for first
        # * time or has been active since a while */
        CurrentState: PX4_NavigationState_t = PX4_NavigationState_t[self.CVT.VehicleStatusMsg.NavState]
        FirstRun: bool = self.StateChangeDetect()
        Active: bool = False
        if not (FirstRun):
            Active = True

        # /* If a state is inactive */
        if CurrentState != PX4_NavigationState_t.PX4_NAVIGATION_STATE_AUTO_LOITER:
            self.LoiterPositionSet = False

        if CurrentState != PX4_NavigationState_t.PX4_NAVIGATION_STATE_AUTO_RTL:
            self.RtlState = RTLState.RTL_STATE_NONE

        # /* First run in a navigation mode */
        if (FirstRun):
            if (CurrentState
                    == PX4_NavigationState_t.PX4_NAVIGATION_STATE_AUTO_TAKEOFF):
                # (void) CFE_EVS_SendEvent(NAV_ACTION_ST_EID, CFE_EVS_INFORMATION,
                #         "Commencing %s", "Takeoff")
                print(f"Commencing Takeoff")
                self.Takeoff()

            elif (CurrentState
                  == PX4_NavigationState_t.PX4_NAVIGATION_STATE_AUTO_LOITER):

                # (void) CFE_EVS_SendEvent(NAV_ACTION_ST_EID, CFE_EVS_INFORMATION,
                #         "Commencing %s", "Loiter")
                print("Commencing Loiter")
                self.Loiter()

            elif (CurrentState
                  == PX4_NavigationState_t.PX4_NAVIGATION_STATE_AUTO_LAND):
                # (void) CFE_EVS_SendEvent(NAV_ACTION_ST_EID, CFE_EVS_INFORMATION,
                #         "Commencing %s", "Land")
                print("Commencing Land")
                self.Land()
            elif (CurrentState
                  == PX4_NavigationState_t.PX4_NAVIGATION_STATE_AUTO_RTL):
                # (void) CFE_EVS_SendEvent(NAV_ACTION_ST_EID, CFE_EVS_INFORMATION,
                #         "Commencing %s", "Return to Launch")
                print("Commencing Return to Launch")
                self.Rtl()

            elif (CurrentState
                  == PX4_NavigationState_t.PX4_NAVIGATION_STATE_AUTO_MISSION):
                # (void) CFE_EVS_SendEvent(NAV_ACTION_ST_EID, CFE_EVS_INFORMATION,
                #         "Commencing %s", "Auto Mission")
                print("Commencing Auto Mission")
                self.HkTlm.AutoMissionItemIndex = 0
                self.MissionResultMsg.InstanceCount = 0
                self.MissionResultMsg.SeqReached = 0
                self.MissionResultMsg.SeqCurrent = 0
                self.MissionResultMsg.SeqTotal = 0
                self.MissionResultMsg.ItemChangedIndex = 0
                self.MissionResultMsg.ItemDoJumpRemaining = 0
                self.MissionResultMsg.Valid = False
                self.MissionResultMsg.Warning = False
                self.MissionResultMsg.Reached = False
                self.MissionResultMsg.Finished = False
                self.MissionResultMsg.Failure = False
                self.MissionResultMsg.StayInFailsafe = False
                self.MissionResultMsg.FlightTermination = False
                self.MissionResultMsg.ItemDoJumpChanged = False
                self.MissionResultUpdated = True
                self.AutoMission()

            else:
                self.CanLoiterAtSetpoint = False

                # /* Clear hk values revelant to mission if not in auto mode */
                self.HkTlm.MissionItemReached = False
                self.HkTlm.WaypointPositionReached = False
                self.HkTlm.WaypointYawReached = False
            # }

        # /* If the mode is active */
        if Active:

            if (CurrentState
                    == PX4_NavigationState_t.PX4_NAVIGATION_STATE_AUTO_TAKEOFF):

                self.TakeoffActive()

            elif (CurrentState
                  == PX4_NavigationState_t.PX4_NAVIGATION_STATE_AUTO_LOITER):
                self.LoiterActive()
            elif (CurrentState
                  == PX4_NavigationState_t.PX4_NAVIGATION_STATE_AUTO_LAND):
                self.LandActive()

            elif (CurrentState
                  == PX4_NavigationState_t.PX4_NAVIGATION_STATE_AUTO_RTL):
                self.RtlActive()
            elif (CurrentState
                  == PX4_NavigationState_t.PX4_NAVIGATION_STATE_AUTO_MISSION):
                self.AutoMissionActive()

            else:
                self.CanLoiterAtSetpoint = False

        # /* If we landed and have not received takeoff setpoint then stay in idle */
        if (self.CVT.VehicleLandDetectedMsg.Landed \
                and not ((self.CVT.VehicleStatusMsg.NavState \
                          == PX4_NavigationState_t.PX4_NAVIGATION_STATE_AUTO_TAKEOFF) \
                         or (self.CVT.VehicleStatusMsg.NavState \
                             == PX4_NavigationState_t.PX4_NAVIGATION_STATE_AUTO_MISSION))):
            self.PositionSetpointTripletMsg.Current.Type = PX4_SetpointType_t.PX4_SETPOINT_TYPE_IDLE
            self.PositionSetpointTripletMsg.Current.Valid = True
            self.PositionSetpointTripletMsg.Previous.Valid = False
            self.PositionSetpointTripletMsg.Next.Valid = False

        # /* Time stamp out going messages */
        Now = PX4LIB_GetPX4TimeUs()
        self.PositionSetpointTripletMsg.Timestamp = Now
        self.MissionResultMsg.Timestamp = Now

        if self.PositionSetpointTripletUpdated:
            self.PositionSetpointTripletMsg.Timestamp = Now
            # TODO:Populate telemetry messae and send it to Airliner
            self.SendPositionSetpointTripletMsg()
            self.PositionSetpointTripletUpdated = False

        if self.MissionResultUpdated:
            # TODO:Populate telemetry messae and send it to Airliner
            # self.SendMissionResultMsg()
            self.MissionResultUpdated = False

        return 0

    def IsMissionItemReached(self) -> bool:
        isMissionItemReached: bool = False
        Now: int = 0
        Dist: float = 0.0
        DistXy: float = 0.0
        DistZ: float = 0.0
        AltAsml: float = 0.0
        TakeoffAlt: float = 0.0
        AltitudeAcceptanceRadius: float = 0.0
        MissionAcceptanceRadius: float = 0.0
        Cog: float = 0.0
        YawErr: float = 0.0
        TimeInside: float = 0.0
        CurrentSetpoint: PX4_PositionSetpoint_t = PX4_PositionSetpoint_t()
        NextSetpoint: PX4_PositionSetpoint_t = PX4_PositionSetpoint_t()
        Range: float = 0.0
        Bearing: float = 0.0
        InnerAngle: float = 0.0

        Now = PX4LIB_GetPX4TimeUs()
        if self.HkTlm.RemainingDelay > 0:
            if self.HkTlm.ReleaseDelayAt <= Now:
                self.HkTlm.RemainingDelay = 0
                self.HkTlm.ReleaseDelayAt = 0
            else:
                self.HkTlm.RemainingDelay = Now - self.HkTlm.RemainingDelay

        if 0 == self.HkTlm.RemainingDelay:
            if self.MissionItem.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_WAIT_FOR_ATP:
                if not (self.HkTlm.WaitingForATP):
                    isMissionItemReached = True
                    # (void) CFE_EVS_SendEvent(NAV_ATP_INF_EID, CFE_EVS_INFORMATION,
                    #                         "ATP")

                # goto MissionItemReached_Exit_Tag
                return isMissionItemReached
            elif self.MissionItem.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_DO_SET_SERVO:
                isMissionItemReached = True
                # goto MissionItemReached_Exit_Tag
                return isMissionItemReached
            elif self.MissionItem.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LAND:
                isMissionItemReached = self.CVT.VehicleLandDetectedMsg.Landed
                return isMissionItemReached
                # goto MissionItemReached_Exit_Tag
            elif self.MissionItem.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LOITER_UNLIM:
                isMissionItemReached = False
                return isMissionItemReached
                # goto MissionItemReached_Exit_Tag
            elif self.MissionItem.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_SET_ATTITUDE:
                isMissionItemReached = True
                return isMissionItemReached
                # goto MissionItemReached_Exit_Tag
            else:
                pass
            # do nothing, this is a 3D waypoint #

            # TODO:Need to add CVT for the following logic

            # if (!CVT.VehicleLandDetectedMsg.Landed && !HkTlm.WaypointPositionReached)
            # {
            #     Dist= -1.0
            #     DistXy = -1.0
            #     DistZ = -1.0

            #     AltAsml = MissionItem.AltitudeIsRelative ?
            #                     MissionItem.Altitude + CVT.HomePositionMsg.Alt : MissionItem.Altitude
            #     Dist= get_distance_to_point_global_wgs84(MissionItem.Lat,
            #             MissionItem.Lon, AltAsml, CVT.VehicleGlobalPosition.Lat, CVT.VehicleGlobalPosition.Lon, CVT.VehicleGlobalPosition.Alt,
            #             &DistXy, &DistZ)
            #     if (MissionItem.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_TAKEOFF
            #             && CVT.VehicleStatusMsg.IsRotaryWing)
            #     {
            #         # We want to avoid the edge elif where the acceptance radius is bigger or equal than
            #         * the altitude of the takeoff waypoint above home. Otherwise, we do not really follow
            #         * take-off procedures like leaving the landing gear down. #
            #         TakeoffAlt = MissionItem.AltitudeIsRelative ?
            #                                 MissionItem.Altitude :
            #                                 (MissionItem.Altitude - CVT.HomePositionMsg.Alt)
            #         AltitudeAcceptanceRadius = GetAltitudeAcceptedRadius()

            #         # It should be safe to takeoff using half of the TakeoffAlt as accepted radius #
            #         if (TakeoffAlt > 0 && TakeoffAlt < AltitudeAcceptanceRadius)
            #         {
            #             AltitudeAcceptanceRadius = TakeoffAlt / 2.0
            #         }

            #         # Require only altitude for takeoff for mc #
            #         if (CVT.VehicleGlobalPosition.Alt > AltAsml - AltitudeAcceptanceRadius)
            #         {
            #             HkTlm.WaypointPositionReached = TRUE
            #         }
            #     }
            #     else if (MissionItem.NavCmd
            #             == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_TAKEOFF)
            #     {
            #         # For takeoff mission items use the parameter for the takeoff acceptance radius #
            #         if (Dist>= 0.0 && Dist<= ConfigTblPtr.NAV_ACC_RAD
            #                 && DistZ <= ConfigTblPtr.NAV_ACC_RAD)
            #         {
            #             HkTlm.WaypointPositionReached = TRUE
            #         }
            #     }
            #     else
            #     {
            #         # For normal mission items used their acceptance radius #
            #         MissionAcceptanceRadius = MissionItem.AcceptanceRadius

            #         # If set to zero use the default instead #
            #         if (MissionAcceptanceRadius < NAV_EPSILON_POSITION)
            #         {
            #             MissionAcceptanceRadius = ConfigTblPtr.NAV_ACC_RAD
            #         }
            #         if (Dist>= 0.0 && Dist<= MissionAcceptanceRadius
            #                 && DistZ <= ConfigTblPtr.NAV_ALT_RAD)
            #         {
            #             HkTlm.WaypointPositionReached = TRUE
            #         }
            #     }

            #     if (HkTlm.WaypointPositionReached)
            #     {
            #         TimeWpReached = Now
            #     }
            # }

            # if(HkTlm.WaypointPositionReached && !HkTlm.WaypointYawReached)
            # {
            #     # Added PX4_VEHICLE_CMD_NAV_LOITER_TO_ALT to PX4_VehicleCmd_t #
            #     if ((CVT.VehicleStatusMsg.IsRotaryWing
            #             || (MissionItem.NavCmd
            #                     == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LOITER_TO_ALT
            #                     && MissionItem.ForceHeading))
            #             && math.isfinite(MissionItem.Yaw))
            #     {
            #         # Check course if defined only for rotary wing except takeoff #
            #         Cog = CVT.VehicleStatusMsg.IsRotaryWing ? 
            #                         CVT.VehicleGlobalPosition.Yaw : 
            #                         atan2f(CVT.VehicleGlobalPosition.VelE, 
            #                             CVT.VehicleGlobalPosition.VelN)
            #         YawErr = _wrap_pi(MissionItem.Yaw - Cog)

            #         # Accept yaw if reached or if timeout is set then we dont ignore force headings #
            #         if (fabsf(YawErr)
            #                 < math.radians((float) ConfigTblPtr.NAV_MIS_YAW_ERR)
            #                 || (ConfigTblPtr.NAV_MIS_YAW_TMT >= FLT_EPSILON
            #                         && !MissionItem.ForceHeading))
            #         {
            #             HkTlm.WaypointYawReached = TRUE
            #         }

            #         # If heading needs to be reached, the timeout is enabled and we don't make it we abort #
            #         if (!HkTlm.WaypointYawReached && MissionItem.ForceHeading
            #                 && (ConfigTblPtr.NAV_MIS_YAW_TMT >= FLT_EPSILON)
            #                 && (Now - TimeWpReached
            #                         >= (uint64) ConfigTblPtr.NAV_MIS_YAW_TMT * 1e6f))
            #         {
            #             SetMissionFaliure("did not reach waypoint before timeout")
            #         }
            #     }
            #     else
            #     {
            #         HkTlm.WaypointYawReached = TRUE
            #     }
            # }

            # # Once the position and yaw waypoint have been set we can start the loiter time countdown #
            # if (HkTlm.WaypointPositionReached && HkTlm.WaypointYawReached)
            # {
            #     if (TimeFirstInsideOrbit == 0)
            #     {
            #         TimeFirstInsideOrbit = Now
            #     }

            #     # Check if the MAV was long enough inside the waypoint orbit #
            #     TimeInside = GetTimeInside(&MissionItem)
            #     if ((TimeInside < FLT_EPSILON)
            #             || Now - TimeFirstInsideOrbit >= (uint64) TimeInside * 1e6f)
            #     {
            #         # Exit xtrack location #
            #         if (MissionItem.LoiterExitXTrack
            #                 && (MissionItem.NavCmd
            #                         == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LOITER_TO_ALT
            #                         || MissionItem.NavCmd
            #                                 == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LOITER_TIME))
            #         {
            #             # Reset lat/lon of loiter waypoint so wehicle follows tangent #
            #             CurrentSetpoint = PositionSetpointTripletMsg.Current
            #             NextSetpoint = PositionSetpointTripletMsg.Next

            #             Range = get_distance_to_next_waypoint(CurrentSetpoint.Lat,
            #                     CurrentSetpoint.Lon, NextSetpoint.Lat, NextSetpoint.Lon)
            #             Bearing = get_bearing_to_next_waypoint(CurrentSetpoint.Lat,
            #                     CurrentSetpoint.Lon, NextSetpoint.Lat, NextSetpoint.Lon)
            #             InnerAngle = M_PI_2_F
            #                     - asinf(MissionItem.LoiterRadius / Range)

            #             # Compute ideal tangent origin #
            #             if (CurrentSetpoint.LoiterDirection > 0)
            #             {
            #                 Bearing -= InnerAngle

            #             }
            #             else
            #             {
            #                 Bearing += InnerAngle
            #             }

            #             # Replace current setpoint Lat/Lon with tangent coordinate #
            #             waypoint_from_heading_and_distance(CurrentSetpoint.Lat, CurrentSetpoint.Lon,
            #                     Bearing, CurrentSetpoint.LoiterRadius, &CurrentSetpoint.Lat,
            #                     &CurrentSetpoint.Lon)
            #         }
            #         isMissionItemReached = TRUE
            #     }
            # }

            isMissionItemReached = self.HkTlm.WaypointPositionReached
            # Copy values to HK #
            self.HkTlm.MissionItemReached = isMissionItemReached

            # All acceptance criteria must be met in the same iteration #
            self.HkTlm.WaypointPositionReached = False
            self.HkTlm.WaypointYawReached = False
        #     }

        # MissionItemReached_Exit_Tag:
        return isMissionItemReached

    # }

    def GetCruisingThrottle(self) -> float:
        msnThrottle = -1.0

        if self.MissionThrottle > sys.float_info.epsilon:
            msnThrottle = self.MissionThrottle

        return msnThrottle

    def ConvertMissionItemToCurrentSetpoint(self, PosSetpoint: PX4_PositionSetpoint_t,
                                            Item: MissionItem):
        # if (!(!Item.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_WAYPOINT
        #         || !Item.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LOITER_UNLIM
        #         || !Item.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LOITER_TIME
        #         || !Item.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LAND
        #         || !Item.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_TAKEOFF))

        # I THINK this is right....
        # print(f"ConvertMissionItemToCurrentSetpoint**1:{Item.NavCmd}")
        if not (not (Item.NavCmd) == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_WAYPOINT) \
                or not (Item.NavCmd) == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LOITER_UNLIM \
                or not (Item.NavCmd) == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LOITER_TIME \
                or not (Item.NavCmd) == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LAND \
                or not (Item.NavCmd) == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_TAKEOFF:

            PosSetpoint.Lat = Item.Lat
            PosSetpoint.Lon = Item.Lon
            # TODO:Implement CVT
            # if Item.AltitudeIsRelative:
            #     PosSetpoint.Alt = Item.Altitude + CVT.HomePositionMsg.Alt
            # else:
            #      PosSetpoint.Alt = Item.Altitude
            PosSetpoint.Yaw = Item.Yaw
            if (abs(Item.LoiterRadius) > self.NAV_EPSILON_POSITION):
                PosSetpoint.LoiterRadius = abs(Item.LoiterRadius)
            else:
                PosSetpoint.LoiterRadius = self.ConfigTblPtr["NAV_LOITER_RAD"]
            if Item.LoiterRadius > 0:

                PosSetpoint.LoiterDirection = 1
            else:
                PosSetpoint.LoiterDirection = -1
            PosSetpoint.AcceptanceRadius = Item.AcceptanceRadius
            PosSetpoint.DisableMcYawControl = Item.DisableMcYaw

            PosSetpoint.YawBody = Item.YawBody
            PosSetpoint.PitchBody = Item.PitchBody
            # print(f"PosSetpoint.PitchBody:{PosSetpoint.PitchBody}")
            PosSetpoint.RollBody = Item.RollBody

            # // PosSetpoint.CruisingSpeed = GetCruisingSpeed()
            PosSetpoint.CruisingSpeed = Item.CruisingSpeed
            PosSetpoint.CruisingThrottle = self.GetCruisingThrottle()
            # print("ConvertMissionItemToCurrentSetpoint**2")
            # TODO: Implement CVT
            #                     if PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_TAKEOFF:
            #                     {
            #                         if (CVT.VehicleStatusMsg.ArmingState == PX4_ArmingState_t.PX4_ARMING_STATE_ARMED
            #                                 && !CVT.VehicleLandDetectedMsg.Landed)
            #                         {
            #                             PosSetpoint.Type = PX4_SetpointType_t.PX4_SETPOINT_TYPE_POSITION
            #                         }
            #                         else
            #                         {
            #                             PosSetpoint.Type = PX4_SetpointType_t.PX4_SETPOINT_TYPE_TAKEOFF
            #                             /* Set pitch and ensure that the hold time is zero */
            #                             PosSetpoint.PitchMin = Item.PitchMin
            #                         }

            #                         break
            #                     }

            #                     case PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LAND:
            #                     {
            #                         PosSetpoint.Type = PX4_SetpointType_t.PX4_SETPOINT_TYPE_LAND
            #                         break
            #                     }

            #                     // Fall through
            #                     case PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LOITER_TIME:
            #                     case PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LOITER_UNLIM:
            #                     {
            #                         PosSetpoint.Type = PX4_SetpointType_t.PX4_SETPOINT_TYPE_LOITER
            #                         break
            #                     }

            if Item.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_SET_ATTITUDE:
                PosSetpoint.Type = PX4_SetpointType_t.PX4_SETPOINT_TYPE_ATTITUDE


            else:
                PosSetpoint.Type = PX4_SetpointType_t.PX4_SETPOINT_TYPE_POSITION
            PosSetpoint.Valid = True
        # print("ConvertMissionItemToCurrentSetpoint**13")

    def ATP(self):
        self.HkTlm.WaitingForATP = False

    def AutoMission(self):

        # /* Don't do anything unless a json is loaded.  Check that is loaded first. */
        if (self.JsonLoaded):
            # memcpy(&MissionItem, &MissionTblPtr.MissionItem[HkTlm.AutoMissionItemIndex], sizeof(MissionItem))
            self.MissionItem = self.MissionItems[self.HkTlm.AutoMissionItemIndex]

            self.HkTlm.ActionStart = PX4LIB_GetPX4TimeUs()
            self.HkTlm.RemainingDelay = self.MissionItem.DelayTime
            self.HkTlm.ReleaseDelayAt = self.HkTlm.ActionStart + self.HkTlm.RemainingDelay
            if self.MissionItem.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_WAIT_FOR_ATP:
                if not (self.HkTlm.WaitingForATP):
                    self.HkTlm.WaitingForATP = True
                    # (void) CFE_EVS_SendEvent(NAV_ATP_INF_EID, CFE_EVS_INFORMATION,
                    #                         "Auto Mission Item %d. Waiting for ATP", HkTlm.AutoMissionItemIndex)
                    print(f"Auto Mission Item {self.HkTlm.AutoMissionItemIndex}. Waiting for ATP")



            else:
                nextMissionItem: MissionItem = MissionItem()
                nextMissionIndex: int = 0

                # memcpy(&PositionSetpointTripletMsg.Previous, &PositionSetpointTripletMsg.Current, sizeof(PositionSetpointTripletMsg.Previous))
                self.PositionSetpointTripletMsg.Previous = self.PositionSetpointTripletMsg.Current

                self.MissionItem.Origin = Origin.ORIGIN_ONBOARD

                # (void) CFE_EVS_SendEvent(NAV_AUTO_MISSION_STATE_EID, CFE_EVS_INFORMATION,
                #     "Auto Mission Item %d", HkTlm.AutoMissionItemIndex)
                print(f"Auto Mission Item {self.HkTlm.AutoMissionItemIndex}")
                self.MissionResultMsg.SeqCurrent = self.HkTlm.AutoMissionItemIndex
                self.MissionResultUpdated = True

                self.ConvertMissionItemToCurrentSetpoint(
                    self.PositionSetpointTripletMsg.Current, self.MissionItem)
                self.PositionSetpointTripletMsg.Current.Valid = True

                if self.MissionItem.DoJumpMissionIndex == self.NAV_NO_JUMP:
                    nextMissionIndex = self.HkTlm.AutoMissionItemIndex + 1
                else:
                    nextMissionIndex = self.MissionItem.DoJumpMissionIndex

                if nextMissionIndex < self.NAV_MISSION_ITEM_MAX:
                    if self.MissionItems[nextMissionIndex].NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NONE:

                        # memset(&nextMissionItem, 0, sizeof(nextMissionItem))
                        nextMissionItem = self.NewEmptyMissionItem()

                    else:
                        # memcpy(&nextMissionItem, &MissionTblPtr.MissionItem[nextMissionIndex], sizeof(nextMissionItem))
                        nextMissionItem = self.MissionItems[nextMissionIndex]

                nextMissionItem.Origin = Origin.ORIGIN_ONBOARD
                self.ConvertMissionItemToCurrentSetpoint(
                    self.PositionSetpointTripletMsg.Next, nextMissionItem)
                self.PositionSetpointTripletMsg.Next.Type = PX4_SetpointType_t.PX4_SETPOINT_TYPE_POSITION
                self.PositionSetpointTripletMsg.Next.Valid = True

                self.PositionSetpointTripletUpdated = True

                # CFE_SB_MessageStringSet(HkTlm.MissionID, MissionTblPtr.MissionID,
                #     sizeof(HkTlm.MissionID), sizeof(MissionTblPtr.MissionID))

    def AutoMissionActive(self):
        MissionItemReachedFlag = self.IsMissionItemReached()
        if MissionItemReachedFlag:

            # Set mission result message #
            self.MissionResultMsg.SeqReached = self.HkTlm.AutoMissionItemIndex
            self.MissionResultMsg.Reached = True
            self.MissionResultMsg.Finished = False

            # Record mission update event in bool #
            self.MissionResultUpdated = True

            if self.MissionItem.DoJumpMissionIndex == self.NAV_NO_JUMP:
                self.HkTlm.AutoMissionItemIndex = self.HkTlm.AutoMissionItemIndex + 1

            else:
                self.HkTlm.AutoMissionItemIndex = self.MissionItem.DoJumpMissionIndex

            if self.HkTlm.AutoMissionItemIndex < self.NAV_MISSION_ITEM_MAX:
                # MissionTblPtr Should be replaced with Mission List from JSON
                if self.MissionItems[self.HkTlm.AutoMissionItemIndex].NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NONE:
                    self.MissionItem = self.NewEmptyMissionItem()
                    # memset(&MissionItem, 0, sizeof(MissionItem))

                    # (void) CFE_EVS_SendEvent(NAV_AUTO_MISSION_STATE_EID, CFE_EVS_INFORMATION,
                    #     "Auto Mission Complete.")
                    print("Auto Mission Complete.")

                    self.MissionResultMsg.SeqCurrent = 0
                    self.MissionResultMsg.Finished = True
                    self.MissionResultUpdated = True

                    # Loiter() Implement Loiter

                else:
                    self.AutoMission()
            else:

                # memset(&MissionItem, 0, sizeof(MissionItem))

                # (void) CFE_EVS_SendEvent(NAV_AUTO_MISSION_STATE_EID, CFE_EVS_INFORMATION,
                #     "Auto Mission Complete.")

                print("Auto Mission Complete.")

                self.MissionResultMsg.SeqCurrent = 0
                self.MissionResultMsg.Finished = False
                self.MissionResultUpdated = False

                # Loiter() Implement Loiter
