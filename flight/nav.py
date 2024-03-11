from enum import Enum
from flight.px4_lib import PX4_MissionResultMsg_t, PX4_NavigationState_t, PX4_PositionSetpoint_t, PX4_PositionSetpointTripletMsg_t, PX4_SetpointType_t, PX4_VehicleCmd_t, PX4LIB_GetPX4TimeUs, RTLState
import sys

# /**
#  * \brief mission origin
#  */
class  Origin(Enum):
    # /**! Mavlink originated mission item */
    ORIGIN_MAVLINK = 0,
    # /*! Onboard originated mission item */
    ORIGIN_ONBOARD = 1


class MissionItem():
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
        self.PitchBody = 0 # Body pitch in Theta mode. Alpha in Alpha Mode. Alpha/Theta mode can be set in FAC table.
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
        #* \navtlmmnemonic \NAV_CMDACPTCNT
        #  \brief Count of accepted commands #
        self.usCmdCnt: int = 0

        #* \navtlmmnemonic \NAV_CMDRJCTCNT
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

        #* \brief True if we're waiting for an Authorization to Proceed command #
        self.WaitingForATP: bool = False

        #* \brief The time a Mission Item action started. #
        self.ActionStart: int = 0
        
        #* \brief The time a Mission Item action started. #
        self.ReleaseDelayAt: int = 0

        #* \brief The number of milliseconds NAV is waiting to proceed to the next Mission Item. #
        self.RemainingDelay: int = 0

        # \brief Flag for if current mission waypoint position is reached #
        self.WaypointPositionReached: bool = False
        
        self.AutoMissionItemIndex: int = 0

        # \brief Id for current mission #
        self.MissionID: str = ""
    
class Nav():

    def __init__(self) -> None:
        self.NAV_NO_JUMP = -1
        self.NAV_MISSION_ITEM_MAX = 512
        self.NAV_EPSILON_POSITION =  0.001
        self.M_PI_2_F =                1.57079632
        self.DELAY_SIGMA =             0.01
        self.NAV_LAT_SHORT_FORM =      1000
        self.NAV_LON_SHORT_FORM  =    1000
        self.CONVERT_DECIMAL_DEGREES = 1e7

        self.ConfigTblPtr = {} 

        self.HkTlm: NAV_HkTlm_t = NAV_HkTlm_t()
        # # \brief Output Data published at the end of cycle */
        # # \brief The mission result message */
        self.MissionResultMsg: PX4_MissionResultMsg_t = PX4_MissionResultMsg_t()

        # /** \brief The position set point triplet message */
        self.PositionSetpointTripletMsg: PX4_PositionSetpointTripletMsg_t = PX4_PositionSetpointTripletMsg_t()

        self.MissionItem: MissionItem = MissionItem()
        self.MissionItems = []
        # //osalbool missionStarted
        # //osalbool waypointStarted
        # \brief Flag is set to true if a previously unseen command is encountered */
        self.NewCommandArrived: bool = False
        # \brief Will allow to loiter at setpoint */
        self.CanLoiterAtSetpoint: bool  = False
        # \brief True if loiter position is set */
        self.LoiterPositionSet: bool = False
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
        self.ForceDescentTarget: float  = 0

        self.JsonLoaded = False # This replaces "MissionTblPtr" check in original NAV code
    
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
                self.HkTlm.RemainingDelay =  Now - self.HkTlm.RemainingDelay
        
        
        if 0 == self.HkTlm.RemainingDelay:
            if MissionItem.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_WAIT_FOR_ATP:
                    if not(self.HkTlm.WaitingForATP):
                        isMissionItemReached = True
                        # (void) CFE_EVS_SendEvent(NAV_ATP_INF_EID, CFE_EVS_INFORMATION,
                        #                         "ATP")
                    
                    # goto MissionItemReached_Exit_Tag
            elif PX4_VehicleCmd_t.PX4_VEHICLE_CMD_DO_SET_SERVO:
                isMissionItemReached = True
                # goto MissionItemReached_Exit_Tag
            # elif PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LAND:
            #     isMissionItemReached = CVT.VehicleLandDetectedMsg.Landed
                # goto MissionItemReached_Exit_Tag
            elif PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LOITER_UNLIM:
                isMissionItemReached = False
                # goto MissionItemReached_Exit_Tag
            elif PX4_VehicleCmd_t.PX4_VEHICLE_CMD_SET_ATTITUDE:
                isMissionItemReached = True
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
            #             && isfinite(MissionItem.Yaw))
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
            if not(not(Item.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_WAYPOINT) \
                    or not(Item.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LOITER_UNLIM) \
                    or not(Item.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LOITER_TIME) \
                    or not(Item.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_LAND) \
                    or not(Item.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NAV_TAKEOFF)):
            
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
                PosSetpoint.RollBody = Item.RollBody

                # // PosSetpoint.CruisingSpeed = GetCruisingSpeed()
                PosSetpoint.CruisingSpeed = Item.CruisingSpeed
                PosSetpoint.CruisingThrottle = self.GetCruisingThrottle()
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

                if self.Item == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_SET_ATTITUDE:
                    PosSetpoint.Type = PX4_SetpointType_t.PX4_SETPOINT_TYPE_ATTITUDE
                    

                else:
                    PosSetpoint.Type = PX4_SetpointType_t.PX4_SETPOINT_TYPE_POSITION
                    PosSetpoint.Valid = True
             



















    def AutoMission(self):

        # /* Don't do anything unless a json is loaded.  Check that is loaded first. */
        if(self.JsonLoaded):
            # memcpy(&MissionItem, &MissionTblPtr.MissionItem[HkTlm.AutoMissionItemIndex], sizeof(MissionItem))

            self.MissionItem = self.MissionItems[self.HkTlm.AutoMissionItemIndex]
            
            self.HkTlm.ActionStart = PX4LIB_GetPX4TimeUs()
            self.HkTlm.RemainingDelay = self.MissionItem.DelayTime
            self.HkTlm.ReleaseDelayAt = self.HkTlm.ActionStart + self.HkTlm.RemainingDelay

            if self.MissionItem.NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_WAIT_FOR_ATP:
                if not(self.HkTlm.WaitingForATP):
                    self.HkTlm.WaitingForATP = True
                    # (void) CFE_EVS_SendEvent(NAV_ATP_INF_EID, CFE_EVS_INFORMATION,
                    #                         "Auto Mission Item %d. Waiting for ATP", HkTlm.AutoMissionItemIndex)
                    
                    

            else:
                nextMissionItem: MissionItem = MissionItem()
                nextMissionIndex: int  = 0

                # memcpy(&PositionSetpointTripletMsg.Previous, &PositionSetpointTripletMsg.Current, sizeof(PositionSetpointTripletMsg.Previous))

                self.MissionItem.Origin = Origin.ORIGIN_ONBOARD

                # (void) CFE_EVS_SendEvent(NAV_AUTO_MISSION_STATE_EID, CFE_EVS_INFORMATION,
                #     "Auto Mission Item %d", HkTlm.AutoMissionItemIndex)
                print(f"Auto Mission Item {self.HkTlm.AutoMissionItemIndex}")
                self.MissionResultMsg.SeqCurrent = self.HkTlm.AutoMissionItemIndex
                self.MissionResultUpdated = True

                self.ConvertMissionItemToCurrentSetpoint(
                    self.PositionSetpointTripletMsg.Current, self.MissionItem)
                self.PositionSetpointTripletMsg.Current.Valid = True

                if MissionItem.DoJumpMissionIndex == self.NAV_NO_JUMP:
                    nextMissionIndex = self.HkTlm.AutoMissionItemIndex + 1
                else:
                    nextMissionIndex = MissionItem.DoJumpMissionIndex

                if nextMissionIndex < self.NAV_MISSION_ITEM_MAX:
                    if self.MissionItems[nextMissionIndex].NavCmd == PX4_VehicleCmd_t.PX4_VEHICLE_CMD_NONE:
                    
                        # memset(&nextMissionItem, 0, sizeof(nextMissionItem))
                        nextMissionItem = None
                    
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
                self.HkTlm.AutoMissionItemIndex = MissionItem.DoJumpMissionIndex

            if self.HkTlm.AutoMissionItemIndex < self.NAV_MISSION_ITEM_MAX:
                # MissionTblPtr Should be replaced with Mission List from JSON
                if MissionTblPtr.MissionItem[HkTlm.AutoMissionItemIndex].NavCmd == PX4_VEHICLE_CMD_NONE:
                
                    # memset(&MissionItem, 0, sizeof(MissionItem))

                    # (void) CFE_EVS_SendEvent(NAV_AUTO_MISSION_STATE_EID, CFE_EVS_INFORMATION,
                    #     "Auto Mission Complete.")

                    self.MissionResultMsg.SeqCurrent = 0
                    self.MissionResultMsg.Finished = True
                    self.MissionResultUpdated = True

                    # Loiter() Implement Loiter
                
                else:
                    AutoMission()
            else:
            
                # memset(&MissionItem, 0, sizeof(MissionItem))

                # (void) CFE_EVS_SendEvent(NAV_AUTO_MISSION_STATE_EID, CFE_EVS_INFORMATION,
                #     "Auto Mission Complete.")
                
                print("Auto Mission Complete.")

                self.MissionResultMsg.SeqCurrent = 0
                self.MissionResultMsg.Finished = False
                self.MissionResultUpdated = False

                 # Loiter() Implement Loiter
            
        

