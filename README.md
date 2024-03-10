# Pyliner

Pyliner is a Python 2.7.x package for sending Airliner commands to a drone
using a user-friendly scripting interface.

# To Run

```
make virtual-env
make run
```

# Mission Item

```
/**
 * \brief mission planning structure
 */
typedef struct
{
    /** \brief The latitude in degrees */
    double Lat;
    /** \brief The longitude in degrees */
    double Lon;
    /** \brief The time that the MAV should stay inside the radius before advancing in seconds */
    float TimeInside;
    /** \brief The minimal pitch angle for fixed wing takeoff waypoints */
    float PitchMin;
    /** \brief The default radius in which the mission is accepted as reached in meters */
    float AcceptanceRadius;
    /** \brief lThe oiter radius in meters, 0 for a VTOL to hover, negative for counter-clockwise */
    float LoiterRadius;
    /** \brief The in radians NED -PI..+PI, NAN means don't change yaw. By "yaw" we mean heading here. */
    float Yaw;
    float YawBody;
    float PitchBody; // Body pitch in Theta mode. Alpha in Alpha Mode. Alpha/Theta mode can be set in FAC table.
    float RollBody;
    /** \brief The latitude padding */
    float LatFloatPadding;
    /** \brief The longitude padding */
    float LonFloatPadding;
    /** \brief The altitude in meters    (AMSL) */
    float Altitude;
    float CruisingSpeed;
    /** \brief The array to store mission command values for MAV_FRAME_MISSION */
    float Params[7];
    /** \brief The navigation command */
    uint16 NavCmd;
    /** \brief The index where the do jump will go to */
    int16 DoJumpMissionIndex;
    /** \brief The how many times do jump needs to be done */
    uint16 DoJumpRepeatCount;
    /** \brief The count how many times the jump has been done */
    uint16 DoJumpCurrentCount;
    /** \brief The mission frame */
    uint16 Frame;
    /** \brief How the mission item was generated */
    uint16 Origin;
    /** \brief The exit xtrack location: 0 for center of loiter wp, 1 for exit location */
    uint16 LoiterExitXTrack;
    /** \brief The heading needs to be reached */
    uint16 ForceHeading;
    /** \brief True if altitude is relative from start point */
    uint16 AltitudeIsRelative;
    /** \brief True if next waypoint should follow after this one */
    uint16 AutoContinue;
    /** \brief Disables multi-copter yaw with this flag */
    uint16 DisableMcYaw;
    /** \brief Number of milliseconds to delay until proceeding to the next mission item */
    uint64 DelayTime;
}NAV_MissionItem_t;

```

# IDE Notes
- Set the working directory to `airliner_old/tools/pyliner` in PyCharm



1. cd software/airliner/public

2. git checkout pyliner_commands

3. git submodule update --init --recursive

4. make quad-sitl

5. make quad-sitl-workspace

6. cd build/multirotor/quad/

7. terminator -g term-sitl.cfg

8. Add "PX4_Vechicle_Global_Position" to downlink

9. Open a new shell

10. Kill yamcs. At the moment pyliner will use the same port number as YAMCS. And we also don't have TO channels.

11. cd core/tools/pyliner

12. make virtual-env

13. source venv/bin/activate

14. PYTHONPATH=PYTHONPATH:..:. python spiral001.py 

