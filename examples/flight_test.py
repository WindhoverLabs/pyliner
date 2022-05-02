from os import path, sys

from util import get_time

sys.path.append( path.dirname( path.dirname( path.abspath(__file__) ) ) )
from pyliner import Pyliner
import time
import math
from px4_msg_enums import *

# Initialize pyliner object
airliner = Pyliner(**{"airliner_map": "cookiecutter.json", 
                      "ci_port": 5009,
                      "to_port": 5012,
                      "script_name": "flight_test",
                      "log_dir": "./logs/"})

# Subscribe to es HK commander counter
airliner.subscribe({'tlm': ['/Airliner/ES/HK/CmdCounter']})

# Subscribe to sensor combined message acceleration and baro alt
airliner.subscribe({'tlm': ['/Airliner/SENS/HK/Acc',
                            '/Airliner/SENS/HK/BaroAlt']})
# Subscribe to gps lat and long ( 
# NOTE currently not being populated in GPS flight software
airliner.subscribe({'tlm': ['/Airliner/GPS/HK/Lat',
                            '/Airliner/GPS/HK/Lon']})


# Subscribe to desired telemetry
#airliner.subscribe({'tlm': []})

# Perform 15 batches of sending commands and receiving telemetry
#for i in range(15):
    # Print all data for CmdCounter
    #print "Telemetry: %s   Value: %s   Timestamp: %s" % (cmd_count_data['name'], cmd_count_data['value'], cmd_count_data['time'])
    
    # Get just the value of other subscribed items
    #log_mode = airliner.get_tlm_value('/Airliner/ES/HK/SysLogMode')
    

    # Print the fresh telemetry values
    #print "Log mode: " + str(log_mode)

    # Send NoOp command

#    airliner.send_telemetry({'name':'/Airliner/CNTL/SetpointTriplet', 'args':[
#                             {'name':'Prev_Lat', 'value':1.0},
#                             {'name':'Prev_Lon', 'value':1.0},
#                             {'name':'Prev_X', 'value':1.0},
#                             {'name':'Prev_Y', 'value':1.0},
#                             {'name':'Prev_Z', 'value':1.0},
#                             {'name':'Prev_VX', 'value':1.0},
#                             {'name':'Prev_VY', 'value':1.0},
#                             {'name':'Prev_VZ', 'value':1.0},
#                             {'name':'Prev_Alt', 'value':1.0},
#                             {'name':'Prev_Yaw', 'value':1.0},
#                             {'name':'Prev_Yawspeed', 'value':1.0},
#                             {'name':'Prev_LoiterRadius', 'value':1.0},
#                             {'name':'Prev_PitchMin', 'value':1.0},
#                             {'name':'Prev_AX', 'value':1.0},
#                             {'name':'Prev_AY', 'value':1.0},
#                             {'name':'Prev_AZ', 'value':1.0},
#                             {'name':'Prev_AcceptanceRadius', 'value':1.0},
#                             {'name':'Prev_CruisingSpeed', 'value':1.0},
#                             {'name':'Prev_CruisingThrottle', 'value':1.0},
#                             {'name':'Prev_Valid', 'value':True},
#                             {'name':'Prev_Type', 'value':1},
#                             {'name':'Prev_PositionValid', 'value':True},
#                             {'name':'Prev_VelocityValid', 'value':True},
#                             {'name':'Prev_YawValid', 'value':True},
#                             {'name':'Prev_DisableMcYawControl', 'value':True},
#                             {'name':'Prev_YawspeedValid', 'value':True},
#                             {'name':'Prev_LoiterDirection', 'value':1},
#                             {'name':'Prev_AccelerationValid', 'value':True},
#                             {'name':'Prev_AccelerationIsForce', 'value':True},
#                             {'name':'Cur_Lat', 'value':1.0},
#                             {'name':'Cur_Lon', 'value':1.0},
#                             {'name':'Cur_X', 'value':1.0},
#                             {'name':'Cur_Y', 'value':1.0},
#                             {'name':'Cur_Z', 'value':1.0},
#                             {'name':'Cur_VX', 'value':1.0},
#                             {'name':'Cur_VY', 'value':1.0},
#                             {'name':'Cur_VZ', 'value':1.0},
#                             {'name':'Cur_Alt', 'value':1.0},
#                             {'name':'Cur_Yaw', 'value':1.0},
#                             {'name':'Cur_Yawspeed', 'value':1.0},
#                             {'name':'Cur_LoiterRadius', 'value':1.0},
#                             {'name':'Cur_PitchMin', 'value':1.0},
#                             {'name':'Cur_AX', 'value':1.0},
#                             {'name':'Cur_AY', 'value':1.0},
#                             {'name':'Cur_AZ', 'value':1.0},
#                             {'name':'Cur_AcceptanceRadius', 'value':1.0},
#                             {'name':'Cur_CruisingSpeed', 'value':1.0},
#                             {'name':'Cur_CruisingThrottle', 'value':1.0},
#                             {'name':'Cur_Valid', 'value':True},
#                             {'name':'Cur_Type', 'value':1},
#                             {'name':'Cur_PositionValid', 'value':True},
#                             {'name':'Cur_VelocityValid', 'value':True},
#                             {'name':'Cur_YawValid', 'value':True},
#                             {'name':'Cur_DisableMcYawControl', 'value':True},
#                             {'name':'Cur_YawspeedValid', 'value':True},
#                             {'name':'Cur_LoiterDirection', 'value':1},
#                             {'name':'Cur_AccelerationValid', 'value':True},
#                             {'name':'Cur_AccelerationIsForce', 'value':True},
#                             {'name':'Next_Lat', 'value':1.0},
#                             {'name':'Next_Lon', 'value':1.0},
#                             {'name':'Next_X', 'value':1.0},
#                             {'name':'Next_Y', 'value':1.0},
#                             {'name':'Next_Z', 'value':1.0},
#                             {'name':'Next_VX', 'value':1.0},
#                             {'name':'Next_VY', 'value':1.0},
#                             {'name':'Next_VZ', 'value':1.0},
#                             {'name':'Next_Alt', 'value':1.0},
#                             {'name':'Next_Yaw', 'value':1.0},
#                             {'name':'Next_Yawspeed', 'value':1.0},
#                             {'name':'Next_LoiterRadius', 'value':1.0},
#                             {'name':'Next_PitchMin', 'value':1.0},
#                             {'name':'Next_AX', 'value':1.0},
#                             {'name':'Next_AY', 'value':1.0},
#                             {'name':'Next_AZ', 'value':1.0},
#                             {'name':'Next_AcceptanceRadius', 'value':1.0},
#                             {'name':'Next_CruisingSpeed', 'value':1.0},
#                             {'name':'Next_CruisingThrottle', 'value':1.0},
#                             {'name':'Next_Valid', 'value':True},
#                             {'name':'Next_Type', 'value':1},
#                             {'name':'Next_PositionValid', 'value':True},
#                             {'name':'Next_VelocityValid', 'value':True},
#                             {'name':'Next_YawValid', 'value':True},
#                             {'name':'Next_DisableMcYawControl', 'value':True},
#                             {'name':'Next_YawspeedValid', 'value':True},
#                             {'name':'Next_LoiterDirection', 'value':1},
#                             {'name':'Next_AccelerationValid', 'value':True},
#                             {'name':'Next_AccelerationIsForce', 'value':True},
#                             {'name':'Timestamp', 'value':airliner.get_time()}]})


def vehicle_arm():
    airliner.send_telemetry(
        {'name':'/Airliner/CNTL/VehicleCommand', 'args':[
        {'name':'Timestamp', 'value': get_time()},
        {'name':'Command', 'value':int(MsgVehicleCmd.component_arm_disarm)},
        # 1.0 = True = Arm
        {'name':'Param1', 'value':1.0},
        {'name':'Param2', 'value':0.0},
        {'name':'Param3', 'value':0.0},
        {'name':'Param4', 'value':0.0},
        {'name':'Param5', 'value':0.0},
        {'name':'Param6', 'value':0.0},
        {'name':'Param7', 'value':0.0},
        {'name':'TargetSystem', 'value':1},
        {'name':'TargetComponent', 'value':1},
        {'name':'SourceSystem', 'value':0},
        {'name':'SourceComponent', 'value':0},
        {'name':'Confirmation', 'value':0}]})
    time.sleep(1)


def vehicle_disarm():
    airliner.send_telemetry(
        {'name':'/Airliner/CNTL/VehicleCommand', 'args':[
        {'name':'Timestamp', 'value': get_time()},
        {'name':'Command', 'value':int(MsgVehicleCmd.component_arm_disarm)},
        {'name':'Param1', 'value':0.0},
        {'name':'Param2', 'value':0.0},
        {'name':'Param3', 'value':0.0},
        {'name':'Param4', 'value':0.0},
        {'name':'Param5', 'value':0.0},
        {'name':'Param6', 'value':0.0},
        {'name':'Param7', 'value':0.0},
        {'name':'TargetSystem', 'value':1},
        {'name':'TargetComponent', 'value':1},
        {'name':'SourceSystem', 'value':0},
        {'name':'SourceComponent', 'value':0},
        {'name':'Confirmation', 'value':0}]})


def vehicle_takeoff():
    airliner.send_telemetry(
        {'name':'/Airliner/CNTL/VehicleCommand', 'args':[
        {'name':'Timestamp', 'value': get_time()},
        {'name':'Command', 'value':int(MsgVehicleCmd.nav_takeoff)},
        {'name':'Param1', 'value':0.0},
        {'name':'Param2', 'value':0.0},
        {'name':'Param3', 'value':0.0},
        {'name':'Param4', 'value':0.0},
        {'name':'Param5', 'value':0.0},
        {'name':'Param6', 'value':0.0},
        {'name':'Param7', 'value':0.0},
        {'name':'TargetSystem', 'value':1},
        {'name':'TargetComponent', 'value':1},
        {'name':'SourceSystem', 'value':0},
        {'name':'SourceComponent', 'value':0},
        {'name':'Confirmation', 'value':0}]})
    time.sleep(5)


def vehicle_posctl_mode():
    custom_mode = int(MavModeFlag.custom) + int(MavModeFlag.stabilize) + int(MavModeFlag.manual) + int(MavModeFlag.safety)
    # First transition to altitude hold mode.
    airliner.send_telemetry(
        {'name':'/Airliner/CNTL/VehicleCommand', 'args':[
        {'name':'Timestamp', 'value': get_time()},
        {'name':'Command', 'value':int(MsgVehicleCmd.do_set_mode)},
        {'name':'Param1', 'value':custom_mode},
        {'name':'Param2', 'value':int(MavCustomMainMode.altctl)},
        # Ready not currently used in commander.
        {'name':'Param3', 'value':int(MavCustomSubModeAuto.ready)},
        {'name':'Param4', 'value':0.0},
        {'name':'Param5', 'value':0.0},
        {'name':'Param6', 'value':0.0},
        {'name':'Param7', 'value':0.0},
        {'name':'TargetSystem', 'value':1},
        {'name':'TargetComponent', 'value':1},
        {'name':'SourceSystem', 'value':0},
        {'name':'SourceComponent', 'value':0},
        {'name':'Confirmation', 'value':0}]})
    time.sleep(0.1)
    # Transition to posctl mode.
    airliner.send_telemetry(
        {'name':'/Airliner/CNTL/VehicleCommand', 'args':[
        {'name':'Timestamp', 'value': get_time()},
        {'name':'Command', 'value':int(MsgVehicleCmd.do_set_mode)},
        {'name':'Param1', 'value':custom_mode},
        {'name':'Param2', 'value':int(MavCustomMainMode.posctl)},
        # Ready not currently used in commander.
        {'name':'Param3', 'value':int(MavCustomSubModeAuto.ready)},
        {'name':'Param4', 'value':0.0},
        {'name':'Param5', 'value':0.0},
        {'name':'Param6', 'value':0.0},
        {'name':'Param7', 'value':0.0},
        {'name':'TargetSystem', 'value':1},
        {'name':'TargetComponent', 'value':1},
        {'name':'SourceSystem', 'value':0},
        {'name':'SourceComponent', 'value':0},
        {'name':'Confirmation', 'value':0}]})
    time.sleep(0.1)


def vehicle_stable_hover():
    # 50% Throttle
    airliner.send_telemetry(
        {'name':'/Airliner/CNTL/ManualSetpoint', 'args':[
        {'name':'Timestamp', 'value': get_time()},
        {'name':'X', 'value':0.0},
        {'name':'Y', 'value':0.0},
        {'name':'Z', 'value':0.5},
        {'name':'R', 'value':0.0},
        {'name':'Flaps', 'value':0.0},
        {'name':'Aux1', 'value':0.0},
        {'name':'Aux2', 'value':0.0},
        {'name':'Aux3', 'value':0.0},
        {'name':'Aux4', 'value':0.0},
        {'name':'Aux5', 'value':0.0},
        {'name':'ModeSwitch', 'value':0},
        {'name':'ReturnSwitch', 'value':0},
        {'name':'RattitudeSwitch', 'value':0},
        {'name':'PosctlSwitch', 'value':0},
        {'name':'LoiterSwitch', 'value':0},
        {'name':'AcroSwitch', 'value':0},
        {'name':'OffboardSwitch', 'value':0},
        {'name':'KillSwitch', 'value':0},
        {'name':'TransitionSwitch', 'value':0},
        {'name':'GearSwitch', 'value':0},
        {'name':'ArmSwitch', 'value':0},
        {'name':'StabSwitch', 'value':0},
        {'name':'ManSwitch', 'value':0},
        {'name':'ModeSlot', 'value':0},
        {'name':'DataSource', 'value':0}]})


def vehicle_full_forward():
    airliner.send_telemetry(
        {'name':'/Airliner/CNTL/ManualSetpoint', 'args':[
        {'name':'Timestamp', 'value': get_time()},
        {'name':'X', 'value':1.0},
        {'name':'Y', 'value':0.0},
        {'name':'Z', 'value':0.5},
        {'name':'R', 'value':0.0},
        {'name':'Flaps', 'value':0.0},
        {'name':'Aux1', 'value':0.0},
        {'name':'Aux2', 'value':0.0},
        {'name':'Aux3', 'value':0.0},
        {'name':'Aux4', 'value':0.0},
        {'name':'Aux5', 'value':0.0},
        {'name':'ModeSwitch', 'value':0},
        {'name':'ReturnSwitch', 'value':0},
        {'name':'RattitudeSwitch', 'value':0},
        {'name':'PosctlSwitch', 'value':0},
        {'name':'LoiterSwitch', 'value':0},
        {'name':'AcroSwitch', 'value':0},
        {'name':'OffboardSwitch', 'value':0},
        {'name':'KillSwitch', 'value':0},
        {'name':'TransitionSwitch', 'value':0},
        {'name':'GearSwitch', 'value':0},
        {'name':'ArmSwitch', 'value':0},
        {'name':'StabSwitch', 'value':0},
        {'name':'ManSwitch', 'value':0},
        {'name':'ModeSlot', 'value':0},
        {'name':'DataSource', 'value':0}]})


def vehicle_full_left():
    airliner.send_telemetry(
        {'name':'/Airliner/CNTL/ManualSetpoint', 'args':[
        {'name':'Timestamp', 'value': get_time()},
        {'name':'X', 'value':0.0},
        {'name':'Y', 'value':-1.0},
        {'name':'Z', 'value':0.5},
        {'name':'R', 'value':0.0},
        {'name':'Flaps', 'value':0.0},
        {'name':'Aux1', 'value':0.0},
        {'name':'Aux2', 'value':0.0},
        {'name':'Aux3', 'value':0.0},
        {'name':'Aux4', 'value':0.0},
        {'name':'Aux5', 'value':0.0},
        {'name':'ModeSwitch', 'value':0},
        {'name':'ReturnSwitch', 'value':0},
        {'name':'RattitudeSwitch', 'value':0},
        {'name':'PosctlSwitch', 'value':0},
        {'name':'LoiterSwitch', 'value':0},
        {'name':'AcroSwitch', 'value':0},
        {'name':'OffboardSwitch', 'value':0},
        {'name':'KillSwitch', 'value':0},
        {'name':'TransitionSwitch', 'value':0},
        {'name':'GearSwitch', 'value':0},
        {'name':'ArmSwitch', 'value':0},
        {'name':'StabSwitch', 'value':0},
        {'name':'ManSwitch', 'value':0},
        {'name':'ModeSlot', 'value':0},
        {'name':'DataSource', 'value':0}]})


def vehicle_full_reverse():
    airliner.send_telemetry(
        {'name':'/Airliner/CNTL/ManualSetpoint', 'args':[
        {'name':'Timestamp', 'value': get_time()},
        {'name':'X', 'value':-1.0},
        {'name':'Y', 'value':0.0},
        {'name':'Z', 'value':0.5},
        {'name':'R', 'value':0.0},
        {'name':'Flaps', 'value':0.0},
        {'name':'Aux1', 'value':0.0},
        {'name':'Aux2', 'value':0.0},
        {'name':'Aux3', 'value':0.0},
        {'name':'Aux4', 'value':0.0},
        {'name':'Aux5', 'value':0.0},
        {'name':'ModeSwitch', 'value':0},
        {'name':'ReturnSwitch', 'value':0},
        {'name':'RattitudeSwitch', 'value':0},
        {'name':'PosctlSwitch', 'value':0},
        {'name':'LoiterSwitch', 'value':0},
        {'name':'AcroSwitch', 'value':0},
        {'name':'OffboardSwitch', 'value':0},
        {'name':'KillSwitch', 'value':0},
        {'name':'TransitionSwitch', 'value':0},
        {'name':'GearSwitch', 'value':0},
        {'name':'ArmSwitch', 'value':0},
        {'name':'StabSwitch', 'value':0},
        {'name':'ManSwitch', 'value':0},
        {'name':'ModeSlot', 'value':0},
        {'name':'DataSource', 'value':0}]})


def vehicle_full_right():
    airliner.send_telemetry(
        {'name':'/Airliner/CNTL/ManualSetpoint', 'args':[
        {'name':'Timestamp', 'value': get_time()},
        {'name':'X', 'value':0.0},
        {'name':'Y', 'value':1.0},
        {'name':'Z', 'value':0.5},
        {'name':'R', 'value':0.0},
        {'name':'Flaps', 'value':0.0},
        {'name':'Aux1', 'value':0.0},
        {'name':'Aux2', 'value':0.0},
        {'name':'Aux3', 'value':0.0},
        {'name':'Aux4', 'value':0.0},
        {'name':'Aux5', 'value':0.0},
        {'name':'ModeSwitch', 'value':0},
        {'name':'ReturnSwitch', 'value':0},
        {'name':'RattitudeSwitch', 'value':0},
        {'name':'PosctlSwitch', 'value':0},
        {'name':'LoiterSwitch', 'value':0},
        {'name':'AcroSwitch', 'value':0},
        {'name':'OffboardSwitch', 'value':0},
        {'name':'KillSwitch', 'value':0},
        {'name':'TransitionSwitch', 'value':0},
        {'name':'GearSwitch', 'value':0},
        {'name':'ArmSwitch', 'value':0},
        {'name':'StabSwitch', 'value':0},
        {'name':'ManSwitch', 'value':0},
        {'name':'ModeSlot', 'value':0},
        {'name':'DataSource', 'value':0}]})

def vehicle_land():
    airliner.send_telemetry(
        {'name':'/Airliner/CNTL/VehicleCommand', 'args':[
        {'name':'Timestamp', 'value': get_time()},
        {'name':'Command', 'value':int(MsgVehicleCmd.nav_land)},
        {'name':'Param1', 'value':0.0},
        {'name':'Param2', 'value':0.0},
        {'name':'Param3', 'value':0.0},
        {'name':'Param4', 'value':0.0},
        {'name':'Param5', 'value':0.0},
        {'name':'Param6', 'value':0.0},
        {'name':'Param7', 'value':0.0},
        {'name':'TargetSystem', 'value':1},
        {'name':'TargetComponent', 'value':1},
        {'name':'SourceSystem', 'value':0},
        {'name':'SourceComponent', 'value':0},
        {'name':'Confirmation', 'value':0}]})
    time.sleep(10)


def vehicle_fly_square_ccw():
    vehicle_full_forward()
    time.sleep(1)
    vehicle_stable_hover()
    time.sleep(1)
    vehicle_full_left()
    time.sleep(1)
    vehicle_stable_hover()
    time.sleep(1)
    vehicle_full_reverse()
    time.sleep(1)
    vehicle_stable_hover()
    time.sleep(1)
    vehicle_full_right()
    time.sleep(1)
    vehicle_stable_hover()
    time.sleep(1)


def vehicle_fly_square_cw():
    vehicle_full_forward()
    time.sleep(1)
    vehicle_stable_hover()
    time.sleep(1)
    vehicle_full_right()
    time.sleep(1)
    vehicle_stable_hover()
    time.sleep(1)
    vehicle_full_reverse()
    time.sleep(1)
    vehicle_stable_hover()
    time.sleep(1)
    vehicle_full_left()
    time.sleep(1)
    vehicle_stable_hover()
    time.sleep(1)

# vehicle control
vehicle_arm()
vehicle_takeoff()
vehicle_posctl_mode()
vehicle_stable_hover()
vehicle_fly_square_ccw()
vehicle_fly_square_cw()
vehicle_land()
vehicle_disarm()

# Send one NoOp command
airliner.send_command({'name':'/Airliner/ES/Noop'})
time.sleep(2)

# print received telemetry
es_hk_cmdcnt = airliner.tlm_value('/Airliner/ES/HK/CmdCounter')
print "es_hk_cmdcnt: " + str(es_hk_cmdcnt)
accel_sensor_combined = airliner.tlm_value('/Airliner/SENS/HK/Acc')
print "accel_sensor_combined_z: " + str(accel_sensor_combined[2])
baro_sensor_combined = airliner.tlm_value('/Airliner/SENS/HK/BaroAlt')
print "baro_sensor_combined: " + str(baro_sensor_combined)
# GPS not yet implemented on flight side
gps_lat = airliner.tlm_value('/Airliner/GPS/HK/Lat')
print "/Airliner/ES/GPS/Lat: " + str(gps_lat)
gps_lon = airliner.tlm_value('/Airliner/GPS/HK/Lon')
print "/Airliner/ES/GPS/Lon: " + str(gps_lon)

all_test_passed = True

# check gravity vector
if accel_sensor_combined[2] < -9.0 and accel_sensor_combined[2] > -11.0:
    description = "passed check gravity vector " + str(accel_sensor_combined[2])
    airliner.assert_true(True, description)
    print description
else:
    description = "failed check gravity vector " + str(accel_sensor_combined[2])
    airliner.assert_true(False,description)
    print description
    all_test_passed = False

# check baro height
if baro_sensor_combined > 0.0 and baro_sensor_combined  < 1.0:
    description = "passed check baro height " + str(baro_sensor_combined)
    airliner.assert_true(True, description)
    print description
else:
    description = "failed check baro height " + str(baro_sensor_combined)
    airliner.assert_true(False, description)
    print description
    all_test_passed = False
    
# check es command count
if es_hk_cmdcnt == 1:
    description = "passed check command count"
    airliner.assert_true(True, description)
    print description
else:
    description = "failed check command count"
    airliner.assert_true(False, description)
    print description
    all_test_passed = False

# print test results to log and generate junit for jenkins
airliner.finish_test()
