from os import path, sys
sys.path.append( path.dirname( path.dirname( path.abspath(__file__) ) ) )

import pyliner
import time

# Callback 1
def cb_1(data):
    print "%s = %s" % (data['name'], data['value'])
    
# Callback 2
def cb_2(data):
    print "%s = %s" % (data['name'], data['value'])

# Initialize pyliner object
airliner = pyliner.Pyliner(**{"airliner_map": "cookiecutter.json", 
                      "ci_port": 5009,
                      "to_port": 5012,
                      "script_name": "demo_test",
                      "log_dir": "./logs/"})

# Subscribe to only CmdCounter for callback 1
airliner.subscribe({'tlm': ['/Airliner/ES/HK/CmdCounter']}, cb_1)       

# Subscribe to remaining items for callback 2             
airliner.subscribe({'tlm': ['/Airliner/ES/HK/MaxProcessorResets', 
                            '/Airliner/ES/HK/SysLogMode', 
                            '/Airliner/ES/HK/SysLogBytesUsed', 
                            '/Airliner/ES/HK/SysLogEntries']}, cb_2)

# Perform 15 batches of sending commands and receiving telemetry
for i in range(15):
    # Send NoOp command
    airliner.send_command({'name':'/Airliner/ES/Noop'})
    
    # Set max cpu resets equal to loop iteration
    airliner.send_command({'name':'/Airliner/ES/SetMaxPRCount', 'args':[
                                 {'name':'MaxProcResets', 'value':i}]})
    # At 10 iterations clear logs
    if i == 10:
        airliner.send_command({'name':'/Airliner/ES/ClearSysLog'})
        airliner.send_command({'name':'/Airliner/ES/ClearERLog'})
    
    # Toggle log mode on even/odd iterations
    if i % 2 == 0:
        airliner.send_command({'name':'/Airliner/ES/OverwriteSysLog', 'args':[
                             {'name':'OverwriteMode', 'value':1}]})
    else:
        airliner.send_command({'name':'/Airliner/ES/OverwriteSysLog', 'args':[
                             {'name':'OverwriteMode', 'value':0}]})
    time.sleep(1)
