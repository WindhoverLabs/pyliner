from os import path, sys
sys.path.append( path.dirname( path.dirname( path.abspath(__file__) ) ) )
from pyliner import Pyliner
import time

# Initialize pyliner object
airliner = Pyliner(**{"airliner_map": "cookiecutter.json", 
                      "ci_port": 5009,
                      "to_port": 5012,
                      "script_name": "demo_test",
                      "log_dir": "./logs/"})

# Subscribe to desired telemetry
airliner.subscribe({'tlm': ['/Airliner/ES/HK/CmdCounter',
                            '/Airliner/ES/HK/SysLogMode', 
                            '/Airliner/ES/HK/MaxProcessorResets', 
                            '/Airliner/ES/HK/SysLogBytesUsed', 
                            '/Airliner/ES/HK/SysLogEntries']})

# Perform 15 batches of sending commands and receiving telemetry
for i in range(15):
    # Get entire telemetry data dictionary for CmdCounter
    cmd_count_data = airliner.tlm('/Airliner/ES/HK/CmdCounter')
    
    # Print all data for CmdCounter
    print "Telemetry: %s   Value: %s   Timestamp: %s" % (cmd_count_data['name'],
                                                         cmd_count_data['value'], 
                                                         cmd_count_data['time'])
    
    # Get just the value of other subscribed items
    log_mode = airliner.tlm_value('/Airliner/ES/HK/SysLogMode')
    max_proc_resets = airliner.tlm_value('/Airliner/ES/HK/MaxProcessorResets')
    log_size = airliner.tlm_value('/Airliner/ES/HK/SysLogBytesUsed')
    log_entries = airliner.tlm_value('/Airliner/ES/HK/SysLogEntries')

    # Print the fresh telemetry values
    print "Log mode: " + str(log_mode)
    print "Max processor resets: " + str(max_proc_resets)
    print "Log size: " + str(log_size)
    print "Log entries: " + str(log_entries)

    # Send NoOp command
    airliner.send_message({'name': '/Airliner/ES/Noop'})
    
    # Set max cpu resets equal to loop iteration
    airliner.send_message({'name': '/Airliner/ES/SetMaxPRCount', 'args':[
                                 {'name':'MaxProcResets', 'value':i}]})
    # At 10 iterations clear logs
    if i == 10:
        airliner.send_message({'name': '/Airliner/ES/ClearSysLog'})
        airliner.send_message({'name': '/Airliner/ES/ClearERLog'})
    
    # Toggle log mode on even/odd iterations
    if i % 2 == 0:
        airliner.send_message({'name': '/Airliner/ES/OverwriteSysLog', 'args':[
                             {'name':'OverwriteMode', 'value':1}]})
    else:
        airliner.send_message({'name': '/Airliner/ES/OverwriteSysLog', 'args':[
                             {'name':'OverwriteMode', 'value':0}]})
                             
    # Sleep for duration of a major frame
    time.sleep(1)
