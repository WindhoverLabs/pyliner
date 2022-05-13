# TODO Document all of these with expected arguments. Maybe move them into
# their own Apps. These shouldn't really be defined outside of an App for
# complete App independence, although it might be useful to have a few pre-
# declared actions available for everyone. I don't know.

# Geographic
ACTION_CALC_PBD = 'ACTION_CALC_PBD'
ACTION_CALC_BEARING = 'ACTION_CALC_BEARING'
ACTION_CALC_DISTANCE = 'ACTION_CALC_DISTANCE'

# Controller
ACTION_ARM = 'ACTION_ARM'
ACTION_ATP = 'ACTION_ATP'
ACTION_DISARM = 'ACTION_DISARM'
ACTION_RTL = 'ACTION_RTL'
ACTION_TAKEOFF = 'ACTION_TAKEOFF'

# Communication
ACTION_SEND_COMMAND = 'ACTION_SEND_COMMAND'
ACTION_SEND_COMMAND = 'ACTION_SEND_TELEMETRY'
ACTION_SEND_BYTES = 'ACTION_SEND_BYTES'

ACTION_CONTROL_GRANT = 'ACTION_CONTROL_GRANT'
ACTION_CONTROL_REQUEST = 'ACTION_CONTROL_REQUEST'
ACTION_CONTROL_RELEASE = 'ACTION_CONTROL_RELEASE'
ACTION_CONTROL_REVOKE = 'ACTION_CONTROL_REVOKE'

ACTION_TELEM = 'ACTION_TELEM'

# BaseVehicle
ACTION_APP_ATTACH = 'ACTION_APP_ATTACH'
ACTION_APP_DETACH = 'ACTION_APP_DETACH'
ACTION_APP_LIST = 'ACTION_APP_LIST'

ACTION_VEHICLE_SHUTDOWN = 'ACTION_VEHICLE_SHUTDOWN'

# FlightDirector
ACTION_AXIS_SET = 'ACTION_AXIS_SET'
ACTION_AXIS_ZERO = 'ACTION_AXIS_ZERO'

# Navigation
ACTION_GOTO = 'ACTION_GOTO'
