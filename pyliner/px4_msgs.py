from flufl.enum import Enum


class MavModeFlag(Enum):
    custom                      = 1
    test                        = 2
    auto                        = 4
    guided                      = 8
    stabilize                   = 16
    hil                         = 32
    manual                      = 64
    safety                      = 128
    end                         = 129


class MavCustomMainMode(Enum):
    manual                      = 1
    altctl                      = 2
    posctl                      = 3
    auto                        = 4
    acro                        = 5
    offboard                    = 6
    stabilized                  = 7
    rattitude                   = 8
    simple                      = 9


class MavCustomSubModeAuto(Enum):
    ready                       = 1
    takeoff                     = 2
    loiter                      = 3
    mission                     = 4
    rtl                         = 5
    land                        = 6
    rtgs                        = 7
    follow_target               = 8


class MsgVehicleCmd(Enum):
    custom0                     = 0
    custom1                     = 1
    custom2                     = 2
    nav_waypoint                = 16
    nav_loiter_unlim            = 17
    nav_loiter_turns            = 18
    nav_loiter_time             = 19
    nav_return_to_launch        = 20
    nav_land                    = 21
    nav_takeoff                 = 22
    nav_precland                = 23
    nav_roi                     = 80
    nav_path_planning           = 81
    nav_vtol_takeoff            = 84
    nav_vtol_land               = 85
    nav_guided_limits           = 90
    nav_guided_master           = 91
    nav_guided_enabled          = 92
    nav_delay                   = 93
    nav_last                    = 95
    condition_delay             = 112
    condition_change_alt        = 113
    condition_distance          = 114
    condition_yaw               = 115
    condition_last              = 159
    do_set_mode                 = 176
    do_jump                     = 177
    do_change_speed             = 178
    do_set_home                 = 179
    do_set_parameter            = 180
    do_set_relay                = 181
    do_repeat_relay             = 182
    do_set_servo                = 183
    do_repeat_servo             = 184
    do_flight_termination       = 185
    do_land_start               = 189
    do_go_around                = 191
    do_reposition               = 192
    do_pause_continue           = 193
    do_set_roi_location         = 195
    do_set_roi_wpnext_off       = 196
    do_set_roi_none             = 197
    do_control_video            = 200
    do_set_roi                  = 201
    do_digicam_control          = 203
    do_mount_configure          = 204
    do_mount_control            = 205
    do_set_cam_trigg_dist       = 206
    do_fence_enable             = 207
    do_parachute                = 208
    do_inverted_flight          = 210
    do_set_cam_trigg_int        = 214
    do_mount_control_quat       = 220
    do_guided_master            = 221
    do_guided_limits            = 222
    do_last                     = 240
    preflight_calibration       = 241
    preflight_set_sens_offs     = 242
    preflight_storage           = 245
    preflight_reboot_shutdown   = 246
    mission_start               = 300
    component_arm_disarm        = 400
    start_rx_pair               = 500
    set_camera_mode             = 530
    do_trigger_control          = 2003
    do_vtol_transition          = 3000
    arm_auth_request            = 3001
    payload_prepare_deploy      = 30001
    payload_control_deploy      = 30002
    preflight_uavcan            = 243
    logging_start               = 2510
    logging_stop                = 2511
