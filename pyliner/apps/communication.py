"""
The communication module provides a Communication Service for sending commands
over UDP to the vehicle.

Services:
    Communication  Handles UDP interface with physical vehicle.
"""
import random
import threading
import json

import enum
import time
from datetime import datetime

from pyliner import util
from pyliner.apps.xtce_msg_parser import XTCEParser
from datetime import datetime

try:
    import queue
except ModuleNotFoundError:
    import Queue

import socketserver

from pyliner.action import ACTION_SEND_COMMAND, ACTION_SEND_BYTES, ACTION_TELEM, \
    ACTION_CONTROL_REQUEST, ACTION_CONTROL_GRANT, ACTION_CONTROL_REVOKE, \
    ACTION_CONTROL_RELEASE
from pyliner.arte_ccsds import CCSDS_TlmPkt_t, CCSDS_CmdPkt_t
from pyliner.conversions import hertz
from pyliner.intent import IntentFilter, Intent, FutureTimeoutError, \
    IntentExplicitFailure
from pyliner.pyliner_error import PylinerError
from pyliner.app import App
from pyliner.util import init_socket, handler_factory, CallableDefaultDict, \
    RealTimeThread, OrderedSetQueue


class ParseMode(enum.IntEnum):
    PROTOBUF = 1
    XTCE = 2


class CommandAuthorizationError(PylinerError):
    """Raised if an App does not have permission to send commands."""
    pass


class InvalidCommandException(PylinerError):
    """Raised if a command does not exist."""
    pass


class InvalidOperationException(PylinerError):
    """Raised if an operation is invalid."""
    pass


class _Telemetry(object):
    """Represents an operational path of telemetry from Airliner."""

    def __init__(self, name=None, time=None, value=None):
        self.name = name
        self.time = time
        self.value = value
        self.obj = None

        self._listener = set()

    def add_listener(self, listener):
        if not callable(listener):
            raise TypeError('Listener must be callable.')
        self._listener.add(listener)

    def remove_listener(self, listener):
        self._listener.remove(listener)

    def update(self, value, time=None, obj=None):
        self.time = time
        self.value = value
        self.obj = obj
        for listener in self._listener:
            listener(self)


class ControlToken(object):
    """Created by the Communication App and passed to Apps that are granted
    control of the vehicle. All commands sent that require authentication
    must use the request method to wrap their request."""

    def __init__(self, app_name):
        self.app_name = app_name
        # self._token =

    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.app_name[-10:])

    def request(self, data):
        return ControlRequest(self, data)


class ControlRequest(object):
    """Created via a ControlToken, used to authenticate a request sent to the
    Communication App that requires granted control of the vehicle."""

    def __init__(self, token, data):
        self.token = token
        """:type: ControlToken"""
        self.data = data

    def __repr__(self):
        return '{}({}, {})'.format(
            self.__class__.__name__, self.token, self.data)


class Communication(App):
    """Provide methods to send and receive telemetry to a vehicle.

    In some ways this service acts more like a singleton that is guaranteed to
    be present on the vehicle, because there is only one link to the software
    bus, and if there is no link the user could not communicate anyway. As much
    as possible, this service tries not to act like a God Object, but
    occasionally it happens.

    Access to sending the vehicle raw bytes or to-be-serialized commands is
    guarded through control tokens generated by this App and sent to Apps when
    their request for control is granted.

    Requesting Telemetry:
        This App responds to ACTION_TELEM intents. The data attribute of the
        intent must be either a string, a list, or a dictionary.

        Responses will be of the form:
            <op_path> -> Telemetry(op_path)
            [<op_pathX>, ...] -> [Telemetry(op_path), ...]
            {keyX: <op_pathY>, ...} -> {keyX: Telemetry(op_pathY), ...}

        Where op_path is the operational path for some telemetry on the vehicle,
        and Telemetry(op_path) is a telemetry object representing that path.

    Requesting Control:
        Apps request control by broadcasting an ACTION_CONTROL_REQUEST. The
        origin field of the intent must be filled with the requesting App name.
        If the request was enqueued it will reply True, otherwise it will reply
        False. Duplicate requests will also return False.

        Eventually the enqueued request will pass through the control queue. The
        requesting App will be sent an ACTION_CONTROL_GRANT intent, which must
        be promptly acknowledged by sending a response of True. The intent will
        include a ControlToken in its data attribute. The data attribute of all
        commands sent to the vehicle must be wrapped using the token's request()
        method. The App may now send ACTION_SEND_COMMAND and ACTION_SEND_BYTES
        intents that will be sent to the vehicle.

        From the receiving App's side it might look like this:
        >>> def granted(self, intent):
        ...     self.token = intent.data
        ...     self.vehicle.broadcast(Intent(
        ...         action=ACTION_SEND_COMMAND,
        ...         data=self.token.request(some_command())))

        When an App is done controlling the vehicle it must broadcast an
        ACTION_CONTROL_RELEASE, with the given ControlToken if it is currently
        in control, or no data.

        If an App is in control of the vehicle for an excessive amount of time,
        and there are other Apps waiting for control, the App will receive an
        ACTION_CONTROL_REVOKE, indicating that it is no longer in control. The
        app is placed back into the control queue automatically and eventually
        it will receive another GRANT intent after the Apps in the queue before
        it have been granted control.

        If an App does not promptly respond to a GRANT intent with True, the
        system will assume that the App no longer needs control or has stalled,
        and will REVOKE control without placing it back in the control queue.
    """
    CONTROL_ACK_WAIT = 1.0 / 16.0
    CONTROL_ROTATE_EVERY = hertz(4)

    def __init__(self, airliner_map, parse_mode: ParseMode, parser: XTCEParser, address='localhost',
                 ci_port=5110, to_port=5012):
        """
        Args:
            airliner_map (dict): Airliner Mapping, typically read from a JSON.
            address (str): Address to connect to the vehicle.
            ci_port (int): Command-Ingest port
            to_port (int): Telemetry-Output port
        """
        super(Communication, self).__init__()

        if not isinstance(airliner_map, dict):
            raise TypeError('airliner_map is expecting a dict but got {}'
                            .format(type(airliner_map)))

        # Telemetry variables
        self.address = address
        self.airliner_map = airliner_map
        self.all_telemetry = []
        self.ci_port = ci_port
        self.ci_socket = init_socket()
        self.control_current = None
        """:type: ControlToken"""
        self.control_lock = threading.RLock()
        self.control_thread = None
        """:type: PeriodicExecutor"""
        self.control_queue = OrderedSetQueue()
        self.subscribers = []
        self.to_port = to_port

        self._telemetry = CallableDefaultDict(
            default_factory=lambda k: self.subscribe(k))
        """:type: dict[str, _Telemetry]"""

        # Receive Telemetry
        # self.tlm_listener = socketserver.UDPServer(
        #     ("0.0.0.0", self.to_port), handler_factory(self._on_recv_telemetry))
        # self.listener_thread = threading.Thread(
        #     target=self.tlm_listener.serve_forever)
        #
        # self.listener_thread.setDaemon(True)
        # self.listener_thread.start()
        self.parse_mode = parse_mode
        self.parser = parser
        for i in range(2048):
            time.sleep(.004)
            self.send_command("/cfs/cpd/apps/px4lib/PX4_POSITION_SETPOINT_TRIPLET_MID")

    def attach(self, vehicle):
        super(Communication, self).attach(vehicle)
        self.control_thread = RealTimeThread(
            name='ControlRotateThread', target=self.control_rotate,
            every=Communication.CONTROL_ROTATE_EVERY)
        self.control_thread.start()

        def filter_control(data, call):
            if not isinstance(data, ControlRequest):
                raise TypeError(
                    'Sending data must be wrapped in a ControlRequest produced '
                    'from a valid ControlToken.')
            elif data.token is not self.control_current:
                raise CommandAuthorizationError(
                    'App is not authorized to send commands.')
            else:
                return call(data.data)

        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_SEND_COMMAND]),
            lambda i: filter_control(i.data, self.send_command))
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_SEND_BYTES]),
            lambda i: filter_control(i.data, self.send_bytes))
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_TELEM]),
            lambda i: self.telemetry(i.data))
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_CONTROL_REQUEST]),
            self.control_request)
        self.vehicle.add_filter(
            IntentFilter(actions=[ACTION_CONTROL_RELEASE]),
            self.control_release)

    def detach(self):
        self.vehicle.clear_filter()
        self.control_thread.stop()
        super(Communication, self).detach()

    def control_grant(self):
        """Generate a ControlToken and pass it to the App which was granted
        control.

        Assumes control_lock
        """

        def get_or_stop():
            try:
                yield self.control_queue.get(block=False)
            except queue.Empty:
                raise StopIteration

        for app_name in get_or_stop():
            # self.debug('Grant Control: {}'.format(app_name))
            self.control_current = ControlToken(app_name=app_name)
            try:
                control_ack = self.vehicle.broadcast(Intent(
                    action=ACTION_CONTROL_GRANT, component=app_name,
                    data=self.control_current
                )).first(Communication.CONTROL_ACK_WAIT).result
            except IntentExplicitFailure:  # App was disconnected
                self.error('Broadcast could not find {}'.format(app_name))
                # self.control_current = None
                continue
            except FutureTimeoutError:
                self.error('{} did not acknowledge control handoff within '
                           '{} seconds.'.format(
                    app_name, Communication.CONTROL_ACK_WAIT))
                self.control_revoke()
                continue
            else:
                if control_ack is not True:
                    self.debug('{} refused control.'.format(app_name))
                    self.control_revoke()
                    continue
                else:
                    return  # Control granted successfully.

    def control_release(self, intent):
        """Called if an App willingly releases control."""
        with self.control_lock:
            success = True
            if intent.origin in self.control_queue:
                self.debug('Control request dequeued by ' + str(intent.origin))
                self.control_queue.remove(intent.origin)
            elif intent.data is self.control_current:
                self.debug('Control Release by ' + str(intent.data.app_name))
                self.control_current = None
                self.control_grant()
            else:
                self.error('Invalid control release from ' + str(intent.data))
                success = False
            return success

    def control_request(self, intent):
        """Called when an App requests control. The App name is put into
        a queue.

        Returns:
            True if the request was enqueued. False otherwise.
        """
        with self.control_lock:
            if not intent.origin or intent.origin in self.control_queue:
                return False
            else:
                self.control_queue.put(intent.origin)
                if not self.control_current:
                    self.control_grant()
                return True

    def control_revoke(self, reason=None):
        self.vehicle.broadcast(Intent(
            action=ACTION_CONTROL_REVOKE, data=reason,
            component=self.control_current.app_name
        ))
        self.control_current = None

    def control_rotate(self):
        """If other Apps are waiting for control, rotate control.

        Put App which was rotated out back into queue.
        """
        with self.control_lock:
            if self.control_current is not None \
                    and not self.control_queue.empty():
                self.debug('Rotate Out: ' + str(self.control_current.app_name))
                self.control_queue.put(self.control_current.app_name)
                self.control_revoke()
                self.control_grant()

    @property
    def qualified_name(self):
        return 'com.windhover.pyliner.apps.communication'

    def send_bytes(self, message: bytes):
        self.ci_socket.sendto(message, (self.address, self.ci_port))
        return True

    def send_command(self, telemetry_path: str):
        buffer = self._serialize(telemetry_path)

        # self.vehicle.debug(
        #     'Sending telemetry to airliner: %s', msg)
        self.send_bytes(buffer)
        return True

    def telemetry(self, args):
        if isinstance(args, str):
            return self._telemetry[args]
        elif isinstance(args, dict):
            return {k: self._telemetry[v] for k, v in args.items()}
        elif isinstance(args, list):
            return [self._telemetry[t] for t in args]
        else:
            raise TypeError('Can only parse str, dict, and list data.')

    def _get_airliner_op(self, op_path):
        """
        Receive a ops path and returns the dict for that op defined in
        the input file.

        Args:
            op_path (str): Operational path of command to retrieve. Leading "/"
                is required. "/Airliner/CNTL/ManualSetpoint"

        Returns:
            dict: Operational dictionary of requested path.

        Raises:
            KeyError: If any part of the Airliner map is invalid. Including if
                the requested path does not exist.
        """
        ops_names = op_path.split('/')[1:]

        def get_app(app_ops_name):
            for app in self.airliner_map[ops_names[0]]['apps'].values():
                if app['app_ops_name'] == app_ops_name:
                    return app

        return get_app(ops_names[1])['operations'][ops_names[2]]

    def get_op_mid(self, op_path):
        """
        Receive a ops path and returns the message ID for that op defined in
        the input file.

        Args:
            op_path (str): Operational path of command to retrieve. Leading "/"
                is required. "/Airliner/CNTL/ManualSetpoint"

        Returns:
            int: Message ID of op if found
            None: If op does not exist
        """
        try:
            op = self._get_airliner_op(op_path)
        except KeyError:
            return None

        return int(op["airliner_mid"], 0)

    @staticmethod
    def _get_ccsds_msg(op):
        """Receive a ops dict and returns a ccsds msg

        Args:
            op (dict): Operational dictionary of an app operation.

        Returns:
            Union[CCSDS_TlmPkt_t, CCSDS_CmdPkt_t]: Ctypes packet structure
                based on the operational dictionary.
        """
        # If the command code is -1 this is telemetry
        if op["airliner_cc"] == -1:
            ret_msg = CCSDS_TlmPkt_t()
            ret_msg.clear_packet()
            ret_msg.init_packet()
            ret_msg.PriHdr.StreamId.bits.app_id = int(op["airliner_mid"], 0)
        else:
            ret_msg = CCSDS_CmdPkt_t()
            ret_msg.clear_packet()
            ret_msg.init_packet()
            ret_msg.PriHdr.StreamId.bits.app_id = int(op["airliner_mid"], 0)
            ret_msg.SecHdr.Command.bits.code = int(op["airliner_cc"])
        return ret_msg

    def _get_op_attr(self, op_path):
        """ Gets the operation path from airliner data for a specified attribute

        Args:
            op_path (str): Operation path as located in input file
                (E.g. "/Airliner/ES/Noop")

        Returns:
            True path to access this attribute in protobuf message
                (E.g. Payload.CmdCounter)
        """
        op = self._get_airliner_op(op_path)
        ops_names = op_path.split('/')[1:]
        if len(ops_names) == 3:
            return None

        for fsw, fsw_data in self.airliner_map.items():
            if fsw == ops_names[0]:
                for app, app_data in fsw_data["apps"].items():
                    if app_data["app_ops_name"] == ops_names[1]:
                        for op_name, op_data in app_data["proto_msgs"] \
                                [op["airliner_msg"]] \
                                ["operational_names"].items():
                            if op_name == ops_names[3]:
                                return op_data["field_path"]
        return None

    def _on_recv_telemetry(self, tlm: (bytes,)):
        """ Callback for TO socket listener

        Args:
            tlm(str): Raw bytes received from socket
        """
        self.all_telemetry.append(tlm)
        # FIXME:Use these for unit testing
        # tlm_value = self.parser.validate_packet(tlm[0],
        #                                         "/cfs/cpd/core/cfe/cfe_es/CFE_ES_APP_TLM_MID.Payload.AppInfo.Name", 0)
        # tlm_value = self.parser.validate_packet(tlm[0], "/cfs/cpd/core/cfe/cfe_es/CFE_ES_APP_TLM_MID.Payload.AppInfo.Type", 0)
        # tlm_value = self.parser.validate_packet(tlm[0], "/cfs/cpd/apps/ci/CI_HK_TLM_MID.usCmdCnt", 0)
        tlm_value = self.parser.validate_packet(tlm[0], "/cfs/cpd/core/cfe/cfe_es/CFE_ES_HK_TLM_MID.Payload.PerfFilterMask", 0)
        if tlm_value is not None:
            print(f'value:{tlm_value}')
        # self.vehicle.debug("Recvd tlm: %s", tlm)

        # self.parser.validate_packet(tlm[0], None)
        # # TODO: Check if needed

        # Iterate over subscribed telemetry to check if we care
        for subscribed_tlm in self.subscribers:
            # if self.parse_mode == ParseMode.PROTOBUF:
            #     # if int(subscribed_tlm['airliner_mid'], 0) == \
            #     #         int(tlm_pkt.PriHdr.StreamId.data):
            #         # Get pb msg for this msg
            #         op_path = subscribed_tlm['op_path']
            #         # pb_msg = self._get_pb_decode_obj(tlm[0][12:], op_path)
            #         callback = subscribed_tlm['callback']
            #         telemItem = subscribed_tlm['telemItem']
            #
            #         # telemItem.update(
            #         #     value=self._get_pb_value(pb_msg, op_path), time=tlm_time, obj=pb_msg)
            #
            #         # Update telemetry dictionary with fresh data
            #         # self._telemetry[op_path].update(
            #         #     value=self._get_pb_value(pb_msg, op_path), time=tlm_time)
            #
            #         if callable(callback):
            #             callback(self._telemetry[op_path])
            if self.parse_mode == ParseMode.XTCE:
                self.parser.validate_packet(tlm[0], subscribed_tlm['op_path'])

    def subscribe(self, tlm_item, callback=0):
        """
        Receives an operational path to an airliner msg attribute to subscribe
        to, as well as an optional callback function.

        Args:
            tlm_item (dict[str, list[str]]): Dictionary specifying the telemetry
                items to subscribe to, using the telemetry item's operational
                names.
                E.g. {'tlm': ['/Airliner/ES/HK/CmdCounter']}
                    or
                     {'tlm': ['/Airliner/ES/HK/CmdCounter',
                              '/Airliner/ES/HK/ErrCounter']}
        """

        self.vehicle.info('Subscribing to: {}'.format(tlm_item))
        # Get operation for specified telemetry
        op = self._get_airliner_op(tlm_item)
        if not op:
            err_msg = "Invalid telemetry operational name received. " \
                      "Operation (%s) not defined." % tlm_item
            self.vehicle.error(err_msg)
            raise InvalidOperationException(err_msg)

        if not self._get_op_attr(tlm_item) and len(tlm_item.split('/')) == 5:
            err_msg = "Invalid telemetry operational name received. " \
                      "Symbol: {} exists, but attribute: {} does not.".format(
                tlm_item.split('/')[-2], tlm_item.split('/')[-1])
            self.vehicle.error(err_msg)
            raise InvalidOperationException(err_msg)

        newTelemetry = _Telemetry(name=tlm_item)

        # Add entry to subscribers list
        if self.parse_mode == ParseMode.PROTOBUF:
            self.subscribers.append({'op_path': tlm_item,
                                     'airliner_mid': op['airliner_mid'],
                                     'tlmSeqNum': 0,
                                     'callback': callback,
                                     'telemItem': newTelemetry})
        elif self.parse_mode == ParseMode.XTCE:
            self.subscribers.append({'op_path': tlm_item,
                                     'airliner_mid': op['airliner_mid'],
                                     'tlmSeqNum': 0,
                                     'callback': callback,
                                     'telemItem': newTelemetry})
        # Add entry to telemetry dictionary to prevent key errors
        # in user scripts and set default values.
        return newTelemetry

    def _serialize(self, path: str) -> bytes:
        # FIXME:Too large int for python
        dt = datetime.today()  # Get timezone naive now
        t = int(dt.timestamp())
        args = {'Timestamp': 5,
                'Previous.Lat': 150,
                'Previous.Lon': 1.0,
                'Previous.X': 1.0,
                'Previous.Y': 0.0,
                'Previous.Z': 1.0,
                'Previous.VX': 20.0,
                'Previous.VY': 1.0,
                'Previous.VZ': 1.0,
                'Previous.Alt': 1.0,
                'Previous.Yaw': 1.0,
                'Previous.Yawspeed': 10.0,
                'Previous.LoiterRadius': 200.0,
                'Previous.PitchMin': 1.0,
                'Previous.AX': 1.0,
                'Previous.AY': 1.0,
                'Previous.AZ': 1.0,
                'Previous.AcceptanceRadius': 1.0,
                'Previous.CruisingSpeed': 100.0,
                'Previous.CruisingThrottle': -1.0,
                'Previous.Valid': 1,
                'Previous.Type': 1,
                'Previous.PositionValid': 1,
                'Previous.VelocityValid': 1,
                'Previous.YawValid': 1,
                'Previous.DisableMcYawControl': 1,
                'Previous.YawspeedValid': 1,
                'Previous.LoiterDirection': 1,
                'Previous.AccelerationValid': 1,
                'Previous.AccelerationIsForce': 1,
                'Previous.VelocityFrame': 1,
                'Current.Lat': 200.0,
                'Current.Lon': 1.0,
                'Current.X': 1.0,
                'Current.Y': 1.0,
                'Current.Z': 1.0,
                'Current.VX': 1.0,
                'Current.VY': 1.0,
                'Current.VZ': 1.0,
                'Current.Alt': 1.0,
                'Current.Yaw': 1.0,
                'Current.Yawspeed': 1.0,
                'Current.LoiterRadius': 1.0,
                'Current.PitchMin': 1.0,
                'Current.AX': 1.0,
                'Current.AY': 1.0,
                'Current.AZ': 1.0,
                'Current.AcceptanceRadius': 1.0,
                'Current.CruisingSpeed': 1.0,
                'Current.CruisingThrottle': 1.0,
                'Current.Valid': 1,
                'Current.Type': 1,
                'Current.PositionValid': 1,
                'Current.VelocityValid': 1,
                'Current.YawValid': 1,
                'Current.DisableMcYawControl': 1,
                'Current.YawspeedValid': 1,
                'Current.LoiterDirection': 1,
                'Current.AccelerationValid': 1,
                'Current.AccelerationIsForce': 1,
                'Current.VelocityFrame': 1,
                'Next.Lat': 1.0,
                'Next.Lon': 1.0,
                'Next.X': 1.0,
                'Next.Y': 1.0,
                'Next.Z': 30.0,
                'Next.VX': 1.0,
                'Next.VY': 1.0,
                'Next.VZ': 1.0,
                'Next.Alt': 1.0,
                'Next.Yaw': 1.0,
                'Next.Yawspeed': 20.0,
                'Next.LoiterRadius': 1.0,
                'Next.PitchMin': 1.0,
                'Next.AX': 1.0,
                'Next.AY': 1.0,
                'Next.AZ': 1.0,
                'Next.AcceptanceRadius': 1.0,
                'Next.CruisingSpeed': 120.0,
                'Next.CruisingThrottle': -1.0,
                'Next.Valid': 1,
                '_spare0': 0,
                'Next.Type': 1,
                'Next.PositionValid': 1,
                'Next.VelocityValid': 1,
                'Next.YawValid': 1,
                'Next.DisableMcYawControl': 1,
                'Next.YawspeedValid': 1,
                'Next.LoiterDirection': 1,
                'Next.AccelerationValid': 1,
                'Next.AccelerationIsForce': 1,
                'Next.VelocityFrame': 1,
                }
        for arg in list(args.keys())[:2]:
            args[arg] = random.uniform(1.1, 20.5) * random.randint(0, 30000)

        return self.parser.craft_tlm_command(path, args)

        # return self.parser.craft_tlm_command("/cfs/cpd/apps/px4lib/PX4_ACTUATOR_ARMED_MID", { })
