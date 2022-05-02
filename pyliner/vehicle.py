from pyliner.apps.flight_director import FlightDirector
from pyliner.apps.controller import Controller
from pyliner.apps.navigation import Navigation
from pyliner.apps.geofence import Geofence
from pyliner.apps.geographic_app import GeographicApp
from pyliner.apps.time_app import TimeApp

# from pyliner.apps import Controller, flight_director, Geofence, GeographicApp, \
#     Navigation, TimeApp
from pyliner.apps.geofence import LayerKind
from pyliner.base_vehicle import BaseVehicle


class Vehicle(BaseVehicle):
    """Represents a vehicle that the user may control.

    The basic apps that a vehicle presents are:
        com (Communication): Provides methods to communicate with the physical
            vehicle in the air.
        fd (FlightDirector): Provides methods for controlling the raw x, y, z,
            and rotation axes of the vehicle.
        nav (Navigation): Provides methods for controlling the direction and
            speed of the vehicle.

    The user is free to replace the defaults of these apps in the constructor
    but it is highly recommended that replacements subclass the default apps.
    """

    def __init__(self, vehicle_id, communication, geographic=None, time=None,
                 logger=None):
        """Create an instance of Pyliner.

        Args:
            vehicle_id: Vehicle ID. Should be unique.
            communication (Communication): Communication App.
                Not exposed as a public module for direct access, but the user
                is given the option to use a custom class if they desire.
            geographic: If None, defaults to Geographic().
            logger: If None, defaults to 'logging.getLogger(vehicle_id)'.
            time: If None, default to TimeSensor().
        """
        super(Vehicle, self).__init__(vehicle_id, logger)

        # Attributes
        self.atp_override = None

        # Component Defaults
        communication = communication
        controller = Controller()
        flight_director = FlightDirector()
        geofence = Geofence()
        geographic = geographic or GeographicApp()
        navigation = Navigation()
        time = time or TimeApp()
        #
        # # Attach defaults
        self.attach_app(communication)
        self.attach_app(geographic)
        self.attach_app(time)
        self.attach_app(geofence)
        # self.attach_app(controller)
        # self.attach_app(flight_director)
        # self.attach_app(navigation)
        #
        # # Add helpful default settings
        # geofence.add_layer(0, 'base', LayerKind.ADDITIVE)
        # navigation.defaults.update({'timeout': None, 'underflow': 5.0})
