#!/usr/bin/env python

# Imports to start Isaac Sim from this script
import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": True})

from omni.isaac.core.utils.extensions import enable_extension
# Default Livestream settings
simulation_app.set_setting("/app/fastShutdown", True)
simulation_app.set_setting("/app/livestream/proto", "ws")
simulation_app.set_setting("/app/livestream/allowResize", True)
simulation_app.set_setting("/app/window/title", "Isaac Sim Headless Native Livestream")
simulation_app.set_setting("/app/window/drawMouse", True)
simulation_app.set_setting("/app/window/hideUi", False)
simulation_app.set_setting("/app/window/width", 1920)
simulation_app.set_setting("/app/window/height", 1080)

# Enable Native Livestream extension
enable_extension("omni.kit.streamsdk.plugins-3.2.1")
enable_extension("omni.kit.livestream.core-3.2.0")
enable_extension("omni.kit.livestream.native-4.1.0")


# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.core.utils.extensions import disable_extension, enable_extension

# Enable/disable ROS bridge extensions to keep only ROS2 Bridge
disable_extension("omni.isaac.ros_bridge")
enable_extension("omni.isaac.ros2_bridge")

# Import the Pegasus API for simulating drones
import sys, os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(os.path.join(BASE_DIR, 'PegasusSimulator/extensions/pegasus.simulator'))
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend
from pegasus.simulator.logic.graphs import ROS2CameraGraph
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Auxiliary scipy and numpy modules
from scipy.spatial.transform import Rotation

class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        # Create the vehicle
        # Try to spawn the selected robot in the world to the specified namespace
        config_multirotor = MultirotorConfig()
        config_multirotor.backends = [ROS2Backend(vehicle_id=1, config={
            "namespace": 'drone',
            "pub_sensors": True,
            "pub_graphical_sensors": True,
            "pub_state": True,
            "pub_tf": False,
            "sub_control": False}
            ),
            MavlinkBackend()
        ]
        config_multirotor.graphical_sensors = [
            MonocularCamera("cam_0", config={"stage_prim_path":"/World/quadrotor/body/OmniCam/cam_0", 'resolution': [1280, 960], "update_rate": 60.0}), 
            MonocularCamera("cam_1", config={"stage_prim_path":"/World/quadrotor/body/OmniCam/cam_1", 'resolution': [1280, 960], "update_rate": 60.0}),
            MonocularCamera("cam_2", config={"stage_prim_path":"/World/quadrotor/body/OmniCam/cam_2", 'resolution': [1280, 960], "update_rate": 60.0}), 
            MonocularCamera("cam_3", config={"stage_prim_path":"/World/quadrotor/body/OmniCam/cam_3", 'resolution': [1280, 960], "update_rate": 60.0})
        ]

        Multirotor(
            "/World/quadrotor",
            ROBOTS['OmniNxt'],
            0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:

            # Update the UI of the app and perform the physics step
            self.world.step(render=True)
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():

    # Instantiate the template app
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()
