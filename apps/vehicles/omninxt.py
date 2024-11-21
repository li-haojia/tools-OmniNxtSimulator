import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

from .quadrotor import Quadrotor
from sensors.camera import ROS2Camera

class OmniNxt(Quadrotor):

    # Configuration
    config = {
        "alpha0": 45.0, # deg
        "mass": 0.8, # kg
        "gravity": 9.81, # m/s^2
        "prop_radius": 0.045, # m
        "arm_length": 0.11, # m
        "min_rpm": 1200.0, # rpm
        "max_rpm": 35000.0, # rpm
    }
    enable_cameras = False
    body_path = None

    def __init__(self, id, init_pos = (0.0, 0.0, 0.5), enable_cameras = False):
        super().__init__(id, self.config)
        self.enable_cameras = enable_cameras
        self.ISAAC_SIM_CFG = ArticulationCfg(
            prim_path=f"/World{self.name_space}/OmniNxt",
            spawn=sim_utils.UsdFileCfg(
                usd_path=f"{__file__}/../assets/omninxt.usd",
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=False,
                    max_depenetration_velocity=10.0,
                    enable_gyroscopic_forces=True,
                ),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=False,
                    solver_position_iteration_count=4,
                    solver_velocity_iteration_count=0,
                    sleep_threshold=0.005,
                    stabilization_threshold=0.001,
                ),
                copy_from_source=False,
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=init_pos,
                joint_pos={
                    ".*": 0.0,
                },
                joint_vel={
                    "joint0": -2000.0,
                    "joint1": -2000.0,
                    "joint2": 2000.0,
                    "joint3": 2000.0,
                },
            ),
            actuators={
                "dummy": ImplicitActuatorCfg(
                    joint_names_expr=[".*"],
                    stiffness=0.0,
                    damping=0.0,
                ),
            },
        )

        self.body_path = self.ISAAC_SIM_CFG.prim_path + "/body"
        self.define_sensors()

    def define_sensors(self):
        self.sensor_list_ = []

        mono_depth_camera_config = {
            "id": 0,
            "body_path": self.body_path,
            "position": [0.12, 0.0, 0.067],
            "quaternion": [ 0.0, 0.0, 0.0, 1.0],
            "camera_model": "pinhole",
            "resolution": [100, 100],
            "focal_length": 15.0 / 10, # mm to cm
            "types": ['depth_pcl'],
            "publish_labels": False,
            "namespace": self.name_space,
        }

        right_front_camera_config = {
            "id": 1,
            "body_path": self.body_path,
            "position": [0.0735, -0.0735, 0.067],
            "quaternion": [  0.924, 0, 0, -0.383],
            "camera_model": "fisheye",
            "resolution": [1280, 720],
            "fov": 220.0,
            "types": ['rgb', 'depth'],
            "publish_labels": False,
            "namespace": self.name_space,
        }

        right_back_camera_config = {
            "id": 2,
            "body_path": self.body_path,
            "position": [-0.0735, -0.0735, 0.067],
            "quaternion": [ 0.383, 0, 0, -0.924],
            "camera_model": "fisheye",
            "resolution": [1280, 720],
            "fov": 220.0,
            "types": ['rgb', 'depth'],
            "publish_labels": False,
            "namespace": self.name_space,
        }

        left_back_camera_config = {
            "id": 3,
            "body_path": self.body_path,
            "position": [-0.0735, 0.0735, 0.067],
            "quaternion": [ 0.383, 0, 0, 0.924],
            "camera_model": "fisheye",
            "resolution": [1280, 720],
            "fov": 220.0,
            "types": ['rgb', 'depth'],
            "publish_labels": False,
            "namespace": self.name_space,
        }

        left_front_camera_config = {
            "id": 4,
            "body_path": self.body_path,
            "position": [0.0735, 0.0735, 0.067],
            "quaternion": [ 0.924, 0, 0, 0.383],
            "camera_model": "fisheye",
            "resolution": [1280, 720],
            "fov": 220.0,
            "types": ['rgb', 'depth'],
            "publish_labels": False,
            "namespace": self.name_space,
        }

        if self.enable_cameras:
            self.sensor_list_ += [
                ROS2Camera(mono_depth_camera_config),
                ROS2Camera(right_front_camera_config),
                ROS2Camera(right_back_camera_config),
                ROS2Camera(left_back_camera_config),
                ROS2Camera(left_front_camera_config),
            ]