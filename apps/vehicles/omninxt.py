from .quadrotor import Quadrotor

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
        "id": 0,
    }

    ISAAC_SIM_CFG = None

    def __init__(self, id, init_pos = (0.0, 0.0, 0.5)):
        super().__init__(id, self.config)

        try:
            import omni.isaac.lab.sim as sim_utils
            from omni.isaac.lab.actuators import ImplicitActuatorCfg
            from omni.isaac.lab.assets import ArticulationCfg
            from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
            self.ISAAC_SIM_CFG = ArticulationCfg(
                prim_path="{ENV_REGEX_NS}/Robot",
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
        except ImportError:
            ISAAC_SIM_CFG = None
            print("Warning: Isaac SDK not available.")