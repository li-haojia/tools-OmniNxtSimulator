import carb
import omni.graph.core as og
from isaacsim.sensors.rtx import LidarRtx

from simulator.tf import ROS2StaticTFPub

# All supported LiDAR configurations
# https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/isaac_sim_sensors_rtx_based_lidar.html
CONFIG_FILES = [
    "Hesai_XT32_SD10",
    "Velodyne_VLS128",
    "Example_Rotary",
    "Example_Solid_State",
]

class ROS2Lidar:
    '''
    config = {
        "id": 0,                                    # sensor id
        "body_path": "",                            # base path for the sensor
        "position": [0.0, 0.0, 0.0],                # position of the lidar in the body frame
        "quaternion": [0.0, 0.0, 0.0, 1.0],         # orientation of the lidar in the body frame
        "config_file": "Hesai_XT32_SD10",            # configuration file for the lidar
        "namespace": "",                            # namespace for the lidar (default is vehicle name in Isaac Sim)
        "tf_frame_id": ""                           # tf frame id for the lidar (default is lidar name in Isaac Sim)
    }
    '''
    lidar = None

    def __init__(self, config: dict):
        self.id = config.get("id", 0)
        self.body_path = config.get("body_path", "")
        self.lidar_prim_path = self.body_path + f"/sensors/lidar_{self.id}"
        self.position = config.get("position", [0.0, 0.0, 0.0])
        self.quaternion = config.get("quaternion", [0.0, 0.0, 0.0, 1.0])
        self.config_file = config.get("config_file", "Hesai_XT32_SD10")
        self.namespace = config.get("namespace", "")
        self.tf_frame_id = config.get("tf_frame_id", f"lidar_{self.id}")
        self.base_topic = f"lidar_{self.id}"

    def initialize(self):
        self.lidar = LidarRtx(
            prim_path=self.lidar_prim_path,
            config_file_name=self.config_file,
            translation=self.position,
            orientation=self.quaternion,
        )
        ROS2StaticTFPub(self.lidar_prim_path, self.body_path.split("/")[-1], self.tf_frame_id, self.position, [self.quaternion[1], self.quaternion[2], self.quaternion[3], self.quaternion[0]])

        graph_path = f"{self.lidar_prim_path}_pub"
        graph_specs = {
            "graph_path": graph_path,
            "evaluator_name": "execution",
        }
        keys = og.Controller.Keys
        graph_config = {
            keys.CREATE_NODES: [
                ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                ("isaac_run_one_simulation_frame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("isaac_create_render_product", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("ros2_rtx_lidar_point_helper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                ("ros2_rtx_lidar_scan_helper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            keys.CONNECT: [
                ("on_playback_tick.outputs:tick", "isaac_run_one_simulation_frame.inputs:execIn"),
                ("isaac_run_one_simulation_frame.outputs:step", "isaac_create_render_product.inputs:execIn"),
                ("isaac_create_render_product.outputs:execOut", "ros2_rtx_lidar_point_helper.inputs:execIn"),
                ("isaac_create_render_product.outputs:execOut", "ros2_rtx_lidar_scan_helper.inputs:execIn"),
                ("isaac_create_render_product.outputs:renderProductPath", "ros2_rtx_lidar_point_helper.inputs:renderProductPath"),
                ("isaac_create_render_product.outputs:renderProductPath", "ros2_rtx_lidar_scan_helper.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("isaac_create_render_product.inputs:cameraPrim", f"{self.lidar_prim_path}"),
                ("ros2_rtx_lidar_point_helper.inputs:frameId", self.tf_frame_id),
                ("ros2_rtx_lidar_scan_helper.inputs:frameId", self.tf_frame_id),
                ("ros2_rtx_lidar_point_helper.inputs:topicName", f"{self.base_topic}/point_cloud"),
                ("ros2_rtx_lidar_scan_helper.inputs:topicName", f"{self.base_topic}/scan"),
                ("ros2_rtx_lidar_point_helper.inputs:type", "point_cloud"),
                ("ros2_rtx_lidar_scan_helper.inputs:type", "laser_scan"),
            ],
        }
        # Create graph
        (graph, _, _, _) = og.Controller.edit(
            graph_specs,
            graph_config
        )

        # Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
        og.Controller.evaluate_sync(graph)