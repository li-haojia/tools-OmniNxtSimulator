import carb

from omni.isaac.core.utils import stage
import omni.graph.core as og
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.sensor import Camera

FishEyeConfig = {
    "id": 0,
    "sensor_base_path": "",
    "position": [0.0, 0.0, 0.0],
    "quaternion": [1.0, 0.0, 0.0, 0.0],
    "camera_model": "fisheye",
    "resolution": [1280, 720],
    "focal_length": 400.0,
    "fov": 220.0,
    "types": ['rgb', 'depth'],
    "publish_labels": False,
    "namespace": "",
    "tf_frame_id": "map",
}

class ROS2Camera:
    '''
    config = {
        "id": 0,                                    # sensor id
        "sensor_base_path": "",                     # base path for the sensor
        "position": [0.0, 0.0, 0.0],                # position of the camera in the body frame
        "quaternion": [0.0, 0.0, 0.0, 1.0],         # orientation of the camera in the body frame
        "camera_model": "fisheye",                   # camera model (fisheye, pinhole)
        "resolution": [1280, 720],                  # output video stream resolution in pixels [width, height]
        "focal_length": 400.0,                       # focal length of the camera in pixels
        "fov": 220.0,                                # field of view of the camera in degrees
        "types": ['rgb', 'depth'],           # rgb, depth, depth_pcl, instance_segmentation, semantic_segmentation, bbox_2d_tight, bbox_2d_loose, bbox_3d
        "publish_labels": False,                    # publish labels for segmentation and bounding boxes
        "namespace": ""                            # namespace for the camera (default is vehicle name in Isaac Sim)
        "tf_frame_id": ""}                         # tf frame id for the camera (default is camera name in Isaac Sim)
    '''
    camera = None
    def __init__(self, config: dict):
        self.id = config.get("id", 0)
        self.camera_prim_path = config.get("sensor_base_path", "") + f"/camera_{self.id}"
        self.position = config.get("position", [0.0, 0.0, 0.0])
        self.quaternion = config.get("quaternion", [1.0, 0.0, 0.0, 0.0])
        self.camera_model = config.get("camera_model", "pinhole")
        self.resolution = config.get("resolution", [1280, 720])
        self.focal_length = config.get("focal_length", 400.0)
        self.fov = config.get("fov", 90.0)
        self.types = config.get("types", ['rgb', 'depth'])
        self.publish_labels = config.get("publish_labels", False)
        self.namespace = config.get("namespace", "")
        self.tf_frame_id = config.get("tf_frame_id", "")
        self.base_topic = f"camera_{self.id}"

        self.check_params()


    def check_params(self):
        valid_camera_types = ['rgb', 'depth', 'depth_pcl', 'instance_segmentation', 'semantic_segmentation', 'bbox_2d_tight', 'bbox_2d_loose', 'bbox_3d']
        if not all([camera_type in valid_camera_types for camera_type in self.types]):
            carb.log_error(f"Invalid camera types: {self.types}")
            return

        valid_camera_models = ['fisheye', 'pinhole']
        if self.camera_model not in valid_camera_models:
            carb.log_error(f"Invalid camera model: {self.camera_model}")
            return

    def initialize(self):
        self.camera = Camera(
            prim_path=self.camera_prim_path,
            translation=self.position,
            orientation=self.quaternion,
            resolution=self.resolution,
        )
        if self.camera_model == "fisheye":
            self.camera.set_projection_type("fisheyePolynomial")
            self.camera.set_fisheye_polynomial_properties(
                nominal_width = 1936,
                nominal_height = 1216.0,
                optical_centre_x = 970.94244,
                optical_centre_y = 600.37482,
                max_fov=self.fov,
                polynomial = [0.0, 0.0, 0.0, 0.0, 0.0]
            )
        elif self.camera_model == "pinhole":
            self.camera.set_focal_length(self.focal_length)

        self.camera.initialize()

        if not is_prim_path_valid(self.camera_prim_path):
            carb.log_error(f"Invalid camera prim path: {self.camera_prim_path}")
            return

        graph_path = f"{self.camera_prim_path}_pub"
        graph_specs = {
            "graph_path": graph_path,
            "evaluator_name": "execution",
        }
        keys = og.Controller.Keys
        graph_config = {
            keys.CREATE_NODES: [
                ("on_tick", "omni.graph.action.OnTick"),
                ("create_viewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                ("get_render_product", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
                ("set_viewport_resolution", "omni.isaac.core_nodes.IsaacSetViewportResolution"),
                ("set_camera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                ("camera_info_helper", "omni.isaac.ros2_bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("on_tick.outputs:tick", "create_viewport.inputs:execIn"),
                ("create_viewport.outputs:execOut", "get_render_product.inputs:execIn"),
                ("create_viewport.outputs:viewport", "get_render_product.inputs:viewport"),
                ("create_viewport.outputs:execOut", "set_viewport_resolution.inputs:execIn"),
                ("create_viewport.outputs:viewport", "set_viewport_resolution.inputs:viewport"),
                ("set_viewport_resolution.outputs:execOut", "set_camera.inputs:execIn"),
                ("get_render_product.outputs:renderProductPath", "set_camera.inputs:renderProductPath"),
                ("set_camera.outputs:execOut", "camera_info_helper.inputs:execIn"),
                ("get_render_product.outputs:renderProductPath", "camera_info_helper.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("create_viewport.inputs:viewportId", 0),
                ("create_viewport.inputs:name", f"{self.namespace}/camera_{self.id}"),
                ("set_viewport_resolution.inputs:width", self.resolution[0]),
                ("set_viewport_resolution.inputs:height", self.resolution[1]),
                ("camera_info_helper.inputs:nodeNamespace", self.namespace),
                ("camera_info_helper.inputs:frameId", self.tf_frame_id),
                ("camera_info_helper.inputs:topicName", f"{self.base_topic}/camera_info"),
            ],
        }

        for camera_type in self.types:
            camera_helper_name = f"camera_helper_{camera_type}"
            graph_config[keys.CREATE_NODES] += [
                (camera_helper_name, "omni.isaac.ros2_bridge.ROS2CameraHelper")
            ]
            graph_config[keys.CONNECT] += [
                ("set_camera.outputs:execOut", f"{camera_helper_name}.inputs:execIn"),
                ("get_render_product.outputs:renderProductPath", f"{camera_helper_name}.inputs:renderProductPath")
            ]
            graph_config[keys.SET_VALUES] += [
                (f"{camera_helper_name}.inputs:nodeNamespace", self.namespace),
                (f"{camera_helper_name}.inputs:frameId", self.tf_frame_id),
                (f"{camera_helper_name}.inputs:topicName", f"{self.base_topic}/{camera_type}"),
                (f"{camera_helper_name}.inputs:type", camera_type)
            ]

            if self.publish_labels and camera_type in ["semantic_segmentation", "instance_segmentation", "bbox_2d_tight", "bbox_2d_loose", "bbox_3d"]:
                graph_config[keys.SET_VALUES] += [
                    (camera_helper_name + ".inputs:enableSemanticLabels", True),
                    (camera_helper_name + ".inputs:semanticLabelsTopicName", f"{self.base_topic}/{camera_type}/labels")
                ]
        # Create graph
        (graph, _, _, _) = og.Controller.edit(
            graph_specs,
            graph_config
        )

        # Connect camera to the graphs
        set_targets(
            prim=stage.get_current_stage().GetPrimAtPath(f"{graph_path}/set_camera"),
            attribute="inputs:cameraPrim",
            target_prim_paths=[self.camera_prim_path]
        )

        # Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
        og.Controller.evaluate_sync(graph)