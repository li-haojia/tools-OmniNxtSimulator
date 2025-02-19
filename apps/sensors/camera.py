import carb
import omni.graph.core as og
from isaacsim.sensors.camera import Camera

from simulator.tf import ROS2StaticTFPub

FishEyeConfig = {
    "id": 0,
    "body_path": "",
    "position": [0.0, 0.0, 0.0],
    "quaternion": [1.0, 0.0, 0.0, 0.0],
    "camera_model": "fisheye",
    "resolution": [1280, 720],
    "focal_length": 400.0,
    "fov": 220.0,
    "types": ['rgb', 'depth'],
    "publish_labels": False,
    "namespace": "",
    "tf_frame_id": "",
}

class ROS2Camera:
    '''
    config = {
        "id": 0,                                    # sensor id
        "body_path": "",                     # base path for the sensor
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
        self.body_path = config.get("body_path", "")
        self.camera_prim_path = self.body_path + f"/sensors/camera_{self.id}"
        self.position = config.get("position", [0.0, 0.0, 0.0])
        self.quaternion = config.get("quaternion", [1.0, 0.0, 0.0, 0.0])
        self.camera_model = config.get("camera_model", "pinhole")
        self.resolution = config.get("resolution", [1280, 720])
        self.focal_length = config.get("focal_length", 50.0)
        self.fov = config.get("fov", 220.0)
        self.types = config.get("types", ['rgb', 'depth'])
        self.publish_labels = config.get("publish_labels", False)
        self.namespace = config.get("namespace", "")
        self.tf_frame_id = config.get("tf_frame_id", f"camera_{self.id}")
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
            self.camera.set_projection_type("pinhole")
            self.camera.set_focal_length(self.focal_length)
            if self.camera.get_focal_length() != self.focal_length:
                carb.log_error(f"Failed to set focal length to {self.focal_length}")

        self.camera.initialize()
        ROS2StaticTFPub(self.camera_prim_path, self.body_path.split("/")[-1], self.tf_frame_id, self.position, [self.quaternion[1], self.quaternion[2], self.quaternion[3], self.quaternion[0]])

        graph_path = f"{self.camera_prim_path}_pub"
        graph_specs = {
            "graph_path": graph_path,
            "evaluator_name": "execution",
        }
        keys = og.Controller.Keys
        graph_config = {
            keys.CREATE_NODES: [
                ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                ("isaac_create_render_product", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("camera_info_helper", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("on_playback_tick.outputs:tick", "isaac_create_render_product.inputs:execIn"),
                ("isaac_create_render_product.outputs:execOut", "camera_info_helper.inputs:execIn"),
                ("isaac_create_render_product.outputs:renderProductPath", "camera_info_helper.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                # isaac_create_render_product
                ("isaac_create_render_product.inputs:cameraPrim", f"{self.camera_prim_path}"),
                ("isaac_create_render_product.inputs:enabled", True),
                ("isaac_create_render_product.inputs:width", self.resolution[0]),
                ("isaac_create_render_product.inputs:height", self.resolution[1]),
                # camera_info_helper
                ("camera_info_helper.inputs:nodeNamespace", self.namespace),
                ("camera_info_helper.inputs:frameId", self.tf_frame_id),
                ("camera_info_helper.inputs:topicName", f"{self.base_topic}/camera_info"),
            ],
        }

        for camera_type in self.types:
            camera_helper_name = f"camera_helper_{camera_type}"
            graph_config[keys.CREATE_NODES] += [
                (camera_helper_name, "isaacsim.ros2.bridge.ROS2CameraHelper")
            ]
            graph_config[keys.CONNECT] += [
                ("isaac_create_render_product.outputs:execOut", f"{camera_helper_name}.inputs:execIn"),
                ("isaac_create_render_product.outputs:renderProductPath", f"{camera_helper_name}.inputs:renderProductPath")
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

        # Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
        og.Controller.evaluate_sync(graph)