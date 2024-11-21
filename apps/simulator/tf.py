import carb
import omni.graph.core as og
import usdrt.Sdf

class ROS2TFPub:
    # TODO: BUG: This class is not working as expected. The Odometry is not being updated in IsaacComputeOdometry node.
    def __init__(self, target_prim):
        self.target_prim = target_prim

        graph_path = f"{target_prim}_tf_pub"
        graph_specs = {
            "graph_path": graph_path,
            "evaluator_name": "execution",
        }
        keys = og.Controller.Keys

        # graph_config = {
        #     keys.CREATE_NODES: [
        #         ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
        #         ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
        #         ("isaac_compute_odometry_node", "omni.isaac.core_nodes.IsaacComputeOdometry"),
        #         ("ros2_publish_raw_transform_tree", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
        #     ],
        #     keys.CONNECT: [
        #         ("on_playback_tick.outputs:tick", "isaac_compute_odometry_node.inputs:execIn"),
        #         ("isaac_read_simulation_time.outputs:simulationTime", "ros2_publish_raw_transform_tree.inputs:timeStamp"),
        #         ("isaac_compute_odometry_node.outputs:execOut", "ros2_publish_raw_transform_tree.inputs:execIn"),
        #         ("isaac_compute_odometry_node.outputs:orientation", "ros2_publish_raw_transform_tree.inputs:rotation"),
        #         ("isaac_compute_odometry_node.outputs:position", "ros2_publish_raw_transform_tree.inputs:translation"),
        #     ],
        #     keys.SET_VALUES: [
        #         ("isaac_compute_odometry_node.inputs:chassisPrim", [usdrt.Sdf.Path(self.target_prim)]),
        #         ("ros2_publish_raw_transform_tree.inputs:childFrameId", self.target_prim.split("/")[-1]),
        #         ("ros2_publish_raw_transform_tree.inputs:parentFrameId", "world"),
        #     ],
        # }

        graph_config = {
            keys.CREATE_NODES: [
                ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("ros2_publish_transform_tree", "omni.isaac.ros2_bridge.ROS2PublishTransformTree")
            ],
            keys.CONNECT: [
                ("on_playback_tick.outputs:tick", "ros2_publish_transform_tree.inputs:execIn"),
                ("isaac_read_simulation_time.outputs:simulationTime", "ros2_publish_transform_tree.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("ros2_publish_transform_tree.inputs:targetPrims", [usdrt.Sdf.Path(self.target_prim)]),
            ],
        }

        # Create graph
        (graph, _, _, _) = og.Controller.edit(
            graph_specs,
            graph_config
        )

        # Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
        og.Controller.evaluate_sync(graph)

class ROS2StaticTFPub:
    def __init__(self, target_prim, parent_frame_id, child_frame_id, translation, rotation):
        self.target_prim = target_prim

        graph_path = f"{target_prim}_static_tf_pub"
        graph_specs = {
            "graph_path": graph_path,
            "evaluator_name": "execution",
        }
        keys = og.Controller.Keys
        graph_config = {
            keys.CREATE_NODES: [
                ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                ("isaac_read_simulation_time", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("ros2_publish_raw_transform_tree", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
            ],
            keys.CONNECT: [
                ("on_playback_tick.outputs:tick", "ros2_publish_raw_transform_tree.inputs:execIn"),
                ("isaac_read_simulation_time.outputs:simulationTime", "ros2_publish_raw_transform_tree.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("ros2_publish_raw_transform_tree.inputs:childFrameId", child_frame_id),
                ("ros2_publish_raw_transform_tree.inputs:parentFrameId", parent_frame_id),
                ("ros2_publish_raw_transform_tree.inputs:translation", translation),
                ("ros2_publish_raw_transform_tree.inputs:rotation", rotation),
                ("ros2_publish_raw_transform_tree.inputs:topicName", "/tf"),
            ],
        }

        # Create graph
        (graph, _, _, _) = og.Controller.edit(
            graph_specs,
            graph_config
        )

        # Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
        og.Controller.evaluate_sync(graph)