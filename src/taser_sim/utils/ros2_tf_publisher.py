import omni.graph.core as og

keys = og.Controller.Keys


def set_up_omni_graph():
    og.Controller.edit(
        {"graph_path": "/Graph/ROS_TF", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ],
            keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", False),
            ],
        },
    )


def add_tf_publisher(robot_name: str, target_prim: str, tf_publisher_topic: str):
    graph_handle: og.Graph = og.get_graph_by_path("/Graph/ROS_TF")

    all_nodes = graph_handle.get_nodes()
    for node in all_nodes:
        node_path = node.get_prim_path()
        node_type = node.get_type_name()
        if (
            node_type == "omni.graph.action.OnPlaybackTick"
            or node_type == "omni.graph.action.OnTick"
        ):
            tick_node = node_path
        elif node_type == "isaacsim.ros2.bridge.ROS2Context":
            context_node = node_path
        elif node_type == "isaacsim.core.nodes.IsaacReadSimulationTime":
            sim_time_node = node_path

    og.Controller.edit(
        graph_handle,
        {
            keys.CREATE_NODES: [
                (robot_name, "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.SET_VALUES: [
                (robot_name + ".inputs:parentPrim", "/World"),
                (robot_name + ".inputs:targetPrims", target_prim),
                (robot_name + ".inputs:topicName", tf_publisher_topic),
            ],
            keys.CONNECT: [
                (tick_node + ".outputs:tick", robot_name + ".inputs:execIn"),
                (
                    sim_time_node + ".outputs:simulationTime",
                    robot_name + ".inputs:timeStamp",
                ),
                (context_node + ".outputs:context", robot_name + ".inputs:context"),
            ],
        },
    )
