from pxr import UsdGeom, Gf
import omni.usd
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
import omni.syntheticdata
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
import numpy as np
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.nodes.scripts.utils import set_target_prims
from isaacsim.ros2.bridge import read_camera_info

# -------------------------------
# 1. Open USD stage
# -------------------------------
stage_path = "/home/ghost/Downloads/fireloop_assignment/isaac-usd/isaac_sim_scene/scene.usda"
omni.usd.get_context().open_stage(stage_path)

# -------------------------------
# 2. Camera setup
# -------------------------------
q = rot_utils.euler_angles_to_quats(np.array([0, -90, 90]), degrees=True)
camera = Camera(
    prim_path="/World/camera_rgb",
    position=np.array([0.2, 0.0, 2.8]),
    frequency=20,
    resolution=(640, 480),
    orientation=np.array([q[1], q[2], q[3], q[0]]),
)
camera.initialize()

camera_prim = camera.prim_path
camera_frame_id = camera_prim.split("/")[-1]
ros_camera_graph_path = "/CameraTFActionGraph"

# -------------------------------
# 3. OmniGraph TF setup (safe)
# -------------------------------
if not is_prim_path_valid(ros_camera_graph_path):
    og.Controller.edit(
        {
            "graph_path": ros_camera_graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("IsaacClock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("RosPublisher", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),
                ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),
            ],
        },
    )

publish_tf_node = ros_camera_graph_path + "/PublishTF_" + camera_frame_id
publish_raw_node = ros_camera_graph_path + "/PublishRawTF_" + camera_frame_id + "_world"

if not is_prim_path_valid(publish_tf_node) and not is_prim_path_valid(publish_raw_node):
    og.Controller.edit(
        ros_camera_graph_path,
        {
            og.Controller.Keys.CREATE_NODES: [
                ("PublishTF_" + camera_frame_id, "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("PublishRawTF_" + camera_frame_id + "_world", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishTF_" + camera_frame_id + ".inputs:topicName", "/tf"),
                ("PublishRawTF_" + camera_frame_id + "_world.inputs:topicName", "/tf"),
                ("PublishRawTF_" + camera_frame_id + "_world.inputs:parentFrameId", camera_frame_id),
                ("PublishRawTF_" + camera_frame_id + "_world.inputs:childFrameId", camera_frame_id + "_world"),
                ("PublishRawTF_" + camera_frame_id + "_world.inputs:translation", [0.13473, 0.0, 1.0]),
            ],
            og.Controller.Keys.CONNECT: [
                (ros_camera_graph_path + "/OnTick.outputs:tick", "PublishTF_" + camera_frame_id + ".inputs:execIn"),
                (ros_camera_graph_path + "/OnTick.outputs:tick", "PublishRawTF_" + camera_frame_id + "_world.inputs:execIn"),
                (ros_camera_graph_path + "/IsaacClock.outputs:simulationTime", "PublishTF_" + camera_frame_id + ".inputs:timeStamp"),
                (ros_camera_graph_path + "/IsaacClock.outputs:simulationTime", "PublishRawTF_" + camera_frame_id + "_world.inputs:timeStamp"),
            ],
        },
    )

set_target_prims(
    primPath=publish_tf_node,
    inputName="inputs:targetPrims",
    targetPrimPaths=[camera_prim],
)

# -------------------------------
# 4. ROS2 Publishers
# -------------------------------
render_product = camera._render_product_path

# RGB
rv_rgb = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
writer_rgb = rep.writers.get(rv_rgb + "ROS2PublishImage")
writer_rgb.initialize(frameId="camera_rgb", topicName="camera/color/image_raw", queueSize=1, nodeNamespace="")
writer_rgb.attach([render_product])

# Depth
rv_depth = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)
writer_depth = rep.writers.get(rv_depth + "ROS2PublishImage")
writer_depth.initialize(frameId="camera_rgb", topicName="camera/depth/image_raw", queueSize=1, nodeNamespace="")
writer_depth.attach([render_product])

# Camera info
writer_info = rep.writers.get("ROS2PublishCameraInfo")
camera_info, _ = read_camera_info(render_product_path=render_product)
writer_info.initialize(
    frameId="camera_rgb",
    topicName="camera/color/camera_info",
    queueSize=1,
    nodeNamespace="",
    width=camera_info.width,
    height=camera_info.height,
    projectionType=camera_info.distortion_model,
    k=camera_info.k.reshape([1, 9]),
    r=camera_info.r.reshape([1, 9]),
    p=camera_info.p.reshape([1, 12]),
)
writer_info.attach([render_product])

# Pointcloud
rv_pc = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.DistanceToImagePlane.name)
writer_pc = rep.writers.get(rv_pc + "ROS2PublishPointCloud")
writer_pc.initialize(frameId="camera_rgb_world", topicName="camera/pointcloud", queueSize=1, nodeNamespace="")
writer_pc.attach([render_product])

# -------------------------------
# 5. Save stage
# -------------------------------
stage = omni.usd.get_context().get_stage()
stage.Save()
print("✅ Scene saved with camera + ROS2 publishers safely!")


