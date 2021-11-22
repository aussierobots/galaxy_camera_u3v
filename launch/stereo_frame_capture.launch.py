"""Launch uv3_image_pub stereo left & right in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
  """Generate launch description with multiple components."""
  params=[{'image_path': '/tmp'}]

  container1 = ComposableNodeContainer(
    name='stereo_frame_capture',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    composable_node_descriptions=[
      ComposableNode(
        package='galaxy_camera_u3v',
        plugin='stereo_capture::StereoFrameCap',
        name='stereo_frame_cap',
        parameters=params
      )
    ]
  )

  return launch.LaunchDescription([container1])
