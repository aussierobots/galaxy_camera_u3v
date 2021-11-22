"""Launch uv3_image_pub stereo left & right in a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
  """Generate launch description with multiple components."""
  params=[{'image_path': '/tmp'},
          {'size': '8x12'},
          {'calibration': 'ChArUco'}]

  container1 = ComposableNodeContainer(
    name='calib_stereo_image_cap',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    composable_node_descriptions=[
      ComposableNode(
        package='galaxy_camera_u3v',
        plugin='camera::CalibStereoImageCap',
        name='calib_stereo_image_cap',
        parameters=params
      )
    ]
  )

  return launch.LaunchDescription([container1])
