"""Launch uv3_image_pub stereo left & right in a component container."""

""" DONT USE """
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
  """Generate launch description with multiple components."""
  left_params=[{'topic': 'stereo/left'},
               {'device_sn': 'FDS20110009'},
               {'pixel_format': 0x110000D},  # RG10
              #  {'pixel_format': 0x1080009},  # RG8
               {'image_encoding': 'BAYER_RGGB16'}]
              #  {'image_encoding': 'BAYER_RGGB8'}]
              #  {'image_encoding': 'RGB8'}]
  right_params=[{'topic': 'stereo/right'},
                {'pixel_format': 0x110000D},  # RG10
              #  {'pixel_format': 0x1080009},  # RG8
                {'device_sn': 'FDS20110008'},
                {'image_encoding': 'BAYER_RGGB16'}]
                # {'image_encoding': 'BAYER_RGGB8'}]
              #  {'image_encoding': 'RGB8'}]

  left = ComposableNodeContainer(
    name='left_image_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
      ComposableNode(
        package='galaxy_camera_u3v',
        plugin='camera::U3vImagePub',
        name='left_image_pub',
        parameters=left_params
      ),
    ]
  )
  right = ComposableNodeContainer(
    name='right_image_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
      ComposableNode(
        package='galaxy_camera_u3v',
        plugin='camera::U3vImagePub',
        name='right_image_pub',
        parameters=right_params
      ),
    ]
  )
  trigger = ComposableNodeContainer(
    name='trigger_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
      ComposableNode(
        package='galaxy_camera_u3v',
        plugin='camera::CaptureTrigger',
        name='capture_trigger',
        parameters=[{'trigger_hz': 30}]
      ),
    ]
  )

  return launch.LaunchDescription([trigger, left, right])
