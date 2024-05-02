from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pkg_dir = get_package_share_directory('grid_map_pcl')
    param_path = pkg_dir + '/config/parameters.yaml'

    node_params = [
        {'param_path': param_path},
        {'folder_path': 'path/to/pcd/file'},
        {'pcd_filename': 'plane_noisy'},
        {'map_rosbag_topic': 'grid_map'},
        {'output_grid_map': 'elevation_map.bag'},
        {'map_frame': 'map'},
        {'map_layer_name': 'elevation'},
        {'prefix': ''},
        {'set_verbosity_to_debug': False}
    ]

    pcl_loader_node = Node(
        package='grid_map_pcl',
        executable='grid_map_pcl_loader_node',
        name='grid_map_pcl_loader_node',
        output='screen',
        parameters=node_params
    )

    ld = LaunchDescription()

    ld.add_action(pcl_loader_node)

    return ld
