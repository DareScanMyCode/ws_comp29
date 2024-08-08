from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明参数
    param1 = DeclareLaunchArgument('param1', default_value='default_value1', description='Parameter 1')
    param2 = DeclareLaunchArgument('param2', default_value='default_value2', description='Parameter 2')

    # hardware节点
    hardware_uwb_node = Node(
        package='comp29hardware',
        executable='uwb2ros2',
        name='hardware_uwb_node',
        output='screen',
        # parameters=[]
    )
    
    hardware_gport_node = Node(
        package='comp29hardware',
        executable='gport2ros2',
        name='hardware_gport_node',
        output='screen',
        # parameters=[]
    )
    
    hardware_uav_node = Node(
        package='comp29hardware',
        executable='uav2ros2',
        name='hardware_uav_node',
        output='screen',
        # parameters=[]
    )
    
    return LaunchDescription([
        param1,
        param2,
        hardware_uwb_node,
        hardware_gport_node,
        hardware_uav_node
        # node2,
    ])
