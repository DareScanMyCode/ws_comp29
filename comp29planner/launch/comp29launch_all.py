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
    


    comp29_detector_node=Node(
        package='comp29detector',
        executable='number_detect',
        name='number_detector',
        output='screen',
    )
    
    comp29_communicate_node=Node(
        package='comp29communicator',
        executable='rcl_fsm_main',
        name='Communicator_FSM',
        output='screen',        
    )

    comp29_localization_node=Node(
        package='comp29localization',
        executable='ekf2_main',
        name='ekf2_main_w_uwb',
        output='screen',
    )      
    
    comp29_rend_node=Node(
        package='comp29planner',
        executable='test_rend',
        name='test_rend_node',
        output='screen',
    )      
    
    comp29_planner_node=Node(
        package='comp29planner',
        executable='comp29main',
        name='comp29_main_node',
        output='screen',
        parameters=[
            {'use_ekf_pos': False,}
        ]
    )
      
    return LaunchDescription([
        param1,
        param2,
        hardware_uwb_node,
        hardware_gport_node,
        hardware_uav_node,
        comp29_detector_node,
        comp29_rend_node,
        comp29_localization_node,
        comp29_communicate_node,
        comp29_planner_node,
        # node2,
    ])
