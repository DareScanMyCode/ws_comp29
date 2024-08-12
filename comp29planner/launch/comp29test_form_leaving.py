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
        name='hw_uwb_node',
        output='screen',
        # parameters=[]
    )
    
    hardware_gport_node = Node(
        package='comp29hardware',
        executable='gport2ros2',
        name='hw_gport_node',
        output='screen',
        # parameters=[]
    )
    
    hardware_uav_node = Node(
        package='comp29hardware',
        executable='uav2ros2',
        name='hw_uav_node',
        output='screen',
        # parameters=[]
    )
    
    # detector节点
    # color_detector_node = Node(
    #     package='comp29detector',
    #     executable='color_detect',
    #     name='color_det_node',
    #     output='screen',
    #     # parameters=[]
    # )
    
    number_detector_node = Node(
        package='comp29detector',
        executable='number_detect',
        name='number_det_node',
        output='screen',
        # parameters=[]
    )
    
    # main节点
    main_node = Node(
        package='comp29planner',
        executable='comp29main',
        name='main_node',
        output='screen',
        # parameters=[]
    )
    
    # 通讯节点
    fsm_node = Node(
        package='comp29communicator',
        executable='rcl_fsm_main',
        name='rcl_fsm_main_node',
        output='screen',
    )
    
    return LaunchDescription([
        param1,
        param2,
        hardware_uwb_node,
        hardware_gport_node,
        hardware_uav_node,
        # color_detector_node,
        number_detector_node,
        main_node,
        fsm_node,
        # number_detector_node,
        # node2,
    ])
