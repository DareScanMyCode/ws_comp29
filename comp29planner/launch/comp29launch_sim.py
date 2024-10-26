from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    uav_num = 4
    Node_list = []
    for uav_id in range(uav_num):
        uav_id = uav_id+1
        # Node_list.append(PushRosNamespace(f'uav_{uav_id}'))
        Node_list.append(Node(
            package='comp29planner',
            executable='comp29main',
            name='uav_'+str(uav_id),
            parameters=[{
                'uav_id': uav_id,
            }],
            remappings=[
                # pubs
                ('/pos_ned_fcu_int', f'uav_{uav_id}'+'/pos_ned_fcu_int'),
                ('/vel_set_frd', f'uav_{uav_id}'+'/vel_set_frd'),
                ('/vel_set_ned', f'uav_{uav_id}'+'/vel_set_ned'),
                ('/arm', f'uav_{uav_id}'+'/arm'),
                ('gimbalrpy_setpoint', f'uav_{uav_id}'+'/gimbalrpy_setpoint'),
                
                # subs
                (f'/uav{uav_id}/ekf2/pose', f'uav_{uav_id}'+'/ekf2/pose'),
                (f'/uav{uav_id}/comm_info', f'uav_{uav_id}'+'/comm_info'),
                ('/local_position_ned', f'uav_{uav_id}'+'/local_position_ned'),
                ('/number_detected', f'uav_{uav_id}'+'/number_detected'),
                ('/uwb_filtered', f'uav_{uav_id}'+'/uwb_filtered'),
                ('/velocity_and_angular', f'uav_{uav_id}'+'/velocity_and_angular'),
                ('/lidar_height', f'uav_{uav_id}'+'/lidar_height'),
                ('/super_mission_state', f'uav_{uav_id}'+'/super_mission_state'),
                ('/guard_pos_est_ned', f'uav_{uav_id}'+'/guard_pos_est_ned'),
                ('/color_offset', f'uav_{uav_id}'+'/color_offset'),
            ],
        ))
    Node_list = []
    Node_list.append(Node(
            package='comp29planner',
            executable='real2sim',
            name='real2sim_node',
            parameters=[{
                'uav_num': uav_num,
            }]
             )
    )
    return LaunchDescription(Node_list)
