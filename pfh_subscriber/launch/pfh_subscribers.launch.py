from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(package='pfh_subscriber',
             executable='pfh_sub_1',
             name='pfh_sub_1',
             parameters=[
                 {'cycles': 8},
             ],
             ),
        Node(package='pfh_subscriber',
             executable='pfh_sub_2',
             name='pfh_sub_2',
             parameters=[
                 {'cycles': 8},
             ],
             ),
        Node(package='pfh_subscriber',
             executable='pfh_sub_3',
             name='pfh_sub_3',
             parameters=[
                 {'cycles': 8},
             ],
             ),
        Node(package='pfh_subscriber',
             executable='pfh_sub_4',
             name='pfh_sub_4',
             parameters=[
                 {'cycles': 8},
             ],
             ),
        Node(package='pfh_subscriber',
             executable='pfh_sub_5',
             name='pfh_sub_5',
             parameters=[
                 {'cycles': 8},
             ],
             ),
        Node(package='pfh_subscriber',
             executable='pfh_sub_6',
             name='pfh_sub_6',
             parameters=[
                 {'cycles': 8},
             ],
             ),
        Node(package='pfh_subscriber',
             executable='pfh_sub_7',
             name='pfh_sub_7',
             parameters=[
                 {'cycles': 8},
             ],
             ),
        Node(package='pfh_subscriber',
             executable='pfh_sub_8',
             name='pfh_sub_8',
             parameters=[
                 {'cycles': 8},
             ],
             ),
    ])
