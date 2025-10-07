from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
    	Node(
    		package='turtlesim',
    		executable='turtlesim_node',
	),
        # Fill in the strings here so that the correct
        # nodes are launched
        Node(
        	package='lab2',
            	executable='draw_square',
            	namespace='turtle1',
        ),
    ])
