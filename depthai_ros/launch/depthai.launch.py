from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    depthai_yolo_spatial_publisher_node = Node(package='depthai_ros', executable='depthai_yolo_spatial_publisher')
    tf2_object_broadcaster_node = Node(package='depthai_ros', executable='tf2_object_broadcaster')

    ld.add_action(depthai_yolo_spatial_publisher_node)
    ld.add_action(tf2_object_broadcaster_node)

    return ld

if __name__ == '__main__':
    generate_launch_description()