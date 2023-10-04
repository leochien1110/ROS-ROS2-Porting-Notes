


     ```python
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package="cpp_parameters",
                executable="minimal_param_node",
                name="custom_minimal_param_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"my_parameter": "earth"}
                ]
            )
        ])
    ```


# Reference
---
* [Migrating launch files from ROS 1 to ROS 2](https://docs.ros.org/en/foxy/How-To-Guides/Launch-files-migration-guide.html)