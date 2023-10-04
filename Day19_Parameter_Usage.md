> 開始之前先來複習一下`ROS`和`ROS2`Parameter的機制，會讓在Migrating的人更好理解，不然當初我寫的時候都一頭霧水。首先`ROS2`已經沒有Master Server，所以Parameter Server也一起消失了。這樣的好處是可以隨時啟動node不用先跑`roscore`。
> 
> 但是這也意味著Parameter必須依賴Node，必須在Node內被宣告後才能在使用。這也是為什麼`ROS`中我們可以直接用`get_param`來取得參數，但是`ROS2`必須要先`declare_parameter`然後再`get_parameter`才能取得參數。接下來我們來看看`ROS2`的Parameter怎麼用。

# C++
---
1. 首先我們建立一個新的c++ package，並且加入`rclcpp`的dependency:
    ```bash
    ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp
    ```
2. 在`ros2_ws/src/cpp_parameters/src`底下新增`cpp_parameters_node.cpp`:
    ```cpp
    #include <chrono>
    #include <functional>
    #include <string>

    #include <rclcpp/rclcpp.hpp>

    using namespace std::chrono_literals;

    class MinimalParam : public rclcpp::Node
    {
    public:
    MinimalParam()
    : Node("minimal_param_node")
    {
        this->declare_parameter("my_parameter", "world");

        timer_ = this->create_wall_timer(
        1000ms, std::bind(&MinimalParam::timer_callback, this));
    }

    void timer_callback()
    {
        std::string my_param = this->get_parameter("my_parameter").as_string();

        RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

        std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
        this->set_parameters(all_new_parameters);
    }

    private:
    rclcpp::TimerBase::SharedPtr timer_;
    };

    int main(int argc, char ** argv)
    {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalParam>());
    rclcpp::shutdown();
    return 0;
    }
    ```
    
    解析：
    ```cpp
    class MinimalParam : public rclcpp::Node
    {
    public:
        MinimalParam()
        : Node("minimal_param_node")
        {
            this->declare_parameter("my_parameter", "world");

            timer_ = this->create_wall_timer(
            1000ms, std::bind(&MinimalParam::timer_callback, this));
        }
    ```
    首先來看Constructor。在`ROS2`中，必須要先`declare_parameter`才能使用`get_parameter`，所以先在這裡宣告一個`my_parameter`，並且給他預設值`world`。接著建立一個timer，每秒鐘會呼叫`timer_callback`。

    ```cpp
    void timer_callback()
    {
    std::string my_param = this->get_parameter("my_parameter").as_string();

    RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    this->set_parameters(all_new_parameters);
    }
    ```
    在`timer_callback`中，我們先用`get_parameter`取得`my_parameter`的值，並且用`RCLCPP_INFO`印出來。接著建立一個`std::vector`，並且用`set_parameters`來設定`my_parameter`的值返回為`world`。這樣使用者在外部更改`my_parameter`的值，`timer_callback`就會將他改回`world`。
    
    這邊用的`set_parameters`可以一次用來設定多個參數。當然也可以用`set_parameter`來設定單一參數。`set_parameters`的用法如下：
    ```cpp
    this->set_parameter(rclcpp::Parameter("my_parameter", "world"));
    ```

    其餘的用法都和前面的範例一樣，smart pointer宣告node，`rclcpp::init`和`rclcpp::shutdown`來啟動和關閉node，就不再贅述。

3. 在`CMakeLists.txt`的`find_package(rclcpp REQUIRED)`下方加入：
    ```cmake
    add_executable(minimal_param_node src/cpp_parameters_node.cpp)
    ament_target_dependencies(minimal_param_node rclcpp)

    install(TARGETS
        minimal_param_node
    DESTINATION lib/${PROJECT_NAME}
    )
    ```
    可以看到我們並沒有增新的dependency，因為parameter是`rclcpp`的一部分。
4. Build:
    ```bash
    cd ~/ros2_ws
    # install dependencies (if any)
    rosdep install -i --from-path src --rosdistro foxy -y

    colcon build --packages-select cpp_parameters
    source install/setup.bash

    ```
5. 執行:
    ```bash
    ros2 run cpp_parameters minimal_param_node
    ```
    就可以看到:
    ```bash
    [INFO] [1630549430.000000000] [minimal_param_node]: Hello world!
    ```
6. 開一個新的terminal查看parameter:
    ```bash
    ros2 param list
    ```
    可以看到:
    ```bash
    my_parameter
    ```
7. 接著將`world`換成其他字，像是`there`
    ```bash
    ros2 param set /minimal_param_node my_parameter there
    ```
    再回到第一個terminal，可以看到成功更改`world`:
    ```bash
    [INFO] [minimal_param_node]: Hello earth!
    ```
8. 當然也可以在launch file中更改parameter:
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
    ros launch 的部分後面會再詳細介紹，這邊只要知道`Node`中的`parameters=[]`可以用來設定參數就好。
    1. 回到`CMakeLists.txt`中安裝`launch`:
        ```cmake
        install(
            DIRECTORY launch
            DESTINATION share/${PROJECT_NAME}
        )
        ```
    2. Build with launch
        ```bash
        colcon build --symlink-install --packages-select cpp_parameters
        source install/setup.bash
        ```
        這邊用`--symlink-install`是因為我們要修改`src`底下得launch file，這樣可以讓`install`內的launch file也跟著改變。
    3. 執行
        ```bash
        ros2 launch cpp_parameters cpp_parameters_launch.py
        ```
        可以看到更改成功:
        ```bash
        [INFO] [minimal_param_node]: Hello earth!
        ```

# Python
---
Python的流程和C++基本上大同小異，這邊講解的部分只會針對不同的地方做說明。
1. 在`ros2_ws/src`底下新增一個python package:
    ```bash
    ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy
    ```
2. 在`ros2_ws/src/python_parameters/python_parameters`底下新增`python_parameters_node.py`:
    ```python
    import rclpy
    import rclpy.node

    class MinimalParam(rclpy.node.Node):
        def __init__(self):
            super().__init__('minimal_param_node')

            self.declare_parameter('my_parameter', 'world')

            self.timer = self.create_timer(1, self.timer_callback)

        def timer_callback(self):
            my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

            self.get_logger().info('Hello %s!' % my_param)

            my_new_param = rclpy.parameter.Parameter(
                'my_parameter',
                rclpy.Parameter.Type.STRING,
                'world'
            )
            all_new_parameters = [my_new_param]
            self.set_parameters(all_new_parameters)

    def main():
        rclpy.init()
        node = MinimalParam()
        rclpy.spin(node)

    if __name__ == '__main__':
        main()
    ```


# Reference
---
* [Using parameters in C++](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)
* [Using parameters in Python](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)