今天的範例會延用昨天的創的package`learning_tf2_cpp`，然後把`turtlesim`和`tf2`的功能結合起來。雖然看起來很多，但其實都長得超像XD 就是幾個function的變形而已，不要害怕

# Broadcaster
---
Broadcaster顧名思義就是廣播的意思，將frame之間的關係發布出去。下面的範例是將客製化訊息`turtlesim::msg::Pose`轉換成`tf2`。
1. 首先下載`turtle_tf2_broadcaster.cpp`
    ```bash
    cd ~/ros2_ws/src/learning_tf2_cpp/src
    wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/turtle_tf2_broadcaster.cpp
    ```
2. 檢視`turtle_tf2_broadcaster.cpp`
    ```cpp
    #include <functional>
    #include <memory>
    #include <sstream>
    #include <string>

    #include "geometry_msgs/msg/transform_stamped.hpp"
    #include "rclcpp/rclcpp.hpp"
    #include "tf2/LinearMath/Quaternion.h"
    #include "tf2_ros/transform_broadcaster.h"
    #include "turtlesim/msg/pose.hpp"

    class FramePublisher : public rclcpp::Node
    {
    public:
    FramePublisher()
    : Node("turtle_tf2_frame_publisher")
    {
        // Declare and acquire `turtlename` parameter
        turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle");

        // Initialize the transform broadcaster
        tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        // callback function on each message
        std::ostringstream stream;
        stream << "/" << turtlename_.c_str() << "/pose";
        std::string topic_name = stream.str();

        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
        topic_name, 10,
        std::bind(&FramePublisher::handle_turtle_pose, this, std::placeholders::_1));
    }

    private:
    void handle_turtle_pose(const std::shared_ptr<turtlesim::msg::Pose> msg)
    {
        geometry_msgs::msg::TransformStamped t;

        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = turtlename_.c_str();

        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg->x;
        t.transform.translation.y = msg->y;
        t.transform.translation.z = 0.0;

        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string turtlename_;
    };

    int main(int argc, char * argv[])
    {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
    }
    ```

    可以看到結構和昨天的`static broadcaster`很像，都有`TransformBroadcaster`和`TransformStamped`，只是`TransformStamped`的內容是從`turtlesim`的`Pose`中取得，然後再廣播出去。

    ```cpp
    turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle");
    ```
    首先定義參數`turtlename`，預設參數為`turtle`，但也可以在執行時指定`turtlename`，例如：`turtle1`或`turtle2`。

    ```cpp
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
    topic_name, 10,
    std::bind(&FramePublisher::handle_turtle_pose, this, _1));
    ```
    這邊訂閱`turtlesim`的`Pose`，並且指定callback function為`handle_turtle_pose`。

    ```cpp
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = turtlename_.c_str();

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = 0.0;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    ```
    可以看到這裡非常眼熟，只是將`TransformStamped`的內容改成`turtlesim`的`Pose`。

3. 撰寫launch file 來簡化操作多個terminal流程
    ```python
    from launch import LaunchDescription
    from launch_ros.actions import Node


    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='turtlesim',
                executable='turtlesim_node',
                name='sim'
            ),
            Node(
                package='learning_tf2_cpp',
                executable='turtle_tf2_broadcaster',
                name='broadcaster1',
                parameters=[
                    {'turtlename': 'turtle1'}
                ]
            ),
        ])
    ```
    第一個Node用來啟動`turtlesim`，第二個Node用來啟動`turtle_tf2_broadcaster`，並且指定`turtlename`參數為`turtle1`。
4. 編輯`package.xml`，新增launch file的dependencies
    ```xml
    <exec_depend>launch</exec_depend>
    <exec_depend>launch_ros</exec_depend>
    ```
5. 編輯`CMakeLists.txt`，新增可執行檔和安裝可執行檔和launch file
    ```cmake
    add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
    ament_target_dependencies(
        turtle_tf2_broadcaster
        geometry_msgs
        rclcpp
        tf2
        tf2_ros
        turtlesim
    )
    ...
    install(TARGETS
        turtle_tf2_broadcaster
        DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
    ```
6. Build
    ```bash
    cd ~/ros2_ws
    rosdep install -i --from-path src --rosdistro foxy -y
    colcon build --packages-select learning_tf2_cpp
    source ~/ros2_ws/install/setup.bash
    ```
7. 啟動`turtlesim`和`turtle_tf2_broadcaster`
    ```bash
    ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py
    ```
8. 開啟第二個terminal
    ```bash
    ros2 run tf2_ros tf2_echo world turtle1
    ```
    可以看到`turtle1`在`world`的frame座標
    ```bash
    At time 1625137663.912474878
   - Translation: [5.276, 7.930, 0.000]
   - Rotation: in Quaternion [0.000, 0.000, 0.934, -0.357]
   At time 1625137664.950813527
   - Translation: [3.750, 6.563, 0.000]
   - Rotation: in Quaternion [0.000, 0.000, 0.934, -0.357]
   At time 1625137665.906280726
   - Translation: [2.320, 5.282, 0.000]
   - Rotation: in Quaternion [0.000, 0.000, 0.934, -0.357]
   At time 1625137666.850775673
   - Translation: [2.153, 5.133, 0.000]
   - Rotation: in Quaternion [0.000, 0.000, -0.365, 0.931]
   ```


# Listener
---
這個範例跟上面反過來，將訊息從`tf2`中取出並發布Topic出去。這個範例還偷渡了前面學到的Service，用來產生新的烏龜。
1. 下載`turtle_tf2_listener.cpp`
    ```bash
    cd ~/ros2_ws/src/learning_tf2_cpp/src
    wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/turtle_tf2_listener.cpp
    ```
2. 檢視`turtle_tf2_listener.cpp`
    ```cpp
    #include <chrono>
    #include <functional>
    #include <memory>
    #include <string>

    #include "geometry_msgs/msg/transform_stamped.hpp"
    #include "geometry_msgs/msg/twist.hpp"
    #include "rclcpp/rclcpp.hpp"
    #include "tf2/exceptions.h"
    #include "tf2_ros/transform_listener.h"
    #include "tf2_ros/buffer.h"
    #include "turtlesim/srv/spawn.hpp"

    using namespace std::chrono_literals;

    class FrameListener : public rclcpp::Node
    {
    public:
    FrameListener()
    : Node("turtle_tf2_frame_listener"),
        turtle_spawning_service_ready_(false),
        turtle_spawned_(false)
    {
        // Declare and acquire `target_frame` parameter
        target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle1");

        tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create a client to spawn a turtle
        spawner_ =
        this->create_client<turtlesim::srv::Spawn>("spawn");

        // Create turtle2 velocity publisher
        publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 1);

        // Call on_timer function every second
        timer_ = this->create_wall_timer(
        1s, std::bind(&FrameListener::on_timer, this));
    }

    private:
    void on_timer()
    {
        // Store frame names in variables that will be used to
        // compute transformations
        std::string fromFrameRel = target_frame_.c_str();
        std::string toFrameRel = "turtle2";

        if (turtle_spawning_service_ready_) {
        if (turtle_spawned_) {
            geometry_msgs::msg::TransformStamped t;

            // Look up for the transformation between target_frame and turtle2 frames
            // and send velocity commands for turtle2 to reach target_frame
            try {
            t = tf_buffer_->lookupTransform(
                toFrameRel, fromFrameRel,
                tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            return;
            }

            geometry_msgs::msg::Twist msg;

            static const double scaleRotationRate = 1.0;
            msg.angular.z = scaleRotationRate * atan2(
            t.transform.translation.y,
            t.transform.translation.x);

            static const double scaleForwardSpeed = 0.5;
            msg.linear.x = scaleForwardSpeed * sqrt(
            pow(t.transform.translation.x, 2) +
            pow(t.transform.translation.y, 2));

            publisher_->publish(msg);
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully spawned");
            turtle_spawned_ = true;
        }
        } else {
        // Check if the service is ready
        if (spawner_->service_is_ready()) {
            // Initialize request with turtle name and coordinates
            // Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
            auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
            request->x = 4.0;
            request->y = 2.0;
            request->theta = 0.0;
            request->name = "turtle2";

            // Call request
            using ServiceResponseFuture =
            rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
            auto response_received_callback = [this](ServiceResponseFuture future) {
                auto result = future.get();
                if (strcmp(result->name.c_str(), "turtle2") == 0) {
                turtle_spawning_service_ready_ = true;
                } else {
                RCLCPP_ERROR(this->get_logger(), "Service callback result mismatch");
                }
            };
            auto result = spawner_->async_send_request(request, response_received_callback);
        } else {
            RCLCPP_INFO(this->get_logger(), "Service is not ready");
        }
        }
    }

    // Boolean values to store the information
    // if the service for spawning turtle is available
    bool turtle_spawning_service_ready_;
    // if the turtle was successfully spawned
    bool turtle_spawned_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_;
    };

    int main(int argc, char * argv[])
    {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameListener>());
    rclcpp::shutdown();
    return 0;
    }
    ```
    這裡新增`transform_listener.h`用來接收`tf2`的訊息
    ```cpp
    tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    ```
    創建`TransformListener`，接著就會機受來自`tf2`的所有座標轉換訊息。

    ```cpp
    t = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
    ```
    `lookupTransform`用來query特定的transform，而他有三個參數：
    
    第一個參數是目標座標(Target Frame)，這裡是`turtle2`。
    
    第二個參數是來源座標(Source Frame)，這裡是`turtle1`。
    
    第三個參數是時間，這邊使用`tf2::TimePointZero`，代表最新的時間。
    
3. 編輯`CMakeLists.txt`，新增可執行檔和安裝可執行檔
    ```cmake
    add_executable(turtle_tf2_listener src/turtle_tf2_listener.cpp)
    ament_target_dependencies(
        turtle_tf2_listener
        geometry_msgs
        rclcpp
        tf2
        tf2_ros
        turtlesim
    )
    ...
    install(TARGETS
        turtle_tf2_listener
        DESTINATION lib/${PROJECT_NAME})
    ```
4. 創建launch file
    ```python
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration

    from launch_ros.actions import Node


    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='turtlesim',
                executable='turtlesim_node',
                name='sim'
            ),
            Node(
                package='learning_tf2_cpp',
                executable='turtle_tf2_broadcaster',
                name='broadcaster1',
                parameters=[
                    {'turtlename': 'turtle1'}
                ]
            ),
            DeclareLaunchArgument(
                'target_frame', default_value='turtle1',
                description='Target frame name.'
            ),
            Node(
                package='learning_tf2_cpp',
                executable='turtle_tf2_broadcaster',
                name='broadcaster2',
                parameters=[
                    {'turtlename': 'turtle2'}
                ]
            ),
            Node(
                package='learning_tf2_cpp',
                executable='turtle_tf2_listener',
                name='listener',
                parameters=[
                    {'target_frame': LaunchConfiguration('target_frame')}
                ]
            ),
        ])
    ```
    除了前面的兩個Node，這裡新增了一個Argument用來宣告target_frame。另外還有兩個Node，第三個Node是用來廣播`turtle2`的frame，第四個Node是用來接收`tf2`的訊息，並且發布出去。
5. Build
    ```bash
    cd ~/ros2_ws
    rosdep install -i --from-path src --rosdistro foxy -y
    colcon build --packages-select learning_tf2_cpp
    source ~/ros2_ws/install/setup.bash
    ```
6. 執行launch file
    ```bash
    ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py
    ```
7. 在第二個terminal中執行鍵盤控制
    ```bash
    ros2 run turtlesim turtle_teleop_key
    ```
    就可以用方向鍵操作烏龜，然後可以看到turtle2會跟著turtle1移動。


# Frame
---
這個小節會介紹如何將fixed frame和dynamic frame透過C++ API加入系統。這邊基本上和`broadcaster`相似，不過會多介紹一些功能。

> ⚠️ `tf2 tree` 不允許有loop，也就是frame之間不允許有循環的關係。這點必須要小心。

## Fixed frame broadcaster
1. 下載`fixed_frame_tf2_broadcaster.cpp`:
    ```bash
    cd ~/ros2_ws/src/learning_tf2_cpp/src
    wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/fixed_frame_tf2_broadcaster.cpp
    ```
2. 查看`fixed_frame_tf2_broadcaster.cpp`
    ```cpp
    #include <chrono>
    #include <functional>
    #include <memory>

    #include "geometry_msgs/msg/transform_stamped.hpp"
    #include "rclcpp/rclcpp.hpp"
    #include "tf2_ros/transform_broadcaster.h"

    using namespace std::chrono_literals;

    class FixedFrameBroadcaster : public rclcpp::Node
    {
    public:
    FixedFrameBroadcaster()
    : Node("fixed_frame_tf2_broadcaster")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
        100ms, std::bind(&FixedFrameBroadcaster::broadcast_timer_callback, this));
    }

    private:
    void broadcast_timer_callback()
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "turtle1";
        t.child_frame_id = "carrot1";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 2.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    };

    int main(int argc, char * argv[])
    {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FixedFrameBroadcaster>());
    rclcpp::shutdown();
    return 0;
    }
    ```

    ```cpp
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "turtle1";
    t.child_frame_id = "carrot1";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 2.0;
    t.transform.translation.z = 0.0;
    ```
    這裡從`turtle1`為patern，新增一個新的child frame叫做`carrot1`，並且設定`carrot1`相對於`turtle1`的位置。這樣在`turtle1`在移動時，`carrot1`永遠會在`turtle1`的左邊2m的位置。

3. 編輯`CMakeLists.txt`，加入可執行檔和安裝位置
    ```cmake
    add_executable(fixed_frame_tf2_broadcaster src/fixed_frame_tf2_broadcaster.cpp)
    ament_target_dependencies(
        fixed_frame_tf2_broadcaster
        geometry_msgs
        rclcpp
        tf2_ros
    )
    ...
    install(TARGETS
        fixed_frame_tf2_broadcaster
        DESTINATION lib/${PROJECT_NAME})
    ```
4. 撰寫launch file
    ```python
    import os

    from ament_index_python.packages import get_package_share_directory

    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    from launch_ros.actions import Node


    def generate_launch_description():
        demo_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('learning_tf2_cpp'), 'launch'),
                '/turtle_tf2_demo.launch.py']),
            )

        return LaunchDescription([
            demo_nodes,
            Node(
                package='learning_tf2_cpp',
                executable='fixed_frame_tf2_broadcaster',
                name='fixed_broadcaster',
            ),
        ])
    ```
    這裡使用`IncludeLaunchDescription`來引入前面範例的launch file，然後再新增一個Node來廣播`carrot1`的frame。
    > 其實這個node可以用昨天提到的`static broadcaster`來實現，不過這邊是為了示範`tf2`的功能。
5. Build
    ```bash
    cd ~/ros2_ws
    rosdep install -i --from-path src --rosdistro foxy -y
    colcon build --packages-select learning_tf2_cpp
    source ~/ros2_ws/install/setup.bash
    ```
6. 執行launch file
    ```bash
    ros2 launch learning_tf2_cpp turtle_tf2_fixed_frame_demo.launch.py
    ```
7. 開啟第二個terminal，查看`carrot1`的frame
    ```bash
    ros2 run tf2_tools view_frames.py
    ```
    可以看到`carrot1`的frame是`turtle1`的child frame
8. 我們可以讓`turtle2`改跟著`carrot1`走
    ```bash
    ros2 launch learning_tf2_cpp turtle_tf2_fixed_frame_demo.launch.py target_frame:=carrot1
    ```
    或是更改`turtle_tf2_fixed_frame_demo.launch.py`
    ```python
    def generate_launch_description():
    demo_nodes = IncludeLaunchDescription(
        ...,
        launch_arguments={'target_frame': 'carrot1'}.items(),
        )
    ```
9. 因為官方忘記用`--symlink-install`，所以需要重build XD
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select learning_tf2_cpp
    ```
    這樣烏龜二號就可以跟個蘿蔔一號走了～～～
## Dynamic frame broadcaster
動態frame的廣播器，可以讓frame的位置隨著時間改變。這邊會延用前面的範例，讓`carrot1`的frame隨著時間改變。程式的架構本身不變，但是在位置上我們會加入時間的變數。
1. 下載`dynamic_frame_tf2_broadcaster.cpp`
    ```bash
    cd ~/ros2_ws/src/learning_tf2_cpp/src
    wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/dynamic_frame_tf2_broadcaster.cpp
    ```
2. 檢視`dynamic_frame_tf2_broadcaster`
    ```cpp
    #include <chrono>
    #include <functional>
    #include <memory>

    #include "geometry_msgs/msg/transform_stamped.hpp"
    #include "rclcpp/rclcpp.hpp"
    #include "tf2_ros/transform_broadcaster.h"

    using namespace std::chrono_literals;

    const double PI = 3.141592653589793238463;

    class DynamicFrameBroadcaster : public rclcpp::Node
    {
    public:
    DynamicFrameBroadcaster()
    : Node("dynamic_frame_tf2_broadcaster")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
        100ms, std::bind(&DynamicFrameBroadcaster::broadcast_timer_callback, this));
    }

    private:
    void broadcast_timer_callback()
    {
        rclcpp::Time now = this->get_clock()->now();
        double x = now.seconds() * PI;

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "turtle1";
        t.child_frame_id = "carrot1";
        t.transform.translation.x = 10 * sin(x);
        t.transform.translation.y = 10 * cos(x);
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    };

    int main(int argc, char * argv[])
    {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicFrameBroadcaster>());
    rclcpp::shutdown();
    return 0;
    }
    ```
    可以看到多了時間和三角函數，用來改變`carrot1`的相對位置
    ```cpp
    double x = now.seconds() * PI;
    ...
    t.transform.translation.x = 10 * sin(x);
    t.transform.translation.y = 10 * cos(x);
    ```
3. 編輯`CMakeLists.txt`，加入可執行檔和安裝位置
    ```cmake
    add_executable(dynamic_frame_tf2_broadcaster src/dynamic_frame_tf2_broadcaster.cpp)
    ament_target_dependencies(
        dynamic_frame_tf2_broadcaster
        geometry_msgs
        rclcpp
        tf2_ros
    )
    ...
    install(TARGETS
        dynamic_frame_tf2_broadcaster
        DESTINATION lib/${PROJECT_NAME})
    ```
4. 撰寫launch file
    ```python
    import os

    from ament_index_python.packages import get_package_share_directory

    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    from launch_ros.actions import Node


    def generate_launch_description():
        demo_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('learning_tf2_cpp'), 'launch'),
                '/turtle_tf2_demo.launch.py']),
            launch_arguments={'target_frame': 'carrot1'}.items(),
            )

        return LaunchDescription([
            demo_nodes,
            Node(
                package='learning_tf2_cpp',
                executable='dynamic_frame_tf2_broadcaster',
                name='dynamic_broadcaster',
            ),
        ])
    ```
5. Build
    ```bash
    cd ~/ros2_ws
    rosdep install -i --from-path src --rosdistro foxy -y
    colcon build --packages-select learning_tf2_cpp
    source ~/ros2_ws/install/setup.bash
    ```
6. 執行launch file
    ```bash
    ros2 launch learning_tf2_cpp turtle_tf2_dynamic_frame_demo.launch.py
    ```
    就可以看到`carrot`的相對位置一直跑來跑去。如果移動`turtle1`，`carrot1`的位置也會跟著改變。

# Time
---
`tf2`本身除了座標資訊外，還會有時間的訊息，預設可以存10秒的歷史資料。這邊會介紹如何使用時間的資訊。

1. 讓我們回去複習`turtle_tf2_listener.cpp`，裡面的`lookupTransform()`
    ```cpp
    transformStamped = tf_buffer_->lookupTransform(
        toFrameRel,
        fromFrameRel,
        tf2::TimePointZero);
    ```
    最後一個參數`tf2::TimePointZero`等於0，意思是指定buffer內最後的transform。
    
2. 現在我們不要buffer內最後的transform，而是指定現在的時間，可以用
    ```cpp
    rclcpp::Time now = this->get_clock()->now();
    transformStamped = tf_buffer_->lookupTransform(
        toFrameRel,
        fromFrameRel,
        now);
    ```
    build完後執行launch file，可以看到錯誤訊息
    ```bash
    [INFO] [1629873136.345688064] [listener]: Could not transform turtle1 to turtle2: Lookup would
    require extrapolation into the future.  Requested time 1629873136.345539 but the latest data
    is at time 1629873136.338804, when looking up transform from frame [turtle1] to frame [turtle2]
    ```
    原因有兩個。第一個是listener有自己的buffer用來存不同broadcasters的transforms。第二個是broadcaster送出transform會花掉一些時間。上敘的buffer+時間差造成`lookupTransform()`會找不到現在及時的transform。
3. 解決方法是給他一些幾毫秒的緩衝時間。在`ROS`中會在`lookupTransform`前，使用`waitforTransform`。然而在`ROS2`這點被優化了，可以在第四個參數指定緩衝時間。
    ```cpp
    transformStamped = tf_buffer_->lookupTransform(
        toFrameRel,
        fromFrameRel,
        now,
        50ms);
    ```
    這樣會block住直到有transform可以用，或是超過50ms。
4. 檢查一下是否運作正常，如果跨機器的話這個時間可能要再延長。
    ```bash
    ros2 launch learning_tf2_cpp turtle_tf2_demo.launch.py
    ```

## Time Travel
如果我們不想要`turtle2`馬上追著`carrot1`跑，而是追著`carrot1`5秒鐘前的位置呢？`loopupTransform`都幫你想到了(osrf怎麼這麼貼心??)。

1. 編輯`turtle_tf2_listener.cpp`
    ```cpp
    rclcpp::Time when = this->get_clock()->now() - rclcpp::Duration(5, 0);
    t = tf_buffer_->lookupTransform(
        toFrameRel,
        fromFrameRel,
        when,
        50ms);
    ```
    將`now`換成`when`，而`when`則是被我們操作過的時間，被抹去了5秒，跟迪亞波羅一樣？？？？(JOJO!!!)
2. 這時候重build後啟動launch file
    ```bash
    ros2 launch learning_tf2_cpp turtle_tf2_fixed_frame_demo.launch.py
    ```
    就可以看到`turtle2`會追著`carrot1`5秒前的位置跑。
3. (Optional)`lookupTransform`不只可以魔改source frame的時間，也可以魔改target frame的時間。最多可以有6個參數

    ```cpp
    rclcpp::Time now = this->get_clock()->now();
    rclcpp::Time when = now - rclcpp::Duration(5, 0);
    t = tf_buffer_->lookupTransform(
        toFrameRel,
        now,
        fromFrameRel,
        when,
        "world",
        50ms);
    ```
    1. Target frame`turtle2`
    2. Transform的時間
    3. Source frame`carrot1`
    4. Source frame的時間估計
    5. Fixed frame`world`
    6. Target Frame的等待容許時間

    這樣`tf2`會計算`carrot1`到`world`的transform。接著在`world`frame，tf2時光旅行到5秒前，然後再計算`world`到`turtle2`的transform。這樣就可以得到`turtle2`在5秒前的位置了。
4. 重新build後啟動launch file
    ```bash
    ros2 launch learning_tf2_cpp turtle_tf2_fixed_frame_demo.launch.py
    ```
    就可以看到跟Step2一樣，`turtle2`會追著`carrot1`5秒前的位置跑。


# ROS vs. ROS2
---
這邊的差異只要是在`ROS tf`和`ROS2 tf2`的差異，因為`ROS2`的`tf2`相比`ROS`又有些不同，讓人很頭痛。
| 說明 | ROS tf | ROS2 tf2 |
| :--- | :--- | :--- |
| frame之間的關係 | `rosrun tf tf_echo [src_frame] [target_frame]` | `ros2 run tf2_ros tf2_echo [src_frame] [target_frame]` |
| transform 時間buffer | `waitforTransform`+`lookupTransform` | `lookupTransform` Only |
| 最後的buffer | `ros::Time(0)` | `tf2::TimePointZero` |
| 現在的時間 | `ros::Time::now()` | `this->get_clock()->now()` |
| lookupTransform 回傳 | `void` <br>需要pass transform by reference | `geometry_msgs::msg::TransformStamped` |

> `tf2`其實將滿多`tf`的功能拆分成`tf2`和`tf2_ros`，散落在兩個packages中



# Reference
---
* [ROS2 Writing a static broadcaster (C++)](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html#writing-a-static-broadcaster-c)
* [ROS2 Writing a broadcaster (C++)](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html)
* [ROS2 Writing a listener (C++)](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html)
* [ROS2 Adding a frame (C++)](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Adding-A-Frame-Cpp.html)
* [ROS2 Using time (C++)](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Learning-About-Tf2-And-Time-Cpp.html)
* [Traveling in time (C++)](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Time-Travel-With-Tf2-Cpp.html)
* [ROS tf API](http://docs.ros.org/en/indigo/api/tf/html/c++/classtf_1_1Transformer.html)
* [ROS2 tf2_ros API](http://docs.ros.org/en/indigo/api/tf2_ros/html/c++/classtf2__ros_1_1Buffer.html)
* [蛤 - ROS TF](https://ithelp.ithome.com.tw/articles/10249067?sc=rss.iron)