> 今年的鐵人賽好硬阿，遇到一堆連假，好險我前面就被淘汰了(誤，不然國慶連假還要趕稿的話就太慘了哈哈。![/images/emoticon/emoticon70.gif](/images/emoticon/emoticon70.gif)

# Transform
---
在Robotics中，Coordinates Transform(座標轉換)是一個非常重要的概念，因為每個Sensor(感測器)、Actuator(作器氣)、Robot本身都有自己的Coordinates(座標系統)，而必須要知道自身各個部件的相對位置，才能做Sensor Fusion(感測器融合)的任務，最後才能做到定位、避障、導航等等的功能。

以Perception為例，實作上會有數個不同的測距sensors(LiDAR、Radar和Depth Camera)，分別有不同優勢，像是LiDAR精度高距離遠、Radar測速度很準距離遠、Depth Camera幀數高密度高。這時Robot必須要知道Sensors相對的位置，看到前方障礙物的距離才不會有誤差造成誤判。

控制系統也需要精準的座標轉換，像是手臂控制系統，需要知道手臂各個關節的相對位置，才能做到精準的控制。輪型機器人也需要知道輪子的相對位置、轉速、轉向角度，才能做到精準的控制。

座標轉換是一門很大的學問，其中牽涉許多線性代數、相對座標和旋轉的概念。好在`ROS2`提供了`tf2`套件，可以幫助我們做到座標轉換的功能。不過還是建議大家要先了解一些線性代數的概念，才能更好的理解tf2的功能。

tf2功能很多，這邊只會利用官方的Turtlebot介紹一些基本的功能，詳細的功能可以參考[ROS2 tf2](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Tf2-Main.html)。

## 基本概念
在三維坐標軸中，一般使用笛卡爾座標系統(Cartesian Coordinate System)來表示。平移(translation)中有三個軸，分別是x、y和z軸。旋轉(rotation)的則是繞著軸的方向，分別是x(roll)、y(pitch)和z(yaw)。所以可以用六個數字來表示一個座標，分別是x、y、z、roll、pitch和yaw。

平移是屬於**線性的運算，可以相加減**，而旋轉則是屬於***非*線性的運算，不能相加減**，而是要用三角函數或是四元數來計算。旋轉矩陣一個3x3的矩陣，而四元數則是一個4維的向量。

旋轉矩陣的疊加是用矩陣相乘，用來表示旋轉再旋轉。具體的操作可以參考維基百科的[旋轉矩陣](https://en.wikipedia.org/wiki/Rotation_matrix):

![Genral 3D Rotations](https://wikimedia.org/api/rest_v1/media/math/render/svg/234c5831df9d48e5dc4a1cc130475d3426a64ce1)

不過由於笛卡爾座標系下的旋轉矩陣會有[Gimbal Lock](https://en.wikipedia.org/wiki/Gimbal_lock#:~:text=Gimbal%20lock%20is%20the%20loss,a%20degenerate%20two%2Ddimensional%20space)(萬向鎖)的問題，所以為了避免這個狀況，會使用四元數(Quaternion)來表示旋轉。

四元數的疊加是指將兩個四元數相乘，用來表示旋轉再旋轉。具體的操作可以參考維基百科的[四元數](https://en.wikipedia.org/wiki/Quaternion)或是[Visualizing Quaternions](https://eater.net/quaternions)。


## Turtlebot Simulation
`ROS2`提供了簡單的2D Turtle Simulation，可以用來學習很多**ROS**的基礎功能，像是Publisher、Subscriber、Service、Action、Parameter、Launch、Bag等等。這邊我們會利用Turtlebot來學習tf2的功能。

我們首先來開一個烏龜模擬器，然後用鍵盤控制烏龜移動，好讓我們從
範例中觀察tf2。

1. 安裝turtlesim:
    ```bash
    sudo apt-get install ros-foxy-turtle-tf2-py ros-foxy-tf2-tools ros-foxy-tf-transformations
    ```
2. 開啟烏龜模擬器`tf demo` Node:
    ```bash
    ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py
    ```
    你會看到turtlesim的視窗中有兩隻烏龜。
    ![turtle_tf2_demo](https://docs.ros.org/en/foxy/_images/turtlesim_follow1.png)
3. 在另一個terminal中，開啟控制烏龜的Node:
    ```bash
    ros2 run turtlesim turtle_teleop_key
    ```
    就可以用鍵盤控制烏龜移動啦！你會發現其中一隻烏龜會追著你跑。

## tf2介紹
在上述的turtlesim中，我們有三個frames(座標)，分別是`world`、`turtle1`和`turtle2`。上面另一隻烏龜會追著你跑就是利用`tf2`的`broadcaster`發布烏龜們的座標系統，然後用`tf2`的`listener`去計算兩隻烏龜的相對位置，然後控制另一隻烏龜追著你跑。

除了上述API的`broadcast`和`listen`之外，`tf2`還有不少實用的功能:
1. `view_frames`: 畫出frame之間的關係。
    
    透過`ros2 run tf2_tools view_frames.py`，就可以將目前Scene中的frame關係畫出來並存到`frames.pdf`中，讓我們可以觀察frame之間的關係。
2. `tf2_echo`: 顯示frame之間的關係。
    
    透過`ros2 run tf2_ros tf2_echo [source_frame] [target_frame]`，就可以顯示兩個frame之間的關係。

    上述例子中，我們可以透過`ros2 run tf2_ros tf2_echo turtle1 turtle2`來顯示`turtle1`和`turtle2`之間的關係。

3. `rviz`: 3D視覺化工具。
    ```bash
    ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz
    ```
4. `rqt_tf_tree`: 顯示frame之間的關係。
    ```bash
    ros2 run rqt_tf_tree rqt_tf_tree
    ```
關於`rviz`和`rqt_`等使用者介面的工具，我們會在後面的章節介紹，這邊提早出場讓大家玩玩看。

# Static Boardcaster
---
在一開始介紹時有提到Sensor之間的相對位置是固定的，這時我們可以使用`static broadcaster`來發布frame之間的關係，讓`tf2`可以計算出frame之間的關係。

這邊直接時做一個package來示範`static broadcaster`的用法。
1. 創建一個package:
    ```bash
    ros2 pkg create --build-type ament_cmake --dependencies geometry_msgs rclcpp tf2 tf2_ros turtlesim -- learning_tf2_cpp
    ```
2. 撰寫static broadcaster node。這邊我們偷懶直接下載官方的範例，然後修改一下。
    ```bash
    cd ~/ros2_ws/src/learning_tf2_cpp/src
    wget https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_cpp/src/static_turtle_tf2_broadcaster.cpp
    ```
3. 開啟`static_turtle_tf2_broadcaster.cpp`:
    ```cpp
    #include <memory>

    #include "geometry_msgs/msg/transform_stamped.hpp"
    #include "rclcpp/rclcpp.hpp"
    #include "tf2/LinearMath/Quaternion.h"
    #include "tf2_ros/static_transform_broadcaster.h"

    class StaticFramePublisher : public rclcpp::Node
    {
    public:
    explicit StaticFramePublisher(char * transformation[])
    : Node("static_turtle_tf2_broadcaster")
    {
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Publish static transforms once at startup
        this->make_transforms(transformation);
    }

    private:
    void make_transforms(char * transformation[])
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = transformation[1];

        t.transform.translation.x = atof(transformation[2]);
        t.transform.translation.y = atof(transformation[3]);
        t.transform.translation.z = atof(transformation[4]);
        tf2::Quaternion q;
        q.setRPY(
        atof(transformation[5]),
        atof(transformation[6]),
        atof(transformation[7]));
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_static_broadcaster_->sendTransform(t);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    };

    int main(int argc, char * argv[])
    {
    auto logger = rclcpp::get_logger("logger");

    // Obtain parameters from command line arguments
    if (argc != 8) {
        RCLCPP_INFO(
        logger, "Invalid number of parameters\nusage: "
        "$ ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster "
        "child_frame_name x y z roll pitch yaw");
        return 1;
    }

    // As the parent frame of the transform is `world`, it is
    // necessary to check that the frame name passed is different
    if (strcmp(argv[1], "world") == 0) {
        RCLCPP_INFO(logger, "Your static turtle name cannot be 'world'");
        return 1;
    }

    // Pass parameters and initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticFramePublisher>(argv));
    rclcpp::shutdown();
    return 0;
    }
    ```

    * `transform_stamped.hpp`中的`TransformStamped`的message讓我們可以用來發布到tranformation tree。
    * `Quaternion.h`用來處理四元數的運算。在**ROS**中，旋轉可以用Euler Angle(歐拉角, RPY)或是[Quaternion](https://openhome.cc/Gossip/ComputerGraphics/QuaternionsRotate.htm)(四元數, Quaternion)來表示，而四元數的運算比較不會遇到Gimbal Lock(萬向鎖)的問題，所以在**ROS**中較常使用四元數來表示旋轉。更多旋轉的表示法可以參考[這篇](https://silverwind1982.pixnet.net/blog/post/258069682)。
    * `static_transform_broadcaster.h`中的`StaticTransformBroadcaster`可以讓我們發布static transformation。

    ```cpp
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish static transforms once at startup
    this->make_transforms(transformation);
    ```
    在constructor中，我們先創建`StaticTransformBroadcaster`的物件，類似於`Publisher`的功能。

    ```cpp
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = transformation[1];
    ```
    在`make_transforms`中，我們先創建`TransformStamped`的物件，並且設定`header`的時間戳記、frame id和child frame id。

    ```cpp
    t.transform.translation.x = atof(transformation[2]);
    t.transform.translation.y = atof(transformation[3]);
    t.transform.translation.z = atof(transformation[4]);
    tf2::Quaternion q;
    q.setRPY(
        atof(transformation[5]),
        atof(transformation[6]),
        atof(transformation[7]));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    ```
    這裡`atof`將字串轉成浮點數，`setRPY`將roll、pitch和yaw轉成四元數，然後將四元數的值設定到`TransformStamped`的`transform`中。

    ```cpp
    tf_static_broadcaster_->sendTransform(t);
    ```
    最後透過`sendTransform`發布`TransformStamped`，類似於`publish`的功能。
4. 編輯`CMakeLists.txt`，新增可執行檔以及安裝檔案:
    ```cmake
    add_executable(static_turtle_tf2_broadcaster src/static_turtle_tf2_broadcaster.cpp)
    ament_target_dependencies(
        static_turtle_tf2_broadcaster
        geometry_msgs
        rclcpp
        tf2
        tf2_ros
    )
    ...
    install(TARGETS
        static_turtle_tf2_broadcaster
        DESTINATION lib/${PROJECT_NAME})
    ```
5. Build:
    ```bash
    cd ~/ros2_ws
    rosdep install -i --from-path src --rosdistro foxy -y
    colcon build --packages-select learning_tf2_cpp
    ```
6. 執行`static_turtle_tf2_broadcaster`:
    ```bash
    ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0
    ```
    這會讓`mystaticturtle`這個frame的座標設定在`(0, 0, 1)`且沒有旋轉。
7. 查看static transformation:
    ```bash
    ros2 topic echo --qos-reliability reliable --qos-durability transient_local /tf_static
    ```
    可以看到`/tf_static`的topic中有`mystaticturtle`的座標資訊。
    ```bash
    transforms:
    - header:
        stamp:
            sec: 1622908754
            nanosec: 208515730
        frame_id: world
    child_frame_id: mystaticturtle
    transform:
        translation:
            x: 0.0
            y: 0.0
            z: 1.0
        rotation:
            x: 0.0
            y: 0.0
            z: 0.0
            w: 1.0
    ```
## Command Line Tools
其實一般實作上不會特地寫一個static broadcaster node，而是直接使用`tf2`提供的command line tools來發布static transformation。兩種旋轉表示的模板如下:
```bash
# RPY: roll, pitch, yaw
ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll frame_id child_frame_id
# Quaternion: x, y, z, w
ros2 run tf2_ros static_transform_publisher x y z qx qy qz qw frame_id child_frame_id
```

也可以直接用在launch中:

XML
```xml
<launch>
    <node pkg="tf2_ros" type="static_transform_publisher" name="my_static_transform_publisher" args="0 0 1 0 0 0 world mystaticturtle"/>
</launch>
```
Python
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '1', '0', '0', '0', 'world', 'mystaticturtle']
        ),
    ])
```

鑒於篇幅，Broadcaster、Listener、Frame、Time等進階功能留到明天再說吧。


# ROS vs. ROS2
---
| 說明 | ROS | ROS2 |
| :--- | :--- | :--- |
| frame之間的關係 | `rosrun tf tf_echo [src_frame] [target_frame]` | `ros2 run tf2_ros tf2_echo [src_frame] [target_frame]` |



# Reference
---
* [ROS2 tf2](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)
* [ROS2 Quaternions](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html)
* [Roation Matrix](https://en.wikipedia.org/wiki/Rotation_matrix)
* [Quaternion](https://en.wikipedia.org/wiki/Quaternion)
* [Gimbal Lock](https://en.wikipedia.org/wiki/Gimbal_lock#:~:text=Gimbal%20lock%20is%20the%20loss,a%20degenerate%20two%2Ddimensional%20space.)
* [Visualizing Quaternions](https://eater.net/quaternions)
* [蛤 - ROS TF](https://ithelp.ithome.com.tw/articles/10249067?sc=rss.iron)