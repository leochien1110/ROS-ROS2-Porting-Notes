> 雖然官方已經有提供不少好用的Message，但是有時候還是會需要自定義的Message，像是影像偵測完需要畫框，或是需要自定義的Service來執行特別的Robot Health Check。由於篇幅會過長，Service的部分放在Day15來介紹。

# 預設 Message
這邊先介紹常用的Message有：
* `std_msgs`：一些基礎的資料型態，可以去下面Message Fieldtype查看
* geometry_msgs：一些幾何的資料型態，例如Point, Pose, Twist等等
* sensor_msgs：一些感測器的資料型態，例如Image, PointCloud2, NavSatFix等等
* nav_msgs：導航的資料型態，例如Odometry, Path等等
* visualization_msgs：rviz用來畫圖的plugin，例如Marker, MarkerArray等等

其他還有路徑、地圖、Action等等，可以上Github看[ROS2 Common Interface](https://github.com/ros2/common_interfaces/tree/foxy)的內容，裡面有很多預設的Message可以使用。也可以查看[ROS2 Foxy API - msgs & srvs](https://docs.ros2.org/foxy/api/)。

# 自定義 Message
---
之前File Structure的部分沒有提到，一般來說會把Message和Service放在msg和srv資料夾中，這樣在編譯時才會自動產生相關的檔案，如果沒有放在這兩個資料夾中，就需要自己手動產生相關的檔案。

然而習慣上我們會把自定義訊息獨立成一個Package，這樣可以讓其他Packages也可以使用，而不是每個Package都要自己寫一次，如果之間有修改容易造成Message不一致的問題。

這個不成文的規定在ROS2中又更明顯被限制。由於C++和Python已經不能共用Package，而自定義Message又只能用CMake來建立，所以將Message獨立成一個Package，讓不管是C++或是Python的Package都可以使用。

1. 首先建立一個Package，這邊我們叫做`tutorial_interfaces`:
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake tutorial_interfaces
    ```
2. 在`tutorial_interfaces`中建立msg資料夾，用來存放`.msg`檔案:
    ```bash
    cd ~/ros2_ws/src/tutorial_interfaces
    mkdir msg && cd msg
    touch Num.msg
    ```
3. 在`tutorial_interfaces/msg`中建立自定義Message`Num.msg`，其內容為
    ```
    int64 num
    ```
    定義的方式類似C/C++，先宣告型態，再宣告變數名稱。型態的部分可以參考下面的Message Fieldtype。

    如果有其他相似性質的Message，可以一起放在這個資料夾中。譬如說我們再建立一個`Sphere.msg`，其內容為
    ```
    geometry_msgs/Point center
    float64 radius
    ```
4. 編輯`CMakeLists.txt`:
    ```cmake
    find_package(geometry_msgs REQUIRED)
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Num.msg"
        "msg/Sphere.msg"
        # srv也是放這
        DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
    )
    ```
5. 編輯`package.xml`:
    ```xml
    <depend>geometry_msgs</depend>
    <buildtool_depend>rosidl_default_generators</buildtool_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```
    `rosidl_default_generators`可以用來將定義的Message轉成不同語言，編譯時會需要，所以必須要在這裡將Dependencies寫好。`rosidl_default_runtime`則是執行時的dependency。`rosidl_interface_packages`則是宣告這些interface在package內。
6. Build Package:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select tutorial_interfaces
    source install/setup.bash
    ```
7. 確認Message是否有被建立:
    ```bash
    ros2 interface show tutorial_interfaces/msg/Num
    ```
    ```
    會看到以下的結果:
    ```
    int64 num
    ```
    ```bash
    ros2 interface show tutorial_interfaces/msg/Sphere
    ```
    則會看到以下的結果:
    ```
    geometry_msgs/Point center
            float64 x
            float64 y
            float64 z
    float64 radius
    ```
    這樣就完成了自定義Message的建立。

# 測試
---
我們用之前的Publisher和Subscriber來測試一下，這邊我們用Python來測試。回到之前[Day9](Day9_Publisher_Subscriber_Python.md)的Package `py_pubsub`，修改Publisher和Subscriber。或是寫一份新的Python Script也可以，不過要記得修改`setup.py`和`package.xml`。第二種方法比較推薦，因為這樣可以保留之前的範例。

Publisher:
```python
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num    # CHANGE


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Num, 'topic', 10)     # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Num()                                           # CHANGE
        msg.num = self.i                                      # CHANGE
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.num)  # CHANGE
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Subscriber:
```python
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num        # CHANGE


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Num,                                              # CHANGE
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
            self.get_logger().info('I heard: "%d"' % msg.num) # CHANGE


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

`package.xml`新增以下的部分:
```xml
<exec_depend>tutorial_interfaces</exec_depend>
```

接著build:
```bash
cd ~/ros2_ws
colcon build --packages-select py_pubsub
source install/setup.bash
```

最後在兩個不同的Terminal中分別執行:
```bash
ros2 run py_pubsub py_publisher
```
```bash
ros2 run py_pubsub py_subscriber
```

就可以看到Publisher和Subscriber的結果了：
```bash
[INFO] [minimal_publisher]: Publishing: '0'
[INFO] [minimal_publisher]: Publishing: '1'
[INFO] [minimal_publisher]: Publishing: '2'
...
```

# ROS vs. ROS2
---
|說明|ROS|ROS2|差異|
|:---|:---|:---|:---|
|Python API| `from pkg.msg import Msg` | `from pkg.msg import Msg` | 無|
|C++ API| `#include <pkg/Msg.h>` | `#include <pkg/msg/msg.hpp>` | 多一層`/msg`|
|CMakeLists.txt | `find_package(catkin REQUIRED COMPONENTS [deps] message_generation)` <br> `catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs message_runtime)` <br> `generate_messages(DEPENDENCIES std_msgs)` <br>  `add_message_files(FILES custom.msg)` | `find_package(rosidl_default_generators REQUIRED)` <br> `rosidl_generate_interfaces(${PROJECT_NAME} "msg/custom.msg" DEPENDENCIES /deps)`| simplify from `catkin` and use `rosidl_` Macros instead|
| package.xml| `<build_depend>message_generation</build_depend>` <br> `<exec_depend>message_runtime</exec_depend>` | `<depend>[deps]_msgs</depend>` <br> `<buildtool_depend>rosidl_default_generators</buildtool_depend>` <br> `<exec_depend>rosidl_default_runtime</exec_depend>` <br> `<member_of_group>rosidl_interface_packages</member_of_group>` | 新增`rosidl`相關Macro以及`<member_of_group>`|

> 上面的`/deps` 是指其他相依，不是真的有這個package。

# Message Fieldtype
---
內建的Type可以參考[官方文件](https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html#field-types):

| Type name | [C++](https://design.ros2.org/articles/generated_interfaces_cpp.html) | [Python](https://design.ros2.org/articles/generated_interfaces_python.html) | [DDS type](https://design.ros2.org/articles/mapping_dds_types.html) |
| --------- | --------------------------------------------------------------------- | --------------------------------------------------------------------------- | ------------------------------------------------------------------- |
| bool      | bool                                                                  | builtins.bool                                                               | boolean                                                             |
| byte      | uint8_t                                                               | builtins.bytes\*                                                            | octet                                                               |
| char      | char                                                                  | builtins.str\*                                                              | char                                                                |
| float32   | float                                                                 | builtins.float\*                                                            | float                                                               |
| float64   | double                                                                | builtins.float\*                                                            | double                                                              |
| int8      | int8_t                                                                | builtins.int\*                                                              | octet                                                               |
| uint8     | uint8_t                                                               | builtins.int\*                                                              | octet                                                               |
| int16     | int16_t                                                               | builtins.int\*                                                              | short                                                               |
| uint16    | uint16_t                                                              | builtins.int\*                                                              | unsigned short                                                      |
| int32     | int32_t                                                               | builtins.int\*                                                              | long                                                                |
| uint32    | uint32_t                                                              | builtins.int\*                                                              | unsigned long                                                       |
| int64     | int64_t                                                               | builtins.int\*                                                              | long long                                                           |
| uint64    | uint64_t                                                              | builtins.int\*                                                              | unsigned long long                                                  |
| string    | std::string                                                           | builtins.str                                                                | string                                                              |
| wstring   | std::u16string                                                        | builtins.str                                                                | wstring 


內建也有Array Type:
| Type name               | [C++](https://design.ros2.org/articles/generated_interfaces_cpp.html) | [Python](https://design.ros2.org/articles/generated_interfaces_python.html) | [DDS type](https://design.ros2.org/articles/mapping_dds_types.html) |
| ----------------------- | --------------------------------------------------------------------- | --------------------------------------------------------------------------- | ------------------------------------------------------------------- |
| static array            | std::array<T, N>                                                      | builtins.list\*                                                             | T[N]                                                                |
| unbounded dynamic array | std::vector                                                           | builtins.list                                                               | sequence                                                            |
| bounded dynamic array   | custom_class<T, N>                                                    | builtins.list\*                                                             | sequence<T, N>                                                      |
| bounded string          | std::string         

Array的定義方式如下:
```python
int32[] unbounded_integer_array # 無界定整數陣列
int32[5] five_integers_array # 大小為5的整數陣列
int32[<=5] up_to_five_integers_array # 大小不超過5的整數陣列

string string_of_unbounded_size # 無界定字串
string<=10 up_to_ten_characters_string # 長度不超過10的字串

string[<=5] up_to_five_unbounded_strings # 長度不超過5的無界定字串陣列
string<=10[] unbounded_array_of_strings_up_to_ten_characters_each # 無界定長度的字串陣列，每個字串長度不超過10
string<=10[<=5] up_to_five_strings_up_to_ten_characters_each # 長度不超過5的無界定字串陣列，每個字串長度不超過10

```

# Reference
---
* [ROS2 - Creating custom msg and srv files](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
* [ROS - Creating a ROS msg and srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
* [Implementing custom interfaces](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html)
* [ROS Interfaces](https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html)
* [ROS Common Interface Github](https://github.com/ros2/common_interfaces/tree/foxy)
* [ROS2 API - msgs & srvs](https://docs.ros2.org/foxy/api/)