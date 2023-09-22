> 發現使用官方的範例有點像是搬運工，不過我覺得他們寫得很清楚，就不特別去寫一個自己的範例了，不過會加上自己的註解，希望可以更好懂。這邊補充說明一下，程式碼內我會用`...`表示省略，不要真的寫進去![/images/emoticon/emoticon01.gif](/images/emoticon/emoticon01.gif)
 
# Publisher
---
一樣我們下載官方的範例後再來講解。

1. 首先Python和C++原則上不會再像`ROS`一樣共用Package了，因此需要重新建立一個
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake cpp_pubsub
    ```
2. 接著下載範例
    ```bash
    cd ~/ros2_ws/src/cpp_pubsub/src
    wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/foxy/rclcpp/topics/minimal_publisher/member_function.cpp
    ```
3. 就可以看到`publisher_member_function.cpp`
    ```cpp
    #include <chrono>
    #include <functional>
    #include <memory>
    #include <string>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"

    using namespace std::chrono_literals;

    /* This example creates a subclass of Node and uses std::bind() to register a
    * member function as a callback from the timer. */

    class MinimalPublisher : public rclcpp::Node
    {
    public:
        MinimalPublisher()
        : Node("minimal_publisher"), count_(0)
        {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
        }

    private:
        void timer_callback()
        {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
    };

    int main(int argc, char * argv[])
    {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
    }
    ```
## 解析
```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
```
這邊引入需要的Library，其中`rclcpp/rclcpp.hpp`就是ROS2的C++ Library，`std_msgs/msg/string.hpp`就是我們要用到的`std_msgs::msg::String`。記得等一下要把這些Library加入到`CMakeLists.txt`和`package.xml`中。

```cpp
class MinimalPublisher : public rclcpp::Node
```
這邊建立一個Class，並且繼承`rclcpp::Node`，這樣就可以使用ROS2的功能了。這也是為什麼檔名叫做member_function，因為是使用Class的member function來實現的，而不是`ROS`[範例](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)中寫在while loop內的方法。

```cpp
public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }
```
接著Constructor將Node初始化命名為`minimal_publisher`，初始化`count_`為0。

`this->create_publisher<std_msgs::msg::String>("topic", 10)`在這個Node(`this`)建立一個Publisher，要發布的Topic name為`topic`，Queue Size為10。

`this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this))`則是建立Timer，時間為500ms觸發一次，並用`std::bind`將Callback Function`timer_callback`和`this(這個class本身，類似Python的self.)`綁定。

```cpp
private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
```
Callback Function會在Timer觸發時執行，會將`count_`加1，並且Publish一個`std_msgs::msg::String`的Message。

> 注意這邊logger的寫法和rclpy不一樣，這邊是使用`RCLCPP_INFO`，而rclpy是使用`self.get_logger().info()`。
> 
> logger一樣有分成`INFO`、`DEBUG`、`WARN`、`ERROR`、`FATAL`。
```cpp
RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
RCLCPP_DEBUG(this->get_logger(), "Debugging");
RCLCPP_WARN(this->get_logger(), "Warning");
RCLCPP_ERROR(this->get_logger(), "Error");
RCLCPP_FATAL(this->get_logger(), "Fatal");
```

回到程式碼
```cpp
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
size_t count_;
```
這邊則是class member的變數宣告，可以看到`ROS2`大量運用smart pointer，這邊就是使用`SharedPtr`，這樣就不用自己管理記憶體了。但也讓`ROS2` C++入門門檻變高了不少。

```cpp
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
```
最後就是main function，這邊就是初始化ROS2，並且執行`MinimalPublisher`，最後再關閉ROS2，除了用`std::make_shared`直接宣告`MinimalPublisher`而不是創建一個物件外，其他都跟Python差不多。

## Package設定
不要忘記更新dependencies到`package.xml`和`CMakeLists.txt`中。雖然之前沒有特別提到，但是等專案變大後，`<description>`,`<maintainer>`, `<license>`等資訊也最好更新一下以便維護。
```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```
```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
...
add_executable(talker src/publisher_member_function.cpp)
# 沒有用到非ROS2的Library，所以不用加入target_link_libraries
ament_target_dependencies(talker rclcpp std_msgs)
...
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
```

官方教學也有提供乾淨版的CMakeLists.txt，清掉不少`pkg create`模板的註解，可以參考一下：
```cmake
cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

我們等Subcriber實作完後再來Build和執行。

# Subscriber
---
1. 下載Subscriber範例
    ```bash
    cd ~/ros2_ws/src/cpp_pubsub/src
    wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/foxy/rclcpp/topics/minimal_subscriber/member_function.cpp
    ```
2. 打開`subscriber_member_function.cpp`
    ```cpp
    #include <memory>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"
    using std::placeholders::_1;

    class MinimalSubscriber : public rclcpp::Node
    {
    public:
        MinimalSubscriber()
        : Node("minimal_subscriber")
        {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        }

    private:
        void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
        {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    };

    int main(int argc, char * argv[])
    {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
    }
    ```
## 解析
```cpp
public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }
```
這邊就是建立一個Subscriber，Topic為`topic`，Queue Size為10，並且將Callback Function`topic_callback`和`this`綁定。

後面的`_1`是因為`topic_callback`有一個參數，因此要用`std::placeholders::_1`來綁定。之後進階使用到timesync時會用需要同步多個Topic，因此會用到`_2`、`_3`等等，bind最多可以吃到八個arguments。

```cpp
private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
```
Callback Function會在收到Message時執行，並且印出Message的內容。

main則完全一樣，這邊就不再列出。

## Package設定
沒有新增加的Library，因此只需將subscriber加入`CMakeLists.txt`
```cmake
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
...
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```

## Build
每次有新的package時，最好先確認一下rosdep
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro foxy -y
```
接著Build Package
```bash
colcon build --packages-select cpp_pubsub
```
最後source
```bash
source install/setup.bash
```

## Run
先在一個terminal執行Publisher
```bash
ros2 run cpp_pubsub talker
```
應該就可以看到Publisher在每500ms發送一個Message
```bash
[INFO] [1600000000.000000000] [minimal_publisher]: Publishing: 'Hello, world! 1'
[INFO] [1600000000.500000000] [minimal_publisher]: Publishing: 'Hello, world! 2'
...
```
接著在另一個terminal執行Subscriber
```bash
ros2 run cpp_pubsub listener
```
就可以看到Subscriber收到Message
```bash
[INFO] [1600000000.000000000] [minimal_subscriber]: I heard: 'Hello, world! 1'
[INFO] [1600000000.500000000] [minimal_subscriber]: I heard: 'Hello, world! 2'
...
```
有趣的是，這邊的talker和listener可以和Python的talker和listener互相溝通，因為他們的Topic name都是`topic`，Message的格式都一樣是`std_msgs::msg::String`。可以試著打開`py_pubsub`的`talker`和`cpp_pubsub`的`listener`，或是`py_pubsub`的`listener`和`cpp_pubsub`的`talker`，就可以看到他們互相溝通了。

>:warning: 注意不可以同時打開兩個packages的`talker`或是`listener`，因為的node name都是一樣的，會造成衝突。
>
> 可以試著改變其中一個的node name，就可以同時打開兩個`talker`或是兩個`listener`了，看看會有什麼變化。

# Reference
---
* [Write a Simple Publisher and Subscriber (C++)](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
* [rclcpp logging](https://docs.ros2.org/bouncy/api/rclcpp/logging_8hpp.html)
* [Writing a Simple Publisher and Subscriber (C++) ROS ver.](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)