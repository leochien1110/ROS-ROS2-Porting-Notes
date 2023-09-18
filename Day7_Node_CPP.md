> 延續昨天的主題，今天要來探討C++版本的Node，不過C++的使用上就比較搞剛一些。我會把說明附在comment裡面，這樣可以直接對照每一行的功能。
> 另外，從這邊開始會帶到一些Command Line Tools(CLI)，這邊會用到`ros2 pkg`和`ros2 run`，這兩個指令可以幫助我們快速的創建Package和執行Node。


# C++ Node
C++的API從`ROS`的`roscpp`改成`ROS2`的`rclcpp`，所以在include的時候要注意。這邊先來簡單的創一個Package和Hello World的Node。

## 創建Package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --node-name hello_world beginner_tutorials_cpp
```

創建完後，會在`~/ros2_ws/src`底下看到`beginner_tutorials_cpp`這個資料夾，裡面有：
```bash
beginner_tutorials_cpp/
├── CMakeLists.txt
├── include
│   └── beginner_tutorials_cpp
├── package.xml
├── src
    └──  hello_world.cpp
```

其中`hello_world.cpp`就是我們的ROS Node，今天不會寫到Class，所以`include`底下的`beginner_tutorials_cpp`目前是空的，否則一般會放一個`hello_world.hpp`。

## 撰寫 Hello World Node
1. 首先編輯`hello_world.cpp`：
    ```cpp
    #include "rclcpp/rclcpp.hpp"

    // Node header file，目前沒有Class所以不用include
    // #include "beginner_tutorials_cpp/hello_world.hpp"

    int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);   // 初始化ROS
        
        // 創建一個叫做hello_world_node的Node 
        auto node = rclcpp::Node::make_shared("hello_world_node");  
        
        // 用Node的get_logger() function來print出Hello World!
        RCLCPP_INFO(node->get_logger(), "Hello World!");
        
        // 讓Node持續運行
        rclcpp::spin(node);
        
        // 關閉ROS
        rclcpp::shutdown();
        
        return 0;
    }
    ```
2. 再來編輯`CMakeLists.txt`，加入`find_package(rclcpp REQUIRED)`和`ament_target_dependencies(hello_world_node rclcpp)`：
    ```cmake
    cmake_minimum_required(VERSION 3.5)
    project(beginner_tutorials_cpp)

    # Default to C++14
    if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 14)
    endif()

    ...

    # find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)

    ...

    # 增加一個executable，名稱叫做hello_world_node，並且link rclcpp
    add_executable(hello_world_node src/hello_world.cpp)
    ament_target_dependencies(hello_world_node rclcpp)

    # install這個executable
    install(TARGETS
        hello_world_node
        DESTINATION lib/${PROJECT_NAME}
    )

    ...

    # install這個package
    install(DIRECTORY
        include/
        DESTINATION include/
    )

    ament_package()
    ```

3. 最後編輯`package.xml`，加入`<build_depend>rclcpp</build_depend>`和`<exec_depend>rclcpp</exec_depend>`：
    ```xml
    ...
    <buildtool_depend>ament_cmake</buildtool_depend>
    <build_depend>rclcpp</build_depend>

    <exec_depend>rclcpp</exec_depend>
    ...
    ```

## 執行
回到Workspace底下，執行build：
```bash
cd ~/ros2_ws
colcon build --packages-select beginner_tutorials_cpp
```

`--symlink-install`這邊對C++沒有用，所以每次修改完程式碼後都要重新colcon build。

執行完後記得先source：
```bash
source ~/ros2_ws/install/setup.bash
```

執行：
```bash
ros2 run beginner_tutorials_cpp hello_world_node
```

就可以跟昨天一樣看到`Hello World!`了。

## 持續執行
和昨天一樣可以用Loop來讓Node持續輸出，但是C++的寫法跟Python不太一樣，這邊來簡單的介紹一下。

### `ROS` 寫法
首先第一個方法，跟`ROS`寫法比較接近，使用rate + loop:
```cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);   // 初始化ROS
    
    // 創建一個叫做hello_world_node的Node 
    auto node = rclcpp::Node::make_shared("hello_world_node");  
    
    // 用Node的get_logger() function來print出Hello World!
    RCLCPP_INFO(node->get_logger(), "Hello World!");
    
    // use rate to loop at 1Hz
    rclcpp::WallRate loop_rate(1);

    // 讓Node持續運行
    while(rclcpp::ok()) {
        rclcpp::spin_some(node);
        RCLCPP_INFO(node->get_logger(), "Hello World in Loop!");
        loop_rate.sleep();
    }
    
    // 關閉ROS
    rclcpp::shutdown();
    
    return 0;
}
```

首先了解`ROS`的人會先發現，我們已經慢慢進入Smart Pointer的世界了。在`ROS2`中，Node的型別是指針，所以不是用傳統的Object oriented的寫法，而是用`make_shared()`來創建一個Node。這邊的`make_shared`是C++的Smart Pointer，可以參考[Geeks for geeks 的這篇](https://www.geeksforgeeks.org/auto_ptr-unique_ptr-shared_ptr-weak_ptr-2/)。

再來可以到看，`ROS`的`ros::spinOnce`已經被`rclcpp::spin_some`取代了，而`ROS` C++中的`ros::Rate`則是被`rclcpp::WallRate`取代了。


### `ROS2` 寫法
第二個方法比較進階，但也是`ROS2` callback function的寫法，也可以使用Lambda取代callback function。以下列出兩種寫法，把其中一種註解掉就可以執行另一種：
```cpp
#include "rclcpp/rclcpp.hpp"
#include <chrono>

void callback(rclcpp::Node::SharedPtr node) {
    RCLCPP_INFO(node->get_logger(), "Hello World in Loop!");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);   // 初始化ROS
    
    // 創建一個叫做hello_world_node的Node 
    auto node = rclcpp::Node::make_shared("hello_world_node");  
    
    // 用Node的get_logger() function來print出Hello World!
    RCLCPP_INFO(node->get_logger(), "Hello World!");

    callback function寫法
    auto timer = node->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&callback, node)
    );

    // // 創建一個Timer，每秒執行一次，Lambda寫法
    // auto timer = node->create_wall_timer(
    //     std::chrono::seconds(1), 
    //     [&node]() -> void {
    //         RCLCPP_INFO(node->get_logger(), "Hello World!");
    //     }
    // );
    
    // 讓Node持續運行
    rclcpp::spin(node);
    
    // 關閉ROS
    rclcpp::shutdown();
    
    return 0;
}
```

這邊T我們用`node->create_wall_timer()`來創建一個Timer，第一個參數是時間，第二個參數是一個function，可以是callback function或是Lambda。這邊我們用`std::bind()`來把callback function綁定到`node`。Lambda的話則是直接用`[&node](){}`來寫。

完成後記得要重新`colcon build`，然後執行：
```bash
ros2 run beginner_tutorials_cpp hello_world_node
```
就可以看到每秒print一次`Hello World!`了。


# ROS2指令 Command Line Tools(CLI)
`ROS2`的指令跟`ROS`的指令有些不同，這邊來簡單的介紹一下。

## ros2 pkg
`ros2 pkg`可以幫助我們快速的創建Package，也可以查看Package的資訊。以我們的實作為例，我們可以用`ros2 pkg`來查看他的資訊：
```bash
ros2 pkg list | grep beginner_tutorials
```
如果把`| grep beginner_tutorials`拿掉，可以看到所有的`ROS2` Package。

可以看到我們的`beginner_tutorials_cpp`和`beginner_tutorials_py`。

可以查看可執行的ROS Node:
```bash
ros2 pkg executables beginner_tutorials_cpp
```

## ros2 node
`ros2 node`可以查看目前正在執行的Node，也可以查看Node的資訊。以我們的實作為例，我們可以用`ros2 node`來查看目前正在執行的Node。但首先要先執行一個Node，這邊我們用`ros2 run`來執行`hello_world_node`：
```bash
ros2 run beginner_tutorials_cpp hello_world_node
```
接著，我們可以用`ros2 node list`來查看目前正在執行的Node：
```bash
ros2 node list
```
可以看到目前正在執行的Node有`/hello_world_node`。再來我們可以用`ros2 node info`來查看Node的資訊：
```bash
ros2 node info /hello_world_node
```
這個指令也可以用來檢查Node是否訂閱或發佈某個Topic，或是是否提供或使用某個Service，後面會再介紹。

最後，我們可以用`ros2 node -h`來查看`ros2 node`的其他指令：
```bash
ros2 node -h
```


# ROS vs. ROS2

|功能 | ROS | ROS2 |
|---|---|---|
|Python API | rospy | rclpy |
|Python Rate | `rate = rospy.Rate(10) rate.sleep()` | `loop_rate = node.create_rate(10) loop_rate.sleep()` |
|C++ API | roscpp | rclcpp |
|C++ Node Declaration | `ros::NodeHandle nh;`` | `rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("node_name");`` |
|C++ Rate | `roscpp::rate rate(10); ros::spinOnce(); rate.sleep();` | `rclcpp::WallRate loop_rate(10); rclcpp::spin_some(node); loop_rate.sleep();` |
|CMakeLists.txt | `find_package(catkin REQUIRED COMPONENTS <dependency1> <dependency2> ...)` | `find_package(ament_cmake REQUIRED) find_package(<dependency1> REQUIRED) find_package(<dependency2> REQUIRED) ...` |
| Loop Usage | Rate + spinOnce | Timer + callback + spin |
| Run Node | `rosrun pkg_name node_name` | `ros2 run pkg_name node_name` |


# Reference
---
* [蛤 - Day 07 - 使用C++撰寫ROS Node](https://ithelp.ithome.com.tw/articles/10204122)
* [ROS2 Official Tutorial - Creating a package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#create-a-package)
* [Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
* [C++ ROS Client Library API](https://docs.ros2.org/foxy/api/rclcpp/index.html)