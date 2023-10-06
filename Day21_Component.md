# Composition
---
在運行Robot時，有時需要一些簡單的程式幫我們做一些簡單的pipeline工作，像是影像濾波器處理完後丟給影像壓縮的流水線，但如果分成不同Node跑在不同thread上時，在傳遞資料時會有一些額外的開銷，所以`ROS`提供了一個機制叫做**Nodelet**，可以將多個小Node跑在同一個thread中避免不必要的訊息複製。這個機制可以想像是一個生產線，可以慢慢的把不同的工具和工法加進去輸送帶，去精修或微調產線。

而`ROS`中運行程式主要靠兩個單位:
1. **Node**，會經由`catkin build`變成一個可執行的binary檔案
2. **Nodelet**，在編譯之後會變成一個library檔案，會在runtime時被容器(**Nodelet Manager**)呼叫。

在`ROS2`中這種機制則叫做**Composistion**，用來管理多個小任務(component)。**Nodelet**改名成**Component**，而**Nodelet Manager**改名成**Component Container**，整體的概念是一樣的。其中改善了`ROS`的一些缺點，像是**Node**和**Component**可以共用API。

雖然**Nodelet**和**Component**都有提供多執行序的功能，但僅適合對順序不敏感的工作，例如Pointcloud的passthrough filter會拆成x-y-z分開過濾，順序不影響最後結果的話就可以用**多執行序**。如果想確保訊息的傳遞順序，例如影像處理要先降噪再做特徵點偵測，則要使用**單執行緒**的方式。

## 撰寫Component
因為Component是shared library，他不會有`main` function，而其通常繼承自`rclcpp::Node`，所以我們可以在`constructor`中撰寫我們的程式。另外要注意的點是，Component是強制單執行緒的，所以要盡量簡化處理的工作，也不要使用Blocking的工作造成程式卡頓。

這邊我們參照官方給的[Talker Component](https://github.com/ros2/demos/blob/foxy/composition/src/talker_component.cpp)範例。
```cpp
...
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(composition::Talker)
```

可以看到是沒有`main`，還有底下的`RCLCPP_COMPONENTS_REGISTER_NODE`，這是用來註冊Component的，讓`ROS2`知道這個library是一個Component。

而`CMakeLists.txt`可以參考範例，有幾個必要加入的項目:
```cmake
find_package(rclcpp_components REQUIRED)

add_library(talker_component SHARED
  src/talker_component.cpp)
ament_target_dependencies(talker_component
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
rclcpp_components_register_nodes(talker_component "composition::Talker")
```
範例上有額外的`target_compile_definitions(talker_component  PRIVATE "COMPOSITION_BUILDING_DLL")`是給Windows Compile用的。`set(node_plugins`則是build test時用來管理file structure的。

在`ROS2`中要把component轉換成node變得非常簡單，只要寫一個簡單的node:
```cpp
#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "composition/talker_component.hpp"

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<composition::TalkerComponent>(
      rclcpp::NodeOptions()));

  rclcpp::shutdown();

  return 0;
}
```
接著在`CMakeLists.txt`中加入:
```cmake
add_executable(talker_node src/driver/talker_node.cpp)
ament_target_dependencies(talker_node
  rclcpp
)
target_link_libraries(talker_node talker_component)
```
這樣就可以把component轉換成node了，當然component也還是可以直接被container呼叫。



## 執行
Component其實是個大坑，真的要介紹的話至少要兩到三篇。礙於篇幅和自己也不太熟悉手刻container，這邊直接給出範例的執行方式，對於其他細節有興趣的可以參考官方的[教學](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Composition.html)。

1. 首先去下載[demos/composition](https://github.com/ros2/demos/tree/foxy/composition)範例package:
    ```bash
    cd ~/Downloads
    git clone -b foxy https://github.com/ros2/demos.git
    cp -r demos/composition ~/ros2_ws/src
    ```

2. colcon編譯:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select composition
    source install/setup.bash
    ```

3. 查看component的list:
    ```bash
    ros2 component types
    # 可以看到所有可以用的component
    (... components of other packages here)
    composition
    composition::Talker
    composition::Listener
    composition::Server
    composition::Client
    ```
4. 使用`ROS2`的Services讓Composition跑起來
    ```bash
    ros2 run rclcpp_components component_container
    ```
    這邊`component_container`是官方提供的container，是用Single Thread的方式運行component。
    開啟另一個terminal，檢查component container是否運行成功:
    ```bash
    ros2 component list
    # 可以看到component的名稱
    /ComponentManager
    ```
5. 在第二個terminal中載入talker component:
    ```bash
    ros2 component load /ComponentManager composition composition::Talker
    # 可以看到載入的component ID和node名稱
    Loaded component 1 into '/ComponentManager' container node as '/talker'
    ```
    此時第一個terminal會顯示talker node發布的訊息。
6. 接著開啟第三個terminal來載入listener component:
    ```bash
    ros2 component load /ComponentManager composition composition::Listener
    # 可以看到載入的component ID和node名稱
    Loaded component 2 into '/ComponentManager' container node as '/listener'
    ```
    此時第一個terminal會顯示talker和listener node的訊息。
7. 查看container中的component:
    ```bash
    ros2 component list
    # 可以看到載入的component ID和node名稱
    /ComponentManager
        1  /talker
        2  /listener
    ```

## 用launch來執行
上面的步驟會需要開很多個terminal，但常常composition是用來做一些pipeline的工作，所以我們可以用launch來幫我們一次執行多個component。
可以到`ros2_ws/src/composition/launch`中看到官方提供的範例，這邊我們來看看`composition_demo.launch.py`:
```python
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='composition',
                    plugin='composition::Talker',
                    name='talker'),
                ComposableNode(
                    package='composition',
                    plugin='composition::Listener',
                    name='listener')
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
```

`ComposableNodeContainer`是用來呼叫container。接著可以看到他的`composable_node_descriptions`是一個list，可以載入多個components，這邊我們載入了talker和listener component。最後回傳一個`launch.LaunchDescription`物件。

接著我們來執行:
```bash
ros2 launch composition composition_demo.launch.py
```

這樣一行就可以達成我們剛剛用三個terminal做的事情。

# ROS nodelet vs. ROS2 component
---
| 說明 | ROS | ROS2 |
| :--- | :--- | :--- |
| 語言 | C++ | C++ |
| 容器 | nodelet manager | component container |
| 執行位置 | `Class::onInit()` | Class Constructor |
| 和Node共用性 | 不同API，需要另外寫 | 相同API，可以把component當成library，node只要include component class就可以使用 |
| config檔案 | nodelet.xml | 不需要 |
| CMakeLists.txt | `find_package(catkin .. nodelet)` <br>`install(FILES your_nodelet.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})` <br>`install(TARGETS your_nodelet LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})` | `find_package(rclcpp_components REQUIRED)` <br>`rclcpp_components_register_nodes(your_component "composition::<your_component_name>")` |
| package.xml | `<depend>nodelet</depend>` | `<depend>rclcpp_components</depend>` |


# Reference 
---
* [About Composition](https://docs.ros.org/en/foxy/Concepts/About-Composition.html)
* [Composing multiple nodes in a single process](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Composition.html)
* [ROS2 demos](https://github.com/ros2/demos/tree/foxy)
* [ROS nodelet](https://wiki.ros.org/nodelet)