> 突然發現我先寫C++再寫Python會不會太殘忍XD，不過如果看得懂C++，Python應該就不難了吧。而且筆者對Python反而比較沒有C++熟悉，大部分資料都取自官方文件，如果有錯請不吝指教。

# Python
Python的Package應該會長的類似這樣：
```bash
py_package_1/
      package.xml
      resource/py_package_1
      setup.cfg
      setup.py
      py_package_1/
```
## setup.py
和`package.xml`有一樣的資訊，請確保兩者一致。
一般填完後就不會去修改，唯一會動到的大部分是entry_points，這邊是用來描述這個package的可執行檔，例如：
```python
entry_points={
    'console_scripts': [
        'talker = py_package_1.publisher_member_function:main',
        'listener = py_package_1.subscriber_member_function:main',
    ],
},
```
這邊是用來描述這個package的可執行檔。以`talker`為例，這邊是用來描述`talker`這個可執行檔，他的執行檔是`py_package_1.publisher_member_function`，他的main function是`main`。`listener`也是同理。

## setup.cfg
這個檔案是用來讓`ros2 run`可以找到package的可執行檔，一般也是不會去修改的。

# Package Comparison

結果發現C++和Python的比重差好多，今天的文章特別短哈哈。這邊就來比較一下ROS和ROS2的差異吧。

## ROS vs. ROS2
| 說明  | ROS    | ROS2 |
| :----| :-----  | :---- |
| CMake Macro | catkin | ament|
| 創建Package指令| catkin_create_pkg <package_name> [depend1] [depend2] [depend3] | ros2 pkg create --build-type [depend1] [depend2]  <package_name> |
| Build Package指令 | catkin build | colcon build|
| Package語言獨立性 | C++和Python可以放在同一個package內 | C++和Python應獨立成兩個packages |
| Package 描述檔| CMakeLists.txt & package.xml | C++: CMakeLists.txt & package.xml; Python: setup.py & setup.cfg |
|package.xml buildtool|`<buildtool_depend>catkin</buildtool_depend>` | `<buildtool_depend>ament_cmake</buildtool_depend>` |
|package.xml depend| `<build_depend>roscpp</build_depend><build_depend>std_msgs</build_depend>` | `<depend>rclcpp</depend><depend>std_msgs</depend>` |

## ROS2 Package C++ vs. Python
|說明| C++ | Python |
| :----| :-----  | :---- |
| Package語言獨立性 | C++和Python可以放在同一個package內 | C++和Python應獨立成兩個packages |
| Package 描述檔| CMakeLists.txt & package.xml | C++: CMakeLists.txt & package.xml; Python: setup.py & setup.cfg |
| package.xml buildtool|`<buildtool_depend>ament_cmake</buildtool_depend>` | `<buildtool_depend>ament_python</buildtool_depend>` |
| package.xml depend| `<depend>rclcpp</depend><depend>std_msgs</depend>` | `<exec_depend>rclpy</exec_depend><exec_depend>std_msgs</exec_depend>` |

# Reference
---
* [ROS2 Creating a Package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
* [package.xml](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#customize-package-xml)