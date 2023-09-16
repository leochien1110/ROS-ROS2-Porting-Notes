`ROS2`中Package比較大的改動就是C++和Python無法在混用。應該是原則上沒辦法，但爬文還是可以找到workaround，大意上就是利用`CMakeLists.txt`去Compile Python，不過不是正統的方式這邊就不多說明。

# C++
使用Colcon 創建C++ ROS2 Package：
```
ros2 pkg create --build-type ament_cmake <package_name>
```

如同昨天Workspace提到，C++的Package應該會長的類似這樣：
```
cpp_package_1/
  CMakeLists.txt
  include/cpp_package_1/
  package.xml
  src/
```

`include`及`src`是用來存放程式主要source及header file的地方。

`CMakeLists.txt`就跟一般C++一樣，不過裡面多了`ament`的一些ROS2相關的Macro，可以更方便的找到路徑底下的ROS2 Packages和安裝binary到目標位置，方便管理。

`package.xml` 則是列出了所以ros dependencies的相依和執行層級，必須同時出現在cpp_package內。

## CMakeLists.txt
在`ROS`時代，用的是`catkin`，而在`ROS2`時代，則是用`ament`，兩者的差異如下：

### catkin
1. 找到`catkin`的package，並且找到`roscpp`、`rospy`、`std_msgs`的相依。這個步驟目的是找到相對應的ROS header和library:
    ```cmake
    find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    )

    # Non-ROS
    find_package(PCL REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(OpenCV REQUIRED)
    ```
2. 接著用`catkin_package`的Macro來描述這個package的相依和執行層級，用來建構、生成package config和CMake的訊息：
    ```cmake
    catkin_package(
      CATKIN_DEPENDS roscpp rospy std_msgs
      INCLUDE_DIRS include
      LIBRARIES cpp_package_1
      DEPENDS system_lib
    )
    ```
    * `CATKIN_DEPENDS`：列出這個package的相依，這邊是`roscpp`、`rospy`、`std_msgs`。
    * `INCLUDE_DIRS`：列出這個package的include路徑，也就是`src`內的header file，目的是給其他的package使用，這邊是`include`。
    * `LIBRARIES`：列出這個package的library，這邊是`cpp_package_1`，可以讓其他的package使用。
    * `DEPENDS`：列出非ROS的相依，這邊是`system_lib`。

3. 接著是`include_directories`，這邊是用來指定include的路徑給compiler，若有非ROS的library，也都要在這邊指定路徑給compiler：
    ```cmake
    include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
    )
    ```
    首先要先指定`include`的路徑，這邊是`include`，然後是ROS的library，這邊是`${catkin_INCLUDE_DIRS}`，最後隨便舉了幾個常用的非ROS的libraries。

    `link_directories`則是用來指定library的路徑給compiler，一般不建議使用，因為find_package會自動幫你找到library的路徑。

4. `add_executable`，這邊是用來指定執行檔的名稱及source code的路徑：
    ```cmake
    add_executable(${PROJECT_NAME}_node src/cpp_package_1_node.cpp)
    ```
    如果有要build非可執行檔的library(沒有main function)以及`pluginlib`，則是用`add_library`：
    ```cmake
    add_library(helper src/cpp_without_main.cpp)
    ```

5. 最後是`target_link_libraries`，這邊是用來連接library和執行檔：
    ```cmake
    target_link_libraries(${PROJECT_NAME}_node
      ${catkin_LIBRARIES} 
      ${PCL_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${Eigen3_LIBRARIES}
      helper
    )
    ```
    這邊的`helper`是上面`add_library`的library。

6. 幾本上這樣可執行檔就可以建好在`build/`，如果要安裝到`install/`，則是在CMakeLists.txt內用`install`：
    ```cmake
    install(TARGETS ${PROJECT_NAME}_node
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )
    ```
    這邊的`CATKIN_PACKAGE_BIN_DESTINATION`以及`CATKIN_PACKAGE_LIB_DESTINATION`是`catkin`的Macro，會自動幫你找到`bin`和`library`的路徑。

    可以連同launch, plugins, config等等一起安裝：
    ```cmake
    install(DIRECTORY launch
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
    )
    install(DIRECTORY config
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
      FILES_MATCHING PATTERN "*.yaml" PATTERN "*.cfg"
    )
    install(DIRECTORY plugins
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins
      FILES_MATCHING PATTERN "*.xml"
    )
    ```
    

其實還有其他的類型的Macro，像是`add_service_files`、`add_action_files`、`add_message_files`、`add_dependencies`等等，不過這些都是用來描述ROS的相關訊息，所以會在`package.xml`內描述，這邊就不多說明。
* Message
* Service
* Action
* Dynamic Reconfigure


### ament
1. 找到`ament`的package，並且找到`rclcpp`、`std_msgs`的相依。這個步驟目的是找到相對應的ROS header和library:
    ```cmake
    # ROS
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

    # Non-ROS
    find_package(PCL REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(OpenCV REQUIRED)
    ```
    > :warning: 注意：`ament_cmake`是`ament`的一個package，所以要先找到`ament_cmake`才能找到其他的package。

    > 另外在`ROS2`, python一般來說是獨立成一個package，所以不會出現在`ament_cmake`的`find_package`內。
    > 不過如果有需要，也可以用`find_package(ament_cmake_python REQUIRED)`來找到python的package。

2. `include_directory`是用來指定include的路徑給compiler：
    ```cmake
    include_directories(
        include
        ${PCL_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS})
    ```
    可以明顯的看到，`ament`的`include_directory`沒有ROS的library，因為`ament`會自動幫你找到ROS的library。
    
    然而且`ament`也不會幫你找到非ROS的library，所以要自己指定路徑給compiler。

3. 接著是`add_executable`，用來指定執行檔的名稱及source code的路徑：
    ```cmake
    add_executable(${PROJECT_NAME}_node src/cpp_package_1_node.cpp)
    ```
    如果有要build非可執行檔的library(沒有main function)，則是用`add_library`：
    ```cmake
    add_library(helper src/cpp_without_main.cpp)
    ```

    `plugin`在ROS2有做更動，從原本幾乎都在`package.xml`內描述，改成在`CMakeLists.txt`內描述。這邊先列出`plugin`的cmake寫法，之後會再詳細說明：
    ```cmake
    pluginlib_export_plugin_description_file(cpp_package_1 plugin.xml)
    ```
4. `target_link_libraries`用來連接非ROS的library和執行檔：
    ```cmake
    target_link_libraries(${PROJECT_NAME}_node
        helper
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${Eigen3_LIBRARIES}
    )
    ```
5. 最後ROS2的改動，`ament_target_dependencies`，用來連接ROS2的library和執行檔：
    ```cmake
    ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
        std_msgs
    )
    ```

6. `install`的Macro則是在`ros2 pkg create --build-type ament_cmake <package_name>`時自動產生：
   ```cmake
   install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
   )
   ```

7. 最後是`ament_package`用來安裝`package.xml`、將package註冊到`ament index`的系統中，以及安裝CMake config已便其他的package可以找到：
    ```cmake
    ament_package()
    ```

    這個指令負責做最後的配置，所以應該放在CMakeLists.txt的最後面。

    
8. 這邊補充一個官方沒有提到symlink的安裝`ament_cmake_symlink_install_files`:
    ```cmake
    ament_cmake_symlink_install_files(FILES
        launch
        config
        DESTINATION share/${PROJECT_NAME})
    ```
    這個指令可以將`launch`和`config`等資料夾建立symlink到`share/${PROJECT_NAME}`，這樣就可以在執行檔的路徑下找到`launch`和`config`的資料夾，更新launch參數時也不需要重新build。
    

**PCL補充**
---
全名Point Cloud Library，是一個開源的3D點雲處理函式庫，他的CMakeLists.txt比較龜毛，所以要特別注意。`find_package`後，一定還要宣告下列三行才能正常使用：
```cmake
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
```

## package.xml
`package.xml`是C++和Python都會用到的描述檔，用來描述這個package的相依。
C++和Python的`package.xml`差異不大，這邊就直接用C++的`package.xml`來說明。
### C++
```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<depend>rclcpp</depend>
<depend>std_msgs</depend>
...
<export>
    <build_type>ament_cmake</build_type>
</export>
```
* `buildtool_depend`：這邊是用來描述這個package的build tool，這邊是`ament_cmake`。
* `depend`：這邊是用來描述這個package的ROS相依，這邊是`rclcpp`、`std_msgs`。
* `export`：這邊是用來描述這個package的build type，這邊是`ament_cmake`。

### Python
```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
...
<export>
    <build_type>ament_python</build_type>
</export>
```
沒錯，Python的`package.xml`只有`export`和`exec_depend`，`buildtool_depend`則是因為Python會靠setup.py和setup.cfg而不再需要。

發現有點講太細了，把Python Package的部分獨立出來，明天再來說明。

# Reference
---
* [ROS2 Creating a Package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
* [Ament Cmake Documentation](https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html)
* [package.xml](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#customize-package-xml)