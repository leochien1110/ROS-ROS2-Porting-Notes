
# ROS2 Bag
---
Bag是**ROS**用來儲存Topic的格式，可以再錄製完後回放，非常適合用來做測試和資料集的收集。可以看到很多Github上定位的演算法的repo都是用Bag來做測試的。主要有幾個好處：
* 對於測試環境較嚴苛，譬如沒有螢幕只能遠端的情況下，可以先錄製好資料，再回去重播。
* 相較於分開收集PCAP(LiDAR Packet等), MP4(影像串流)等各種格式資料，ROSBag可以全部整合在一起並且標上時間戳記，方便後續的資料處理。
* 有很多相應的工具可以用，譬如`rqt_bag`，`rosbag`等等，可以快進、跳轉，甚至裁切等等。

在`ROS`中是使用自定義的binary格式來儲存，所以僅能用`ROS`相關的工具開啟(rqt_bag, rosbag, plotjuggler等等)。`ROS2`則改用`sqlite3`來儲存，所以可以用一般的工具開啟，理論上啦XD

## 安裝
如果你沒有安裝的話，可以手動安裝ros2 bag的套件：
```bash
sudo apt-get install ros-foxy-ros2bag \
                     ros-foxy-rosbag2-converter-default-plugins \
                     ros-foxy-rosbag2-storage-default-plugins
```

## 基本操作
因為`foxy`版本較舊，很多新版的功能像是`burst`, `convert`, `list`, `reindex`等操作還沒有引入。不過因為這些功能屬於比較進階的操作，下面三個功能基本上就夠用了：
* `ros2 bag info <bag_name>`: 顯示bag的資訊，包含topic, topic type, message count, duration, size等等。
* `ros2 bag play <bag_name>`: 播放bag，可以指定速度、只發布特定的topic、自動重播、remap等等。
* `ros2 bag record <topic1> <topic2> ...`: 錄製bag，可以指定要錄製的topic、壓縮格式、最大檔案大小(超過會分割)等等。錄製時間則是在`foxy-future`中，並沒有在`foxy`中實作。

使用上和前面介紹過的指令一樣，可以用`ros2 bag --help`、`ros2 bag info --help`等等來查看詳細的使用方式。

由於是採用`sqlite3`的儲存方式，不像`ROS`的bag一樣只有單一檔案，`ROS2`的bag會是一個資料夾，裡面會有一個`metadata.yaml`和一個`db3`檔案，`db3`檔案就是儲存資料的地方，`metadata.yaml`則是儲存bag的資訊，包含topic, topic type, message count, duration, size等等。不過上述的指令中`bag_name`可以是資料夾或是底下的`db3`檔案，都可以正常運作，不過資料夾的話會顯示整個資料夾的資訊，而`db3`檔案則只會顯示單一檔案的資訊。

要注意的是，`ros2 bag`並不會知道會錄多久，也不是事先檢查硬碟空間，這部分要注意一下，不然就會像筆者一樣發生悲劇，出去繞一圈後硬碟爆掉，只錄了一半的資料QQ 

另外還要注意硬碟的寫入速度，像是比較小台的樹莓派或是Jetson Nano等等，因為使用SD卡或是eMMC，寫入速度都不太快，在寫入大量資料時，硬碟可能會跟不上，造成資料遺失。

## turtlesim 範例
這邊使用`turtlesim`來做範例
1. 首先先開啟`turtlesim`和鍵盤控制的節點：
    ```bash
    ros2 run turtlesim turtlesim_node
    ```
    在另一個terminal開啟鍵盤控制的節點：
    ```bash
    ros2 run turtlesim turtle_teleop_key
    ```

2. 接著先創建資料夾給bag，這樣才不會太亂。筆者習慣放在`workspace`底下，所以先`cd`到`workspace`底下。開啟第三個terminal：
    ```bash
    cd ~/ros2_ws
    mkdir bag && cd bag
    ```

3. 列出Topic
    ```bash
    ros2 topic list
    # 會看到
    /parameter_events
    /rosout
    /turtle1/cmd_vel    # 這是我們鍵盤控制的topic
    /turtle1/color_sensor
    /turtle1/pose
    ```
    可以把我們控制的topic印出來
    ```bash
    ros2 topic echo /turtle1/cmd_vel
    # 會看到類似這樣
    linear:
        x: 2.0
        y: 0.0
        z: 0.0
    angular:
        x: 0.0
        y: 0.0
        z: 0.0
    ---
    ```
4. 將我們的鍵盤控制錄下來
    ```bash
    ros2 bag record -o turtle_test /turtle1/cmd_vel
    # 會看到類似下面資訊，表示錄製成功
    [INFO] [rosbag2_storage]: Opened database 'turtle_test'.
    [INFO] [rosbag2_transport]: Listening for topics...
    [INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/cmd_vel'
    [INFO] [rosbag2_transport]: All requested topics are subscribed. Stopping discovery...
    ```
    這時候就可以開始控制`turtlesim`，控制一段時間後，按下`Ctrl+C`結束錄製。會到看到資料夾底下多了一個`turtle_test`的資料夾，就是剛剛錄的bag了。這裡`-o`可以指定bag的名稱，如果不指定的話，預設會是用timestamp命名,`rosbag2_YYYY_MM_DD_HH_MM_SS`。

    如果要錄製多個Topics的話可以測試下面的指令：
    ```bash
    ros2 bag record /turtle1/cmd_vel /turtle1/pose # record 2 topics
    ros2 bag record -a # record all topics
    ```
5. 查看bag資訊
    ```bash
    ros2 bag info turtle_test
    # 會看到類似下面資訊
    Files:             turtle_test.db3
    Bag size:          228.5 KiB
    Storage id:        sqlite3
    Duration:          48.47s
    Start:             Oct 11 2019 06:09:09.12 (1570799349.12)
    End                Oct 11 2019 06:09:57.60 (1570799397.60)
    Messages:          3013
    Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 9 | Serialization Format: cdr
                    Topic: /turtle1/pose | Type: turtlesim/msg/Pose | Count: 3004 | Serialization Format: cdr
    ```
6. 回放bag，記得保持`turtlesim`開啟
    ```bash
    ros2 bag play turtle_test
    ```
    就可以看到烏龜開始模仿我們剛剛的移動方式了！


# Bag API
---
其實上述的`ros2 bag`是官方包好的功能，底層是用`rosbag2`來實作的，所以如果要自己寫bag的話，可以直接用`rosbag2`的API來實作。這邊就簡單介紹一下`rosbag2`的API，如果要更詳細的介紹可以參考[官方文件](https://github.com/ros2/rosbag2)。

雖然官方有提供錄製的API範例，但研究過後發現沒什麼必要，基本上`ros2 bag record`就涵蓋了大部分的功能，所以這邊就不介紹錄製的API了，只介紹讀取的API。

讀取的部分之所以有趣是因為可以客製化轉檔，像是把bag轉成我們要的dataset丟給機器學習模型訓練等等。這邊就簡單介紹一下讀取的API。不過由於`foxy`官方文件沒有提供讀取的範例，rosbag_cpp的API在`galactic`之後版本也有改變，使用上有些許差異，所以這邊就分成兩個版本來介紹。

1. 創建一個`bag_reading_cpp`的package
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake --license Apache-2.0 bag_reading_cpp --dependencies rclcpp rosbag2_cpp turtlesim
    ```
2. 在`bag_reading_cpp/src`下創建`simple_bag_reader.cpp`
    Foxy
    ```cpp
    #include <iostream>

    #include <rclcpp/rclcpp.hpp>
    #include <rclcpp/serialization.hpp>
    #include <rosbag2_cpp/readers/sequential_reader.hpp>
    #include <rosbag2_cpp/storage_options.hpp>
    #include <rosbag2_cpp/converter_options.hpp>

    #include <turtlesim/msg/pose.hpp>

    int main(int argc, char * argv[])
    {
        if (argc < 2)
        {
            std::cerr << "Usage: " << argv[0] << " <bag_dir>" << std::endl;
            return 1;
        }

        std::string bag_dir = argv[1];

        rosbag2_cpp::StorageOptions storage_options;
        storage_options.uri = bag_dir;
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        rosbag2_cpp::readers::SequentialReader reader;
        reader.open(storage_options, converter_options);

        std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg;
        auto serialization = rclcpp::Serialization<turtlesim::msg::Pose>();
        while (reader.has_next())
        {
            msg = reader.read_next();

            // print msg if available
            if (msg->topic_name == "/turtle1/pose")
            {
                turtlesim::msg::Pose msg_pose;
                rcutils_uint8_array_t raw_data = *msg->serialized_data;
                auto serialized_msg = rclcpp::SerializedMessage(raw_data);
                serialization.deserialize_message(&serialized_msg, &msg_pose);

                std::cerr << "turtle1 pose: (" << msg_pose.x << ", " << msg_pose.y << ")" << std::endl;

            }

        }
        return 0;
    }
    ```

    Galactic or later(including foxy-future), `rosbag2_cpp`的`open`被簡化，只需要讀取bag的路徑即可，不需要再指定`storage_id`和`converter_options`。而這個[ticket](https://github.com/ros2/rosbag2/pull/457)也將`rosbag2_storage::SerializedBagMessage`由`rclcpp::SerializedMessage`取代而簡化：
    ```cpp
    #include <iostream>

    #include <rclcpp/rclcpp.hpp>
    #include <rclcpp/serialization.hpp>
    #include <rosbag2_cpp/reader.hpp>
    #include <turtlesim/msg/pose.hpp>

    int main(int argc, char * argv[])
    {
        if (argc < 2)
        {
            std::cerr << "Usage: " << argv[0] << " <bag_dir>" << std::endl;
            return 1;
        }

        std::string bag_dir = argv[1];

        rosbag2_cpp::Reader reader;
        reader.open(bag_dir);

        rclcpp::Serialization<turtlesim::msg::Pose> serialization;

        while (reader.has_next())
        {
            rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_.read_next();
            if (msg->topic_name == "/turtle1/pose")
            {
                rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
                turtlesim::msg::Pose::SharedPtr ros_msg = std::make_shared<turtlesim::msg::Pose>();

                serialization.deserialize_message(&serialized_msg, ros_msg.get());

                std::cerr << "turtle1 pose: (" << ros_msg->x << ", " << ros_msg->y << ")" << std::endl;

            }

        }
        return 0;
    }
    ```

3. 編輯`CMakeLists.txt`，加入執行檔和安裝位置
    ```cmake
    add_executable(simple_bag_reader src/simple_bag_reader.cpp)
    ament_target_dependencies(simple_bag_reader rclcpp rosbag2_cpp turtlesim)

    install(TARGETS
    simple_bag_reader
    DESTINATION lib/${PROJECT_NAME}
    )
    ```

4. Build
    ```bash
    cd ~/ros2_ws
    rosdep install -i --from-path src --rosdistro foxy -y
    colcon build --packages-select bag_reading_cpp
    source install/setup.bash
    ```

5. 印出bag內`turtlesim`的pose
    ```bash
    ros2 run bag_reading_cpp simple_bag_reader <bag_dir>
    ```

# ROS vs. ROS2
---

| 說明 | ROS | ROS2 |
| :--- | :--- | :--- |
| 儲存格式 | 客製化binary | sqlite3 |
| 儲存方式 | 單一檔案 | 資料夾 |
| rqt_bag | 有 | `foxy`以前沒有 |
| 指令 | `rosbag` | `ros2 bag` |
| API | rosbag | rosbag2 |
| API使用 | 完全都 | 不一樣 |

## Foxy vs. Later distros
`ROS2`官方migrate到`foxy`時大致已經成型，唯獨`rosbag2`還東缺西缺，要到`galactic`之後功能才比較完整。對於在Ubuntu 20.04(`foxy`，已經EOL)的朋友，可以考慮用`rosbag2`中的`foxy-future` branch，裡面有把新功能merge進去，但是還沒有release，所以要自己build。使用Ubuntu 22.04(`humble`)的朋友就直接用就可以了。

以`foxy`為例，想使用`rosbag2`新功能的話
```bash
cd ~/ros2_ws/src
git clone -b foxy-future https://github.com/ros2/rosbag2.git
cd ..
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --packages-up-to rosbag2
source install/setup.bash
```

# Reference
---
* [Recording a bag from a node(C++)](https://docs.ros.org/en/rolling/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-CPP.html)
* [Reading from a bag file(C++)](https://docs.ros.org/en/rolling/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html)
* [ROS2 bag之API](https://blog.csdn.net/lovely_yoshino/article/details/129147825)
* [rosbag2](https://github.com/ros2/rosbag2/tree/rolling)
* [rclcpp](https://github.com/ros2/rclcpp/tree/rolling)
* [ROS CLI](http://wiki.ros.org/rosbag/Commandline)
* [rosbag](http://wiki.ros.org/rosbag)