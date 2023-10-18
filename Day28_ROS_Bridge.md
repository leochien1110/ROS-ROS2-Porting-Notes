# ROS1 Bridge
---
目前`ROS2`最大的問題就是沒有`ROS1`的龐大生態系，因此`ROS1`和`ROS2`的橋樑就變得非常重要，`ROS1 Bridge`就是其中一個橋樑，它可以讓`ROS1`和`ROS2`的Node互相溝通。這樣在快速開發的時候，就可以先不用把`ROS`的Package轉換到`ROS2`，省去很多Migrating的時間。


## 系統需求和安裝
* Ubuntu 20.04
* ROS1 Noetic
* ROS2 Foxy

目前最適合的作業系統是Ubuntu 20.04，因為`ROS1 Noetic`和`ROS2 Foxy`都是針對Ubuntu 20.04開發的，因此在安裝的時候也會比較順利。在Ubuntu 22.04以後就不再支援`ROS`，得要趁20.04好好把握機會測試和Porting。

安裝：
```bash
sudo apt install ros-foxy-ros1-bridge
```
在使用ROS1 Bridge的時候，不能在`bashrc`中事先source ROS2的環境，因為ROS1 Bridge會自動source ROS2的環境，因此如果在`bashrc`中source ROS2的環境，會造成ROS2環境被source兩次，造成錯誤。

正確的方式是，在要使用`ROS`的Terminal中source `ROS`的環境。在需要使用`ROS2`的Terminal中source `ROS`和`ROS2`的環境。

## 訊息相容性
> 這裡訊息指的是Message和Service。
一般而言，`ROS`和`ROS2`會自動依照訊息的名稱和符合幾個條件之後，自動轉換。但是如果訊息的名稱不同，就需要客製化的轉換。

Mapping的條件如下：
1. `ROS`Package名稱的`_msgs`需對應到`ROS2`Package名稱的`_msgs`或`_interfaces`
2. 訊息的名稱必須相同
3. 訊息的型態必須相同

官方有提供[Mapping Rules](https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst#how-can-i-specify-custom-mapping-rule-for-messages)。


## 客製化訊息的轉換
對於客製化的訊息就不能用binary的方式安裝`ros1_bridge`，而是需要從source code編譯安裝，以下是編譯安裝的方式：
```bash
cd ~/ros2_ws/src
git clone https://github.com/ros2/ros1_bridge/tree/master
cd ~/ros2_ws
colcon build --symlink-install --packages-select ros1_bridge 
```

且必須要在`ROS`Package和`ROS2`Package都編譯過後，重新編譯`ros1_bridge`，客製化的才會生效。

## 使用方式
### ROS1 talker to ROS2 listener
1. 在Terminal 1中，啟動`ROS` Server
    ```bash
    source /opt/ros/noetic/setup.bash
    roscore
    ```
2. 在Terminal 2中，啟動**ros1 bridge**
    ```bash
    source /opt/ros/noetic/setup.bash
    source /opt/ros/foxy/setup.bash
    export ROS_MASTER_URI=http://localhost:11311
    ros2 run ros1_bridge dynamic_bridge
    ```
3. 在Terminal 3中，啟動`ROS1`的talker
    ```bash
    source /opt/ros/noetic/setup.bash
    rosrun roscpp_tutorials talker
    ```
    如果你沒有安裝的話可以使用以下指令安裝
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/ros/ros_tutorials.git
    cd ~/catkin_ws
    catkin build roscpp_tutorials
    source ~/catkin_ws/devel/setup.bash
    ```
    接著在執行上述的talker指令。
4. 在Terminal 4中，啟動`ROS2`的listener
    ```bash
    source /opt/ros/foxy/setup.bash
    ros2 run demo_nodes_cpp listener
    ```
這樣就可以看到`ROS1`的talker和`ROS2`的listener互相溝通了。

### Bridge 影像
除了簡單的std_msgs::String之外，ROS1 Bridge也支援影像的轉換，以下是ROS2的image publisher和ROS1的image subscriber的範例。
1. 在Terminal 1中，啟動`ROS` Server
    ```bash
    source /opt/ros/noetic/setup.bash
    roscore
    ```
2. 在Terminal 2中，啟動**ros1 bridge**
    ```bash
    source /opt/ros/noetic/setup.bash
    source /opt/ros/foxy/setup.bash
    export ROS_MASTER_URI=http://localhost:11311
    ros2 run ros1_bridge dynamic_bridge
    ```
3. 在Terminal 3中，啟動`ROS2`的image publisher
    ```bash
    source /opt/ros/foxy/setup.bash
    ros2 run image_tools cam2image
    ```
4. 在Terminal 4中，啟動`ROS1`的rqt_image_view
    ```bash
    source /opt/ros/noetic/setup.bash
    rqt_image_view /image
    ```
這樣就可以在`ROS`的`rqt_image_view`中看到`ROS2`的`cam2image`的影像了。

### Service
當然也可以Bridge Service，以下是ROS1的Service Server和ROS2的Service Client的範例。同樣地，Service的Package名稱、Service名稱、Service的型態都必須相同。

1. 在Terminal 1中，啟動`ROS` Server
    ```bash
    source /opt/ros/noetic/setup.bash
    roscore -p 11311
    ```
2. 在Terminal 2中，啟動**ros1 bridge**
    ```bash
    source /opt/ros/noetic/setup.bash
    source /opt/ros/foxy/setup.bash
    export ROS_MASTER_URI=http://localhost:11311
    ros2 run ros1_bridge dynamic_bridge
    ```
3. 在Terminal 3中，啟動`ROS1`的TwoInts Server
    ```bash
    source /opt/ros/noetic/setup.bash
    export ROS_MASTER_URI=http://localhost:11311
    rosrun roscpp_tutorials add_two_ints_server
    ```
4. 在Terminal 4中，啟動`ROS2`的TwoInts Client
    ```bash
    source /opt/ros/foxy/setup.bash
    ros2 run demo_nodes_cpp add_two_ints_client
    ```


### 進階Bridge
其中`ros1_bridge`有提供更進階的使用方式，可以透過編輯`bridge.yaml`，選擇要bridge的topics和services。確切的範例可以參考這個[連結](https://github.com/ros2/ros1_bridge/tree/master#example-4-bridge-only-selected-topics-and-services)。

在`ROS2`中引入的QoS(Quaility of Service)也可以透過`parameter_bridge`來設定。確切的範例可以參考這個[連結](https://github.com/ros2/ros1_bridge/tree/master#parametrizing-quality-of-service)。




# Reference 
---
* [ros1_bridge](https://github.com/ros2/ros1_bridge/tree/master)
* [ros1 bridge mapping rule](https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst#how-can-i-specify-custom-mapping-rule-for-messages)
