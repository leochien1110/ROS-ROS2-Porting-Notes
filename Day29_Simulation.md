# Simulation
---
模擬器在機器人領域非常重要，可以把昂貴的機器人放在虛擬世界中，進行測試，也可以在虛擬世界中進行演算法的開發，減少實體機器人的損耗。還有不同感測器的相對位置和遮擋也都可以在模擬器中進行測試，減少在實體機器人上的調整時間。

模擬器有很多種，有些事針對特定情境的模擬器像是自駕車或是無人機，而有些則是通用型的模擬器，可以用來模擬各種不同的機器人。而這些給機器人用的模擬器大多也支援**ROS**，不論事原生支援還是透過**ROS Bridge**的方式來支援，都可以在**ROS**中使用。

這裡就介紹幾個比較有名的模擬器，不過具體使用方式超出太多就不講解了，僅介紹一下各自的特色和使用情境。基本上這邊每一個模擬器都可以出一篇鐵人


## Gazebo(Ignition simulator)
![gazebo preview](https://repository-images.githubusercontent.com/255865265/c843f697-2479-4f26-84e8-37d4c25897c4)
**Gazebo**是**ROS**本身自帶的模擬器，可以在**ROS**中直接使用，也可以單獨使用。**Gazebo**的模擬器是基於**OGRE**的引擎，是一個很古老的開源的3D圖形引擎。

其中裡面有很多官方預設的模型，也有很多第三方的模型，可以直接使用，也可以自己建立模型。**Gazebo**主要是由**SDF**來描述模型，**SDF**是**XML**的格式，可以用文字編輯器來編輯，也可以用**GUI**的方式來編輯，**Gazebo**也有提供**GUI**的編輯器，可以直接在**Gazebo**中編輯模型。

由於Gazebo比較簡單，這邊會介紹比較詳細一些。

### 安裝
這邊提供[Ubuntu 20.04 + ROS Foxy](https://gazebosim.org/docs/garden/install_ubuntu)的安裝連結，其他版本請參考Ignition Gazebo的[官方文件](https://gazebosim.org/docs/latest/ros_installation)。

### 簡單範例
啟動Gazebo world `visualize_lidar.sdf`:
```bash
ign gazebo -v 4 -r visualize_lidar.sdf
```
應該可以看到下面的視窗，有遊樂園和一個藍色的車子。
![gazebo_lidar](https://docs.ros.org/en/foxy/_images/gazebo_diff_drive.png)

檢查Ignition Gazebo的Topics:
```bash
ign topic -l
```

> ⚠️ 注意：這邊的`ign topic`和`ros2 topic`不同，`ign topic`是Ignition Gazebo的指令，`ros2 topic`是ROS2的指令。

要把`ROS2`的Topic和`Ignition Gazebo`的Topic連接起來，可以使用`ros_ign_bridge`的package。首先安裝：
```bash
sudo apt-get install ros-foxy-ros-ign-bridge
```
接著啟動`ros_ign_bridge`：
```bash
source /opt/ros/foxy/setup.bash
ros2 run ros_ign_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
```
更多`ros_ign_bridge`的使用方式可以參考[官方文件](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge)。

接著我們可以控制車子的速度，有以下幾種方法
1. 使用`ros2 topic pub`的方式
    ```bash
    ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/Twist  "linear: { x: 0.1 }"
    ```
2. 使用`teleop_twist_keyboard`的方式，記得要remap `/cmd_vel`到`/model/vehicle_blue/cmd_vel`
    ```bash
    # 如果還沒安裝的話
    sudo apt-get install ros-foxy-teleop-twist-keyboard

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/model/vehicle_blue/cmd_vel
    ```

不只指令可以從gazebo bridge到`ROS2`，Sensor的資料也可以。上述的範例中帶有LiDAR，可以將LiDAR的資料bridge到`ROS2`，並且使用`rviz2`來顯示。首先開啟lidar bridge：
```bash
ros2 run ros_ign_bridge parameter_bridge /lidar2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar2:=/laser_scan
```

接著開啟`rviz2`：
```bash
rviz2
```
Fixed Frame選擇`vehicle_blue/lidar_link/gpu_lidar`。
`Add`->`By Topic`->`/laser_scan/LaserScan`，就可以看到LiDAR的資料了。
![rviz2_lidar_scan](https://docs.ros.org/en/foxy/_images/rviz2.png)


## Webots
![webots](https://upload.wikimedia.org/wikipedia/commons/8/82/Wbinterface.png)
這也是`ROS2`官方建議的模擬器，也是一個開源的模擬器，可以用來模擬各種不同的機器人，也可以用來模擬各種不同的感測器。操作相較於`Gazebo`更直覺，模擬器的引擎也比較新，看起來會比Gazebo更真實。


### 安裝
可以選擇使用Distributio安裝或Source Code安裝。這邊使用Ubuntu Distribution安裝，其他方式、作業系統請參考[官網說明](https://docs.ros.org/en/foxy/Tutorials/Advanced/Simulators/Webots/Simulation-Webots.html)。
    
```bash
sudo apt-get install ros-foxy-webots-ros2
```

要測試是否安裝成功，啟動`webots_ros2_universal_robot`
```bash
source /opt/ros/foxy/setup.bash
export WEBOTS_HOME=/usr/local/webots    # 設定Webots的安裝路徑
ros2 launch webots_ros2_universal_robot multirobot_launch.py
```

要測試簡單範例的話可以參考[這邊的教學](https://docs.ros.org/en/foxy/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html)，做出一個圓柱體小車的機器人。


## Carla
[![Carla](https://img.youtube.com/vi/AZhzZy57XeU/sddefault.jpg)](https://www.youtube.com/watch?v=AZhzZy57XeU)


Carla模擬器是一種開放源碼的自動駕駛模擬器，是基於Unreal Engine所開發的。它可以生成逼真的城市環境和交通情況。Carla模擬器可以用於開發和測試自動駕駛系統，以及進行交通研究。

Carla模擬器的功能包括：

* 逼真的城市環境，包括道路、建築物、交通標誌和交通燈。
* 多種交通情況，包括擁堵、道路施工和交通事故。
* 多種車輛模型，包括汽車、卡車和公共汽車。
* 多種傳感器模型，包括攝像頭、雷達和激光雷達。
* 模擬器 API，允許開發人員創建自己的自動駕駛系統。

Carla模擬器已經被廣泛用於自動駕駛研究。它被用來開發和測試自動駕駛系統的各種技術，包括感知、規劃和控制。Carla模擬器還被用來進行交通研究，例如研究交通流量和交通事故。

Carla模擬器是一種強大的工具，可用於開發和測試自動駕駛系統。它提供了一個逼真的環境來測試自動駕駛系統的性能，並且可以用來進行交通研究。

以下是Carla模擬器的一些具體應用：

* 自動駕駛系統的開發和測試
* 交通研究
* 娛樂和遊戲
Carla模擬器是一種免費的開源工具，可在[GitHub](https://github.com/carla-simulator/carla)上找到，並且可以在[官方文件](https://carla.readthedocs.io/en/latest/)上找到更多教學和介紹。


## AWSIM
![awsim](https://github.com/tier4/AWSIM/blob/main/README_img/AWSIM.png?raw=true)

**AWSIM**是由**Tier IV**所開發的模擬器，主要是針對自駕車的模擬器，可以模擬自駕車在不同的環境中的行為，也可以模擬不同的感測器在不同的環境中的表現。**AWSIM**是基於**Unreal Engine**所開發的，**Unreal Engine**是一個商業的3D圖形引擎，可以用來開發遊戲，也可以用來開發模擬器。

**Tier IV**本身是做車用相機的，不過最有名的還是全開源的自駕車軟體**Autoware**。**Autoware**是一個完整的自駕車軟體，包含了感測器的蒐集、感測器的融合、路徑規劃、控制等等，**AWSIM**就是為了**Autoware**所開發的模擬器，可以用來測試**Autoware**的各種功能。而且**Autoware**本身就是建立在`ROS2`之上，而**AWSIM**又是專門給**Autoware**使用的，因此強烈推薦要玩自駕車的人可以嘗試。

不過比較麻煩的是，目前**Autoware**是採用最新的`ROS2 Humble`，而最新的Nvidia AGX Orin邊緣運算電腦還還在Ubuntu 20.04僅能安裝`ROS2 Foxy`，所以得要用Docker。


## ISAAC Sim
![isaac sim](https://docs.omniverse.nvidia.com/isaacsim/latest/_images/isaac_main_intro_2.png)

**ISAAC Sim**是由**Nvidia**所開發的模擬器，基於**Omniver**渲染器所開發的機器人互動平台。這個模擬器最大的優勢就是善用了**Nvidia**官方的Ray Tracing功能，相較於其他的模擬器，**ISAAC Sim**的畫面更加逼真，也更加真實。

其中一個很強大的功能就是Gym的支援，可以同時模擬上百組機器人用來做強化學習的訓練。另外Semantics的支援也很好，可以用來做Semantic Segmentation標註的訓練。不過因為這些過於豐富的功能和可以寫程式擴充的特性，**ISAAC Sim**的學習曲線也是這些模擬器中最高的。


# Reference
---
* [Setting up a robot simulation (Ignition Gazebo)](https://docs.ros.org/en/foxy/Tutorials/Advanced/Simulators/Ignition/Ignition.html)
* [Setting up a robot simulation (Webot)](https://docs.ros.org/en/foxy/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html)
* [ROS Gazebo Installation](https://gazebosim.org/docs/latest/ros_installation)
* [Webots](https://en.wikipedia.org/wiki/Webots)
* [Carla](https://carla.readthedocs.io/en/latest/)
* [AWSIM](https://github.com/tier4/AWSIM)
* [ISAAC Sim](https://developer.nvidia.com/isaac-sim)
* [ISAAC Sim Document](https://www.google.com/search?q=isaac+sim&oq=isaac+sim&gs_lcrp=EgZjaHJvbWUyCwgAEEUYJxg5GIoFMgYIARBFGDsyBggCEEUYQDIJCAMQIxgnGIoFMgcIBBAAGIAEMgYIBRBFGDwyBggGEEUYPDIGCAcQRRg80gEINDAwOWowajeoAgCwAgA&sourceid=chrome&ie=UTF-8)