> 今天終於要來講**ROS**的最後一個通訊方式**Parameter**，雖然有些人認為他不是通訊方式，不過藉由rqt_reconfigure是可以在Runtime中設定參數，所以我還是把他歸類在通訊方式中。

# Parameters
---
**ROS**中第四種通訊方式是**Parameter**。不過和前面三個介紹的方式不一樣的是，Parameter不是用來讓Node之間交換訊息的，而是讓使用者在Runtime中設定設定一些Node的參數。

Parameter的優點是可以在Runtime中設定，不需要重新編譯程式，可以讓C++程式更有彈性。這樣的特性也可以在launch file中設定參數，讓程式更有彈性。譬如一個相機driver可以用來instantiate多個相機，但是每個相機的參數都不一樣，這時候就可以調用Parameter來開啟多個不同namespace的相機，但都用同一個driver node。

一個parameter內會有三個部分：
* Key - string type
* Value - basic type(bool, int64, float64, string, byte[], bool[], int64[], float64[] or string[])
* Descriptor - any further information about the parameter(default: None)

>:warning: 由於`ROS2`每個node間已經獨立，不再有Master Server，意味著不再有Parameter Server可以handle參數了。Parameter必須依賴Node，在Node內被宣告後才能在外部使用，像是`ros2 run`和`ros2 launch`。


## Command line
`ROS`的參數設定是用`rosparam`，`ROS2`則是用`ros2 param`。
不過基本功能都有migrate過來：
* `ros2 param set` - 設定參數
* `ros2 param get` - 取得參數
* `ros2 param list` - 列出所有參數
* `ros2 param dump` - 將參數存成YAML檔
* `ros2 param load` - 從YAML檔讀取參數
* `ros2 param delete` - 刪除參數
* `ros2 param describe` - (新功能)取得參數的描述

要注意的是，因為`ROS2`不再有master server，所以無法直接空跑`ros2 param`來設定參數，node必須要先跑起來。拿Day6的`hello_world`來說明:
```bash
ros2 run beginner_tutorials_py hello_world_py

ros2 param set /hello_world_py score 100
```

這時再開啟一個terminal來查看參數：
```bash
ros2 param list
```
再將參數顯示出來：
```bash
ros2 param get /hello_world_py score
```

另外，parameter也可以隨著`ros2 run`一起執行，只要在後面加上`--ros-args -p param_name:=param_value`，模板如下
```bash
ros2 run package_name executable_name --ros-args -p param_name:=param_value
```
也可以載入YAML檔(下一個小節會介紹)：
```bash
ros2 run package_name executable_name --ros-args --params-file my_params.yaml
```


## YAML file
與`ROS`最大的差別就是`ROS2`需要指名namespace和node_name，還需要`ros__parameters`來指名以下是需要設定的參數。

舉例來說，`ROS`的參數檔長這樣：
```yaml
lidar_name: foo
lidar_id: 10
ports: [11312, 11311, 21311]
debug: true
```
可以看到非常的簡潔，但是`ROS2`的參數檔長這樣：
```yaml
/lidar_ns:
  lidar_node_name:
    ros__parameters:
      lidar_name: foo
      id: 10
imu:
  ros__parameters:
    ports: [2438, 2439, 2440]
/**:
  ros__parameters:
    debug: true
```
* `lidar_name`和`id`就是`/lidar_ns`底下`lidar_node_name`這個node的參數。
* `ports`是`/imu`這個node的參數。
* `/**`是所有node的參數。

## rqt_reconfigure
`rqt_reconfigure`是一個GUI介面，可以用來設定參數。在`ROS`中是用`rosrun rqt_reconfigure rqt_reconfigure`，在`ROS2`中是用`ros2 run rqt_reconfigure rqt_reconfigure`。

這個GUI有提供slider, checkbox, value box等基本功能。不過`ROS2`的`rqt_reconfigure`目前還沒有`ROS`的下拉選單，如果有需要用到的話得要自己手刻qt的code。

這個GUI在做調整時非常實用，譬如影像的jpeg壓縮等級，可以用這個GUI調整壓縮幅度並同時觀看影像的變化。亦或是校正兩個Pointclouds時，可以即時顯示校正的結果。

然而，再一次的由於`ROS2`不再有master server，所以這部分得要等node跑起來才能使用，不再像`ROS`可以直接寫一個parameter config file然後寄宿在parameter server(master)上。所以要限制parameter的範圍等都要寫在node內，node會看起來複雜一些。



# ROS通訊方式小結
---
**ROS**的主要通訊方式在這裡告一個段落，總共有四種通訊方式，分別是**Topic**、**Service**、**Action**和**Parameter**。
| 通訊方式 | 用途 | 優點 | 缺點 |
| :-------- | :-------- | :-------- | :-------- |
| Topic | 用來傳送stream訊息，像是sensor data | 可以用來傳送大量資料 | 傳遞較複雜任務時，兩邊node速度差太多會造成queue 滿溢 |
| Service | 用來傳送任務導向的訊息，像是全域路徑規劃或Health Check | 可以用來傳送任務訊息 | 需等待Server回應，期間沒有任何動作 |
| Action | 用來傳送動態任務導向的訊息，像是避障路徑規劃 | 可以用來傳送動態任務訊息不會被阻塞 | 寫法較複雜 |
| Parameter | 用來傳送Node的參數 | 1. 可以在Runtime中設定參數 <br>2. 和在launch檔instantiate不同namespace的node | 只能使用基本型態和YAML的參數，無法客製化 |



# ROS vs. ROS2
---
| 說明 | ROS | ROS2 |
| --- | --- | --- |
| 參數CLI | `rosparam` | `ros2 param` |
| 依附 | Parameter Server(Master) | Node |
| run node | `rosrun package_name executable_name _param_name:=param_value` | `ros2 run package_name executable_name --ros-args -p param_name:=param_value` |
| 參數檔格式 | 直接宣告參數 | 需要指名node_name和namespace(若有)+`ros__parameters:` |
| rqt_reconfigure | 獨立parameter config file | 需要node跑起來才能使用 |




# Reference
---
* [Undertanding ROS2 Parameters](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
* [About parameters in ROS 2](https://docs.ros.org/en/foxy/Concepts/About-ROS-2-Parameters.html)
* [Migrating YAML parameter files from ROS 1 to ROS 2](https://docs.ros.org/en/foxy/How-To-Guides/Parameters-YAML-files-migration-guide.html)
* [Passing ROS arguments to nodes via the command-line](https://docs.ros.org/en/foxy/How-To-Guides/Node-arguments.html)
* [蛤-ROS Topic / Service / Action 比較](https://ithelp.ithome.com.tw/articles/10247100)
* [jease0502 - ROS Cpp Parameter Server]()