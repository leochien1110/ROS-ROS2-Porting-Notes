> 本來想在Topic結束先介紹指令，但Service和Topic概念很像，所以等到有Topic和Service時一起寫在這邊。如果有忘記的地方的話，可以回去查前幾天的文章！

# Command Line
**ROS**中我個人認為第二重要的就是要如何活用各種工具，包括圖像介面工具(GUI)和Command Line Interface(CLI)，他可以幫助我們Debug，找出為什麼有些Node無法運作或是接收不到訊息，甚至是監看Topic的速度、頻寬和內容等等。

其中最重要也最適用在各個場景的就是CLI，因為**ROS**到後期常常會需要遠端操作機器人，因此GUI就不適用了，而且GUI的功能也沒有CLI完整，因此熟悉CLI就變得非常重要。

## Topic
首先我們可以先把之前的Day9的Publisher範例跑起來:
```bash
ros2 run py_pubsub pub_member_function
```
> 由於我們自定義message時有改過`pub_member_function`，可以回去複製Day9的範例，或是開一個新的node，然後把Day9的範例的內容貼上去。
1. 想必大家都忘記我們的Topic名稱是什麼了，這很常見，於是我們有了第一個工具`ros2 topic list`，跑起來後應改可以看到：
    ```bash
    /topic
    /rosout
    ```
2. 其中`/rosout`是**ROS**的系統Topic，用來顯示系統的log。另一個`/topic`就是我們的Topic了。這時我們如果想要看`/topic`的資訊，包括訊息格式、Publisher和Subscriber的數量等等，可以使用`ros2 topic info /topic`，應該會看到:
    ```bash
    Type: std_msgs/msg/String
    Publisher count: 1
    Subscription count: 0
    ```
3. 再來我們會好奇到底發布了什麼訊息，這時候就可以使用`ros2 topic echo /topic`，應該會看到:
    ```bash
    data: Hello World: 0
    ---
    data: Hello World: 1
    ---
    data: Hello World: 2
    ...
    ```
4. 對於streaming data，通常很介意framerate（幀數），這時可以用`ros2 topic hz /topic`來看發布的速度，應該會看到:
    ```bash
    average rate: 9.999
        min: 0.100s max: 0.100s std dev: 0.00000s window: 9
    average rate: 9.999
        min: 0.100s max: 0.100s std dev: 0.00000s window: 19
    average rate: 9.999
        min: 0.100s max: 0.100s std dev: 0.00000s window: 29
    ..
5. 當有跨裝置的串流時，遇到FULL HD的UYVY影像，通常會有60MB/s或更高，網路就會被他塞爆。這時候就可以用`ros2 topic bw /topic`來看頻寬，應該會看到:
    ```bash
    average: 0.000MB/s
        mean: 0.000MB/s, stddev: 0.000MB/s, window: 0.000MB/s
    average: 0.000MB/s
        mean: 0.000MB/s, stddev: 0.000MB/s, window: 0.000MB/s
    average: 0.000MB/s
        mean: 0.000MB/s, stddev: 0.000MB/s, window: 0.000MB/s
    ...
    ```
    這時候可以看到頻寬是0，因為我們的訊息很小，但如果是影像的話就會看到頻寬很大了。
    > 所以在做影像串流時會使用`image_transport`的package或是用h264,h265等技術來壓縮影像，把頻寬降到~2MB/s，這樣就可以在網路上傳輸了。
6. 上面大多是測試publisher，CLI也可以手動發布訊息來測試subscriber。首先將Publisher關掉，改開`ros2 run py_pubsub sub_member_function`，這時應該不會看到任何訊息，因為沒有Publisher。
    
    這時用`ros2 topic pub /topic std_msgs/msg/String "{data: Hello World}"`來發布訊息，應該會看到:
    ```bash
    publisher: beginning loop
    publishing #1: std_msgs.msg.String(data='Hello World')
    ```
    同時Subscriber也收到訊息:
    ```bash
    [INFO] [1600000000.000000000] [minimal_subscriber]: I heard: Hello World
    [INFO] [1600000000.000000000] [minimal_subscriber]: I heard: Hello World
    ...
    ```

    > 訊息的格式是用YAML或是python dictionary，因此要用`{}`包起來，並且使用`:`來分隔key和value。最外面要再用`""`包成字串，才不會被shell解析成其他指令。

7. 其實`ros2 topic`還有很多功能，可以用`ros2 topic -h`來看更多功能清單。每個功能也都可以再用`-h`來看更多細節, e.g. `ros2 topic echo -h`，可以看到他還可以設定很多不同的QoS。

## Service
Service的部分也有很多指令，先來啟動Service Server:
```bash
ros2 run py_srv service_member_function
```

1. 這時候可以用`ros2 service list`來看有哪些Service，應該會看到:
    ```bash
    /add_three_ints
    ```
2. 想查看service的資訊，可以用`ros2 service type /add_three_ints`，應該會看到:
    ```bash
    tutorial_interfaces/srv/AddThreeInts
    ```
    這個資訊待會可以幫助我們用CLI來執行Service Client。

    其實`ros2 service list -t`也可以看到同樣的資訊，但是`-t`會顯示更多資訊，包括service的type和package，應該會看到:
    ```bash
    /add_three_ints [tutorial_interfaces/srv/AddThreeInts]
    ```
3. 再來是查詢service的格式，可以用`ros2 interface show tutorial_interfaces/srv/AddThreeInts`，應該會看到:
    ```bash
    int64 a
    int64 b
    int64 c
    ---
    int64 sum
    ```
    這個格式待會也會用來來執行CLI的Service Client。

4. 我們會遇到需要手動設定Service Client來測試Service Server的運作。這時候可以用`ros2 service call /add_three_ints tutorial_interfaces/srv/AddThreeInts "{a: 1, b: 2, c: 3}"`，應該會看到:
    ```bash
    sum: 6
    ```
    這時候就可以看到Service Server的運作正常了。

5. Service的CLI不像Topic那麼多，因為Server一定要寫成程式。更多的功能可以用`ros2 service -h`來看，每個功能也都可以再用`-h`來看更多細節, e.g. `ros2 service call -h`。

# ROS vs. ROS2
其實很多`ROS`的CLI功能都還沒有在`ROS2`中實現，像是`rostopic echo -n 5 /topic`可以用來只顯示最新的5筆訊息，但是`ros2 topic echo`沒有這個功能。不過`ROS2`的CLI功能也還在持續開發中，所以未來應該會越來越多。  

下面是一些常用的指令，其中還有新增ros node的指令，可以用來查看node的資訊，像是publisher和subscriber的topic name和service name等等。

| 說明 | ROS | ROS2 | 差異 |
| --- | --- | --- | --- |
| 列出所有Topic | `rostopic list` | `ros2 topic list` |
| 監聽Topic | `rostopic echo /topic` | `ros2 topic echo /topic` | NA |
| 版本限定 echo 功能 | `-n`指定顯示最新幾筆訊息 `-b`讀取bag的topic | `--qos-?`qos相關設定 `-l`截斷array `--csv`輸出成csv  | NA |
| pub data 使用 | `rostopic pub [topic_name] [topic_type] [data]` | `ros2 topic pub [topic_name] [topic_type] "{data: msgs}"` | `ROS2` data 無法直接使用，要用`"{}"`包起來 |
| 列出所有Service | `rosservice list` | `ros2 service list` | NA |
| 版本限定 call 功能 | `--wait`等待service設定 | `-r` 設定request的速度 | NA |
| call args 使用 | `rosservice call /add_three_ints 1 2 3` | `ros2 service call /add_three_ints tutorial_interfaces/srv/AddThreeInts "{a: 1, b: 2, c: 3}"` | `ROS2` args 無法直接使用，要用`"{}"`包起來 |
| 列出所有Node | `rosnode list` | `ros2 node list` |


# Reference
---
* [ROS Topic](http://wiki.ros.org/rostopic)
* [ROS2 Topic](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
* [ROS Service](http://wiki.ros.org/rosservice)
* [ROS2 Service](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
