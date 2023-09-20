> 個人認為Topic算是**ROS**的核心精神之一。在這之前，機器人之間的訊息交換要不就透過Socket，不然就是Memory的操作，實作起來門檻高非常多。而**ROS**的Topic就是為了解決這個問題而生的，讓機器人之間的訊息交換變得非常簡單。`ROS2`中引入QoS的概念稍微複雜，這裡會盡量簡單的介紹，讓使用者可以快速上手。

# Topic 介紹
Topic的發布和訂閱機制很像Youtube的訂閱+小鈴鐺，只要你訂閱的頻道有新的影片，Youtube就會通知你。**ROS**的Topic也是如此，只要你(Node)訂閱(Subscribe)的Topic有新發布(Publish)的訊息，**ROS**就會通知你。

Publisher 和 Subscriber 可以是一對一、一對多、多對一、多對多的關係，同一個Topic可以由多個Publisher發布，也可以由多個Subscriber訂閱。（貴圈真亂

下圖是官方的GIF，可以看到Publisher和Subscriber之間的關係。右側的Node發布訊息到Topic，左側的Node訂閱Topic，當Topic有新的訊息時，左側的Node就會收到。
![Publisher & Subscriber](https://docs.ros.org/en/foxy/_images/Topic-SinglePublisherandSingleSubscriber.gif)

第二個圖則是展示多個Publisher和Subscriber之間的關係。上方的兩個Publishers皆可以發布訊息至Topic，下方的兩個Subscribers皆可以訂閱Topic，當Topic有新的訊息時，下方的兩個Subscribers就會收到。
![Publisher & Subscribers](https://docs.ros.org/en/foxy/_images/Topic-MultiplePublisherandMultipleSubscriber.gif)



# Quality of Service(QoS)
`ROS2`與`ROS`通訊機制上大同小異，但是`ROS2`採用Data Distribution Service(DDS)取代原本傳統的TCP/UDP的buffer(queue)機制，在溝通時多了一個`Quality of Service`的概念，可以讓使用者針對不同的Topic設定不同的傳輸品質，以確保訊息的可靠性。

## QoS Policy
QoS Policy底下有八個選項，可以用來設定訊息傳輸的品質和效期：
* **History**：
  * *Keep last*：保留最新的N筆訊息，可以透過queue的選項進行配置。
  * *Keep all*：保留所有的訊息，受制於底層middleware的系統限制。
* **Depth**：
  * *Queue size*：設定queue的大小，僅在`History`為`Keep last`時有效。
* **Reliability**：
  * *Best effort*：試著傳遞訊息，但是不保證訊息一定會傳遞成功，類似UDP的傳輸方式。
  * *Reliable*：保證訊息一定會傳遞成功，若失敗會進行重傳，類似TCP的傳輸方式。
* **Durability**：
  * *Transient local*：Publisher會試著保存樣本給晚加入的Subscriber。
  * *Volatile*：不保存樣本。
* **Deadline**：
  * *Duration*：Topic上訊息與訊息之間最大時間間隔。
* **Lifespan**：
  * *Duration*：Topic從被Publish到被Subscribe的最大時間間隔。超過這個時間間隔的訊息會被悄悄丟棄(不會有任何提示)。
* **Liveliness**：
  * *Automatic*：一個Node有任一Publisher發布訊息，則該Node就會被認為是活著的，有效時間為`Liveliness lease duration`。
  * *Manual by topic*：透過Publisher API手動設定Publisher的活著狀態，有效時間為`Liveliness lease duration`。
* **Lease Duration**：
  * *Duration*：Publisher的活著狀態的最長有效時間，否則系統將會認為Publisher已經失去作用。可以用來作為失效指標。

上述的*無duration*的policies，都有一個預設的`system default`的選項，會使用middleware預設的值。對於*有duration*的policies，也都有一個預設的`default`的選項，會使用middleware預設的值，其值為`infinite` 無限長。

### `ROS` vs. `ROS2`

| 功能 | ROS | ROS2 |
| --- | --- | --- |
| Queue size | queue_size | history & depth policies |
| Reliability | roscpp::UDPROS & roscpp::TCPROS | reliability policy |
| Durability (一般是給tf publisher) | latching publisher | transient local + depth |

剩下的QoS Policy都是`ROS2`獨有的，也就是在`ROS`中沒有對應的功能。


## QoS Profile
`ROS2`提供了幾個預設的QoS Profile，可以讓使用者根據情境快速的設定不同的QoS Policy。具體的參數可以參考[rclcpp API](https://docs.ros2.org/foxy/api/rclcpp/namespacerclcpp.html)中以`QoS`結尾的class。

* **Default for Publisher & Subscirber**：為了方邊ROS2 Porting，官方預設Publisher和Subscriber的QoS Profile(不在rclcpp API)，在不指定的情況下呼叫Publisher和Subscriber的建構子時，會使用這個Profile：
  * History: Keep last
  * Depth: 10
  * Reliability: Reliable
  * Durability: Volatile
  * Deadline: Default
  * Lifespan: Default
  * Liveliness: System default
  * Lease Duration: Default

* [**Services**](https://docs.ros2.org/foxy/api/rclcpp/namespacerclcpp.html)：針對Service的QoS Profile，Server和Client。
* [**Sensor Data**](https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1SensorDataQoS.html)：隊sensor的資料，我們注重在他的接收即時性，而不是資料的穩定性。也就是說開發者更在意最新的資料，而可以忍受遺失一些資料，所以reliability是best effort，queue size也比較小。
* [**Parameters**](https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1ParametersQoS.html)：Parameter在**ROS**中是為Service的一種，所以他的QoS Profile類似Service。唯一的不同是depth queue size大很多，這樣parameter client在失去parameter server的情況下依然可以取得參數。
* [**System Default**](https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1SystemDefaultsQoS.html)：底層RMW預設的Profile。

## QoS 相容性
以下是`ROS2`官方文件中提供的QoS Policy相容性表格，可以讓使用者快速的判斷Publisher和Subscriber之間的相容性。

*Compatibility of reliability QoS policies*:

| Publisher | Subscriber | 相容性 |
| --- | --- | --- |
| Best Effort | Best Effort | Yes |
| Reliable | Best Effort | Yes |
| Best Effort | Reliable | No |
| Reliable | Reliable | Yes |

*Compatibility of durability QoS policies*:

| Publisher | Subscriber | 相容性 | 訊息結果 |
| --- | --- | --- | --- |
| Volatile | Volatile | Yes | New messages only |
| Volatile | Transient Local | No | No communication |
| Transient Local | Volatile | Yes | New messages only |
| Transient Local | Transient Local | Yes | New and old messages |


# Message
這邊先簡單的介紹一下Message，為明天的Topic Publisher和Subscriber做準備。

除了官方有提供個常用的Message(e.g. `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, ...)，使用者也可以自行定義Message，只要符合Message格式即可。

自定義的訊息有可以給Topic用的`.msg`，也有給Service用的`.srv`。具體的使用之後會在另外一篇文章中介紹，這裡僅簡單的介紹一下Message的使用方式。


## `ROS`比較
`ROS2`中Message的改動不大，除了多了一層namespace `/msg/`，其他的格式都是一樣的。

還有`.msg`檔案，`ROS`中的`Header`在`ROS2`中就變成`std_msgs/Header`。可以參考[`ROS`的`Image.msg`](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/Image.msg)和[`ROS2`的`Image.msg`](https://github.com/ros2/common_interfaces/blob/foxy/sensor_msgs/msg/Image.msg)。

### C++ API
最主要的差異在header的引入方式，`ROS`中是`#include <std_msgs/String.h>`，而`ROS2`中是`#include <std_msgs/msg/string.hpp>`。

**ROS Version**
```cpp
#include <std_msgs/String.h>

std_msgs::String msg;

msg.data = "Hello World!";
```

**ROS2 Version**
```cpp
#include <std_msgs/msg/string.hpp>

std_msgs::msg::String msg;

msg.data = "Hello World!";
```

TODO: Validate the usage of `std_msgs::String` in `ROS2`.
### Python API
Python的用法沒有更動

**Both ROS & ROS2 Version**
```python
from std_msgs.msg import String

msg = String()

msg.data = "Hello World!"
```


# Reference
---
* [蛤 Day 09 - ROS Topics
](https://ithelp.ithome.com.tw/articles/10204810)
* [ROS2 Humble Quality of Service](https://docs.ros.org/en/iron/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
* [ROS2 Understanding Topics](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
* [RCLCPP QoS](https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1QoS.html)
* [RCLCPP](https://docs.ros2.org/foxy/api/rclcpp/namespacerclcpp.html)
* [ROS2 from the Ground Up: Part 7- Achieving Reliable Communication in ROS 2 with QoS Configurations](https://medium.com/@nullbyte.in/ros2-from-the-ground-up-part-7-achieving-reliable-communication-in-ros-2-with-qos-configurations-83c534c3aff5)