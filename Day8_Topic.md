> 個人認為Topic算是**ROS**的核心精神之一。在這之前，機器人之間的訊息交換要不就透過Socket，不然就是Memory的操作，實作起來門檻高非常多。而**ROS**的Topic就是為了解決這個問題而生的，讓機器人之間的訊息交換變得非常簡單。

# Topic 介紹
Topic的發布和訂閱機制很像Youtube的訂閱+小鈴鐺，只要你訂閱的頻道有新的影片，Youtube就會通知你。**ROS**的Topic也是如此，只要你(Node)訂閱(Subscribe)的Topic有新發布(Publish)的訊息，**ROS**就會通知你。

![Publisher & Subscriber](https://docs.ros.org/en/foxy/_images/Topic-SinglePublisherandSingleSubscriber.gif)
![Publisher & Subscribers](https://docs.ros.org/en/foxy/_images/Topic-MultiplePublisherandMultipleSubscriber.gif)

Publisher 和 Subscriber 可以是一對一、一對多、多對一、多對多的關係，同一個Topic可以由多個Publisher發布，也可以由多個Subscriber訂閱。

# Quality of Service(QoS)
`ROS2`與`ROS`通訊機制上大同小異，但是`ROS2`採用Data Distribution Service(DDS)取代原本傳統的TCP/UDP的buffer(queue)機制，在溝通時多了一個`Quality of Service`的概念，可以讓使用者針對不同的Topic設定不同的傳輸品質，以確保訊息的可靠性。

## QoS Policy
* **History**：決定Publisher發布的訊息要不要保留，以及保留多久的歷史訊息。
* **Depth**：決定Publisher發布的訊息要保留多少筆。
* **Reliability**：決定Subscriber要不要確保收到Publisher發布的訊息。
* **Durability**：決定Publisher發布的訊息要不要保留，以及保留多久的歷史訊息。
* **Deadline**：決定Publisher發布的訊息要在多久內送達。
* **Lifespan**：決定Publisher發布的訊息要在多久內保留。
* **Liveliness**：決定Publisher和Subscriber之間的連線狀態。
* **Lease Duration**：決定Publisher和Subscriber之間的連線要保持多久。

### `ROS`比較與轉換

| 功能 | ROS | ROS2 |
| --- | --- | --- |
| Queue size | queue_size | history & depth |
| Reliability | roscpp::UDPROS & roscpp::TCPROS | best effort & reliable |
| Durability | latching publisher | transient local |

剩下的QoS Policy都是`ROS2`獨有的，也就是在`ROS`中沒有對應的功能。


## QoS Profile
`ROS2`提供了幾個預設的QoS Profile，可以讓使用者根據情境快速的設定不同的QoS Policy。

* **Default**：預設的QoS Profile，Publisher和Subscriber都是使用這個Profile:
  * History: Keep last
  * Depth: 10
  * Reliability: Reliable
  * Durability: Volatile
  * Deadline: Default
  * Lifespan: Default
  * Liveliness: System default
  * Lease Duration: Default
* **Services**：針對Service的QoS Profile，Publisher和Subscriber都是使用這個Profile。
* **Sensor Data**：針對Sensor Data的QoS Profile，Publisher和Subscriber都是使用這個Profile。
* **Parameters**：針對Parameters的QoS Profile，Publisher和Subscriber都是使用這個Profile。
* **System Default**：針對System Default的QoS Profile，Publisher和Subscriber都是使用這個Profile。

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



# Reference
---
* [蛤 Day 09 - ROS Topics
](https://ithelp.ithome.com.tw/articles/10204810)
* [ROS2 Humble Quality of Service](https://docs.ros.org/en/iron/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
* [ROS2 Understanding Topics](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)