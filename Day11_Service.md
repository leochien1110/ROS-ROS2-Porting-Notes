> 今天介紹**ROS**的第二種通訊機制：Services

# Services
---
Service是**ROS**中的另一種通訊方式，與網頁機制中的Request-Response是一樣概念。與Topic不同的是，當Service Server收到Service Client的請求後，會立即回傳結果，而不是像Topic一樣，Server會持續的發送資料給Client。不過Server有Blocking的機制，也就是說，當Server收到Client的請求後，Server會一直等待處理完畢後才會回傳結果，這也是Service與Topic的差異之一。Topic是用來傳送串流資料和連續資料，而Service只有在被呼叫時才傳送資料。

本來想畫圖，但官方的GIF實在太好了，就直接拿來用了。
![Service](https://docs.ros.org/en/foxy/_images/Service-SingleServiceClient.gif)
首先可以看到Client發送Request到Service，接著Service會幫我們把Request傳送給Server，Server處理完後會回傳Response給Service，Service再將Response傳送給Client。其中這段時間，Client會一直等待Server回傳Response，這就是Blocking的機制。

![Mutli-Client Services](https://docs.ros.org/en/foxy/_images/Service-MultipleServiceClient.gif)
與Topic不同的是，Services可以有多個Clients，不過只能有一個Server，Server會依序處理Client的Request，也就是說，Server會依序回傳Response給Client。

實際應用上，Service通常用在需要Server回傳結果的情況，例如：機器人的導航系統，Client會傳送目標位置給路徑規劃Server，Server會計算路徑並回傳給Client。此時，Client會再將回傳的路徑傳送給機器人控制系統的Action Server(**ROS**的第三種通訊機制)，機器人控制系統會依照路徑移動。

# QoS
---
Topic中介紹過的QoS在Service中也是可以使用的，畢竟其底層都是使用DDS。不論是Service Server還是Service Client，都可以透過QoS來設定Service的傳輸品質。可以回顧[**Day8 ROS2 Topic**
](https://ithelp.ithome.com.tw/articles/10323283)中QoS的機制。

# Srv
---
> Topic是用來傳送Message，而Service是用來傳送Service。這邊滿容易混淆的要注意。
> 
> 另外還有一點容易混淆的是，API當中會稱Server為Service，這邊的Service是指Service Server。譬如C++的API中，就是以`node->create_service`來建立Service Server，而`node->create_client`則是建立Service Client。

Service的訊息格式是`.srv`，與Topic的`.msg`不同，`.srv`的格式是`Request`和`Response`，因此會用`---`來分隔。除此之外，其他的格式都是一樣的。
```python
# request
int64 a
int64 b
---
# response
int64 sum
```


接下來就會來寫寫看Service Server和Service Client，也會介紹如何自定義Service。


# Reference
---
* [蛤 ROS Services](https://ithelp.ithome.com.tw/articles/10206721)
* [ROS2 Service](https://docs.ros.org/en/galactic/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
* [QoS in Services from a Github Issue](https://github.com/ros2/rclcpp/issues/1785)
* [QoS Python Example from Stackoverflow](https://answers.ros.org/question/417523/ros2-deadline-qos-issue/)