> `ROS2`的基礎大致上都介紹完了，接下來介紹一些好用的工具吧！

# RQT
---
RQT是ROS的GUI工(ROS QT?)，可以用來顯示**ROS**的資訊，也可以用來控制**ROS**的運作。其中整合了很多前面就紹過的指令，包含Topic, Service, Parameter等等，也可以用來顯示TF的資訊。

在系統專案還小時用Command Line比較方便，但是規模一旦變大，就會需要一些GUI的工具來幫助除錯，這時候RQT就派上用場了。

## 安裝
```bash
sudo apt install ros-foxy-rqt-*
```
這會安裝所有rqt的相關套件，也可以案`Tab`查看有哪些套件可以安裝。

## 實用
### rqt_graph
顯示目前的topic-node關係圖。可以用來檢查topic是否有正確的連接，或是用來檢查topic的名稱是否正確。

```bash
ros2 run rqt_graph rqt_graph
```
應該會看到類似下面的圖
![rqt_graph](http://wiki.ros.org/rqt_graph?action=AttachFile&do=get&target=snap_rqt_graph_moveit_demo.png)

### rqt_topic
顯示目前的Topic資訊，包含Topic的名稱、Type、頻寬、頻率、數值等等。

```bash
ros2 run rqt_topic rqt_topic
```
應該會看到類似下面的圖
![https://ithelp.ithome.com.tw/upload/images/20231013/20135014FP8f6O3ubz.png](https://ithelp.ithome.com.tw/upload/images/20231013/20135014FP8f6O3ubz.png)

可以看到左側的收折圖示，打開後就是msg定義的格式。而checkbox則是是否訂閱該topic，打勾的話就會開始顯示數值。

不過要注意，如果Topic是自定義的msg，必須要source workspace之後才能顯示細節，不然只會有Topic的名稱。

另外一點是，如果看到大型陣列的Topic像是`image`, `pointcloud2`等等，不建議點開旁邊的收折圖示，會因為資料量太大而卡頓甚至當機。

### rqt_service_caller
可以用來呼叫Service，並顯示回傳的結果。

```bash
ros2 run rqt_service_caller rqt_service_caller
```

應該會看到類似下面的圖
![https://ithelp.ithome.com.tw/upload/images/20231013/20135014b02heBlLWL.png](https://ithelp.ithome.com.tw/upload/images/20231013/20135014b02heBlLWL.png)

可以在下拉選單中選擇要呼叫的Service。

這邊拿之前介紹過的`add_two_ints`來做範例，可以看到左側是Service的名稱，右側則是Service的輸入，可以直接在右側輸入數值，按下`Call`後就會顯示回傳的結果。

### rqt_image_view
可以用來顯示圖片，可以用來顯示`sensor_msgs/Image`的Topic。

```bash
ros2 run rqt_image_view rqt_image_view
```

應該會看到類似下面的圖
![rqt_image_view](http://wiki.ros.org/rqt_image_view?action=AttachFile&do=get&target=snap_rqt_image_view_1.png)

可以在下拉選單中選擇要顯示的Image Topic。

### rqt_gui
這個指令類似於rqt plugin的容器，可以用來啟用上述和更多的plugin。

```bash
ros2 run rqt_gui rqt_gui
# 或是簡化指令
rqt
```
`rqt`是個巨集，會直接啟動`rqt_gui`。

應該會看到類似下面的圖
![rqt_gui](http://wiki.ros.org/rqt?action=AttachFile&do=get&target=ros_gui.png)
可以看到上面有一排的選單，可以用來啟用不同的plugin。

像是我們可以到`Plugins`->`Visualization`->`Image View`，就可以啟用`rqt_image_view`。而且可以啟動多個plugin，像是我們可以同時啟用`rqt_image_view`和`rqt_topic`，就可以同時顯示圖片和topic的資訊，另外plugin也可以拖曳來調整位置。

這邊demo啟動兩個`rqt_image_view`用來檢視多個影像。
![rqt_gui_demo](http://wiki.ros.org/rqt_image_view?action=AttachFile&do=get&target=snap_rqt_image_view_twin.png)

# ROS vs. ROS2
---
| 說明 | ROS | ROS2 | 差異 |
| :--- | :--- | :--- | :--- |
| 啟動 | `rqt_*` | `ros2 run rqt_* rqt_*` | 多數`ROS2` RQT需要`ros2 run`才能啟動 |
| rqt_bag | 有 | `foxy`以前沒有 | `ROS2`需要自行安裝`rosbag2`的`foxy-future`或更新的版本 |


# Reference
---
* [rqt_graph](http://wiki.ros.org/rqt_graph)
* [rqt_topic](http://wiki.ros.org/rqt_topic)
* [rq_service_caller](http://wiki.ros.org/rqt_service_caller)
* [rqt_image_view](http://wiki.ros.org/rqt_image_view)
* [rqt_gui](http://wiki.ros.org/rqt_gui)
* [ROS2 - How to Send a Service Request with rqt(YouTube)](https://www.youtube.com/watch?v=hEVBXbspThA)