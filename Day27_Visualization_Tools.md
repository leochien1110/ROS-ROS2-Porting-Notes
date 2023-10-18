# Visualization
資料視覺化是一種將資料以圖形的方式呈現的技術，透過視覺化的方式，可以讓使用者更容易理解資料的意義，並且可以更容易找出資料中的關聯性，進而做出更好的決策。

拿最簡單的影像來說，就是將RGB的陣列顯示成圖片，這樣就可以讓使用者更容易理解影像的內容。深度裝置的點雲則是需要用3D的引擎來預覽並且可以平移旋轉來查看。而慣性導航裝置(IMU)在視覺話後，可以清楚地看到運動的姿態。路徑規劃的部分則可以在事先建立好的地圖上顯示出路徑，並且可以看到機器人的位置和周遭的障礙物，更好監控演算法是否正確。前面介紹過的transform在3D的場景中則是可以清楚地看到不同frame之間的相對位置。

## RViz
**ROS**官方就有提供這個3D引擎，稱作**RViz**，可以用來顯示機器人的模型、感測器的資料、路徑規劃的結果等等。**RViz**是一個非常好用的工具，可以讓使用者更容易理解機器人的狀態，並且可以用來做除錯。

啟動**RViz**的指令如下：
```bash
rviz2
```

其中可以看到**RViz**的介面如下圖所示：
![rviz2](https://moveit.picknik.ai/humble/_images/rviz_empty.png)

接著首現要設定`Global Options`中的`Fixed Frame`。一般會設定在機器人的`base_footprint`，也就是`base`frame的影子的概念，永遠會在地上。如果存粹要顯示Sensor的話，會把`Fixed Frame`設定在Sensor的frame(像是`velodyne`)，這樣就可以看到Sensor的資料(像是`velodyne_points`)了。

可以在`Add`的按鈕中顯示各種預設的資料:
![rviz2_add](https://moveit.picknik.ai/humble/_images/rviz_click_add.png)
其中可以選則`by display type`或是`by topic`來顯示。

新增完後可以看到Topic的下拉選單，選擇欲顯示的Topic後，就可以看到資料了。如果沒有東西的話一般會先檢查`Fixed Frame`是否設定正確。再來可能就是TF的問題，可以`Add` TF來看看相對位置是否正確或甚至是有沒有TF的資料。最後就是檢查Topic有沒有正常發布。

在上方工具列的`File`可以儲存設定好的Layout成`.rviz`，方便下次開啟時可以直接套用。要在開啟時載入`.rviz`，使用以下指令：
```bash
rviz2 -d <path_to_rviz_file>
```

### Plugin
RViz本身有許多插件可以使用。其中，最常使用的就是Marker，可以在RViz中顯示各種形狀的物件，像是球體、立方體、箭頭甚至文字等等。Marker的使用方式可以參考ROS[官方教學](http://wiki.ros.org/rviz/DisplayTypes/Marker)。

另外還有IMU的插件，可以顯示慣性導航裝置的姿態和加速的大小。具體使用方式可以參考ROS[官方教學](http://wiki.ros.org/rviz/DisplayTypes/IMU)。

雖然是`ROS`的教學，但這不分沒有太多變化，就是把marker的部分改成`rviz2`就可以了。

再來就是imu plugin了。雖然官方沒有提供正式的imu plugin，目前主流是用[`imu-tools`](https://github.com/CCNYRoboticsLab/imu_tools/tree/foxy)。在安裝完後，可以在RViz2中Add IMU的Topic，就可以看到慣性導航裝置的姿態(箭頭)和加速的大小(箭頭長度)了。
![imu_plugin](http://wiki.ros.org/rviz_imu_plugin?action=AttachFile&do=get&target=rviz_imu_plugin.png)

其時還有很多可以自製的plugin，像是在RViz內自製搖桿、電池電量顯示等等。不過因為RViz是用Qt寫的，所以要自製plugin的話，需要先學習Qt的基礎，已經超出本教學的範圍了。[這篇文章](https://github.com/ros2/rviz/blob/rolling/docs/plugin_development.md)有提到如何自製plugin，有興趣的可以參考看看。

## Foxglove Studio
Foxglove Studio是現在加州開放自駕車的第一批公司之一，Cruise所開發的的工具，可以用來視覺化ROS2的資料，並在網頁上顯示和調整版面，是非常好用且美觀的工具。他是基於原本的開源`webviz`所延伸出來的。`ROS2`官方也有簡單的介紹[教學](https://docs.ros.org/en/foxy/How-To-Guides/Visualizing-ROS-2-Data-With-Foxglove-Studio.html)，或是直接到[Foxglove Studio](https://foxglove.dev/docs/studio)的網站上看看。
![foxglove studio](https://foxglove.dev/images/blog/introducing-foxglove-studio/hero.webp)

# ROS vs. ROS2
| 說明 | ROS | ROS2 |
| --- | --- | --- |
| RViz | rviz | rviz2 |

# Reference
---
* [MoveIt](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html)
* [rviz imu plugin](http://wiki.ros.org/rviz_imu_plugin)
* [ros2 plugin](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Pluginlib.html)
* [ros plugin](http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin)
* [Visualizing ROS 2 data with Foxglove Studio](https://docs.ros.org/en/foxy/How-To-Guides/Visualizing-ROS-2-Data-With-Foxglove-Studio.html)