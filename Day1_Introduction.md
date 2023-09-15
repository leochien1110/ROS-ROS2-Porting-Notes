![Image from THE ROS 1 VS ROS 2 TRANSITION](https://ithelp.ithome.com.tw/upload/images/20230913/20135014a1EF4A9Jge.jpg)
# 前言
有鑑於這幾年在用ROS時不時會上來查詢一些筆記，決定壓線參加鐵人賽提供一些自己在使用ROS2及Porting from ROS to ROS2上的心得和筆記回饋給這個社群。這邊特別Shout out to [蛤](https://ithelp.ithome.com.tw/users/20112348/ironman/1965)，他的ROS文章很簡單易懂，每天的題目也很切分的很剛好，很常上來複習他的筆記，這邊推薦給要學ROS的人。這次鐵人也會參照他的主題分類。

雖然碩論專題就是寫和無人機自動控制和Mapping，不過當時不知道ROS，被老闆說服自己用C++手刻Socket, OpenGL, ImGUI, Mapping Algorithm等等，刻出訊息交換、資料視覺化和3D UIUX，現在想想還真瘋XD 雖然現在coding技術應該在那時之上了，但不依靠ROS寫出一個無人機（機器人）專案真的太殘忍，所以這邊想讓大家可以更快上手ROS2，降低入坑恐懼。

> 警語：ROS2雖然宣稱可以支援其他平台，但是目前除了Windows的Sublinux有裝起來外，MacOS和原生Windows的設定都滿複雜還有很多版本限制，非常不建議使用Linux(Ubuntu, Mint, CentOS,etc.)和WSL以外的作業系統
# ROS2介紹
Robot Operating System(又稱ROS)，雖叫作業系統，但許多人更習慣稱他為Middleware，比較像是介面的工具，又或是社區的告示排，可以讓不同的Node(節點)可以彼此溝通。Node是ROS中的程式基本單元，當然程式可大可小，就看開發者如何設計了，具體細節會在ROS Node的章節更詳細介紹。

實際應用上的例子以經典自走車為例，Sensor偵測到前方障礙物時，會拋訊息到ROS Server上，而車輛的馬達控制系統會去ROS Server上找障礙物訊息，看到有障礙物的訊息時就煞車。當然因為ROS Server是告示排，所以大家都可以去看。譬如警報器系統也會去ROS Server找障礙物訊息，找到的話會發出警告聲響。這個被稱之為Master-Client的架構，由ROS Master為中心，而各個程式則為繞著他。

下圖[from [Robotics Back-End](https://roboticsbackend.com/what-is-ros/)]是一個很經典的範例：相機用Python讀進來，Motion Planning在背景跑，還有Python的搖桿控制系統，最後是用C++輸出到馬達上驅動小車。而圖中的ROS辦演著訊息交換的角色。
![ROS Framework Example from The Robotics Back-End](https://ithelp.ithome.com.tw/upload/images/20230913/20135014yfnYBQnXOY.jpg)

ROS是早在2007年Standford University機器人研究室就推出的，目的是為了解決機器人過於龐大且多元的領域造成知識上的隔閡。像是路徑規劃和馬達控制的溝通，在兩組專業人馬眼中或許都不算太難，但是到了要整合時，往往兩方過於獨立作業造成訊息格式、原理等不相同，最終要打掉重練的悲劇。

有了ROS架構，開發者可以專注於開發程式，並留意訊息交換格式即可與其他程式對接，不懂具體細節原理也沒關係。在這樣的生態系之下，開源的ROS底下有許許多多實用的Packages，讓機器人更複雜卻更易於管理和發開。套句ROS開發者常説的話，站在巨人的肩膀上

然而隨著時間的演進，ROS的底層架構開始跟不上時代的潮流。因為底層是依靠網路溝通，Socket中的UDP,TCP等架構會有延遲等問題，造成無法符合real-time operating system (RTOS, 時時作業系統)的規範。這個架構在系統小、訊息頻寬可以負荷時並沒有太大的問題，但在系統變複雜、訊息量變多後就顯的不堪負荷，Queue常常會延遲甚至掉訊息。於是ROS在一連串的更新後ROS迎來最後一個版本`Noetic`，之後便進入了ROS2的時代。

![https://ithelp.ithome.com.tw/upload/images/20230914/20135014wnzbt1oIAE.png](https://ithelp.ithome.com.tw/upload/images/20230914/20135014wnzbt1oIAE.png)

ROS2的目標就是實現RTOS，具體上就是利用Data Distribution Service (DDS)的架構重新建構底層訊息交換機制，引入Quality of Service (QoS) 的設定，可以在不同情境下設定訊息的品質、歷程、Delay等等。另外跨裝置的設定也在ROS2後變得更容易，且得意於QoS，跨裝置的訊息交換也得到提昇。

[Exploring the performance of ROS2](https://ieeexplore.ieee.org/document/7743223)論文中的架構圖可以看到兩者的差異：
![https://ithelp.ithome.com.tw/upload/images/20230913/20135014krBLFoC5rZ.png](https://ithelp.ithome.com.tw/upload/images/20230913/20135014krBLFoC5rZ.png)

圖上也會發現ROS2不再有Master-Client的關係，取而代之的是每個Node可以獨立運作而不再需要ROS Master。等於是ROS2放棄原本維護的網路架構而採用較為成熟的DDS架構，將經歷花在中上層串接API的部份。

最後的最後雖然沒有明文寫出來，ROS2上採用大量的Smart Pointer來避免Context Overwrite的情況，雖然對C++初學來說門檻會比較高，但是對程式的穩定度會有一定的加分，Python的部份則是沒有太多改動，畢竟只有物件可以使用XD



# 挑戰賽大綱
1. ROS2 環境安裝（Foxy）
2. ROS2 Package
3. ROS2 Node
4. ROS2 Topic
5. ROS2 Service
6. ROS2 custom msg & srv
7. ROS2 Parameters
8. ROS2 Action
9. ROS2 TF Tree
10. ROS2 Command Line
11. ROS2 Tools
12. ROS2 Simulation
13. ROS2 Applications
14. Further: ISAAC SIM

會在每個章節說明ROS2 Porting的部份，最後希望可以弄一個Cheatsheet方便大家查詢。
完蛋，打完手在抖，不曉得可以講完多少XD

# Reference
這邊特別感謝以下網站讓我的筆記更完整：

* ROS2官方教學：https://docs.ros.org/en/foxy/Tutorials.html
* ROS2 Github範例: https://github.com/ros2/examples
* ROS Wiki: https://en.wikipedia.org/wiki/Robot_Operating_System
* 蛤ROS自學筆記：https://ithelp.ithome.com.tw/users/20112348/ironman/1965
* 郭鎮穎學習筆記：https://hackmd.io/@evshary/ROS2Note/https%3A%2F%2Fhackmd.io%2F%40evshary%2FAboutMe
* What is ROS? https://roboticsbackend.com/what-is-ros/
* Exploring the performance of ROS2: [Maruyama, Yuya, Shinpei Kato, and Takuya Azumi. "Exploring the performance of ROS2." In Proceedings of the 13th International Conference on Embedded Software, pp. 1-10. 2016.](https://ieeexplore.ieee.org/document/7743223)
* THE ROS 1 VS ROS 2 TRANSITION: https://www.swri.org/industry/industrial-robotics-automation/blog/the-ros-1-vs-ros-2-transition
