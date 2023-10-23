# 遠端控制和監測
---
上述的各種操作都是在本機端進行，但是如果要在遠端進行呢？在一般的使用情境下，我們並無法把螢幕鍵盤滑鼠都接在機器人上跟著走，所以會需要透過遠端來進行操作和訊息傳遞。

**ROS**是基於網路架構的，所以只要兩台電腦在同一個區域網路下，就可以進行遠端操作。

## ROS
在`ROS`中，主要透過設定`ROS_MASTER_URI`和上`ROS_IP`來進行遠端操作。
1. 在機器人(ROS Master Server)上:
    ```bash
    export ROS_IP=<robot_ip>
    roscore
    ```
2. 在電腦(ROS Client)上:
    ```bash
    export ROS_MASTER_URI=http://<robot_ip>:11311
    export ROS_IP=<client_ip>
    ```

## ROS2
在`ROS2`中，底層已經使用`DDS`，所以在區域網路中是以**Multicast**，所以不需要設定`ROS_MASTER_URI`和`ROS_IP`，只需要設定`ROS_DOMAIN_ID`即可，一般預設為`0`。兩台電腦都設定一樣才能讓`ROS2`跨裝置溝通。以下為設定範例：

1. 首先要確認兩台電腦都在同一個區域網路下，並且拿到機器人的IP位址:
    ```bash
    ifconfig
    ```
2. (Optional)設定ssh id以便從電腦連到機器人
    ```bash
    vim ~/.ssh/config
    ```
    接著將Hostname, ip等設定好
    ```
    Host <my_robot>
        Hostname <robot_ip>
        User <robot_user>
    ```
    在機器人上設定ssh id
    ```bash
    ssh-copy-id <my_robot>
    ```
3. 遠端連線到機器人
    如果有設定ssh id，則可以直接連線:
    ```bash
    ssh <my_robot>
    ```
    否則需要輸入密碼、使用者名稱和機器人IP:
    ```bash
    ssh <robot_user>@<robot_ip>
    ```
4. 測試multicast是否啟動
    在電腦上開啟multicast接收器用來測試
    ```bash
    ros2 multicast receive
    ```
    在機器人上發送測試訊息
    ```bash
    ros2 multicast send
    ```
    如果在電腦上可以收到訊息，則表示multicast正常運作。
    1. 若無法收到，則可能是防火牆的問題，可以將防火牆設定為允許multicast:
        ```bash
        sudo ufw allow in proto udp to 224.0.0.0/4
        sudo ufw allow in proto udp from 224.0.0.0/4
        ```
    2. 也有可能是multicast的網域不同。可以透過上述`ifconfig`來查看網域：
        ```bash
        # 有線的話會類似
        eno1: flags=4163<...,MULTICAST>

        # 無線的話則會類似
        wlp2s0: flags=4163<...,MULTICAST>
        ```
        如果Multicast不是在想要的網域，假設想要設定到`eth0`的話，可以透過以下指令來設定：
        ```bash
        # for newly added
        sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0 

        # or update it
        sudo route replace -net 224.0.0.0 -netmask 240.0.0.0 dev eth0
        ```

        **MacOS**的話，假設目標是`en0`，可以透過以下指令來設定：
        ```bash
        # for newly added
        sudo route add 224.0.0.0 -netmask 240.0.0.0 -interface en0

        # or update it
        sudo route change 224.0.0.0 -netmask 240.0.0.0 -interface en0
        ```
        
5. 在機器人上設定ROS_DOMAIN_ID
    ```bash
    export ROS_DOMAIN_ID=30
    ```
6. 在電腦開啟另一個terminal，設定ROS_DOMAIN_ID
    ```bash
    export ROS_DOMAIN_ID=30
    ```
7. 在機器人上啟動talker
    ```bash
    ros2 run demo_nodes_cpp talker
    ```
8. 在電腦上啟動listener
    ```bash
    ros2 run demo_nodes_cpp listener
    ```
    如果可以在電腦上看到機器人發送的訊息，則表示遠端操作成功！！！

## ROS vs. ROS2
---
| 說明 | ROS | ROS2 |
| --- | --- | --- |
| 機器人上 | `export ROS_IP=<robot_ip>` | `export ROS_DOMAIN_ID=30` |
| 電腦上 | `export ROS_MASTER_URI=http://<robot_ip>:11311`<br>`export ROS_IP=<client_ip>` | `export ROS_DOMAIN_ID=30` |



# 總結
---
好啦，ROS2鐵人終於到這邊結束了(其實第三天就結束了XD)，謝謝大家一路看到這邊。當初之所以會想參加這個鐵人，其實是希望`ROS2`可以多一點完善的中文的資源，不然雖然有很多優質短文，但大都零零落落的很沒有組織性。

期間很謝謝很多前輩的文章，尤其是[蛤](https://ithelp.ithome.com.tw/users/20112348/ironman)前輩，他的[ROS筆記](https://ithelp.ithome.com.tw/users/20112348/ironman/1965)是讓我想要回饋ITHOME鐵人的主因。另外還有感謝**ROS2**官方的文件和範例，讓我省去很多測試的時間，另外還有copilot的幫忙，主要在文章後面將`ROS` vs `ROS2`的表格中很快速地幫我寫好語法，讓我省去很多時間，真的很推薦大家都去訂閱哈哈。

這次的鐵人賽，我也是第一次參加，也是第一次寫這麼多的文章，雖然有點累，但也學到了很多東西，也希望大家可以從這些文章中學到一些東西。這些文章的Markdown我都放在[GitHub](https://github.com/leochien1110/ROS-ROS2-Porting-Notes)，去找吧(誤) 歡迎大家一起來編輯修改，讓這些`ROS2`的文章更加完善，幫助中文圈的人迎向大機器人時代！



# Reference
---
* [ROS Remote monitoring and control](https://wiki.ros.org/action/fullsearch/robotican/Tutorials/Remote%20monitoring%20and%20control?action=fullsearch&context=180&value=linkto%3A%22robotican%2FTutorials%2FRemote+monitoring+and+control%22)
* [The ROS_DOMAIN_ID](https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html)
* [ROS-ROS2-Porting-Notes](https://github.com/leochien1110/ROS-ROS2-Porting-Notes)