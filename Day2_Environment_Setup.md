> 接下來會開始用到許多Terminal 指令，適合有Ubuntu 或Linux 操作經驗的人，使用起來才不會太困難![/images/emoticon/emoticon04.gif](/images/emoticon/emoticon04.gif)
# 需求


-----


## 硬體
ROS2在硬體的要求上並不高，`x86`及`arm`不論`32bit`或`64bit`要跑起來都沒有問題，只在支援度上有會有差異。樹莓派的話一般會安裝的是`arm32`版本，但根據[官方文件](https://www.ros.org/reps/rep-2000.html)表示，支援度會稍差一點，是屬於Tier 3的硬體架構，不過大致上基本功能都可以運作。

## 作業系統

雖然[官方宣稱](https://docs.ros.org/en/foxy/Installation.html#binary-packages)可以支援除Ubuntu外還有MacOS及Windows，不過目前測試下來還是只有Linux(`Ubuntu`, `CentOS`, `Windows Sublinux(WSL) - Ubuntu`)才可以無痛安裝不需要依靠其他容器或虛擬環境。

使用Windows且不想要裝雙系統跑原生的Ubuntu的話，可以用WSL，安裝步驟可以參考[指令安裝](https://www.youtube.com/watch?v=CgkBV-4_lVM)和[使用介面安裝](https://www.youtube.com/watch?v=s62cH0X8M7o)的影片。

容器的話需要對Docker的操作有一定的了解，如果熟悉的話可以直接從官方的[Docker Repo](https://hub.docker.com/r/osrf/ros)下載對應的Image（`Foxy` or `Iron`）。

> WSL目前雖然支援很強大，甚至連CUDA都可以從Windows pass進去，但是用來跑ROS還是要注意兩點：
> 1. 其實WSL也是虛擬環境，如果要跟其他機器溝通，網路的設定上比較繁雜。
> 2. 再來是USB裝置的問題，雖然[usbip-win](https://learn.microsoft.com/zh-tw/windows/wsl/connect-usb)可以透過網路把裝置傳進去，但不是全部的電腦和裝置都支援。
> 
> WSL跑ROS比較建議拿來做教學、測試、檢視rosbag(ROS的log格式)。


## ROS2 版本
ROS2版本則是建議使用`Foxy`或是`Iron`，這兩個版本是LTS，維護時間較長及較其他版本穩定。雖然`Foxy`已經在2023年中EOL，但因為是前一代的LTS，有很多Packages和Documentation是以這個版本為基礎，轉換到`Iron`的問題也會比初代ROS2 LTS `Dashing`好很多。22.04的使用者就不用煩惱，直上`Iron`就沒問題了。

以下是建議的作業系統及對應的ROS2版本：

* Ubuntu 20.04 - ROS2 Foxy(EOL)
* Ubuntu 22.04 - ROS2 Humble
* WSL + Ubuntu 20.04 - ROS2 Foxy(EOL)
* WSL + Ubuntu 22.04 - ROS2 Iron
* Docker(Ubuntu 20.04) - ROS2 Foxy(EOL)
* Docker(Ubuntu 22.04) - ROS2 Iron

筆者在工作上是用Ubuntu 20.04 + ROS2 Foxy，因為需要時常切換到ROS Noetic，僅支援Ubuntu 20.04。
自己的電腦則是Ubuntu 22.04 + ROS2 Iron。當然有需要用到ROS Noetic 再裝Docker(Ubuntu 20.04) + Foxy即可。

# 安裝


-----


以下有幾種安裝方式:

## Debian(推薦)
ROS2最推薦可以從Debian安裝編譯好的Binary，只需要幾行指令即可安裝完成。確切的指令和步驟可以參考[官方ROS 2 Foxy Fitzroy安裝文件](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)。

Github上也有開發者把官方文件寫成[一鍵安裝的Script](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu)，可以無腦執行，**但跑之前還是建議去比對官方指令**，以防安裝奇怪的東西或移除重要的東西。
1. 下載repo:
    ```
    git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
    ```
2. 將`run.sh`內的版本改成Foxy或Iron，package維持預設`desktop`即可，`ros-base`一般來說是給產品部署用的，只會安裝一些基礎的package。
3. 接著跑`./run.sh`即可安裝。

## Source Code
當然也可以用[Source code安裝](https://docs.ros.org/en/foxy/Installation.html#building-from-source)，這樣可以讓你在不同的作業系統安裝不同的版本，不過也會遇到比較多相容性的問題，這邊並不推薦。

## Others
如果要安裝在其他作業系統環境的話官方也有提供文件，可以參考[這個文件](https://docs.ros.org/en/foxy/Installation.html)。

# Reference


-----


ROS2 官方安裝文件: https://docs.ros.org/en/foxy/Installation.html
郭振穎的Hackmd: https://hackmd.io/@evshary/ROS2Note/https%3A%2F%2Fhackmd.io%2F%40evshary%2FROS2installation
挨踢實驗室：https://www.youtube.com/@itlab.
