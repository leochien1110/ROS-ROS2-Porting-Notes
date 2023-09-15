> 發現我常常會把 `ROS` 和 `ROS2` 混著用，這裡說明一下。
> 首先並沒有`ROS1`這種說法，但**ROS**整個架構和第一代`ROS`在寫Document實在很難區分。
> 所以以後在說架構時會用粗體**ROS**，至於在說版本時則用`ROS`。
> 另外我會列出對應的`ROS`指令，讓ROS2 Porting可以參考，實作時只需跑`ROS2`的版本即可。
# ROS2 Software Hiearachy
---
`ROS2`中的Hiearachy(抱歉不太會翻譯，類似階級的概念？)，是從 Workspace -> Package -> Node/Component(Nodelet in `ROS`)。所以今天會來說明一下Package，順便帶過Workspace。

# Workspace
---
在**ROS**中，一般都是以一個專案為一個單位，並以資料夾打包起來，而這個資料夾就稱之為Workspace(`ws`)。

這邊來個簡單的範例。首先我們會先打開Terminal(`Ctrl`+`Alt`+`T`)創建一個資料夾，這邊用`ros2_ws`來表示專案名稱，當然也可以任意命名：
```bash
mkdir ros2_ws
```

接著進入到workspace，並創建`src`的資料夾用來放source code:
```bash
cd ros2_ws
mkdir src
#上述三行指令可以濃縮成一行：
mkdir -p ros2_ws/src
```

這樣就完成了你第一個ROS2專案! (誤

這邊附上`ROS2`官方文件中提到一個Workspace應該會長的樣子，明天會實作到，你也以用`tree -D 3 ros2_ws`跑跑看：
```bash
workspace_folder/
    src/
      cpp_package_1/
          CMakeLists.txt
          include/cpp_package_1/
          package.xml
          src/

      py_package_1/
          package.xml
          resource/py_package_1
          setup.cfg
          setup.py
          py_package_1/
      ...
      cpp_package_n/
          CMakeLists.txt
          include/cpp_package_n/
          package.xml
          src/
```

# Colcon 指令
---
## 編譯Package
用過`ROS`的人應該會很熟悉`catkin`這個**ROS**官方提供的Compiler Macro，可以幫助連接package之間的相依和支持在任意路徑下使用`rosrun`, `roslaunch`等CLI。
而在ROS2則改為`colcon`，大體上使用方式與`catkin`一樣。
### 用法
Colcon 安裝須在 workspace的根目錄，也就是上面的`ros2_ws`下執行。安裝完Source過後則可以再任意地方執行。

1. 安裝`src`底下所有的Package:
    `ROS`
    ```bash
    catkin_make   # opt1: ROS built in macro
    catkin build  # opt2: with python wrapped catkin tools
    ```
    `ROS2`
    ```bash
    colcon build
    ```
    
    目前src還是空的，但build起來是沒問題的。
    
    > 這邊要小抱怨一下`ROS2`(後面還會更多抱怨XD)，`catkin build`的介面很漂亮，會用顏色區隔不同的訊息幫助debug，`colcon build`則是一整片白，看了眼睛超痛。
    
2. 安裝特定Package
    `ROS`
    ```bash
    catkin build <package1> <package2> ...
    ```
    `ROS2`
    ```bash
    colcon build --packages-select <package1> <package2> ...
    ```
    
執行完後會在workspace看到`build`, `install`, `log`的資料夾，就是我們的binary executable files附帶`ROS2`的指令包。
與`ROS`最大的不同是，`ROS`預設會產生`build`, `devel`資料夾，`install`的話則是要另外指定才能啟用。另外`setup.bash`的位置也不一樣，可以參考下面的指令去。

要使用**ROS**的指令去跑程式(`ros2 run`, `ros2 launch`等指令)，會需要在terminal設定環境：
```bash
# ros2
cd ros2_ws
source install/setup.bash

# ros
source devel/setup.bash
```



### 補充
Colcon最常用到的就是上面兩個指令，當然還有很多其他功能，像是更改CMake Build Type為`Release`(下面有範例)等等，這邊就不再贅述。Colcon已經複雜到**ROS**為他 專門做一期視頻(X 專門寫一個Documentation(O，可以參考[這個網站](https://colcon.readthedocs.io/en/released/index.html)。

`Build Type`設成`Release`更改還滿常會用到的，可以大幅優化PCL, OpenCV等第三方API，筆記筆記
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release <package1> <package2> ...
```

至於Python則常用到symlink，可以避免每次修改script後要重新跑`colcon build`
```bash
colcon build --symlink-install <package1> <package2> ...
```


不知不覺越寫越多，今天就先到這邊好了，Package留給明天。

# ROS vs. ROS2
---
|功能| ROS | ROS2|
|:----| ----|----|
|Compiler Macro |Catkin | Colcon|
|安裝Package|`catkin_make` 或`catkin build`|`colcon build`|
|預設產生資料夾|build, devel, log|install, build, log|
|source 位置|`devel/setup.bash`|`install/setup.bash`|
|ROS指令 | `rosrun`, `roslaunch`,... | `ros2 run`, `ros2 launch`,... |



# Reference
---

* Day 11 - 編譯工具 catkin v.s colcon[蛤]: https://ithelp.ithome.com.tw/articles/10244139
* Colcon: https://colcon.readthedocs.io/en/released/