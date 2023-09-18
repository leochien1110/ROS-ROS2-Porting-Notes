> 這是ROS2最重要的一章，因為ROS2的Node是最基本的單位，也是ROS2的核心，所以這邊會花比較多的篇幅來介紹。一樣會拆成C++和Python兩個部分來介紹，並且會比較ROS和ROS2的差異。鑑於上次的經驗，C++的篇幅和Python會差很多，所以這邊會先介紹Node和Python範例，C++的範例改到下一篇再介紹，讓篇幅比較平均。

# 介紹

之前在[Workspace](https://ithelp.ithome.com.tw/articles/10318186)中有提到，Node是**ROS**中最小單位。一個Package可以有多個Node，用來執行不同的操作。每個Node都可以跟別的Node透過Topic、Service、Action、Parameter等方式溝通來取得資訊或是執行動作。

Node其實就是一般的可執行檔，可以是C++ Compiler出來的binary，也可以是Python的script，所以在語言的使用上是比較彈性的。舉例來說，可以用C++寫相機的driver node，然後用Python寫影像辨識的node，最後再用C++寫顯示影像的node，這樣就可以達到C++和Python混用的效果，只要訊息的格式是一樣的，Node之間就可以互相溝通。

這裡引用官方的動畫來說明Node的概念：
![Node](https://docs.ros.org/en/foxy/_images/Nodes-TopicandService.gif)


舉例來說，有一個叫做image_process的package，他的底下會有幾個可能的Node：
* image_publisher：負責讀取影像並且publish到影像的Topic。
* image_resizer: 負責訂閱影像的Topic，並且將影像縮放後publish到新的影像的Topic。
* image_subscriber：負責訂閱影像的Topic，並且將影像顯示出來。


整個流程會長這樣：

Video Source -> **image_publisher** -> `/image` -> **image_resizer** -> `/image_resized` -> **image_subscriber** -> Display

# 特色
這樣的設計有幾個特色：
* **可擴充性**：Node之間透過Topic、Service、Action、Parameter等方式溝通，只要訊息的格式是一樣的，Node之間就可以互相溝通，所以可以很容易在不增加複雜度的前提下增加新的功能。
* **語言彈性**：Node可以是C++ 的binary，也可以是Python的script，所以在語言的使用上是比較彈性的。
* **獨立性**：Node之間是獨立的，所以可以很容易的將Node拿出來單獨測試。同時也增加的容錯率，如果其中一個Node掛掉，其他Node不會受到影響，頂多就是少了一個功能。另外也可以單獨監測每個Node的狀態，方便除錯。


# Python Node
Python的API從`ROS`的`rospy`改成`ROS2`的`rclpy`，所以在import的時候要注意。這邊先來簡單的創一個Package和Hello World的Node。
## 創建Package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --node-name hello_world beginner_tutorials_py
```
這邊`--node-name`可以用來創建一個初始的node，並在`setup.py`內幫你寫好entrypoint，這裡是`hello_world_py`。

創建完後，會在`~/ros2_ws/src`底下看到`beginner_tutorials_py`這個資料夾，裡面會：
```bash
beginner_tutorials_py/
├── beginner_tutorials_py
│   │── __init__.py
│   └── hello_world.py
├── package.xml
├── resource
│   └── beginner_tutorials
├── test
│   │── test_copyright.py
│   └── test_flake8.py
│   └── test_pep257.py
├── setup.cfg
└── setup.py
```

其中`beginner_tutorials_py/hello_world.py`就是我們的ROS Node。

## 撰寫 Hello World Node
```python
import rclpy

def main(args=None):
    rclpy.init(args=args)   # 初始化ROS
    
    # 創建一個叫做hello_world_py的Node 
    node = rclpy.create_node('hello_world_py')  
    
    # 用Node的get_logger() function來print出Hello World!
    node.get_logger().info('Hello World!') 

    # 讓Node持續運行
    rclpy.spin(node)
    
    # 關閉ROS
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 執行
回到Workspace底下，執行：
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select beginner_tutorials_py
```
這裡用`--symlink-install`可以避免每次修改完程式碼後都要重新colcon build，可以直接執行。僅限Python，C++不行。

執行完後記得先source：
```bash
source ~/ros2_ws/install/setup.bash
```

接著就可以執行：
```bash
ros2 run beginner_tutorials_py hello_world_py
```

理論上就可以看到Timestamp + Hello World!了。不過我們有加了`rclpy.spin(node)`，所以他會一直執行，如果要結束的話，可以按`Ctrl+C`。

## 持續執行 Hello World Node
也可以讓Node每秒print一次Hello World!，只要在`hello_world.py`加上loop取代`rclpy.spin(node)`就可以了：

```python

import rclpy

def main(args=None):
    rclpy.init(args=args)   # 初始化ROS
    
    # 創建一個叫做hello_world_py的Node 
    node = rclpy.create_node('hello_world_py')  
    
    # 用Node的get_logger() function來print出Hello World!
    node.get_logger().info('Hello World!') 

    # 指定rate為1Hz
    rate = node.create_rate(1)

    # 讓Node持續運行直到ROS關閉(Ctrl+C)
    while rclpy.ok():
        node.get_logger().info('Hello World in Loop!')
        
        rclpy.spin_once(node)  # 讓Node執行一次，會等待1/rate秒

    # 關閉ROS
    rclpy.shutdown()
```

跟`ROS`不一樣的是，不需要在loop內執行`rate.sleep()`，只要事先宣告`node.create_rate`，`rclpy.spin_once(node)`就會自動等待。

這裡僅展示物件導向的寫法，指針和callback的寫法在Topic的部分會再介紹。


# Reference
---
* [Creating a package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#create-a-package)
* [Understanding nodes](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
* [rclpy API](https://docs.ros2.org/foxy/api/rclpy/api.html)