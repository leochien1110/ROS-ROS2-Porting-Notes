> 終於來到實作的部分了，今天要來實作一個簡單的 Publisher，並且將它發布到 ROS 環境中。發現我的文章都有點太過細節，這點要加油，不然會勸退好多人的感覺。

# Publisher
---
由於官方就有提供`Publisher`和`Subscriber`的[範例](https://github.com/ros2/examples/tree/foxy/rclpy/topics)，就不用從頭撰寫了。詳細流程也可次參考[官方教學](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)。

1. 這邊先建立一個Package
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python py_pubsub
    ```
2. 再來，把Node下載到對應的地方
   ```bash
   cd ros2_ws/src/py_pubsub/py_pubsub
   wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
   ```
3. 接著打開`publisher_member_function.py`就可以看到完整的程式碼
   ```python
   import rclpy
    from rclpy.node import Node

    from std_msgs.msg import String


    class MinimalPublisher(Node):

        def __init__(self):
            super().__init__('minimal_publisher')
            self.publisher_ = self.create_publisher(String, 'topic', 10)
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.i = 0

        def timer_callback(self):
            msg = String()
            msg.data = 'Hello World: %d' % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1


    def main(args=None):
        rclpy.init(args=args)

        minimal_publisher = MinimalPublisher()

        rclpy.spin(minimal_publisher)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_publisher.destroy_node()
        rclpy.shutdown()


    if __name__ == '__main__':
        main()
    ```
## 解析
```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
```
這邊引入了`rclpy`和`Node`，`rclpy`是`ROS2`的Python API，`Node`是`rclpy`中的一個class，用來建立Node。再來引入了`std_msgs`中的`String`，這是一個常用的String Message，用來傳送字串。

```python
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
```
再來是將Publisher建立成一個Node的class。首先用`super`來初始化繼承的Node，並且給予Node一個名稱`minimal_publisher`。接著用`self.create_publisher`來建立一個Publisher。`create_publisher`的參數有三個，第一個是Message的型別(`String`)，第二個是Topic的名稱(`'topic'`)，第三個是QoS(queue depth=10)，如果是設定queue depth的話可以直接塞`int`的數字，其他QoS選項的話則要使用QoS的物件，後面會再介紹。

接著是建立一個Timer，這邊的Timer是用來定時發布訊息的，第一個參數是時間間隔，這邊是0.5秒發布一次訊息(2Hz)。第二個參數是Timer的callback function，每次Timer觸發時，就會執行。最後是一個計數器`self.i`，用來計算發布了幾次訊息。

```python
def timer_callback(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1
```
這邊是Timer的callback function，每次Timer觸發時，就會執行這個function。首先建立一個`String`的Message，並且將`self.i`的值放入Message中。

[String Message](https://github.com/ros2/common_interfaces/blob/foxy/std_msgs/msg/String.msg)的格式為：
```
string data
```

接著用`self.publisher_.publish(msg)`來發布訊息，`self.publisher_`就是上面建立的Publisher。最後用`self.get_logger().info('Publishing: "%s"' % msg.data)`來印出訊息。

`self.get_logger()`是用來取得Node的Logger，`info`是用來印出訊息的function。這邊還有其他不同level的logger，像是`debug`、`warn`、`error`、`fatal`等等，可以依照需求使用。
> Logger補充：可使用`self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)`來設定logger的level，預設是`INFO`。亦或是在執行`ros2 run`時加上`--ros-args --log-level DEBUG`來設定。

```python
self.get_logger().info('Show logger anyway')
self.get_logger().debug('Show logger only when debug')
self.get_logger().warn('Show logger only when warn')
self.get_logger().error('Show logger only when error')
self.get_logger().fatal('Show logger only when fatal')
```
```bash
```

回到程式碼：
```python
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```
最後就是main function，這邊先用`rclpy.init(args=args)`來初始化`rclpy`，接著建立一個`MinimalPublisher`的物件，並且用`rclpy.spin(minimal_publisher)`來讓Node執行，直到Node被關閉。最後用`minimal_publisher.destroy_node()`來關閉Node，如不關閉Node，程式結束時依然會自動關閉Node。最後用`rclpy.shutdown()`來關閉`rclpy`。

## Package設定
在Build之前不要忘了要更新`package.xml`, `setup.py`和`setup.cfg`。首先將dependency加入`package.xml`，這邊需要`rclpy`和`std_msgs`
```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```
接著是`setup.py`，這邊要加入`entry_points`，這是用來讓`ROS2`知道Node在哪裡
```python
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```
最後檢查一下`setup.cfg`，確認可執行檔是否在`lib/py_pubsub`中，以便`ros2 run`可以找到
```
[develop]
script-dir=$base/lib/py_pubsub
[install]
install-scripts=$base/lib/py_pubsub
```

# Subscriber
---
下載Subscriber的範例
```bash
cd ~/ros2_ws/src/py_pubsub/py_pubsub
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```
接著打開`subscriber_member_function.py`就可以看到完整的程式碼
```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
## 解析
```python
def MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
```

這邊是Subscriber的class，首先用`super`來初始化繼承的Node，並且給予Node一個名稱`minimal_subscriber`。接著用`self.create_subscription`來建立一個Subscriber。`create_subscription`的參數有四個，第一個是Message的型別(`String`)，第二個是Topic的名稱(`'topic'`)，第三個是callback function，第四個是QoS(queue depth=10)。


最後先呼叫一次`self.subscription`，這是為了避免沒有宣告使用subscription而產生`unused variable warning`。

```python
def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
```
這邊是callback function，當subscriber一收到訊息時，就會執行這個function，並且印出`Publisher`發布的訊息(訊息時間和個數)。


```python
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
```
最後就是subscriber的main function，和publisher完全一樣，就不再贅述。

## Package設定
因為是在同一個package中，dependencies也和publisher一樣，就不需要調整。唯一要調整的是`setup.py`，這邊要加入`entry_points`，這是用來讓`ROS2`知道Node在哪裡
```python
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```

# Build and Run
## Build
養成好習慣，在workspace時可以先跑`rosdep`來檢查一下dependency是否有安裝
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro foxy -y
```
接著build package
```bash
colcon build --packages-select py_pubsub
```
最後不要忘了source setup讓`ROS2`知道package的位置
```bash
source ~/ros2_ws/install/setup.bash
```
## Run
接著就可以執行Node了，首先在一個terminal執行
```bash
ros2 run py_pubsub talker
```
就可以看到publisher開始發布訊息了，應該會類似下面這樣，分別是[Log Level]、[時間]、[Node名稱]：[訊息]
```bash
[INFO] [1600000000.000000000] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1600000000.500000000] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [1600000001.000000000] [minimal_publisher]: Publishing: "Hello World: 2"
...
```

接著在另一個terminal執行
```bash
ros2 run py_pubsub listener
```
就可以看到subscriber開始接收訊息了。應該可以看到類似下面的訊息
```bash
[INFO] [1600000000.000000000] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [1600000000.500000000] [minimal_subscriber]: I heard: "Hello World: 1"
[INFO] [1600000001.000000000] [minimal_subscriber]: I heard: "Hello World: 2"
...
```
不過因為Subscriber是後開，只會收到開始跑之後的訊息，除非要去設定QoS。另外也可以先跑Subscriber，再跑Publisher，這樣就可以收到所有的訊息。

另外Publisher的部分也可以用指令來執行，會等Service介紹完後，一起介紹。

# QoS
---

當然也可以自行設定QoS，或是更改QoS Profile成Reliable，depth為10。
## Publisher
```python
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        )
        self.publisher_ = self.create_publisher(String, 'topic', qos_profile)
```

## Subscriber
```python
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        )
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            qos_profile)
```

其中Publisher和Subscriber的reliability的相容性必需要符合規定，否則將收不到訊息，可以參考昨天的[Day8 ROS2 Topic
](https://ithelp.ithome.com.tw/articles/10323283)中**QoS相容性**表格。


# Reference
---
* [Wrtie a simple publisher and subscriber(Python)](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
* [rclpy topic exmaple](https://github.com/ros2/examples/tree/foxy/rclpy/topics)
* [ros2 demo](https://github.com/ros2/demos/tree/master/demo_nodes_py/demo_nodes_py/topics)
* [Taiyou Kou - ROS2 QoS](https://hackmd.io/@st9540808/r1zrNKBWU/%2F%401IzBzEXXRsmj6-nLXZ9opw%2FBkaxoWRiI)