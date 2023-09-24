> 今天用Service Server 和 Client來寫個簡單的加法器，Server會接收兩個數字，然後回傳兩個數字的和。

# Service Server Node
Service Server的建立和Subscriber很像，是屬於被動的Node，會等待Client的請求，然後回傳結果。直接來看看程式碼吧。

1. 首先創建一個新的Python Package叫做`py_srv`
    ```bash
    ros2 pkg create --build-type ament_python py_srv --dependencies rclpy example_interfaces
    ```
    `--dependencies`會自動將相依寫進`package.xml`，這裡除了`rclpy`之外，還需要`example_interfaces`，因為我們要用到`std_srvs`的[`AddTwoInts.srv`](https://github.com/ros2/example_interfaces/blob/foxy/srv/AddTwoInts.srv)，官方範例的訊息格式。這邊介紹一下`AddTwoInts.srv`的格式：
    ```bash
    int64 a
    int64 b
    ---
    int64 sum
    ```
    前面兩行是request，`---`分隔線後面一行是response。
2. 別忘了更新`package.xml`和`setup.py`內的description, maintainer, license等等資訊。
3. 接著在`ros2_ws/src/py_srv/py_srv`下新增檔案`service_member_function.py`:
    ```python
    from example_interfaces.srv import AddTwoInts

    import rclpy
    from rclpy.node import Node


    class MinimalService(Node):

        def __init__(self):
            super().__init__('minimal_service')
            self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

        def add_two_ints_callback(self, request, response):
            response.sum = request.a + request.b
            self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

            return response


    def main(args=None):
        rclpy.init(args=args)

        minimal_service = MinimalService()

        rclpy.spin(minimal_service)

        rclpy.shutdown()


    if __name__ == '__main__':
        main()
    ```
## 解析
```python
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node
```
除了我們熟悉的`rclpy`之外，我們還需要`example_interfaces.srv`，這邊是ROS2的官方範例的訊息格式`AddTwoInts.srv`。

```python
class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
```
接著是建構子的部分，先初始化繼承的`Node`。接著使用`Node`的`create_service`建立Service，這邊的參數分別是srv的格式`AddTwoInts`、Service的名稱`add_two_ints`、還有callback function`add_two_ints_callback`。

其中`create_service`後面還有兩個參數[`create_service(srv_type, srv_name, callback, *, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)`](https://docs.ros2.org/foxy/api/rclpy/api/node.html#rclpy.node.Node.create_service)，分別是QoS Profile和callback_group，可以做更進階的設定，像是設定QoS的deadline等等，以及多個額外的callback functions，可以自己玩玩看。

```python
def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

    return response
```
callback function的參數固定會是`request`和`response`，而request內則是依照srv的格式可以有多個attributes，這邊有`a`和`b`兩個變數。而response則是回傳的結果也可以有多個attributes，這邊只有`sum`一個變數。最後回傳response。

```python
def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()
```
最後就是main function，這邊和Publisher一樣，先初始化`rclpy`，接著建立`MinimalService`，最後使用`rclpy.spin`來讓Node持續運行，最後再關閉`rclpy`。

## Package 設定
由於我們已經在前面設定好dependencies，因此只需要在`setup.py`中加入entry_points即可。這邊將`service_member_function`加入entry_points並命名為`service_member_function`已讓`ros2 run`可以找到:
```python
entry_points={
    'console_scripts': [
        'service_member_function = py_srv.service_member_function:main',
    ],
},
```

# Service Client Node
Service Client的則是和Publisher很像，是主動的Node，會主動的發送請求給Server，然後等待回傳結果。

我們在`ros2_ws/src/py_srv/py_srv`下新增檔案`client_member_function.py`:
```python
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
```

## 解析
```python
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
```
這邊多引入了`sys`，因為我們要從command line中使用`sys.argv`來取得參數。

```python
class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
```
建構子的部分，先初始化繼承的`Node`，接著使用`Node`的`create_client`建立Service Client，這邊的參數分別是srv的格式`AddTwoInts`、Service的名稱`add_two_ints`。

```python       
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()
```
這邊是等待Service Server，如果Server沒有啟動，就會一直等待，直到timeout_sec秒數到期。這邊的timeout_sec可以自己設定，預設是1秒。接著初始化request。接著建立一個`AddTwoInts`的Request物件。

```python
    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```
這個function會將request的兩個變數`a`和`b`設定好，接著使用`call_async`來發送非同步的請求以便程式可以繼續運行，並且使用`spin_until_future_complete`來等待回傳結果。最後回傳結果。
> 這邊官方有寫錯，並不是`__inti__`的`while` loop在檢查`future`而是`rclpy.spin_until_future_complete`。

```python
def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()
```
這邊會讀取使用者的輸入，並且將輸入的兩個數字傳送給`send_request`，接著將回傳的結果印出來。

## Package 設定
這邊需要在`setup.py`中加入將`client_member_function`加入entry_points並命名為`client_member_function`:

```python
entry_points={
    'console_scripts': [
        'service_member_function = py_srv.service_member_function:main',
        'client_member_function = py_srv.client_member_function:main',
    ],
},
```

# Build and Run
## Build
首先我們要先安裝dependencies，尤其這邊我們需要還沒安裝`example_interfaces`:
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro foxy -y
```
接著build package:
```bash
colcon build --packages-select py_srv
```
最後Source:
```bash
source install/setup.bash
```

## Run
首先開啟一個terminal，執行Service Server:
```bash
ros2 run py_srv service_member_function
```
這個Service Server會等待Client的請求...

接著開啟另一個terminal，執行Service Client:
```bash
ros2 run py_srv client_member_function 1 2
```
就可以看到client 收到server的回傳結果了:
```bash
[INFO] [minimal_client_async]: Result of add_two_ints: for 1 + 2 = 3
```
同時也可以看到server的terminal印出了request的內容:
```bash
[INFO] [minimal_service]: Incoming request
a: 1 b: 2
```

其實Client的部分也跟Publisher一樣可以用指令來執行，會跟Topic統一到後面一起介紹。


# ROS vs. ROS2
|說明| ROS    | ROS2 |
| :----| :-----  | :---- |
| service API | 在`rospy`底下 | 在`rclpy.node.Node`底下 |
| service object 建立 | `rospy.Service(srv_name, srv_type, callback)` | `self.create_service(srv_type, srv_name, callback)` |
| client API | 在`rospy`底下 | 在`rclpy.node.Node`底下 |
| client object 建立 | `rospy.ServiceProxy(srv_name, srv_type)` | `self.create_client(srv_type, srv_name)` |
| `.srv` 格式 | 一樣 | 一樣 |


# Reference
---
* [ROS - Writing a Simple Service and Client](http://wiki.ros.org/rospy_tutorials/Tutorials/WritingServiceClient)
* [ROS2 - Writing a simple service and client (Python)](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
* [蛤 - 使用Python撰寫Server Node](https://ithelp.ithome.com.tw/articles/10206931)
* [蛤 - 使用Python撰寫Client Node](https://ithelp.ithome.com.tw/articles/10207083)