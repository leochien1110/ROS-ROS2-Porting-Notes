> 今天來延續昨天的內容，來看看如何自定義 ROS2 的 Service 吧！

# 自定義 Service 
---
自定義Service大致上跟Message一樣，只差在格式以及API的使用上有些許差異，以下將會介紹如何自定義Service，並且使用它。
之前我們做過兩個整數相加的Service使用官方的srv，這邊我們自己來做一個三個整數相加的Service。

1. 沿用昨天的`tutorial_interfaces` package，並且在裡面新增一個`srv`資料夾，並且在裡面新增一個`AddThreeInts.srv`檔案，內容如下：
    ```bash
    cd ~/ros2_ws/src/tutorial_interfaces
    mkdir srv && cd srv
    touch AddThreeInts.srv
    ```
2. 在`AddThreeInts.srv`檔案中，輸入以下內容：
    ```bash
    int64 a
    int64 b
    int64 c
    ---
    int64 sum
    ```
    * 第一行到第三行為要被相加的Request，第五行為加總後回傳的Response。
3. 編輯`CMakelists.txt`，將`.srv`加入昨天`.msg`的下方如下：
    ```cmake
    find_package(geometry_msgs REQUIRED)
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Num.msg"
        "msg/Sphere.msg"
        "srv/AddThreeInts.srv"
        DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
    )
    ```
4. 確認`package.xml`，這邊理論上昨天就寫好不需要更動。
    ```xml
    <buildtool_depend>rosidl_default_generators</buildtool_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```
5. Build `tutorial_interfaces` package
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select tutorial_interfaces
    source install/setup.bash
    ```
6. 確認Service是否有建立成功:
    ```bash
    ros2 srv show tutorial_interfaces/srv/AddThreeInts
    ```
    應該會看到以下內容:
    ```bash
    int64 a
    int64 b
    int64 c
    ---
    int64 sum
    ```

# 測試
---
我們回去更改之前Service的範例`ros2_ws/src/py_srv/py_srv/service_member_function.py`，將兩個整數相加改成三個整數相加。

```python
from tutorial_interfaces.srv import AddThreeInts     # CHANGE

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)        # CHANGE

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c                                                  # CHANGE
        self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c)) # CHANGE

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Client:
```python
from tutorial_interfaces.srv import AddThreeInts       # CHANGE
import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddThreeInts.Request()                                   # CHANGE

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.req.c = int(sys.argv[3])                  # CHANGE
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_three_ints: for %d + %d + %d = %d' %                               # CHANGE
                    (minimal_client.req.a, minimal_client.req.b, minimal_client.req.c, response.sum)) # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
檢查`package.xml`是否有:
```xml
<exec_depend>tutorial_interfaces</exec_depend>
```

## 執行
Build:
```bash
cd ~/ros2_ws
colcon build --packages-select py_srv
source install/setup.bash
```

執行Server:
```bash
ros2 run py_srv service_member_function
```

執行Client:
```bash
ros2 run py_srv client_member_function 1 2 3
```

應該會看到以下結果:
```bash
[INFO] [minimal_service]: Service [/add_three_ints] added
[INFO] [minimal_client_async]: Waiting for service to be available...
[INFO] [minimal_client_async]: Result of add_three_ints: for 1 + 2 + 3 = 6
```

# 總結
---
到這邊可以看到自定義Message和Service上沒有太大的差異，僅多了Request和Response的部分，Message Fieldtype也是通用到Service的，可以參考昨天的表格。

至於`ROS`和`ROS2`的Message和Service的差異，主要變動是在`CMakeLists.txt`上，不需要再寫`add_message_files`, `add_service_files`, 和`generate_messages`，只需要寫`rosidl_generate_interfaces`就可以了，嚴格來說是簡化了不少流程。而`.msg`和`.srv`也都可以寫在同一個`rosidl_generate_interfaces`中，不需要分開寫。`package.xml`也有做語法上的更新，但也就是按照昨天的寫法把Macro放上去就可以了。

# Reference
---
* [ROS2 - Creating custom msg and srv files](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
* [ROS - Creating a ROS msg and srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
* [Implementing custom interfaces](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html)
* [ROS Interfaces](https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html)
* [ROS Common Interface Github](https://github.com/ros2/common_interfaces/tree/foxy)
* [ROS2 API - msgs & srvs](https://docs.ros2.org/foxy/api/)