> 終於來到第20天，要開始講解如何使用Launch檔案來啟動ROS2的節點了，這個部分是ROS2的重點，也是ROS2的特色之一，讓我們來看看如何使用吧！

# Launch
---
前面跑`ros2 run`時，一個node就要開一個terminal，在Node不多的情況下還好，但常常到專案後期，Node會越來越多，彼此間的namespace也可能互相關聯，這時候Launch就派上用場了。

在`ROS`中，Launch檔案是用XML來寫，但是`ROS2`則是新增了Python和YAML來寫，這樣的好處是可以直接使用Python的語法來寫Launch檔案或是YAML高可讀性。YAML寫起來比XML更侷促些，而Python可以花式開Node，譬如用for loop來開啟多個Node，這是XML做不到的，不過複雜度也相對高了一點。所以對於簡單的Launch檔案，我們會用XML或YAML來寫，複雜的Launch檔案則會用Python來寫，而且他們彼此是可以互相include的。

> YAML的話目前為止都沒有看過較大的專案使用過，先不介紹了，等之後如果有機會再補上。

## XML
首先來介紹launch xml的基本架構
Simeple Node:
```xml
<launch>
    <node pkg="package_name" exec="node_name" name="node_name" namespace="ns" output="screen" args="arg1 arg2" />
</launch>
```
Node with Parameter and Remap:
```xml
<launch>
    <node pkg="package_name" exec="node_name" name="node_name" namespace="ns" output="screen">
        <param name="param_name" value="param_value" />
        <remap from="from_topic" to="to_topic" />
    </node>
</launch>
```
* `launch` - 最外層的tag，所有的launch檔案都要有這個tag
* `node` - 用來開啟一個node，可以指定package name、node name、namespace、output、args等等
* `param` - 用來設定參數value，對應到`ros2 run pkg node --ros-args -p param_name:=param_value`
* `remap` - 用來重新對應topic name，對應到`ros2 run pkg node --ros-args -r from_topic:=to_topic`
  
跟`ROS`一樣，launch內可以設定變數argument，並用`$(var argument_name)`來代換，可以增加launch的彈性，可以在`ros2 launch`時更換argument，範例如下：
```xml
<launch>
    <!-- Argument 代換 -->
    <arg name="node_name" default="camera1" />
    <!-- 尋找package下param.yaml路徑 -->
    <arg name="param_file" default="$(find-pkg-share <pkg-name>)/param/param.yaml" />
    
    <node = pkg="package_name" exec="node_name" name="$(var node_name)">
        <param from="$(var param_file)" />
    </node>
</launch>
```
* `arg` - 用來設定argument
* `$(find-pkg-share <pkg-name>)` - 用來找尋package在`install/share`底下的路徑。這樣可以幫助找到package底下特定檔案的路徑，譬如`param.yaml`
* `$(var argument_name)` - 用來代換argument，這邊的`argument_name`是指argument的名稱，譬如`node_name`和`param_file`
* 更詳細的Tag可以看[官方的文件](https://design.ros2.org/articles/roslaunch_xml.html#launch-tag)。

### Include
和`ROS`一樣可以include其他的launch檔案，
```xml
<launch>
    <include file="$(find-pkg-share <pkg-name>)/launch/camera.launch.xml" />
</launch>
```
不過namespace的功能被移除了。workaround如下：
```xml
<launch>
    <group>
        <push-ros-namespace namespace="camera1"/>
        <include file="$(find-pkg-share <pkg-name>)/launch/camera.launch.xml" />
    </group>
</launch>
```


### 判斷式
雖然XML比較簡單，但還是可以做一些簡單的判斷式，譬如`if`和`unless`，範例如下：
```xml
<launch>
    <arg name="node_name" default="camera1" />

    <!-- 判斷式 -->
    <node if="$(arg node_name) == 'camera1'" pkg="package_name" exec="node_name" name="camera1" />
    
    <!-- 如果node_name不是camera1，則叫做camera2 -->
    <node unless="$(arg node_name) == 'camera1'" pkg="package_name" exec="node_name" name="camera2" />
    
</launch>
```
* `if` - 如果條件成立，則執行
* `unless` - 如果條件不成立，則執行


### 執行
執行launch的指令是`ros2 launch package_name launch_file.py`。

至於要代換arguments的話，可以用`arg_name:=arg_value`
來更換node name。拿上面代換的例子來說，launch的指令會像是這樣：
```bash
ros2 launch package_name launch_file.py node_name:=camera2 param_file:=/home/user/ros2_ws/src/package_name/param/param.yaml
```

### 範例
拿之前的`talker`來做簡單的範例，假設我們要開啟兩個`talker`，一個叫做`talker1`，另一個叫做`talker2`，並且`talker1`的topic name要改成`chatter1`，`talker2`的topic name要改成`chatter2`。
1. 首先在`ros2_ws/src/py_pubsub`底下新增一個`launch`資料夾，並在裡面新增一個`two_pub.xml`:
    ```xml
    <launch>
        <node pkg="py_pubsub" exec="talker" name="talker1" output="screen">
            <remap from="topic" to="chatter1" />
        </node>
        <node pkg="py_pubsub" exec="talker" name="talker2" output="screen">
            <remap from="topic" to="chatter2" />
        </node>     
    ```
2. 接著再新增一個`two_sub.xml`:
    ```xml
    <launch>
        <node pkg="py_pubsub" exec="listener" name="listener1" output="screen">
            <remap from="topic" to="chatter1" />
        </node>
        <node pkg="py_pubsub" exec="listener" name="listener2" output="screen">
            <remap from="topic" to="chatter2" />
        </node>
    </launch>
    ```
3. 更新`CMakeLists.txt`:
    ```cmake
    ...
    install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
    )
    ```
4. 更新`package.xml`:
    ```xml
    ...
    <exec_depend>ros2launch</exec_depend>
    ```
    這樣可以確保`ros2 launch`在build完後可以正常使用。
5. 用symlink重新build:
    ```bash
    colcon build --symlink-install --packages-select py_pubsub
    source install/setup.bash
    ```
    這樣launch就會以symlink的方式連結到`install`底下，這樣修改launch檔案時，`install`底下的launch檔案也會跟著改變。
6. 接著來執行看看：
    ```bash
    ros2 launch py_pubsub two_pub.xml
    ```
    可以看到兩個`talker`都開啟了，並且topic name也都改變了：
    ```bash
    [INFO] [talker1]: Publishing: 'Hello World: 1'
    [INFO] [talker2]: Publishing: 'Hello World: 1'
    ...
    ```
再來開啟兩個新的terminal來執行`two_sub.xml`:
```bash
ros2 launch py_pubsub two_sub.xml
```
可以看到兩個`listener`都開啟了，並且topic和node name也都改變了：
```bash
[INFO] [listener1]: I heard: [Hello World: 1]
[INFO] [listener2]: I heard: [Hello World: 1]
...
```
## Python
接著來介紹Python Launch的基本架構：
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="package_name",
            executable="node_name",
            name="node_name",
            namespace="ns",
            output="screen",
            emulate_tty=True,
            args=["arg1", "arg2"],
            parameters=[
                {"param_name": "param_value"}
            ],
            remappings=[
                ("from_topic", "to_topic")
            ]
        )
    ])
# 或是用list來包含多個Node as action
def generate_launch_description():
    ld = LaunchDescription()
    first_node = Node(
        package="package_name",
        executable="node_name",
        name="node_name1",
        output="screen",
    )
    second_node = Node(
        package="package_name",
        executable="node_name",
        name="node_name2",
        output="screen",
    )
    ld.add_action(first_node)
    ld.add_action(second_node)
    return ld
```

基本架構和XML差不多，只是用Python的語法來寫。
* `from launch import LaunchDescription` - 用來import LaunchDescription，是`launch.py`的基本單位
* `from launch_ros.actions import Node` - 用來import Node
* `generate_launch_description()`回傳值是LaunchDescription物件，可以是一個LaunchDescription的list，裡面可以放很多個Node
* `Node` - 宣告Node的物件，可以指定package name、node name、namespace、output、args等等的參數。其中`output="screen"`會print ROS的log，而`emulate_tty`則是print所有的訊息。
* `parameters` - 用來設定參數value，對應到`ros2 run pkg node --ros-args -p param_name:=param_value`
* `remappings` - 用來重新對應topic name，對應到`ros2 run pkg node --ros-args -r from_topic:=to_topic`

Python的變數代換就更直覺了，不過如果要接收`ros2 launch`指令必須要用`LaunchConfiguration`，範例如下：
```python
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    node_name = LaunchConfiguration("node_name", default="camera1")
    param_file = LaunchConfiguration("param_file", default=FindPackageShare("package_name") + "/param/param.yaml")

    # Following line DOES NOT WORK!!! Please check下方個人筆記的部分
    # print("node_name: {}".format(node_name))

    return LaunchDescription([
        Node(
            package="package_name",
            executable="node_name",
            name=node_name,
            parameters=[param_file],
            output="screen",
        )
    ])
```
* `LaunchConfiguration` - 用來設定argument，可以從`ros2 launch`來更換argument。等同於XML的`arg`
* `FindPackageShare` - 用來找尋package在`install/share`底下的路徑。這樣可以幫助找到package底下特定檔案的路徑，譬如`param.yaml`。等同於XML的`$(find-pkg-share <pkg-name>)`

### Include
Python的Launch檔案也可以include其他的Launch檔案，例如我們要include `camera.launch.py`並且更改node name，範例如下：
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
    launch_file = LaunchConfiguration("launch_file", default=FindPackageShare("package_name") + "/launch/camera.launch.py")

    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_file),
            launch_arguments={
                "node_name": "camera1"
            }.items()
        )
    ])
```
> 可以看到比XML複雜許多，所以若沒有特殊需求還是推薦用XML來寫。

首先必須要找到launch file的路徑，可以用上面提到的`FindPackageShare`。接著用`IncludeLaunchDescription`來include launch file，並且可以用`launch_arguments`來更改launch file內的argument。

如果不知道Launch的格式可以用`AnyLaunchDescriptionSource`來自動判斷，如果知道Launch的格式可以用`PythonLaunchDescriptionSource`或`XmlLaunchDescriptionSource`來指定。

再來是namespace的問題，這邊可以用`GroupAction`和`PushRosNamespace`來解決，範例如下：
```python
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import GroupAction

def generate_launch_description():
    launch_file = LaunchConfiguration("launch_file", default=FindPackageShare("package_name") + "/launch/camera.launch.py")

    return LaunchDescription([
        GroupAction([
            PushRosNamespace("camera1"),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(launch_file),
                launch_arguments={
                    "node_name": "camera1"
                }.items()
            )
        ])
    ])
```

### 判斷式
這邊有個問題，就是`LaunchConfiguration`拿到的物件並不是string，不能直接像一般Python用if-else。如果要做判斷式，必須要用`launch.conditions`，範例如下：
```python
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    node_name = LaunchConfiguration("node_name", default="camera1")

    return LaunchDescription([
        Node(
            package="package_name",
            executable="node_name",
            name="camera1",
            parameters=[param_file],
            output="screen",
            condition=IfCondition(node_name == "camera1")
        ),
        Node(
            package="package_name",
            executable="node_name",
            name="camera2",
            output="screen",
            condition=UnlessCondition(node_name == "camera1")
        )
    ])
```

### 範例
拿之前的`talker`來做簡單的範例，假設我們要開啟兩個`talker`，一個叫做`talker1`，另一個叫做`talker2`，並且`talker1`的topic name要改成`chatter1`，`talker2`的topic name要改成`chatter2`。
1. 首先在`ros2_ws/src/py_pubsub/launch`資料夾新增一個`two_pub.launch.py`:
    ```python
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package="py_pubsub",
                executable="talker",
                name="talker1",
                output="screen",
                remappings=[
                    ("topic", "chatter1")
                ]
            ),
            Node(
                package="py_pubsub",
                executable="talker",
                name="talker2",
                output="screen",
                remappings=[
                    ("topic", "chatter2")
                ]
            )
        ])
    ```
2. 接著再新增一個`two_sub.launch.py`:
    ```python
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package="py_pubsub",
                executable="listener",
                name="listener1",
                output="screen",
                remappings=[
                    ("topic", "chatter1")
                ]
            ),
            Node(
                package="py_pubsub",
                executable="listener",
                name="listener2",
                output="screen",
                remappings=[
                    ("topic", "chatter2")
                ]
            )
        ])
    ```
3. 如果跳過XML的範例，記得要回去看如何更新`CMakeLists.txt`和`package.xml`。
4. 用symlink重新build:
    ```bash
    colcon build --symlink-install --packages-select py_pubsub
    source install/setup.bash
    ```
5. 接著來執行talkers：
    ```bash
    ros2 launch py_pubsub two_pub.launch.py
    ```
    可以看到兩個`talker`都開啟了，並且topic name也都改變了：
    ```bash
    [INFO] [talker1]: Publishing: 'Hello World: 1'
    [INFO] [talker2]: Publishing: 'Hello World: 1'
    ...
    ```
6. 再來開啟兩個新的terminal來執行`two_sub.launch.py`:
    ```bash
    ros2 launch py_pubsub two_sub.launch.py
    ```
    可以看到兩個`listener`都開啟了，並且topic和node name也都改變了：
    ```bash
    [INFO] [listener1]: I heard: [Hello World: 1]
    [INFO] [listener2]: I heard: [Hello World: 1]
    ...
    ```
## YAML
TODO - 之後開始比較多人使用時再補上XD

# ROS vs. ROS2
---
| 說明 | ROS | ROS2 |
| :-------- | :-------- | :-------- |
| Launch | XML | XML, Python, YAML |
| XML element 對照 | `ns` <br>name <br>pkg <br>`type` <br>args <br>\$(`arg` arg1) <br>\$(`find` pkg) | `namespace` <br>name <br>pkg <br>`exec` <br>args <br>\$(`var` arg1) <br>\$(`find-pkg-share` pkg) |
| Include with namespace | 可以用`ns` | 沒有`ns`，但可以用`group + push-ros-namespace`和`action_group`繞過 |



# 不要看(個人筆記)

## Argument -> String in Python launch

上面Python用`launch.conditions.IfCondition()`來做判斷式，
但僅能對**ROS**的物件做操作，如果我們需要用傳統的if-else，則必續用`OpaqueFunction`+`DeclareLaunchArgument`+`context.perform_substitution`等比較複雜的方式，這邊筆記一下。

假設我們需要把相機和光達裝在車上，但車隊有不同的車種，不同車上的有不同的配置，有的只有相機，有的只有光達，有的全部感測器都有，我們又需要print出來debug，這時候就可以用傳統的if-else了，範例如下：

```python
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode

from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource


def launch_camera(args)->list:
    ld = []
    camera_num = args['camera_num']
    
    camera_launch_file = os.path.join(get_package_share_directory('gscam'), 'launch', 'v4l2.launch.xml')
    
    for i in range(camera_num):
        camera_info = 'file://' + os.path.join(get_package_share_directory('sensor-launch'), 'config', project_name, 'camera' + str(i) + '.yaml')
        camera_name = 'camera' + str(i)
        camera_launch = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(camera_launch_file),
            launch_arguments={'camera_name': camera_name,
                                'camera_info_url': camera_info,
                                'DEVICE': '/dev/video'+str(i),
                                }.items(),
        )
        camera_launch_with_namespace = GroupAction([
            PushRosNamespace(args['vehicle_name']+'/camera'+str(i)),
            camera_launch,
        ])
        ld.append(camera_launch_with_namespace)
        
    return ld

def launch_lidar(args)->list:
    ...
    return ld

def launch_setup(context, *args, **kwargs):
    
    ld = []

    # get the arguments
    vehicle_name = context.perform_substitution(LaunchConfiguration('vehicle_name'))
    vehicle_type = context.perform_substitution(LaunchConfiguration('vehicle_type'))
    camera_on = context.perform_substitution(LaunchConfiguration('camera_on'))
    lidar_on = context.perform_substitution(LaunchConfiguration('lidar_on'))
    viz = context.perform_substitution(LaunchConfiguration('viz'))
    
    print('---Launch Arguments---')
    print(f'vehicle_name: {vehicle_name}\nvehicle_type: {vehicle_type}\ncamera_on: {camera_on}\nlidar_on: {lidar_on}\nviz: {viz}')
    print('----------------------')

    """
    Configuration
    """
    package_dir = os.path.join(get_package_share_directory('sensor-launch'))
    config_dir = os.path.join(package_dir, 'config', project_name)

    camera_num = 5

    # wrap arguments above into dictionary
    args = {'vehicle_name': vehicle_name,
            'vehicle_type': vehicle_type,
            'camera_on': camera_on,
            'camera_num': camera_num,
            'lidar_on': lidar_on,
            'viz': viz,
            'package_dir': package_dir,
            'config_dir': config_dir,
            }
    
    # launch cameras
    if camera_on is 'true':
        print('launching cameras')
        ld.extend(launch_camera(args))
    
    # launch velodyne
    if lidar_on is 'true':
        print('launching lidar')
        ld.extend(launch_velodyne(args))
    
    # launch rviz2 with config file
    print('rviz: ', viz)
    if viz == 'true':
        print('launching rviz2 with config file')
        ld.append(launch_rviz2(args))
    
    return ld

def generate_launch_description():
    """
    Variables
    """
    declare_arguments = []
    declare_arguments.append(
        DeclareLaunchArgument(name='vehicle_name', default_value='toyota',
                              description='vehicle name'))
    declare_arguments.append(
        DeclareLaunchArgument(name='vehicle_type', default_value='corolla',
                              description='vehicle type'))
    declare_arguments.append(
        DeclareLaunchArgument(name='camera_on', default_value='false',
                              description='turn on camera'))
    declare_arguments.append(
        DeclareLaunchArgument(name='lidar_on', default_value='false',
                                description='turn on lidar'))
    declare_arguments.append(
        DeclareLaunchArgument(name='viz', default_value='false',
                                description='visualize data in rviz2'))
    
    return LaunchDescription(declare_arguments + [OpaqueFunction(function=launch_setup)])
```
這樣的話，`generate_launch_description()`裡面就只失接收argument，然後用`OpaqueFunction`把argument包成context塞進`launch_setup()`，`launch_setup()`再用`context.perform_substitution`來取得argument的字串，這樣就可以用傳統的if-else來做判斷了。

## Component Node in Python launch
In `ROS2`, `nodelet` has been replace with `component`, which can also performance zero-copy behavior. 

For **Foxy** and earlier version, there is **NO** `component_node` element in `.launch.xml`. The workaround is to run a `ros2 run` within the `executable` element. 

Take Velodyne driver example, `ROS2` **Humbler** or later:

```xml
<load_composable_node target="velodyne_container">
    <composable_node pkg="velodyne_driver" plugin="velodyne_driver::VelodyneDriver" name="velodyne_driver_node">
        <param from="$(var driver_param_file)" />
    </composable_node>
    <composable_node pkg="velodyne_pointcloud" plugin="velodyne_pointcloud::Convert" name="velodyne_convert_node" namespace="">
        <param from="$(var pointcloud_param_file)" />
        <param name="calibration" value="$(var lidar_calibration_file)" />
    </composable_node>
</load_composable_node>
```

But for **Foxy** or earlier you have to use `ros2 component` command and there is not way to load `param_file`. The parameters has to be load manually thru cmd:

```xml
<executable cmd="ros2 component load /velodyne_container velodyne_driver velodyne_driver::VelodyneDriver 
                     -p gps_time:=$(var gps_time)
                     -p time_offset:=$(var time_offset)
                     -p enabled:=$(var enabled)
                     -p read_once:=$(var read_once)
                     -p read_fast:=$(var read_fast)
                     -p repeat_delay:=$(var repeat_delay)
                     -p frame_id:=$(var frame_id)
                     -p model:=$(var model)
                     -p rpm:=$(var rpm)
                     -p port:=$(var port)"
             output="screen"/>
```

Otherwise, you have to use `.launch.py`:

```python
velodyne_driver = ComposableNodeContainer(
        name='velodyne_driver_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='velodyne_driver',
                plugin='velodyne_driver::VelodyneDriver',
                name='velodyne_driver_node',
                parameters=[driver_params]),
        ],
        output='both',
)
```


# Reference
---
* [Creating a Launch File](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
* [Migrating launch files from ROS 1 to ROS 2](https://docs.ros.org/en/foxy/How-To-Guides/Launch-files-migration-guide.html)
* [Using Python, XML, and YAML for ROS 2 Launch Files](https://docs.ros.org/en/foxy/How-To-Guides/Launch-file-different-formats.html)