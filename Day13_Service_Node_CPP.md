> 今天要來寫Service C++ Node囉！有點不太了解為什麼官方範例在Python時用Class，C++卻只寫Main Function，在想是不是看起來比較簡潔。就當成學兩種寫法吧，一種把Service用Class包起來，一種直接寫在Main Function裡面。


# Service Server Node
1. 首先要來創建一個新的C++ Package叫做`cpp_srv`
    ```bash
    ros2 pkg create --build-type ament_cmake cpp_srv --dependencies rclcpp example_interfaces
    ```
    `--dependencies`會自動將相依寫進`package.xml`和`CMakeLists.txt`。這裡除了`rclcpp`之外，還需要`example_interfaces`，因為我們要用到`std_srvs`的[`AddTwoInts.srv`](https://github.com/ros2/example_interfaces/blob/foxy/srv/AddTwoInts.srv)。這邊在介紹一次給那些跳過Python的人，`AddTwoInts.srv`的格式：
    ```bash
    int64 a
    int64 b
    ---
    int64 sum
    ```
    前面兩行是request，`---`分隔線後面一行是response。
2. 別忘了更新`package.xml`和`CMakeLists.txt`內的description, maintainer, license等等資訊。
3. 在`ros2_ws/src/cpp_srv/cpp_srv`下新增檔案`add_two_ints_server.cpp`:
    ```cpp
    #include "rclcpp/rclcpp.hpp"
    #include "example_interfaces/srv/add_two_ints.hpp"

    #include <memory>

    void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
            std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
    {
    response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                    request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
    }

    int main(int argc, char **argv)
    {
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
        node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    }
    ```

## 解析
這邊就跳過`include`和`namespace`的部分，直接看`add`:
```cpp
void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
         std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
    response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
        request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}
```
整體架構和Python的`add_two_ints_callback`函示一樣，不過這邊C++用Pass by Pointer的方式，所以可以直接修改`response`的值，而Python則是用return的方式回傳。另外，C++的`request`和`response`都是`std::shared_ptr`，所以要用`->`來取值。

```cpp
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");
```
首先用argc和argv初始化rclcpp。接著建立名稱為`add_two_ints_server`的Node。
```cpp
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service = node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);
```
建立名稱為`add_two_ints`的Service，並且把`add`函式傳進去。這裡用Template的方式定義srv的格式，所以只有兩個參數，第一個是service name，第二個是callback function。

```cpp
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}
```
最後就是spin node不讓程式結束直到收到Ctrl-C，然後shutdown。

## Package 設定
在`CMakeLists.txt`加上`add_executable(add_two_ints_server src/add_two_ints_server.cpp)`建立執行檔，並且加上`ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)`**ROS**的相依性。

```cmake
...
add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)
...
```

為了讓`ros2 run`可以執行，還需新增`add_two_ints_server`到`install`的`CMakeLists.txt`:
```cmake
...
install(TARGETS
    server
  DESTINATION lib/${PROJECT_NAME})
...
```

等Client寫好再一起Build吧！

# Service Client Node

在`ros2_ws/src/cpp_srv/cpp_srv`下新增檔案`add_two_ints_client.cpp`:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
    node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}
```

## 解析
```cpp
std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
  node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
```
直接來看`create_client`，這邊也是用Template的方式定義srv的格式，所以只有一個service name的參數。這邊的`node`是用`make_shared`的方式建立，所以可以直接用`->`來呼叫`create_client`。

```cpp
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = atoll(argv[1]);
    request->b = atoll(argv[2]);
```
這邊用smart pointer建立`request`，並且把command line的參數，也就是`argv[1]`和`argv[2]`，轉成`int64`的型態，分別給`request->a`和`request->b`。

```cpp
     while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
```
這邊用`wait_for_service`來等待Service Server，如果等待超過1秒就會顯示：`service not available, waiting again...`，如果等待過程中收到Ctrl-C或是**ROS**被中斷，就會顯示錯誤：`Interrupted while waiting for the service. Exiting.`。

```cpp
    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }
```
這裡將剛剛包好的request用非同步的方式(`async_send_request`)傳給Service Server，並且等待結果。`rclcpp::spin_until_future_complete`類似spin，會等待Server Response。不像`rclpy`一樣回傳的是future，這邊回傳直接就是`result`，所以可以直接用`result.get()`來取得response。
如果成功的話，就會顯示`Sum: `，失敗的話就顯示`Failed to call service add_two_ints`。

## Package 設定
在`CMakeLists.txt`加上`add_executable(client src/add_two_ints_client.cpp)`建立執行檔，並且加上`ament_target_dependencies(client rclcpp example_interfaces)`**ROS**的相依性。最終`CMakeLists.txt`如下：
```cmake
cmake_minimum_required(VERSION 3.5)
project(cpp_srv)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client rclcpp example_interfaces)

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

# Build & Run
## Build
1. 在build之前先安裝遺失的dependencies:
    ```bash
    cd ~/ros2_ws
    rosdep install -i --from-path src --rosdistro foxy -y
    ```
2. Build:
    ```bash
    colcon build --packages-select cpp_srv
    ```
3. Source:
    ```bash
    source install/setup.bash
    ```
## Run
1. 開啟一個terminal，執行Service Server:
    ```bash
    ros2 run cpp_srv server
    ```
    這個Service Server會等待Client的請求...
2. 接著開啟另一個terminal，執行Service Client:
    ```bash
    ros2 run cpp_srv client 1 2
    ```
    就可以看到client 收到server的回傳結果了:
    ```bash
    [INFO] [rclcpp]: Sum: 3
    ```
3. 同時在Server的terminal也可以看到收到Client的請求:
    ```bash
    [INFO] [rclcpp]: Incoming request
    a: 1 b: 2
    [INFO] [rclcpp]: sending back response: [3]
    ```
4. `Ctrl-C`結束程式。

# ROS vs. ROS2
|說明| ROS    | ROS2 |
|----|--------|------|
|Service Object 建立|`ros::ServiceServer`|`rclcpp::Service`|
|Service Init | `node.advertiseService(srv_name, callback)` | `node->create_service<srv_type>(srv_name, callback)` |
|Service Client Object 建立 | `ros::ServiceClient` | `rclcpp::Client` |
|Service Client Init | `node.serviceClient<srv_type>(srv_name)` | `node->create_client<srv_type>(srv_name)` |
|srv_type定義位置 | callback的參數 | Service template 及 callback 的參數 |


# 補充
---
如果要把Server Node寫成member function的形式和昨天的Python版本一樣，這邊附上我自己寫的，不過還沒有測試過，有興趣的可以試試看，記得在`CMakeLists.txt`加上`add_executable`和`ament_target_dependencies`。

`service_member_function.cpp`:
```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"

class MinimalService : public rclcpp::Node
{
public:
    MinimalService()
    : Node("minimal_service")
    {
        srv_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints", std::bind(&MinimalService::add_two_ints, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void add_two_ints(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld" " b: %ld", request->a, request->b);
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MinimalService>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
```

`client_member_function.cpp`:
```cpp
#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MinimalClient : public rclcpp::Node
{
public:
    MinimalClient()
    : Node("minimal_client")
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    }

    void send_request()
    {
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_INFO(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = 1;
        request->b = 2;

        auto result = client_->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Sum: %ld", result.get()->sum);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service add_two_ints");
        }
    }

private:
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MinimalClient>();
    node->send_request();

    rclcpp::shutdown();
    return 0;
}
```




# Reference
---
* [ROS2 Tutorials - Writing a simple service and client (C++)](https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Service-And-Client/)
* [ROS2 Github Example](https://github.com/ros2/examples/tree/foxy)
* [ROS Tutorials - Writing a Simple Service and Client (C++)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)