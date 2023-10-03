> 沒想到禮拜四聽完Hans Zimmer後把電腦忘在公司，想起來時只能找到存好的Day16草稿，接下來幾天要努力一點看有沒有辦法一天兩篇補回來。Action的使用比較進階，這邊會一口氣介紹加範例，不過礙於篇幅，只會介紹比較複雜的C++範例，Python則要自行參考[官方教學](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)。

# Action
---
Action是**ROS**中的第三種通訊方式，用來處理長時間的任務，由三個部分組成: **Goal**、**Feedback**、**Result**。

Action是建立在Topic和Service之上的，Goal和Result是用Service來傳遞，Feedback是用Topic來傳遞，如下圖官方的示意圖:
![Action](https://docs.ros.org/en/foxy/_images/Action-SingleActionClient.gif)

Action主要功能和Service很接近，不過是Non-blocking的，也就是說Action Client可以隨時取消任務，而且可以隨時接收Feedback，而不是像Service一樣只能等到任務完成才會回傳結果。這個機制可以讓我們在任務進行中就可以做一些事情，像是在機器人移動的過程中，可以隨時接收到目前的位置，這樣就可以在任務進行中做一些判斷，而不是等到任務完成才能做判斷，也可以即時取消路徑規劃預防撞到障礙物。

## 自定義Action
Action和Service, Topic一樣可以自定義訊息格式，檔名`.action`，一般會放在`package_name/action`底下，格式如下:
```bash
# goal
---
# feedback
---
# result
```

跟`.srv`一樣用`---`來分隔，而順序相比`ROS`則是移到了中間的位置，這邊不能搞錯！

接下來介紹具體的例子。官方拿`Fibonacci.action`來當範例，這樣得先介紹一下。

[Fibonacci](https://en.wikipedia.org/wiki/Fibonacci_sequence)(費氏數列)，是一個數學上的數列，第一項和第二項都是1，第三項開始每一項都是前兩項的和，也就是說第三項是1+1=2，第四項是1+2=3，第五項是2+3=5，以此類推，數列如下:
```bash
1, 1, 2, 3, 5, 8, 13, 21, 34, ...
```

# Example
---
## Action Interface
1. 首先建立一個package
    ```bash
    cd ros2_ws/src
    ros2 pkg create action_tutorials_interfaces
    ```
2. 定義.action，所以資料格式如下:
    ```bash
    int32 order
    ---
    int32[] sequence
    ---
    int32[] partial_sequence
    ```
3. 編輯`CMakelists.txt`
    ```cmake
    find_package(rosidl_default_generators REQUIRED)
    rosidl_generate_interfaces(${PROJECT_NAME}
        "action/Fibonacci.action"
    )
    ```
4. 編輯`package.xml`
    ```xml
    <buildtool_depend>rosidl_default_generators</buildtool_depend>

    <depend>action_msgs</depend>

    <member_of_group>rosidl_interface_packages</member_of_group>
    ```
5. build
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select action_tutorials_interfaces
    source install/setup.bash
    ```
6. 查看interfaces
    ```bash
    ros2 interface show action_tutorials_interfaces/action/Fibonacci
    ```
    應該會看到:
    ```bash
    int32 order
    ---
    int32[] sequence
    ---
    int32[] partial_sequence
    ```
## Server Node
1. 建立一個新Package，並相依於`action_tutorials_interfaces`:
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp
    ```
2. 建立一個新檔`action_tutorials_cpp/src/fibonacci_action_server.cpp`:
    ```cpp
    #include <functional>
    #include <memory>
    #include <thread>

    #include "action_tutorials_interfaces/action/fibonacci.hpp"
    #include "rclcpp/rclcpp.hpp"
    #include "rclcpp_action/rclcpp_action.hpp"
    #include "rclcpp_components/register_node_macro.hpp"

    #include "action_tutorials_cpp/visibility_control.h"

    namespace action_tutorials_cpp
    {
    class FibonacciActionServer : public rclcpp::Node
    {
    public:
    using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

    ACTION_TUTORIALS_CPP_PUBLIC
    explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("fibonacci_action_server", options)
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<Fibonacci>(
        this,
        "fibonacci",
        std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
        std::bind(&FibonacciActionServer::handle_cancel, this, _1),
        std::bind(&FibonacciActionServer::handle_accepted, this, _1));
    }

    private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto & sequence = feedback->partial_sequence;
        sequence.push_back(0);
        sequence.push_back(1);
        auto result = std::make_shared<Fibonacci::Result>();

        for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
            result->sequence = sequence;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        // Update sequence
        sequence.push_back(sequence[i] + sequence[i - 1]);
        // Publish feedback
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");

        loop_rate.sleep();
        }

        // Check if goal is done
        if (rclcpp::ok()) {
        result->sequence = sequence;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
    };  // class FibonacciActionServer

    }  // namespace action_tutorials_cpp

    RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)
    ```

    Constrcutor內創建了一個Action Server
    ```cpp
    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
    this,
    "fibonacci",
    std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
    std::bind(&FibonacciActionServer::handle_cancel, this, _1),
    std::bind(&FibonacciActionServer::handle_accepted, this, _1));
    ```
    這邊需要有：
    1. `Fibonacci` Template是在`.action`中定義的格式。
    2. `create_server`是繼承的Node底下的function，所以需要用`this`來呼叫。
    3. `"fobonacci"`是我們的action名稱
    4. `handle_goal`是當收到Goal時要執行的function
    5. `handle_cancel`是當收到Cancel時要執行的function
    6. `handle_accepted`是當收到Accepted時要執行的function

    ```cpp
    void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    ```
    則是主要的Loop，這邊會先收到Goal，然後開始執行，每秒會publish一次Feedback，直到收到Cancel，或是執行完畢將`goal_handle`標記為`succeed`，然後結束。


3. 編輯`CMakeLists.txt`，這邊會相較其他interface複雜許多。將下面的code複製到`find_package`之後:
    ```cmake
    add_library(action_server SHARED
    src/fibonacci_action_server.cpp)
    target_include_directories(action_server PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
    target_compile_definitions(action_server
    PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
    ament_target_dependencies(action_server
    "action_tutorials_interfaces"
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components")
    rclcpp_components_register_node(action_server PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
    install(TARGETS
    action_server
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
    ```
4. Build
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select action_tutorials_cpp
    source install/setup.bash
    ```
5. Run
    ```bash
    ros2 run action_tutorials_cpp fibonacci_action_server
    ```

### 解析

## Client Node
1. 建立一個新檔`action_tutorials_cpp/src/fibonacci_action_client.cpp`:
    ```cpp
    #include <functional>
    #include <future>
    #include <memory>
    #include <string>
    #include <sstream>

    #include "action_tutorials_interfaces/action/fibonacci.hpp"

    #include "rclcpp/rclcpp.hpp"
    #include "rclcpp_action/rclcpp_action.hpp"
    #include "rclcpp_components/register_node_macro.hpp"

    namespace action_tutorials_cpp
    {
    class FibonacciActionClient : public rclcpp::Node
    {
    public:
    using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

    explicit FibonacciActionClient(const rclcpp::NodeOptions & options)
    : Node("fibonacci_action_client", options)
    {
        this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
        this,
        "fibonacci");

        this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&FibonacciActionClient::send_goal, this));
    }

    void send_goal()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        }

        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = 10;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback =
        std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
        std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
        std::bind(&FibonacciActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleFibonacci::SharedPtr,
        const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Next number in sequence received: ";
        for (auto number : feedback->partial_sequence) {
        ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    void result_callback(const GoalHandleFibonacci::WrappedResult & result)
    {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        std::stringstream ss;
        ss << "Result received: ";
        for (auto number : result.result->sequence) {
        ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        rclcpp::shutdown();
    }
    };  // class FibonacciActionClient

    }  // namespace action_tutorials_cpp

    RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClient)
    ```
2. 編輯`CMakeLists.txt`，將下面的code放在`find_package`之後:
    ```cmake
    add_library(action_client SHARED
    src/fibonacci_action_client.cpp)
    target_include_directories(action_client PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
    target_compile_definitions(action_client
    PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
    ament_target_dependencies(action_client
    "action_tutorials_interfaces"
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components")
    rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
    install(TARGETS
    action_client
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
    ```

    在Constructor內創建了一個Action Client
    ```cpp
    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
      this,
      "fibonacci");
    ```
    這邊需要有：
    1. `Fibonacci` Template是在`.action`中定義的格式。
    2. `create_client`是繼承的Node底下的function，所以需要用`this`來呼叫。
    3. `"fobonacci"`是我們的action名稱，必須跟Server使用的名稱相符才能溝通。

    ```cpp
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FibonacciActionClient::send_goal, this));
    ```
    和publisher一樣用timer來控制發送Goal的頻率，這邊設定為每0.5秒發送一次。

    ```cpp
    void send_goal()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        }

        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = 10;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback =
        std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
        std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
        std::bind(&FibonacciActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }
    ```
    這邊則是發送Goal的function，這邊做了幾件事情:
    1. `this->timer_->cancel()`取消timer，避免重複發送Goal。
    2. `this->client_ptr_->wait_for_action_server()`等待Action Server啟動，如果沒有啟動就會一直等待，直到timeout。
    3. 建立Goal，這邊設定order為10。
    4. 設定`send_goal_options`，這邊設定了三個callback，分別是`goal_response_callback`、`feedback_callback`、`result_callback`。
    5. `this->client_ptr_->async_send_goal(goal_msg, send_goal_options)`將Goal發送出去。


3. Build
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select action_tutorials_cpp
    source install/setup.bash
    ```
4. Run
    ```bash
    ros2 run action_tutorials_cpp fibonacci_action_client
    ```
    應該就可以看到Server跑完一次就會publish一次Feedback，Client收到Feedback後會印出來，最後Server跑完後會publish一次Result，Client收到後會印出來。

# ROS vs. ROS2
---
|說明| ROS    | ROS2 |
|:---|:-------|:-----|
|action 格式順序| Goal->Result->Feedback | Goal->Feedback->Result |  
|`CMakeList.txt`| `find_package(catkin REQUIRED COMPONENTS actionlib_msgs)` <br>`add_action_files(DIRECTORY action FILES Fibonacci.action)` <br>`generate_messages( DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs)` <br>`catkin_package( CATKIN_DEPENDS actionlib_msgs)` | `find_package(rosidl_default_generators REQUIRED)` <br>`rosidl_generate_interfaces(${PROJECT_NAME} "action/Fibonacci.action")` |
|`package.xml` | `<exec_depend>message_generation</exec_depend>` | `<buildtool_depend>rosidl_default_generators</buildtool_depend>` <br>`<depend>action_msgs</depend>` <br>`<member_of_group>rosidl_interface_packages</member_of_group>`|

# Reference
---
* [蛤- ROS Action](https://ithelp.ithome.com.tw/articles/10244969?sc=hot)
* [ROS2 Understanding Action](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
* [ROS2 - Writing an action server and client (C++)](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
* [ROS2 - Writing an action server and client (Python)](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)