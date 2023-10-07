# Plugin
---
**ROS**中有個特別的package叫做`pluginlib`，這個package提供了一個plugin的架構，讓我們可以在不修改原始碼的情況下，動態的載入plugin，類似於C++的多型(polymorphism)。這樣可以讓程式更有彈性，也更容易擴充。

舉例來說，在自駕車中避障的功能可能同時有不同的演算法，我們會需要在不同的情境下啟用不同的避障功能，像是白天就可以使用LiDAR+相機的plugin，但夜間可能相機感光度不夠只能依賴LiDAR的plugin。

# 範例
---
這邊拿官方的多邊形為例子來說明plugin的使用方法。首先我們會先定義一個多邊形的base class，接著會有三角形和長方形的plugin連結到base class。

這邊借用官方的流程圖:
![ploygon plugin](http://wiki.ros.org/pluginlib?action=AttachFile&do=get&target=plugin_model.png)

1. 安裝`pluginlib` package
    ```bash
    sudo apt-get install ros-foxy-pluginlib
    ```
2. 建立一個`polygon_base` package
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake polygon_base --dependencies pluginlib --node-name area_node
    ```
3. 在`ros2_ws/src/polygon_base/include/polygon_base/regular_polygon.hpp`中定義一個多邊形的base class
    ```cpp
    #ifndef POLYGON_BASE_REGULAR_POLYGON_HPP
    #define POLYGON_BASE_REGULAR_POLYGON_HPP

    namespace polygon_base
    {
    class RegularPolygon
    {
        public:
        virtual void initialize(double side_length) = 0;
        virtual double area() = 0;
        virtual ~RegularPolygon(){}

        protected:
        RegularPolygon(){}
    };
    }  // namespace polygon_base

    #endif  // POLYGON_BASE_REGULAR_POLYGON_HPP
    ```
    這些基本上都是C++的pure virtual function，沒有辦法單獨運作，需要被繼承才能使用。
4. 接著編輯`CMakeLists.txt`讓這個header可以被其他Package的Class繼承:
    ```cmake
    install(
    DIRECTORY include/
    DESTINATION include
    )
    ...
    ament_export_include_directories(
    include
    )
    # before ament_package
    ```
5. 創建`polygon_plugins`的package
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake polygon_plugins --dependencies polygon_base pluginlib --library-name polygon_plugins
    ```
6. 編輯`ros2_ws/src/polygon_plugins/src/polygon_plugins.cpp`:
    ```cpp
    #include <polygon_base/regular_polygon.hpp>
    #include <cmath>

    namespace polygon_plugins
    {
    class Square : public polygon_base::RegularPolygon
    {
        public:
        void initialize(double side_length) override
        {
            side_length_ = side_length;
        }

        double area() override
        {
            return side_length_ * side_length_;
        }

        protected:
        double side_length_;
    };

    class Triangle : public polygon_base::RegularPolygon
    {
        public:
        void initialize(double side_length) override
        {
            side_length_ = side_length;
        }

        double area() override
        {
            return 0.5 * side_length_ * getHeight();
        }

        double getHeight()
        {
            return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
        }

        protected:
        double side_length_;
    };
    }

    #include <pluginlib/class_list_macros.hpp>

    PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
    PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
    ```
    這邊我們定義了兩個plugin，一個是正方形，一個是三角形，裡面定義了上面pure virtual function的實作。
    Classes後面接著才是重點，這邊我們使用了`pluginlib`提供的`PLUGINLIB_EXPORT_CLASS`，這個macro會幫將這些classes註冊到`plugin`中。第一個參數是class的名稱，第二個參數是base class的名稱。
7. Plugin loader還會需要一個`plugin.xml`來表達資訊。創建一個`ros2_ws/src/polygon_plugins/plugins.xml`:
    ```xml
    <library path="polygon_plugins">
    <class type="polygon_plugins::Square" base_class_type="polygon_base::RegularPolygon">
        <description>This is a square plugin.</description>
    </class>
    <class type="polygon_plugins::Triangle" base_class_type="polygon_base::RegularPolygon">
        <description>This is a triangle plugin.</description>
    </class>
    </library>
    ```
    * `library`的`path`是指定plugin的library。與`ROS`不同的是`ROS`會需要`lib/`的prefix，這邊的話會變成`lib/polygon_plugins`。但這點在`ROS2`中已經被修改，不需要加上`lib/`。
    * `type`是plugin的名稱。
    * `base_class_type`是指定base class的名稱。
    * `description`是plugin的描述。
8. 在`ROS`中**註冊plugin**是寫在`package.xml`而`ROS2`則是寫在`CMakeLists.txt`，不過也變的冗長許多從三行變成好幾行。編輯`CMakeLists.txt`，將下面的程式碼加到`ros2_ws/src/polygon_plugins/CMakeLists.txt`的`find_package(pluginlib REQUIRED)`下面:
    ```cmake
    add_library(polygon_plugins src/polygon_plugins.cpp)
    target_include_directories(polygon_plugins PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
    ament_target_dependencies(
    polygon_plugins
    polygon_base
    pluginlib
    )

    pluginlib_export_plugin_description_file(polygon_base plugins.xml)

    install(
    TARGETS polygon_plugins
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    )
    ```
    `pluginlib_export_plugin_description_file`的第一個參數是指定base class的package，第二個參數是指定`plugin.xml`的路徑。

    最後在`ament_package`前面加上:
    ```cmake
    ament_export_libraries(
    polygon_plugins
    )
    ament_export_targets(
    export_${PROJECT_NAME}
    )
    ```

這樣就大功告成寫好了plugin，接著我們要來寫一個node來使用這些plugin。

## 使用plugin
理論上這些plugin可以在任何一個package中使用，這邊我們寫在base package裡。
1. 新增`ros2_ws/src/polygon_base/src/area_node.cpp`:
    ```cpp
    #include <pluginlib/class_loader.hpp>
    #include <polygon_base/regular_polygon.hpp>

    int main(int argc, char** argv)
    {
        // To avoid unused parameter warnings
        (void) argc;
        (void) argv;

        pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("polygon_base", "polygon_base::RegularPolygon");

        try
        {
            std::shared_ptr<polygon_base::RegularPolygon> triangle = poly_loader.createSharedInstance("polygon_plugins::Triangle");
            triangle->initialize(10.0);

            std::shared_ptr<polygon_base::RegularPolygon> square = poly_loader.createSharedInstance("polygon_plugins::Square");
            square->initialize(10.0);

            printf("Triangle area: %.2f\n", triangle->area());
            printf("Square area: %.2f\n", square->area());
        }
        catch(pluginlib::PluginlibException& ex)
        {
            printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
        }

        return 0;
    }
    ```
    `ClassLoader`是`pluginlib`提供的class，可以用來載入plugin。
        * template內是放base class和plugin的名稱。
        * 第一個參數是指定base class的package。
        * 第二個參數是指定base class的名稱。
    > :warning: 注意這邊`ploygon_base`並沒有指出`polygon_plugins`的dependency而是直接使用，所以在使用時須自行確認是否有安裝`polygon_plugins`。
2. Build:
    ```bash
    colcon build --packages-select polygon_base polygon_plugins

    source install/setup.bash
    ```
3. 接著執行:
    ```bash
    ros2 run polygon_base area_node
    ```
    會得到:
    ```bash
    Triangle area: 43.30
    Square area: 100.00
    ```
    這邊我們可以看到，我們在`area_node.cpp`中並沒有直接使用到`polygon_plugins`，但是在執行時卻會載入`polygon_plugins`的plugin。

# ROS vs. ROS2
---
由於plugin不是**ROS**的原生的功能而是以package的形式，所以改變的幅度並沒有很大，只是在註冊plugin的部分有些許差異。
| 說明 | ROS | ROS2 |
| :--- | :--- | :--- |
| plugin.xml的library | lib/ | 不需要lib/ |
| 註冊plugin檔案 | package.xml | CMakeLists.txt |



# Reference
---
* [Creating and using plugins (C++)](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Pluginlib.html)
* [ROS pluginlib](http://wiki.ros.org/pluginlib)
* [蛤 - ROS Pluginlib](https://ithelp.ithome.com.tw/articles/10251441)