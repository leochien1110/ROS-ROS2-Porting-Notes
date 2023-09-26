> 雖然官方已經有提供不少好用的Message，但是有時候還是會需要客製化的Message，像是影像偵測完需要畫框，或是需要客製化的Service，像是特規機器人的導航等等，這時候就需要自己來客製化Message和Service。

# 預設 Message
這邊先介紹常用的Message有：
* std_msgs：一些基礎的資料型態，可以去下面Message Fieldtype查看
* geometry_msgs：一些幾何的資料型態，例如Point, Pose, Twist等等
* sensor_msgs：一些感測器的資料型態，例如Image, PointCloud2, NavSatFix等等
* nav_msgs：導航的資料型態，例如Odometry, Path等等
* visualization_msgs：rviz用來畫圖的plugin，例如Marker, MarkerArray等等

其他還有路徑、地圖、Action等等，可以上Github看[ROS2 Common Interface](https://github.com/ros2/common_interfaces/tree/foxy)的內容，裡面有很多預設的Message可以使用。也可以查看[ROS2 Foxy API - msgs & srvs](https://docs.ros2.org/foxy/api/)。

# 客制化 Message


# 客製化 Service


# Message Fieldtype
---
內建的Type可以參考[官方文件](https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html#field-types):

| Type name | [C++](https://design.ros2.org/articles/generated_interfaces_cpp.html) | [Python](https://design.ros2.org/articles/generated_interfaces_python.html) | [DDS type](https://design.ros2.org/articles/mapping_dds_types.html) |
| --------- | --------------------------------------------------------------------- | --------------------------------------------------------------------------- | ------------------------------------------------------------------- |
| bool      | bool                                                                  | builtins.bool                                                               | boolean                                                             |
| byte      | uint8_t                                                               | builtins.bytes\*                                                            | octet                                                               |
| char      | char                                                                  | builtins.str\*                                                              | char                                                                |
| float32   | float                                                                 | builtins.float\*                                                            | float                                                               |
| float64   | double                                                                | builtins.float\*                                                            | double                                                              |
| int8      | int8_t                                                                | builtins.int\*                                                              | octet                                                               |
| uint8     | uint8_t                                                               | builtins.int\*                                                              | octet                                                               |
| int16     | int16_t                                                               | builtins.int\*                                                              | short                                                               |
| uint16    | uint16_t                                                              | builtins.int\*                                                              | unsigned short                                                      |
| int32     | int32_t                                                               | builtins.int\*                                                              | long                                                                |
| uint32    | uint32_t                                                              | builtins.int\*                                                              | unsigned long                                                       |
| int64     | int64_t                                                               | builtins.int\*                                                              | long long                                                           |
| uint64    | uint64_t                                                              | builtins.int\*                                                              | unsigned long long                                                  |
| string    | std::string                                                           | builtins.str                                                                | string                                                              |
| wstring   | std::u16string                                                        | builtins.str                                                                | wstring 


內建也有Array Type:
| Type name               | [C++](https://design.ros2.org/articles/generated_interfaces_cpp.html) | [Python](https://design.ros2.org/articles/generated_interfaces_python.html) | [DDS type](https://design.ros2.org/articles/mapping_dds_types.html) |
| ----------------------- | --------------------------------------------------------------------- | --------------------------------------------------------------------------- | ------------------------------------------------------------------- |
| static array            | std::array<T, N>                                                      | builtins.list\*                                                             | T[N]                                                                |
| unbounded dynamic array | std::vector                                                           | builtins.list                                                               | sequence                                                            |
| bounded dynamic array   | custom_class<T, N>                                                    | builtins.list\*                                                             | sequence<T, N>                                                      |
| bounded string          | std::string         

Array的定義方式如下:
```python
int32[] unbounded_integer_array # 無界定整數陣列
int32[5] five_integers_array # 大小為5的整數陣列
int32[<=5] up_to_five_integers_array # 大小不超過5的整數陣列

string string_of_unbounded_size # 無界定字串
string<=10 up_to_ten_characters_string # 長度不超過10的字串

string[<=5] up_to_five_unbounded_strings # 長度不超過5的無界定字串陣列
string<=10[] unbounded_array_of_strings_up_to_ten_characters_each # 無界定長度的字串陣列，每個字串長度不超過10
string<=10[<=5] up_to_five_strings_up_to_ten_characters_each # 長度不超過5的無界定字串陣列，每個字串長度不超過10

```

# Reference
---
* [Implementing custom interfaces](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html)
* [ROS Interfaces](https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html)
* [ROS Common Interface Github](https://github.com/ros2/common_interfaces/tree/foxy)
* [ROS2 Foxy API - msgs & srvs](https://docs.ros2.org/foxy/api/)