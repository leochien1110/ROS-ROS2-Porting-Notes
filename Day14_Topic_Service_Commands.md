> 本來想在Topic結束先介紹指令，但想說Service和Topic概念很像，想說一口氣說完ros node的部分，然後把所有指令放在一起。但又想到一口氣介紹所有指令太可怕，所以就在介紹完Topic和Service後來介紹指令吧！如果有忘記的地方的話，不要忘了可以回去查前幾天的文章唷！

# Command Line
Service Client也可以使用command line來執行，一樣先開啟Service Server:
```bash
ros2 run py_srv service_member_function
```
接著查看Service的資訊:
```bash
ros2 service list
# 應該會看到
/add_two_ints
```
可以查看srv的格式:
```bash
ros2 interface show example_interfaces/srv/AddTwoInts
# 應該會看到
int64 a
int64 b
---
int64 sum
```

接著執行Service Client:
```bash
# ros2 service call <service_name> <srv_type> <request>
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
# 應該會看到
sum: 3
```
注意cli的request格式是yaml或是python dictionary，因此要用`{}`包起來，並且使用`:`來分隔key和value。最外面要再用`""`包成字串，才不會被shell解析成其他指令。

# Reference
---
