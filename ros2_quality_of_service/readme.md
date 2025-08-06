# Overview

ROS2提供了一系列的QOS，用于调整节点间的通信。设置合适的QOS，可以让节点间通信实现类似可靠的TCP或者best-effort的UDP那样。
* 通过设置适当的QOS，可以在网络环境不好的情况下，实现reliable通信
* 对应实时性要求严格的场景，设置QOS满足ddl要求

[QOS官网文档](https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html)

QOS可以应用于Pub、Sub、Client、Server这些。

**注意: 如果发送和接收端使用的QOS不同，可能消息无法正确发送**

# 策略

当前提供的策略有：
* history
  * keeplast：消息队列至多保留N长度的数据
  * keepall：存储所有消息，最大值会受到底层DDS的的限制
* depth：
  * 队列大小，只有当history是keeplast的时候才起作用
* reliability
  * best effort：会最大努力发消息，但是可能会因为网络问题而丢失消息
  * reliable：保证每条消息都被发送到，发送失败时会重复发送消息
* durability(耐久性):
  * transient(短暂的) local: publisher负责保存那些 late-joining subscription 的消息样本
  * volatile：不会关心如何保存消息样本
* deadline：
  * duration：序列化好的消息到被发布出去的最大时间间隔
* lifespan：
  * duration：从发布到接收消息之间的最长时间，而消息不会被视为过时或过期（过期的消息会被自动丢弃，实际上永远不会被接收到）。
* liveliness：
  * automatic：当一个node中的任意一个publisher在 lease duration期间发送了消息，就认为这个node中的所有publisher都是alive状态
  * manual by topic: 在lease duration内，node手动通过相关的API判断publisher是否是alive状态，若是，则系统认为改publisher是alive
* lease duration：
  * duration：系统认为publisher还活着的最大的周期间隔时间，在此期间publisher必须要表明自己还活着，否则系统认为改publihser失去活性(一种failure)

# QoS Profile

Profiles可以让开发者关注于应用本身，而不是配置QoS。一个Profile会定义一堆policies。
当前有些预先定义好的, 可以参考 [已定义](https://github1s.com/ros2/rmw/blob/foxy/rmw/include/rmw/qos_profiles.h#L25)
默认情况下的QoS：
* Publisher$Subscriptions: 
  * keep last ，queen size = 10，reliable，
* Services:
* Sensor data：
  * best effort
* Parameters:
  * Parameters在ros2中是基于Service的
