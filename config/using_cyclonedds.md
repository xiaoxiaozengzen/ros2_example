参考：https://docs.ros.org/en/foxy/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html

```bash
# 1.安装
sudo apt install ros-foxy-rmw-cyclonedds-cpp

# 2.导入环境变量
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 3.可选的，指定cyclonedds的xml
export CYCLONEDDS_URI=/opt/cyclonedds.xml

```

## cyclonedds.xml

文件内容大致为：

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config
        https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
<Domain id="any">
        <Internal>
                <MinimumSocketReceiveBufferSize>30MB</MinimumSocketReceiveBufferSize>
                <MinimumSocketSendBufferSize>30MB</MinimumSocketSendBufferSize>
        </Internal>
        <General>
                <NetworkInterfaceAddress>bond0</NetworkInterfaceAddress>
                <MaxMessageSize>8000 B</MaxMessageSize>
                <MaxRexmitMessageSize>8000 B</MaxRexmitMessageSize>
                <!--<FragmentSize>9000 B</FragmentSize>-->
                <AllowMulticast>spdp</AllowMulticast>
                <EnableMulticastLoopback>true</EnableMulticastLoopback>
        </General>
</Domain>
</CycloneDDS>
```

```
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="" xmlns:xsi="" xsi:schemaLocation=" ">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>auto</NetworkInterfaceAddress> 可以使用IP地址或者网卡名. 暂不支持配置多张网卡, 以后会支持.
            <AllowMulticast>default</AllowMulticast> 是否多播支持, 在有线网络上多播可以初始发现多个对等实体, 但在wifi网络上仅仅用户SPDP的初始发现, 因为wifi多播非常不可靠.
            <MaxMessageSize>67108864B</MaxMessageSize> 设定RTPS消息的负载大小, 
            <FragmentSize>32768B</FragmentSize> 设定RTPS消息的分片大小, 用于提升通讯速度.最大 64000
        </General>
        <Internal>
            <Watermarks>
                <WhcHigh>32MB</WhcHigh> 没有被读出的最大的缓冲数据量

            </Watermarks>
        </Internal>
        <Tracing>
            <Verbosity>config</Verbosity>  Verbosity 可以配置是否跟踪控制, 有很多调试级别可以设置.
            <OutputFile>stdout</OutputFile> 输出可以配置为文件.
        </Tracing>
    </Domain>
</CycloneDDS>
```
