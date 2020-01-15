# Intro

HbipcPlugin（Hbipc Plugin）为传输插件，主要负责实现AP与CP端的双向通信功能，用于AP与CP间的BIF通信功能，CP端通过HbipcPlugin将智能帧等数据发送到AP端，或者通过HbipcPlugin接收AP端发过来的配置数据与命令数据。

# Detail

hbipc接口是系统软件 cheng.chen 提供的一套用于CP、AP通信的接口（API），拥有统一的上层接口，支持包括bifspi、bifsd等不同的底层传输协议。具体可以参考：[HBIPC Service API](http://wiki.hobot.cc/display/SystemSoftware/HBIPC+Service+API)

hbipcplugin 分为两部分，分别为hbipcplugin与hbipcsession，hbipcplugin为xpluginflow框架下的接口，hbipcsession为系统软软件接口的实例，此plugin主要用于AP与CP间的通信：

- HbipcPlugin在CP端主要与SmartPlugin与VioPlugin进行通信，获取从两个Plugin发来的消息，经过串行化后通过底层接口发送给AP端；
- HbipcPlugin通过底层接口与AP端进行通信，将从AP端发过来的串行化数据发送给CP端的总线；
- HbipcPlugin支持通过配置文件选择不同的通信方式；

# Build

运行命令：
`sh cicd/scripts/build_aarch.sh`

### Dependency
- build.properties
- build.properties.local

### 编译环境设置
- build.gradle
- hobot_util.cmake

### 编译选项设置
- CMakeLists.txt

# Usage

### 使用说明
**默认配置文件：** hbipc_config.json

**配置文件说明：**
```
{
  // 通信域名
  "domain_name": "X2SD001",
  // 通信server id
  "server_id": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 14]
}
```

**接口调用顺序：**
```
  auto sc_plg = std::make_shared<HbipcPlugin>("./configs/hbipc_config.json");

  sc_plg->Init();

  sc_plg->Start();

  sc_plg->Stop();

  sc_plg->Deinit();
```
**如果创建对象时没有传入配置文件，就是用默认配置，默认配置为：**
```
std::string domain_name_ = "X2SD001";
uuid server_id_ = {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7,
                     0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf};
```
### 模块消息
**模块消息定义：**
```
struct CustomHbipcMessage : HbipcMessage {
  explicit CustomHbipcMessage(std::string proto) : proto_(proto) {
    type_ = TYPE_HBIPC_MESSAGE;
  }
  std::string Serialize() override;

 private:
  std::string proto_;
};
```
- proto_：存储接收到的AP端串行化数据
- Serialize()：提供Hbipc的串行化数据，本质上为直接返回proto_存储的protobuf字串流

HbipcPlugin的消息暂时只向AP发送，所有的串行化数据存储于消息对象中的proto_数据成员中。

### 性能开销

### 维护人员
hao.tian@horizon.ai

