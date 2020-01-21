<!-- TOC -->

- [综述](#综述)
    - [地平线X2](#地平线x2)
    - [人脸抓拍识别](#人脸抓拍识别)
- [BUILD](#build)
    - [安装交叉编译工具链](#安装交叉编译工具链)
    - [开源repo的编译方式](#开源repo的编译方式)
- [Deploy](#deploy)
    - [总体架构](#总体架构)
        - [XPP(X2 prototype platform)](#xppx2-prototype-platform)
        - [XRoc-framework](#xroc-framework)
        - [从模型训练到部署XPP&XRoc的位置](#从模型训练到部署xppxroc的位置)
- [工程组织](#工程组织)
    - [目录结构](#目录结构)
    - [external](#external)
            - [预编译库](#预编译库)
    - [开源库](#开源库)
- [基于该repo二次开发升级solution](#基于该repo二次开发升级solution)

<!-- /TOC -->
# 综述
该repo基于地平线X2芯片构建了人脸抓拍识别solution方案。    

## 地平线X2
X2是地平线研发的第二代面向AIoT场景的AI加速芯片，在视觉领域可用于智能相机、智能门禁系统等多种场景。  
BPU的架构如下图：
![BPU-ARCHITECTURE](./doc/images/BPU-ARCH.png)
* X2芯片主要包含VIO（视频输入处理单元）、BPU（CNN加速单元）、BIF（通过spi等对外传输模块）;
* X2支持camera以mipi或者bt1120接口传递视频到VIO单元，从VIO可以获得一帧帧NV12格式的图片;
* 从VIO获取的图片输入BPU单元，配合不同的基于地平线工具链编译的模型可以得到对应的智能结果，如人脸检测框、人脸特征值等;
* 通常可通过BIF模块或者网口（如果存在的话）将智能桢结果传输出去。
## 人脸抓拍识别
人脸抓拍识别方案用于从移动的场景中抓取人脸并提取人脸特征，提取的人脸特征可进一步和底库中的人脸比对，从而判断出该人是否出现在数据库中;   
该方案可以用人脸门禁、智能安防等场景。  
人脸识别方案通常包括人脸检测模块、人脸跟踪模块、人脸质量模块、人脸打分模块、人脸优选模块、人脸提特征模块等，如下图：
![face-snap-feature-arch](./doc/images/face-snap-feature-arch.png)

# BUILD
## 安装交叉编译工具链
为编译在X2 Soc板上运行的可执行程序或库文件, 需要安装工具链：x2j2-aarch64-6.5.0
具体安装交叉工具链压缩包, 如在公司内部可从以下地址下载：
http://gallery.hobot.cc/download/aiot/toolchain/x2j2-aarch64/project/snapshot/linux/x86_64/general/basic/6.5.0/x2j2-aarch64-6.5.0.tar.xz
如在公司外部，请联系技术支持人员获取工具链。

安装完成后，把交叉编译工具路径加入到环境变量PATH中。  
`export PATH=/path/to/toolchain/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin:$PATH`    
## 开源repo的编译方式
> mkdir build & cd build  
> sh ../build.sh      

编译完成的库在build/lib, main程序为bin/xppcp_smart。

# Deploy
编译完成之后:
> cd build      
> sh ../deploy.sh  

在build目录下可以得到新的部署包`xppcp_smart.tgz`.     
完成vio适配后，将其copy到设备上运行`sh xpp_start.sh`就可以执行。

##  总体架构
该Solution方案主要由两层架构构成：   
### XPP(X2 prototype platform)
* 主要为了解耦合输入VIO与智能处理单元（通常需要BPU处理）、以及输出模块（通过BIF传输等）;XPP实现了一个松耦合的消息订阅与分发机制，输入、智能处理、输出等模块被抽象成一个个插件单元，每个插件可以通过总线订阅感兴趣的消息，同时也可以把合法的消息通过总线分发出去。   
* 该solution定义的插件及消息类型如下：
![xpp-plugin-message](./doc/images/xpp-message-arch.png)
**vioplugin**:VIO处理单元，产生一帧帧图像帧消息；如果智能模块来不及处理，vioplugin还会主动发送丢帧消息；     　　　
**smartplugin**:智能处理模块，监听VIO输出的图像帧消息，完成智能化处理，并产生智能帧消息（人脸检测框、人脸特征值等）；该智能单元基于XRoc-framework定义了一个人脸抓拍识别的workflow;   
**hbipcplugin**:基于BIF的输出模块，监听VIO丢帧消息以及smartplugin发送对智能帧消息，完成数据转发；*该solution暂未加载该插件*    

参考链接：   
* [基于XPP的消息订阅与分发Sample](./xpluginflow/sample/sample_plugin.cpp);  
* [Solution main函数入口](./smartplugin/sample/smart_main.cpp) 

### XRoc-framework
* XRoc-framework是一种基于数据流的编程框架，在smartplugin中我们基于xroc定义了一个人脸抓拍识别的workflow;
* 该框架可以通过JSON配置构建workflow，workflow是一个有向拓扑图，图中每个节点（Node）管理了一个或多个同类型method的实例;
* 在XRoc中method表示一种能力，通常是某类模型能力（人脸检测、人脸Pose等）或者算法策略（过滤策略、融合策略、优选策略等）;   
* 将一组能力串联在一起就构成了一个workflow，workflow表示一个范式，比如人脸检测、跟踪、属性（pose、blur等）以及优选等能力级联起来可以构建一个人脸抓拍范式；   
* 除此之外XRoc-Framework定义了一套面向workflow的sdk C++通用接口，通过设置不同的配置文件同一套接口可以运行不同的workflow。   

参考链接：
* [tutorials](xroc-framework/tutorials/README.md) 
* [XRoc的开发手册](xroc-framework/README.md);   

### 从模型训练到部署XPP&XRoc的位置
下图是在ipc平台，从算法训练到部署环节XPP与XRoc的位置：  
![xpp&xpp-arch](doc/images/xpp-xroc-arch-ipc.png)
- 这张图以XPP IPC的架构为例，IPC为CP-AP架构，X2作为CP(Co-Processor)主要用于AI加速，AP(Application Processor)作为应用处理器，通常负责视频、图像处理（如编解码）。
- XPP划分的基本模块：VIOPlugin（X2视频输入基本模块）、SmartPlugin（智能业务基本模块）、HbipcPlugin（X2 bif基础模块，负责CP-AP侧通信）;
- 为了适配不同的智能化应用及智能化应用的跨平台迁移，XPP将输入（VIO）、智能化模块、输出（hbipc）等抽象为独立的插件（plugin），并定义了插件之间的标准通信接口，再通过消息总线完成不同插件之间的数据交互；基于上述架构，通过抽象vioplugin、smartplugin、hbipcplugin通用接口，对不同的平台（96board 或者X2 IPC）需要做的只是适配vioplugin与hbipcplugin，smartplugin可以自由在不同X2 平台迁移。
- 输入（VIOPlugin）、输出（HbipcPlugin）模块已沉淀为基础组件，通常开发者只需要关注算法或者算法策略（SmartPlugin）。
- SmartPlugin目前默认实现是基于XRoc-framework面向workflow的通用接口实现了单workflow的AI应用运行框架；XROC framework是组织业务workflow的运行框架，同时该框架提供了基于配置串联workflow的能力，以及由workflow构建算法sdk的通用接口等；XRoc workflow由一系列method组成，每个method都表示一个算法能力或策略；
- **通过地平线工具链训练完成模型后，开发者在XRoc框架下只需要实现对应Method用于模型对前后处理，该method加入workflow中，和其他模块一起构建了一个AI solution SDK．**
- 上图中X2 IPC　AP侧对智能结果不做处理，仅仅透传给pc前端,前端负责完成一些渲染工作．
# 工程组织
## 目录结构
```
.
├── cnnmethod
│   ├── example
│   ├── include
│   ├── model
│   ├── src
│   └── test
├── doc
│   └── images
├── external
│   ├── bpu_predict
│   ├── gtest
│   ├── hbipc
│   ├── hobotlog
│   ├── hobotsdk
│   ├── ipc_tracking
│   ├── jsoncpp
│   ├── libjpeg-turbo
│   ├── libyuv
│   ├── MOTMethod
│   ├── opencv
│   ├── protobuf
│   ├── vision_type
│   ├── x2_prebuilt
│   ├── xroc-framework
│   └── xroc-imagetools
├── facesnapfiltermethod
│   ├── cmake
│   ├── config
│   ├── include
│   ├── src
│   └── test
├── fasterrcnnmethod
│   ├── configs
│   ├── example
│   ├── include
│   ├── output
│   ├── src
│   └── test
├── GradingMethod
│   ├── config
│   ├── include
│   ├── src
│   └── test
├── hbipcplugin
│   ├── config
│   ├── include
│   ├── sample
│   ├── src
│   └── test
├── smartplugin
│   ├── deploy
│   ├── docs
│   ├── include
│   ├── models
│   ├── sample
│   ├── src
│   └── test
├── SnapshotMethod
│   ├── config
│   ├── include
│   ├── src
│   └── test
├── vioplugin
│   ├── configs
│   ├── include
│   ├── src
│   └── test
├── xpluginflow
│   ├── doc
│   ├── include
│   ├── sample
│   ├── src
│   └── test
├── xpluginflow_msgtype
│   ├── doc
│   ├── include
│   ├── src
│   └── test
└── xroc-framework
    ├── cmake
    ├── config
    ├── deps -> ../external/
    ├── doc
    ├── example
    ├── include
    ├── src
    ├── test
    ├── tools
    └── tutorials
```
## external
依赖库安装目录，其中包含相关源码暂未开源给用户或公共第三方库， 比如 bpu_predict、hbipc、libyuv、opencv等;   

#### 预编译库
* jsoncpp：第三方开源库，用于json解析;
* libjpeg-turbo：第三方开源库，用于jpeg格式图像处理;
* libyuv：第三方开源库，用于yuv格式图像处理;
* opencv： 第三方开源库，主要用于计算机视觉处理;
* protobuf： 第三方开源库，多平台或者多模块间消息交互协议;
* gtest：第三方开源库，google test框架;
* hobotlog：地平线提供的日志系统，支持不同的优先级控制等特性;
* x2_prebuilt： x2平台底层系统库或者hbdk基础库等;
* bpu_predict：地平线bpu预测库;
* hbipc: CP-AP传输基础库;
* hobotsdk：另一种基于数据流的编程框架;
* ipc_tracking:基于IOU的多目标跟踪（MOT）策略库;
* MOTMethod：基于XRoc-framework封装的多目标跟踪（MOT）方法模块;
* vision_type： 提供了视觉业务需要常见数据结构定义，如bbox、landmark等;
* xroc-imagetools：对常用图像处理的封装;

## 开源库
* xpluginflow: 消息订阅与分发的编程框架，每个基础组件以插件（plugin）的形式管理;  
* xpluginflow_msgtype：定义了XPP基础消息类型;   
* vioplugin： 基于xpluginflow封装的用于vio管理的插件;   
* hbipcplugin：基于xpluginflow封装的用于AP-CP通信的插件;   
* xroc-framework： 一套基于数据流的编程框架;   
* fasterrcnnMethod: 基于地平线bpu的Fasterrcnn模型处理模块;
* CNNMethod: 基于地平线bpu常用cnn模型处理模块;
* facesnapfiltermethod: 人脸过滤策略模块;
* GradingMethod: 人脸综合打分模块，作为优选模块（Snapshotmethod）输入，是优选过程的重要参考;
* Snapshotmethod: 人脸优选模块。
* SmartPlugin: 基于xpluginflow的智能处理插件，监听vioplugin输出信息;默认实现是基于XRoc-framework面向workflow的通用接口实现了单workflow的AI应用运行框架;  
***基于该工程构建solution的入口main函数在smartplugin/sample/smart_main.cpp***

# 基于该repo二次开发升级solution
1. **适配vio**  
* 不同的终端设备可能会有不同的输入接口(mipi、bt1120)、不同的cam配置（单目、双目），以及不同的应用场景（实时视频流或者回灌模式）等，因此在构建solution之前需要先适配vio配置;   
* vio配置参数详解见[vioplugin](./vioplugin/README.md);   
* 如果当前vioplugin不能满足终端需求，请及时联系地平线人员提供支持。
2. **基于XRoc-framework集成模型、开发算法策略模块等**  
* XRoc-framework提供了面向数据流的编程框架，基于该框架可以将多个业务模块串联起来构建一个workflow，模型集成或者算法策略模块以method的形式加入workflow中。
* [tutorials](xroc-framework/tutorials/README.md) 、[XRoc的开发手册](xroc-framework/README.md);
* 该repo预提供了几个method（fasterrcnnmethod, cnnmethod, Snapfiltermethod, Gradingmethod，Snapshotmethod等）， 基于这些method可以构建人脸抓拍识别workflow;  
* Fasterrcnnmethod、CNNMethod对模型集成做了部分抽象，建议用户基于此完成自己的模型适配或者集成新的模型;
3. **基于SmartPlugin构建Solution**
* Smartplugin为该工程构建solution的入口，包含main程序（smartplugin/sample/smart_main.cpp）;
* 默认实现是基于XRoc-framework面向workflow的通用接口实现了单workflow的AI应用运行框架;大部分情况下，用户可基于该实现通过简单修改完成自己的solution构建，构建过程详细参考[smartplugin](./smartplugin/README.md);
* 如果默认构建不满足需求，用户还可以自定义plugin实现，自定义plugin的方式详见[xpluginflow](./xpluginflow/sample/sample_plugin.cpp);


