
#
子workflow Json格式
#

配置文件以json格式表示。

json中，{} 表示一个对象。

在一个XRoc配置文件中，有如下几种对象：
* Root
* Template
* TemplateRef
* Workflow
* Node

##
Root对象
##
基本格式如下：
```
{
    "type": "root",    // 表示Root对象
	"version": "1.1",  // 表示配置版本
	"templates": [     // Template对象，可选
	],
	"workflows": [     // Workflow对象
	]
}
```
Root对象包含了：
* version：协议的版本；若配置包含子workflow，则version字段必需
* templates：定义的模板；
* workflows：定义的工作流；

##
Template对象
##
Template对象用于模板功能。用户可以通过Template对象定义一些带参数的模板，然后通过TemplateRef对象引用这些模板，即可实例化对象。

基本格式如下：
```
{
    "type": "template",           // 表示Template对象
	"template_name": "XXXX",      // 模板名称，需唯一化
	"parameters": [               // 模板参数
	],
	"template": {                 // 模板内容
	}
}
```
* template_name 字段表示该模板名称，是模板的唯一性标志；
* parameters 字段指明了这个template所需要的参数；
* template 字段是这个模板的内容；在模板内容中，${xxx}格式的字符串，表示parameters字段中所定义的参数，在实例化该模板时直接替换为参数内容。需要注意，在定义的模板中，若存在workflow输入输出name参数，只需在global inputs、outputs用${name}格式表示，模板其他位置直接用参数name表示。

如下为例，定义了一个名为"cnn.tpl"的Template对象：
```
{
	"type": "template",
	"template_name": "cnn.tpl",
	"parameters": [
	    "name",
	    "pre_method",
	    "post_method",
	    "inputs",
	    "outputs"
	],
	"template": {
	    "name": "${name}",
	    "type": "workflow",
	    "inputs": ["${inputs}"],
	    "outputs": ["${outputs}]",
	    "workflow": [
	        {
	            "type": "node",
	            "method_type": "${pre_method}",
	            "unique_name": "pre",
	            "inputs": ["inputs"],
	            "outputs": [
	                "pre_out0",
	                "pre_out1"
	            ]
	        },
	        {
	            "type": "node",
	            "method_type": "CNNMethod",
	            "unique_name": "cnn",
	            "inputs": [
	                "pre_out0",
	                "pre_out1"
	            ],
	            "outputs": [
	                "cnn_out0"
	            ]
	        },
	        {
	            "type": "node",
	            "method_type": "${post_method}",
	            "unique_name": "post",
	            "inputs": ["cnn_out0"],
	            "outputs": ["outputs"]
	        }
	    ]
	}
}
```

##
TemplateRef对象
##
TemplateRef对象即对之前定义的Template的引用。
基本格式如下：
```
{
    "type": "template_ref",       // 表示TemplateRef对象
	"template_name": "XXXX",      // 引用的模板名称
	"parameters": {               // 模板参数
	}
}
```
其中：
* template_name 指明了需要引用的template的名字；
* parameters 指定了这个template 所需要的参数的值。
如下为例，定义了一个TemplateRef对象：
```
{
    "type": "template_ref",
	"template_name": "cnn.tpl",
	"parameters": {
    	"name": "vehicle_cnn",
		"pre_method": "VehiclePreProcess",
		"post_method": "VehiclePostProcess",
		"input": "image",
		"output": "vehicle_box"
	}
}
```
结合Template对象，上述TemplateRef对象会被展开，等价于如下定义：
```
{
    "name": "vehicle_cnn",
    "type": "workflow",
    "inputs": ["image"],
    "outputs": ["vehicle_box"],
    "workflow": [
        {
            "type": "node",
            "method_type": "VehiclePreProcess",
            "unique_name": "pre",
            "inputs": ["image"],
            "outputs": [
                "pre_out0",
                "pre_out1"
            ]
        },
        {
            "type": "node",
            "method_type": "CNNMethod",
            "unique_name": "cnn",
            "inputs": [
                "pre_out0",
                "pre_out1"
            ],
            "outputs": [
                "cnn_out0"
            ]
        },
        {
            "type": "node",
            "method_type": "VehiclePostProcess",
            "unique_name": "post",
            "inputs": ["cnn_out0"],
            "outputs": ["vehicle_box"]
        }
	]
}
```
##
Workflow对象
##
基本格式如下：
```
{
	"name": "xxx",         // workflow名称，需唯一化
	"type": "workflow",    // 表示Workflow对象
	"inputs": [],          // 输入slots名称
	"outputs": [],         // 输出slots名称
	"workflow": [          // Workflow内容，包括Node或Workflow对象(子workflow)
	]
}
```
示例：
```
{
	"name": "main",
	"type": "workflow",
	"inputs": ["image"],
	"outputs": [
        "face_lmk",
        "face_pose",
        "face_box"
    ],
	"workflow": [
	    {
	        "type":"node",
	        "unique_name": "fasterrcnndet",
	        "method_type": "FasterRCNNMethod",
	        "inputs": ["image"],
	        "outputs": ["face_box"]
	        //...
	    },
	    {
	        "type": "template_ref",
	        "template_name": "cnn.tpl",
	        "parameters": {
	            "name": "face_cnn",
	            "pre_method": "FacePreProcess",
	            "post_method": "FacePostProcess",
	            "input": ["face_box"],
	            "output": ["face_lmk", "face_pose"]
	       }
	    }  // 该TemplateRef对象展开后是Workflow对象
	]
}
```
##
Node对象
##
基本格式：
```
{
    "type": "node",                   // 表示Node对象
    "method_type": "TestMethod",      // Method类型，需要实现该Method
    "unique_name": "method",          // Node名称，唯一性标志
    "inputs": ["global_in"],          // 输入slots
    "outputs": ["global_out"]         // 输出slots
    // ...
}
```

##
include机制
##
在配置文件中，如果一个字符串的内容为：
```
"@include: path/to/file"
```
那么，这个字符串形成了一个include指令。这个字符串会被替换成path/to/file这个路径指定的文件的内容。
例如：
```
{
    "type": "node",
    "unique_name": "method",
    "method_type": "BBoxFilter",
    "method_config": "@include:conf/bbox.json"
    // ...
}
```
如果bbox.json的内容为：
```
{
    "threshold": 2.0
}
```
那么，上述node对象会被定义为：
```
{
    "type": "node",
    "unique_name": "method",
    "method_type": "BBoxFilter",
    "method_config": {
        "threshold": 2.0
    }
    // ...
}
```
