# tools

# 1、简介
本目录是作为xroc—framework各工具集而创建

# 2、文件列表

# 2.1、xroc_framework_datashow.py

该文件为xroc-framework工程的json数据流向图可视化工具，通过python + graphviz + dot来实现的。
工具通过解析json文件，会生成一份自定义格式的文档或图片，以直观的方式展现json文件中的数据流向。

工具的详细设计文档见：http://wiki.hobot.cc/pages/viewpage.action?pageId=77972994

注： 
    1、运行该文件前，系统需安装 python、graphviz库、json库
    2、该文件的运行格式为：python xxx.py xxx.json，若不带json文件，会提示“请在python文件后带json文件参数”

workflow配置合法性检查:
    1、某个node的inputs得不到feed，图中体现为：绿色填充，标签中带有字段：(No-Feed)
    2、全局inputs和全局outputs重名，图中体现为：红色双箭头边相连，边带有字段：same-global-inputs-outputs
    3、某些node间的outputs重名，图中体现为：玉米蓝色双箭头边相连，边带有字段：same-node-outputs

