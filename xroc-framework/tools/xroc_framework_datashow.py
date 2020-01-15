# !/usr/bin/pyton
# coding:utf-8
# Copyright (c) 2019 Horizon Robotics. All rights reserved.
# @author: shiqing.xie
# @email : shiqing.xie@horizon.ai
# @data  : 2019.12.09

# before running this python file, you must install python-lib: json,graphviz
# executing format: python xxx.py xxx.json

import json
import sys
from graphviz import Digraph

# 画颜色说明图


def draw_Illustrate(grap_g):
    with grap_g.subgraph(name='cluster0') as sub_g0:
        sub_g0.attr(label='颜色说明', color='green3')
        sub_g0.attr('node', style="filled")
        sub_g0.node('A', 'global slots', color="grey")
        sub_g0.node('B', 'inputs slots', color="red")
        sub_g0.node('C', 'outpus slots', color="Turquoise")
        sub_g0.node('D', 'No-Feed-inputs', color="green3")

# 画global-inputs输入图


def draw_inputs(p, q, global_intputs, node_input_record, grap_g):
    with grap_g.subgraph(name='cluster1') as sub_g1:
        sub_g1.attr(label='global-inputs')
        sub_g1.attr('node', color='grey', style='filled')
        tmp_node_inputs_name = 'input_type_' + str(p) + '_inputs_' + str(q)
        sub_g1.node(tmp_node_inputs_name, global_intputs)
        tmp_node_global_inputs = {tmp_node_inputs_name: global_intputs}
        # 将inputs记录到node_input_record，以便画edge
        node_input_record.update(tmp_node_global_inputs)

# 画method_single_inputs_node图


def draw_method_siginputs(inputs_node, i, j, sub_g, node_input_record,
                          cluster_input_node_record):
    tmp_node_method_inputs_name = 'node' + \
        str(i) + '_inputs_type_0' + '_inputs_' + str(j)
    if j == 0:
        tmp_cluster_input_node_record =\
            {'cluster-node_' + str(i + 1): tmp_node_method_inputs_name}
        cluster_input_node_record.update(tmp_cluster_input_node_record)
    # 画method的inputs节点,第i个method的第j个inputs
    sub_g.node(tmp_node_method_inputs_name,
               inputs_node[j], color="red")

    Is_inputs_get_feed = False
    for value in node_input_record.values():
        if inputs_node[j] == value:
            # 若inputs_node的label和node_input_record输出字典中的
            # 某一个值相匹配，则画边连接该node_input_record和
            # tmp_node_method_inputs_name，且由前者指向后者
            sub_g.edge(list(node_input_record.keys())[
                       list(node_input_record.values()).index(value)],
                       tmp_node_method_inputs_name)
            Is_inputs_get_feed = True
    # 检测node的inputs是否有feed
    if Is_inputs_get_feed is False:
        sub_g.node(tmp_node_method_inputs_name,
                   inputs_node[j] + '(No-Feed)', color="green3")

# 画method_multi_inputs_node图


def draw_method_mulinputs(inputs_node, i, j, k, sub_g, node_input_record,
                          cluster_input_node_record):
    tmp_node_method_inputs_name = 'node' + \
        str(i) + '_inputs_type_' + str(j) + '_inputs_' + str(k)
    if j == 0:
        tmp_cluster_input_node_record =\
            {'cluster-node_' + str(i + 1): tmp_node_method_inputs_name}
        cluster_input_node_record.update(tmp_cluster_input_node_record)
    # 画method的inputs节点,第i个method的第j个inputs
    sub_g.node(tmp_node_method_inputs_name,
               inputs_node[j]['inputs'][k], color="red")
    Is_inputs_get_feed = False
    for value in node_input_record.values():
        if inputs_node[j]['inputs'][k] == value:
            # 若inputs_node[j]['inputs']的label和node_input_record输
            # 出字典中的某一个值相匹配，则画边连接该node_input_record和
            # tmp_node_method_inputs_name，且由前者指向后者
            sub_g.edge(list(node_input_record.keys())[
                list(node_input_record.values()).index(value)],
                tmp_node_method_inputs_name)
            Is_inputs_get_feed = True
    # 检测node的inputs是否有feed
    if Is_inputs_get_feed is False:
        sub_g.node(tmp_node_method_inputs_name,
                   inputs_node[j]['inputs'][k] + '(No-Feed)', color="green3")

# 画method_single_outputs_node图


def draw_method_sigoutputs(outputs_node, i, k, sub_g, node_output_record,
                           node_input_record, tmp_node_output_record, grap_g):
    tmp_node_method_outputs_name = 'node' + \
        str(i) + '_outputs_type_0' + '_outputs_' + str(k)
    # 画method的outputs节点,第i个method的第k个outputs
    sub_g.node(tmp_node_method_outputs_name,
               outputs_node[k], color="Turquoise")
    tmp_node_method_outputs = {
        tmp_node_method_outputs_name: outputs_node[k]}

    if i > 0:
        for it in range(len(tmp_node_output_record)):
            if list(tmp_node_output_record.values())[it] ==\
                    outputs_node[k]:
                sub_g.edge(tmp_node_method_outputs_name,
                           list(tmp_node_output_record)[it],
                           color='dodgerblue', dir='both',
                           label='same-node-outputs',
                           fontcolor='dodgerblue', fontsize='20')
    # 加入到tmp_node_output_record 检测node间是否有重名的outputs
    tmp_node_output_record.update(tmp_node_method_outputs)
    # print(outputs_node[k])
    # print(tmp_node_output_record)
    # 将每个method的outputs添加到node_input_record，作为判
    # 断是否为下个method的inputs，以便画edge
    node_input_record.update(tmp_node_method_outputs)
    for value in node_output_record.values():
        if outputs_node[k] == value:
            # 若outputs_node的label和node_output_record输出字典中的某一个
            # 值相匹配，则画边连接该tmp_node_method_outputs_name和
            # node_output_record，且由前者指向后者
            sub_g.edge(tmp_node_method_outputs_name,
                       list(node_output_record.keys())[
                           list(node_output_record.values()).index(value)])

# 画method_multi_outputs_node图


def draw_method_muloutputs(outputs_node, i, x, y, sub_g, node_output_record,
                           node_input_record, tmp_node_output_record, grap_g):
    tmp_node_method_outputs_name = 'node' + \
        str(i) + '_outputs_type_' + \
        str(x) + '_outputs_' + str(y)
    # 画method的outputs节点,第i个method的第k个outputs
    sub_g.node(tmp_node_method_outputs_name,
               outputs_node[x]['outputs'][y], color="Turquoise")
    tmp_node_method_outputs = {
        tmp_node_method_outputs_name:
        outputs_node[x]['outputs'][y]}

    if i > 0:
        for it in range(len(tmp_node_output_record)):
            if list(tmp_node_output_record.values())[it] ==\
                    outputs_node[x]['outputs'][y]:
                sub_g.edge(tmp_node_method_outputs_name,
                           list(tmp_node_output_record)[it],
                           color='dodgerblue', dir='both',
                           label='same-node-outputs',
                           fontcolor='dodgerblue', fontsize='20')
    # 加入到tmp_node_output_record 检测node间是否有重名的outputs
    tmp_node_output_record.update(tmp_node_method_outputs)
    # print(tmp_node_output_record)
    # 将每个method的outputs添加到node_input_record，
    # 作为判断是否为下个method的inputs，以便画edge
    node_input_record.update(tmp_node_method_outputs)
    for value in node_output_record.values():
        if outputs_node[x]['outputs'][y] == value:
            # 若outputs_node的label和node_output_record输出字典中的某一个值相匹配，
            # 则画边连接该tmp_node_method_outputs_name和node_output_record，且由前者指向后者
            sub_g.edge(tmp_node_method_outputs_name,
                       list(node_output_record.keys())[
                           list(node_output_record.values()).index(value)])

# 画method图


def draw_method(i, method_type, unique_name, inputs_node, outputs_node,
                node_input_record, node_output_record, grap_g,
                cluster_input_node_record, tmp_node_output_record):
    with grap_g.subgraph(name='cluster-node_' + str(i + 1)) as sub_g:
        sub_g.attr(label='method_type:' + method_type +
                   '\n' + 'unique_name: ' + unique_name, rank='same')
        sub_g.attr('node', color='grey', style='filled')
        # 画inputs的node
        if type(inputs_node[0]) == str:
            for j in range(len(inputs_node)):
                draw_method_siginputs(inputs_node, i, j, sub_g,
                                      node_input_record,
                                      cluster_input_node_record)
        else:
            for j in range(len(inputs_node)):
                for k in range(len(inputs_node[j]['inputs'])):
                    draw_method_mulinputs(inputs_node, i, j, k, sub_g,
                                          node_input_record,
                                          cluster_input_node_record)
        # 画outputs的node
        if type(outputs_node[0]) == str:
            for k in range(len(outputs_node)):
                draw_method_sigoutputs(
                    outputs_node, i, k, sub_g, node_output_record,
                    node_input_record, tmp_node_output_record, grap_g)
        else:
            for x in range(len(outputs_node)):
                for y in range(len(outputs_node[x]['outputs'])):
                    draw_method_muloutputs(
                        outputs_node, i, x, y, sub_g, node_output_record,
                        node_input_record, tmp_node_output_record, grap_g)


# 画global-outputs输出图


def draw_outputs(j, k, global_outputs, node_output_record,
                 grap_g, node_input_record):
    with grap_g.subgraph(name='cluster4') as sub_g:
        sub_g.attr(label='global-outputs')
        sub_g.attr('node', color='grey', style='filled')
        tmp_node_output_name = 'output_type_' + str(j) + '_outputs_' + str(k)
        sub_g.node(tmp_node_output_name, global_outputs)
        tmp_node_global_outputs = {tmp_node_output_name: global_outputs}
        # 将outputs记录到node_output_record，以便画edge
        node_output_record.update(tmp_node_global_outputs)
        # 检测全局输入和输出重名
        for it in range(len(node_input_record)):
            if global_outputs == list(node_input_record.values())[it]:
                grap_g.edge(list(node_input_record)[it], tmp_node_output_name,
                            color='red', dir='both', fontcolor='red',
                            label='same-global-inputs-outputs', fontsize='20')


def draw_in_out_puts(inputs_in, label, execute_num, node_input_record,
                     node_output_record, grap_g):
    # 判断inputs第一个输出是否为字符串，若是，则输入为单一的默认的 input_type：
    if type(inputs_in[0]) == str:
        for p in range(len(inputs_in)):
            if execute_num == 1:
                draw_inputs(p + 1, p + 1, inputs_in[p],
                            node_input_record, grap_g)
            elif execute_num == 2:
                draw_outputs(p + 1, p + 1, inputs_in[p],
                             node_output_record, grap_g, node_input_record)
    # 判断inputs第一个输出是否为字符串，若否，则输入为josn
    # 设置的多种input_type型：input1\input2......
    else:
        for p in range(len(inputs_in)):
            for q in range(len(inputs_in[p][label])):
                if execute_num == 1:
                    draw_inputs(p + 1, q + 1, inputs_in[p][label][q],
                                node_input_record, grap_g)
                elif execute_num == 2:
                    # 画第 p + 1 个类型的第 q + 1 个inputs
                    draw_outputs(p + 1, q + 1, inputs_in[p][label][q],
                                 node_output_record, grap_g, node_input_record)


def analyz_json(inputs, optional, outputs, workflow, arry_len,
                node_input_record, node_output_record, grap_g,
                cluster_input_node_record):
    # 画颜色说明
    draw_Illustrate(grap_g)
    # 画global-inputs图
    inputs_label = 'inputs'
    execute_num = 1
    draw_in_out_puts(inputs, inputs_label, execute_num,
                     node_input_record, node_output_record, grap_g)

    # 画global-outputs输出图
    outputs_label = 'outputs'
    execute_num = 2
    draw_in_out_puts(outputs, outputs_label, execute_num,
                     node_input_record, node_output_record, grap_g)
    tmp_node_output_record = {}
    # 画method图
    for i in range(len(workflow)):
        method_type = workflow[i]['method_type']
        unique_name = workflow[i]['unique_name']
        inputs_node = workflow[i]['inputs']
        outputs_node = workflow[i]['outputs']
        draw_method(i, method_type, unique_name, inputs_node,
                    outputs_node, node_input_record, node_output_record,
                    grap_g, cluster_input_node_record, tmp_node_output_record)
    '''
    for i in range(len(cluster_input_node_record)):
        if i < len(cluster_input_node_record) - 1:
            grap_g.edge(list(cluster_input_node_record.values())[i],
                    list(cluster_input_node_record.values())[i + 1],
                    lhead=list(cluster_input_node_record)[i + 1],
                    ltail=list(cluster_input_node_record)[i],
                    arrowhead='none', style='dashed'
            )
    '''
    # 显示结果图
    grap_g.render('json数据流向图.gv', view=True)
    node_input_record.clear()
    node_output_record.clear()


def read_json(inputs, optional, outputs, workflow, arry_len):
    if len(sys.argv) != 2:
        print("请在python文件后带json文件参数")
        sys.exit()

    file_name = sys.argv[1]
    data = open(file_name)
    strJson = json.load(data)

    arry_len[0] = 0                          # optional_len
    arry_len[1] = len(strJson)               # json_len
    arry_len[2] = len(strJson["inputs"])     # input_len
    arry_len[3] = len(strJson["outputs"])    # output_len
    arry_len[4] = len(strJson["workflow"])   # workflow_len

    for key in strJson.keys():
        if key == "optional":
            arry_len[0] = len(strJson["optional"])
            optional.update(strJson["optional"])   # 添加字典元素
        elif key == "max_running_count":
            max_running_count = strJson["max_running_count"]

    inputs.extend(strJson["inputs"])
    outputs.extend(strJson["outputs"])
    workflow.extend(strJson["workflow"])


def run_main():
    inputs = []                   # 记录json文件的inputs数据
    outputs = []                  # 记录json文件的outputs数据
    optional = {}                 # 记录json文件的optional数据
    workflow = []                 # 记录json文件的workflow数据
    # 记录optional、json、inputs、outputs、workflow各自的个数
    arry_len = [0, 0, 0, 0, 0]
    # 记录从global-inputs到各个method的inputs，以列表的方式记录，
    # 以便画箭头连线，记录类型为{"节点name":'节点label'}，若label相等，则画edge
    node_input_record = {}
    # 记录从各个method的outputs到global-outputs的ouputs，以列表的方式记录，
    # 记录类型为{"节点name":'节点label'}，若label相等，则画edge
    node_output_record = {}
    cluster_input_node_record = {}
    # 画主图
    grap_g = Digraph(format="png", graph_attr={"color": 'blue',
                                               "fontname": 'FangSong',
                                               "rankdir": 'LR',
                                               "compound": 'true',
                                               "ranksep": "2"},
                     node_attr={'color': 'grey', 'style': 'filled'})

    read_json(inputs, optional, outputs, workflow, arry_len)
    analyz_json(inputs, optional, outputs, workflow, arry_len,
                node_input_record, node_output_record, grap_g,
                cluster_input_node_record)


if __name__ == "__main__":
    run_main()
