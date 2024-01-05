import random
import numpy as np
import json
import pandas as pd
import matplotlib.pyplot as plt


# 这个地方传drone_path的拷贝
# 返回值：路径是否成立，卡车路径，总共耗时，总价值
# 此函数是根据无人机的路径为卡车做路径规划
def get_truck_path(drone_path, drone_value):
    total_time_cost = 0
    nodes_not_passed_by_drones = [i for i in range(node_num) if i not in drone_path]
    new_drone_path = drone_path.copy()
    current_node = end_node = drone_path.pop(0)
    truck_path = [current_node]
    total_value = drone_value
    while True:
        flag = True
        if len(drone_path) >= 4 and flag:
            drone_flight_distance = graph[str(current_node)][str(drone_path[0])] + graph[str(drone_path[1])][str(drone_path[0])] + graph[str(drone_path[1])][str(drone_path[2])] + graph[str(drone_path[3])][str(drone_path[2])]
            drone_flight_time = drone_flight_distance / drone_speed
            # 在此处充电
            if drone_flight_distance < max_drone_range:
                # 卡车去一个无人机未去过的节点
                truck_node1 = None
                truck_node1_distance = np.inf
                truck_node2 = None
                truck_node2_distance = np.inf
                for node in nodes_not_passed_by_drones:
                    if graph[str(current_node)][str(node)] < truck_node1_distance and node in nodes_not_passed_by_drones:
                        truck_node1 = node
                        truck_node1_distance = graph[str(current_node)][str(node)]
                    if graph[str(drone_path[3])][str(node)] < truck_node2_distance and node in nodes_not_passed_by_drones:
                        truck_node2 = node
                        truck_node2_distance = graph[str(drone_path[3])][str(node)]
                if truck_node1 and truck_node1 == truck_node2 and (truck_node1_distance + truck_node2_distance) / truck_speed <= drone_flight_time:
                    truck_distance = truck_node1_distance + truck_node2_distance
                    truck_path.append(truck_node1)
                    nodes_not_passed_by_drones.remove(truck_node1)
                    total_value += value_lst[truck_node1]
                    truck_path.append(drone_path[3])
                elif truck_node1 != truck_node2 and (truck_node1_distance + truck_node2_distance + graph[str(truck_node2)][str(truck_node1)]) / truck_speed <= drone_flight_time:
                    truck_distance = truck_node1_distance + truck_node2_distance + graph[str(truck_node2)][str(truck_node1)]
                    truck_path.append(truck_node1)
                    truck_path.append(truck_node2)
                    truck_path.append(drone_path[3])
                    nodes_not_passed_by_drones.remove(truck_node1)
                    nodes_not_passed_by_drones.remove(truck_node2)
                    total_value += (value_lst[truck_node1] + value_lst[truck_node2])
                # 直接去下一个节点
                else:
                    truck_distance = graph[str(current_node)][str(drone_path[3])]
                    truck_path.append(drone_path[3])
                # 充电时间
                total_time_cost += charging_time
                # 赶路时间
                total_time_cost += max(drone_flight_distance / drone_speed, truck_distance / truck_speed)
                flag = False
                for _ in range(3):
                    drone_path.pop(0)
                current_node = drone_path.pop(0)
        if len(drone_path) >= 3 and flag:
            drone_flight_distance = graph[str(current_node)][str(drone_path[0])] + graph[str(drone_path[1])][
                str(drone_path[0])] + graph[str(drone_path[1])][str(drone_path[2])]
            drone_flight_time = drone_flight_distance / drone_speed
            # 在此处充电
            if drone_flight_distance < max_drone_range:
                # 卡车去两个无人机未去过的节点
                truck_node1 = None
                truck_node1_distance = np.inf
                truck_node2 = None
                truck_node2_distance = np.inf
                for node in nodes_not_passed_by_drones:
                    if graph[str(current_node)][str(node)] < truck_node1_distance and node in nodes_not_passed_by_drones:
                        truck_node1 = node
                        truck_node1_distance = graph[str(current_node)][str(node)]
                    if graph[str(drone_path[2])][str(node)] < truck_node2_distance and node in nodes_not_passed_by_drones:
                        truck_node2 = node
                        truck_node2_distance = graph[str(drone_path[2])][str(node)]

                if truck_node1 and truck_node1 == truck_node2 and (truck_node1_distance + truck_node2_distance) / truck_speed <= drone_flight_time:
                    truck_distance = truck_node1_distance + truck_node2_distance
                    truck_path.append(truck_node1)
                    nodes_not_passed_by_drones.remove(truck_node1)
                    total_value += value_lst[truck_node1]
                    truck_path.append(drone_path[2])
                elif truck_node1 != truck_node2 and (truck_node1_distance + truck_node2_distance + graph[str(truck_node2)][str(truck_node1)]) / truck_speed <= drone_flight_time:
                    truck_distance = truck_node1_distance + truck_node2_distance + graph[str(truck_node2)][str(truck_node1)]
                    truck_path.append(truck_node1)
                    truck_path.append(truck_node2)
                    truck_path.append(drone_path[2])
                    nodes_not_passed_by_drones.remove(truck_node1)
                    nodes_not_passed_by_drones.remove(truck_node2)
                    total_value += (value_lst[truck_node1] + value_lst[truck_node2])
                # 直接去下一个节点
                else:
                    truck_distance = graph[str(current_node)][str(drone_path[2])]
                    truck_path.append(drone_path[2])
                # 充电时间
                total_time_cost += charging_time
                # 赶路时间
                total_time_cost += max(drone_flight_distance / drone_speed, truck_distance / truck_speed)
                flag = False
                for _ in range(2):
                    drone_path.pop(0)
                current_node = drone_path.pop(0)
        if len(drone_path) >= 2 and flag:
            drone_flight_distance = graph[str(current_node)][str(drone_path[0])] + graph[str(drone_path[1])][
                str(drone_path[0])]
            drone_flight_time = drone_flight_distance / drone_speed
            # 在此处充电
            if drone_flight_distance < max_drone_range:
                # 卡车去两个无人机未去过的节点
                truck_node1 = None
                truck_node1_distance = np.inf
                truck_node2 = None
                truck_node2_distance = np.inf
                for node in nodes_not_passed_by_drones:
                    if graph[str(current_node)][str(node)] < truck_node1_distance and node in nodes_not_passed_by_drones:
                        truck_node1 = node
                        truck_node1_distance = graph[str(current_node)][str(node)]
                    if graph[str(drone_path[1])][str(node)] < truck_node2_distance and node in nodes_not_passed_by_drones:
                        truck_node2 = node
                        truck_node2_distance = graph[str(drone_path[1])][str(node)]
                if truck_node1 and truck_node1 == truck_node2 and (
                        truck_node1_distance + truck_node2_distance) / truck_speed <= drone_flight_time:
                    truck_distance = truck_node1_distance + truck_node2_distance
                    truck_path.append(truck_node1)
                    nodes_not_passed_by_drones.remove(truck_node1)
                    total_value += value_lst[truck_node1]
                    truck_path.append(drone_path[1])
                elif truck_node1 != truck_node2 and (
                        truck_node1_distance + truck_node2_distance + graph[str(truck_node2)][
                    str(truck_node1)]) / truck_speed <= drone_flight_time:
                    truck_distance = truck_node1_distance + truck_node2_distance + graph[str(truck_node2)][
                        str(truck_node1)]
                    truck_path.append(truck_node1)
                    truck_path.append(truck_node2)
                    truck_path.append(drone_path[1])
                    nodes_not_passed_by_drones.remove(truck_node1)
                    nodes_not_passed_by_drones.remove(truck_node2)
                    total_value += (value_lst[truck_node1] + value_lst[truck_node2])
                # 直接去下一个节点
                else:
                    truck_distance = graph[str(current_node)][str(drone_path[1])]
                    truck_path.append(drone_path[1])
                # 充电时间
                total_time_cost += charging_time
                # 赶路时间
                total_time_cost += max(drone_flight_distance / drone_speed, truck_distance / truck_speed)
                flag = False
                drone_path.pop(0)
                current_node = drone_path.pop(0)
        if len(drone_path) >= 1 and flag:
            drone_flight_distance = graph[str(current_node)][str(drone_path[0])]
            drone_flight_time = drone_flight_distance / drone_speed
            # 在此处充电
            if drone_flight_distance < max_drone_range:
                # 卡车去两个无人机未去过的节点
                truck_node1 = None
                truck_node1_distance = np.inf
                truck_node2 = None
                truck_node2_distance = np.inf
                for node in nodes_not_passed_by_drones:
                    if graph[str(current_node)][str(node)] < truck_node1_distance and node in nodes_not_passed_by_drones:
                        truck_node1 = node
                        truck_node1_distance = graph[str(current_node)][str(node)]
                    if graph[str(drone_path[0])][str(node)] < truck_node2_distance and node in nodes_not_passed_by_drones:
                        truck_node2 = node
                        truck_node2_distance = graph[str(drone_path[0])][str(node)]
                if truck_node1 and truck_node1 == truck_node2 and (
                        truck_node1_distance + truck_node2_distance) / truck_speed <= drone_flight_time:
                    truck_distance = truck_node1_distance + truck_node2_distance
                    truck_path.append(truck_node1)
                    nodes_not_passed_by_drones.remove(truck_node1)
                    total_value += value_lst[truck_node1]
                    truck_path.append(drone_path[0])
                elif truck_node1 != truck_node2 and (
                        truck_node1_distance + truck_node2_distance + graph[str(truck_node2)][
                    str(truck_node1)]) / truck_speed <= drone_flight_time:
                    truck_distance = truck_node1_distance + truck_node2_distance + graph[str(truck_node2)][
                        str(truck_node1)]
                    truck_path.append(truck_node1)
                    truck_path.append(truck_node2)
                    truck_path.append(drone_path[0])
                    nodes_not_passed_by_drones.remove(truck_node1)
                    nodes_not_passed_by_drones.remove(truck_node2)
                    total_value += (value_lst[truck_node1] + value_lst[truck_node2])
                # 直接去下一个节点
                else:
                    truck_distance = graph[str(current_node)][str(drone_path[0])]
                    truck_path.append(drone_path[0])
                # 充电时间
                total_time_cost += charging_time
                # 赶路时间
                total_time_cost += max(drone_flight_distance / drone_speed, truck_distance / truck_speed)
                current_node = drone_path.pop(0)
        if total_time_cost > max_time:
            return False, None, None, None
        # 判断是否为终点
        if current_node == end_node:
            # 到终点不用充电
            total_time_cost -= charging_time
            break
    a = [i for i in range(node_num) if i not in new_drone_path]
    value = 0
    for node in truck_path:
        if node in a:
            value += value_lst[node]
            a.remove(node)
    # if value != total_value-drone_value:
    #     print(value, total_value-drone_value)
    # print(type(drone_value))
    # return True, truck_path, total_time_cost, total_value
    return True, truck_path, total_time_cost, value + drone_value


# 生成初始种群
def generate_population():
    population = []
    for i in range(population_size):
        while True:
            flag = True
            current_node = start_node
            drone_path = [start_node]
            drone_value = 0
            drone_distance = 0
            nodes_not_passed_by_drones = [j for j in range(node_num)]
            choose_next_node_range = given_choose_next_node_range
            while True:
                nodes_not_passed_by_drones.remove(current_node)
                little_graph = graph[str(current_node)].copy()
                if len(drone_path) > min_drone_path_length and graph[str(current_node)][str(start_node)] < max_drone_range:
                    drone_distance += graph[str(current_node)][str(start_node)]
                    current_node = start_node
                else:
                    sorted_items = sorted(little_graph.items(), key=lambda x: x[1], reverse=False)
                    top_n_keys = [item[0] for item in sorted_items[:choose_next_node_range]]
                    loop_num = 0
                    while True:
                        loop_num += 1
                        next_node = int(random.choice(top_n_keys))
                        if next_node in nodes_not_passed_by_drones and graph[str(next_node)][str(current_node)] < max_drone_range:
                            break
                        if loop_num >= 100:
                            choose_next_node_range += 2
                            top_n_keys = [item[0] for item in sorted_items[:choose_next_node_range]]
                            loop_num -= 100
                            if choose_next_node_range > 30:
                                break
                    if choose_next_node_range <= 30:
                        drone_distance += graph[str(current_node)][str(next_node)]
                        current_node = next_node
                if choose_next_node_range > 30:
                    flag = False
                    break
                drone_value += value_lst[current_node]
                drone_path.append(current_node)
                if drone_distance > (max_time - timing_margin) * drone_speed / 2:
                    flag = False
                    # print('1')
                    break
                if current_node == start_node:
                    # print('2')
                    break
            if not flag:
                continue
            result, truck_path, total_time_cost, total_value = get_truck_path(drone_path.copy(), int(drone_value))
            if result:
                population.append([drone_path, drone_value, truck_path, total_time_cost, total_value])
                break
        # print(population[-1])
        # if int(population[-1][-1]) == 151:
        #     print('无人机路径：', population[-1][0])
        #     print('卡车路径：', population[-1][2])
        # print('无人机路径长度：{:d} 卡车路径长度：{:d} 耗时：{:.2f} 总价值：{:d}'.format(len(population[-1][0]), len(population[-1][2]), population[-1][-2], int(population[-1][-1])))
    return population


# 检查无人机路径相邻两节点间距离是否都小于无人机最大距离
def check_drone_path(drone_path):
    current_node = drone_path[0]
    for node in drone_path[1:]:
        if graph[str(current_node)][str(node)] >= max_drone_range:
            return False
        current_node = node
    return True


# 计算路径的总价值
def get_path_value(path):
    total_value = 0
    nodes_not_passed = [i for i in range(node_num)]
    for node in path:
        if node in nodes_not_passed:
            total_value += value_lst[node]
            nodes_not_passed.remove(node)
    return total_value


# 交叉
# [drone_path, drone_value, truck_path, total_time_cost, total_value]
# 此处的交叉操作和传统遗传算法中的交叉操作不同，是同一条染色体相邻的两个位置交换顺序
def crossover(parent):
    while True:
        crossover_position = random.randint(1, len(parent[0]) - 3)
        chile_path = parent[0].copy()
        a = chile_path[crossover_position]
        chile_path[crossover_position] = chile_path[crossover_position+1]
        chile_path[crossover_position + 1] = a
        if not check_drone_path(chile_path):
            continue
        result, truck_path, total_time_cost, total_value = get_truck_path(chile_path.copy(), int(parent[1]))
        if result:
            break
    # print('cross_over')
    return [chile_path, int(parent[1]), truck_path, total_time_cost, total_value]


# 变异
# [drone_path, drone_value, truck_path, total_time_cost, total_value]
# 此处的变异操作，对于一个染色体，其中的两个节点有概率被丢弃0.2/替换0.4/后面增添0.4
def mutate(parent):
    mutate_num = 0
    mutated_path = parent[0].copy()
    while True:
        nodes_not_passed_by_drones = [i for i in range(node_num) if i not in mutated_path]
        rand_num = random.random()
        mutate_position = random.randint(1, len(mutated_path) - 2)
        random_choice = random.choice(nodes_not_passed_by_drones)
        if len(nodes_not_passed_by_drones):
            if rand_num < 0.2:
                mutated_path.remove(mutated_path[mutate_position])
            elif 0.2 <= rand_num < 0.6:
                mutated_path[mutate_position] = random_choice
            else:
                mutated_path.insert(mutate_position+1, random_choice)
        else:
            mutated_path.remove(mutated_path[mutate_position])
        if check_drone_path(mutated_path):
            mutate_num += 1
            if mutate_num == 2:
                result, truck_path, total_time_cost, total_value = get_truck_path(mutated_path.copy(), get_path_value(mutated_path.copy()))
                if result:
                    break
                else:
                    mutate_num = 0
                    mutated_path = parent[0].copy()
        else:
            mutate_num = 0
            mutated_path = parent[0].copy()
    # print('mutate_over')
    return [mutated_path, get_path_value(mutated_path), truck_path, total_time_cost, total_value]


# 验证路程的总耗时
def total_time_cost_validation(drone_path, truck_path):
    start_node = drone_path.pop(0)
    truck_path.pop(0)
    drone_current_node = start_node
    truck_current_node = start_node
    total_time_cost = 0
    while len(truck_path):
        truck_distance = 0
        drone_distance = 0
        while True:
            new_truck_node = truck_path.pop(0)
            truck_distance += graph[str(truck_current_node)][str(new_truck_node)]
            truck_current_node = new_truck_node
            if new_truck_node in drone_path:
                while True:
                    new_drone_node = drone_path.pop(0)
                    drone_distance += graph[str(drone_current_node)][str(new_drone_node)]
                    drone_current_node = new_drone_node
                    if new_truck_node == new_drone_node:
                        total_time_cost += max(drone_distance / drone_speed, truck_distance / truck_speed)
                        total_time_cost += charging_time
                        break
                break
    total_time_cost -= charging_time
    print('total_time_cost:{:.2f}'.format(total_time_cost))


# 打印输出结果表格
def result_table(drone_path, truck_path):
    total_time_per_group = []
    nodes_not_passed = [i for i in range(80)]
    drone_path_in_group = []
    drone_path_in_groups = []
    truck_path_in_group = []
    truck_path_in_groups = []
    value_per_group = []
    drone_node = drone_path.pop(0)
    drone_path_in_group.append(drone_node)
    truck_node = truck_path.pop(0)
    truck_path_in_group.append(truck_node)
    while len(drone_path):
        drone_node = drone_path.pop(0)
        drone_path_in_group.append(drone_node)
        if drone_node in truck_path:
            drone_path_in_groups.append(drone_path_in_group)
            drone_path_in_group = [drone_node]
            while True:
                truck_node = truck_path.pop(0)
                truck_path_in_group.append(truck_node)
                if truck_node == drone_node:
                    truck_path_in_groups.append(truck_path_in_group)
                    truck_path_in_group = [truck_node]
                    break
    for i in range(len(truck_path_in_groups)):
        value = 0
        truck_node = truck_path_in_groups[i][0]
        truck_distance = 0
        drone_node = drone_path_in_groups[i][0]
        drone_distance = 0
        for j in truck_path_in_groups[i]:
            if j != truck_node:
                truck_distance += graph[str(truck_node)][str(j)]
            truck_node = j
            if j in nodes_not_passed:
                value += value_lst[j]
                nodes_not_passed.remove(j)
        for j in drone_path_in_groups[i]:
            if j != drone_node:
                drone_distance += graph[str(drone_node)][str(j)]
            drone_node = j
            if j in nodes_not_passed:
                nodes_not_passed.remove(j)
                value += value_lst[j]
        value_per_group.append(value)
        total_time_per_group.append(max(drone_distance / drone_speed, truck_distance / truck_speed)*3600)
    # print(drone_path_in_groups)
    # print(truck_path_in_groups)
    # print(value_per_group)
    # print(total_time_per_group)
    print('第i组\t\t\t每组任务的完成时间(s)\t\t\t无人机行驶路线\t\t\t卡车路线\t\t\t获得的价值')
    for i in range(len(drone_path_in_groups)):
        drone_str = '-'.join(list(map(str, drone_path_in_groups[i])))
        truck_str = '-'.join(list(map(str, truck_path_in_groups[i])))
        print('{}\t\t\t{:.2f}\t\t\t{}\t\t\t{}\t\t\t{:}'.format(i+1, total_time_per_group[i], drone_str, truck_str, value_per_group[i]))


# 画路径图
def draw_map(drone_path, truck_path):
    file_path = 'C:/Users/10146/Desktop/第四章/需求/运行文件/coords_and_values.csv'
    data = pd.read_csv(file_path).to_numpy()[:, 1:4]
    # 提取 x 和 y 坐标
    x = data[:, 0]
    y = data[:, 1]
    value = data[:, 2]
    # 创建并设置图形
    plt.figure()
    plt.scatter(x[value <= 1], y[value <= 1], s=20)
    plt.scatter(x[value == 2], y[value == 2], marker='s', s=20)
    plt.scatter(x[value > 2], y[value > 2], marker='*', s=80)
    # 设置绘图字体
    plt.rcParams['font.sans-serif'] = ['SimHei']

    plt.xlabel('经度')
    plt.ylabel('维度')
    plt.title('卡车-无人机协同搜救方案图')
    # 标注节点序号
    for i, txt in enumerate(range(1, len(data) + 1)):
        plt.annotate(txt-1, (x[i], y[i]), textcoords="offset points", xytext=(5, 5), ha='center')
    # 边缘点序号(excel表中的序号)
    outer_points_id = [56, 16, 59, 29, 65, 49, 13, 80, 15, 19, 38, 26, 14, 52]
    outer_points_indices = [outer_points_id[i] - 1 for i in range(len(outer_points_id))]
    # 连接最外面的边缘点成一条封闭的线
    outer_points = data[:, 0:2][outer_points_indices]
    # 添加第一个点以封闭线
    outer_points = np.vstack((outer_points, outer_points[0]))
    # 画边缘的点连线
    # plt.plot(outer_points[:, 0], outer_points[:, 1])
    # 画无人机和卡车行驶过的点
    drone_points = data[:, 0:2][drone_path]
    truck_points = data[:, 0:2][truck_path]
    plt.plot(drone_points[:, 0], drone_points[:, 1], color='red', linewidth=1.6, linestyle='--')
    plt.plot(truck_points[:, 0], truck_points[:, 1], color='green', linewidth=1.2,)
    plt.show()


if __name__ == '__main__':
    # 边缘点序号(共14个)
    outer_nodes_id = [56, 16, 59, 29, 65, 49, 13, 80, 15, 19, 38, 26, 14, 52]
    outer_nodes_index = [outer_nodes_id[i] - 1 for i in range(len(outer_nodes_id))]
    # 问题参数
    truck_speed = 25
    drone_speed = 70
    max_drone_range = 80
    max_drone_visits = 4
    charging_time = 0.4
    max_time = 36
    node_num = 80
    # 遗传算法参数
    population_size = 50
    num_generations = 50
    mutation_rate = 0.2
    elite_individuals_rate = 0.2
    crossover_rate = 0.6
    # 其他超参数
    # 初始生成的无人机路径最短长度
    min_drone_path_length = 35
    # 生成种群时，无人机选择距离最近的n个节点中的一个
    given_choose_next_node_range = 1
    # 生成的无人机路径耗时要留有一定的空间
    timing_margin = 0
    max_value_lst = []
    # 从文件中加载字典
    with open('C:/Users/10146/Desktop/第四章/需求/运行文件/graph.json', 'r') as f:
        graph = json.load(f)
    file_path = 'C:/Users/10146/Desktop/第四章/需求/运行文件/coords_and_values.csv'
    value_lst = pd.read_csv(file_path).to_numpy()[:, 3]
    population = []
    for start_node in outer_nodes_index:
        new_population = generate_population()
        # print(population[0])
        # print(max(population[:][-1]))
        population.extend(new_population)
    # 单个个体[drone_path, drone_value, truck_path, total_time_cost, total_value]
    # 遗传算法主循环
    for generation in range(num_generations):
        # 计算适应度并排序
        fitness_scores = [individual[4] for individual in population]
        sorted_indices = np.argsort(fitness_scores)[::-1]
        max_value = population[sorted_indices[0]][4]
        max_value_lst.append(max_value)
        # print(max_value)
        # 打印迭代结果
        print(f"Generation {generation}: Max Value = {max_value}")
        population = [population[i] for i in sorted_indices]
        # 选择精英个体
        elite_size = int(population_size * len(outer_nodes_id) * elite_individuals_rate)
        new_population = population[:elite_size]
        # 交叉操作
        for _ in range(int(population_size * len(outer_nodes_id) * crossover_rate)):
            parent = random.choice(population[:elite_size])
            child = crossover(parent.copy())
            new_population.append(child)
        # 变异操作
        for _ in range(int(population_size * len(outer_nodes_id) * mutation_rate)):
            parent = random.choice(population[:elite_size])
            mutated = mutate(parent)
            new_population.append(mutated)
        population = new_population
    # 获取最优路径
    best_path = population[0]
    # 输出最优路径的结果
    print('Best Path:\n', best_path)
    # 最大价值迭代结果
    # print(max_value_lst)
    drone_path = best_path[0]
    truck_path = best_path[2]
    # 耗费总时间验证
    # total_time_cost_validation(drone_path.copy(), truck_path.copy())
    # 打印输出各自的价值
    total_value = best_path[4]
    drone_value = 0
    for a in drone_path:
        if a not in truck_path:
            drone_value += value_lst[a]
    print('无人机单独获取的价值：{:d}  卡车单独获取的价值：{:d}  共同获取的价值：{:d}'.format(int(drone_value), int(total_value - best_path[1]), int(best_path[1] - drone_value)))
    # 输出结果表格
    result_table(drone_path.copy(), truck_path.copy())
    # 画结果的地图
    draw_map(drone_path.copy(), truck_path.copy())
    # 画出迭代过程折线图
    x = range(len(max_value_lst))
    # 设置绘图字体
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.figure()
    plt.plot(x, max_value_lst)
    plt.xlabel('迭代次数', fontsize=14)
    plt.ylabel('适应度函数', fontsize=14)
    plt.title('算法迭代曲线', fontsize=20)
    plt.show()



