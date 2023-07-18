from globals import *
from environments.env_classical_mapf import EnvClassicalMAPF
import random
import heapq


class Node:
    def __init__(self, x, y, t=0, neighbours=None, new_ID=None):
        if new_ID:
            self.ID = new_ID
        else:
            self.ID = f'{x}_{y}_{t}'
        self.xy_name = f'{x}_{y}'
        self.x = x
        self.y = y
        self.t = t
        if neighbours is None:
            self.neighbours = []
        else:
            self.neighbours = neighbours
        # self.neighbours = neighbours

        self.h = 0
        self.g = t
        self.parent = None
        self.g_dict = {}

    def f(self):
        return self.t + self.h
        # return self.g + self.h

    def reset(self, target_nodes=None, **kwargs):
        if 'start_time' in kwargs:
            self.t = kwargs['start_time']
        else:
            self.t = 0
        self.h = 0
        self.g = self.t
        self.ID = f'{self.x}_{self.y}_{self.t}'
        self.parent = None
        if target_nodes is not None:
            self.g_dict = {target_node.xy_name: 0 for target_node in target_nodes}
        else:
            self.g_dict = {}


class ListNodes:
    def __init__(self, target_name=None):
        self.heap_list = []
        # self.nodes_list = []
        self.dict = {}
        self.h_func_bool = False
        if target_name:
            self.h_func_bool = True
            self.target_name = target_name

    def __len__(self):
        return len(self.heap_list)

    def remove(self, node):
        if self.h_func_bool:
            self.heap_list.remove((node.g_dict[self.target_name], node.xy_name))
            del self.dict[node.xy_name]
        else:
            if node.ID not in self.dict:
                raise RuntimeError('node.ID not in self.dict')
            self.heap_list.remove(((node.f(), node.h), node.ID))
            del self.dict[node.ID]
        # self.nodes_list.remove(node)

    def add(self, node):
        if self.h_func_bool:
            heapq.heappush(self.heap_list, (node.g_dict[self.target_name], node.xy_name))
            self.dict[node.xy_name] = node
        else:
            heapq.heappush(self.heap_list, ((node.f(), node.h), node.ID))
            self.dict[node.ID] = node
        # self.nodes_list.append(node)

    def pop(self):
        heap_tuple = heapq.heappop(self.heap_list)
        node = self.dict[heap_tuple[1]]
        if self.h_func_bool:
            del self.dict[node.xy_name]
        else:
            del self.dict[node.ID]
        # self.nodes_list.remove(node)
        return node

    def get(self, ID):
        return self.dict[ID]

    def get_nodes_list(self):
        return [self.dict[item[1]] for item in self.heap_list]


# funct_graph
def dist_heuristic(from_node, to_node):
    return np.abs(from_node.x - to_node.x) + np.abs(from_node.y - to_node.y)


def h_func_creator(h_dict):
    def h_func(from_node, to_node):
        if to_node.xy_name in h_dict:
            h_value = h_dict[to_node.xy_name][from_node.x, from_node.y]
            if h_value > 0:
                return h_value
        return np.abs(from_node.x - to_node.x) + np.abs(from_node.y - to_node.y)
        # return np.sqrt((from_node.x - to_node.x) ** 2 + (from_node.y - to_node.y) ** 2)

    return h_func


def h_get_node(successor_xy_name, node_current, nodes_dict):
    if node_current.xy_name == successor_xy_name:
        return None
    return nodes_dict[successor_xy_name]


def build_heuristic_for_one_target(target_node, nodes, map_dim, to_save=True, plotter=None, middle_plot=False):
    # print('Started to build heuristic...')
    copy_nodes = nodes
    nodes_dict = {node.xy_name: node for node in copy_nodes}
    target_name = target_node.xy_name
    target_node = nodes_dict[target_name]
    # target_node = [node for node in copy_nodes if node.xy_name == target_node.xy_name][0]
    # open_list = []
    # close_list = []
    open_nodes = ListNodes(target_name=target_node.xy_name)
    closed_nodes = ListNodes(target_name=target_node.xy_name)
    # open_list.append(target_node)
    open_nodes.add(target_node)
    iteration = 0
    # while len(open_list) > 0:
    while len(open_nodes) > 0:
        iteration += 1
        # node_current = get_node_from_open(open_list, target_name)
        node_current = open_nodes.pop()
        # if node_current.xy_name == '30_12':
        #     print()
        for successor_xy_name in node_current.neighbours:
            node_successor = h_get_node(successor_xy_name, node_current, nodes_dict)
            if node_successor:
                successor_current_g = node_current.g_dict[target_name] + 1  # h(now, next)

                # INSIDE OPEN LIST
                if node_successor.xy_name in open_nodes.dict:
                    if node_successor.g_dict[target_name] <= successor_current_g:
                        continue
                    open_nodes.remove(node_successor)
                    node_successor.g_dict[target_name] = successor_current_g
                    node_successor.parent = node_current
                    open_nodes.add(node_successor)

                # INSIDE CLOSED LIST
                elif node_successor.xy_name in closed_nodes.dict:
                    if node_successor.g_dict[target_name] <= successor_current_g:
                        continue
                    closed_nodes.remove(node_successor)
                    node_successor.g_dict[target_name] = successor_current_g
                    node_successor.parent = node_current
                    open_nodes.add(node_successor)

                # NOT IN CLOSED AND NOT IN OPEN LISTS
                else:
                    node_successor.g_dict[target_name] = successor_current_g
                    node_successor.parent = node_current
                    open_nodes.add(node_successor)

                # node_successor.g_dict[target_name] = successor_current_g
                # node_successor.parent = node_current

        # open_nodes.remove(node_current, target_name=target_node.xy_name)
        closed_nodes.add(node_current)

        if plotter and middle_plot and iteration % 1000 == 0:
            plotter.plot_lists(open_list=open_nodes.get_nodes_list(),
                               closed_list=closed_nodes.get_nodes_list(), start=target_node, nodes=copy_nodes)
        if iteration % 100 == 0:
            print(f'\riter: {iteration}', end='')

    if plotter and middle_plot:
        plotter.plot_lists(open_list=open_nodes.get_nodes_list(),
                           closed_list=closed_nodes.get_nodes_list(), start=target_node, nodes=copy_nodes)

    h_table = np.zeros(map_dim)
    for node in copy_nodes:
        h_table[node.x, node.y] = node.g_dict[target_name]
    # h_dict = {target_node.xy_name: h_table}
    # print(f'\rFinished to build heuristic at iter {iteration}.')
    return h_table


def build_heuristic_for_multiple_targets(target_nodes, nodes, map_dim, to_save=True, plotter=None, middle_plot=False):
    # print('Started to build heuristic...')
    h_dict = {}
    _ = [node.reset(target_nodes) for node in nodes]
    iteration = 0
    for node in target_nodes:
        h_table = build_heuristic_for_one_target(node, nodes, map_dim, to_save, plotter, middle_plot)
        h_dict[node.xy_name] = h_table

        # print(f'\nFinished to build heuristic for node {iteration}.')
        iteration += 1
    return h_dict


def make_neighbours(nodes):
    for node_1 in nodes:
        node_1.neighbours.append(node_1.xy_name)
        for node_2 in nodes:
            if node_1.xy_name != node_2.xy_name:
                dist = math.sqrt((node_1.x - node_2.x) ** 2 + (node_1.y - node_2.y) ** 2)
                if dist == 1.0:
                    node_1.neighbours.append(node_2.xy_name)


def distance_nodes(node1, node2, h_func: dict = None):
    if h_func is None:
        # print('regular distance')
        return np.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)
    else:
        heuristic_dist = h_func[node1.x][node1.y][node2.x][node2.y]
        # direct_dist = np.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)
        return heuristic_dist


# ------------------------------------------------------------------------------------- #
# ------------------------------------------------------------------------------------- #
# ------------------------------------------------------------------------------------- #
# ------------------------------------------------------------------------------------- #
# ------------------------------------------------------------------------------------- #


def check_future_constr(node_current, v_constr_dict, e_constr_dict, perm_constr_dict, ignore_dict, start):
    # NO NEED FOR wasted waiting
    if node_current.xy_name in ignore_dict:
        return False
    new_t = node_current.t + 1
    time_window = node_current.t - start.t
    if time_window < 20:
        return False

    future_constr = False
    for nei_xy_name in node_current.neighbours:
        if v_constr_dict and new_t in v_constr_dict[nei_xy_name]:
            future_constr = True
            break
        if e_constr_dict and (node_current.x, node_current.y, new_t) in e_constr_dict[nei_xy_name]:
            future_constr = True
            break
        if perm_constr_dict:
            if len(perm_constr_dict[nei_xy_name]) > 0:
                if new_t >= perm_constr_dict[nei_xy_name][0]:
                    future_constr = True
                    break
    return future_constr


def get_max_final(perm_constr_dict):
    if perm_constr_dict:
        final_list = [v[0] for k, v in perm_constr_dict.items() if len(v) > 0]
        max_final_time = max(final_list) if len(final_list) > 0 else 1
        return max_final_time
    return 1


def get_node(successor_xy_name, node_current, nodes, nodes_dict, open_nodes, closed_nodes, v_constr_dict, e_constr_dict,
             perm_constr_dict, max_final_time, **kwargs):
    new_t = node_current.t + 1

    if successor_xy_name not in nodes_dict:
        return None, ''

    if v_constr_dict:
        if new_t in v_constr_dict[successor_xy_name]:
            return None, ''

    if e_constr_dict:
        if (node_current.x, node_current.y, new_t) in e_constr_dict[successor_xy_name]:
            return None, ''

    if perm_constr_dict:
        if len(perm_constr_dict[successor_xy_name]) > 0:
            if len(perm_constr_dict[successor_xy_name]) != 1:
                raise RuntimeError('len(perm_constr_dict[successor_xy_name]) != 1')
            final_time = perm_constr_dict[successor_xy_name][0]
            if new_t >= final_time:
                return None, ''

    if max_final_time:
        if node_current.t >= max_final_time:
            new_t = max_final_time + 1

    new_ID = f'{successor_xy_name}_{new_t}'
    if new_ID in open_nodes.dict:
        return open_nodes.dict[new_ID], 'open_nodes'
    if new_ID in closed_nodes.dict:
        return closed_nodes.dict[new_ID], 'closed_nodes'

    node = nodes_dict[successor_xy_name]
    return Node(x=node.x, y=node.y, t=new_t, neighbours=node.neighbours), 'new'


def reset_nodes(start, goal, nodes, **kwargs):
    _ = [node.reset() for node in nodes]
    start.reset(**kwargs)
    return start, goal, nodes


def a_star(start, goal, nodes, h_func,
           v_constr_dict=None, e_constr_dict=None, perm_constr_dict=None,
           plotter=None, middle_plot=False,
           iter_limit=1e100, nodes_dict=None, **kwargs):
    """
    new_t in v_constr_dict[successor_xy_name]
    """
    start_time = time.time()
    # start, goal, nodes = deepcopy_nodes(start, goal, nodes)  # heavy!
    start, goal, nodes = reset_nodes(start, goal, nodes, **kwargs)
    # print('\rStarted A*...', end='')
    open_nodes = ListNodes()
    closed_nodes = ListNodes()
    node_current = start
    node_current.h = h_func(node_current, goal)
    open_nodes.add(node_current)
    max_final_time = get_max_final(perm_constr_dict)
    future_constr = False
    iteration = 0
    while len(open_nodes) > 0:
        iteration += 1
        if iteration > iter_limit:
            print(f'\n[ERROR]: out of iterations (more than {iteration})')
            return None, {'runtime': time.time() - start_time, 'n_open': len(open_nodes.heap_list),
                          'n_closed': len(closed_nodes.heap_list)}
        node_current = open_nodes.pop()

        if node_current.xy_name == goal.xy_name:
            # break
            # if there is a future constraint of a goal
            if v_constr_dict and len(v_constr_dict[node_current.xy_name]) > 0:
                # we will take the maximum time out of all constraints
                max_t = max(v_constr_dict[node_current.xy_name])
                # and compare to the current time
                # if it is greater, we will continue to expand the search tree
                if node_current.t > max_t:
                    # otherwise break
                    break
            else:
                break
        for successor_xy_name in node_current.neighbours:
            node_successor, node_successor_status = get_node(
                successor_xy_name, node_current, nodes, nodes_dict, open_nodes, closed_nodes,
                v_constr_dict, e_constr_dict, perm_constr_dict, max_final_time, **kwargs
            )

            successor_current_time = node_current.t + 1  # h(now, next)
            if node_successor is None:
                continue

            # INSIDE OPEN LIST
            if node_successor_status == 'open_nodes':
                if node_successor.t <= successor_current_time:
                    continue
                open_nodes.remove(node_successor)

            # INSIDE CLOSED LIST
            elif node_successor_status == 'closed_nodes':
                if node_successor.t <= successor_current_time:
                    continue
                closed_nodes.remove(node_successor)

            # NOT IN CLOSED AND NOT IN OPEN LISTS
            else:
                node_successor.h = h_func(node_successor, goal)
            node_successor.t = successor_current_time
            node_successor.g = node_successor.t
            node_successor.parent = node_current
            open_nodes.add(node_successor)

        # open_nodes.remove(node_current)
        closed_nodes.add(node_current)

        if plotter and middle_plot and iteration % 10 == 0:
            plotter.plot_lists(open_list=open_nodes.get_nodes_list(),
                               closed_list=closed_nodes.get_nodes_list(),
                               start=start, goal=goal, nodes=nodes, a_star_run=True)
        # print(f'\r(a_star) iter: {iteration}, closed: {len(closed_nodes.heap_list)}', end='')

    path = None
    if node_current.xy_name == goal.xy_name:
        path = []
        while node_current is not None:
            path.append(node_current)
            node_current = node_current.parent
        path.reverse()

    if plotter and middle_plot:
        plotter.plot_lists(open_list=open_nodes.get_nodes_list(),
                           closed_list=closed_nodes.get_nodes_list(),
                           start=start, goal=goal, path=path, nodes=nodes, a_star_run=True)
    # print('\rFinished A*.', end='')
    # if path is None:
    #     print()
    return path, {'runtime': time.time() - start_time, 'n_open': len(open_nodes.heap_list),
                  'n_closed': len(closed_nodes.heap_list), 'future_constr': future_constr}


def try_a_map_from_pic():
    n_agents = 1
    path_to_maps = '../maps'
    # map_dir = 'random-32-32-10.map'
    map_dir = 'warehouse-10-20-10-2-1.map'
    to_render = True
    # to_render = False
    env = EnvClassicalMAPF(map_dir, path_to_maps=path_to_maps, n_agents=n_agents, to_render=to_render, plot_every=1)
    env.create_new_problem()
    # ------------------------- #
    # build_heuristic_for_multiple_targets(target_nodes, nodes, map_dim, to_save=True, plotter=None, middle_plot=False)
    h_dict = build_heuristic_for_multiple_targets(env.goal_positions, env.nodes, map_dim=(env.height, env.width))
    h_func = h_func_creator(h_dict)
    # ------------------------- #
    # h_func = dist_heuristic
    # ------------------------- #
    # ------------------------- #
    # constraint_dict = None
    v_constr_dict = {node.xy_name: [] for node in env.nodes}
    e_constr_dict = {node.xy_name: [] for node in env.nodes}
    # v_constr_dict = {'30_12': [69], '29_12': [68, 69]}
    perm_constr_dict = {node.xy_name: [] for node in env.nodes}
    # ------------------------- #
    # ------------------------- #
    # result = a_star(start=node_start, goal=node_goal, nodes=nodes, h_func=h_func, plotter=plotter, middle_plot=False)
    profiler.enable()
    node_start = env.start_positions[0]
    node_goal = env.goal_positions[0]
    result, info = a_star(start=node_start, goal=node_goal, nodes=env.nodes, h_func=h_func,
                          v_constr_dict=v_constr_dict, e_constr_dict=e_constr_dict, perm_constr_dict=perm_constr_dict,
                          plotter=None, middle_plot=True, nodes_dict=env.nodes_dict,
                          )
    profiler.disable()
    env.render(info={'i_step': 1, 'one_agent_path': result})
    if result:
        print('The result is:', *[node.xy_name for node in result], sep='->')
        print('The result is:', *[node.ID for node in result], sep='->')
    # ------------------------- #
    # ------------------------- #
    plt.show()
    # plt.close()


if __name__ == '__main__':
    random_seed = True
    # random_seed = False
    seed = random.choice(range(1000)) if random_seed else 121
    random.seed(seed)
    np.random.seed(seed)
    print(f'SEED: {seed}')
    profiler = cProfile.Profile()
    try_a_map_from_pic()  # !!!
    # stats.print_stats()
    stats = pstats.Stats(profiler).sort_stats('cumtime')
    stats.dump_stats('../stats/results_a_star.pstat')

# def decision_rule(self, other):
#     # return bool(random.getrandbits(1))
#     return self.x > other.x or self.y > other.y
#
# def __lt__(self, other):
#     return self.decision_rule(other)
#
# def __le__(self, other):
#     return self.decision_rule(other)
#
# def __eq__(self, other):
#     return self.decision_rule(other)
#
# def __ne__(self, other):
#     return self.decision_rule(other)
#
# def __gt__(self, other):
#     return self.decision_rule(other)
#
# def __ge__(self, other):
#     return self.decision_rule(other)


# def set_nei(name_1, name_2, nodes_dict):
#     if name_1 in nodes_dict and name_2 in nodes_dict and name_1 != name_2:
#         node1 = nodes_dict[name_1]
#         node2 = nodes_dict[name_2]
#         dist = distance_nodes(node1, node2)
#         if dist == 1:
#             node1.neighbours.append(node2.xy_name)
#             node2.neighbours.append(node1.xy_name)
#
#
# def make_self_neighbour(nodes):
#     for node_1 in nodes:
#         node_1.neighbours.append(node_1.xy_name)
#
#
# def build_graph_from_np(img_np, show_map=False):
#     # 0 - wall, 1 - free space
#     nodes = []
#     nodes_dict = {}
#
#     x_size, y_size = img_np.shape
#     # CREATE NODES
#     for i_x in range(x_size):
#         for i_y in range(y_size):
#             if img_np[i_x, i_y] == 1:
#                 node = Node(i_x, i_y)
#                 nodes.append(node)
#                 nodes_dict[node.xy_name] = node
#
#     # CREATE NEIGHBOURS
#     # make_neighbours(nodes)
#
#     name_1, name_2 = '', ''
#     for i_x in range(x_size):
#         for i_y in range(y_size):
#             name_2 = f'{i_x}_{i_y}'
#             set_nei(name_1, name_2, nodes_dict)
#             name_1 = name_2
#
#     print('finished rows')
#
#     for i_y in range(y_size):
#         for i_x in range(x_size):
#             name_2 = f'{i_x}_{i_y}'
#             set_nei(name_1, name_2, nodes_dict)
#             name_1 = name_2
#     make_self_neighbour(nodes)
#     print('finished columns')
#
#     if show_map:
#         plt.imshow(img_np, cmap='gray', origin='lower')
#         plt.show()
#         # plt.pause(1)
#         # plt.close()
#
#     return nodes, nodes_dict
#
#
# def build_graph_nodes(img_np, show_map=False):
#     return build_graph_from_np(img_np, show_map)


# def main():
#     nodes = [
#         Node(x=1, y=1, neighbours=[]),
#         Node(x=1, y=2, neighbours=[]),
#         Node(x=1, y=3, neighbours=[]),
#         Node(x=1, y=4, neighbours=[]),
#         Node(x=2, y=1, neighbours=[]),
#         Node(x=2, y=2, neighbours=[]),
#         Node(x=2, y=3, neighbours=[]),
#         Node(x=2, y=4, neighbours=[]),
#         Node(x=3, y=1, neighbours=[]),
#         Node(x=3, y=2, neighbours=[]),
#         Node(x=3, y=3, neighbours=[]),
#         Node(x=3, y=4, neighbours=[]),
#         Node(x=4, y=1, neighbours=[]),
#         Node(x=4, y=2, neighbours=[]),
#         Node(x=4, y=3, neighbours=[]),
#         Node(x=4, y=4, neighbours=[]),
#     ]
#     make_neighbours(nodes)
#     node_start = nodes[0]
#     node_goal = nodes[-1]
#     # plotter = Plotter(map_dim=(5, 5), subplot_rows=1, subplot_cols=3)
#     result = a_star(start=node_start, goal=node_goal, nodes=nodes, h_func=dist_heuristic, plotter=None,
#                     middle_plot=True)
#
#     plt.show()
#     print(result)
#     plt.close()
