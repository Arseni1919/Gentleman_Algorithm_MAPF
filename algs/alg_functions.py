from globals import *


def c_v_check_for_agent(agent_1: str, path_1, results: dict, immediate=False):
    """
    c_v_for_agent_list: (agents name 1, agents name 2, x, y, t)
    """
    if type(agent_1) is not str:
        raise RuntimeError('type(agent_1) is not str')
    c_v_for_agent_list = []
    if len(path_1) < 1:
        return c_v_for_agent_list
    for agent_2, path_2 in results.items():
        # if type(agent_2) is not str:
        #     raise RuntimeError('type(agent_2) is not str')
        if agent_2 != agent_1:
            for t in range(max(len(path_1), len(path_2))):
                node_1 = path_1[min(t, len(path_1) - 1)]
                node_2 = path_2[min(t, len(path_2) - 1)]
                if (node_1.x, node_1.y) == (node_2.x, node_2.y):
                    c_v_for_agent_list.append((agent_1, agent_2, node_1.x, node_1.y, t))
                    if immediate:
                        return c_v_for_agent_list
    return c_v_for_agent_list


def vertex_col_check(results, immediate=False):
    vertex_col_list = []
    for agent_1, path_1 in results.items():
        c_v_for_agent_list = c_v_check_for_agent(agent_1, path_1, results, immediate=immediate)
        vertex_col_list.extend(c_v_for_agent_list)
        if immediate:
            return vertex_col_list
    return vertex_col_list


def c_e_check_for_agent(agent_1: str, path_1, results: dict, immediate=False):
    """
    c_e_check_for_agent: (agents name 1, agents name 2, x, y, x, y, t)
    """
    if type(agent_1) is not str:
        raise RuntimeError('type(agent_1) is not str')
    c_e_for_agent_list = []
    if len(path_1) <= 1:
        return c_e_for_agent_list
    for agent_2, path_2 in results.items():
        if agent_2 != agent_1:
            if len(path_2) > 1:
                prev_node_1 = path_1[0]
                prev_node_2 = path_2[0]
                for t in range(1, min(len(path_1), len(path_2))):
                    node_1 = path_1[t]
                    node_2 = path_2[t]
                    if (prev_node_1.x, prev_node_1.y, node_1.x, node_1.y) == (node_2.x, node_2.y, prev_node_2.x, prev_node_2.y):
                        c_e_for_agent_list.append((agent_1, agent_2, prev_node_1.x, prev_node_1.y, node_1.x, node_1.y, t))
                        if immediate:
                            return c_e_for_agent_list
                    prev_node_1 = node_1
                    prev_node_2 = node_2
    return c_e_for_agent_list


def edge_col_check(results, immediate=False):
    edge_col_list = []
    for agent_1, path_1 in results.items():
        c_e_for_agent_list = c_e_check_for_agent(agent_1, path_1, results, immediate=immediate)
        edge_col_list.extend(c_e_for_agent_list)
        if immediate:
            return edge_col_list
    return edge_col_list


def check_for_collisions(results, immediate=False):
    """
    results: {'agents str': [Nodes heap_list]}
    """
    print('\nStart check_for_collisions...')
    if results:
        vertex_col_list = vertex_col_check(results, immediate=immediate)
        edge_col_list = edge_col_check(results, immediate=immediate)
        there_is_col = len(vertex_col_list) > 0 or len(edge_col_list) > 0
        return there_is_col, vertex_col_list, edge_col_list
    return True, [], []


def get_agents_in_conf(c_v_list, c_e_list):
    # c_v_for_agent_list.append((agent_1, agent_2, node_1.x, node_1.y, t))
    agents_in_conf = [(conf[1], conf[2], conf[3], conf[4]) for conf in c_v_list]
    # c_e_for_agent_list.append((agent_1, agent_2, prev_node_1.x, prev_node_1.y, node_1.x, node_1.y, t))
    agents_in_conf.extend([(conf[1], conf[4], conf[5], conf[6]-1) for conf in c_e_list])

    # sorted_agents_in_conf = sorted(agents_in_conf, key=lambda x: x[3], reverse=True)
    sorted_agents_in_conf = sorted(agents_in_conf, key=lambda x: x[3], reverse=False)
    return sorted_agents_in_conf


def build_perm_constr_dict(nodes, final_list):
    perm_constr_dict = {node.xy_name: [] for node in nodes}
    for final_item in final_list:
        xy_name, t = final_item
        perm_constr_dict[xy_name].append(t)
    return perm_constr_dict


def build_constraints(nodes, other_paths):
    v_constr_dict = {node.xy_name: [] for node in nodes}
    e_constr_dict = {node.xy_name: [] for node in nodes}
    perm_constr_dict = {node.xy_name: [] for node in nodes}

    for agent_name, path in other_paths.items():
        if len(path) > 0:
            final_node = path[-1]
            perm_constr_dict[final_node.xy_name].append(final_node.t)

            prev_node = path[0]
            for node in path:
                # vertex
                v_constr_dict[f'{node.x}_{node.y}'].append(node.t)
                # edge
                if prev_node.xy_name != node.xy_name:
                    e_constr_dict[f'{prev_node.x}_{prev_node.y}'].append((node.x, node.y, node.t))
                prev_node = node
    return v_constr_dict, e_constr_dict, perm_constr_dict
