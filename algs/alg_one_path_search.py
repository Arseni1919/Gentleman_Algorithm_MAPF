from globals import *
from alg_functions import *
from functions import set_seed
from environments.env_classical_mapf import EnvClassicalMAPF
from algs.alg_space_time_a_star import build_heuristic_for_multiple_targets, h_func_creator, a_star, distance_nodes


class OnePathNode:
    def __init__(self, x, y, t=0, neighbours=None):
        self.ID = f'{x}_{y}_{t}'
        self.xy_name = f'{x}_{y}'
        self.x = x
        self.y = y
        self.t = t
        if neighbours is None:
            self.neighbours = []
        else:
            self.neighbours = neighbours


def check_next_node_for_constraint(next_node, iteration, v_constr_dict, e_constr_dict, perm_constr_dict):
    # v_c
    v_c_times_list = v_constr_dict[next_node.xy_name]
    if len(v_c_times_list) > 0 and iteration <= max(v_c_times_list):
        return True
    # e_c - cannot be, because otherwise the v_c constraint will appear
    # e_c_times_list = [e_c[2] for e_c in e_constr_dict[next_node.xy_name]]
    # if len(e_c_times_list) and iteration <= max(e_c_times_list):
    #     return True
    # perm_c
    if len(perm_constr_dict[next_node.xy_name]) > 0:
        print(f'[ERROR]: pos {next_node.xy_name} has a value of {perm_constr_dict[next_node.xy_name]} in perm_constr_dict - no no no, that is nasty...')
        # raise RuntimeError(f'[ERROR]: pos {next_node.xy_name} has a value of {perm_constr_dict[next_node.xy_name]} in perm_constr_dict - no no no, that is nasty...')
    return False


def one_path_search(path_to_go, other_paths_dict, nodes, start_time=0):
    v_constr_dict, e_constr_dict, perm_constr_dict = build_constraints(nodes, other_paths_dict)
    first_node = path_to_go[0]
    prev_node = OnePathNode(first_node.x, first_node.y, start_time, first_node.neighbours)
    new_plan = [prev_node]
    iteration = start_time
    for next_node in path_to_go[1:]:
        there_is_collision = True
        while there_is_collision:
            iteration += 1
            if iteration > 1000:
                raise RuntimeError('[ERROR]: Is in the loop that is wrong!')
            there_is_collision = check_next_node_for_constraint(next_node, iteration,
                                                                v_constr_dict, e_constr_dict, perm_constr_dict)
            if there_is_collision:
                # wait in prev_node
                wait_node = OnePathNode(prev_node.x, prev_node.y, iteration, prev_node.neighbours)
                new_plan.append(wait_node)
                prev_node = wait_node
                continue
        next_node = OnePathNode(next_node.x, next_node.y, iteration, next_node.neighbours)
        new_plan.append(next_node)
        prev_node = next_node
    return new_plan


def main():
    set_seed(random_seed_bool=False, i_seed=121)
    # set_seed(random_seed_bool=True)
    profiler = cProfile.Profile()
    n_agents = 1
    path_to_maps = '../maps'
    # map_dir = 'random-32-32-10.map'
    map_dir = 'warehouse-10-20-10-2-1.map'
    to_render = True
    # to_render = False
    # ------------------------- #
    env = EnvClassicalMAPF(map_dir, path_to_maps=path_to_maps, n_agents=n_agents, to_render=to_render, plot_every=1)
    env.create_new_problem()
    # ------------------------- #
    h_dict = build_heuristic_for_multiple_targets(env.goal_positions, env.nodes, map_dim=(env.height, env.width))
    h_func = h_func_creator(h_dict)
    v_constr_dict = {node.xy_name: [] for node in env.nodes}
    e_constr_dict = {node.xy_name: [] for node in env.nodes}
    perm_constr_dict = {node.xy_name: [] for node in env.nodes}
    node_start = env.start_positions[0]
    node_goal = env.goal_positions[0]
    result, info = a_star(start=node_start, goal=node_goal, h_func=h_func,
                          v_constr_dict=v_constr_dict, e_constr_dict=e_constr_dict, perm_constr_dict=perm_constr_dict,
                          nodes_dict=env.nodes_dict, nodes=env.nodes
                          )
    # ------------------------- #
    profiler.enable()
    other_paths = {'agent_1': [
        OnePathNode(9, 3, 10, []),
        OnePathNode(8, 4, 10, []),
        OnePathNode(7, 5, 10, []),
    ]}
    new_plan = one_path_search(result, other_paths, env.nodes)
    profiler.disable()
    stats = pstats.Stats(profiler).sort_stats('cumtime')
    stats.dump_stats('../stats/one_path_search.pstat')
    # ------------------------- #
    env.render(info={'i_step': 1, 'one_agent_path': new_plan})
    if result:
        print('\nThe result is:\n', *[node.xy_name for node in result], sep='->')
        print('\nThe result IDs is:\n', *[node.ID for node in result], sep='->')
        print('\nThe new_plan is:\n', *[node.xy_name for node in new_plan], sep='->')
        print('\nThe new_plan IDs is:\n', *[node.ID for node in new_plan], sep='->')
    plt.show()
    # plt.close()
    # ------------------------- #
    # ------------------------- #


if __name__ == '__main__':
    main()

