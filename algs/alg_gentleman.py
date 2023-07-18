from functions import *
from environments.env_classical_mapf import EnvClassicalMAPF
from algs.alg_space_time_a_star import build_heuristic_for_multiple_targets, h_func_creator, a_star


class ALgGA:
    def __init__(self, env, h_func):
        self.name = "Gentleman Algorithm"
        self.env = env
        self.h_func = h_func

    def plan_for_one(self):
        pass

    def plan_for_all(self):
        pass

    def plan(self):
        plans = {}
        return plans


def main():
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

    # alg
    alg = ALgGA(env, h_func)
    plans = alg.plan()

    env.render(info={'i_step': 1})
    plt.show()

    # node_start = env.start_positions[0]
    # node_goal = env.goal_positions[0]
    # result, info = a_star(start=node_start, goal=node_goal, nodes=env.nodes, h_func=h_func,
    #                       # v_constr_dict=v_constr_dict, e_constr_dict=e_constr_dict, perm_constr_dict=perm_constr_dict,
    #                       plotter=None, middle_plot=True, nodes_dict=env.nodes_dict,
    #                       )
    # env.render(info={'i_step': 1, 'one_agent_path': result})


if __name__ == '__main__':
    set_seed(True)
    # set_seed(random_seed_bool=False, i_seed=121)
    profiler = cProfile.Profile()
    profiler.enable()
    main()
    profiler.disable()
    stats = pstats.Stats(profiler).sort_stats('cumtime')
    stats.dump_stats('../stats/results_ga.pstat')



