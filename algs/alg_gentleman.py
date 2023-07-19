from typing import List

from functions import *
from environments.env_classical_mapf import EnvClassicalMAPF
from algs.alg_space_time_a_star import build_heuristic_for_multiple_targets, h_func_creator, a_star, distance_nodes
from alg_functions import *


class AgentAlgGA:
    def __init__(self, agent_index, node_start, node_goal):
        self.name = f'agent_{agent_index}'
        self.agent_index = agent_index
        self.node_start, self.node_goal = node_start, node_goal
        self.plan = None
        self.aside_path = None
        self.aside_node = None
        self.col_node = None
        self.plan_before = None
        self.plan_towards = None
        self.plan_backwards = None
        self.plan_after = None

    def reset(self):
        self.plan = None
        self.aside_path = None
        self.aside_node = None
        self.col_node = None
        self.plan_before = None
        self.plan_towards = None
        self.plan_backwards = None
        self.plan_after = None


class ALgGA:
    def __init__(self, env: EnvClassicalMAPF, h_func):
        self.name = "Gentleman Algorithm"
        self.env = env
        self.h_func = h_func
        # agents
        self.n_agents = len(self.env.start_positions)
        self.agents, self.agents_dict = [], {}
        self.create_agents()

    def create_agents(self):
        for a_index, (node_start, node_goal) in enumerate(zip(self.env.start_positions, self.env.goal_positions)):
            agent = AgentAlgGA(a_index, node_start, node_goal)
            self.agents.append(agent)
            self.agents_dict[agent.name] = agent

    def reset(self):
        for agent in self.agents:
            agent.reset()

    def build_initial_plan(self, agent, node_start, node_goal):
        init_path, info = a_star(start=node_start, goal=node_goal, h_func=self.h_func,
                                 nodes=self.env.nodes, nodes_dict=self.env.nodes_dict)
        self.agents_dict[agent.name].plan = init_path

    def get_list_of_collided_agents(self, agent):
        other_agents_plans = {}
        for i_agent in self.agents:
            if i_agent.plan:
                other_agents_plans[i_agent.name] = i_agent.plan
        del other_agents_plans[agent.name]
        c_v_confs = c_v_check_for_agent(agent.name, agent.plan, other_agents_plans, immediate=True)
        c_e_confs = c_e_check_for_agent(agent.name, agent.plan, other_agents_plans, immediate=True)
        sorted_agents_in_conf = get_agents_in_conf(c_v_confs, c_e_confs)
        return sorted_agents_in_conf

    @staticmethod
    def cut_before_and_after(i_agent: AgentAlgGA, i_x, i_y, i_t):
        for node_index, node in enumerate(i_agent.plan):
            if (node.x, node.y, node.t) == (i_x, i_y, i_t):
                i_agent.plan_before = i_agent.plan[:node_index]
                i_agent.plan_after = i_agent.plan[node_index:]
                i_agent.col_node = node
                return
        raise RuntimeError('[ERROR]: there is no such node in the path - an ERROR!')

    def build_path_to_aside_node(self, i_agent, forbidden_nodes_on_path_list):
        aside_path, info = a_star(start=i_agent.col_node, goal=i_agent.aside_node, h_func=self.h_func,
                                  nodes=self.env.nodes, nodes_dict=self.env.nodes_dict)
        aside_path_names = [node.xy_name for node in aside_path]
        for node in forbidden_nodes_on_path_list:
            if node.xy_name in aside_path_names:
                return False
        i_agent.aside_path = aside_path
        # is it touches anybody else?
        return True

    # needed to be recursive
    def find_aside_node_and_path(self, i_agent: AgentAlgGA, curr_agent: AgentAlgGA,
                                 higher_priority_agents: List[AgentAlgGA]):
        succeeded = False
        # build forbidden_list and forbidden_nodes_on_path_list
        forbidden_list = [node.xy_name for node in curr_agent.plan]
        for h_agent in higher_priority_agents:
            forbidden_list.extend([node.xy_name for node in h_agent.aside_path])
        forbidden_nodes_on_path_list = [curr_agent.plan[-1]]
        for h_agent in higher_priority_agents:
            forbidden_nodes_on_path_list.append(h_agent.col_node)
        # get the closest nodes (needed to be BFS)
        node_dist_list = [(node, distance_nodes(node, i_agent.col_node)) for node in self.env.nodes]
        sorted_node_dist_list = sorted(node_dist_list, key=lambda x: x[1])
        # try one by one if suited to be the aside_node
        for node, node_dist in sorted_node_dist_list:
            if node.xy_name in forbidden_list:
                continue
            i_agent.aside_node = node
            succeeded = self.build_path_to_aside_node(i_agent, forbidden_nodes_on_path_list)
            if succeeded:
                break
        return succeeded

    def execute_aside_calc(self, curr_agent, conf):
        aside_confs_todo = []
        aside_agents_done = []

        conf_name, conf_x, conf_y, conf_t = conf
        conf_agent: AgentAlgGA = self.agents_dict[conf_name]
        aside_confs_todo.append((conf_agent, conf_x, conf_y, conf_t))

        while len(aside_confs_todo) > 0:
            next_aside_conf = aside_confs_todo.pop(0)
            conf_agent, conf_x, conf_y, conf_t = next_aside_conf
            # freeze everybody
            self.cut_before_and_after(conf_agent, conf_x, conf_y, conf_t)
            # find aside_node + aside_path
            pass
            # add to the list of conflicted further agents
            pass

        # succeeded = self.find_aside_node_and_path(i_agent, curr_agent, aside_agents)
        # aside_agents.append(i_agent)
        # return succeeded, aside_agents
        return True, []

    @staticmethod
    def calc_real_plans_to_aside_locations(aside_agents):
        for i_agent in aside_agents:
            i_time = i_agent.plan_before[-1].t + 1
            for node in i_agent.aside_path:
                node.t += i_time
                node.ID = f'{node.x}_{node.y}_{node.t}'
            i_agent.plan_before.extend(i_agent.aside_path)
            i_agent.plan = i_agent.plan_before
        return True

    def recalc_plan_main_agent(self, curr_agent: AgentAlgGA):
        other_paths = {}
        for agent in self.agents:
            if agent.name != curr_agent.name:
                if agent.plan:
                    other_paths[agent.name] = agent.plan
        v_constr_dict, e_constr_dict, perm_constr_dict = build_constraints(self.env.nodes, other_paths=other_paths)
        new_plan, info = a_star(start=curr_agent.node_start, goal=curr_agent.node_goal, h_func=self.h_func,
                                v_constr_dict=v_constr_dict, e_constr_dict=e_constr_dict,
                                perm_constr_dict=perm_constr_dict,
                                nodes=self.env.nodes, nodes_dict=self.env.nodes_dict,
                                )
        if new_plan:
            succeeded = True
            curr_agent.plan = new_plan
        else:
            succeeded = False
        return succeeded

    @staticmethod
    def concat_backwards_plan(agent, backwards_plan):
        i_time = agent.plan[-1].t + 1
        for node in backwards_plan[1:]:
            node.t += i_time
            node.ID = f'{node.x}_{node.y}_{node.t}'
        agent.plan.extend(backwards_plan[1:])
        i_time = agent.plan[-1].t + 1
        for node in agent.plan_after[1:]:
            node.t += i_time
            node.ID = f'{node.x}_{node.y}_{node.t}'
        agent.plan.extend(agent.plan_after[1:])

    def cut_the_time(self, other_paths, t):
        cut_other_paths = {}
        # cut the paths
        for agent_name, path in other_paths.items():
            if len(path) >= t:
                cut_other_paths[agent_name] = path[t:]
        # update their time
        for agent_name, path in cut_other_paths.items():
            counter = 0
            for node in path:
                node.t = counter
                counter += 1
        return cut_other_paths

    def calc_plan_backwards(self, curr_agent, aside_agents):

        other_paths = {curr_agent.name: curr_agent.plan}
        for aside_agent in aside_agents:
            cut_other_paths = self.cut_the_time(other_paths, aside_agent.plan[-1].t)
            v_constr_dict, e_constr_dict, perm_constr_dict = build_constraints(self.env.nodes, other_paths=cut_other_paths)
            backwards_plan, info = a_star(start=aside_agent.aside_node, goal=aside_agent.col_node, h_func=self.h_func,
                                          v_constr_dict=v_constr_dict, e_constr_dict=e_constr_dict,
                                          perm_constr_dict=perm_constr_dict,
                                          nodes=self.env.nodes, nodes_dict=self.env.nodes_dict,
                                          )
            self.concat_backwards_plan(aside_agent, backwards_plan)
            other_paths[aside_agent.name] = aside_agent.plan
        return True

    def plan_for_one(self, agent, node_start, node_goal):
        print(f'[INFO]: Plan for {agent.name}')
        succeeded = False
        # initial plan
        self.build_initial_plan(agent, node_start, node_goal)
        sorted_agents_in_conf = self.get_list_of_collided_agents(agent)

        while len(sorted_agents_in_conf) > 0:
            conf = sorted_agents_in_conf.pop(0)
            # propagate and calc aside paths for all TODO:
            succeeded, aside_agents = self.execute_aside_calc(agent, conf)
            if not succeeded:
                break
            # calculate real plan to aside location TODO:
            succeeded = self.calc_real_plans_to_aside_locations(aside_agents)  # updates plan!
            if not succeeded:
                break

            # return to the main agent - calculate the path for him given aside paths
            succeeded = self.recalc_plan_main_agent(agent)  # updates plan!
            if not succeeded:
                break

            # calculate paths back to initial plans of agents TODO:
            succeeded = self.calc_plan_backwards(agent, aside_agents)  # updates plan!
            if not succeeded:
                break

            # get collided agents
            sorted_agents_in_conf = self.get_list_of_collided_agents(agent)

        return succeeded

    def plan_mapf(self):
        for agent in self.agents:
            succeeded = self.plan_for_one(agent, agent.node_start, agent.node_goal)

        return {agent.name: agent.plan for agent in self.agents}


def main():
    n_agents = 2
    path_to_maps = '../maps'
    # map_dir = 'random-32-32-10.map'
    # map_dir = 'empty-48-48.map'
    map_dir = 'room-5-5.map'

    # map_dir = 'warehouse-10-20-10-2-1.map'
    to_render = True
    # to_render = False
    env = EnvClassicalMAPF(map_dir, path_to_maps=path_to_maps, n_agents=n_agents, to_render=to_render, plot_every=1)
    start_points = [(0, 0), (3, 0), (4, 0)]
    goal_points = [(4, 0), (3, 0), (0, 0)]
    # start_points = [(0, 0), (4, 0)]
    # goal_points = [(4, 0), (0, 0)]
    env.create_new_problem(start_points, goal_points)
    # ------------------------- #
    # build_heuristic_for_multiple_targets(target_nodes, nodes, map_dim, to_save=True, plotter=None, middle_plot=False)
    h_dict = build_heuristic_for_multiple_targets(env.goal_positions, env.nodes, map_dim=(env.height, env.width))
    h_func = h_func_creator(h_dict)

    # alg
    alg = ALgGA(env, h_func)
    plans = alg.plan_mapf()

    env.render(info={'i_step': 1, 'ga_plans': plans})
    plt.show()

    # node_start = env.start_positions[0]
    # node_goal = env.goal_positions[0]
    # result, info = a_star(start=node_start, goal=node_goal, nodes=env.nodes, h_func=h_func,
    #                       # v_constr_dict=v_constr_dict, e_constr_dict=e_constr_dict, perm_constr_dict=perm_constr_dict,
    #                       plotter=None, middle_plot=True, nodes_dict=env.nodes_dict,
    #                       )
    # env.render(info={'i_step': 1, 'one_agent_path': result})


if __name__ == '__main__':
    # set_seed(True)
    set_seed(random_seed_bool=False, i_seed=133)
    profiler = cProfile.Profile()
    profiler.enable()
    main()
    profiler.disable()
    stats = pstats.Stats(profiler).sort_stats('cumtime')
    stats.dump_stats('../stats/results_ga.pstat')
