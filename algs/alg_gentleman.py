import copy
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
    """
    For every next agent X:

        Calculate an initial path, while ignoring others

        If collided:
            Find path aside for X outside all the previous (higher priority) paths -
            the aside path does not allowed to cross the start and goal nodes of all the higher priority agents

            Calculate plans for yourself and for the path aside for all the above
            starting with the lowest (agent X) to highest (...4, 3, 2) priority if you have an aside path.
            If you do not have an aside path - you skip this step

            1-st agent calculates its path to the goal node

            All agents calculate their paths to goal nodes from their aside nodes (if no aside node -> start node),
            starting from 2, 3, 4 … until the agent X
    """
    def __init__(self, env: EnvClassicalMAPF, h_func):
        self.name = "Gentleman Algorithm"
        self.env = env
        self.h_func = h_func
        # agents
        self.n_agents = len(self.env.start_positions)
        self.agents: List[AgentAlgGA] = []
        self.agents_dict = {}
        self.create_agents()

    def create_agents(self):
        for a_index, (node_start, node_goal) in enumerate(zip(self.env.start_positions, self.env.goal_positions)):
            agent = AgentAlgGA(a_index, node_start, node_goal)
            self.agents.append(agent)
            self.agents_dict[agent.name] = agent

    def reset(self):
        for agent in self.agents:
            agent.reset()

    def build_initial_plan(self, curr_agent: AgentAlgGA):
        init_path, info = a_star(start=curr_agent.node_start, goal=curr_agent.node_goal, h_func=self.h_func,
                                 nodes=self.env.nodes, nodes_dict=self.env.nodes_dict)
        self.agents_dict[curr_agent.name].plan = init_path

    def get_list_of_collided_agents(self, agent):
        other_agents_plans = {}
        for i_agent in self.agents:
            if i_agent.plan:
                other_agents_plans[i_agent.name] = i_agent.plan
        del other_agents_plans[agent.name]
        c_v_confs = c_v_check_for_agent(agent.name, agent.plan, other_agents_plans, immediate=False)
        c_e_confs = c_e_check_for_agent(agent.name, agent.plan, other_agents_plans, immediate=False)
        sorted_agents_in_conf = get_agents_in_conf(c_v_confs, c_e_confs)
        return sorted_agents_in_conf

    @staticmethod
    def roll_until_conf_t(plan, conf_t):
        new_plan = plan[:]
        last_node = new_plan[-1]
        last_time = len(new_plan) - 1
        while last_time < conf_t-1:
            last_time += 1
            new_node = copy.deepcopy(last_node)
            new_node.t = last_time
            new_node.ID = f'{new_node.x}_{new_node.y}_{new_node.t}'
            new_plan.append(new_node)
        return new_plan

    def cut_before_and_after(self, curr_agent, conf_t):
        # sets plan_before, plan_after, col_node
        curr_agent.plan_before = curr_agent.plan[:conf_t + 1]
        curr_agent.plan_after = curr_agent.plan[conf_t + 1:]
        curr_agent.col_node = curr_agent.plan_before[-1]

        # for agent in self.agents:
        #     if agent.plan:
        #         if len(agent.plan) > conf_t:
        #             agent.plan_before = agent.plan[:conf_t+1]
        #             agent.plan_after = agent.plan[conf_t+1:]
        #         else:
        #             new_plan = self.roll_until_conf_t(agent.plan, conf_t)
        #             agent.plan = new_plan
        #             agent.plan_before = new_plan
        #             agent.plan_after = []
        #         agent.col_node = agent.plan_before[-1]

    def build_path_to_aside_node(self, curr_agent, err_aside_path_list):
        final_list = [(xy_name, 0) for xy_name in err_aside_path_list]
        perm_constr_dict = build_perm_constr_dict(self.env.nodes, final_list)
        aside_path, info = a_star(start=curr_agent.col_node, goal=curr_agent.aside_node, h_func=self.h_func,
                                  perm_constr_dict=perm_constr_dict,
                                  nodes=self.env.nodes, nodes_dict=self.env.nodes_dict)
        if not aside_path:
            return False
        aside_path_names = [node.xy_name for node in aside_path]
        for node_name in aside_path_names:
            if node_name in err_aside_path_list:
                raise RuntimeError('[ERROR]: cannot be!')
        curr_agent.aside_path = aside_path
        return True

    @staticmethod
    def get_err_lists(curr_agent, agents_with_plan):
        # build err_aside_node_list and err_aside_path_list
        err_aside_node_list = [node.xy_name for node in curr_agent.plan]
        for h_agent in agents_with_plan:
            err_aside_node_list.extend([node.xy_name for node in h_agent.plan])
        err_aside_path_list = []
        for h_agent in agents_with_plan:
            err_aside_path_list.append(h_agent.plan[0].xy_name)
            err_aside_path_list.append(h_agent.plan[-1].xy_name)
        return err_aside_node_list, err_aside_path_list

    def find_aside_node_and_path(self, curr_agent, agents_with_plan):
        # build err_aside_node_list and err_aside_path_list
        err_aside_node_list, err_aside_path_list = self.get_err_lists(curr_agent, agents_with_plan)

        # get the list of closest nodes (needed to be BFS)
        node_dist_list = [(node, distance_nodes(node, curr_agent.col_node)) for node in self.env.nodes]
        sorted_node_dist_list = sorted(node_dist_list, key=lambda x: x[1])

        # try one by one if suited to be the aside_node
        succeeded = True
        for node, node_distance in sorted_node_dist_list:
            succeeded = False
            if node.xy_name in err_aside_node_list:
                # cannot be used as an aside_node
                continue
            curr_agent.aside_node = node
            succeeded = self.build_path_to_aside_node(curr_agent, err_aside_path_list)
            if succeeded:
                break
        if not succeeded:
            raise RuntimeError('[ERROR]: What happened?')

    def check_for_conflicts_with_aside_path(self, curr_agent, conf_agent, aside_agents_done, conf_t):
        more_confs = []
        conf_agent_path_names = [node.xy_name for node in conf_agent.aside_path]
        for agent in self.agents:
            if agent.name != conf_agent.name and agent.name != curr_agent.name:  # other agent
                if agent.plan and agent not in aside_agents_done:  # that has a plan but without aside path
                    if agent.col_node.xy_name in conf_agent_path_names:  # and has a conflict with the conf_agent
                        more_confs.append((agent, agent.col_node.x, agent.col_node.y, conf_t))
        return more_confs

    @staticmethod
    def create_path_to_go(plan1: List, plan2: List):
        full_plan = plan1[:]
        full_plan.extend(plan2)
        path_to_go = []
        for node in full_plan:
            if len(path_to_go) > 0:
                last_node = path_to_go[-1]
                if last_node.xy_name == node.xy_name:
                    continue
            path_to_go.append(node)
        return path_to_go

    def create_one_forward_plan_to_one_aside_path(self, curr_agent, others_to_consider):
        new_plan = curr_agent.plan_before[:-1]
        path_to_go = self.create_path_to_go(new_plan, curr_agent.aside_path)
        new_plan = self.go_along_aside_path(path_to_go, others_to_consider)
        curr_agent.plan = new_plan  # updates plan

    def calc_forward_plans_to_aside_paths(self, curr_agent, agents_with_plan):
        reversed_agents_with_plan = agents_with_plan[::-1]
        others_to_consider = []
        if curr_agent.aside_path:
            self.create_one_forward_plan_to_one_aside_path(curr_agent, others_to_consider)  # updates plan
            others_to_consider.append(curr_agent)
        for aside_agent in reversed_agents_with_plan:
            if aside_agent.aside_path:
                self.create_one_forward_plan_to_one_aside_path(aside_agent, others_to_consider)  # updates plan
                others_to_consider.append(aside_agent)

    @staticmethod
    def check_next_node_for_constraint(path_node, iteration, prev_node, new_plan, others_to_consider):
        for other_agent in others_to_consider:
            # vertex conf check
            for o_node in other_agent.plan:
                if (o_node.x, o_node.y) == (path_node.x, path_node.y):
                    # this node will be passed by others in the future
                    if o_node.t >= iteration:
                        # wait in prev_node
                        prev_node = copy.deepcopy(prev_node)
                        prev_node.t = iteration
                        prev_node.ID = f'{prev_node.x}_{prev_node.y}_{prev_node.t}'
                        new_plan.append(prev_node)
                        return True
        return False

    def go_along_aside_path(self, path_to_go, others_to_consider):
        prev_node = copy.deepcopy(path_to_go[0])
        prev_node.t = 0
        prev_node.ID = f'{prev_node.x}_{prev_node.y}_{prev_node.t}'
        new_plan = [prev_node]
        for path_node in path_to_go[1:]:
            there_is_collision = True
            iteration = -1
            while there_is_collision:
                iteration += 1
                if iteration > 1000:
                    raise RuntimeError('[ERROR]: Is in the loop that is wrong!')
                there_is_collision = self.check_next_node_for_constraint(path_node, iteration, prev_node, new_plan, others_to_consider)

            next_node = copy.deepcopy(path_node)
            next_node.t = iteration
            next_node.ID = f'{next_node.x}_{next_node.y}_{next_node.t}'
            new_plan.append(next_node)
            prev_node = next_node
        return new_plan

    def create_one_backward_plan(self, aside_agent, others_to_consider):
        # updates plan
        succeeded = True
        # every aside_node is the last items of i_agent.plan
        new_plan = aside_agent.plan[:-1]
        next_t = len(new_plan)

        # append back from aside path
        reversed_aside_path = aside_agent.aside_path[::-1]
        path = self.go_along_aside_path(new_plan, reversed_aside_path, next_t, others_to_consider)

        # append continuing the path
        for next_node in aside_agent.plan_after:
            next_node.t = next_t
            next_node.ID = f'{next_node.x}_{next_node.y}_{next_node.t}'
            new_plan.append(next_node)
            next_t += 1

        aside_agent.plan = new_plan

        return succeeded

    def calc_backward_plans(self, agents_with_plan):
        """
        All agents calculate their paths to goal nodes from their aside nodes (if no aside node -> start node),
        starting from 2, 3, 4 … until the agent X
        """
        first_agent, other_agents, other_agents_dict = self.get_all_agents_except_first_one(agents_with_plan)
        others_to_consider = [first_agent]
        for next_agent in other_agents:

            if next_agent.aside_path:
                path_to_go = self.create_path_to_go(next_agent.plan, next_agent.plan_after)
            else:
                path_to_go = self.create_path_to_go(first_agent.plan, [])

            new_plan = self.go_along_aside_path(path_to_go, others_to_consider)
            next_agent.plan = new_plan
            others_to_consider.append(next_agent)

    def get_all_agents_except_first_one(self, agents_with_plan):
        first_agent = agents_with_plan[0]
        other_agents, other_agents_dict = [], {}
        for o_agent in self.agents:
            if o_agent.plan and o_agent.name != first_agent.name:
                other_agents.append(o_agent)
                other_agents_dict[o_agent.name] = o_agent.plan
        return first_agent, other_agents, other_agents_dict

    def recalc_plan_main_agent(self, agents_with_plan):
        # updates plan
        first_agent, other_agents, other_agents_dict = self.get_all_agents_except_first_one(agents_with_plan)
        path_to_go = self.create_path_to_go(first_agent.plan, [])
        new_plan = self.go_along_aside_path(path_to_go, other_agents)
        if new_plan:
            first_agent.plan = new_plan
        else:
            raise RuntimeError('[ERROR]: no no')

    def plan_for_one(self, curr_agent):
        agents_with_plan = [i_agent for i_agent in self.agents if i_agent.plan]
        print(f'[INFO]: agents_with_plan: {len(agents_with_plan)}')
        # initial plan
        self.build_initial_plan(curr_agent)
        sorted_agents_in_conf = self.get_list_of_collided_agents(curr_agent)

        if len(sorted_agents_in_conf) > 0:
            conf = sorted_agents_in_conf.pop(0)
            """
            Find path aside for X outside all the previous (higher priority) paths -
            the aside path does not allowed to cross the starting nodes of all the higher priority agents
            """
            # freeze everybody
            conf_name, conf_x, conf_y, conf_t = conf
            self.cut_before_and_after(curr_agent, conf_t)  # sets plan_before, plan_after, col_node

            # propagate and calc aside paths for all
            self.find_aside_node_and_path(curr_agent, agents_with_plan)  # sets aside_node, aside_path
            """
            Calculate plans for yourself and for the path aside for all the above
            starting with the lowest (agent X) to highest (...4, 3, 2) priority if you have an aside path.
            If you do not have an aside path - you skip this step
            """
            # calculate real plan to aside location
            self.calc_forward_plans_to_aside_paths(curr_agent, agents_with_plan)  # updates plan!
            """
            1-st agent calculates its path to the goal node
            """
            # now back to the main agent - calculate the path for him given aside paths
            self.recalc_plan_main_agent(agents_with_plan)  # updates plan!
            """
            All agents calculate their paths to goal nodes from their aside nodes (if no aside node -> start node),
            starting from 2, 3, 4 … until the agent X
            """
            # calculate paths back to initial plans of agents
            self.calc_backward_plans(agents_with_plan)  # updates plan!

        # get collided agents
        sorted_agents_in_conf = self.get_list_of_collided_agents(curr_agent)
        if len(sorted_agents_in_conf) > 0:
            raise RuntimeError('[ERROR]: Nah! Check your code up there, something is screwed up.')

    def plan_mapf(self):
        # TODO: try different orders
        for agent in self.agents:
            self.plan_for_one(agent)

        return {agent.name: agent.plan for agent in self.agents}


def main():
    n_agents = 10
    path_to_maps = '../maps'
    # map_dir = 'warehouse-10-20-10-2-1.map'
    # map_dir = 'random-32-32-10.map'
    # map_dir = 'empty-48-48.map'
    map_dir = 'room-5-5.map'

    # map_dir = 'warehouse-10-20-10-2-1.map'
    to_render = True
    # to_render = False
    env = EnvClassicalMAPF(map_dir, path_to_maps=path_to_maps, n_agents=n_agents, to_render=to_render, plot_every=1)
    # start_points = [(0, 0), (4, 0), (3, 0)]
    # goal_points = [(4, 0), (0, 0), (3, 0)]
    # start_points = [(0, 0), (3, 0)]
    # goal_points = [(4, 0), (3, 0)]
    start_points = [(0, 0), (4, 0)]
    goal_points = [(4, 0), (0, 0)]
    # start_points, goal_points = None, None
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


if __name__ == '__main__':
    # set_seed(True)
    set_seed(random_seed_bool=False, i_seed=133)
    profiler = cProfile.Profile()
    profiler.enable()
    main()
    profiler.disable()
    stats = pstats.Stats(profiler).sort_stats('cumtime')
    stats.dump_stats('../stats/results_ga.pstat')

# v_constr_dict, e_constr_dict, perm_constr_dict = build_constraints(self.env.nodes, other_paths=other_paths_dict)
# new_plan, info = a_star(start=first_agent.node_start, goal=first_agent.node_goal, h_func=self.h_func,
#                         v_constr_dict=v_constr_dict, e_constr_dict=e_constr_dict,
#                         perm_constr_dict=perm_constr_dict,
#                         nodes=self.env.nodes, nodes_dict=self.env.nodes_dict,
#                         )
