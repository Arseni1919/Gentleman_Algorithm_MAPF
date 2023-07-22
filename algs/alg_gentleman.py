import copy
from typing import List

from functions import *
from environments.env_classical_mapf import EnvClassicalMAPF
from algs.alg_space_time_a_star import build_heuristic_for_multiple_targets, h_func_creator, a_star, distance_nodes
from alg_one_path_search import one_path_search, OnePathNode
from alg_functions import *


class AgentAlgGA:
    def __init__(self, agent_index, node_start, node_goal):
        self.name = f'agent_{agent_index}'
        self.agent_index = agent_index
        self.node_start, self.node_goal = node_start, node_goal
        self.full_plan = [self.node_start]
        self.finished = False
        self.col_node = None
        self.plan = None
        self.aside_path = None
        self.aside_node = None
        self.plan_before = None
        self.plan_after = None
        self.plan_aside = None

    def reset(self):
        self.plan = None
        self.aside_path = None
        self.aside_node = None
        self.plan_before = None
        self.plan_after = None
        self.plan_aside = None


class ALgGA:
    """
    - For every next agent X:
        - Build path for agent X
        - Rest of agents don’t have intentions to move to their goal nodes until the agent X finishes his plan
        - If collided:
            - Choose some order of the rest of agents
            - Rest of agents find path aside from your path according to the order
            - Rest of agents calculate plans for those paths aside in the reversed order
            - Agent X calculates plan given their plans aside
            - Rest of agents calculate their plan back according to the order
            - Execute plans and stay on your final position
    """
    def __init__(self, env: EnvClassicalMAPF, h_func):
        self.name = "Gentleman Algorithm"
        self.env = env
        self.h_func = h_func
        # agents
        self.n_agents = len(self.env.start_positions)
        self.agents: List[AgentAlgGA] = []
        self.agents_dict = {}

    def create_agents(self):
        self.agents: List[AgentAlgGA] = []
        self.agents_dict = {}
        for a_index, (node_start, node_goal) in enumerate(zip(self.env.start_positions, self.env.goal_positions)):
            agent = AgentAlgGA(a_index, node_start, node_goal)
            self.agents.append(agent)
            self.agents_dict[agent.name] = agent

    def reset(self):
        for agent in self.agents:
            agent.reset()

    def build_initial_plan(self, curr_agent: AgentAlgGA):
        if curr_agent.finished:
            raise RuntimeError('[ERROR]: Nu how??')
        init_path, info = a_star(start=curr_agent.node_start, goal=curr_agent.node_goal, h_func=self.h_func,
                                 nodes=self.env.nodes, nodes_dict=self.env.nodes_dict)
        correct_times_in_nodes(init_path)
        self.agents_dict[curr_agent.name].plan = init_path

    def freeze_others(self, curr_agent):
        rest_of_agents = []
        for agent in self.agents:
            agent.reset()
            if agent.name != curr_agent.name:
                rest_of_agents.append(agent)
                if agent.finished:
                    agent.col_node = agent.node_goal
                else:
                    agent.col_node = agent.node_start
        return rest_of_agents

    def create_other_agents_plans(self, rest_of_agents, with_plans=False):
        other_agents_plans = {}
        for i_agent in rest_of_agents:
            if with_plans:
                if i_agent.plan:
                    other_agents_plans[i_agent.name] = i_agent.plan
                    continue
            if i_agent.finished:
                other_agents_plans[i_agent.name] = [i_agent.node_goal]
            else:
                other_agents_plans[i_agent.name] = [i_agent.node_start]
        return other_agents_plans

    def create_other_agents_full_plans(self, rest_of_agents, with_plans=False):
        other_agents_plans = {}
        for i_agent in rest_of_agents:
            if with_plans:
                if i_agent.full_plan:
                    other_agents_plans[i_agent.name] = i_agent.full_plan
                    continue
            if i_agent.finished:
                other_agents_plans[i_agent.name] = [i_agent.node_goal]
            else:
                other_agents_plans[i_agent.name] = [i_agent.node_start]
        return other_agents_plans

    def check_plans(self, curr_agent, rest_of_agents):
        """
        The assumption is: rest of agents stay in place.
        """
        other_agents_plans = self.create_other_agents_plans(rest_of_agents, with_plans=True)
        c_v_confs = c_v_check_for_agent(curr_agent.name, curr_agent.full_plan, other_agents_plans, immediate=False)
        c_e_confs = c_e_check_for_agent(curr_agent.name, curr_agent.full_plan, other_agents_plans, immediate=False)
        sorted_agents_in_conf = get_agents_in_conf(c_v_confs, c_e_confs)
        return sorted_agents_in_conf

    def check_full_plans(self, curr_agent, rest_of_agents):
        """
        The assumption is: rest of agents stay in place.
        """
        other_agents_plans = self.create_other_agents_full_plans(rest_of_agents, with_plans=True)
        c_v_confs = c_v_check_for_agent(curr_agent.name, curr_agent.full_plan, other_agents_plans, immediate=False)
        c_e_confs = c_e_check_for_agent(curr_agent.name, curr_agent.full_plan, other_agents_plans, immediate=False)
        sorted_agents_in_conf = get_agents_in_conf(c_v_confs, c_e_confs)
        return sorted_agents_in_conf

    def get_list_of_collided_agents(self, curr_agent, rest_of_agents):
        """
        The assumption is: rest of agents stay in place.
        """
        other_agents_plans = self.create_other_agents_full_plans(rest_of_agents, with_plans=False)
        c_v_confs = c_v_check_for_agent(curr_agent.name, curr_agent.plan, other_agents_plans, immediate=False)
        c_e_confs = c_e_check_for_agent(curr_agent.name, curr_agent.plan, other_agents_plans, immediate=False)
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

    def build_path_to_aside_node(self, curr_agent, err_aside_path_list):
        final_list = [(xy_name, 0) for xy_name in err_aside_path_list]
        perm_constr_dict = build_perm_constr_dict(self.env.nodes, final_list)
        aside_path, info = a_star(start=curr_agent.col_node, goal=curr_agent.aside_node, h_func=self.h_func,
                                  perm_constr_dict=perm_constr_dict,
                                  nodes=self.env.nodes, nodes_dict=self.env.nodes_dict)
        if not aside_path:
            return False, None
        correct_times_in_nodes(aside_path)
        aside_path_names = [node.xy_name for node in aside_path]
        for node_name in aside_path_names:
            if node_name in err_aside_path_list:
                raise RuntimeError('[ERROR]: cannot be!')
        return True, aside_path

    @staticmethod
    def get_err_lists(curr_agent, higher_order_rest_agents):
        # err_aside_node_list
        err_aside_node_list = [node.xy_name for node in curr_agent.plan]
        for h_agent in higher_order_rest_agents:
            err_aside_node_list.extend([node.xy_name for node in h_agent.aside_path])
        err_aside_node_list = list(set(err_aside_node_list))
        # err_aside_path_list
        err_aside_path_list = [curr_agent.node_start.xy_name]
        for h_agent in higher_order_rest_agents:
            err_aside_path_list.append(h_agent.col_node.xy_name)
        err_aside_path_list = list(set(err_aside_path_list))
        return err_aside_node_list, err_aside_path_list

    @staticmethod
    def create_path_to_go(plans: List):
        full_plan = []
        for i_plan in plans:
            full_plan.extend(i_plan)
        path_to_go = []
        for node in full_plan:
            if len(path_to_go) > 0:
                last_node = path_to_go[-1]
                if last_node.xy_name == node.xy_name:
                    continue
            path_to_go.append(node)
        update_times_in_nodes(path_to_go)
        return path_to_go

    def create_one_forward_plan_to_one_aside_path(self, some_agent, others_to_consider):
        new_plan = [some_agent.col_node]
        path_to_go = self.create_path_to_go([new_plan, some_agent.aside_path])
        other_paths = {agent.name: agent.plan for agent in others_to_consider}
        new_plan = one_path_search(path_to_go, other_paths, self.env.nodes)
        some_agent.plan = new_plan  # updates plan
        some_agent.plan_aside = new_plan[:]

    def get_all_agents_except_first_one(self, agents_with_plan):
        first_agent = agents_with_plan[0]
        other_agents, other_agents_dict = [], {}
        for o_agent in self.agents:
            if o_agent.plan and o_agent.name != first_agent.name:
                other_agents.append(o_agent)
                other_agents_dict[o_agent.name] = o_agent.plan
        return first_agent, other_agents, other_agents_dict

    @staticmethod
    def cut_before_and_after(curr_agent, conf_t):
        # sets plan_before, plan_after, col_node
        curr_agent.plan_before = curr_agent.plan[:conf_t + 1]
        curr_agent.plan_after = curr_agent.plan[conf_t + 1:]
        curr_agent.col_node = curr_agent.plan_before[-1]

    def find_aside_node_and_path(self, curr_agent, rest_of_agents):
        """
        - Rest of agents find path aside from your path according to the order
        """
        # build err_aside_node_list and err_aside_path_list
        higher_order_rest_agents = []
        for other_agent in rest_of_agents:
            err_aside_node_list, err_aside_path_list = self.get_err_lists(curr_agent, higher_order_rest_agents)
            # get the list of closest nodes (needed to be BFS)
            node_dist_list = [(node, distance_nodes(node, other_agent.col_node)) for node in self.env.nodes]
            sorted_node_dist_list = sorted(node_dist_list, key=lambda x: x[1])
            # try one by one if suited to be the aside_node
            succeeded = True
            for node, node_distance in sorted_node_dist_list:
                succeeded = False
                if node.xy_name in err_aside_node_list:
                    # cannot be used as an aside_node
                    continue
                other_agent.aside_node = node
                succeeded, aside_path = self.build_path_to_aside_node(other_agent, err_aside_path_list)
                other_agent.aside_path = aside_path
                if succeeded:
                    break
            if not succeeded:
                raise RuntimeError('[ERROR]: What happened?')

    def calc_forward_plans_to_aside_paths(self, curr_agent, rest_of_agents):
        reversed_rest_of_agents = rest_of_agents[::-1]
        others_to_consider = []
        # then all others ...4, 3, 2
        for aside_agent in reversed_rest_of_agents:
            if aside_agent.aside_path:
                self.create_one_forward_plan_to_one_aside_path(aside_agent, others_to_consider)  # updates plan
                others_to_consider.append(aside_agent)

    def recalc_plan_main_agent(self, curr_agent, rest_of_agents):
        # updates plan
        path_to_go = self.create_path_to_go([curr_agent.plan])
        other_paths = {agent.name: agent.plan for agent in rest_of_agents}
        new_plan = one_path_search(path_to_go, other_paths, self.env.nodes)
        if new_plan:
            curr_agent.plan = new_plan
        else:
            raise RuntimeError('[ERROR]: no no')

    def create_plans_to_consider(self, next_agent, others_to_consider):
        # all higher priority agents
        plans_to_consider_dict = {agent.name: agent.plan for agent in others_to_consider}
        return plans_to_consider_dict

    def calc_backward_plans(self, curr_agent, rest_of_agents):
        """
        - Rest of agents calculate their plan back according to the original order
        """
        # if curr_agent.name == 'agent_10':
        #     print()
        others_to_consider = [curr_agent]
        for next_agent in rest_of_agents:

            path_to_go = self.create_path_to_go([next_agent.aside_path[::-1]])
            plans_to_consider_dict = self.create_plans_to_consider(next_agent, others_to_consider)
            start_time = next_agent.plan[-1].t
            plan_after = one_path_search(path_to_go, plans_to_consider_dict, self.env.nodes, start_time)
            next_agent.plan.extend(plan_after[1:])
            others_to_consider.append(next_agent)

    def extend_full_plan(self, curr_agent, rest_of_agents, start_time):
        # all wait until start-time
        for agent in self.agents:
            last_node = agent.full_plan[-1]
            last_time = len(agent.full_plan)
            while len(agent.full_plan) < start_time:
                agent.full_plan.append(OnePathNode(last_node.x, last_node.y, last_time, last_node.neighbours))
                last_time += 1
        # curr_agent
        for node in curr_agent.plan:
            node.t += start_time
            node.ID = f'{node.x}_{node.y}_{node.t}'
        curr_agent.full_plan.extend(curr_agent.plan[1:])
        curr_agent.finished = True
        # rest_of_agents
        for o_agent in rest_of_agents:
            if o_agent.plan:
                for node in o_agent.plan:
                    node.t += start_time
                    node.ID = f'{node.x}_{node.y}_{node.t}'
                o_agent.full_plan.extend(o_agent.plan[1:])

    def get_new_start_time(self):
        new_start_time_list = []
        for agent in self.agents:
            new_start_time_list.append(len(agent.full_plan))
        return max(new_start_time_list)

    def plan_for_one(self, curr_agent, start_time):
        """
        - For every next agent X:
        """

        """
        - Rest of agents don’t have intentions to move to their goal nodes until the agent X finishes his plan
        """
        rest_of_agents_list = self.freeze_others(curr_agent)
        """
        - Build path for agent X
        """
        self.build_initial_plan(curr_agent)
        """
        - Choose some order of the rest of agents 
        """
        for rest_of_agents in itertools.permutations(rest_of_agents_list):
            rest_of_agents = list(rest_of_agents)
            _ = [agent.reset() for agent in rest_of_agents]
            print([i.name for i in rest_of_agents])
            """
            - If collided:
            """
            sorted_agents_in_conf = self.get_list_of_collided_agents(curr_agent, rest_of_agents)
            if len(sorted_agents_in_conf) > 0:
                conf = sorted_agents_in_conf.pop(0)
                """
                - Rest of agents find path aside from your path according to the order
                """
                # propagate and calc aside paths for all
                self.find_aside_node_and_path(curr_agent, rest_of_agents)  # sets aside_node, aside_path

                """
                - Rest of agents calculate plans for those paths aside in the reversed order
                """
                # calculate real plan to aside location
                self.calc_forward_plans_to_aside_paths(curr_agent, rest_of_agents)  # updates plan!

                """
                - Agent X calculates plan given their plans aside
                """
                # now back to the main agent - calculate the path for him given aside paths
                self.recalc_plan_main_agent(curr_agent, rest_of_agents)  # updates plan!
                """
                - Rest of agents calculate their plan back according to the order
                """
                # calculate paths back to initial plans of agents
                self.calc_backward_plans(curr_agent, rest_of_agents)  # updates plan!
            # get collided agents
            sorted_agents_in_conf = self.check_plans(curr_agent, rest_of_agents)
            if len(sorted_agents_in_conf) > 0:
                print('[ERROR]: Nah! Not this combo')
            else:
                break
        """
        - Execute plans and stay on your final position
        """
        self.extend_full_plan(curr_agent, rest_of_agents_list, start_time)
        # get collided agents
        sorted_agents_in_conf = self.check_full_plans(curr_agent, rest_of_agents_list)
        if len(sorted_agents_in_conf) > 0:
            print('[ERROR]: Nah! Check your code up there, something is screwed up.')
        new_start_time = self.get_new_start_time()
        print(f'Plan for {curr_agent.name} is done.')
        return new_start_time

    def plan_mapf(self):
        self.create_agents()
        # TODO: try different orders
        start_time = 0
        for agent in self.agents:
            start_time = self.plan_for_one(agent, start_time)

        return {agent.name: agent.full_plan for agent in self.agents}


def main():
    # set_seed(True)
    set_seed(random_seed_bool=False, i_seed=133)
    profiler = cProfile.Profile()

    n_agents = 20  # does not relevant if you provide start_points and goal_points
    path_to_maps = '../maps'
    # map_dir = 'warehouse-10-20-10-2-1.map'
    # map_dir = 'random-32-32-10.map'
    # map_dir = 'empty-48-48.map'
    map_dir = 'room-5-5.map'

    # map_dir = 'warehouse-10-20-10-2-1.map'
    to_render = True
    # to_render = False
    # ------------------------- #
    env = EnvClassicalMAPF(map_dir, path_to_maps=path_to_maps, n_agents=n_agents, to_render=to_render, plot_every=1)
    start_points = [(0, 0), (2, 0), (4, 0), (3, 1)]
    goal_points = [(3, 0), (2, 1), (1, 1), (1, 0)]
    # start_points = [(0, 0), (2, 0), (4, 0)]
    # goal_points = [(3, 0), (2, 1), (1, 1)]
    # start_points = [(0, 0), (3, 0)]
    # goal_points = [(4, 0), (3, 0)]
    # start_points = [(1, 0), (4, 0)]
    # goal_points = [(3, 0), (0, 0)]
    # start_points, goal_points = None, None
    env.create_new_problem(start_points, goal_points)
    # ------------------------- #
    h_dict = build_heuristic_for_multiple_targets(env.goal_positions, env.nodes, map_dim=(env.height, env.width))
    h_func = h_func_creator(h_dict)
    profiler.enable()
    alg = ALgGA(env, h_func)
    plans = alg.plan_mapf()
    profiler.disable()
    stats = pstats.Stats(profiler).sort_stats('cumtime')
    stats.dump_stats('../stats/results_ga.pstat')
    # ------------------------- #
    env.render(info={'i_step': 1, 'ga_plans': plans, 'pause_time': 0.5})
    plt.show()


if __name__ == '__main__':
    main()
