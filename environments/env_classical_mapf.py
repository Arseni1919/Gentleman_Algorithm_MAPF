from environments.env_functions import *
from plot_functions.plot_functions import *


class EnvClassicalMAPF:
    def __init__(self, map_dir, path_to_maps, show_map=False, n_agents=3, to_render=True, plot_every=1):
        self.name = 'Classical MAPF Problem'
        self.map_dir = map_dir
        self.path_to_maps = path_to_maps
        self.show_map = show_map
        self.n_agents = n_agents
        self.to_render = to_render
        self.plot_every = plot_every
        self.map_np, (self.height, self.width), self.nodes, self.nodes_dict = None, (-1, -1), None, None
        self.start_positions = None
        self.goal_positions = None

        if self.to_render:
            self.fig, self.ax = plt.subplot_mosaic("AAB;AAC;AAD", figsize=(12, 8))

    def create_new_problem(self):
        self.map_np, (self.height, self.width), self.nodes, self.nodes_dict = create_nodes_from_map(
            self.map_dir, path=self.path_to_maps, show_map=self.show_map)
        positions_pool = random.sample(self.nodes, self.n_agents * 2)
        self.start_positions = positions_pool[:self.n_agents]
        self.goal_positions = positions_pool[self.n_agents:]

    def reset(self):
        pass

    def render(self, info):
        if self.to_render:
            info = AttributeDict(info)
            if info.i_step % self.plot_every == 0:
                info.update({
                    'env_name': self.name,
                    'map_dir': self.map_dir,
                    'map_np': self.map_np,
                    'height': self.height,
                    'width': self.width,
                    'nodes': self.nodes,
                    'nodes_dict': self.nodes_dict,
                    'n_agents': self.n_agents,
                    'start_positions': self.start_positions,
                    'goal_positions': self.goal_positions,
                })

                plot_mapf_problem(self.ax['A'], info)

                plt.pause(0.001)


def main():
    n_agents = 10
    path_to_maps = '../maps'
    # map_dir = 'random-32-32-10.map'
    map_dir = 'warehouse-10-20-10-2-1.map'

    to_render = True
    # to_render = False

    env = EnvClassicalMAPF(map_dir, path_to_maps=path_to_maps, n_agents=n_agents, to_render=to_render, plot_every=1)
    env.create_new_problem()
    env.render({'i_step': 1})
    plt.show()


if __name__ == '__main__':
    main()
