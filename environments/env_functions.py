from globals import *


class EnvNode:
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


def get_dims_from_pic(map_dir, path='maps'):
    with open(f'{path}/{map_dir}') as f:
        lines = f.readlines()
        height = int(re.search(r'\d+', lines[1]).group())
        width = int(re.search(r'\d+', lines[2]).group())
    return height, width


def get_np_from_dot_map(map_dir, path='maps'):
    with open(f'{path}/{map_dir}') as f:
        lines = f.readlines()
        height, width = get_dims_from_pic(map_dir, path)
        img_np = np.ones((height, width))
        for height_index, line in enumerate(lines[4:]):
            for width_index, curr_str in enumerate(line):
                if curr_str == '.':
                    img_np[height_index, width_index] = 0
        return img_np, (height, width)


def distance_nodes(node1, node2):
    return np.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)


def set_nei(name_1, name_2, nodes_dict):
    if name_1 in nodes_dict and name_2 in nodes_dict and name_1 != name_2:
        node1 = nodes_dict[name_1]
        node2 = nodes_dict[name_2]
        dist = distance_nodes(node1, node2)
        if dist == 1:
            node1.neighbours.append(node2.xy_name)
            node2.neighbours.append(node1.xy_name)


def make_self_neighbour(nodes):
    for node_1 in nodes:
        node_1.neighbours.append(node_1.xy_name)


def build_graph_from_np(img_np, show_map=False):
    # 0 - wall, 1 - free space
    nodes = []
    nodes_dict = {}

    x_size, y_size = img_np.shape
    # CREATE NODES
    for i_x in range(x_size):
        for i_y in range(y_size):
            if img_np[i_x, i_y] == 0:
                node = EnvNode(i_x, i_y)
                # node = EnvNode(i_y, i_x)
                nodes.append(node)
                nodes_dict[node.xy_name] = node

    # CREATE NEIGHBOURS
    name_1, name_2 = '', ''
    for i_x in range(x_size):
        for i_y in range(y_size):
            name_2 = f'{i_x}_{i_y}'
            set_nei(name_1, name_2, nodes_dict)
            name_1 = name_2
    # print('finished rows')

    for i_y in range(y_size):
        for i_x in range(x_size):
            name_2 = f'{i_x}_{i_y}'
            set_nei(name_1, name_2, nodes_dict)
            name_1 = name_2
    make_self_neighbour(nodes)
    # print('finished columns')

    if show_map:
        plt.imshow(img_np, cmap='gray', origin='lower')
        plt.show()
        # plt.pause(1)
        # plt.close()

    logging.debug('finished creating nodes')
    return nodes, nodes_dict


def create_nodes_from_map(map_dir, path='maps', show_map=False):
    # height - x, width - y
    map_np, (height, width) = get_np_from_dot_map(map_dir, path=path)
    nodes, nodes_dict = build_graph_from_np(map_np, show_map=show_map)
    return map_np, (height, width), nodes, nodes_dict


def main():
    # map_dir = 'random-32-32-10.map'
    map_dir = 'warehouse-10-20-10-2-1.map'
    show_map = True
    # show_map = False
    map_np, (height, width), nodes, nodes_dict = create_nodes_from_map(map_dir, path='../maps', show_map=show_map)
    print(len(nodes))


if __name__ == '__main__':
    main()

