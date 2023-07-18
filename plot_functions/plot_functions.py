import numpy as np

from globals import *


font = {
    # 'family': 'serif',
    # 'color':  'darkred',
    'weight': 'normal',
    'size': 22,
}


def plot_mapf_problem(ax: matplotlib.axes.Axes, info):
    ax.cla()
    info = AttributeDict(info)
    ax.imshow(info.map_np.T, cmap='gray', origin='lower')

    # add start_positions
    s_x, s_y = [], []
    for node in info.start_positions:
        s_x.append(node.x)
        s_y.append(node.y)
    ax.scatter(s_x, s_y, marker='o', c='green')
    # add goal_positions
    g_x, g_y = [], []
    for node in info.goal_positions:
        g_x.append(node.x)
        g_y.append(node.y)
    ax.scatter(g_x, g_y, marker='x', c='red')

    if 'one_agent_path' in info:
        one_agent_path = info.one_agent_path
        x_list, y_list = [], []
        for node in one_agent_path:
            x_list.append(node.x)
            y_list.append(node.y)
        ax.plot(x_list, y_list, c='yellow')

    ax.set_title(f'{info.env_name}')

    # ax.set_xlim(0, info.max_steps)
    # ax.set_xlabel('Remained Coverage Req.', fontdict=font)


def plot_aom(ax, info):
    # Amount of messages

    ax.cla()
    info = AttributeDict(info)

    ax.plot(info.aom)
    ax.set_xlim(0, info.max_steps)
    ax.set_title('Amount of messages')


