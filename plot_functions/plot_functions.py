import numpy as np

from globals import *


font = {
    # 'family': 'serif',
    # 'color':  'darkred',
    'weight': 'normal',
    'size': 22,
}


def plot_ga_plans(ax: matplotlib.axes.Axes, info: dict):
    ax.cla()
    info = AttributeDict(info)
    plans = info.ga_plans
    pause_time = info.pause_time if 'pause_time' in info else 1
    max_duration = max([len(plan) for _, plan in plans.items()])

    for t in range(max_duration):
        ax.cla()
        # show map
        ax.imshow(info.map_np.T, cmap='gray', origin='lower')
        # show start_positions
        s_x, s_y = [], []
        for node in info.start_positions:
            s_x.append(node.x)
            s_y.append(node.y)
        ax.scatter(s_x, s_y, marker='o', c='green')
        # show goal_positions
        g_x, g_y = [], []
        for node in info.goal_positions:
            g_x.append(node.x)
            g_y.append(node.y)
        ax.scatter(g_x, g_y, marker='x', c='red')
        # show agents
        a_x, a_y = [], []
        for agent_name, plan in plans.items():
            # show path
            p_x, p_y = [], []
            for node in plan:
                p_x.append(node.x)
                p_y.append(node.y)
            ax.plot(p_x, p_y, c='gray', alpha=0.5)
            if t < len(plan):
                last_node = plan[t]
            else:
                last_node = plan[-1]
            a_x.append(last_node.x)
            a_y.append(last_node.y)
            ax.text(last_node.x, last_node.y, f'{agent_name}', color='orange', size=7)
        ax.scatter(a_x, a_y, marker='*', c='yellow')
        ax.set_title(f"Gentleman's Plans, TIME: {t}")
        plt.pause(pause_time)
        # plt.pause(0.0001)

    ax.set_title("Gentleman's Plans")


def plot_mapf_problem(ax: matplotlib.axes.Axes, info):
    ax.cla()
    info = AttributeDict(info)

    # show map
    ax.imshow(info.map_np.T, cmap='gray', origin='lower')

    # show start_positions
    s_x, s_y = [], []
    for node in info.start_positions:
        s_x.append(node.x)
        s_y.append(node.y)
    ax.scatter(s_x, s_y, marker='o', c='green')
    # show goal_positions
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


def plot_something(ax: matplotlib.axes.Axes, info: dict):
    ax.cla()
    info = AttributeDict(info)

    ax.plot([1, 2, 3])

    ax.set_xlim(0, info.max_steps)
    ax.set_xlabel('x label', fontdict=font)
    ax.set_title('title')


