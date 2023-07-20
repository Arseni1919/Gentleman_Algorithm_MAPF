"""
DONE:
- build environment

TODO:
- build Gentleman Algorithm (GA)
    - build alg for one agent
    - build function for multiple agents
    - check for v, e and perm conflicts
- build metrics: SoC, makespan, time
- build PP
- compare to PP
- plot graphs
- save results
- prepare presentation
- compare to PIBT and others

For every next agent X:
    If collided:
        find path aside for X outside of all the previous (higher priority) paths -
        the aside path does not allowed to cross the starting nodes of all the higher priority agents

        Calculate plans for yourself and for the path aside for all the above
        starting with the lowest (agent X) to highest (...4, 3, 2) priority if you have an aside path.
        If you do not have an aside path - you skip this step

        First agent calculates its path to the goal node

        All agents calculate their paths to goal nodes from their aside locations (if they have one) or start locations,
        starting from 2, 3, 4 … until the agent X

"""