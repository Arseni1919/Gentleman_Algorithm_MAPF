"""
DONE:

TODO:
- build environment
- build Gentleman Algorithm (GA)
- build metrics: SoC, makespan, time
- build PP
- compare to PP
- plot graphs
- save results
- prepare presentation
- compare to PIBT and others

Agent Q starts to plan
Agents that have conflict with the agent Q are propagating the priorities from bigger to smaller from closer to further
With the priorities they plan a way aside “to wait” for others to pass. These paths are out of other higher priority paths.
Then if succeeded they send the “success” message to upper priorities
If the emerged line of priorities didn’t work out - the agents pick another combination of the priorities unit they find the working one
Then agent Q calculates its path according to the aside paths of others and sends to all of them
Then others calculate their path back to target location according to the previously decided priorities
Everybody is on the spot
"""