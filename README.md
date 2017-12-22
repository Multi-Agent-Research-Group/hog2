# hog2 - MAPF extensions

HOG2 is a package containing combinatorial search algorithms forked from [Nathan Sturtevant's repository](https://github.com/nathansttt/hog2)

This fork extends HOG2 with multi-agent pathfinding algorithms

## Multi-Agent Pathfinding (MAPF) Definition

The muli-agent planning problem is defined as a set of **k** agents **A={a<sub>1</sub>,…,a<sub>k</sub>}**,
a graph **G=(V,E)** of the state space where **V** is set of all possible states that an agent may take,
for example, a state could be latitude/longitude coordinates, or x,y,z location, etc.
**E** is the set of edges between states representing actions that each agent may take to transition between states.
Each agent has an initial state **{s<sub>1</sub>,…,s<sub>k</sub>}** and a goal state **{g<sub>1</sub>,…,g<sub>k</sub>}**.
The objective is to find a set of paths **P={p<sub>1</sub>,…,p<sub>k</sub>}** for each agent from it’s start to it’s goal.
Each path **p<sub>i</sub>** is a sequence of states such that **p<sub>i</sub>[0]=s<sub>i</sub>** and **p<sub>i</sub>[end]=g<sub>i</sub>**
and all states in each **p<sub>i</sub>** are adjacent, meaning they are connected by an edge in **E**.

In addition, we require that there are no conflicts in **P**. That is, no pair of path points **p<sub>i</sub>[n],p<sub>j</sub>[m]** may occupy
the same state at the same time. (In other words the agents cannot collide into each other.)
We also seek solutions which are optimal, meaning each p<sub>i</sub> is as short as possible
-- it minimizes the distance (or time or fuel, etc.) required to maneuver from start to goal.
Optimality can be measured with respect to **makespan** (minimization of the largest cost to completion for all agents),
or **flowspan** (sum of costs for all agents). The algorithms herein are targeted toward flowspan but are
usable for makespan with minimal changes.

## Dependencies

C++11 or newer

Google Test

OpenGL (optional -- code can be compiled with OpenGL turned off using **OPENGL=STUB**)

## Building
```cd build/gmake && make -j4```

## Running

### Algorithm implementations

* MAAStar - Multi-agent A*
* AACBS - Any-Angle Conflict-Based-Search
* MO - Multi-Objective Conflict-Based-Search
* MAPF - Conflict-Based-Search
* icts - Increasing Cost Tree Search
* MACBS - Meta-Agent Conflict-Based-Search with ICTS as the meta-agent solver

### Test instances
#### There are 2 ways to set up a map:
1. Using map dimensions
  The `-dimensions` flag must be used followed by width, depth and height. For 2D maps height defaults to 1.
2. Using a map file
  The `-mapfile` flag must be followed by the path to a map relative to the root directory of hog2. See [this](https://github.com/thaynewalker/hog2/blob/conf-avoidance/test/environments/instances/8x8/10/0.map) example.
  
#### There are 3 ways to set up agent waypoints:
1. With an environment file and a problem file
  The environment file indicates the environment to use for sets of agents. See [https://github.com/thaynewalker/hog2/blob/conf-avoidance/apps/MAPF/Driver.cpp#L612](https://github.com/thaynewalker/hog2/blob/conf-avoidance/apps/MAPF/Driver.cpp#L612)
  The problem file indicates space-delimited waypoints in x,y,z coordinates, one agent per line. See [here](https://raw.githubusercontent.com/thaynewalker/hog2/conf-avoidance/test/environments/instances/8x8/10/0.csv) for an example.
2. With a configuration file
  The configuration file indicates the environment and waypoints for agents individually.
3. With a scenario file
  The scenario file is always linked to a map and indicates the map name and a single start/goal per agent. See [here](https://github.com/thaynewalker/hog2/blob/conf-avoidance/test/environments/instances/8x8/10/0.map.scen).
  
### Examples
#### Unobstructed map with gui soved with CBS
```
cd build/gmake && make -j4 release && ../../bin/release/MAPF -probfile ../../test/environments/instances/8x8/10/0.csv -envfile env9.csv -dimensions 8,8,1 -radius .25 
```
#### Map with obstacles icts
```
cd build/gmake && make -j4 release && ../../bin/release/icts -nagents 10 -scenfile ../../test/environments/instances/8x8/10/2.map.scen -mode b -agentType 9 -radius .25
```
