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
make
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
