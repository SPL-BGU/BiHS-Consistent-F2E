# Benchmark Instances

This repository contains benchmark instances for heuristic search in the following four domains:

- Pancake
- Tower of Hanoi (ToH)
- STP (100 Korf's instances)
- Grid
- Road

Each domain has its own instance format and interpretation. Use the sections below to describe how to interpret the instances for each domain.

---

## Domain Details

### Pancake
```
0:     2 9 13 6 3 4 10 8 5 11 7 1 12 0
```
This for example represents instance 0, where the stack has 14 pancakes, where 13 is the largest and 0 is the smallest.
The goal is to sort them from 0 to 13.

### Tower of Hanoi (ToH)
*Describe the format and interpretation of ToH instances here.*
```
0:     (0) 4 1 (1) 11 10 7 (2) 8 3 2 (3) 12 9 6 5
```
This for example represents instance 0. There are four pegs labeled (0) to (3). The disks are on the peg to their left
with the largest disk being 12 and the smallest 0. The goal is to put all disks on peg (3).

### Grid
All maps are DAO maps taken from [this benchmark](https://movingai.com/benchmarks/dao/index.html).
The files used can be found in both conference and journal under maps/dao (for map files) and scenarios/dao for the instance coordinates.

```
dao/arena.map:  (1, 13)-(5, 3)
```
This for example suggests that the instance uses the arena map, the start coordinates (left) are (1, 13) and the goal coordinates (right) are (5, 3).

### Road

We only use COL map taken from the [9th DIMACS Implementation Challenge](https://www.diag.uniroma1.it/~challenge9/download.shtml).
The files used can be found in both conference and journal under maps/roadmaps.

```
(-0.158974,-0.142114)-(-0.874619,-0.0197956)
```
Similar to Grid, the start coordinates (left) are (-0.158974,-0.142114) and the goal coordinates (right) are (-0.874619,-0.0197956).
