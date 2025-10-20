**Things to fix / test**
- [ ] Test old vs new pivot sorting order (new could work better with multi-agent).
- [ ] Duplicate / Dominance Detection.
  - [ ] Also need to fix current duplicate checking (doesn't check agent order).
- [ ] Provide vision radius **R** to the LOS functions.
- [ ] Try the method of adding every single unseen cell as a pivot, and then iteratively pruning using the prune_graph function.

**Extensions**
- [ ] Tasks with absolute time limits (https://www.sciencedirect.com/science/article/pii/S1572528610000289). (need to implement waiting at tasks??).
  - [ ] Tasks with inter-dependent time constraints (need to implement waiting at tasks??).
- [ ] Multi-robot tasks. (https://www.sciencedirect.com/science/article/pii/S0377221723000735) Didn't fully read this.
- [ ] Tasks that take a certain amount of time to complete.
- [ ] Search-based collision handling.
- [ ] Priority-based collision handling.
- [ ] Extend algorithm to work on weighted graph (different traversabilities).
- [ ] Rolling horizon variation: Only consider squares within X distance to you, but do consider all tasks?
  - [ ] Alternatively, maybe only consider squares that you are the closest robot to (within some limit of # of squares)?
- [ ] Tasks with precedence constraints (need to implement waiting at tasks??). (https://www.sciencedirect.com/science/article/pii/S0377221723000735) Didn't fully read this.

**Optimizations**
- [ ] Don't need to do the disjoint loop every time to find the biggest shortcut left. We can just find all the shortcuts once and then just go through and delete them in reverse order.
- [ ] What if the expansion only expands one agent at a time (they take turns, or maybe just the agent with a lower makespan). Also, this can be further improved by choosing to expand the node with the lower time each time. Results in a **deeper** but **less wide** tree (could result in less node generation??).
- [ ] Optimizations on future searches by reusing previous search / expanded nodes.
- [ ] Group together squares (i.e. consider 3x3 area as one). That is, perform a heirarchical search (first over long distances), and then figure out the details of traversing each square afterwards.
- [ ] Ignore squares that are gonna be explored anyways by doing tasks (i.e. any square within LOS of a known task).
- [ ] Look into heuristics for the TSP problem instead of actually solving it.
- [ ] Unseen set instead of the full dynamic bitset.

 **Verified Working**
- [x] Implement neighbor function for multi-agent.
- [x] Implement expanding borders function.
- [x] Fix LOS in places where I assume its reversible, since Bresham LOS is not reversible.
- [x] Implement makespan cost.
- [x] Implement Singleton function for multi-agent.
- [x] Implement TSP function for multi-agent.
- [x] Implement max of singleton and TSP function for multi-agent.
- [x] Implement lazy A* for multi agent.
- [x] Handle collisions naively.
- [x] Decrease neighbor expansion if one neighbor can be used to get to another neighbor without worsening cost.
- [x] Add arrows to solution visualization to see intended paths (and how they change upon a task being discovered).
- [x] Fix bug with CPLEX memory management.
- [x] Use Release instead of Debug
- [x] Ignore exploration squares that are strictly easier to visit. i.e. every watcher of square A is also a watcher of square B, thus I don't need to worry about watching square B.
- [x] For disjoint graph pruning, just do O(N^3) loop to check for shortcuts instead of doing fancy BFS logic.
- [x] Make heuristic consistent.

**Improvements / Changes to Remember**
- Change to expanding borders function to avoid neighbor explosion with neighbors that can be reached later on.
- Pivot pruning with shortcut logic.
- Ignore exploration squares that are strictly easier to visit.
- Used Pathmax to ensure heuristic consistency.

**New Features Added**
- Much larger maps
- Tasks

**Time Breakdown (32x32 map, no tasks):** Total 2.7 seconds.
- Lookup precompute: 0.1 seconds
- Getting neighbors: 0.01 seconds
- Building disjoint graph: 0.2 seconds
- Running MTSP heuristic: 2.35 seconds
- Other operations: 0 seconds
- Nodes expanded: 111
- Nodes generated: 1018
- Generations skipped: 1009

**When altering task complexity:**
- Change expansion behavior.
- Change task completion check (in Environment).
- Change task completion check (in search neighbor creation).
- Change heuristic.
- Check if task is still possible (exit early).
