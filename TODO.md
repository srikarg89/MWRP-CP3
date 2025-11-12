**Things to fix / potential optimizations**
- [ ] Test old vs new pivot sorting order (new could work better with multi-agent).
- [ ] Don't need to do the disjoint loop every time to find the biggest shortcut left. We can just find all the shortcuts once and then just go through and delete them in reverse order.
- [ ] After you have a maximum cost bound, you can run path dominance every iteration while pruning out paths that would be above the cost bound.

**Extensions**
- Definitely should do
  - [ ] Tasks that take a certain amount of time to complete.
  - [ ] Provide vision radius **R** to the LOS functions.
- Would be dope extensions
  - [ ] Extend algorithm to continuous space.
- Still dope but probably won't get to
  - [ ] Change problem such that you have probabilities of where tasks are, as opposed to the normal free-space assumption.
- Less important
  - [ ] Search-based collision handling.
  - [ ] Priority-based collision handling.
  - [ ] Rolling horizon variation: Only consider squares within X distance to you, but do consider all tasks?
    - [ ] Alternatively, maybe only consider squares that you are the closest robot to (within some limit of # of squares)?
  - [ ] Tasks with inter-dependent time constraints (need to implement waiting at tasks??).
  - [ ] Group together squares (i.e. consider 3x3 area as one). That is, perform a heirarchical search (first over long distances), and then figure out the details of traversing each square afterwards.

**Docket**
- [ ] Iterative search solution check dominance in both ways (either for suboptimal solution or for admissible heuristic)
- [ ] Iterative search utilizing previous expansions as a admissible heuristic.
- [ ] Cases for iterative searching??

- [ ] Greedy centralized TSP-based search to scale for many agents??
- [ ] Hash graph to reuse TSP calculations??

- [ ] Parallelize decentralized single-agent search?? Should decide this after running experiments with many agents.
- [ ] Iterative search for multi-agent??

**Paper Docket**
- [ ] More experiments on similarly large maps.
  - [ ] Create maps with randomized task layouts.
- [ ] Experiments with 5 robots (more if fast).
- [ ] Implement frontier-based search baseline.
- [ ] Implement frontier-based search + greedy task baseline.
- [ ] Implement our search + greedy task baseline.
- [ ] Experiments on suboptimal methods for centralized search with large number of agents.
  - [ ] Focal search with SOC.
  - [ ] Focal search with MOC.
  - [ ] Weighted A* w/ weights on graph.
  - [ ] Greedy TSP-based solution.
- [ ] Experiments testing different focal epsilons and weight values.
  - [ ] Record optimal solution length, runtime, sum of costs, etc.

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
- [x] Parallelization for heuristic calculation during search expansion (2x speedup).
- [x] Ignore squares that are gonna be explored anyways by doing tasks (i.e. any square within LOS of a known task).
- [x] Implement tasks with deadlines.
- [x] Implement tasks that require multiple robots to be there at the same time.
- [x] What if the expansion only expands one agent at a time (they take turns, or maybe just the agent with a lower makespan). Also, this can be further improved by choosing to expand the node with the lower time each time. Results in a **deeper** but **less wide** tree (could result in less node generation??).
  - For two agents: Better for single-thread (less nodes expanded), worse for multi-thread (since expansions are parallelized).
- [x] Fibonacci heap
- [x] Focal search
  - [x] SOC focal heuristic.
  - [x] Num seen focal heuristic.
  - [x] Weighted A* for focal heuristic.
- [x] Node dominance check.
- [x] Cleanup
  - [x] Delete task deadlines.
  - [x] Delete collision resolution.
  - [x] Pass in epsilon as input into solver config.
- [x] Anytime focal search
  - [x] Lazy removal of nodes using singleton heuristic.
- [x] Use focal search to determine "partition", then run single-agent search on each partition (centralized -> decentralized = heirarchical search framework).
- [x] Heirarchical task partition.
- [x] Heirarchical multi-robot task partition.
  - [x] Set it up like a "deadline" during the individual, decentralized search.
- [x] Speedup lookup precomputation time.
- [x] Max of costs focal heuristic (as opposed to sum of costs).
- [x] Download larger maps

**Tested but Worse**
- [x] Adding in time windows naively into the MTSP formulation.
- [x] Try the method of adding every single unseen cell as a pivot, and then iteratively pruning using the prune_graph function.
- [x] Unseen set instead of the full dynamic bitset.
- [x] Num seen heuristic.

**Time Breakdown (big_maze_tight.json, no tasks, TSP heuristic):** Total 18 seconds.
- Lookup precompute: 0.1 seconds
- Neighbor expansion (expanding borders) time: 0.171 seconds
- Heuristic time: 16.9 seconds
- Other operations: 1 second
- Nodes expanded: 713
- Nodes generated: 4802
- Generations skipped: 5183

**When altering task complexity:**
- Change expansion behavior.
- Change task completion check (in Environment).
- Change task completion check (in search neighbor creation).
- Change heuristic.
- Check if task is still possible (exit early).
