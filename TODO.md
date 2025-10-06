- [ ] Test old vs new pivot sorting order (new could work better with multi-agent).
- [ ] Duplicate / Dominance Detection.
- [ ] Provide vision radius **R** to the LOS functions.

**Extensions**
- [ ] Tasks with absolute time limits.
- [ ] Tasks with precedence constraints (need to implement waiting at tasks??).
- [ ] Tasks with inter-dependent time constraints (need to implement waiting at tasks??).
- [ ] Search-based collision handling.
- [ ] Priority-based collision handling.
- [ ] Extend algorithm to work on weighted graph
- [ ] Rolling horizon variation: Only consider squares within X distance to you, but do consider all tasks?


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


**Optimizations**
- [ ] For disjoint graph pruning, just do O(N^3) loop to check for shortcuts instead of doing fancy BFS logic.
  - [ ] Also don't need to do that loop every time to find the biggest shortcut left. We can just find all the shortcuts once and then just go through and delete them in reverse order.
- [ ] What if the expansion only expands one node at a time (they take turns). Also, this can be further improved by choosing to expand the node with the lower time each time. Results in a **deeper** but **less wide** tree (could result in less node generation??).
