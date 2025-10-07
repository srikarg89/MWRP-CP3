- [ ] Test old vs new pivot sorting order (new could work better with multi-agent).
- [ ] Duplicate / Dominance Detection.
- [ ] Provide vision radius **R** to the LOS functions.
- [ ] Add arrows to solution visualization to see intended paths (and how they change upon a task being discovered).

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
- [x] Decrease neighbor expansion if one neighbor can be used to get to another neighbor without worsening cost.


**Optimizations**
- [ ] For disjoint graph pruning, just do O(N^3) loop to check for shortcuts instead of doing fancy BFS logic.
  - [ ] Also don't need to do that loop every time to find the biggest shortcut left. We can just find all the shortcuts once and then just go through and delete them in reverse order.
- [ ] What if the expansion only expands one node at a time (they take turns). Also, this can be further improved by choosing to expand the node with the lower time each time. Results in a **deeper** but **less wide** tree (could result in less node generation??).
- [ ] Optimizations on iterative search by reusing previous search / expanded nodes.
- [ ] Ignore dominated squares. i.e. every watcher of square A is also a watcher of square B, thus I don't need to worry about watching square B.
  - [ ] Can also ignore squares dominated by tasks (i.e. any square within LOS of a known task).
- [ ] Group together squares (i.e. consider 3x3 area as one). That is, perform a heirarchical search (first over long distances), and then figure out the details of traversing each square afterwards.