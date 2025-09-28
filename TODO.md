- [ ] Implement TSP function for multi-agent.
- [ ] Implement max of singleton and TSP function for multi-agent.
- [ ] Implement lazy A* for multi agent.
- [ ] Test old vs new pivot sorting order (new could work better with multi-agent).
- [ ] Duplicate / Dominance Detection.
- [ ] Provide vision radius **R** to the LOS functions.
- [x] Handle collisions naively.


**Extensions**
- [ ] Extend algorithm to work on weighted graph


**Verified Working**
- [x] Implement neighbor function for multi-agent.
- [x] Implement expanding borders function.
- [x] Fix LOS in places where I assume its reversible, since Bresham LOS is not reversible.
- [x] Implement makespan cost.
- [x] Implement Singleton function for multi-agent.


**Optimizations**
- [ ] For disjoint graph pruning, just do O(N^3) loop to check for shortcuts instead of doing fancy BFS logic.
- [ ] Also don't need to do that loop every time to find the biggest shortcut left. We can just find all the shortcuts once and then just go through and delete them in reverse order.
