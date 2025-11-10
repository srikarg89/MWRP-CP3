**Improvements / Changes to Remember**
- **Major**
  - Cell / Path dominance: Ignore exploration squares that are strictly easier to visit.
  - Focal search.
    - Weighted SOC focal heuristic.
    - Anytime focal search
      - Lazy removal of nodes using singleton heuristic.
  - Heirarchical search
    - Cell partitioning
    - Task partitioning
    - Multi-robot task partitioning
  - Multi robot tasks
    - Force multiple robots to reach task in MTSP heuristic.
    - Setup MTSP heuristic so that waiting robots have infinite cost to other pivots.
- **Minor**
  - Change to expanding borders function to avoid neighbor explosion with neighbors that can be reached later on.
  - Pivot pruning with shortcut logic.
  - Used Pathmax to ensure heuristic consistency.
  - Parallelization of node expansion.
  - Agent waiting state / Dominance needs to check for agent waiting.

**Proofs to figure out**
- That cell / path domination does still lead to optimal solution.
- That mTSP heuristic w/ task pivots is still an admissible heuristic.
- That using previous solution is optimal when previous solution was run on optimal search and f-value is the min f value.
  - I actually think this is false rip.

