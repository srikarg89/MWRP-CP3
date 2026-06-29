# Overview

This repository implements the methods described in our research work Scalable Methods with [Provable Optimality Bounds for the Multiple Watchman Route Problem](https://doi.org/10.1609/icaps.v36i1.42818). If you use our work in your research, please do cite it.

# Usage

## Building

First create a build folder and cd into it:

```
mkdir build/
cd build/
```

Then, build the CMake configuration, followed by the Make configuration

```
cmake ..
cmake --build .
```

This will generate a `run` file that you can execute.

## Runing scenarios

To run a scenario, there are two main files, the **config** file and the **solver.json** file.
- The **config** file describes the map that should be used (located in *maps/*), as well as the agent starting positions, agent movement function, and agent vision function. Some example configs and maps are provided.
- The **solver.json** file allows you to alter the solver properties for this run.
  - `heuristic` refers to the heuristic method used, and can be one of {BFS, Singleton, TSP, MAX, LAZY}.
  - `cell_pruning_method` refers to the state space reduction method used, and can be {NONE, CELL, PATH, CELL_THEN_PATH}
  - `prune_pivots` enables Pivot Pruning if set to true.
  - `run_parallel` enables Parallel Heuristic Computation if set to true, with the `parallel_batch_size` parameter used to regulate the number of heuristic calculations parallelized.
  - `run_decentralized_search` is used to enable postprocessing. If enabled, `num_decentralized_searches` is used to determine how many single-agent postprocessing searches should be run.
  - From here on out, the `centralized` parameters refer to the joint-space initial search, and the `decentralized` parameters refer to the single-agent postprocessing searches.
    - `*_search_time_limit` can be used to set a time limit on the anytime behavior of the search.
    - `*_hard_search_time_limit` can be used to set a hard cutoff time limit on the search, allowing it to fail.
    - `*_astar_weight` is used as the weight for `MxWA*` (or `WA*` for single-agent searches), using 1.0 will run regular A*.
    - `*_focal_epsilon` is used as the focal epislon for the corresponding search, and using 1.0 will run regular A*. If focal search is enabled, `focal_method` is used as the focal method, and should be one of {SOC, MOC}.

Once the **solver.json** file is configured, a scenario instance can be run from the *build/* folder using

```
./run <config_filepath> ../solver.json
```

For example, to get started you can run

```
./run ../configs/maze/11_by_11_multi.json ../solver.json
```

## Visualization

To generate a visualization of the solution, after running the instance, you can switch to the *scripts/* directory and run

```
python replay_final.py <animation_name> <animation_fps>
```

I recommend using 10 or 20 for my fps.