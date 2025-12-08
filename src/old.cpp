// // Old task stuff
//         for(Task t : node.tasks_left){
//             int task_map_idx = map.get_map_idx(t.pos);
//             std::unordered_map<int, int> num_reached_by_time;
//             bool reachable = false;
//             bool completed = false;
//             for(const AgentState& agent : nbr){
//                 int agent_map_idx = map.get_map_idx(agent.pos);
//                 if(agent_map_idx == task_map_idx && t.release_time <= agent.cost && agent.cost <= t.deadline){
//                     num_reached_by_time[agent.cost] += 1;
//                     if(num_reached_by_time[agent.cost] >= t.num_agents_required){
//                         completed = true;
//                         break;
//                     }
//                 }
//             }
//             if(completed) {
//                 // Free up agents that were waiting at this task.
//                 for(AgentState& agent : nbr){
//                     if(agent.waiting_idx == t.id){
//                         agent.waiting_idx = -1;
//                     }
//                 }
//             }
//             else {
//                 nbr_tasks_left.push_back(t);
//             }
//         }

//         // Check if we have a deadlock. That is, check if there is at least one task that is reachable by at least one non-terminated agent.
//         bool task_deadlocked = false;
//         bool task_failed = false;
//         for(const Task& t : nbr_tasks_left){
//             int agents_available = 0;
//             bool can_reach = false;
//             for(const AgentState& agent : nbr){
//                 if(lookup.apsp[map.get_map_idx(agent.pos)][t.map_idx] + agent.cost <= t.deadline){
//                     can_reach = true;
//                 }
//                 if(agent.waiting_idx == t.id || (!agent.terminated && agent.waiting_idx == -1)){
//                     agents_available += 1;
//                     continue;
//                 }
//             }
//             if(agents_available < t.num_agents_required){
//                 task_deadlocked = true;
//                 break;
//             }
//             if(!can_reach){
//                 task_failed = true;
//                 break;
//             }
//         }

//         if(task_failed) {
//             // Don't generate neighbors that have failed tasks.
//             continue;
//         }

//         if(task_deadlocked) {
//             // Don't generate neighbors that have deadlocked tasks.
//             // printf("Deadlocked task detected, skipping neighbor generation.\n");
//             METRICS.num_skipped_task_deadlock += 1;
//             continue;
//         }