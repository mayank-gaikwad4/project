#pragma once

constexpr u32 MaxAgents         = 100;
constexpr u32 DefaultAgentCount = 3000;
constexpr u32 MaxNodes          = 4096;
constexpr u32 MaxNodeDegree     = 4;
constexpr u32 MaxPathLength     = 64;
constexpr u32 TargetNodeCount   = 80;
constexpr u32 HubCount          = 1;
constexpr u32 InvalidNode       = 0xFFFFFFFFu;

constexpr f32 MapCenter         = 4000.0f;
constexpr f32 CellSize          = 32.0f;
constexpr f32 InverseCellSize   = 1.0f / CellSize;
constexpr i32 GridDimension     = 256;
constexpr u32 GridCellCount     = (u32)(GridDimension * GridDimension);

constexpr u64 SimBaseTickNs     = 1000000000ull / 60;
constexpr f32 SimDeltaSeconds   = 1.0f / 60.0f;

constexpr f32 NominalSpeed          = 90.0f;
constexpr f32 LaneOffset            = 3.5f;
constexpr f32 MaxLaneDeviation      = 2.0f;
constexpr f32 ArrivalRadius         = 10.0f;
constexpr f32 NeighborRadius        = 40.0f;
constexpr f32 CarFollowRadius       = 30.0f;
constexpr f32 CarFollowGap          = 9.0f;
constexpr f32 SeparationRadius      = 8.0f;
constexpr i32 CongestionThreshold   = 4;
constexpr f32 MaxTurnRateTurns      = RadiansToTurns(10.0f);
constexpr f32 JitterTurns           = RadiansToTurns(0.3f);

enum AgentState : u8 { AgentState_Waiting = 0, AgentState_Idle, AgentState_Driving };

class Camera { public: f32 X, Y, Zoom; };

class Graph {
public:
    u32 node_count; u32 edge_count;
    f32* node_x; f32* node_y;
    u8* is_hub; u8* is_delivery;
    u32* edge_offset; u32* edge_dest; f32* edge_weight; u8* edge_class;
};

class Agent {
public:
    Vec2 position; Vec2 velocity;
    f32 heading; f32 congestion_factor; f32 wait_timer;
    u32 agent_id; u32 cur_wp; u32 path_len;
    u64 order_id; u64 assigned_tick;
    u8 state; u8 congested;
    u8 start_r, start_g, start_b;
    u8 target_r, target_g, target_b;
};

class AgentRenderState {
public:
    Vec2 position; f32 heading; f32 congestion_factor;
    u8 start_r, start_g, start_b;
    u8 target_r, target_g, target_b;
    u8 waiting; u8 congested;
};

class HeapItem { public: f32 priority; f32 cost; u32 node; };
class TempEdge { public: u32 dest; f32 weight; u8 road_class; };
class TempNode { public: Vec2 position; u8 is_hub, is_delivery, degree; TempEdge edges[MaxNodeDegree]; };

class SimWorld {
public:
    Arena* arena;
    Graph graph;
    Config config;
    u64 seed; u64 rng_state; u64 tick; u64 next_order_id;
    u64 total_deliveries; u64 total_delivery_ticks;

    u32 agent_count;
    Agent* agents_current; Agent* agents_sorted; u32* agent_paths;
    AgentRenderState* render_previous; AgentRenderState* render_current;
    f32* display_congestion;

    i32* active_deliveries;
    u8* node_r; u8* node_g; u8* node_b;
    u8* node_base_r; u8* node_base_g; u8* node_base_b;

    u32* axis_spread; u32* cell_key_of_agent; u32* cell_end;

    f32* path_g_score; u32* path_came_from; u32* path_stamp; u32* path_reverse;
    HeapItem* path_heap; u32 path_ticket; u32 dfs_counter;

    sqlite::Database database;
    EventLogger logger;
};

inline f32 Distance2D(f32 from_x, f32 from_y, f32 to_x, f32 to_y) {
    f32 delta_x = to_x - from_x; f32 delta_y = to_y - from_y;
    return sdl::sqrtf(delta_x * delta_x + delta_y * delta_y);
}

u32 FindPath(SimWorld* world, u32 start_node, u32 goal_node, u32* path_out) {
    Graph* graph = &world->graph;
    world->path_ticket++;
    u32 ticket = world->path_ticket;
    f32 goal_x = graph->node_x[goal_node];
    f32 goal_y = graph->node_y[goal_node];

    world->path_g_score[start_node]  = 0.0f;
    world->path_stamp[start_node]    = ticket;
    world->path_came_from[start_node] = InvalidNode;

    u32 heap_count = 1;
    world->path_heap[0].priority = Distance2D(graph->node_x[start_node], graph->node_y[start_node], goal_x, goal_y);
    world->path_heap[0].cost = 0.0f;
    world->path_heap[0].node = start_node;

    b32 found = 0;
    while (heap_count > 0) {
        HeapItem top = world->path_heap[0];
        heap_count--;
        if (heap_count > 0) {
            HeapItem last = world->path_heap[heap_count];
            u32 hole = 0;
            for (;;) {
                u32 child = hole * 2 + 1;
                if (child >= heap_count) break;
                if (child + 1 < heap_count && world->path_heap[child + 1].priority < world->path_heap[child].priority) child++;
                if (world->path_heap[child].priority >= last.priority) break;
                world->path_heap[hole] = world->path_heap[child];
                hole = child;
            }
            world->path_heap[hole] = last;
        }

        if (top.cost > world->path_g_score[top.node]) continue;
        if (top.node == goal_node) { found = 1; break; }

        for (u32 edge_index = graph->edge_offset[top.node]; edge_index < graph->edge_offset[top.node + 1]; ++edge_index) {
            u32 next_node = graph->edge_dest[edge_index];
            f32 new_cost = top.cost + graph->edge_weight[edge_index];
            if (world->path_stamp[next_node] != ticket || new_cost < world->path_g_score[next_node]) {
                world->path_stamp[next_node] = ticket;
                world->path_g_score[next_node] = new_cost;
                world->path_came_from[next_node] = top.node;

                world->dfs_counter++;
                f32 priority = 0.0f;
                if (world->config.pathfinding_mode == 0) priority = new_cost + Distance2D(graph->node_x[next_node], graph->node_y[next_node], goal_x, goal_y);
                else if (world->config.pathfinding_mode == 1) priority = new_cost;
                else priority = -(f32)world->dfs_counter;

                u32 hole = heap_count++;
                while (hole > 0) {
                    u32 parent = (hole - 1) / 2;
                    if (world->path_heap[parent].priority <= priority) break;
                    world->path_heap[hole] = world->path_heap[parent];
                    hole = parent;
                }
                world->path_heap[hole].priority = priority;
                world->path_heap[hole].cost = new_cost;
                world->path_heap[hole].node = next_node;
            }
        }
    }
    if (!found) return 0;

    u32 reverse_count = 0;
    for (u32 walk = goal_node; walk != InvalidNode && reverse_count < MaxNodes; walk = world->path_came_from[walk]) {
        world->path_reverse[reverse_count++] = walk;
    }
    u32 out_count = 0;
    for (i32 reverse_index = (i32)reverse_count - 1; reverse_index >= 0 && out_count < MaxPathLength; --reverse_index) {
        path_out[out_count++] = world->path_reverse[reverse_index];
    }
    return out_count;
}

void AssignDelivery(SimWorld* world, Agent* agent, u32 from_node) {
    Graph* graph = &world->graph;
    u32 target_node = from_node;
    i32 attempts = 0;
    do {
        target_node = (u32)sdl::rand_below(&world->rng_state, (i32)graph->node_count);
        attempts++;
    } while ((target_node == from_node || graph->is_delivery[target_node] == 0) && attempts < 1000);
    if (attempts >= 1000) {
        do { target_node = (u32)sdl::rand_below(&world->rng_state, (i32)graph->node_count); } while (target_node == from_node);
    }

    u32* path = world->agent_paths + (u64)agent->agent_id * MaxPathLength;
    agent->path_len = FindPath(world, from_node, target_node, path);
    agent->cur_wp = 1;

    agent->start_r = world->node_base_r[from_node]; agent->start_g = world->node_base_g[from_node]; agent->start_b = world->node_base_b[from_node];
    agent->target_r = world->node_base_r[target_node]; agent->target_g = world->node_base_g[target_node]; agent->target_b = world->node_base_b[target_node];

    if (agent->path_len > 1) {
        u32 final_node = path[agent->path_len - 1];
        world->active_deliveries[final_node]++;
        world->node_r[final_node] = agent->target_r; world->node_g[final_node] = agent->target_g; world->node_b[final_node] = agent->target_b;
        agent->order_id = world->next_order_id++;
        agent->assigned_tick = world->tick;
        world->logger.Notify(EventKind_OrderAssigned, world->tick, agent->agent_id, from_node, final_node, agent->path_len, agent->order_id);
    } else {
        agent->order_id = 0; agent->wait_timer = 0.5f; agent->state = AgentState_Waiting;
    }
}

b32 SimInit(SimWorld* world, Arena* permanent, u64 seed, u32 agent_count, const c8* database_path, u64 run_start_ns, i64 epoch_ns) {
    if (agent_count > MaxAgents) agent_count = MaxAgents;
    world->arena = permanent; world->seed = seed; world->rng_state = seed ? seed : 1;
    world->agent_count = agent_count; world->next_order_id = 1;
    world->total_deliveries = 0; world->total_delivery_ticks = 0;
    Graph* graph = &world->graph;

    Arena* scratch = ArenaAlloc();
    TempNode* temp_nodes = PushArray<TempNode>(scratch, TargetNodeCount);
    u32 node_count = 1;
    temp_nodes[0].position = Vec2(MapCenter, MapCenter);

    u32 attempts = 0;
    while (node_count < TargetNodeCount && attempts < TargetNodeCount * 10) {
        attempts++;
        u32 parent = (u32)sdl::rand_below(&world->rng_state, (i32)node_count);
        if (temp_nodes[parent].degree >= MaxNodeDegree) continue;

        f32 angle_turns = (f32)sdl::rand_below(&world->rng_state, 360) / 360.0f;
        if (temp_nodes[parent].degree > 0) {
            u32 existing = temp_nodes[parent].edges[0].dest;
            f32 base_turns = sdl::atan2_turns(temp_nodes[parent].position.Y - temp_nodes[existing].position.Y, temp_nodes[parent].position.X - temp_nodes[existing].position.X);
            i32 turn_choice = sdl::rand_below(&world->rng_state, 3);
            switch (turn_choice) {
                case 0:  angle_turns = base_turns; break;
                case 1:  angle_turns = base_turns + TurnQuarter; break;
                default: angle_turns = base_turns - TurnQuarter; break;
            }
            angle_turns += ((f32)sdl::rand_below(&world->rng_state, 100) / 100.0f - 0.5f) * JitterTurns;
        }

        f32 step_distance = 70.0f + (f32)sdl::rand_below(&world->rng_state, 80);
        Vec2 candidate = Vec2(temp_nodes[parent].position.X + sdl::cos_turns(angle_turns) * step_distance, temp_nodes[parent].position.Y + sdl::sin_turns(angle_turns) * step_distance);

        i32 snap_node = -1;
        f32 best_distance_squared = 65.0f * 65.0f;
        for (u32 other = 0; other < node_count; ++other) {
            if (other == parent) continue;
            f32 delta_x = temp_nodes[other].position.X - candidate.X;
            f32 delta_y = temp_nodes[other].position.Y - candidate.Y;
            f32 distance_squared = delta_x * delta_x + delta_y * delta_y;
            if (distance_squared < best_distance_squared) { best_distance_squared = distance_squared; snap_node = (i32)other; }
        }

        if (snap_node != -1) {
            if (temp_nodes[snap_node].degree < MaxNodeDegree) {
                b32 already_connected = 0;
                for (u32 edge_slot = 0; edge_slot < temp_nodes[parent].degree; ++edge_slot) {
                    if (temp_nodes[parent].edges[edge_slot].dest == (u32)snap_node) already_connected = 1;
                }
                if (!already_connected) {
                    f32 actual = Distance2D(temp_nodes[parent].position.X, temp_nodes[parent].position.Y, temp_nodes[snap_node].position.X, temp_nodes[snap_node].position.Y);
                    TempEdge* forward = &temp_nodes[parent].edges[temp_nodes[parent].degree++];
                    forward->dest = (u32)snap_node; forward->weight = actual; forward->road_class = 1;
                    TempEdge* backward = &temp_nodes[snap_node].edges[temp_nodes[snap_node].degree++];
                    backward->dest = parent; backward->weight = actual; backward->road_class = 1;
                }
            }
        } else {
            u32 new_node = node_count++;
            temp_nodes[new_node].position = candidate;
            temp_nodes[new_node].is_delivery = (sdl::rand_below(&world->rng_state, 100) < 20) ? 1 : 0;
            TempEdge* forward = &temp_nodes[parent].edges[temp_nodes[parent].degree++];
            forward->dest = new_node; forward->weight = step_distance; forward->road_class = 1;
            TempEdge* backward = &temp_nodes[new_node].edges[temp_nodes[new_node].degree++];
            backward->dest = parent; backward->weight = step_distance; backward->road_class = 1;
        }
    }

    f32* distance_to_hub = PushArray<f32>(scratch, node_count);
    u32 first_hub = (u32)sdl::rand_below(&world->rng_state, (i32)node_count);
    temp_nodes[first_hub].is_hub = 1;
    for (u32 node_index = 0; node_index < node_count; ++node_index) {
        f32 delta_x = temp_nodes[node_index].position.X - temp_nodes[first_hub].position.X;
        f32 delta_y = temp_nodes[node_index].position.Y - temp_nodes[first_hub].position.Y;
        distance_to_hub[node_index] = delta_x * delta_x + delta_y * delta_y;
    }
    for (u32 hub_number = 1; hub_number < HubCount; ++hub_number) {
        i32 best_node = -1; f32 best_distance = -1.0f;
        for (u32 node_index = 0; node_index < node_count; ++node_index) {
            if (temp_nodes[node_index].is_hub) continue;
            if (distance_to_hub[node_index] > best_distance) { best_distance = distance_to_hub[node_index]; best_node = (i32)node_index; }
        }
        if (best_node < 0) break;
        temp_nodes[best_node].is_hub = 1;
        for (u32 node_index = 0; node_index < node_count; ++node_index) {
            f32 delta_x = temp_nodes[node_index].position.X - temp_nodes[best_node].position.X;
            f32 delta_y = temp_nodes[node_index].position.Y - temp_nodes[best_node].position.Y;
            f32 distance_squared = delta_x * delta_x + delta_y * delta_y;
            if (distance_squared < distance_to_hub[node_index]) distance_to_hub[node_index] = distance_squared;
        }
    }
    for (u32 node_index = 0; node_index < node_count; ++node_index) {
        if (!temp_nodes[node_index].is_hub) continue;
        for (u32 edge_slot = 0; edge_slot < temp_nodes[node_index].degree; ++edge_slot) {
            TempEdge* edge = &temp_nodes[node_index].edges[edge_slot];
            edge->road_class = 2;
            TempNode* neighbor = &temp_nodes[edge->dest];
            for (u32 back_slot = 0; back_slot < neighbor->degree; ++back_slot) {
                if (neighbor->edges[back_slot].dest == node_index) neighbor->edges[back_slot].road_class = 2;
            }
        }
    }

    u32 total_edges = 0;
    for (u32 node_index = 0; node_index < node_count; ++node_index) total_edges += temp_nodes[node_index].degree;

    graph->node_count = node_count; graph->edge_count = total_edges;
    graph->node_x = PushArray<f32>(permanent, node_count); graph->node_y = PushArray<f32>(permanent, node_count);
    graph->is_hub = PushArray<u8>(permanent, node_count); graph->is_delivery = PushArray<u8>(permanent, node_count);
    graph->edge_offset = PushArray<u32>(permanent, node_count + 1); graph->edge_dest = PushArray<u32>(permanent, total_edges);
    graph->edge_weight = PushArray<f32>(permanent, total_edges); graph->edge_class = PushArray<u8>(permanent, total_edges);

    u32 edge_cursor = 0;
    for (u32 node_index = 0; node_index < node_count; ++node_index) {
        graph->node_x[node_index] = temp_nodes[node_index].position.X; graph->node_y[node_index] = temp_nodes[node_index].position.Y;
        graph->is_hub[node_index] = temp_nodes[node_index].is_hub; graph->is_delivery[node_index] = temp_nodes[node_index].is_delivery;
        graph->edge_offset[node_index] = edge_cursor;
        for (u32 edge_slot = 0; edge_slot < temp_nodes[node_index].degree; ++edge_slot) {
            graph->edge_dest[edge_cursor] = temp_nodes[node_index].edges[edge_slot].dest;
            graph->edge_weight[edge_cursor] = temp_nodes[node_index].edges[edge_slot].weight;
            graph->edge_class[edge_cursor] = temp_nodes[node_index].edges[edge_slot].road_class;
            edge_cursor++;
        }
    }
    graph->edge_offset[node_count] = edge_cursor;

    world->active_deliveries = PushArray<i32>(permanent, node_count);
    world->node_r = PushArray<u8>(permanent, node_count); world->node_g = PushArray<u8>(permanent, node_count); world->node_b = PushArray<u8>(permanent, node_count);
    world->node_base_r = PushArray<u8>(permanent, node_count); world->node_base_g = PushArray<u8>(permanent, node_count); world->node_base_b = PushArray<u8>(permanent, node_count);
    
    for (u32 node_index = 0; node_index < node_count; ++node_index) {
        color_gen::Rgb rgb = color_gen::get_next_distinct_color(&world->rng_state);
        world->node_base_r[node_index] = rgb.r;
        world->node_base_g[node_index] = rgb.g;
        world->node_base_b[node_index] = rgb.b;
    }

    world->agents_current = PushArray<Agent>(permanent, agent_count); world->agents_sorted = PushArray<Agent>(permanent, agent_count);
    world->agent_paths = PushArray<u32>(permanent, (u64)agent_count * MaxPathLength);
    world->render_previous = PushArray<AgentRenderState>(permanent, agent_count); world->render_current = PushArray<AgentRenderState>(permanent, agent_count);
    world->display_congestion = PushArray<f32>(permanent, agent_count);

    world->axis_spread = PushArray<u32>(permanent, GridDimension); world->cell_key_of_agent = PushArray<u32>(permanent, agent_count); world->cell_end = PushArray<u32>(permanent, GridCellCount);
    for (u32 axis_value = 0; axis_value < (u32)GridDimension; ++axis_value) {
        u32 spread = axis_value;
        spread = (spread | (spread << 4)) & 0x0F0Fu; spread = (spread | (spread << 2)) & 0x3333u; spread = (spread | (spread << 1)) & 0x5555u;
        world->axis_spread[axis_value] = spread;
    }

    world->path_g_score = PushArray<f32>(permanent, node_count); world->path_came_from = PushArray<u32>(permanent, node_count);
    world->path_stamp = PushArray<u32>(permanent, node_count); world->path_reverse = PushArray<u32>(permanent, MaxNodes);
    world->path_heap = PushArray<HeapItem>(permanent, (u64)total_edges + 2);

    if (!world->database.Open(database_path)) { sdl::log("failed to open database %s", database_path); scratch->Release(); return 0; }
    world->logger.Init(permanent, &world->database, run_start_ns);

    c8 sql_buffer[256];
    sdl::snprintf(sql_buffer, sizeof(sql_buffer), "INSERT INTO run_meta VALUES('seed','%" SDL_PRIu64 "')", seed); world->database.Exec(sql_buffer);
    sdl::snprintf(sql_buffer, sizeof(sql_buffer), "INSERT INTO run_meta VALUES('start_epoch_ns','%" SDL_PRIs64 "')", epoch_ns); world->database.Exec(sql_buffer);
    sdl::snprintf(sql_buffer, sizeof(sql_buffer), "INSERT INTO run_meta VALUES('agent_count','%u')", agent_count); world->database.Exec(sql_buffer);
    sdl::snprintf(sql_buffer, sizeof(sql_buffer), "INSERT INTO run_meta VALUES('node_count','%u')", node_count); world->database.Exec(sql_buffer);
    sdl::snprintf(sql_buffer, sizeof(sql_buffer), "INSERT INTO run_meta VALUES('edge_count','%u')", total_edges); world->database.Exec(sql_buffer);
    sdl::snprintf(sql_buffer, sizeof(sql_buffer), "INSERT INTO run_meta VALUES('sim_base_tick_ns','%" SDL_PRIu64 "')", SimBaseTickNs); world->database.Exec(sql_buffer);

    for (u32 node_index = 0; node_index < node_count; ++node_index) {
        world->logger.insert_node.BindInt64(1, node_index);
        world->logger.insert_node.BindDouble(2, graph->node_x[node_index]);
        world->logger.insert_node.BindDouble(3, graph->node_y[node_index]);
        world->logger.insert_node.BindInt64(4, graph->is_hub[node_index]);
        world->logger.insert_node.BindInt64(5, graph->is_delivery[node_index]);
        world->logger.insert_node.BindInt64(6, world->node_base_r[node_index]);
        world->logger.insert_node.BindInt64(7, world->node_base_g[node_index]);
        world->logger.insert_node.BindInt64(8, world->node_base_b[node_index]);
        world->logger.insert_node.Step(); world->logger.insert_node.Reset();
        world->database.rows_written++;

        for (u32 edge_index = graph->edge_offset[node_index]; edge_index < graph->edge_offset[node_index + 1]; ++edge_index) {
            world->logger.insert_edge.BindInt64(1, edge_index);
            world->logger.insert_edge.BindInt64(2, node_index);
            world->logger.insert_edge.BindInt64(3, graph->edge_dest[edge_index]);
            world->logger.insert_edge.BindDouble(4, graph->edge_weight[edge_index]);
            world->logger.insert_edge.BindInt64(5, graph->edge_class[edge_index]);
            world->logger.insert_edge.Step(); world->logger.insert_edge.Reset();
            world->database.rows_written++;
        }
    }

    u32* hub_list = PushArray<u32>(scratch, HubCount + 1);
    u32 hub_count = 0;
    for (u32 node_index = 0; node_index < node_count; ++node_index) {
        if (graph->is_hub[node_index] && hub_count <= HubCount) hub_list[hub_count++] = node_index;
    }
    if (hub_count == 0) hub_list[hub_count++] = 0;

    for (u32 agent_index = 0; agent_index < agent_count; ++agent_index) {
        Agent agent = world->agents_current[agent_index];
        u32 start_node = hub_list[sdl::rand_below(&world->rng_state, (i32)hub_count)];
        agent.agent_id = agent_index;
        agent.position = Vec2(graph->node_x[start_node], graph->node_y[start_node]);
        agent.wait_timer = ((f32)sdl::rand_below(&world->rng_state, 100) / 100.0f) * 2.0f;
        agent.state = AgentState_Waiting;

        world->logger.insert_agent.BindInt64(1, agent_index);
        world->logger.insert_agent.BindInt64(2, start_node);
        world->logger.insert_agent.Step(); world->logger.insert_agent.Reset();
        world->database.rows_written++;

        AssignDelivery(world, &agent, start_node);
        world->agents_current[agent_index] = agent;

        AgentRenderState* render_state = &world->render_current[agent_index];
        render_state->position = agent.position; render_state->heading = agent.heading;
        render_state->start_r = agent.start_r; render_state->start_g = agent.start_g; render_state->start_b = agent.start_b;
        render_state->target_r = agent.target_r; render_state->target_g = agent.target_g; render_state->target_b = agent.target_b;
        render_state->waiting = 1; render_state->congested = 0;
        world->render_previous[agent_index] = *render_state;
    }

    world->logger.Flush(); world->database.Commit(); world->database.Begin();
    scratch->Release();
    return 1;
}

void SimTick(SimWorld* world) {
    Graph* graph = &world->graph;
    Agent* current = world->agents_current;
    Agent* sorted = world->agents_sorted;
    u32 agent_count = world->agent_count;

    if (world->config.use_spatial_grid) {
        sdl::memset(world->cell_end, 0, GridCellCount * sizeof(world->cell_end[0]));
        for (u32 agent_index = 0; agent_index < agent_count; ++agent_index) {
            i32 cell_x = SDL_clamp((i32)(current[agent_index].position.X * InverseCellSize), 0, GridDimension - 1);
            i32 cell_y = SDL_clamp((i32)(current[agent_index].position.Y * InverseCellSize), 0, GridDimension - 1);
            u32 cell_key = world->axis_spread[cell_x] | (world->axis_spread[cell_y] << 1);
            world->cell_key_of_agent[agent_index] = cell_key;
            world->cell_end[cell_key]++;
        }
        u32 running_total = 0;
        for (u32 cell_key = 0; cell_key < GridCellCount; ++cell_key) {
            u32 bucket_size = world->cell_end[cell_key];
            world->cell_end[cell_key] = running_total;
            running_total += bucket_size;
        }
        for (u32 agent_index = 0; agent_index < agent_count; ++agent_index) {
            u32 destination = world->cell_end[world->cell_key_of_agent[agent_index]]++;
            sorted[destination] = current[agent_index];
        }
    } else {
        for (u32 i = 0; i < agent_count; ++i) sorted[i] = current[i];
    }

    for (u32 sorted_index = 0; sorted_index < agent_count; ++sorted_index) {
        Agent agent = sorted[sorted_index];
        u32* path = world->agent_paths + (u64)agent.agent_id * MaxPathLength;

        Vec2 seek = Vec2(0.0f, 0.0f);
        Vec2 separation = Vec2(0.0f, 0.0f);
        f32 speed_limit = NominalSpeed;
        i32 nearby_count = 0;
        b32 integrate = 0;

        switch (agent.state) {
            case AgentState_Waiting: {
                agent.wait_timer -= SimDeltaSeconds;
                agent.velocity = agent.velocity.Scale(0.8f);
                agent.position = agent.position.Add(agent.velocity.Scale(SimDeltaSeconds));
                agent.congestion_factor *= 0.9f;
                if (agent.wait_timer <= 0.0f) agent.state = (agent.cur_wp < agent.path_len) ? AgentState_Driving : AgentState_Idle;
            } break;

            case AgentState_Idle: {
                u32 here = (agent.path_len > 0) ? path[agent.path_len - 1] : 0;
                agent.state = AgentState_Driving;
                AssignDelivery(world, &agent, here);
                integrate = 1;
            } break;

            case AgentState_Driving: {
                integrate = 1;
                u32 previous_node = path[agent.cur_wp > 0 ? agent.cur_wp - 1 : 0];
                u32 next_node = path[agent.cur_wp];
                Vec2 previous_position = Vec2(graph->node_x[previous_node], graph->node_y[previous_node]);
                Vec2 next_position = Vec2(graph->node_x[next_node], graph->node_y[next_node]);

                Vec2 road_direction = next_position.Sub(previous_position);
                f32 road_length = road_direction.Length();
                if (road_length > 0.001f) {
                    road_direction = road_direction.Scale(1.0f / road_length);
                    Vec2 lane_normal = Vec2(-road_direction.Y, road_direction.X);
                    Vec2 lane_start = previous_position.Add(lane_normal.Scale(LaneOffset));
                    Vec2 lane_end   = next_position.Add(lane_normal.Scale(LaneOffset));

                    Vec2 from_lane_start = agent.position.Sub(lane_start);
                    f32 along_lane = from_lane_start.X * road_direction.X + from_lane_start.Y * road_direction.Y;
                    Vec2 projection = lane_start.Add(road_direction.Scale(along_lane));
                    if (along_lane > road_length) projection = lane_end;
                    else if (along_lane < 0.0f)   projection = lane_start;

                    Vec2 to_lane = projection.Sub(agent.position);
                    f32 lane_distance = to_lane.Length();

                    if (lane_distance > MaxLaneDeviation) agent.position = projection.Sub(to_lane.Scale(MaxLaneDeviation / lane_distance));
                    if (lane_distance > 0.1f) seek = seek.Add(to_lane.Scale(200.0f / lane_distance));
                    seek = seek.Add(road_direction.Scale(120.0f));

                    f32 target_distance = Distance2D(agent.position.X, agent.position.Y, next_position.X, next_position.Y);

                    if (world->config.use_spatial_grid) {
                        i32 cell_x = SDL_clamp((i32)(agent.position.X * InverseCellSize), 0, GridDimension - 1);
                        i32 cell_y = SDL_clamp((i32)(agent.position.Y * InverseCellSize), 0, GridDimension - 1);
                        for (i32 neighbor_delta_y = -1; neighbor_delta_y <= 1; ++neighbor_delta_y) {
                            for (i32 neighbor_delta_x = -1; neighbor_delta_x <= 1; ++neighbor_delta_x) {
                                i32 neighbor_cell_x = cell_x + neighbor_delta_x;
                                i32 neighbor_cell_y = cell_y + neighbor_delta_y;
                                if (neighbor_cell_x < 0 || neighbor_cell_y < 0 || neighbor_cell_x >= GridDimension || neighbor_cell_y >= GridDimension) continue;
                                u32 neighbor_key = world->axis_spread[neighbor_cell_x] | (world->axis_spread[neighbor_cell_y] << 1);
                                u32 bucket_begin = (neighbor_key == 0) ? 0 : world->cell_end[neighbor_key - 1];
                                u32 bucket_end = world->cell_end[neighbor_key];
                                for (u32 other_index = bucket_begin; other_index < bucket_end; ++other_index) {
                                    if (other_index == sorted_index) continue;
                                    Agent* other = &sorted[other_index];
                                    Vec2 relative = other->position.Sub(agent.position);
                                    f32 distance = relative.Length();
                                    if (distance > 0.01f && distance < NeighborRadius) {
                                        nearby_count++;
                                        if (world->config.use_boids) {
                                            f32 forward_dot = (relative.X * road_direction.X + relative.Y * road_direction.Y) / distance;
                                            f32 other_speed = other->velocity.Length();
                                            f32 velocity_alignment = (other_speed > 1.0f) ? (other->velocity.X * road_direction.X + other->velocity.Y * road_direction.Y) / other_speed : 1.0f;
                                            if (forward_dot > 0.85f && distance < CarFollowRadius && velocity_alignment > 0.3f) {
                                                f32 allowed_speed = (distance - CarFollowGap) * 3.0f;
                                                if (allowed_speed < 0.0f) allowed_speed = 0.0f;
                                                if (allowed_speed < speed_limit) speed_limit = allowed_speed;
                                            }
                                            if (distance < SeparationRadius) {
                                                f32 strength = (SeparationRadius - distance) / SeparationRadius;
                                                separation = separation.Sub(relative.Scale((strength * 120.0f) / distance));
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    } else {
                        for (u32 other_index = 0; other_index < agent_count; ++other_index) {
                            if (other_index == sorted_index) continue;
                            Agent* other = &sorted[other_index];
                            Vec2 relative = other->position.Sub(agent.position);
                            f32 distance = relative.Length();
                            if (distance > 0.01f && distance < NeighborRadius) {
                                nearby_count++;
                                if (world->config.use_boids) {
                                    f32 forward_dot = (relative.X * road_direction.X + relative.Y * road_direction.Y) / distance;
                                    f32 other_speed = other->velocity.Length();
                                    f32 velocity_alignment = (other_speed > 1.0f) ? (other->velocity.X * road_direction.X + other->velocity.Y * road_direction.Y) / other_speed : 1.0f;
                                    if (forward_dot > 0.85f && distance < CarFollowRadius && velocity_alignment > 0.3f) {
                                        f32 allowed_speed = (distance - CarFollowGap) * 3.0f;
                                        if (allowed_speed < 0.0f) allowed_speed = 0.0f;
                                        if (allowed_speed < speed_limit) speed_limit = allowed_speed;
                                    }
                                    if (distance < SeparationRadius) {
                                        f32 strength = (SeparationRadius - distance) / SeparationRadius;
                                        separation = separation.Sub(relative.Scale((strength * 120.0f) / distance));
                                    }
                                }
                            }
                        }
                    }

                    b32 is_congested = nearby_count > CongestionThreshold;
                    if (world->config.congestion_avoidance && is_congested) {
                        speed_limit *= 0.6f;
                        if (speed_limit < 15.0f) speed_limit = 15.0f;
                    }
                    if (is_congested && !agent.congested) {
                        i32 cell_x = SDL_clamp((i32)(agent.position.X * InverseCellSize), 0, GridDimension - 1);
                        i32 cell_y = SDL_clamp((i32)(agent.position.Y * InverseCellSize), 0, GridDimension - 1);
                        u32 cell_id = world->axis_spread[cell_x] | (world->axis_spread[cell_y] << 1);
                        world->logger.Notify(EventKind_Congestion, world->tick, agent.agent_id, cell_id, 0, (u32)nearby_count, agent.order_id);
                    }
                    agent.congested = is_congested ? 1 : 0;

                    if (target_distance < ArrivalRadius) {
                        agent.cur_wp++;
                        if (agent.cur_wp >= agent.path_len) {
                            world->active_deliveries[next_node]--;
                            if (world->active_deliveries[next_node] < 0) world->active_deliveries[next_node] = 0;
                            world->logger.Notify(EventKind_OrderDelivered, world->tick, agent.agent_id, next_node, 0, 0, agent.order_id);
                            
                            world->total_deliveries++;
                            world->total_delivery_ticks += (world->tick - agent.assigned_tick);

                            agent.wait_timer = 1.0f;
                            agent.state = AgentState_Waiting;
                            AssignDelivery(world, &agent, next_node);
                        }
                    }
                }
            } break;
        }

        if (integrate) {
            agent.velocity.X += (seek.X + separation.X) * SimDeltaSeconds;
            agent.velocity.Y += (seek.Y + separation.Y) * SimDeltaSeconds;
            agent.velocity = agent.velocity.Scale(0.85f);

            f32 speed = agent.velocity.Length();
            if (speed > speed_limit) agent.velocity = agent.velocity.Scale(speed_limit / speed);

            agent.congestion_factor = SDL_clamp(1.0f - (speed_limit / NominalSpeed), 0.0f, 1.0f);
            agent.position = agent.position.Add(agent.velocity.Scale(SimDeltaSeconds));

            f32 speed_now = agent.velocity.Length();
            if (speed_now > 2.0f) {
                f32 target_heading = sdl::atan2_turns(agent.velocity.Y, agent.velocity.X);
                f32 heading_difference = sdl::wrap_turns_signed(target_heading - agent.heading);
                f32 max_turn = MaxTurnRateTurns * SimDeltaSeconds;
                agent.heading += SDL_clamp(heading_difference, -max_turn, max_turn);
            }
        }

        current[sorted_index] = agent;
        world->render_previous[agent.agent_id] = world->render_current[agent.agent_id];
        AgentRenderState* render_state = &world->render_current[agent.agent_id];
        render_state->position = agent.position; render_state->heading = agent.heading;
        render_state->congestion_factor = agent.congestion_factor;
        render_state->start_r = agent.start_r; render_state->start_g = agent.start_g; render_state->start_b = agent.start_b;
        render_state->target_r = agent.target_r; render_state->target_g = agent.target_g; render_state->target_b = agent.target_b;
        render_state->waiting = (agent.state == AgentState_Waiting) ? 1 : 0;
        render_state->congested = agent.congested;
    }
    world->tick++;
}