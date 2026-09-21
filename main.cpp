#define SDL_MAIN_USE_CALLBACKS 1
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <sqlite3.h>

#include "base_arena.cpp"
#include "sdl_wrapper.cpp"
#include "config.cpp"
#include "color_gen.cpp"
#include "db.cpp"
#include "sim.cpp"
#include "render.cpp"

constexpr i32 SupersampleFactor   = 2;
constexpr i32 MaxRateExponent     = 20;
constexpr i32 MinRateExponent     = -4;
constexpr u64 SimBudgetNs         = 6000000ull;
constexpr u64 StatsWindowNs       = 500000000ull;
constexpr u32 DebugLineChars      = 60;
constexpr u32 DebugLineCount      = 20;
constexpr u64 VertexCapacity      = (u64)MaxNodes * 70 + (u64)MaxAgents * 6;
constexpr u64 IndexCapacity       = (u64)MaxNodes * 180 + (u64)MaxAgents * 6;
constexpr i32 MainCallbackRateHz  = 100;

struct App {
    sdl::Platform platform;
    sdl::Window window;
    sdl::Renderer renderer;
    sdl::RenderTarget aa_target;

    Arena* permanent_arena;
    SimWorld* world;
    Camera camera;

    VertexBuffer vertices;
    IndexBuffer indices;

    Config presets[10];
    b32 waiting_for_preset_save;

    b32 paused;
    b32 dragging;
    b32 debug_overlay;
    f32 drag_x, drag_y;
    i32 rate_exponent;

    u64 seed;
    u64 run_start_ns;
    u64 last_iteration_ns;
    u64 last_render_ns;
    u64 last_flush_ns;
    u64 sim_accumulator_ns;
    u64 render_accumulator_ns;
    u64 render_period_ns;
    u64 tick_period_ns;
    u64 stats_window_start_ns;
    u32 stats_ticks;
    u32 stats_frames;
    f32 measured_tick_rate;
    f32 measured_fps;

    c8 database_path[160];
};

void RenderAll(App* app, f32 alpha, f32 delta_seconds) {
    SimWorld* world = app->world;
    Graph* graph = &world->graph;
    Camera* camera = &app->camera;
    SDL_Renderer* sdl_renderer = app->renderer.handle;

    app->aa_target.Ensure(&app->renderer, &app->window, SupersampleFactor);
    i32 logical_width  = app->aa_target.logical_width;
    i32 logical_height = app->aa_target.logical_height;
    if (logical_width <= 0 || logical_height <= 0) return;

    f32 origin_x = sdl::floorf((f32)logical_width  * 0.5f - camera->X * camera->Zoom);
    f32 origin_y = sdl::floorf((f32)logical_height * 0.5f - camera->Y * camera->Zoom);

    VertexBuffer* vertices = &app->vertices;
    IndexBuffer* indices = &app->indices;
    vertices->Clear();
    indices->Clear();

    SDL_Texture* target = app->aa_target.handle;
    if (target) {
        SDL_SetRenderTarget(sdl_renderer, target);
        SDL_SetRenderScale(sdl_renderer, (f32)app->aa_target.texel_width / (f32)logical_width, (f32)app->aa_target.texel_height / (f32)logical_height);
    }
    SDL_SetRenderDrawColor(sdl_renderer, 15, 18, 24, 255);
    SDL_RenderClear(sdl_renderer);
    SDL_SetRenderDrawBlendMode(sdl_renderer, SDL_BLENDMODE_BLEND);

    constexpr SDL_FColor road_color = { 0.18f, 0.22f, 0.28f, 1.0f };
    for (u32 node_index = 0; node_index < graph->node_count; ++node_index) {
        u32 edge_begin = graph->edge_offset[node_index];
        u32 edge_end   = graph->edge_offset[node_index + 1];
        u8 max_class = 0;

        for (u32 edge_index = edge_begin; edge_index < edge_end; ++edge_index) {
            u32 other_node = graph->edge_dest[edge_index];
            if (graph->edge_class[edge_index] > max_class) max_class = graph->edge_class[edge_index];
            if (node_index >= other_node) continue;

            SDL_FPoint from_point = { graph->node_x[node_index] * camera->Zoom + origin_x, graph->node_y[node_index] * camera->Zoom + origin_y };
            SDL_FPoint to_point   = { graph->node_x[other_node] * camera->Zoom + origin_x, graph->node_y[other_node] * camera->Zoom + origin_y };
            f32 delta_x = to_point.x - from_point.x; f32 delta_y = to_point.y - from_point.y;
            f32 length = sdl::sqrtf(delta_x * delta_x + delta_y * delta_y);
            if (length <= 0.001f) continue;

            f32 half_thickness = (graph->edge_class[edge_index] == 2 ? 8.5f : 6.0f) * camera->Zoom;
            f32 normal_x = -delta_y / length; f32 normal_y =  delta_x / length;
            PushQuad(vertices, indices,
                     { from_point.x + normal_x * half_thickness, from_point.y + normal_y * half_thickness },
                     { from_point.x - normal_x * half_thickness, from_point.y - normal_y * half_thickness },
                     { to_point.x   + normal_x * half_thickness, to_point.y   + normal_y * half_thickness },
                     { to_point.x   - normal_x * half_thickness, to_point.y   - normal_y * half_thickness },
                     road_color);
        }
        if (edge_end > edge_begin) {
            SDL_FPoint center = { graph->node_x[node_index] * camera->Zoom + origin_x, graph->node_y[node_index] * camera->Zoom + origin_y };
            f32 radius = (max_class == 2 ? 8.5f : 6.0f) * camera->Zoom;
            PushArcFan(vertices, indices, center, radius, 0.0f, TurnFull, road_color, radius > 10.0f ? 12 : 8);
        }
    }

    for (u32 node_index = 0; node_index < graph->node_count; ++node_index) {
        if (!graph->is_hub[node_index]) continue;
        SDL_FPoint center = { graph->node_x[node_index] * camera->Zoom + origin_x, graph->node_y[node_index] * camera->Zoom + origin_y };
        f32 radius = SDL_max(11.0f * camera->Zoom, 3.0f);
        PushArcFan(vertices, indices, center, radius * 1.35f, 0.0f, TurnFull, { 0.06f, 0.05f, 0.03f, 0.6f }, 14);
        PushArcFan(vertices, indices, center, radius,         0.0f, TurnFull, { 1.0f, 0.82f, 0.20f, 1.0f }, 14);
    }

    f32 time_seconds = (f32)((sdl::now_ns() / 1000000ull) % 1000000ull) * 0.001f;
    f32 pulse = (sdl::sin_turns(time_seconds * RadiansToTurns(6.0f)) + 1.0f) * 0.5f;
    for (u32 node_index = 0; node_index < graph->node_count; ++node_index) {
        if (world->active_deliveries[node_index] <= 0) continue;
        SDL_FPoint center = { graph->node_x[node_index] * camera->Zoom + origin_x, graph->node_y[node_index] * camera->Zoom + origin_y };
        f32 radius = 8.0f * camera->Zoom + pulse * 5.0f * camera->Zoom;
        SDL_FColor color = { world->node_r[node_index] / 255.0f, world->node_g[node_index] / 255.0f, world->node_b[node_index] / 255.0f, 0.85f };
        PushArcFan(vertices, indices, center, radius, 0.0f, TurnFull, color, 10);
    }

    for (u32 agent_id = 0; agent_id < world->agent_count; ++agent_id) {
        AgentRenderState* previous = &world->render_previous[agent_id];
        AgentRenderState* latest   = &world->render_current[agent_id];

        world->display_congestion[agent_id] = sdl::lerp_exp(world->display_congestion[agent_id], latest->congestion_factor, 12.0f, delta_seconds);

        f32 world_x = previous->position.X + (latest->position.X - previous->position.X) * alpha;
        f32 world_y = previous->position.Y + (latest->position.Y - previous->position.Y) * alpha;
        f32 heading_difference = sdl::wrap_turns_signed(latest->heading - previous->heading);
        f32 heading_turns = previous->heading + heading_difference * alpha;

        SDL_FPoint center = { world_x * camera->Zoom + origin_x, world_y * camera->Zoom + origin_y };
        if (center.x < -20.0f || center.y < -20.0f || center.x > (f32)logical_width + 20.0f || center.y > (f32)logical_height + 20.0f) continue;
        if (vertices->count + 6 > vertices->capacity || indices->count + 6 > indices->capacity) break;

        f32 heading_x = sdl::cos_turns(heading_turns); f32 heading_y = sdl::sin_turns(heading_turns);
        f32 agent_size = SDL_max(3.5f * camera->Zoom, 1.5f);

        SDL_FColor start_color, target_color;
        if (latest->waiting) {
            start_color = target_color = SDL_FColor{ 1.0f, 0.0f, 0.0f, 1.0f };
        } else {
            f32 brake = world->display_congestion[agent_id];
            f32 inverse = 1.0f - brake;
            start_color  = { (latest->start_r  / 255.0f) * inverse + 1.00f * brake, (latest->start_g  / 255.0f) * inverse + 0.12f * brake, (latest->start_b  / 255.0f) * inverse + 0.08f * brake, 1.0f };
            target_color = { (latest->target_r / 255.0f) * inverse + 1.00f * brake, (latest->target_g / 255.0f) * inverse + 0.12f * brake, (latest->target_b / 255.0f) * inverse + 0.08f * brake, 1.0f };

            if (latest->congested) {
                f32 flash = (sdl::sinf(time_seconds * 30.0f) + 1.0f) * 0.5f;
                start_color.r = sdl::lerp(start_color.r, 1.0f, flash); start_color.g = sdl::lerp(start_color.g, 1.0f, flash); start_color.b = sdl::lerp(start_color.b, 1.0f, flash);
                target_color.r = sdl::lerp(target_color.r, 1.0f, flash); target_color.g = sdl::lerp(target_color.g, 1.0f, flash); target_color.b = sdl::lerp(target_color.b, 1.0f, flash);
            }
        }

        SDL_FPoint tip            = { center.x + heading_x * agent_size * 2.2f, center.y + heading_y * agent_size * 2.2f };
        SDL_FPoint left_shoulder  = { center.x - heading_x * agent_size * 0.9f + heading_y * agent_size * 0.85f, center.y - heading_y * agent_size * 0.9f - heading_x * agent_size * 0.85f };
        SDL_FPoint right_shoulder = { center.x - heading_x * agent_size * 0.9f - heading_y * agent_size * 0.85f, center.y - heading_y * agent_size * 0.9f + heading_x * agent_size * 0.85f };
        SDL_FPoint notch          = { center.x - heading_x * agent_size * 0.35f, center.y - heading_y * agent_size * 0.35f };

        i32 base_vertex = (i32)vertices->count;
        SDL_Vertex vertex; vertex.tex_coord = SDL_FPoint{ 0.0f, 0.0f };
        vertex.color = start_color;  vertex.position = tip;             vertices->PushBack(vertex);
        vertex.position = left_shoulder;                                vertices->PushBack(vertex);
        vertex.position = notch;                                        vertices->PushBack(vertex);
        vertex.color = target_color; vertex.position = tip;             vertices->PushBack(vertex);
        vertex.position = notch;                                        vertices->PushBack(vertex);
        vertex.position = right_shoulder;                               vertices->PushBack(vertex);
        for (i32 corner = 0; corner < 6; ++corner) indices->PushBack(base_vertex + corner);
    }

    if (vertices->count > 0) SDL_RenderGeometry(sdl_renderer, nullptr, vertices->data, (i32)vertices->count, indices->data, (i32)indices->count);

    if (target) {
        SDL_SetRenderTarget(sdl_renderer, nullptr);
        SDL_SetRenderScale(sdl_renderer, 1.0f, 1.0f);
        SDL_RenderTexture(sdl_renderer, target, nullptr, nullptr);
    }

    if (app->debug_overlay) {
        c8 lines[DebugLineCount][DebugLineChars + 4];
        u32 line_count = 0;

        f32 efficiency = world->total_deliveries > 0 ? (f32)world->total_delivery_ticks / (f32)world->total_deliveries : 0.0f;

        sdl::snprintf(lines[line_count++], DebugLineChars, "render %.1f fps (%.1f Hz target)", app->measured_fps, 1.0e9f / (f32)app->render_period_ns);
        sdl::snprintf(lines[line_count++], DebugLineChars, "sim %.0f ticks/s (target %.0f)", app->measured_tick_rate, 1.0e9f / (f32)app->tick_period_ns);
        sdl::snprintf(lines[line_count++], DebugLineChars, "rate 2^%d x  tick %" SDL_PRIu64, app->rate_exponent, world->tick);
        sdl::snprintf(lines[line_count++], DebugLineChars, "sim debt %.2f ms%s", (f32)app->sim_accumulator_ns * 1.0e-6f, app->sim_accumulator_ns > app->tick_period_ns ? " (BEHIND)" : "");
        sdl::snprintf(lines[line_count++], DebugLineChars, "agents %u / %u max%s", world->agent_count, MaxAgents, world->agent_count > MaxAgents ? " OVER LIMIT" : "");
        sdl::snprintf(lines[line_count++], DebugLineChars, "efficiency: %.1f avg ticks/delivery", efficiency);
        sdl::snprintf(lines[line_count++], DebugLineChars, "--------------------------------------------------");
        sdl::snprintf(lines[line_count++], DebugLineChars, "Hold [B] + UP/DN: Boids      (%s)", world->config.use_boids ? "ON" : "OFF");
        sdl::snprintf(lines[line_count++], DebugLineChars, "Hold [G] + UP/DN: Grid       (%s)", world->config.use_spatial_grid ? "ON" : "OFF");
        sdl::snprintf(lines[line_count++], DebugLineChars, "Hold [P] + UP/DN: Path Mode  (%s)", world->config.pathfinding_mode == 0 ? "A*" : (world->config.pathfinding_mode == 1 ? "BFS" : "DFS"));
        sdl::snprintf(lines[line_count++], DebugLineChars, "Hold [C] + UP/DN: Congestion (%s)", world->config.congestion_avoidance ? "ON" : "OFF");
        sdl::snprintf(lines[line_count++], DebugLineChars, "--------------------------------------------------");
        sdl::snprintf(lines[line_count++], DebugLineChars, "Ctrl+S then 0-9 to Save Preset");
        sdl::snprintf(lines[line_count++], DebugLineChars, "Ctrl+0-9 to Load Preset");
        sdl::snprintf(lines[line_count++], DebugLineChars, "--------------------------------------------------");
        sdl::snprintf(lines[line_count++], DebugLineChars, "db rows %" SDL_PRIu64 " commits %" SDL_PRIu64, world->database.rows_written, world->database.commit_count);
        sdl::snprintf(lines[line_count++], DebugLineChars, "events %" SDL_PRIu64 " (queued %u)", world->logger.total_notified, (u32)world->logger.events.count);
        sdl::snprintf(lines[line_count++], DebugLineChars, "vertices %u / %u", (u32)vertices->count, (u32)vertices->capacity);
        sdl::snprintf(lines[line_count++], DebugLineChars, "zoom %.2f", camera->Zoom);

        SDL_SetRenderDrawColor(sdl_renderer, 0, 0, 0, 170);
        SDL_FRect background = { 6.0f, 6.0f, (f32)(DebugLineChars * 8 + 12), (f32)(line_count * 12 + 8) };
        SDL_RenderFillRect(sdl_renderer, &background);
        SDL_SetRenderDrawColor(sdl_renderer, 235, 235, 235, 255);
        for (u32 line_index = 0; line_index < line_count; ++line_index) {
            SDL_RenderDebugText(sdl_renderer, 12.0f, 10.0f + (f32)line_index * 12.0f, lines[line_index]);
        }
    }

    if (app->waiting_for_preset_save) {
        SDL_SetRenderDrawColor(sdl_renderer, 0, 0, 0, 220);
        SDL_FRect popup = { (f32)logical_width * 0.5f - 200.0f, (f32)logical_height * 0.5f - 30.0f, 400.0f, 60.0f };
        SDL_RenderFillRect(sdl_renderer, &popup);
        SDL_SetRenderDrawColor(sdl_renderer, 255, 255, 255, 255);
        SDL_RenderDebugText(sdl_renderer, popup.x + 20.0f, popup.y + 25.0f, "PRESS 0-9 TO SAVE PRESET (ESC to Cancel)");
    }
}

#ifdef SDL_MAIN_USE_CALLBACKS
    #define GAME_INIT_FN    SDL_AppInit
    #define GAME_ITERATE_FN SDL_AppIterate
    #define GAME_EVENT_FN   SDL_AppEvent
    #define GAME_QUIT_FN    SDL_AppQuit
#else
    #define GAME_INIT_FN    game_init
    #define GAME_ITERATE_FN game_iterate
    #define GAME_EVENT_FN   game_event
    #define GAME_QUIT_FN    game_quit
#endif

SDL_AppResult GAME_INIT_FN(void** appstate, int argc, char* argv[]) {
    c8 rate_text[16];
    sdl::snprintf(rate_text, sizeof(rate_text), "%d", MainCallbackRateHz);
    SDL_SetHint(SDL_HINT_MAIN_CALLBACK_RATE, rate_text);

    Arena* permanent = ArenaAlloc();
    App* app = PushStruct<App>(permanent);
    app->permanent_arena = permanent;
    *appstate = app;

    if (!app->platform.Init()) { sdl::log("SDL_Init failed: %s", sdl::get_error()); return SDL_APP_FAILURE; }

    SDL_Time epoch_ns = 0;
    SDL_GetCurrentTime(&epoch_ns);
    u64 seed = (argc > 1) ? (u64)sdl::str_to_u64(argv[1], nullptr, 10) : (u64)epoch_ns;
    app->seed = seed;
    c8 message[192];
    sdl::snprintf(message, sizeof(message), "seed %" SDL_PRIu64 " (pass it as argv[1] to recreate this run)", seed);
    sdl::log(message);

    SDL_DateTime date_time;
    SDL_TimeToDateTime(epoch_ns, &date_time, true);
    sdl::snprintf(app->database_path, sizeof(app->database_path), "fleet_%04d%02d%02d_%02d%02d%02d_%03d_seed%" SDL_PRIu64 ".db",
                 date_time.year, date_time.month, date_time.day, date_time.hour, date_time.minute, date_time.second, date_time.nanosecond / 1000000, seed);

    if (!app->window.Create("Logistics AI Fleet Simulator", 1600, 900, SDL_WINDOW_RESIZABLE)) return SDL_APP_FAILURE;
    if (!app->renderer.Create(&app->window)) return SDL_APP_FAILURE;

    app->run_start_ns = sdl::now_ns();
    app->world = PushStruct<SimWorld>(permanent);
    
    LoadAllPresets(app->presets);
    app->world->config = app->presets[0]; // Load slot 0 by default

    if (!SimInit(app->world, permanent, seed, DefaultAgentCount, app->database_path, app->run_start_ns, (i64)epoch_ns)) return SDL_APP_FAILURE;
    sdl::snprintf(message, sizeof(message), "database: %s", app->database_path);
    sdl::log(message);

    app->vertices.Init(permanent, VertexCapacity);
    app->indices.Init(permanent, IndexCapacity);

    app->camera = { MapCenter, MapCenter, 0.4f };
    app->rate_exponent = 0;
    app->render_period_ns = (u64)((f64)1000000000ull / (f64)app->window.RefreshRate());
    app->tick_period_ns = SimBaseTickNs;

    u64 now_ns = sdl::now_ns();
    app->last_iteration_ns = now_ns; app->last_render_ns = now_ns; app->last_flush_ns = now_ns; app->stats_window_start_ns = now_ns;
    
    app->paused = 1;
    app->waiting_for_preset_save = 0;
    return SDL_APP_CONTINUE;
}

SDL_AppResult GAME_EVENT_FN(void* appstate, SDL_Event* event) {
    App* app = (App*)appstate;
    switch (event->type) {
        case SDL_EVENT_QUIT: return SDL_APP_SUCCESS;
        case SDL_EVENT_KEY_DOWN: {
            if (app->waiting_for_preset_save) {
                if (event->key.key >= SDLK_0 && event->key.key <= SDLK_9) {
                    int slot = event->key.key - SDLK_0;
                    app->presets[slot] = app->world->config;
                    SaveAllPresets(app->presets);
                    app->waiting_for_preset_save = 0;
                } else if (event->key.key == SDLK_ESCAPE) {
                    app->waiting_for_preset_save = 0;
                }
                break;
            }

            if (event->key.mod & SDL_KMOD_CTRL) {
                if (event->key.key == SDLK_S) {
                    app->waiting_for_preset_save = 1;
                } else if (event->key.key >= SDLK_0 && event->key.key <= SDLK_9) {
                    int slot = event->key.key - SDLK_0;
                    app->world->config = app->presets[slot];
                    sdl::log("Loaded preset slot %d", slot);
                }
            } else {
                const bool* keys = SDL_GetKeyboardState(NULL);
                
                if (event->key.key == SDLK_UP) {
                    if (keys[SDL_SCANCODE_B]) app->world->config.use_boids = 1;
                    if (keys[SDL_SCANCODE_G]) app->world->config.use_spatial_grid = 1;
                    if (keys[SDL_SCANCODE_P]) app->world->config.pathfinding_mode = (app->world->config.pathfinding_mode + 1) % 3;
                    if (keys[SDL_SCANCODE_C]) app->world->config.congestion_avoidance = 1;
                } else if (event->key.key == SDLK_DOWN) {
                    if (keys[SDL_SCANCODE_B]) app->world->config.use_boids = 0;
                    if (keys[SDL_SCANCODE_G]) app->world->config.use_spatial_grid = 0;
                    if (keys[SDL_SCANCODE_P]) app->world->config.pathfinding_mode = (app->world->config.pathfinding_mode + 2) % 3;
                    if (keys[SDL_SCANCODE_C]) app->world->config.congestion_avoidance = 0;
                } else {
                    switch (event->key.key) {
                        case SDLK_P: app->paused = !app->paused; break;
                        case SDLK_D: app->debug_overlay = !app->debug_overlay; break;
                        case SDLK_LEFTBRACKET: app->rate_exponent = SDL_max(app->rate_exponent - 1, MinRateExponent); app->sim_accumulator_ns = 0; break;
                        case SDLK_RIGHTBRACKET: app->rate_exponent = SDL_min(app->rate_exponent + 1, MaxRateExponent); app->sim_accumulator_ns = 0; break;
                        default: break;
                    }
                }
            }
        } break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN: {
            if (event->button.button == SDL_BUTTON_RIGHT) { app->dragging = 1; app->drag_x = event->button.x; app->drag_y = event->button.y; }
        } break;
        case SDL_EVENT_MOUSE_BUTTON_UP: {
            if (event->button.button == SDL_BUTTON_RIGHT) app->dragging = 0;
        } break;
        case SDL_EVENT_MOUSE_MOTION: {
            if (app->dragging) {
                app->camera.X -= (event->motion.x - app->drag_x) / app->camera.Zoom; app->camera.Y -= (event->motion.y - app->drag_y) / app->camera.Zoom;
                app->drag_x = event->motion.x; app->drag_y = event->motion.y;
            }
        } break;
        case SDL_EVENT_MOUSE_WHEEL: {
            i32 window_width, window_height; app->window.GetSize(&window_width, &window_height);
            f32 mouse_x, mouse_y; sdl::get_mouse_state(&mouse_x, &mouse_y);
            f32 world_x = (mouse_x - (f32)window_width  * 0.5f) / app->camera.Zoom + app->camera.X;
            f32 world_y = (mouse_y - (f32)window_height * 0.5f) / app->camera.Zoom + app->camera.Y;
            app->camera.Zoom = SDL_clamp(app->camera.Zoom * (event->wheel.y > 0 ? 1.15f : 0.85f), 0.04f, 15.0f);
            app->camera.X = world_x - (mouse_x - (f32)window_width  * 0.5f) / app->camera.Zoom;
            app->camera.Y = world_y - (mouse_y - (f32)window_height * 0.5f) / app->camera.Zoom;
        } break;
        case SDL_EVENT_WINDOW_DISPLAY_CHANGED: {
            app->render_period_ns = (u64)((f64)1000000000ull / (f64)app->window.RefreshRate());
        } break;
        default: break;
    }
    return SDL_APP_CONTINUE;
}

SDL_AppResult GAME_ITERATE_FN(void* appstate) {
    App* app = (App*)appstate;
    SimWorld* world = app->world;
    u64 now_ns = sdl::now_ns();
    u64 frame_ns = now_ns - app->last_iteration_ns;
    app->last_iteration_ns = now_ns;

    app->tick_period_ns = (app->rate_exponent >= 0) ? (SimBaseTickNs >> app->rate_exponent) : (SimBaseTickNs << (-app->rate_exponent));
    if (app->tick_period_ns < 1) app->tick_period_ns = 1;

    if (!app->paused) app->sim_accumulator_ns += frame_ns;
    while (app->sim_accumulator_ns >= app->tick_period_ns) {
        SimTick(world);
        app->sim_accumulator_ns -= app->tick_period_ns;
        app->stats_ticks++;
        if (sdl::now_ns() - now_ns >= SimBudgetNs) break;
    }

    if (now_ns - app->last_flush_ns >= DbFlushIntervalNs) {
        app->last_flush_ns = now_ns;
        world->logger.Flush(); world->database.Commit(); world->database.Begin();
    }

    app->render_accumulator_ns += frame_ns;
    if (app->render_accumulator_ns >= app->render_period_ns) {
        app->render_accumulator_ns -= app->render_period_ns;
        if (app->render_accumulator_ns >= app->render_period_ns) app->render_accumulator_ns = 0;

        f32 render_delta_seconds = (f32)(now_ns - app->last_render_ns) * 1.0e-9f;
        app->last_render_ns = now_ns;

        f32 alpha = SDL_clamp((f32)app->sim_accumulator_ns / (f32)app->tick_period_ns, 0.0f, 1.0f);
        RenderAll(app, alpha, render_delta_seconds);
        SDL_RenderPresent(app->renderer.handle);
        app->stats_frames++;
    }

    u64 stats_elapsed_ns = now_ns - app->stats_window_start_ns;
    if (stats_elapsed_ns >= StatsWindowNs) {
        f32 elapsed_seconds = (f32)stats_elapsed_ns * 1.0e-9f;
        app->measured_tick_rate = (f32)app->stats_ticks / elapsed_seconds;
        app->measured_fps = (f32)app->stats_frames / elapsed_seconds;
        app->stats_ticks = 0; app->stats_frames = 0; app->stats_window_start_ns = now_ns;
    }
    return SDL_APP_CONTINUE;
}

void GAME_QUIT_FN(void* appstate, SDL_AppResult result) {
    App* app = (App*)appstate;
    if (!app) return;
    if (app->world) { app->world->logger.Flush(); app->world->logger.Close(); app->world->database.Close(); }
    app->aa_target.Destroy(); app->renderer.Destroy(); app->window.Destroy(); app->platform.Quit();
    app->permanent_arena->Release();
}