#pragma once

struct Config {
    b32 use_boids{1};
    b32 use_spatial_grid{1};
    i32 pathfinding_mode{0}; // 0 = A*, 1 = BFS, 2 = DFS
    b32 congestion_avoidance{1};
};

void SaveAllPresets(Config presets[10]) {
    SDL_IOStream* io = SDL_IOFromFile("presets.txt", "w");
    if (!io) {
        sdl::log("Failed to open presets.txt for writing.");
        return;
    }
    
    char buf[2048];
    buf[0] = '\0';
    
    for (int i = 0; i < 10; ++i) {
        char block[128];
        sdl::snprintf(block, sizeof(block), "[%d]\nboids=%d\ngrid=%d\npath=%d\ncong=%d\n",
                      i, presets[i].use_boids, presets[i].use_spatial_grid, 
                      presets[i].pathfinding_mode, presets[i].congestion_avoidance);
        SDL_strlcat(buf, block, sizeof(buf));
    }
    
    SDL_WriteIO(io, buf, SDL_strlen(buf));
    SDL_CloseIO(io);
    sdl::log("Saved all 10 slots to presets.txt");
}

void LoadAllPresets(Config presets[10]) {
    size_t size;
    void* data = SDL_LoadFile("presets.txt", &size);
    if (data) {
        char* str = (char*)data;
        for (int i = 0; i < 10; ++i) {
            char header[16];
            sdl::snprintf(header, sizeof(header), "[%d]", i);
            char* block = sdl::strstr(str, header);
            if (block) {
                char* boids = sdl::strstr(block, "boids="); if (boids) presets[i].use_boids = boids[6] - '0';
                char* grid  = sdl::strstr(block, "grid=");  if (grid)  presets[i].use_spatial_grid = grid[5] - '0';
                char* path  = sdl::strstr(block, "path=");  if (path)  presets[i].pathfinding_mode = path[5] - '0';
                char* cong  = sdl::strstr(block, "cong=");  if (cong)  presets[i].congestion_avoidance = cong[5] - '0';
            }
        }
        SDL_free(data);
        sdl::log("Loaded presets.txt");
    } else {
        SaveAllPresets(presets);
        sdl::log("Created presets.txt with default configurations");
    }
}