#pragma once

struct VertexBuffer {
    SDL_Vertex* data; u64 count; u64 capacity;
    void Init(Arena* arena, u64 cap) { data = PushArray<SDL_Vertex>(arena, cap); capacity = cap; count = 0; }
    void Clear() { count = 0; }
    b32 PushBack(SDL_Vertex v) { if (count >= capacity) return 0; data[count++] = v; return 1; }
};

struct IndexBuffer {
    i32* data; u64 count; u64 capacity;
    void Init(Arena* arena, u64 cap) { data = PushArray<i32>(arena, cap); capacity = cap; count = 0; }
    void Clear() { count = 0; }
    b32 PushBack(i32 v) { if (count >= capacity) return 0; data[count++] = v; return 1; }
};

void PushArcFan(VertexBuffer* vertices, IndexBuffer* indices, SDL_FPoint center, f32 radius, f32 start_turns, f32 end_turns, SDL_FColor color, i32 segments) {
    if (radius <= 0.0f || segments < 1) return;
    if (vertices->count + (u64)segments + 2 > vertices->capacity) return;
    if (indices->count + (u64)segments * 3 > indices->capacity) return;
    i32 base_vertex = (i32)vertices->count;

    SDL_Vertex vertex; vertex.position = center; vertex.color = color; vertex.tex_coord = SDL_FPoint{0.0f, 0.0f};
    vertices->PushBack(vertex);

    for (i32 segment = 0; segment <= segments; ++segment) {
        f32 angle_turns = start_turns + (end_turns - start_turns) * ((f32)segment / (f32)segments);
        vertex.position = SDL_FPoint{ center.x + sdl::cos_turns(angle_turns) * radius, center.y + sdl::sin_turns(angle_turns) * radius };
        vertices->PushBack(vertex);
    }
    for (i32 segment = 0; segment < segments; ++segment) {
        indices->PushBack(base_vertex); indices->PushBack(base_vertex + 1 + segment); indices->PushBack(base_vertex + 2 + segment);
    }
}

void PushQuad(VertexBuffer* vertices, IndexBuffer* indices, SDL_FPoint corner_a, SDL_FPoint corner_b, SDL_FPoint corner_c, SDL_FPoint corner_d, SDL_FColor color) {
    if (vertices->count + 4 > vertices->capacity || indices->count + 6 > indices->capacity) return;
    i32 base_vertex = (i32)vertices->count;
    SDL_Vertex vertex; vertex.color = color; vertex.tex_coord = SDL_FPoint{0.0f, 0.0f};
    vertex.position = corner_a; vertices->PushBack(vertex);
    vertex.position = corner_b; vertices->PushBack(vertex);
    vertex.position = corner_c; vertices->PushBack(vertex);
    vertex.position = corner_d; vertices->PushBack(vertex);
    constexpr i32 quad_order[6] = { 0, 1, 2, 1, 3, 2 };
    for (i32 order_index = 0; order_index < 6; ++order_index) indices->PushBack(base_vertex + quad_order[order_index]);
}

struct App;
void RenderAll(App* app, f32 alpha, f32 delta_seconds);