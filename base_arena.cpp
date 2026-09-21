#pragma once

#if defined(__APPLE__) || defined(__MACH__)
#error "Apple platforms are explicitly unsupported."
#endif

#include <SDL3/SDL_stdinc.h>

#ifndef ARENA_FREE_LIST
#define ARENA_FREE_LIST 1
#endif

using f32 = float;
using f64 = double;
using u8  = Uint8;
using u16 = Uint16;
using u32 = Uint32;
using u64 = Uint64;
using i8  = Sint8;
using i16 = Sint16;
using i32 = Sint32;
using i64 = Sint64;
using c8  = char;
using b32 = Sint32;

constexpr f32 TurnFull    = 1.0f;
constexpr f32 TurnHalf    = 0.5f;
constexpr f32 TurnQuarter = 0.25f;
constexpr f64 RadiansPerTurn = 6.28318530717958647692;

constexpr f32 TurnsToRadians(f32 turns) { return (f32)(turns * RadiansPerTurn); }
constexpr f32 RadiansToTurns(f32 radians) { return (f32)(radians / RadiansPerTurn); }

consteval u64 KB(u64 x) { return x << 10; }
consteval u64 MB(u64 x) { return x << 20; }
consteval u64 GB(u64 x) { return x << 30; }

constexpr u64 NextPow2(u64 v) noexcept {
    if (v <= 1) return 1;
    v--; v |= v >> 1; v |= v >> 2; v |= v >> 4; v |= v >> 8; v |= v >> 16; v |= v >> 32;
    return v + 1;
}

constexpr u64 ALIGN_POW2(u64 x, u64 p) {
    if (p <= 1) return x;
    p = NextPow2(p);
    return (x + p - 1) & ~(p - 1);
}

constexpr u64 ARENA_DEFAULT_RESERVE = MB(64);
constexpr u64 ARENA_DEFAULT_COMMIT  = KB(64);
constexpr u64 ARENA_HEADER_SIZE     = 128;

enum ArenaFlags : u64 {
    ARENA_FLAG_NONE               = 0,
    ARENA_FLAG_NO_CHAIN          = (1 << 0),
    ARENA_FLAG_IS_BACKING_BUFFER = (1 << 16)
};

#if defined(__has_feature)
    #if __has_feature(address_sanitizer)
        #define ARENA_ASAN_ENABLED 1
    #endif
#endif
#if defined(__SANITIZE_ADDRESS__)
    #define ARENA_ASAN_ENABLED 1
#endif

#if defined(ARENA_ASAN_ENABLED)
    extern "C" {
        void __asan_poison_memory_region(void const volatile *addr, size_t size);
        void __asan_unpoison_memory_region(void const volatile *addr, size_t size);
    }
    #define AsanPoisonMemoryRegion(ptr, size)   __asan_poison_memory_region((ptr), (size))
    #define AsanUnpoisonMemoryRegion(ptr, size) __asan_unpoison_memory_region((ptr), (size))
#else
    #define AsanPoisonMemoryRegion(ptr, size)   ((void)0)
    #define AsanUnpoisonMemoryRegion(ptr, size) ((void)0)
#endif

#if defined(linux) || defined(__linux) || defined(__linux__)
    #include <sys/mman.h>
    #include <unistd.h>
    inline void OS_Init() {}
    inline void* OS_Reserve(u64 size) {
        void* p = mmap(nullptr, size, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
        return (p == MAP_FAILED) ? nullptr : p;
    }
    inline b32 OS_Commit(void* ptr, u64 size) { return mprotect(ptr, size, PROT_READ | PROT_WRITE) == 0; }
    inline void OS_Decommit(void* ptr, u64 size) { madvise(ptr, size, MADV_DONTNEED); mprotect(ptr, size, PROT_NONE); }
    inline void OS_Release(void* ptr, u64 size) { munmap(ptr, size); }
    inline u64 OS_GetPageSize() { return static_cast<u64>(getpagesize()); }
#elif defined(_WIN32)
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>
    inline void OS_Init() {}
    inline void* OS_Reserve(u64 size) { return VirtualAlloc(nullptr, size, MEM_RESERVE, PAGE_NOACCESS); }
    inline b32 OS_Commit(void* ptr, u64 size) { return VirtualAlloc(ptr, size, MEM_COMMIT, PAGE_READWRITE) != nullptr; }
    inline void OS_Decommit(void* ptr, u64 size) { VirtualFree(ptr, size, MEM_DECOMMIT); }
    inline void OS_Release(void* ptr, u64 size) { (void)size; VirtualFree(ptr, 0, MEM_RELEASE); }
    inline u64 OS_GetPageSize() { SYSTEM_INFO si; GetSystemInfo(&si); return static_cast<u64>(si.dwPageSize); }
    inline u64 OS_GetAllocationGranularity() { SYSTEM_INFO si; GetSystemInfo(&si); return static_cast<u64>(si.dwAllocationGranularity); }
#endif

struct ArenaParams {
    u64 reserve_size{ARENA_DEFAULT_RESERVE};
    u64 commit_size{ARENA_DEFAULT_COMMIT};
    u64 flags{ARENA_FLAG_NONE};
    void* optional_backing_buffer{nullptr};
    const char* allocation_site_file{nullptr};
    i32 allocation_site_line{0};
    const char* name{nullptr};
};

class Arena {
public:
    Arena* prev{nullptr};
    Arena* current{nullptr};
    u64 flags{ARENA_FLAG_NONE};
    u64 cmt_size{0};
    u64 res_size{0};
    u64 base_pos{0};
    u64 pos{0};
    u64 cmt{0};
    u64 res{0};
    const char* allocation_site_file{nullptr};
    i32 allocation_site_line{0};
    const char* name{nullptr};

#if ARENA_FREE_LIST
    Arena* free_last{nullptr};
#endif

    [[nodiscard]] void* Push(u64 size, u64 align = 8, b32 zero = 1);
    void  PopTo(u64 target_pos);
    void  Pop(u64 amt);
    void  Clear();
    void  Release();
    [[nodiscard]] u64 Pos() const;
};

static_assert(sizeof(Arena) <= ARENA_HEADER_SIZE, "Arena header exceeds predefined layout boundary size.");

[[nodiscard]] Arena* ArenaAlloc(const ArenaParams& params = {});

template <typename T>
[[nodiscard]] T* PushArray(Arena* arena, u64 count, u64 align = alignof(T), b32 zero = 1) {
    if (!arena) return nullptr;
    if (count > 0 && sizeof(T) > UINT64_MAX / count) return nullptr;
    return static_cast<T*>(arena->Push(sizeof(T) * count, align, zero));
}

template <typename T>
[[nodiscard]] T* PushStruct(Arena* arena, u64 align = alignof(T), b32 zero = 1) {
    return PushArray<T>(arena, 1, align, zero);
}

b32 g_os_layer_initialized = 0;

inline Arena* ArenaAlloc(const ArenaParams& params) {
    if (!g_os_layer_initialized) { OS_Init(); g_os_layer_initialized = 1; }
    u64 page_size = OS_GetPageSize();
    u64 reserve_size = params.reserve_size ? params.reserve_size : ARENA_DEFAULT_RESERVE;
    u64 commit_size  = params.commit_size  ? params.commit_size  : ARENA_DEFAULT_COMMIT;
    u64 flags        = params.flags;

    if (reserve_size > 0x8000000000000000ULL || commit_size > 0x8000000000000000ULL) return nullptr;

    void* base = params.optional_backing_buffer;
    if (base == nullptr) {
        u64 next_res = NextPow2(reserve_size);
        u64 next_cmt = NextPow2(commit_size);
        if (next_res < reserve_size || next_cmt < commit_size) return nullptr;

#if defined(_WIN32)
        u64 alloc_granularity = OS_GetAllocationGranularity();
        reserve_size = ALIGN_POW2(next_res, alloc_granularity);
#else
        reserve_size = ALIGN_POW2(next_res, page_size);
#endif
        commit_size  = ALIGN_POW2(next_cmt, page_size);
        if (commit_size > reserve_size) reserve_size = commit_size;

        base = OS_Reserve(reserve_size);
        if (base != nullptr) {
            if (!OS_Commit(base, commit_size)) { OS_Release(base, reserve_size); return nullptr; }
        }
        if (base == nullptr) return nullptr;
    } else {
        flags |= ARENA_FLAG_IS_BACKING_BUFFER;
        flags |= ARENA_FLAG_NO_CHAIN;
        commit_size = reserve_size;
    }

    AsanPoisonMemoryRegion(base, reserve_size);
    AsanUnpoisonMemoryRegion(base, ARENA_HEADER_SIZE);

    Arena* arena = static_cast<Arena*>(base);
    SDL_memset(arena, 0, sizeof(Arena));
    arena->prev       = nullptr;
    arena->current    = arena;
    arena->flags      = flags;
    arena->cmt_size   = commit_size;
    arena->res_size   = reserve_size;
    arena->base_pos   = 0;
    arena->pos        = ARENA_HEADER_SIZE;
    arena->cmt        = commit_size;
    arena->res        = reserve_size;
    arena->allocation_site_file = params.allocation_site_file;
    arena->allocation_site_line = params.allocation_site_line;
    arena->name       = params.name;
#if ARENA_FREE_LIST
    arena->free_last  = nullptr;
#endif
    return arena;
}

inline void* Arena::Push(u64 size, u64 align, b32 zero) {
    if (!this) return nullptr;
    if (align == 0) align = 1;
    Arena* curr = current;
    if (!curr) return nullptr;

    u64 aligned_pos = ALIGN_POW2(curr->pos, align);
    u64 next_pos = aligned_pos + size;

    if (next_pos < aligned_pos || next_pos < size) return nullptr;

    if (next_pos > curr->res) {
        if (curr->flags & ARENA_FLAG_NO_CHAIN) return nullptr;
        Arena* next_chunk = nullptr;

#if ARENA_FREE_LIST
        {
            Arena* prev_block = nullptr;
            for (Arena* block = free_last; block != nullptr; prev_block = block, block = block->prev) {
                u64 block_aligned_pos = ALIGN_POW2(block->pos, align);
                if (block->res >= block_aligned_pos + size) {
                    if (prev_block != nullptr) prev_block->prev = block->prev;
                    else free_last = block->prev;
                    next_chunk = block;
                    break;
                }
            }
        }
#endif

        if (next_chunk == nullptr) {
            u64 needed_size = ALIGN_POW2(ARENA_HEADER_SIZE, align) + size;
            u64 new_reserve = ALIGN_POW2(needed_size, curr->res_size);
            u64 new_commit  = ALIGN_POW2(needed_size, curr->cmt_size);
            ArenaParams next_params{ .reserve_size = new_reserve, .commit_size  = new_commit, .flags = flags, .name = name };
            next_chunk = ArenaAlloc(next_params);
            if (next_chunk == nullptr) return nullptr;
        }

        next_chunk->base_pos = curr->base_pos + curr->res;
        next_chunk->prev     = curr;
        current              = next_chunk;
        curr = next_chunk;
        aligned_pos = ALIGN_POW2(curr->pos, align);
        next_pos    = aligned_pos + size;
    }

    if (next_pos > curr->cmt) {
        if (curr->flags & ARENA_FLAG_IS_BACKING_BUFFER) return nullptr;
        u64 page_size = OS_GetPageSize();
        u64 needed_commit = ALIGN_POW2(next_pos, curr->cmt_size);
        needed_commit = ALIGN_POW2(needed_commit, page_size);
        if (needed_commit > curr->res) needed_commit = curr->res;
        u64 commit_req = needed_commit - curr->cmt;
        b32 commit_ok = OS_Commit(reinterpret_cast<u8*>(curr) + curr->cmt, commit_req);
        if (!commit_ok) return nullptr;
        curr->cmt = needed_commit;
    }

    void* result = reinterpret_cast<u8*>(curr) + aligned_pos;
    AsanUnpoisonMemoryRegion(result, size);
    if (zero) SDL_memset(result, 0, size);
    curr->pos = next_pos;
    if (curr->cmt > curr->pos) AsanPoisonMemoryRegion(reinterpret_cast<u8*>(curr) + curr->pos, curr->cmt - curr->pos);
    return result;
}

inline u64 Arena::Pos() const {
    if (!this || !current) return 0;
    return current->base_pos + current->pos;
}

inline void Arena::PopTo(u64 target_pos) {
    if (!this) return;
    if (target_pos > Pos()) return;
    Arena* curr = current;
    while (curr != nullptr && target_pos < curr->base_pos) {
        Arena* prev_block = curr->prev;
#if ARENA_FREE_LIST
        curr->pos = ARENA_HEADER_SIZE;
        curr->prev = free_last;
        free_last = curr;
        AsanPoisonMemoryRegion(reinterpret_cast<u8*>(curr) + ARENA_HEADER_SIZE, curr->res - ARENA_HEADER_SIZE);
#else
        AsanUnpoisonMemoryRegion(curr, curr->res);
        if (!(curr->flags & ARENA_FLAG_IS_BACKING_BUFFER)) OS_Release(curr, curr->res);
#endif
        curr = prev_block;
    }
    if (curr != nullptr) {
        current = curr;
        u64 local_pos = target_pos - curr->base_pos;
        if (local_pos < ARENA_HEADER_SIZE) local_pos = ARENA_HEADER_SIZE;
        if (local_pos < curr->pos) AsanPoisonMemoryRegion(reinterpret_cast<u8*>(curr) + local_pos, curr->pos - local_pos);
        curr->pos = local_pos;
    }
}

inline void Arena::Pop(u64 amt) {
    if (!this) return;
    u64 current_pos = Pos();
    if (amt <= current_pos) PopTo(current_pos - amt);
}

inline void Arena::Clear() {
    if (!this) return;
    PopTo(ARENA_HEADER_SIZE);
}

inline void Arena::Release() {
    if (!this) return;
#if ARENA_FREE_LIST
    for (Arena* free_block = free_last; free_block != nullptr;) {
        Arena* prev_block = free_block->prev;
        b32 is_backing = (free_block->flags & ARENA_FLAG_IS_BACKING_BUFFER);
        u64 block_res  = free_block->res;
        AsanUnpoisonMemoryRegion(free_block, block_res);
        if (!is_backing) OS_Release(free_block, block_res);
        free_block = prev_block;
    }
    free_last = nullptr;
#endif
    Arena* active_block = current;
    current = nullptr;
    while (active_block != nullptr) {
        Arena* prev_block = active_block->prev;
        b32 is_backing = (active_block->flags & ARENA_FLAG_IS_BACKING_BUFFER);
        u64 block_res  = active_block->res;
        AsanUnpoisonMemoryRegion(active_block, block_res);
        if (!is_backing) OS_Release(active_block, block_res);
        active_block = prev_block;
    }
}

class Vec2 {
public:
    f32 X;
    f32 Y;

    Vec2() : X(0.0f), Y(0.0f) {}
    Vec2(f32 x, f32 y) : X(x), Y(y) {}

    inline f32& Width() { return X; }
    inline const f32& Width() const { return X; }
    inline f32& Height() { return Y; }
    inline const f32& Height() const { return Y; }

    inline f32& operator[](i32 index) { return (&X)[index]; }
    inline const f32& operator[](i32 index) const { return (&X)[index]; }

    inline Vec2 Add(Vec2 b) const { return Vec2(X + b.X, Y + b.Y); }
    inline Vec2 Sub(Vec2 b) const { return Vec2(X - b.X, Y - b.Y); }
    inline Vec2 Scale(f32 s) const { return Vec2(X * s, Y * s); }
    inline f32 Length() const { return SDL_sqrtf(X * X + Y * Y); }
    inline Vec2 Normalize() const {
        f32 len = Length();
        if (len < 0.0001f) return Vec2(0.0f, 0.0f);
        return Scale(1.0f / len);
    }
};