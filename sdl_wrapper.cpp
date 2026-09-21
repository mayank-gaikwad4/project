#pragma once
#include <SDL3/SDL.h>
#include <source_location>

constexpr bool DebugEnabled = true;

namespace sdl {
    inline int snprintf(char* text, size_t maxlen, const char* fmt, ...) {
        va_list ap; va_start(ap, fmt); int r = SDL_vsnprintf(text, maxlen, fmt, ap); va_end(ap); return r;
    }
    inline void log(const char* fmt, ...) {
        if constexpr (DebugEnabled) {
            va_list ap; va_start(ap, fmt); SDL_LogMessageV(SDL_LOG_CATEGORY_APPLICATION, SDL_LOG_PRIORITY_INFO, fmt, ap); va_end(ap);
        }
    }
    inline const char* get_error() { return SDL_GetError(); }
    inline u64 str_to_u64(const char* str, char** endptr, int base) { return SDL_strtoull(str, endptr, base); }
    inline u32 get_mouse_state(f32* x, f32* y) { return SDL_GetMouseState(x, y); }
    inline void* memset(void* dst, int c, size_t len) { return SDL_memset(dst, c, len); }
    inline char* strstr(const char* haystack, const char* needle) { return SDL_strstr(haystack, needle); }
    
    inline f32 sqrtf(f32 x) { return SDL_sqrtf(x); }
    inline f32 sinf(f32 x) { return SDL_sinf(x); }
    inline f32 cosf(f32 x) { return SDL_cosf(x); }
    inline f32 atan2f(f32 y, f32 x) { return SDL_atan2f(y, x); }
    inline f32 expf(f32 x) { return SDL_expf(x); }
    inline f32 powf(f32 x, f32 y) { return SDL_powf(x, y); }
    inline f32 floorf(f32 x) { return SDL_floorf(x); }
    inline f32 lerp(f32 a, f32 b, f32 t) { return a + (b - a) * t; }

    inline f32 sin_turns(f32 turns) { return SDL_sinf(TurnsToRadians(turns)); }
    inline f32 cos_turns(f32 turns) { return SDL_cosf(TurnsToRadians(turns)); }
    inline f32 atan2_turns(f32 rise, f32 run) { return RadiansToTurns(SDL_atan2f(rise, run)); }

    constexpr f32 wrap_turns_signed(f32 turns) {
        while (turns >  TurnHalf) turns -= TurnFull;
        while (turns < -TurnHalf) turns += TurnFull;
        return turns;
    }

    inline f32 lerp_exp(f32 from, f32 to, f32 decay, f32 delta_seconds) {
        return to + (from - to) * SDL_expf(-decay * delta_seconds);
    }

    inline i32 rand_below(u64* state, i32 count) { return SDL_rand_r(state, count); }
    inline f32 rand01(u64* state) { return SDL_randf_r(state); }

    inline u64 now_ns() {
        u64 counter = SDL_GetPerformanceCounter();
        u64 frequency = SDL_GetPerformanceFrequency();
        return (counter / frequency) * 1000000000ull + ((counter % frequency) * 1000000000ull) / frequency;
    }

    class Platform {
    public:
        b32 Init() { return SDL_Init(SDL_INIT_VIDEO); }
        void Quit() { SDL_Quit(); }
    };

    class Window {
    public:
        SDL_Window* handle;
        b32 Create(const c8* title, i32 width, i32 height, u64 flags) {
            handle = SDL_CreateWindow(title, width, height, flags);
            return handle != nullptr;
        }
        void Destroy() { if (handle) SDL_DestroyWindow(handle); handle = nullptr; }
        void GetSize(i32* width, i32* height) { SDL_GetWindowSize(handle, width, height); }
        f32 PixelDensity() { f32 density = SDL_GetWindowPixelDensity(handle); return density > 0.0f ? density : 1.0f; }
        f32 RefreshRate() {
            const SDL_DisplayMode* mode = SDL_GetCurrentDisplayMode(SDL_GetDisplayForWindow(handle));
            return (mode && mode->refresh_rate > 1.0f) ? mode->refresh_rate : 60.0f;
        }
    };

    class Renderer {
    public:
        SDL_Renderer* handle;
        b32 Create(Window* window) {
            handle = SDL_CreateRenderer(window->handle, nullptr);
            if (handle) SDL_SetRenderVSync(handle, 0);
            return handle != nullptr;
        }
        void Destroy() { if (handle) SDL_DestroyRenderer(handle); handle = nullptr; }
    };

    class RenderTarget {
    public:
        SDL_Texture* handle;
        i32 texel_width, texel_height;
        i32 logical_width, logical_height;
        f32 density;

        void Destroy() { if (handle) SDL_DestroyTexture(handle); handle = nullptr; }
        void Ensure(Renderer* renderer, Window* window, i32 supersample) {
            i32 width, height;
            window->GetSize(&width, &height);
            if (width <= 0 || height <= 0) return;
            f32 window_density = window->PixelDensity();
            if (handle && width == logical_width && height == logical_height && window_density == density) return;

            Destroy();
            logical_width = width; logical_height = height; density = window_density;
            texel_width  = SDL_max(1, (i32)(width  * window_density * supersample));
            texel_height = SDL_max(1, (i32)(height * window_density * supersample));
            handle = SDL_CreateTexture(renderer->handle, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, texel_width, texel_height);
            if (handle) {
                SDL_SetTextureScaleMode(handle, SDL_SCALEMODE_LINEAR);
                SDL_SetTextureBlendMode(handle, SDL_BLENDMODE_BLEND);
            }
        }
    };
}