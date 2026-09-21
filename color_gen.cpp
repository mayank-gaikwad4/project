#pragma once

namespace color_gen {
    struct Oklab { f32 L, a, b; };
    struct Oklch { f32 L, C, H; };
    struct Rgb { u8 r, g, b; };

    Oklab history_oklab_colors[1024];
    u32 history_count = 0;

    f32 random_float(u64* state, f32 min_val, f32 max_val) {
        return min_val + sdl::rand01(state) * (max_val - min_val);
    }

    Oklch sample_vibrant_oklch(u64* state) {
        Oklch c;
        c.L = random_float(state, 0.60f, 0.80f);
        c.C = random_float(state, 0.18f, 0.26f);
        c.H = random_float(state, 0.0f, 360.0f);
        return c;
    }

    Oklab oklch_to_oklab(Oklch oklch) {
        Oklab c;
        c.L = oklch.L;
        f32 rad = oklch.H * (3.1415926535f / 180.0f);
        c.a = oklch.C * sdl::cosf(rad);
        c.b = oklch.C * sdl::sinf(rad);
        return c;
    }

    f32 perceptual_distance_oklab(Oklab c1, Oklab c2) {
        f32 dL = c1.L - c2.L;
        f32 da = c1.a - c2.a;
        f32 db = c1.b - c2.b;
        return sdl::sqrtf(dL * dL + da * da + db * db);
    }

    f32 gamma_correct(f32 val) {
        if (val <= 0.0031308f) return 12.92f * val;
        return 1.055f * sdl::powf(val, 1.0f / 2.4f) - 0.055f;
    }

    f32 clamp(f32 val, f32 min_val, f32 max_val) {
        if (val < min_val) return min_val;
        if (val > max_val) return max_val;
        return val;
    }

    Rgb oklab_to_rgb(Oklab oklab) {
        f32 l_ = oklab.L + 0.3963377774f * oklab.a + 0.2158037573f * oklab.b;
        f32 m_ = oklab.L - 0.1055613458f * oklab.a - 0.0638541728f * oklab.b;
        f32 s_ = oklab.L - 0.0894841775f * oklab.a - 1.2914855480f * oklab.b;

        f32 l = l_ * l_ * l_;
        f32 m = m_ * m_ * m_;
        f32 s = s_ * s_ * s_;

        f32 r_linear = +4.0767416621f * l - 3.3077115913f * m + 0.2309699292f * s;
        f32 g_linear = -1.2684380046f * l + 2.6097574011f * m - 0.3413193965f * s;
        f32 b_linear = -0.0041960863f * l - 0.7034186147f * m + 1.7076147010f * s;

        Rgb rgb;
        rgb.r = (u8)(255.0f * clamp(gamma_correct(r_linear), 0.0f, 1.0f));
        rgb.g = (u8)(255.0f * clamp(gamma_correct(g_linear), 0.0f, 1.0f));
        rgb.b = (u8)(255.0f * clamp(gamma_correct(b_linear), 0.0f, 1.0f));
        return rgb;
    }

    Rgb get_next_distinct_color(u64* state, i32 num_candidates = 50) {
        if (history_count == 0) {
            Oklch best_oklch = sample_vibrant_oklch(state);
            Oklab best_oklab = oklch_to_oklab(best_oklch);
            history_oklab_colors[history_count++] = best_oklab;
            return oklab_to_rgb(best_oklab);
        }

        Oklab best_candidate_oklab = {0,0,0};
        f32 max_min_distance = -1.0f;

        for (i32 i = 0; i < num_candidates; ++i) {
            Oklch candidate_oklch = sample_vibrant_oklch(state);
            Oklab candidate_oklab = oklch_to_oklab(candidate_oklch);

            f32 min_distance = 999999.0f;
            for (u32 j = 0; j < history_count; ++j) {
                f32 dist = perceptual_distance_oklab(candidate_oklab, history_oklab_colors[j]);
                if (dist < min_distance) min_distance = dist;
            }

            if (min_distance > max_min_distance) {
                max_min_distance = min_distance;
                best_candidate_oklab = candidate_oklab;
            }
        }

        if (history_count < 1024) history_oklab_colors[history_count++] = best_candidate_oklab;
        return oklab_to_rgb(best_candidate_oklab);
    }
}