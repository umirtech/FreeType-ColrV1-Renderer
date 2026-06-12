//
// Created by MOHDU on 26-12-2025.
//

#ifndef MEDIAENGINE_COLRV1RENDERER_H
#define MEDIAENGINE_COLRV1RENDERER_H
#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "OutlineDecomposer.h"
#include "Triangulator.h"
#include <ft2build.h>
#include FT_COLOR_H
#include FT_OUTLINE_H

namespace ColrV1 {

    constexpr float FT_TO_PX = 1.0f / 64.0f;
    constexpr float FT_FIXED_TO_FLOAT = 1.0f / 65536.0f;
    constexpr float FT_F2DOT14_TO_FLOAT = 1.0f / 16384.0f;

    enum class PaintKind : int {
        Solid = 0,
        LinearGradient = 1,
        RadialGradient = 2,
        SweepGradient = 3,
    };

    struct ColorStop {
        float offset = 0.0f;
        glm::vec4 color{1.0f};
    };

    struct PaintData {
        PaintKind kind = PaintKind::Solid;
        glm::vec4 solid{1.0f};

        glm::vec2 p0{0.0f};
        glm::vec2 p1{0.0f};
        glm::vec2 p2{0.0f};
        float r0 = 0.0f;
        float r1 = 0.0f;
        float a0 = 0.0f;
        float a1 = 0.0f;

        glm::mat4 sample_transform{1.0f};

        std::vector<ColorStop> stops;
    };

    struct DrawCommand {
        glm::mat4 geometry_transform{1.0f};
        PaintData paint{};
        std::vector<Vertex> vertices;
        std::vector<uint32_t> indices;
        FT_Composite_Mode composite_mode = FT_COLR_COMPOSITE_SRC_OVER;
    };

    namespace detail {

        inline float fixed16_16_to_float(FT_Fixed value) { return static_cast<float>(value) * FT_FIXED_TO_FLOAT; }

        inline glm::vec4 to_rgba(const FT_Color& c, float alpha_mul = 1.0f) {
            constexpr float kInv255 = 1.0f / 255.0f;
            return {static_cast<float>(c.red) * kInv255,
                    static_cast<float>(c.green) * kInv255,
                    static_cast<float>(c.blue) * kInv255,
                    static_cast<float>(c.alpha) * kInv255 * alpha_mul};
        }



        inline float& mat(glm::mat4& m, int col, int row){return m[col][row];}

        inline const float& mat(const glm::mat4& m, int col, int row){return m[col][row];}

        inline glm::vec2 transform_point(const glm::mat4& m,glm::vec2 p)
        {
            glm::vec4 r = m * glm::vec4(p, 1.0f,1.0f);
            return {r.x, r.y};
        }

        inline float fixed_font_units_to_px(FT_Face face, FT_Fixed value, bool y_axis) {
            if (!face || !face->size) return fixed16_16_to_float(value) * FT_TO_PX;
            const FT_Fixed scale = y_axis ? face->size->metrics.y_scale : face->size->metrics.x_scale;
            return static_cast<float>(FT_MulFix(value, scale)) * (FT_FIXED_TO_FLOAT * FT_TO_PX);
        }

        inline float radius_to_px(FT_Face face, FT_Fixed value) {
            const float x = std::abs(fixed_font_units_to_px(face, value, false));
            const float y = std::abs(fixed_font_units_to_px(face, value, true));
            return (x + y) * 0.5f;
        }

        inline glm::vec2 vec_to_px(FT_Face face, const FT_Vector& v) {
            return {fixed_font_units_to_px(face, static_cast<FT_Fixed>(v.x), false),
                    fixed_font_units_to_px(face, static_cast<FT_Fixed>(v.y), true)};
        }

        inline float angle_to_radians(float degrees_div_180) {
            return degrees_div_180 * 3.14159265358979323846f;
        }

        inline glm::mat4 translate(float tx, float ty) {
            glm::mat4 m(1.0f);
            mat(m, 2, 0) = tx;
            mat(m, 2, 1) = ty;
            return m;
        }

        inline glm::mat4 scale(float sx, float sy) {
            glm::mat4 m(1.0f);
            mat(m, 0, 0) = sx;
            mat(m, 1, 1) = sy;
            return m;
        }

        inline double edge(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& p) {
            return (p.x - a.x) * (b.y - a.y) - (p.y - a.y) * (b.x - a.x);
        }

        inline bool IsTopLeft(const glm::vec2& a,const glm::vec2& b){
            const float dx = b.x - a.x;
            const float dy = b.y - a.y;
            return (dy > 0.0f) ||(dy == 0.0f && dx < 0.0f);
        }

        inline float clamp01(float v) {
            return std::clamp(v, 0.0f, 1.0f);
        }

        inline glm::vec4 clamp_color(glm::vec4 c) {
            return {
                clamp01(c.r),
                clamp01(c.g),
                clamp01(c.b),
                clamp01(c.a)
            };
        }

        inline glm::vec4 lerp(glm::vec4 a, glm::vec4 b, float t) {
            return a * (1.0f - t) + b * t;
        }

        inline glm::vec4 sample_stops(const std::vector<ColorStop>& stops, float t, glm::vec4 fallback) {
            if (stops.empty()) return fallback;
            t = clamp01(t);
            if (t <= stops.front().offset) return stops.front().color;
            for (size_t i = 1; i < stops.size(); ++i) {
                if (t <= stops[i].offset) {
                    const float span = std::max(stops[i].offset - stops[i - 1].offset, 1e-6f);
                    return lerp(stops[i - 1].color, stops[i].color, clamp01((t - stops[i - 1].offset) / span));
                }
            }
            return stops.back().color;
        }

        inline float linear_gradient_t(const PaintData& paint, glm::vec2 p) {
            const glm::vec2 axis = paint.p1 - paint.p0;
            const float denom = glm::dot(axis, axis);
            if (denom <= 1e-12f) return 0.0f;
            return glm::dot(p - paint.p0, axis) / denom;
        }

        inline float radial_gradient_t(const PaintData& paint, glm::vec2 p) {
            const glm::vec2 dc = paint.p1 - paint.p0;
            const float dr = paint.r1 - paint.r0;
            const glm::vec2 f = paint.p0 - p;
            const float a = glm::dot(dc, dc) - dr * dr;
            const float b = 2.0f * (glm::dot(f, dc) - paint.r0 * dr);
            const float c = glm::dot(f, f) - paint.r0 * paint.r0;
            if (std::abs(a) < 1e-6f) return std::abs(b) > 1e-6f ? -c / b : 0.0f;
            const float disc = b * b - 4.0f * a * c;
            if (disc < 0.0f) return 0.0f;
            const float root = std::sqrt(disc);
            const float t0 = (-b - root) / (2.0f * a);
            const float t1 = (-b + root) / (2.0f * a);
            if (t0 >= 0.0f && t0 <= 1.0f) return t0;
            return t1;
        }

        inline float sweep_gradient_t(const PaintData& paint, glm::vec2 p) {
            constexpr float kPi = 3.14159265358979323846f;
            float angle = std::atan2(-(p.x - paint.p0.x), p.y - paint.p0.y) / kPi;
            if (angle < 0.0f) angle += 2.0f;
            float start = paint.a0;
            float end = paint.a1;
            float span = end - start;
            while (span <= 0.0f) span += 2.0f;
            float rel = angle - start;
            while (rel < 0.0f) rel += 2.0f;
            return rel / span;
        }

        inline glm::vec4 sample_paint(const glm::mat4 transform,const PaintData& paint, glm::vec2 device_p) {
            const glm::vec2 p = transform_point(glm::inverse(transform) * paint.sample_transform, device_p);
            switch (paint.kind) {
                case PaintKind::LinearGradient:
                    return sample_stops(paint.stops, linear_gradient_t(paint, p), paint.solid);
                case PaintKind::RadialGradient:
                    return sample_stops(paint.stops, radial_gradient_t(paint, p), paint.solid);
                case PaintKind::SweepGradient:
                    return sample_stops(paint.stops, sweep_gradient_t(paint, p), paint.solid);
                case PaintKind::Solid:
                default:
                    return paint.solid;
            }
        }

        inline glm::vec4 premul(glm::vec4 c) { return {c.r * c.a, c.g * c.a, c.b * c.a, c.a}; }

        inline glm::vec4 unpremul(glm::vec4 c) {
            if (c.a <= 1e-6f) return {0.0f, 0.0f, 0.0f, 0.0f};
            return {c.r / c.a, c.g / c.a, c.b / c.a, c.a};
        }

        inline float blend_channel(float s, float d, FT_Composite_Mode mode) {
            switch (mode) {
                case FT_COLR_COMPOSITE_MULTIPLY: return s * d;
                case FT_COLR_COMPOSITE_SCREEN: return s + d - s * d;
                case FT_COLR_COMPOSITE_OVERLAY: return d <= 0.5f ? 2.0f * s * d : 1.0f - 2.0f * (1.0f - s) * (1.0f - d);
                case FT_COLR_COMPOSITE_DARKEN: return std::min(s, d);
                case FT_COLR_COMPOSITE_LIGHTEN: return std::max(s, d);
                case FT_COLR_COMPOSITE_COLOR_DODGE: return s >= 1.0f ? 1.0f : std::min(1.0f, d / (1.0f - s));
                case FT_COLR_COMPOSITE_COLOR_BURN: return s <= 0.0f ? 0.0f : 1.0f - std::min(1.0f, (1.0f - d) / s);
                case FT_COLR_COMPOSITE_HARD_LIGHT: return s <= 0.5f ? 2.0f * s * d : 1.0f - 2.0f * (1.0f - s) * (1.0f - d);
                case FT_COLR_COMPOSITE_SOFT_LIGHT: return (1.0f - 2.0f * s) * d * d + 2.0f * s * d;
                case FT_COLR_COMPOSITE_DIFFERENCE: return std::abs(d - s);
                case FT_COLR_COMPOSITE_EXCLUSION: return s + d - 2.0f * s * d;
                default: return s;
            }
        }

        inline glm::vec4 blend_source_over(glm::vec4 src, glm::vec4 dst, FT_Composite_Mode mode) {
            const float out_a = src.a + dst.a * (1.0f - src.a);
            if (out_a <= 1e-6f) return {0.0f, 0.0f, 0.0f, 0.0f};
            const float br = blend_channel(src.r, dst.r, mode);
            const float bg = blend_channel(src.g, dst.g, mode);
            const float bb = blend_channel(src.b, dst.b, mode);
            return clamp_color({(br * src.a * dst.a + src.r * src.a * (1.0f - dst.a) + dst.r * dst.a * (1.0f - src.a)) / out_a,
                                (bg * src.a * dst.a + src.g * src.a * (1.0f - dst.a) + dst.g * dst.a * (1.0f - src.a)) / out_a,
                                (bb * src.a * dst.a + src.b * src.a * (1.0f - dst.a) + dst.b * dst.a * (1.0f - src.a)) / out_a,
                                out_a});
        }

        inline glm::vec4 composite(glm::vec4 src, glm::vec4 dst, FT_Composite_Mode mode) {
            src = clamp_color(src);
            dst = clamp_color(dst);
            switch (mode) {
                case FT_COLR_COMPOSITE_CLEAR: return {0.0f, 0.0f, 0.0f, 0.0f};
                case FT_COLR_COMPOSITE_SRC: return src;
                case FT_COLR_COMPOSITE_DEST: return dst;
                case FT_COLR_COMPOSITE_DEST_OVER: return unpremul(premul(dst) + premul(src) * (1.0f - dst.a));
                case FT_COLR_COMPOSITE_SRC_IN: return {src.r, src.g, src.b, src.a * dst.a};
                case FT_COLR_COMPOSITE_DEST_IN: return {dst.r, dst.g, dst.b, dst.a * src.a};
                case FT_COLR_COMPOSITE_SRC_OUT: return {src.r, src.g, src.b, src.a * (1.0f - dst.a)};
                case FT_COLR_COMPOSITE_DEST_OUT: return {dst.r, dst.g, dst.b, dst.a * (1.0f - src.a)};
                case FT_COLR_COMPOSITE_SRC_ATOP: return unpremul(premul(src) * dst.a + premul(dst) * (1.0f - src.a));
                case FT_COLR_COMPOSITE_DEST_ATOP: return unpremul(premul(dst) * src.a + premul(src) * (1.0f - dst.a));
                case FT_COLR_COMPOSITE_XOR: return unpremul(premul(src) * (1.0f - dst.a) + premul(dst) * (1.0f - src.a));
                case FT_COLR_COMPOSITE_PLUS: return clamp_color(unpremul(premul(src) + premul(dst)));
                case FT_COLR_COMPOSITE_MULTIPLY:
                case FT_COLR_COMPOSITE_SCREEN:
                case FT_COLR_COMPOSITE_OVERLAY:
                case FT_COLR_COMPOSITE_DARKEN:
                case FT_COLR_COMPOSITE_LIGHTEN:
                case FT_COLR_COMPOSITE_COLOR_DODGE:
                case FT_COLR_COMPOSITE_COLOR_BURN:
                case FT_COLR_COMPOSITE_HARD_LIGHT:
                case FT_COLR_COMPOSITE_SOFT_LIGHT:
                case FT_COLR_COMPOSITE_DIFFERENCE:
                case FT_COLR_COMPOSITE_EXCLUSION:
                    return blend_source_over(src, dst, mode);
                case FT_COLR_COMPOSITE_SRC_OVER:
                default:
                    return unpremul(premul(src) + premul(dst) * (1.0f - src.a));
            }
        }

        inline std::array<uint8_t, 4> pack_bgra8(glm::vec4 c)
        {
            c = clamp_color(c);

            const auto q = [](float v)
            {
                return static_cast<uint8_t>(
                        std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f));
            };

            return {
                    q(c.b),   // B
                    q(c.g),   // G
                    q(c.r),   // R
                    q(c.a)    // A
            };
        }

        inline void write_bgra8(uint8_t* dst, glm::vec4 c)
        {
            auto bgra = pack_bgra8(c);

            dst[0] = bgra[0];
            dst[1] = bgra[1];
            dst[2] = bgra[2];
            dst[3] = bgra[3];
        }

        inline void fill_bgra8(std::vector<uint8_t>& dst, glm::vec4 c)
        {
            auto bgra = pack_bgra8(c);

            for (size_t i = 0; i + 3 < dst.size(); i += 4)
            {
                dst[i + 0] = bgra[0];
                dst[i + 1] = bgra[1];
                dst[i + 2] = bgra[2];
                dst[i + 3] = bgra[3];
            }
        }

    }

    class CommandBuilder {
    public:
        static std::vector<DrawCommand> BuildForGlyph(FT_Face face, FT_UInt glyph_index, FT_UShort palette_index = 0) {
            std::vector<DrawCommand> out;
            if (!face) return out;

            FT_Color* palette = nullptr;
            FT_Palette_Select(face, palette_index, &palette);

            FT_OpaquePaint root{};
            // Outlines are decomposed after FT_Load_Glyph at the active size, so they are already
            // in pixel space. Do not request FreeType's root transform or coordinates get scaled twice.
            if (!FT_Get_Color_Glyph_Paint(face, glyph_index, FT_COLOR_NO_ROOT_TRANSFORM, &root)) return out;

            RenderState state{};
            TraversePaint(face, root, state, palette, out);
            return out;
        }

    private:
        struct RenderState {
            glm::mat4 geometry_transform{1.0f};
            glm::mat4 paint_transform{1.0f};

            PaintData paint{};
            FT_Composite_Mode composite = FT_COLR_COMPOSITE_SRC_OVER;
            bool has_clip_glyph = false;
            FT_UInt clip_glyph = 0;
        };

        static glm::mat4 Translate(float tx, float ty) { return detail::translate(tx, ty); }

        static glm::mat4 Scale(float sx, float sy) { return detail::scale(sx, sy); }

        static glm::mat4 Rotate(float angle_degrees_div_180) {
            glm::mat4 m(1.0f);
            const float angle_radians = detail::angle_to_radians(angle_degrees_div_180);
            const float c = std::cos(angle_radians);
            const float s = std::sin(angle_radians);
            detail::mat(m, 0, 0) = c;
            detail::mat(m, 1, 0) = -s;
            detail::mat(m, 0, 1) = s;
            detail::mat(m, 1, 1) = c;
            return m;
        }

        static glm::mat4 Skew(float ax_degrees_div_180, float ay_degrees_div_180) {
            glm::mat4 m(1.0f);
            detail::mat(m, 1, 0) = std::tan(detail::angle_to_radians(ax_degrees_div_180));
            detail::mat(m, 0, 1) = std::tan(detail::angle_to_radians(ay_degrees_div_180));
            return m;
        }

        static glm::mat4 Affine23ToMat4(FT_Face face, const FT_Affine23& a)
        {
            glm::mat4 m(1.0f);

            m[0][0] = detail::fixed16_16_to_float(a.xx);
            m[0][1] = detail::fixed16_16_to_float(a.yx);

            m[1][0] = detail::fixed16_16_to_float(a.xy);
            m[1][1] = detail::fixed16_16_to_float(a.yy);

            m[3][0] = detail::fixed_font_units_to_px(face, a.dx, false);
            m[3][1] = detail::fixed_font_units_to_px(face, a.dy, true);

            return m;
        }

        static void LoadStops(FT_Face face, FT_ColorLine colorline, FT_Color* palette, std::vector<ColorStop>& dst) {
            dst.clear();
            FT_ColorStopIterator it = colorline.color_stop_iterator;
            FT_ColorStop stop{};
            while (FT_Get_Colorline_Stops(face, &stop, &it)) {
                const FT_Color c = palette ? palette[stop.color.palette_index] : FT_Color{255, 255, 255, 255};
                dst.push_back({static_cast<float>(stop.stop_offset) / 65535.0f,
                               detail::to_rgba(c, static_cast<float>(stop.color.alpha) * FT_F2DOT14_TO_FLOAT)});
            }
            std::sort(dst.begin(), dst.end(), [](const ColorStop& a, const ColorStop& b) { return a.offset < b.offset; });
        }

        static PaintData ResolvePaintNode(FT_Face face, const FT_COLR_Paint& p, FT_Color* palette, const PaintData& fallback) {
            PaintData paint = fallback;

            switch (p.format) {
                case FT_COLR_PAINTFORMAT_SOLID: {
                    paint.kind = PaintKind::Solid;
                    const auto& solid = p.u.solid;
                    const FT_Color c = palette ? palette[solid.color.palette_index] : FT_Color{255, 255, 255, 255};
                    paint.solid = detail::to_rgba(c,static_cast<float>(solid.color.alpha) * FT_F2DOT14_TO_FLOAT);
                    break;
                }
                case FT_COLR_PAINTFORMAT_LINEAR_GRADIENT: {
                    paint.kind = PaintKind::LinearGradient;
                    const auto& g = p.u.linear_gradient;
                    paint.p0 = detail::vec_to_px(face, g.p0);
                    paint.p1 = detail::vec_to_px(face, g.p1);
                    paint.p2 = detail::vec_to_px(face, g.p2);
                    LoadStops(face, g.colorline, palette, paint.stops);
                    break;
                }
                case FT_COLR_PAINTFORMAT_RADIAL_GRADIENT: {
                    paint.kind = PaintKind::RadialGradient;
                    const auto& g = p.u.radial_gradient;
                    paint.p0 = detail::vec_to_px(face, g.c0);
                    paint.p1 = detail::vec_to_px(face, g.c1);
                    paint.r0 = detail::radius_to_px(face, g.r0);
                    paint.r1 = detail::radius_to_px(face, g.r1);
                    LoadStops(face, g.colorline, palette, paint.stops);
                    break;
                }
                case FT_COLR_PAINTFORMAT_SWEEP_GRADIENT: {
                    paint.kind = PaintKind::SweepGradient;
                    const auto& g = p.u.sweep_gradient;
                    paint.p0 = detail::vec_to_px(face, g.center);
                    paint.a0 = detail::fixed16_16_to_float(g.start_angle);
                    paint.a1 = detail::fixed16_16_to_float(g.end_angle);
                    LoadStops(face, g.colorline, palette, paint.stops);
                    break;
                }
                default:
                    break;
            }
            return paint;
        }

        static void EmitGlyph(FT_Face face, FT_UInt glyph_id, const RenderState& state, std::vector<DrawCommand>& out) {

            if (FT_Load_Glyph(face, glyph_id, FT_LOAD_NO_BITMAP | FT_LOAD_NO_HINTING) != 0) return;
            if (face->glyph->format != FT_GLYPH_FORMAT_OUTLINE) return;

            DrawCommand cmd;
            Triangulator triangulator(cmd.vertices, cmd.indices);
            OutlineDecomposer::decompose(face->glyph->outline,triangulator);

            if (cmd.indices.empty()) return;

            cmd.geometry_transform = state.geometry_transform;
            cmd.paint = state.paint;
            cmd.paint.sample_transform = glm::inverse(state.paint_transform);
            cmd.composite_mode = state.composite;
            out.push_back(std::move(cmd));
        }

        static void TraversePaint(FT_Face face, FT_OpaquePaint node, const RenderState& state, FT_Color* palette,
                                  std::vector<DrawCommand>& out) {
            FT_COLR_Paint p{};
            if (!FT_Get_Paint(face, node, &p)) return;

            switch (p.format) {
                case FT_COLR_PAINTFORMAT_SOLID:
                case FT_COLR_PAINTFORMAT_LINEAR_GRADIENT:
                case FT_COLR_PAINTFORMAT_RADIAL_GRADIENT:
                case FT_COLR_PAINTFORMAT_SWEEP_GRADIENT: {
                    if (!state.has_clip_glyph) break;

                    RenderState next = state;
                    next.paint = ResolvePaintNode(face, p, palette, state.paint);
                    EmitGlyph(face, next.clip_glyph, next, out);
                    break;
                }
                case FT_COLR_PAINTFORMAT_COLR_LAYERS: {
                    FT_LayerIterator it = p.u.colr_layers.layer_iterator;
                    FT_OpaquePaint child{};
                    while (FT_Get_Paint_Layers(face, &it, &child)){
                        RenderState next;
                        TraversePaint(face, child, next, palette, out);
                    }
                    break;
                }
                case FT_COLR_PAINTFORMAT_GLYPH: {
                    RenderState next = state;
                    next.has_clip_glyph = true;
                    next.clip_glyph = p.u.glyph.glyphID;
                    TraversePaint(face, p.u.glyph.paint, next, palette, out);
                    break;
                }
                case FT_COLR_PAINTFORMAT_COLR_GLYPH: {
                    FT_OpaquePaint glyph_root{};
                    if (FT_Get_Color_Glyph_Paint(face, p.u.colr_glyph.glyphID, FT_COLOR_NO_ROOT_TRANSFORM, &glyph_root)) {
                        TraversePaint(face, glyph_root, state, palette, out);
                    }
                    break;
                }
                case FT_COLR_PAINTFORMAT_TRANSFORM: {
                    //logI("COLRV1","TRANSFORM");
                    RenderState next = state;

                    if (!next.has_clip_glyph)
                        next.geometry_transform = next.geometry_transform * Affine23ToMat4(face, p.u.transform.affine);

                    next.paint_transform = next.paint_transform * Affine23ToMat4(face,p.u.transform.affine);

                    TraversePaint(face,p.u.transform.paint,next,palette,out);
                    break;
                }
                case FT_COLR_PAINTFORMAT_TRANSLATE: {
                    //logI("COLRV1","TRANSLATE");
                    RenderState next = state;

                    if (!next.has_clip_glyph) {
                        next.geometry_transform = state.geometry_transform * Translate(
                                detail::fixed_font_units_to_px(face, p.u.translate.dx, false),
                                detail::fixed_font_units_to_px(face, p.u.translate.dy, true));
                    }

                    next.paint_transform = state.paint_transform * Translate(
                            detail::fixed_font_units_to_px(face, p.u.translate.dx, false),
                            detail::fixed_font_units_to_px(face, p.u.translate.dy, true));

                    TraversePaint(face, p.u.translate.paint, next, palette, out);
                    break;
                }
                case FT_COLR_PAINTFORMAT_SCALE: {
                    //logI("COLRV1","SCALE");
                    RenderState next = state;
                    const float cx = detail::fixed_font_units_to_px(face, p.u.scale.center_x, false);
                    const float cy = detail::fixed_font_units_to_px(face, p.u.scale.center_y, true);

                    if (!next.has_clip_glyph) {
                        next.geometry_transform = state.geometry_transform * Translate(
                                cx, cy) *
                                Scale(detail::fixed16_16_to_float(p.u.scale.scale_x),
                                      detail::fixed16_16_to_float(p.u.scale.scale_y)) *
                                      Translate(-cx, -cy);
                    }

                    next.paint_transform = state.paint_transform * Translate(
                            cx, cy) *
                              Scale(detail::fixed16_16_to_float(p.u.scale.scale_x),
                                    detail::fixed16_16_to_float(p.u.scale.scale_y)) *
                                    Translate(-cx, -cy);

                    TraversePaint(face, p.u.scale.paint, next, palette, out);
                    break;
                }
                case FT_COLR_PAINTFORMAT_ROTATE: {
                    //logI("COLRV1","ROTATE");
                    RenderState next = state;
                    const float cx = detail::fixed_font_units_to_px(face, p.u.rotate.center_x, false);
                    const float cy = detail::fixed_font_units_to_px(face, p.u.rotate.center_y, true);

                    if (!next.has_clip_glyph) {
                        next.geometry_transform =
                                state.geometry_transform * Translate(cx, cy) *
                                Rotate(detail::fixed16_16_to_float(p.u.rotate.angle)) *
                                Translate(-cx, -cy);
                    }

                    next.paint_transform =
                            state.paint_transform * Translate(cx, cy) *
                            Rotate(detail::fixed16_16_to_float(p.u.rotate.angle)) *
                            Translate(-cx, -cy);

                    TraversePaint(face, p.u.rotate.paint, next, palette, out);
                    break;
                }
                case FT_COLR_PAINTFORMAT_SKEW: {
                    //logI("COLRV1","SKEW");
                    RenderState next = state;
                    const float cx = detail::fixed_font_units_to_px(face, p.u.skew.center_x, false);
                    const float cy = detail::fixed_font_units_to_px(face, p.u.skew.center_y, true);

                    if (!next.has_clip_glyph) {
                        next.geometry_transform =
                                state.geometry_transform * Translate(cx, cy) *
                                Skew(detail::fixed16_16_to_float(p.u.skew.x_skew_angle),
                                     detail::fixed16_16_to_float(p.u.skew.y_skew_angle)) *
                                Translate(-cx, -cy);
                    }

                    next.paint_transform =
                            state.paint_transform * Translate(cx, cy) *
                            Skew(detail::fixed16_16_to_float(p.u.skew.x_skew_angle),
                                 detail::fixed16_16_to_float(p.u.skew.y_skew_angle)) *
                            Translate(-cx, -cy);

                    TraversePaint(face, p.u.skew.paint, next, palette, out);
                    break;
                }
                case FT_COLR_PAINTFORMAT_COMPOSITE: {
                    const auto& c = p.u.composite;
                    RenderState backdrop_state = state;
                    backdrop_state.composite = FT_COLR_COMPOSITE_SRC_OVER;
                    TraversePaint(face, c.backdrop_paint, backdrop_state, palette, out);

                    RenderState source_state = state;
                    source_state.composite = c.composite_mode;
                    TraversePaint(face, c.source_paint, source_state, palette, out);
                    break;
                }
                default:
                    logI("COLRV1","Missing Paint Format %d",p.format);
                    break;
            }
        }
    };

    class Renderer {
    public:
        struct Image {
            int width = 0;
            int height = 0;
            int xMin = 0;
            int yMin = 0;
            int xMax = 0;
            int yMax = 0;
            int bearing_x = 0;
            int bearing_y = 0;
            std::vector<uint8_t> pixels;
        };

        static Image Render(
            const std::vector<DrawCommand>& commands,
            glm::vec4 background = {0.0f, 0.0f, 0.0f, 0.0f}
        ){
            auto bBox = getBoundingBox(commands);

            auto mWidth  = bBox.maxX - bBox.minX + 1;
            auto mHeight = bBox.maxY - bBox.minY + 1;

            /*float scaleFactor = 400.0f / mHeight;

            glm::mat4 scaleMatrix = glm::scale(
                    glm::mat4(1.0f),
                    glm::vec3(
                        scaleFactor,
                        scaleFactor, 1.0f
                    )
            );*/

            glm::mat4 scaleMatrix(1.0f);

            bBox = getBoundingBox(commands,scaleMatrix);

            Image image;

            if (bBox.minX == FLT_MAX)
            {
                image.xMin = 0;
                image.yMin = 0;
                image.xMax = 0;
                image.yMax = 0;

                image.bearing_x = 0;
                image.bearing_y = 0;

                return image;
            }

            image.xMin = static_cast<int>(std::floor(bBox.minX));
            image.yMin = static_cast<int>(std::floor(bBox.minY));

            image.xMax = static_cast<int>(std::ceil(bBox.maxX));
            image.yMax = static_cast<int>(std::ceil(bBox.maxY));

            image.bearing_x = image.xMin;
            image.bearing_y = image.yMax;

            image.width  = image.xMax - image.xMin + 1;
            image.height = image.yMax - image.yMin + 1;

            image.pixels.resize(static_cast<size_t>(image.width * image.height * 4));

            detail::fill_bgra8(image.pixels, background);

            std::vector<MSAAPixel> msaaBuffer(image.width * image.height);

            for (const DrawCommand& command : commands)
            {
                RasterizeCommand(scaleMatrix,command, image,msaaBuffer);
            }

            ResolveMSAA(image,msaaBuffer);

            return image;
        }

    private:

        struct BBox{
            float minX = FLT_MAX;
            float minY = FLT_MAX;
            float maxX = -FLT_MAX;
            float maxY = -FLT_MAX;
        };

        struct Sample
        {
            glm::vec4 color = glm::vec4(0.0f);
            bool covered = false;
        };

        struct MSAAPixel
        {
            Sample samples[4];
        };


        static BBox getBoundingBox(
            const std::vector<DrawCommand>& commands,
            const glm::mat4 transform = glm::mat4(1.0f)
        ){
            BBox bBox;
            for (const DrawCommand& command : commands)
            {
                for (const auto& v : command.vertices)
                {
                    const glm::vec2 p = detail::transform_point(
                            transform * command.geometry_transform,
                            {v.position.x, v.position.y}
                    );

                    bBox.minX = std::min(bBox.minX, p.x);
                    bBox.minY = std::min(bBox.minY, p.y);
                    bBox.maxX = std::max(bBox.maxX, p.x);
                    bBox.maxY = std::max(bBox.maxY, p.y);
                }
            }

            return bBox;
        }

        static void RasterizeCommand(
            const glm::mat4 transform,
            const DrawCommand& command,
            Image& image,std::vector<MSAAPixel>& msaaBuffer
        ) {
            if (command.vertices.empty() || command.indices.size() < 3) return;

            std::vector<glm::vec2> pts;
            pts.reserve(command.vertices.size());

            for (const auto& v : command.vertices) {
                pts.emplace_back(
                    detail::transform_point(
                       transform * command.geometry_transform,
                    {v.position.x, v.position.y})
                );
            }

            for (size_t i = 0; i + 2 < command.indices.size(); i += 3)
            {
                const uint32_t ia = command.indices[i + 0];
                const uint32_t ib = command.indices[i + 1];
                const uint32_t ic = command.indices[i + 2];

                if (ia >= pts.size() || ib >= pts.size() || ic >= pts.size())
                    continue;

                glm::vec2 a = pts[ia];
                glm::vec2 b = pts[ib];
                glm::vec2 c = pts[ic];

                a.x -= static_cast<float>(image.xMin);
                a.y -= static_cast<float>(image.yMin);

                b.x -= static_cast<float>(image.xMin);
                b.y -= static_cast<float>(image.yMin);

                c.x -= static_cast<float>(image.xMin);
                c.y -= static_cast<float>(image.yMin);

                RasterizeTriangle(a, b, c,transform,command, image,msaaBuffer);
            }
        }

        static void RasterizeTriangle(
                glm::vec2 a,
                glm::vec2 b,
                glm::vec2 c,
                const glm::mat4 transform,
                const DrawCommand& command,
                Image& image,
                std::vector<MSAAPixel>& msaaBuffer
        ) {
            constexpr double eps = 1e-12;

            double area = detail::edge(a,b,c);

            if (std::abs(area) <= eps)
                return;

            if (area < 0.0)
            {
                std::swap(b, c);
                area = detail::edge(a,b,c);
            }

            const double inv_area = 1.0 / area;
            const int min_x = std::max(0, static_cast<int>(std::floor(std::min({a.x, b.x, c.x}))));
            const int max_x = std::min(image.width - 1, static_cast<int>(std::ceil(std::max({a.x, b.x, c.x}))));
            const int min_y = std::max(0, static_cast<int>(std::floor(std::min({a.y, b.y, c.y}))));
            const int max_y = std::min(image.height - 1, static_cast<int>(std::ceil(std::max({a.y, b.y, c.y}))));
            if (min_x > max_x || min_y > max_y) return;

            static const glm::vec2 samples[8] = {
                {0.5625f,0.3125f},
                {0.4375f,0.6875f},
                {0.8125f,0.5625f},
                {0.3125f,0.1875f},
                {0.1875f,0.8125f},
                {0.0625f,0.4375f},
                {0.6875f,0.9375f},
                {0.9375f,0.0625f}
            };
            static const int samplesCount = 8;

            const bool e0TopLeft = detail::IsTopLeft(b, c);
            const bool e1TopLeft = detail::IsTopLeft(c, a);
            const bool e2TopLeft = detail::IsTopLeft(a, b);

            for (int y = min_y; y <= max_y; ++y) {
                const int bitmapY = image.height - 1 - y;
                for (int x = min_x; x <= max_x; ++x) {

                    const size_t idx = bitmapY * image.width + x;

                    glm::vec2 c0(static_cast<float>(x) + 0.0f, static_cast<float>(y) + 0.0f);
                    glm::vec2 c1(static_cast<float>(x) + 1.0f, static_cast<float>(y) + 0.0f);
                    glm::vec2 c2(static_cast<float>(x) + 0.0f, static_cast<float>(y) + 1.0f);
                    glm::vec2 c3(static_cast<float>(x) + 1.0f, static_cast<float>(y) + 1.0f);

                    double e00 = detail::edge(b,c,c0) * inv_area;
                    double e01 = detail::edge(b,c,c1) * inv_area;
                    double e02 = detail::edge(b,c,c2) * inv_area;
                    double e03 = detail::edge(b,c,c3) * inv_area;

                    double e10 = detail::edge(c,a,c0) * inv_area;
                    double e11 = detail::edge(c,a,c1) * inv_area;
                    double e12 = detail::edge(c,a,c2) * inv_area;
                    double e13 = detail::edge(c,a,c3) * inv_area;

                    double e20 = detail::edge(a,b,c0) * inv_area;
                    double e21 = detail::edge(a,b,c1) * inv_area;
                    double e22 = detail::edge(a,b,c2) * inv_area;
                    double e23 = detail::edge(a,b,c3) * inv_area;

                    bool fullyInside =
                            e00 >= 0.0 && e01 >= 0.0 &&
                            e02 >= 0.0 && e03 >= 0.0 &&

                            e10 >= 0.0 && e11 >= 0.0 &&
                            e12 >= 0.0 && e13 >= 0.0 &&

                            e20 >= 0.0 && e21 >= 0.0 &&
                            e22 >= 0.0 && e23 >= 0.0;


                    if (fullyInside)
                    {


                        for (int sampleIndex = 0;sampleIndex < samplesCount;++sampleIndex)
                        {
                            glm::vec2 s = samples[sampleIndex];

                            glm::vec2 p(static_cast<float>(x) + s.x,static_cast<float>(y) + s.y);

                            glm::vec2 glyphSamplePos(p.x + static_cast<float>(image.xMin),p.y + static_cast<float>(image.yMin));

                            glm::vec4 src = detail::sample_paint(transform,command.paint,glyphSamplePos);

                            auto& sample = msaaBuffer[idx].samples[sampleIndex];

                            sample.covered = true;

                            sample.color = detail::composite(src,sample.color,command.composite_mode);
                        }

                        continue;
                    }

                    for (int sampleIndex = 0;sampleIndex < samplesCount;++sampleIndex)
                    {
                        glm::vec2 s = samples[sampleIndex];

                        glm::vec2 p(static_cast<float>(x) + s.x,static_cast<float>(y) + s.y);

                        double w0 = detail::edge(b,c,p) * inv_area;

                        double w1 = detail::edge(c,a,p) * inv_area;

                        double w2 = detail::edge(a,b,p) * inv_area;


                        bool inside = (w0 > eps || (std::abs(w0) <= eps && e0TopLeft)) &&
                                (w1 > eps || (std::abs(w1) <= eps && e1TopLeft)) &&
                                (w2 > eps || (std::abs(w2) <= eps && e2TopLeft));

                        if (inside)
                        {

                            glm::vec2 glyphSamplePos(p.x + static_cast<float>(image.xMin),p.y + static_cast<float>(image.yMin));

                            glm::vec4 src = detail::sample_paint(transform,command.paint,glyphSamplePos);

                            auto& sample = msaaBuffer[idx].samples[sampleIndex];

                            sample.covered = true;

                            sample.color = detail::composite(src,sample.color,command.composite_mode);
                        }
                    }
                }
            }
        }


        static void ResolveMSAA(
                Image& image,
                const std::vector<MSAAPixel>& msaaBuffer)
        {
            for (int y = 0; y < image.height; ++y)
            {
                for (int x = 0; x < image.width; ++x)
                {
                    size_t idx = y * image.width + x;
                    glm::vec4 result(0.0f);
                    const auto& pixel = msaaBuffer[idx];

                    for (const auto & sample : pixel.samples)
                    {
                        result += sample.color;
                    }

                    result *= 0.25f;

                    uint8_t* dst = image.pixels.data() + idx * 4;

                    detail::write_bgra8(dst,result);
                }
            }
        }
    };
}

#endif //MEDIAENGINE_COLRV1RENDERER_H
