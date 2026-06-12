
#ifndef MEDIAENGINE_OUTLINEDECOMPOSER_H
#define MEDIAENGINE_OUTLINEDECOMPOSER_H
#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "freetype/ftoutln.h"

constexpr double FT_TO_PX = 1.0 / 64.0;

using Point   = glm::dvec2;   // double precision
using Ring    = std::vector<Point>;
using Polygon = std::vector<Ring>;

struct OutlineVec
{
    int flags = 0;
    std::vector<FT_Vector> points;
    std::vector<unsigned char> tags;
    std::vector<unsigned short> contours;

    OutlineVec() = delete;

    inline explicit OutlineVec(const FT_Outline& src)
    {
        flags = src.flags;
        points.assign(src.points,   src.points   + src.n_points);
        tags.assign(src.tags,       src.tags     + src.n_points);
        contours.assign(src.contours, src.contours + src.n_contours);
    }

    inline FT_Outline makeFTView()
    {
        FT_Outline out{};
        out.n_points   = (short)points.size();
        out.n_contours = (short)contours.size();
        out.points     = points.data();
        out.tags       = tags.data();
        out.contours   = contours.data();
        out.flags      = flags;
        return out;
    }
};

struct UVTransform
{
    Point scale;
    Point offset;
};

class OutlineDecomposer {
private:
    static constexpr double kFreeTypeToPixels = 1.0 / 64.0;
    static constexpr double kMinimumDimension = 1e-12;
    static constexpr int kMaximumSubdivisionDepth = 16;

    struct DecomposeState {
        Triangulator* triangulator = nullptr;
        Point currentPoint{0.0, 0.0};
        bool hasCurrentPoint = false;

        bool normalize = false;
        double inverseWidth = 0.0;
        double inverseHeight = 0.0;
        double minimumX = 0.0;
        double minimumY = 0.0;
        double tolerancePixels = 0.1;
    };

    static Point convertPoint(
            const FT_Vector& point,
            const DecomposeState& state
    ) {
        double x = static_cast<double>(point.x) * kFreeTypeToPixels;
        double y = static_cast<double>(point.y) * kFreeTypeToPixels;

        if (state.normalize) {
            x = (x - state.minimumX) * state.inverseWidth;
            y = (y - state.minimumY) * state.inverseHeight;
        }
        return {x, y};
    }

    static double transformedTolerance(const DecomposeState& state) {
        if (!state.normalize) {
            return state.tolerancePixels;
        }
        return state.tolerancePixels *
               0.5 * (state.inverseWidth + state.inverseHeight);
    }

    static double quadraticFlatness(
            const Point& start,
            const Point& control,
            const Point& end
    ){
        const Point chord = end - start;
        const double lengthSquared = glm::dot(chord, chord);
        if (lengthSquared <= kMinimumDimension) {
            return glm::length(control - start);
        }

        const double projectionAmount = glm::dot(control - start, chord) / lengthSquared;
        const Point projection = start + projectionAmount * chord;
        return glm::length(control - projection);
    }

    static void flattenQuadratic(
            const Point& start,
            const Point& control,
            const Point& end,
            double tolerance,
            DecomposeState& state,
            int depth = 0
    ) {
        if (depth >= kMaximumSubdivisionDepth ||
            quadraticFlatness(start, control, end) <= tolerance) {
            state.triangulator->lineTo(end);
            state.currentPoint = end;
            return;
        }

        const Point startControl = (start + control) * 0.5;
        const Point controlEnd = (control + end) * 0.5;
        const Point midpoint = (startControl + controlEnd) * 0.5;

        flattenQuadratic(start, startControl, midpoint, tolerance, state, depth + 1);
        flattenQuadratic(midpoint, controlEnd, end, tolerance, state, depth + 1);
    }

    static double cubicFlatness(
            const Point& start,
            const Point& firstControl,
            const Point& secondControl,
            const Point& end
    ) {
        const Point chord = end - start;
        const double lengthSquared = glm::dot(chord, chord);
        if (lengthSquared <= kMinimumDimension) {
            return std::max(glm::length(firstControl - start),
                            glm::length(secondControl - start));
        }

        const double firstAmount = glm::dot(firstControl - start, chord) / lengthSquared;
        const double secondAmount = glm::dot(secondControl - start, chord) / lengthSquared;
        const Point firstProjection = start + firstAmount * chord;
        const Point secondProjection = start + secondAmount * chord;
        return std::max(glm::length(firstControl - firstProjection),
                        glm::length(secondControl - secondProjection));
    }

    static void flattenCubic(
            const Point& start,
            const Point& firstControl,
            const Point& secondControl,
            const Point& end,
            double tolerance,
            DecomposeState& state,
            int depth = 0
    ) {
        if (depth >= kMaximumSubdivisionDepth ||
            cubicFlatness(start, firstControl, secondControl, end) <= tolerance) {
            state.triangulator->lineTo(end);
            state.currentPoint = end;
            return;
        }

        const Point startFirst = (start + firstControl) * 0.5;
        const Point firstSecond = (firstControl + secondControl) * 0.5;
        const Point secondEnd = (secondControl + end) * 0.5;
        const Point leftControl = (startFirst + firstSecond) * 0.5;
        const Point rightControl = (firstSecond + secondEnd) * 0.5;
        const Point midpoint = (leftControl + rightControl) * 0.5;

        flattenCubic(start, startFirst, leftControl, midpoint,
                     tolerance, state, depth + 1);
        flattenCubic(midpoint, rightControl, secondEnd, end,
                     tolerance, state, depth + 1);
    }

    static int moveTo(const FT_Vector* to, void* user) {
        auto* state = static_cast<DecomposeState*>(user);
        if (state == nullptr || state->triangulator == nullptr || to == nullptr) {
            return 1;
        }

        const Point point = convertPoint(*to, *state);
        state->triangulator->closePath();
        state->triangulator->moveTo(point);
        state->currentPoint = point;
        state->hasCurrentPoint = true;
        return 0;
    }

    static int lineTo(const FT_Vector* to, void* user) {
        auto* state = static_cast<DecomposeState*>(user);
        if (state == nullptr || state->triangulator == nullptr ||
            !state->hasCurrentPoint || to == nullptr) {
            return 1;
        }

        const Point point = convertPoint(*to, *state);
        state->triangulator->lineTo(point);
        state->currentPoint = point;
        return 0;
    }

    static int conicTo(const FT_Vector* control, const FT_Vector* to, void* user) {
        auto* state = static_cast<DecomposeState*>(user);
        if (state == nullptr || state->triangulator == nullptr ||
            !state->hasCurrentPoint || control == nullptr || to == nullptr) {
            return 1;
        }

        const Point controlPoint = convertPoint(*control, *state);
        const Point end = convertPoint(*to, *state);
        flattenQuadratic(state->currentPoint, controlPoint, end,
                         transformedTolerance(*state), *state);
        return 0;
    }

    static int cubicTo(
            const FT_Vector* firstControl,
            const FT_Vector* secondControl,
            const FT_Vector* to,
            void* user
    ) {
        auto* state = static_cast<DecomposeState*>(user);
        if (state == nullptr || state->triangulator == nullptr ||
            !state->hasCurrentPoint || firstControl == nullptr ||
            secondControl == nullptr || to == nullptr) {
            return 1;
        }

        const Point first = convertPoint(*firstControl, *state);
        const Point second = convertPoint(*secondControl, *state);
        const Point end = convertPoint(*to, *state);
        flattenCubic(state->currentPoint, first, second, end,
                     transformedTolerance(*state), *state);
        return 0;
    }

    static const FT_Outline_Funcs& functions() {
        static const FT_Outline_Funcs callbacks = {
                &OutlineDecomposer::moveTo,
                &OutlineDecomposer::lineTo,
                &OutlineDecomposer::conicTo,
                &OutlineDecomposer::cubicTo,
                0,
                0
        };
        return callbacks;
    }

    static FT_Error decomposeWithState(FT_Outline& outline, DecomposeState& state) {
        state.triangulator->beginPath();
        const FT_Error error = FT_Outline_Decompose(&outline, &functions(), &state);
        if (error != 0) {
            state.triangulator->clearPath();
            return error;
        }

        state.triangulator->closePath();
        state.triangulator->fillPath();
        return 0;
    }

public:
    static FT_Error decompose(
            FT_Outline& outline,
            Triangulator& triangulator,
            double tolerancePixels = 0.1
    ) {
        DecomposeState state;
        state.triangulator = &triangulator;
        state.tolerancePixels = std::max(tolerancePixels, kMinimumDimension);
        return decomposeWithState(outline, state);
    }

    static FT_Error decomposeNormalized(
            FT_Outline& outline,
            Triangulator& triangulator,
            double xMin, double yMin,
            const Size& glyphSize,
            double tolerancePixels = 0.1
    ){
        DecomposeState state;
        state.triangulator = &triangulator;
        state.normalize = true;
        state.minimumX = xMin;
        state.minimumY = yMin;
        state.inverseWidth = 1.0 / std::max((double) glyphSize.width, kMinimumDimension);
        state.inverseHeight = 1.0 / std::max((double) glyphSize.height, kMinimumDimension);
        state.tolerancePixels = std::max(tolerancePixels, kMinimumDimension);
        return decomposeWithState(outline, state);
    }


    static inline FT_Error decompose(OutlineVec* outline,Triangulator& triangulator)
    {
        auto ft = outline->makeFTView();
        return decompose(ft,triangulator);
    }

    static inline FT_Error decomposeNormalized(
            OutlineVec* outline,
            Triangulator& triangulator,
            const double xMin,
            const double yMin,
            const Size& glyphSize
    ){
        auto ft = outline->makeFTView();
        return decomposeNormalized(ft,triangulator,xMin,yMin,glyphSize);
    }

    static inline FT_Error decompose(const std::shared_ptr<OutlineVec>& outline,Triangulator& triangulator)
    {
        return decompose(outline.get(),triangulator);
    }

    static inline FT_Error decomposeNormalized(
            const std::shared_ptr<OutlineVec>& outline,
            Triangulator& triangulator,
            const double xMin,
            const double yMin,
            const Size& glyphSize
    ){
        return decomposeNormalized(outline.get(),triangulator,xMin,yMin,glyphSize);
    }

};


#endif //MEDIAENGINE_OUTLINEDECOMPOSER_H
