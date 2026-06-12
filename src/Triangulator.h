//
// Created by MOHDU on 19-01-2026.
//

#ifndef MEDIAENGINE_TRIANGULATOR_H
#define MEDIAENGINE_TRIANGULATOR_H
#pragma once
#include <vector>
#include "../ext/glm/glm.hpp"
#include "../ext/glm/gtc/constants.hpp"
#include "../ext/earcut/earcut.h"

using Point = glm::dvec2;
using Ring = std::vector<Point>;
using Polygon = std::vector<Ring>;

struct alignas(16) Vertex
{
    glm::vec4 position;
    glm::vec4 normal;

public:
    Vertex() = delete;

    Vertex(const glm::vec4& p,
           const glm::vec4& n)
            : position(p), normal(n){}
};
static_assert(sizeof(Vertex) == 32);

namespace mapbox::util {
    template <>
    struct nth<0, Point>
    {
        static double get(const Point& p)
        {
            return p.x;
        }
    };

    template <>
    struct nth<1, Point>
    {
        static double get(const Point& p)
        {
            return p.y;
        }
    };
}

class Triangulator {
private:
    static constexpr double kEpsilon = 1e-10;

    struct Edge {
        Point low;
        Point high;

        [[nodiscard]] double xAt(double y) const {
            const double t = (y - low.y) / (high.y - low.y);
            return low.x + (high.x - low.x) * t;
        }
    };

    Polygon path_;
    Ring* currentRing_ = nullptr;

    inline void emitTriangle(
            const Point& a,
            const Point& b,
            const Point& c
    ) {
        const double twiceArea = (b.x - a.x) * (c.y - a.y) -
                                 (b.y - a.y) * (c.x - a.x);
        if (std::abs(twiceArea) <= kEpsilon) {
            return;
        }

        static const glm::vec4 kNormal(0.0f, 0.0f, 1.0f, 1.0f);
        const auto base = static_cast<uint32_t>(vertices.size());

        vertices.emplace_back(Vertex{
            {static_cast<float>(a.x), static_cast<float>(a.y),
             0.0f, 1.0f}, kNormal});

        vertices.emplace_back(Vertex{
            {static_cast<float>(b.x), static_cast<float>(b.y),
             0.0f, 1.0f}, kNormal});

        vertices.emplace_back(Vertex{
            {static_cast<float>(c.x), static_cast<float>(c.y),
             0.0f, 1.0f}, kNormal});

        indices.emplace_back(base);
        indices.emplace_back(base + 1);
        indices.emplace_back(base + 2);
    }

    static inline bool almostEqual(const Point& a, const Point& b)
    {
        return glm::length(a - b) < 1e-6;
    }

public:
    std::vector<Vertex>& vertices;
    std::vector<uint32_t>& indices;

    explicit Triangulator(std::vector<Vertex>& v, std::vector<uint32_t>& i)
            : vertices(v), indices(i) {}

    inline void beginPath() {
        path_.clear();
        currentRing_ = nullptr;
    }

    inline void moveTo(const Point& point) {
        path_.emplace_back();
        currentRing_ = &path_.back();
        currentRing_->push_back(point);
    }

    inline void lineTo(const Point& point) {
        if (currentRing_ == nullptr) {
            moveTo(point);
            return;
        }
        if (currentRing_->empty() || !almostEqual(currentRing_->back(), point)) {
            currentRing_->push_back(point);
        }
    }

    inline void closePath() {
        if (currentRing_ != nullptr && currentRing_->size() >= 2 &&
            currentRing_->front() != currentRing_->back()) {
            currentRing_->push_back(currentRing_->front());
        }
    }

    inline void fillPath()
    {
        closePath();
        for (const Ring& ring : path_)
        {
            if (ring.size() < 3)
                continue;

            Polygon poly;
            poly.push_back(ring);

            triangulate(poly);
        }
    }

    inline void clearPath() {
        beginPath();
    }

    [[nodiscard]] const Polygon& path() const {
        return path_;
    }

    inline void triangulate(const Polygon& polygon)
    {
        if (polygon.empty())
            return;

        auto earcutIndices =
                mapbox::earcut<uint32_t>(polygon);

        std::vector<Point> flatPoints;

        for (const auto& ring : polygon)
        {
            for (const auto& p : ring)
            {
                flatPoints.push_back(p);
            }
        }

        static const glm::vec4 kNormal(
                0.f, 0.f, 1.f, 1.f);

        auto base = static_cast<uint32_t>(vertices.size());

        for (const auto& p : flatPoints)
        {
            vertices.emplace_back(Vertex{
                    {
                            (float)p.x,
                            (float)p.y,
                            0.f,
                            1.f
                    },
                    kNormal
            });
        }

        for (uint32_t idx : earcutIndices)
        {
            indices.push_back(base + idx);
        }
    }

    inline void drawThickLine(Point thickness, Point a, Point b,
                              double startOffset = 0.0, double endOffset = 0.0) {
        Point d = b - a;
        const double len2 = glm::dot(d, d);
        if (len2 <= 1e-20) {
            return;
        }

        const double invLen = 1.0 / std::sqrt(len2);
        const Point direction = d * invLen;
        a += direction * startOffset;
        b -= direction * endOffset;

        d = b - a;
        if (glm::dot(d, d) <= 1e-20) {
            return;
        }

        const Point halfThickness = thickness * 0.5;
        const Point offset = Point(-direction.y, direction.x) * halfThickness;
        emitTriangle(a - offset, a + offset, b - offset);
        emitTriangle(b - offset, a + offset, b + offset);
    }

    inline void drawFilledEllipse(const Point& center, const Point& radius,
                                  int segments = 32) {
        if (segments < 3 || radius.x <= 0.0 || radius.y <= 0.0) {
            return;
        }

        const double step = glm::two_pi<double>() / static_cast<double>(segments);
        Point previous = center + Point(radius.x, 0.0);
        for (int i = 1; i <= segments; ++i) {
            const double angle = step * static_cast<double>(i);
            const Point current = center +
                                  Point(std::cos(angle) * radius.x, std::sin(angle) * radius.y);
            emitTriangle(center, previous, current);
            previous = current;
        }
    }

    inline void drawFilledRectangle(const Point& center, const Point& size) {
        if (size.x <= 0.0 || size.y <= 0.0) {
            return;
        }

        const Point half = size * 0.5;
        const Point bottomLeft = center - half;
        const Point bottomRight{center.x + half.x, center.y - half.y};
        const Point topRight = center + half;
        const Point topLeft{center.x - half.x, center.y + half.y};
        emitTriangle(bottomLeft, bottomRight, topRight);
        emitTriangle(topRight, topLeft, bottomLeft);
    }

    inline void drawStrokeEllipse(const Point& center, const Point& radius,
                                  const Point& thickness, int segments = 10) {
        if (segments < 3 || thickness.x <= 0.0 || thickness.y <= 0.0) {
            return;
        }

        const Point halfThickness = thickness * 0.5;
        const Point outerRadius = radius + halfThickness;
        const Point innerRadius = radius - halfThickness;
        if (innerRadius.x <= 0.0 || innerRadius.y <= 0.0) {
            return;
        }

        const double step = glm::two_pi<double>() / static_cast<double>(segments);
        Point previousOuter = center + Point(outerRadius.x, 0.0);
        Point previousInner = center + Point(innerRadius.x, 0.0);
        for (int i = 1; i <= segments; ++i) {
            const double angle = step * static_cast<double>(i);
            const double cosine = std::cos(angle);
            const double sine = std::sin(angle);
            const Point currentOuter = center +
                                       Point(cosine * outerRadius.x, sine * outerRadius.y);
            const Point currentInner = center +
                                       Point(cosine * innerRadius.x, sine * innerRadius.y);
            emitTriangle(previousOuter, previousInner, currentOuter);
            emitTriangle(currentOuter, previousInner, currentInner);
            previousOuter = currentOuter;
            previousInner = currentInner;
        }
    }

    inline void drawStrokeRectangle(const Point& center, const Point& size,
                                    const Point& thickness) {
        if (size.x <= 0.0 || size.y <= 0.0 ||
            thickness.x <= 0.0 || thickness.y <= 0.0) {
            return;
        }

        const Point halfSize = size * 0.5;
        const Point halfThickness = thickness * 0.5;
        const Point outerMin = center - halfSize - halfThickness;
        const Point outerMax = center + halfSize + halfThickness;
        const Point innerMin = center - halfSize + halfThickness;
        const Point innerMax = center + halfSize - halfThickness;
        if (innerMin.x >= innerMax.x || innerMin.y >= innerMax.y) {
            return;
        }

        emitTriangle({outerMin.x, outerMax.y}, {outerMax.x, outerMax.y},
                     {innerMax.x, innerMax.y});
        emitTriangle({innerMax.x, innerMax.y}, {innerMin.x, innerMax.y},
                     {outerMin.x, outerMax.y});
        emitTriangle({outerMin.x, outerMin.y}, {innerMin.x, innerMin.y},
                     {innerMax.x, innerMin.y});
        emitTriangle({innerMax.x, innerMin.y}, {outerMax.x, outerMin.y},
                     {outerMin.x, outerMin.y});
        emitTriangle({outerMin.x, innerMin.y}, {innerMin.x, innerMin.y},
                     {innerMin.x, innerMax.y});
        emitTriangle({innerMin.x, innerMax.y}, {outerMin.x, innerMax.y},
                     {outerMin.x, innerMin.y});
        emitTriangle({innerMax.x, innerMin.y}, {outerMax.x, innerMin.y},
                     {outerMax.x, innerMax.y});
        emitTriangle({outerMax.x, innerMax.y}, {innerMax.x, innerMax.y},
                     {innerMax.x, innerMin.y});
    }
};

#endif //MEDIAENGINE_TRIANGULATOR_H
