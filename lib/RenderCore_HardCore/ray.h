#pragma once

#include "core_settings.h"

namespace lh2core {

class Ray {
public:
    float3 a;
    float3 b;

    Ray() = default;
    Ray(const float3 &a, const float3 &b) : a(a), b(b) {}

    float3 origin() const {
        return a;
    }

    float3 direction() const {
        return b;
    }

    float3 point_at_parameter(float t) const {
        return a + t * b;
    }
};

// Code adapted from slides
optional<float> intersect(const Ray &ray, const CoreTri &tri);

} // namespace lh2core
