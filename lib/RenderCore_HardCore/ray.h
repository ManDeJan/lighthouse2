#pragma once

#include "core_settings.h"

namespace lh2core {

class Ray {
public:
    float3 org;
    float3 dir;

    Ray() = default;
    Ray(const float3 &org, const float3 &dir) : org(org), dir(dir) {}

    float3 origin() const {
        return org;
    }

    float3 direction() const {
        return dir;
    }

    float3 point_at_parameter(float t) const {
        return org + t * dir;
    }
};

// Code adapted from slides
optional<float> intersect(const Ray &ray, const CoreTri &tri);
float3 calculateColor(const Ray &ray, const CoreTri &tri, float t, vector<CoreMaterial> &materials, uint recursion_depth = 3); // default 3 recursions
float3 calculateColor(const Ray &ray,
                      float t,
                      vector<CoreMaterial> &materials,
                      uint recursion_depth = 3); // default 3 recursions
} // namespace lh2core
