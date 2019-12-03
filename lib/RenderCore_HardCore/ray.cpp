#include "ray.h"

namespace lh2core {

optional<float> intersect(const Ray &ray, const CoreTri &tri) {
    float3 edge1, edge2, h, s, q;
    float a, f, u, v;
    edge1 = tri.vertex1 - tri.vertex0;
    edge2 = tri.vertex2 - tri.vertex0;
    h = cross(ray.b, edge2);
    a = dot(edge1, h);

    // if ray is parallel, quit
    if (a > -EPSILON && a < EPSILON) return nullopt;

    f = 1.0 / a;
    s = ray.a - tri.vertex0;
    u = f * dot(s, h);

    if (u < 0.0 || u > 1.0) return nullopt;

    q = cross(s, edge1);
    v = f * dot(ray.b, q);

    if (v < 0.0 || u + v > 1.0) return nullopt;

    // compute t
    float tt = f * dot(edge2, q);
    if (tt > EPSILON && tt < 1 / EPSILON) { // ray intersection
        return tt;
    }
    return nullopt;
}

} // namespace lh2core
