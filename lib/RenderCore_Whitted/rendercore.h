/* rendercore.h - Copyright 2019 Utrecht University

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

	   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#pragma once

#include "core_settings.h"
#include "BVH.h"

namespace lh2core {
    
struct Intersection {
    CoreTri *triangle;
    float3 location;
    float distance = numeric_limits<float>::max();
    uint intersections_count = 0;
};

enum class MaterialType { DIELECTRIC, MIRROR, DIFFUSE, LIGHT };

class Material {
public:
    float3 color;
    float3 transmittance;

    MaterialType type;
};

class Ray_original {
public:
    float3 origin;
    float3 direction;

    Ray_original(float3 _origin, float3 _direction, bool offset = false) {
        origin = _origin;
        direction = _direction;
        if (offset) { origin += direction * EPSILON; }
    }

    // https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection
    float calcIntersectDist(CoreTri &tri) const {
        float3 v0v1 = tri.vertex1 - tri.vertex0;
        float3 v0v2 = tri.vertex2 - tri.vertex0;
        float3 pvec = cross(direction, v0v2);
        float det = dot(v0v1, pvec);

        if (fabs(det) < EPSILON) return UINT_MAX;
        float invDet = 1 / det;

        float3 tvec = origin - tri.vertex0;
        float u = dot(tvec, pvec) * invDet;
        if (u < 0 || u > 1) return UINT_MAX;

        float3 qvec = cross(tvec, v0v1);
        float v = dot(direction, qvec) * invDet;
        if (v < 0 || u + v > 1) return UINT_MAX;

        float t = dot(v0v2, qvec) * invDet;
        if (t < EPSILON || t > 1 / EPSILON) return UINT_MAX;

        return t;
    }
};


template <size_t Size>
struct SIMD_Ray {
    constexpr static auto size = Size;
    array<float3, Size> origins;
    array<float3, Size> directions;
    SIMD_Ray (const array<float3, Size> &origins,
              const array<float3, Size> &directions,
              bool offset = false) : origins(origins), directions(directions) {
        if (offset)
            for (size_t i = 0; i < Size; i++)
                origins[i] += directions[i] * EPSILON;
    }
    SIMD_Ray (float3 origin, float3 direction, bool offset = false) {
        origins[0] = origin;
        directions[0] = direction;
        if (offset) { origins[0] += direction * EPSILON; }
    }
    SIMD_Ray() = default;

    float calcIntersectDist(const CoreTri &tri, const size_t i = 0) const {
        float3 v0v1 = tri.vertex1 - tri.vertex0;
        float3 v0v2 = tri.vertex2 - tri.vertex0;
        float3 pvec = cross(directions[i], v0v2);
        float det = dot(v0v1, pvec);

        if (fabs(det) < EPSILON) return UINT_MAX;
        float invDet = 1 / det;

        float3 tvec = origins[i] - tri.vertex0;
        float u = dot(tvec, pvec) * invDet;
        if (u < 0 || u > 1) return UINT_MAX;

        float3 qvec = cross(tvec, v0v1);
        float v = dot(directions[i], qvec) * invDet;
        if (v < 0 || u + v > 1) return UINT_MAX;

        float t = dot(v0v2, qvec) * invDet;
        if (t < EPSILON || t > 1 / EPSILON) return UINT_MAX;

        return t;
    }
};

using Ray1  = SIMD_Ray<1>;
using Ray2  = SIMD_Ray<2>;
using Ray4  = SIMD_Ray<4>;
using Ray8  = SIMD_Ray<8>;
using Ray16 = SIMD_Ray<16>;

using Ray   = SIMD_Ray<4>;

//  +-----------------------------------------------------------------------------+
//  |  Mesh                                                                       |
//  |  Minimalistic mesh storage.                                           LH2'19|
//  +-----------------------------------------------------------------------------+
class Mesh {
public:
    float4 *vertices = 0;   // vertex data received via SetGeometry
    int vcount = 0;         // vertex count
    CoreTri *triangles = 0; // 'fat' triangle data
};

//  +-----------------------------------------------------------------------------+
//  |  RenderCore                                                                 |
//  |  Encapsulates device code.                                            LH2'19|
//  +-----------------------------------------------------------------------------+
class RenderCore {
public:
    // methods
    void Init();
    void SetTarget(GLTexture *target);
    void SetMaterials(CoreMaterial *mat, const CoreMaterialEx *matEx, const int materialCount);
    void SetGeometry(const int meshIdx,
                     const float4 *vertexData,
                     const int vertexCount,
                     const int triangleCount,
                     const CoreTri *triangles,
                     const uint *alphaFlags = 0);
    void SetLights(const CoreLightTri *areaLights,
                   const int areaLightCount,
                   const CorePointLight *_pointLights,
                   const int pointLightCount,
                   const CoreSpotLight *_spotLights,
                   const int spotLightCount,
                   const CoreDirectionalLight *directionalLights,
                   const int directionalLightCount);
    void SetSkyData(const float3 *pixels, const uint width, const uint height);
    void Render(const ViewPyramid &view, const Convergence converge);
    void Shutdown();
    // internal methods
private:
    Intersection getNearestIntersection(Ray &ray);
    Intersection getNearestIntersectionBVH(Ray &ray);
    template <size_t RaySize> Intersection traverseBVH(const SIMD_Ray<RaySize> &ray, const Node &node, Intersection &inter);
    bool intersectNode(const Ray &ray, const Node &node, float &t);
    float3 calcRayColor(Ray ray, uint depth);
    bool existNearerIntersection(Ray ray, float distance);
    float3 calcLightContributions(float3 mColor, float3 iPoint, float3 triangleN, float closestIntersection);
    void resetAccBuffer();

    // data members
    Bitmap *screen = 0;                     // temporary storage of RenderCore output; will be copied to render target
    int targetTextureID = 0;                // ID of the target OpenGL texture
    vector<Mesh> meshes;                    // mesh data storage
    vector<Material *> materials;           // material data storage
    vector<CorePointLight> pointLights;     // pointlight storage
    vector<CoreDirectionalLight> dirLights; // directional light storage
    vector<CoreSpotLight> spotLights;       // spotlight storage
    vector<CoreLightTri> areaLights;        // arealight storage
    BVH bvh;                       // BVH storage

    // Store skydata
    vector<float3> skyData;
    int skyWidth, skyHeight;

    // Not every pixel color is calculated on very frame (due to NOISE_PROB).
    // Whenever a color is calculated, it is stored in the (x,y,z) of the accumulator buffer and (w) is incremented by 1.
    // The (w) component thus stores how many times a pixel is calculated.
    // When plotting, we divide (x,y,z) by w to find the average value of the pixel over all frames.
    vector<float4> accBuffer;

public:
    CoreStats coreStats; // rendering statistics
};

} // namespace lh2core

// EOF
