/* rendercore.cpp - Copyright 2019 Utrecht University

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

#include "core_settings.h"
#include "rendercore.h"


using namespace lh2core;

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Init                                                           |
//  |  Initialization.                                                      LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Init() {
    accBuffer.clear();
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetTarget(GLTexture *target) {
    // synchronize OpenGL viewport
    targetTextureID = target->ID;
    if (screen != 0 && target->width == screen->width && target->height == screen->height) {
        return; // nothing changed
    }

    delete screen;
    screen = new Bitmap(target->width, target->height);

    // Reset accumulator buffer.
    resetAccBuffer();
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetGeometry                                                    |
//  |  Set the geometry data for a model.                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetGeometry(const int meshIdx,
                             const float4 *vertexData,
                             const int vertexCount,
                             const int triangleCount,
                             const CoreTri *triangleData,
                             const uint *alphaFlags) {
    Mesh newMesh;
    // copy the supplied vertices; we cannot assume that the render system does not modify
    // the original data after we leave this function.
    newMesh.vertices = new float4[vertexCount];
    newMesh.vcount = vertexCount;
    memcpy(newMesh.vertices, vertexData, vertexCount * sizeof(float4));
    // copy the supplied 'fat triangles'
    newMesh.triangles = new CoreTri[vertexCount / 3];
    memcpy(newMesh.triangles, triangleData, (vertexCount / 3) * sizeof(CoreTri));
    meshes.push_back(newMesh);
    bvh.setMesh(meshes.back());
    // print("new mesh added");
}


//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Render                                                         |
//  |  Produce one image.                                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Render(const ViewPyramid &view, const Convergence converge) {
    static bool rebuild_bvh = true;
    if (rebuild_bvh) {
        print("Building BVH");
        bvh.constructBVH();
        print("Done BVH");
        // print("BVH size ", bvh.nodes.size());
        // for (auto &node : bvh.nodes) {
        //     if (node.isLeaf()) {
        //         print("BLAD! : ", node.count, " ", node.first());
        //     } else {
        //         print("TAK!  : ", node.left());
        //     }
        // }
        // for (auto &idx : bvh.indices) {
        //     print("alle_idx ", idx);
        // }
        // printBVH(*bvh.root);
        rebuild_bvh = false;
    }
    constexpr float noise_probability = 0.1f; // To increase speed, we only shoot a primary ray for 10% of the pixels.

    // If camera moved, clear the screen and the accumulator buffer.
    if (converge == Restart) {
        screen->Clear();
        resetAccBuffer();
    }

    using namespace std::chrono;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

/* First loop over all pixels to calculate their color.	
	   The result is stored in the accumulator buffer. */
#pragma omp parallel for
    for (int y = 0; y < screen->height; y++) {
        for (int x = 0; x < screen->width; x++) {
            // Draw noisy to increase speed
            float plotPixel = Rand(1);

            if (plotPixel < noise_probability) {
                // if (true) {
                // Anti-aliasing: shoot the ray through a random position inside the pixel.
                float v = (y + Rand(1)) / (float)screen->height;
                float u = (x + Rand(1)) / (float)screen->width;
                float3 direction = normalize(view.p1 + u * (view.p2 - view.p1) + v * (view.p3 - view.p1) - view.pos);

                Ray primaryRay(view.pos, direction);
                // Intersection i = getNearestIntersection(primaryRay); // duplicate nearest intersection?? Zie calcRayColor

                // Calculate color and store in accumulator buffer.
                float3 pixelColor = calcRayColor(primaryRay, 0);
                accBuffer.at(size_t(y) * screen->width + x) += make_float4(pixelColor, 1.0f);
            }
        }
    }

/* Now plot all pixels to the screen from the buffer. */
#pragma omp parallel for
    for (int y = 0; y < screen->height; y++) {
        for (uint x = 0; x < screen->width; x++) {
            float4 accColor = accBuffer.at(size_t(y) * screen->width + x);
            float3 endColor = make_float3(accColor);

            // Disabling this results in white noise (+0/+0 = positive infinity)
            if (accColor.w > 0) { endColor /= accColor.w; }

            // Check that maximum brightness is 1.
            endColor = fminf(endColor * 255.0f, make_float3(255.0f));

            uint bgrColor = (uint(endColor.z) << 16) + (uint(endColor.y) << 8) + uint(endColor.x);
            screen->Plot(x, y, bgrColor);
        }
        // print("done row: ", y);
    }
    // copy pixel buffer to OpenGL render target texture
    glBindTexture(GL_TEXTURE_2D, targetTextureID);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_RGBA,
                 screen->width,
                 screen->height,
                 0,
                 GL_RGBA,
                 GL_UNSIGNED_BYTE,
                 screen->pixels);

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    print("frame time: ", int(time_span.count() * 1000), " ms");

    //printf("Render finished \n");
}

float3 RenderCore::calcRayColor(Ray ray, uint depth) {
    constexpr uint max_depth = 5; // Maximum recursion depth for each ray.
    constexpr float ambient_light = 0.01f;
    Intersection i = getNearestIntersectionBVH(ray);

    /* If we missed, get the skybox color. */
    if (i.distance >= numeric_limits<float>::max()) {
        // http://gl.ict.usc.edu/Data/HighResProbes/
        float u = 1 + atan2f(ray.direction.x, -ray.direction.z) * INVPI;
        float v = acosf(ray.direction.y) * INVPI;

        int xPixel = float(skyWidth) * 0.5 * u;
        int yPixel = float(skyHeight) * v;
        int pixelIdx = yPixel * skyWidth + xPixel;

        return skyData[max(0, min(skyHeight * skyWidth, pixelIdx))];
    }

    /* Otherwise, get the material we hit. */
    Material m = *materials.at(i.triangle->material);

    // Draw material color if max depth of reflections is reached.
    if (depth > max_depth) { return m.color; }

    // Are we facing the back of the triangle? If so, flip the normal.
    float3 triangleNormal = make_float3(i.triangle->Nx, i.triangle->Ny, i.triangle->Nz);
    bool backFacing = dot(ray.direction, triangleNormal) > 0.0f;
    if (backFacing) { triangleNormal *= -1; }

    // Switch depending on material type.
    switch (m.type) {
    case MaterialType::MIRROR: {
        float3 ndir = normalize(reflect(ray.direction, triangleNormal));
        Ray reflectiveRay(i.location, ndir, true);
        return ambient_light * m.color + (1 - ambient_light) * calcRayColor(reflectiveRay, ++depth);
    }

    case MaterialType::DIELECTRIC: {
        // Taken from https://www.flipcode.com/archives/reflection_transmission.pdf
        // This source makes a mistake in using incident as both + and - the direction of the ray
        // also https://blog.demofox.org/2017/01/09/raytracing-reflection-refraction-fresnel-total-internal-reflection-and-beers-law/

        // Calculate reflective ray
        float3 reflectDir = reflect(ray.direction, triangleNormal);
        Ray reflectiveRay(i.location, normalize(reflectDir), true);

        // Are we going into glass, or out of it?
        bool entering = !backFacing;
        float n1 = entering ? 1.0f : 1.52f;
        float n2 = entering ? 1.52f : 1.0f;
        float n = n1 / n2;

        // Angle of ray with normal
        float cosI = dot(-ray.direction, triangleNormal);
        float sin2T = n * n * (1 - cosI * cosI);

        if (sin2T > 1.0f) {
            // Total internal reflection
            float3 reflectCol = calcRayColor(reflectiveRay, ++depth);
            return m.color * reflectCol;
        }

        // Calculate the refractive ray, and its color
        float3 refractDir = n * ray.direction + (n * cosI - sqrtf(1.0f - sin2T)) * triangleNormal;
        Ray refractiveRay(i.location, normalize(refractDir), true);
        float3 refractCol = calcRayColor(refractiveRay, ++depth);

        // Schlicks approximation to determine the amount of reflection vs refraction
        float R0 = powf((n1 - n2) / (n1 + n2), 2.0f);
        float Fr = R0 + (1.0f - R0) * powf((1.0f - cosI), 5.0f);

        float3 reflectCol = calcRayColor(reflectiveRay, ++depth);
        return m.color * (Fr * reflectCol + (1.0f - Fr) * refractCol);
    }
    case MaterialType::LIGHT: {
        // We made area lights only emit light from the front, so in the back they should not have color.
        if (backFacing) return make_float3(ambient_light);
        return m.color;
    }

    case MaterialType::DIFFUSE: {
        return fmaxf(m.color * ambient_light, calcLightContributions(m.color, i.location, triangleNormal, i.distance));
    }

    default: {
        //Diffuse
        return fmaxf(m.color * ambient_light, calcLightContributions(m.color, i.location, triangleNormal, i.distance));
    }
    }
}

float3
RenderCore::calcLightContributions(float3 mColor, float3 iLocation, float3 triangleN, float closestIntersection) {
    float3 litColor = make_float3(0);

    /* We loop over all instances of all types of light, and calculate how much they
	   illuminate the given location. The general flow is the same for all types. From the intersection
	   location, shoot a shadow ray towards the light source. If any intersection is found between the two
	   then the location is occcluded and we just continue. If not, calculate how much that light source
	   contributes to the illumination of that location.
	*/

    // Handle pointlights.
    for (CorePointLight light : pointLights) {
        float3 sRayDir = light.position - iLocation;
        if (dot(triangleN, sRayDir) > 0.0f) {
            float3 normalizedSRayDir = normalize(sRayDir);
            float distToLight = length(sRayDir);

            Ray shadowRay(iLocation, normalizedSRayDir, true);
            bool occluded = existNearerIntersection(shadowRay, distToLight);

            // Add light transfer
            if (!occluded) {
                float3 scaledIntensity = light.radiance * (1.0f / (distToLight * distToLight));
                litColor += mColor * scaledIntensity * dot(triangleN, normalizedSRayDir); //Lambertian Shading
            }
        }
    }

    //Handle directional lighting
    for (CoreDirectionalLight light : dirLights) {
        float3 sRayDir = -(light.direction);
        if (dot(triangleN, sRayDir) > 0.0f) {
            // Directional lights are assumed to be infinitely far away.
            float3 normalizedSRayDir = normalize(sRayDir);
            float distToLight = UINT_MAX;

            Ray shadowRay(iLocation, normalizedSRayDir, true);
            bool occluded = existNearerIntersection(shadowRay, distToLight);

            // Add light transfer
            if (!occluded) {
                // No distance attenuation for directional lights.
                litColor += mColor * light.radiance * dot(triangleN, normalizedSRayDir); //Lambertian Shading
            }
        }
    }

    //Handle spotlight lighting
    for (CoreSpotLight light : spotLights) {
        float3 sRayDir = light.position - iLocation;
        if (dot(triangleN, sRayDir) > 0.0f) {
            float3 normalizedSRayDir = normalize(sRayDir);
            float distToLight = length(sRayDir);

            Ray shadowRay(iLocation, normalizedSRayDir, true);
            bool occluded = existNearerIntersection(shadowRay, distToLight);

            float cosAngleToLight = dot(-normalizedSRayDir, normalize(light.direction));
            // Add light transfer, check if outside of cosOuter angle
            if (!occluded && cosAngleToLight > light.cosOuter) {
                float3 scaledColor = mColor * light.radiance * (1.0f / (distToLight * distToLight));

                //calculate the dropoff of light when outside of the inner angle
                float dropoff = (cosAngleToLight - light.cosOuter) / (light.cosInner - light.cosOuter);
                dropoff = clamp(dropoff, 0.0, 1.0);
                scaledColor *= dropoff;
                litColor += scaledColor * dot(triangleN, normalizedSRayDir); //Lambertian Shading
            }
        }
    }

    for (CoreLightTri light : areaLights) {
        // Sample a random position on the area light to send a shadow ray to.
        float u = Rand(1), v = Rand(1);
        float3 randomPoint =
            (1 - sqrtf(u)) * light.vertex0 + sqrtf(u) * (1 - v) * light.vertex1 + sqrtf(u) * v * light.vertex2;

        float3 sRayDir = randomPoint - iLocation;
        if (dot(triangleN, sRayDir) > 0.0f && dot(light.N, -sRayDir) > 0.0f) {
            float3 normalizedSRayDir = normalize(sRayDir);
            float distToLight = length(sRayDir) - 2 * EPSILON;

            Ray shadowRay(iLocation, normalizedSRayDir, true);
            bool occluded = existNearerIntersection(shadowRay, distToLight);

            // Add light transfer
            if (!occluded) {
                float3 scaledColor =
                    mColor * light.radiance * (1.0f / (distToLight * distToLight)) * dot(light.N, -normalizedSRayDir);
                litColor += scaledColor * dot(triangleN, normalizedSRayDir); //Lambertian Shading
            }
        }
    }

    return litColor;
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::existNearerIntersection                                        |
//  |  Get the nearest intersection with a primitive for the given ray.		LH2'19|
//  +-----------------------------------------------------------------------------+
Intersection RenderCore::getNearestIntersection(Ray &ray) {
    Intersection i = Intersection();

    for (Mesh &mesh : meshes) {
        for (uint triangleIdx = 0; triangleIdx < mesh.vcount / 3; triangleIdx++) {
            CoreTri &triangle = mesh.triangles[triangleIdx];

            float t = ray.calcIntersectDist(triangle);
            if (t < i.distance && t > 0) {
                i.distance = t;
                i.triangle = &mesh.triangles[triangleIdx];
                i.location = ray.origin + ray.direction * i.distance;
            }
        }
    }
    return i;
}

bool RenderCore::intersectNode(const Ray &ray, const Node &node, float &t) {
    float3 dir_frac = make_float3(0, 0, 0);

    dir_frac.x = 1.0f / ray.direction.x;
    dir_frac.y = 1.0f / ray.direction.y;
    dir_frac.z = 1.0f / ray.direction.z;

    float t1 = (node.bounds.minBounds.x - ray.origin.x) * dir_frac.x;
    float t2 = (node.bounds.maxBounds.x - ray.origin.x) * dir_frac.x;
    float t3 = (node.bounds.minBounds.y - ray.origin.y) * dir_frac.y;
    float t4 = (node.bounds.maxBounds.y - ray.origin.y) * dir_frac.y;
    float t5 = (node.bounds.minBounds.z - ray.origin.z) * dir_frac.z;
    float t6 = (node.bounds.maxBounds.z - ray.origin.z) * dir_frac.z;

    float t_min = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
    float t_max = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));

    if (t_max < 0) {
        t = t_max;
        return false;
    }

    if (t_min > t_max) {
        t = t_max;
        return false;
    }

    t = t_min;
    return true;
}

int traverseBVHcount = 0;

Intersection RenderCore::traverseBVH(const Ray &ray, const Node &node, Intersection &inter) {

    // print("Traversal: ", ++traverseBVHcount);

    if (node.isLeaf()) {
        for (int i = node.first(); i < node.first() + node.count; i++) {
            float t = ray.calcIntersectDist(BVH::primitives[BVH::indices[i]]);
            if (t < inter.distance && t > 0) {
                inter.distance = t;
                inter.triangle = &BVH::primitives[BVH::indices[i]];
                inter.location = ray.origin + ray.direction * inter.distance;
            }
        }
    } else /* if the node is not a leaf */ {
        float t_left  = numeric_limits<float>::max();
        float t_right = numeric_limits<float>::max();

        bool intersects_left  = intersectNode(ray, BVH::nodes[node.left()], t_left);
        bool intersects_right = intersectNode(ray, BVH::nodes[node.right()], t_right);
        
        if (intersects_left && intersects_right) {
            if (t_left < t_right) {
                    traverseBVH(ray, BVH::nodes[node.left()], inter);
                    traverseBVH(ray, BVH::nodes[node.right()], inter);
            } else {
                    traverseBVH(ray, BVH::nodes[node.right()], inter);
                    traverseBVH(ray, BVH::nodes[node.left()], inter);
            }
        } else if (intersects_left) {
            traverseBVH(ray, BVH::nodes[node.left()], inter);
        } else if (intersects_right) {
            traverseBVH(ray, BVH::nodes[node.right()], inter);
        }
    }

    return inter;
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::getNearestIntersectionBVH                                        |
//  |  Get the nearest intersection with a primitive for the given ray.		LH2'19|
//  +-----------------------------------------------------------------------------+
Intersection RenderCore::getNearestIntersectionBVH(Ray &ray) {
    auto i = Intersection();
    return traverseBVH(ray, *bvh.root, i);
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::existNearerIntersection                                        |
//  |  Check whether there exists an intersection between a primitive and the	  |
//	|  given ray that is at least epsilon closer than the given distance.   LH2'19|
//  +-----------------------------------------------------------------------------+
bool RenderCore::existNearerIntersection(Ray ray, float distance) {
    // for (Mesh &mesh : meshes) {
    //     for (uint triangleIdx = 0; triangleIdx < mesh.vcount / 3; triangleIdx++) {
    //         float t = ray.calcIntersectDist(mesh.triangles[triangleIdx]);
    //         if (t < distance + EPSILON) { return true; }
    //     }
    // }
    // return false;
    return getNearestIntersectionBVH(ray).distance < distance;
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetMaterials                                                   |
//  |  Set the material data.                                               LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetMaterials(CoreMaterial *mat, const CoreMaterialEx *matEx, const int materialCount) {
    materials.clear();

    // copy the supplied array of materials
    for (int i = 0; i < materialCount; i++) {
        Material *m = new Material();
        m->color = make_float3(mat[i].diffuse_r, mat[i].diffuse_g, mat[i].diffuse_b);
        m->transmittance = make_float3(mat[i].transmittance_r, mat[i].transmittance_g, mat[i].transmittance_b);

        /* If any of the materials color component is set to >1, we classify it as an (area light).
		 * Otherwise, we use the metallic property to define the material.
		 * 0.0 means diffuse, 1.0 means mirror. Anything in between is (glass) dielectric.
		 */
        float metallicFactor = float((mat[i].parameters.x & (uint(0xFF) << 0)) >> 0) / 255.0f;
        if (m->color.x > 1 || m->color.y > 1 || m->color.z > 1) {
            m->type = MaterialType::LIGHT;
        } else if (metallicFactor == 0.0f) {
            m->type = MaterialType::DIFFUSE;
        } else if (metallicFactor == 1.0f) {
            m->type = MaterialType::MIRROR;
        } else {
            m->type = MaterialType::DIELECTRIC;
        }

        materials.push_back(m);
    }
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetLights                                                      |
//  |  Set the light data.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetLights(const CoreLightTri *_areaLights,
                           const int areaLightCount,
                           const CorePointLight *_pointLights,
                           const int pointLightCount,
                           const CoreSpotLight *_spotLights,
                           const int spotLightCount,
                           const CoreDirectionalLight *_directionalLights,
                           const int directionalLightCount) {
    // Clear all currently stored lights.
    pointLights.clear();
    spotLights.clear();
    dirLights.clear();
    areaLights.clear();

    // Load the new light data.
    pointLights.resize(pointLightCount);
    memcpy(&pointLights[0], _pointLights, pointLightCount * sizeof(CorePointLight));

    spotLights.resize(spotLightCount);
    memcpy(&spotLights[0], _spotLights, spotLightCount * sizeof(CoreSpotLight));

    dirLights.resize(directionalLightCount);
    memcpy(&dirLights[0], _directionalLights, directionalLightCount * sizeof(CoreDirectionalLight));

    areaLights.resize(areaLightCount);
    memcpy(&areaLights[0], _areaLights, areaLightCount * sizeof(CoreLightTri));
}

void RenderCore::SetSkyData(const float3 *pixels, const uint width, const uint height) {
    skyData.clear();
    skyData.resize(width * height);
    memcpy(&skyData[0], pixels, sizeof(float3) * width * height);

    skyWidth = width;
    skyHeight = height;
}

void RenderCore::resetAccBuffer() {
    accBuffer.clear();
    accBuffer.resize(screen->width * screen->height);
    memset(&accBuffer[0], 0, sizeof(float4) * screen->width * screen->height);
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Shutdown                                                       |
//  |  Free all resources.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Shutdown() {
    delete screen;
}

// EOF
