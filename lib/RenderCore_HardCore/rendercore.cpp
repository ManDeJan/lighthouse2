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

using namespace lh2core;

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Init                                                           |
//  |  Initialization.                                                      LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Init() {
    // initialize core
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetTarget(GLTexture *target) {
    // synchronize OpenGL viewport
    targetTextureID = target->ID;
    if (screen != 0 && target->width == screen->width && target->height == screen->height) return; // nothing changed
    delete screen;
    screen = new Bitmap(target->width, target->height);
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
}

template <typename T>
T map(T x, T in_min, T in_max, T out_min, T out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Render                                                         |
//  |  Produce one image.                                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Render(const ViewPyramid &view, const Convergence converge) {
    screen->Clear();

    const auto nx = screen->width;
    const auto ny = screen->height;

    float dx = 1.0f / (nx - 1);
    float dy = 1.0f / (ny - 1);
    // print(">>> Start");

    float abs_min = numeric_limits<float>::max(), abs_max = numeric_limits<float>::min();

    for (int y = 0; y < ny; y++) {
        for (int x = 0; x < nx; x++) {
            float3 sx = x * dx * (view.p2 - view.p1); // screen widt
            float3 sy = y * dy * (view.p3 - view.p1); // screen height
            float3 point = view.p1 + sx + sy;         // point on the screen
            float3 dir = normalize(point - view.pos); // direction
            Ray ray = Ray(view.pos, dir);
            
            float t_min = numeric_limits<float>::max();
            for (Mesh &mesh : meshes) {
                for (int i = 0; i < mesh.vcount / 3; i++) {
                    auto t = intersect(ray, mesh.triangles[i]);
                    if (t && *t < t_min) {
                        t_min = *t;
                        // print(t_min);
                        // if (t != 0) print(t_min);
                        // abs_min = abs_min < t_min ? abs_min : t_min;
                        // abs_max = abs_max > t_min ? abs_max : t_min;
                        // print(abs_min, " ", abs_max);
                    }
                }
            }
            if (t_min != numeric_limits<float>::max()) {
                uint color = map(t_min, 0.0f, 10.0f, 255.0f, 0.0f);
                color = color << 16 | color << 8;// | color;
                screen->Plot(x, y, color);
            }
            // print(">>> pixel done... (", x, ")\n");
        }
        // print(">>> horizontal line done... (", y, ")\n");
    }

    // render minimal
    // screen->Clear();
    // for( Mesh& mesh : meshes ) for( int i = 0; i < mesh.vcount; i++ )
    // {
    // 	// convert a vertex position to a screen coordinate
    // 	int screenx = mesh.vertices[i].x / 80 * (float)screen->width + screen->width / 2;
    // 	int screeny = mesh.vertices[i].z / 80 * (float)screen->height + screen->height / 2;
    // 	screen->Plot( screenx, screeny, 0xffffff /* white */ );
    // }
    // // copy pixel buffer to OpenGL render target texture

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
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Shutdown                                                       |
//  |  Free all resources.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Shutdown() {
    delete screen;
}

// EOF
