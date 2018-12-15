
//
// This source file is part of appleseed.
// Visit https://appleseedhq.net/ for additional information and resources.
//
// This software is released under the MIT license.
//
// Copyright (c) 2018 Francois Beaune, The appleseedhq Organization
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

// appleseed.renderer headers.
#include "renderer/api/log.h"
#include "renderer/api/object.h"
#include "renderer/api/project.h"
#include "renderer/api/rendering.h"
#include "renderer/api/scene.h"
#include "renderer/api/types.h"
#include "renderer/api/utility.h"

// todo: fix.
#include "renderer/kernel/shading/shadingray.h"

// appleseed.foundation headers.
#include "foundation/math/aabb.h"
#include "foundation/math/basis.h"
#include "foundation/math/hash.h"
#include "foundation/math/intersection/aabbtriangle.h"
#include "foundation/math/intersection/rayaabb.h"
#include "foundation/math/intersection/rayplane.h"
#include "foundation/math/minmax.h"
#include "foundation/math/ray.h"
#include "foundation/math/rng/distribution.h"
#include "foundation/math/rng/xoroshiro128plus.h"
#include "foundation/math/sampling/mappings.h"
#include "foundation/math/scalar.h"
#include "foundation/math/vector.h"
#include "foundation/platform/atomic.h"
#include "foundation/platform/types.h"
#include "foundation/utility/api/specializedapiarrays.h"
#include "foundation/utility/containers/dictionary.h"
#include "foundation/utility/job/iabortswitch.h"
#include "foundation/utility/otherwise.h"
#include "foundation/utility/string.h"

// appleseed.main headers.
#include "main/dllvisibility.h"

// Standard headers.
#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

// Forward declarations.
namespace foundation { class SearchPaths; }

namespace asf = foundation;
namespace asr = renderer;

//#define BRUTE_FORCE_OCTREE_INTERSECTION

namespace
{
    //
    // Procedural flakes object.
    //

    const char* Model = "flakes_object";

    class FlakesObject
      : public asr::ProceduralObject
    {
      public:
        // Constructor.
        FlakesObject(
            const char*                 name,
            const asr::ParamArray&      params)
          : asr::ProceduralObject(name, params)
        {
            m_inputs.declare("base_object_instance", asr::InputFormatEntity, "");
        }

        // Delete this instance.
        void release() override
        {
            delete this;
        }

        // Return a string identifying this object model.
        const char* get_model() const override
        {
            return Model;
        }

        // This method is called once before rendering each frame.
        // Returns true on success, false otherwise.
        virtual bool on_frame_begin(
            const asr::Project&         project,
            const asr::BaseGroup*       parent,
            asr::OnFrameBeginRecorder&  recorder,
            asf::IAbortSwitch*          abort_switch) override
        {
            if (!asr::ProceduralObject::on_frame_begin(project, parent, recorder, abort_switch))
                return false;

            const asr::OnFrameBeginMessageContext context("object", this);

            m_shell_thickness = get_uncached_shell_thickness();

            // Retrieve base object instance.
            const asr::ObjectInstance* base_object_instance = get_uncached_base_object_instance();
            if (base_object_instance == nullptr)
            {
                // todo: make sure this message is necessary.
                RENDERER_LOG_ERROR("%sbase object instance not bound.", context.get());
                return false;
            }

            // Retrieve base object.
            const asr::Object& base_object = base_object_instance->get_object();
            const asr::MeshObject* mesh_base_object = dynamic_cast<const asr::MeshObject*>(&base_object);
            if (mesh_base_object == nullptr)
            {
                RENDERER_LOG_ERROR("%sbase object is not a mesh object.", context.get());
                return false;
            }

            // Build octree.
            const asr::GAABB3 octree_root_aabb = compute_local_bbox();
            m_octree.reset(new Octree(octree_root_aabb, *base_object_instance));

            // Initialize intersection statistics.
            m_cast_rays = 0;
            m_visited_voxels = 0;
            m_intersected_flakes = 0;

            return true;
        }

        virtual void on_frame_end(
            const asr::Project&         project,
            const asr::BaseGroup*       parent) override
        {
            const size_t cast_rays = m_cast_rays.load();

            RENDERER_LOG_DEBUG("flakes: cast rays: %s", asf::pretty_uint(cast_rays).c_str());
            RENDERER_LOG_DEBUG("flakes: avg visited voxels/ray: %s", asf::pretty_ratio(m_visited_voxels.load(), cast_rays).c_str());
            RENDERER_LOG_DEBUG("flakes: avg intersected flakes/ray: %s", asf::pretty_ratio(m_intersected_flakes.load(), cast_rays).c_str());
        }

        // Compute the local space bounding box of the object over the shutter interval.
        asr::GAABB3 compute_local_bbox() const override
        {
            asr::GAABB3 bbox;
            bbox.invalidate();

            const asr::ObjectInstance* base_object_instance = get_uncached_base_object_instance();

            if (base_object_instance != nullptr)
            {
                bbox = base_object_instance->compute_parent_bbox();

                const asr::GScalar shell_thickness = get_uncached_shell_thickness();
                bbox.grow(asr::GVector3(shell_thickness));
            }

            return bbox;
        }

        // Access materials slots.
        size_t get_material_slot_count() const override
        {
            return 1;
        }
        const char* get_material_slot(const size_t index) const override
        {
            return "default";
        }

        // Compute the intersection between a ray expressed in object space and
        // the surface of this object and return detailed intersection results.
        void intersect(
            const asr::ShadingRay&  ray,
            IntersectionResult&     result) const override
        {
            result.m_hit = false;
            result.m_distance = std::numeric_limits<double>::max();

            // Flakes are only visible to camera rays.
            if ((ray.m_flags & asr::VisibilityFlags::CameraRay) == 0)
                return;

            Visitor visitor(ray, result);
            m_octree->intersect(ray, visitor);

#if 0

            result.m_distance = std::numeric_limits<double>::max();
            result.m_material_slot = 0;

            constexpr double ShellThickness = 0.005;

            constexpr double FlakeSize = 0.005;
            constexpr double FlakeHalfSize = 0.5 * FlakeSize;
            constexpr size_t FlakesPerVoxel = 1;

            constexpr double VoxelSize = 0.01;
            constexpr double RcpVoxelSize = 1.0 / VoxelSize;

            static_assert(FlakeSize < VoxelSize, "Flakes must be smaller than voxels");

            const double outer_radius = m_radius + ShellThickness;
            const double inner_radius = m_radius;

            //
            // Intersect the outer sphere.
            //

            double outer_t[2];
            size_t outer_hit_count = intersect_sphere(ray, outer_radius, outer_t);

            if (outer_hit_count == 0)
                return;

            // if (outer_hit_count == 1)
            //     outer_t[outer_hit_count++] = ray.m_tmax;

            if (outer_hit_count == 1)
            {
                outer_t[1] = outer_t[0];
                outer_t[0] = ray.m_tmin;
                ++outer_hit_count;
            }

            assert(outer_hit_count == 2);

            //
            // Intersect the inner sphere.
            //

            double inner_t[2];
            const size_t inner_hit_count = intersect_sphere(ray, inner_radius, inner_t);

            if (inner_hit_count > 0)
            {
                if (outer_t[1] > inner_t[0])
                    outer_t[1] = inner_t[0];
            }

            const int step_ix = ray.m_dir.x >= 0 ? +1 : -1;
            const int step_iy = ray.m_dir.y >= 0 ? +1 : -1;
            const int step_iz = ray.m_dir.z >= 0 ? +1 : -1;

            const asf::Vector3d p_enter = ray.point_at(outer_t[0]);
            const asf::Vector3d p_exit = ray.point_at(outer_t[1]);

            int ix = static_cast<int>(asf::fast_floor(p_enter.x * RcpVoxelSize));
            int iy = static_cast<int>(asf::fast_floor(p_enter.y * RcpVoxelSize));
            int iz = static_cast<int>(asf::fast_floor(p_enter.z * RcpVoxelSize));

            const int exit_ix = static_cast<int>(asf::fast_floor(p_exit.x * RcpVoxelSize));
            const int exit_iy = static_cast<int>(asf::fast_floor(p_exit.y * RcpVoxelSize));
            const int exit_iz = static_cast<int>(asf::fast_floor(p_exit.z * RcpVoxelSize));

            //
            // p(t) = org + t * dir
            // p(t).x = org.x + t * dir.x
            //
            // p(tMaxX).x = | (ix + 1) * VoxelSize                 if step_ix > 0
            //              | ix * VoxelSize                       if step_ix < 0
            //
            // org.x + tMaxX * dir.x = | (ix + 1) * VoxelSize      if step_ix > 0
            //                         | ix * VoxelSize            if step_ix < 0
            //
            // tMaxX = | ((ix + 1) * VoxelSize - org.x) / dir.x    if step_ix > 0
            //         | (ix * VoxelSize - org.x) / dir.x          if step_ix < 0
            //

            const int next_ix = step_ix > 0 ? ix + 1 : ix;
            const int next_iy = step_iy > 0 ? iy + 1 : iy;
            const int next_iz = step_iz > 0 ? iz + 1 : iz;

            double t_max_x = (next_ix * VoxelSize - ray.m_org.x) / ray.m_dir.x;
            double t_max_y = (next_iy * VoxelSize - ray.m_org.y) / ray.m_dir.y;
            double t_max_z = (next_iz * VoxelSize - ray.m_org.z) / ray.m_dir.z;

            //
            // p(t) = org + t * dir
            // p(t).x = org.x + t * dir.x
            // p(0).x = org.x
            //
            // p(tDeltaX).x = org.x + step_ix * VoxelSize
            // org.x + tDeltaX * dir.x = org.x + step_ix * VoxelSize
            // tDeltaX * dir.x = step_ix * VoxelSize
            // tDeltaX = step_ix * VoxelSize / dir.x
            //

            const double t_delta_x = step_ix * VoxelSize / ray.m_dir.x;
            const double t_delta_y = step_iy * VoxelSize / ray.m_dir.y;
            const double t_delta_z = step_iz * VoxelSize / ray.m_dir.z;

            /*

            32 seconds
            2018-11-13T19:51:38.913710Z <003>   243 MB debug   | flakes: cast rays: 1,596,801
            2018-11-13T19:51:38.913710Z <003>   243 MB debug   | flakes: avg visited voxels/ray: 35.9
            2018-11-13T19:51:38.913710Z <003>   243 MB debug   | flakes: avg intersected flakes/ray: 3585.3

            27 seconds
            2018-11-13T19:58:21.386423Z <003>   243 MB debug   | flakes: cast rays: 1,596,801
            2018-11-13T19:58:21.386423Z <003>   243 MB debug   | flakes: avg visited voxels/ray: 30.0
            2018-11-13T19:58:21.386423Z <003>   243 MB debug   | flakes: avg intersected flakes/ray: 2995.1

            4.7 seconds
            2018-11-13T20:08:15.119767Z <003>   244 MB debug   | flakes: cast rays: 1,596,801
            2018-11-13T20:08:15.119767Z <003>   244 MB debug   | flakes: avg visited voxels/ray: 32.5
            2018-11-13T20:08:15.119767Z <003>   244 MB debug   | flakes: avg intersected flakes/ray: 390.5

            2.8 seconds
            2018-11-13T20:11:37.012395Z <003>   244 MB debug   | flakes: cast rays: 1,596,801
            2018-11-13T20:11:37.012395Z <003>   244 MB debug   | flakes: avg visited voxels/ray: 37.7
            2018-11-13T20:11:37.012395Z <003>   244 MB debug   | flakes: avg intersected flakes/ray: 75.4

            2.8 seconds
            2018-11-13T20:16:34.417170Z <004>   245 MB debug   | flakes: cast rays: 1,596,801
            2018-11-13T20:16:34.417170Z <004>   245 MB debug   | flakes: avg visited voxels/ray: 40.3
            2018-11-13T20:16:34.417170Z <004>   245 MB debug   | flakes: avg intersected flakes/ray: 40.3

            */

            std::unordered_set<asf::Vector3i, Vector3iHasher> voxels;

            while (true)
            {
                // Schedule visit of the nine voxels around (ix, iy, iz).
                for (int dz = -1; dz <= +1; ++dz)
                {
                    for (int dy = -1; dy <= +1; ++dy)
                    {
                        for (int dx = -1; dx <= +1; ++dx)
                        {
                            // if (voxel_count < MaxVoxels)
                            //     voxels[voxel_count++] = asf::Vector3i(ix + dx, iy + dy, iz + dz);
                            voxels.emplace(ix + dx, iy + dy, iz + dz);
                        }
                    }
                }

                // Advance to next voxel.
                if (t_max_x < t_max_y)
                {
                    if (t_max_x < t_max_z)
                    {
                        if (ix == exit_ix)
                            break;
                        ix += step_ix;
                        t_max_x += t_delta_x;
                    }
                    else
                    {
                        if (iz == exit_iz)
                            break;
                        iz += step_iz;
                        t_max_z += t_delta_z;
                    }
                }
                else
                {
                    if (t_max_y < t_max_z)
                    {
                        if (iy == exit_iy)
                            break;
                        iy += step_iy;
                        t_max_y += t_delta_y;
                    }
                    else
                    {
                        if (iz == exit_iz)
                            break;
                        iz += step_iz;
                        t_max_z += t_delta_z;
                    }
                }
            }

            for (const asf::Vector3i& voxel : voxels)
            {
                const int ix = voxel.x;
                const int iy = voxel.y;
                const int iz = voxel.z;

                const asf::AABB3d voxel_bbox(
                    asf::Vector3d(
                        ix * VoxelSize,
                        iy * VoxelSize,
                        iz * VoxelSize),
                    asf::Vector3d(
                        (ix + 1) * VoxelSize,
                        (iy + 1) * VoxelSize,
                        (iz + 1) * VoxelSize));

                // Initialize RNG for this voxel.
                const asf::uint32 voxel_seed =
                    asf::mix_uint32(
                        static_cast<asf::uint32>(ix),
                        static_cast<asf::uint32>(iy),
                        static_cast<asf::uint32>(iz));
                asf::Xoroshiro128plus rng(voxel_seed, voxel_seed);

                // Force some mixing.
                rng.rand_uint32();

                for (size_t flake_index = 0; flake_index < FlakesPerVoxel; ++flake_index)
                {
                    asf::Vector3d flake_center;
#if 1
                    flake_center.x = asf::rand_double2(rng, voxel_bbox.min.x, voxel_bbox.max.x);
                    flake_center.y = asf::rand_double2(rng, voxel_bbox.min.y, voxel_bbox.max.y);
                    flake_center.z = asf::rand_double2(rng, voxel_bbox.min.z, voxel_bbox.max.z);
#else
                    flake_center.x = (ix + 0.5) * VoxelSize;
                    flake_center.y = (iy + 0.5) * VoxelSize;
                    flake_center.z = (iz + 0.5) * VoxelSize;
#endif

                    const asf::Vector2d s = asf::rand_vector2<asf::Vector2d>(rng);
                    const asf::Vector3d flake_normal = asf::sample_sphere_uniform(s);
                    // const asf::Vector3d flake_normal = asf::Basis3d(asf::normalize(flake_center)).get_tangent_u();
                    assert(asf::is_normalized(flake_normal));

                    double t_flake;
                    if (asf::intersect(ray, flake_center, flake_normal, t_flake))
                    {
                        if (t_flake >= outer_t[0] &&
                            t_flake < outer_t[1] &&
                            t_flake < result.m_distance)
                        {
                            const asf::Vector3d flake_hit = ray.point_at(t_flake);
                            const asf::Vector3d center_to_hit = flake_hit - flake_center;

                            // todo: randomize flake's rotation around its normal.
                            const asf::Basis3d flake_basis(flake_normal);

                            const double u = asf::dot(center_to_hit, flake_basis.get_tangent_u());
                            const double v = asf::dot(center_to_hit, flake_basis.get_tangent_v());

                            if (u >= -FlakeHalfSize && u <= FlakeHalfSize &&
                                v >= -FlakeHalfSize && v <= FlakeHalfSize)
                            {
                                result.m_hit = true;
                                result.m_distance = t_flake;
                                result.m_geometric_normal = flake_normal;
                                result.m_shading_normal = flake_normal;
                                result.m_uv[0] = asf::saturate(static_cast<float>((u + FlakeHalfSize) / FlakeSize));
                                result.m_uv[1] = asf::saturate(static_cast<float>((v + FlakeHalfSize) / FlakeSize));

                                if (voxel_bbox.contains(flake_hit))
                                    goto done;
                            }
                        }
                    }
                }
            }

        done:

            // Update statistics.
            ++m_cast_rays;
            m_visited_voxels += voxels.size();
            m_intersected_flakes += FlakesPerVoxel * voxels.size();

#endif
        }

        // Compute the intersection between a ray expressed in object space and
        // the surface of this object and simply return whether there was a hit.
        bool intersect(
            const asr::ShadingRay&  ray) const override
        {
            // todo: implement.
            return false;
        }

      private:
        class Octree
        {
          public:
            Octree(const asr::GAABB3& root_aabb, const asr::ObjectInstance& object_instance)
              : m_root_aabb(root_aabb)
            {
                const asr::MeshObject& object = static_cast<const asr::MeshObject&>(object_instance.get_object());
                const asf::Transformd& transform = object_instance.get_transform();

                Node root_node;
                root_node.make_leaf();
                root_node.set_solid_bit(true);
                m_nodes.push_back(root_node);

                for (size_t i = 0, e = object.get_triangle_count(); i < e; ++i)
                {
                    const asr::Triangle& triangle = object.get_triangle(i);
                    const asr::GVector3 v0 = transform.point_to_parent(object.get_vertex(triangle.m_v0));
                    const asr::GVector3 v1 = transform.point_to_parent(object.get_vertex(triangle.m_v1));
                    const asr::GVector3 v2 = transform.point_to_parent(object.get_vertex(triangle.m_v2));
                    push_triangle(0, 0, m_root_aabb, v0, v1, v2);
                }
            }

            template <typename Visitor>
            void intersect(const asf::Ray3d& ray, Visitor& visitor) const
            {
#ifdef BRUTE_FORCE_OCTREE_INTERSECTION
                intersect_recursive(
                    0,
                    ray.m_org, ray.m_dir,
                    m_root_aabb.min.x, m_root_aabb.min.y, m_root_aabb.min.z,
                    m_root_aabb.max.x, m_root_aabb.max.y, m_root_aabb.max.z,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0, visitor);
#else
                asf::Vector3d org = ray.m_org;
                asf::Vector3d dir = ray.m_dir;
                size_t a = 0;

                if (dir.x < 0.0)
                {
                    org.x = (m_root_aabb.min.x + m_root_aabb.max.x) - org.x;
                    dir.x = -dir.x;
                    a |= 4;
                }

                if (dir.y < 0.0)
                {
                    org.y = (m_root_aabb.min.y + m_root_aabb.max.y) - org.y;
                    dir.y = -dir.y;
                    a |= 2;
                }

                if (dir.z < 0.0)
                {
                    org.z = (m_root_aabb.min.z + m_root_aabb.max.z) - org.z;
                    dir.z = -dir.z;
                    a |= 1;
                }

                const double x0 = m_root_aabb.min.x;
                const double y0 = m_root_aabb.min.y;
                const double z0 = m_root_aabb.min.z;
                const double x1 = m_root_aabb.max.x;
                const double y1 = m_root_aabb.max.y;
                const double z1 = m_root_aabb.max.z;

                const double tx0 = (x0 - org.x) / dir.x;
                const double tx1 = (x1 - org.x) / dir.x;
                const double ty0 = (y0 - org.y) / dir.y;
                const double ty1 = (y1 - org.y) / dir.y;
                const double tz0 = (z0 - org.z) / dir.z;
                const double tz1 = (z1 - org.z) / dir.z;

                if (asf::max(tx0, ty0, tz0) < asf::min(tx1, ty1, tz1))
                    intersect_recursive(0, org, dir, x0, y0, z0, x1, y1, z1, tx0, ty0, tz0, tx1, ty1, tz1, a, visitor);
#endif
            }

          private:
            class Node
            {
              public:
                // Set/get the node type.
                void make_interior();
                void make_leaf();
                bool is_interior() const;
                bool is_leaf() const;

                // Set/get the solid bit.
                void set_solid_bit(const bool solid);
                bool is_empty() const;
                bool is_solid() const;

                // Set/get the child node index (interior nodes only).
                void set_child_node_index(const size_t index);
                size_t get_child_node_index() const;

              private:
                //
                // The info field of the node is organized as follow:
                //
                //   interior node:
                //
                //     bits 0-30    index of the first child node
                //     bit  31      0
                //
                //   leaf node:
                //
                //     bits 0-29    unused
                //     bit  30      solid bit (0 for empty, 1 for solid)
                //     bit  31      1
                //
                // The maximum size of a single octree is 2^30 = 1,073,741,824 nodes.
                //

                asf::uint32     m_info;
            };

            const asr::GAABB3   m_root_aabb;
            std::vector<Node>   m_nodes;

            void push_triangle(
                const size_t            node_index,
                const size_t            node_depth,
                const asr::GAABB3&      node_aabb,
                const asr::GVector3&    v0,
                const asr::GVector3&    v1,
                const asr::GVector3&    v2)
            {
                assert(asf::intersect(node_aabb, v0, v1, v2));

                if (m_nodes[node_index].is_leaf())
                {
                    m_nodes[node_index].set_solid_bit(true);

                    //const bool must_subdivide =
                    //    asf::max_value(node_aabb.extent()) > asr::GScalar(0.1);
                    const bool must_subdivide = node_depth < 6;

                    if (must_subdivide)
                    {
                        m_nodes[node_index].make_interior();
                        m_nodes[node_index].set_child_node_index(m_nodes.size());

                        for (size_t i = 0; i < 8; ++i)
                        {
                            Node child_node;
                            child_node.make_leaf();
                            child_node.set_solid_bit(false);
                            m_nodes.push_back(child_node);
                        }
                    }
                }

                if (m_nodes[node_index].is_interior())
                {
                    const size_t child_node_index = m_nodes[node_index].get_child_node_index();
                    const size_t child_node_depth = node_depth + 1;
                    const asr::GVector3 node_center = node_aabb.center();

                    for (size_t i = 0; i < 8; ++i)
                    {
                        asr::GAABB3 child_node_aabb;
                        for (size_t d = 0; d < 3; ++d)
                        {
#ifdef BRUTE_FORCE_OCTREE_INTERSECTION
                            const size_t side = i & (1ULL << d);
#else
                            // Account for the differences in child nodes numbering.
                            const size_t side = i & (1ULL << (2 - d));
#endif
                            child_node_aabb.min[d] = side == 0 ? node_aabb.min[d] : node_center[d];
                            child_node_aabb.max[d] = side == 0 ? node_center[d] : node_aabb.max[d];
                        }

                        if (asf::intersect(child_node_aabb, v0, v1, v2))
                        {
                            push_triangle(
                                child_node_index + i,
                                child_node_depth,
                                child_node_aabb,
                                v0, v1, v2);
                        }
                    }
                }
            }

            // Return whether an intersection was found.
            template <typename Visitor>
            bool intersect_recursive(
                const size_t            node_index,
                const asf::Vector3d&    org,
                const asf::Vector3d&    dir,
                const double            x0,
                const double            y0,
                const double            z0,
                const double            x1,
                const double            y1,
                const double            z1,
                const double            tx0,
                const double            ty0,
                const double            tz0,
                const double            tx1,
                const double            ty1,
                const double            tz1,
                const size_t            a,
                Visitor&                visitor) const
            {
#ifdef BRUTE_FORCE_OCTREE_INTERSECTION
                const asf::Ray3d ray(org, dir);
                const asf::RayInfo3d ray_info(ray);
                const asf::AABB3d node_aabb(asf::Vector3d(x0, y0, z0), asf::Vector3d(x1, y1, z1));

                // Check if the ray intersects the bounding box of this node.
                double t_enter, t_leave;
                if (!asf::intersect(ray, ray_info, node_aabb, t_enter, t_leave))
                    return false;

                // Fetch the node.
                const Octree::Node& node = m_nodes[node_index];

                if (node.is_leaf())
                {
                    return node.is_solid() ? visitor.visit(node_aabb, t_enter, t_leave) : false;
                }
                else
                {
                    const asf::Vector3d node_center = node_aabb.center();
                    const size_t child_node_index = m_nodes[node_index].get_child_node_index();
                    bool found_hit = false;

                    for (size_t i = 0; i < 8; ++i)
                    {
                        asf::AABB3d child_node_aabb;
                        for (size_t d = 0; d < 3; ++d)
                        {
                            const size_t side = i & (1ULL << d);
                            child_node_aabb.min[d] = side == 0 ? node_aabb.min[d] : node_center[d];
                            child_node_aabb.max[d] = side == 0 ? node_center[d] : node_aabb.max[d];
                        }

                        if (intersect_recursive(
                                child_node_index + i,
                                org, dir,
                                child_node_aabb.min.x, child_node_aabb.min.y, child_node_aabb.min.z,
                                child_node_aabb.max.x, child_node_aabb.max.y, child_node_aabb.max.z,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0, visitor))
                            found_hit = true;
                    }

                    return found_hit;
                }
#else
                // Exit if the ray does not intersect the voxel.
                if (tx1 < 0.0 || ty1 < 0.0 || tz1 < 0.0)
                    return false;

                // Fetch the node.
                const Octree::Node& node = m_nodes[node_index];

                // If the node is a leaf, visit it.
                if (node.is_leaf())
                {
                    if (!node.is_solid())
                        return false;

                    const asf::AABB3d leaf_bbox(
                        asf::Vector3d(x0, y0, z0),
                        asf::Vector3d(x1, y1, z1));

                    const double t_enter = asf::max(tx0, ty0, tz0);
                    const double t_leave = asf::min(tx1, ty1, tz1);

                    return visitor.visit(leaf_bbox, t_enter, t_leave);
                }

                // Compute the center of the node.
                const double xm = 0.5 * (x0 + x1);
                const double ym = 0.5 * (y0 + y1);
                const double zm = 0.5 * (z0 + z1);

                // Compute the intersection between the ray and the three middle planes.
                const double txm = dir.x != 0.0 ? 0.5 * (tx0 + tx1) : org.x < xm ? asf::FP<double>::pos_inf() : asf::FP<double>::neg_inf();
                const double tym = dir.y != 0.0 ? 0.5 * (ty0 + ty1) : org.y < ym ? asf::FP<double>::pos_inf() : asf::FP<double>::neg_inf();
                const double tzm = dir.z != 0.0 ? 0.5 * (tz0 + tz1) : org.z < zm ? asf::FP<double>::pos_inf() : asf::FP<double>::neg_inf();

                // Find the initial octant to visit.
                size_t current_octant = 0;
                if (tx0 > ty0)
                {
                    if (tx0 > tz0)
                    {
                        // max(tx0, ty0, tz0) is tx0: entry plane is YZ.
                        if (tym < tx0) current_octant |= 2;
                        if (tzm < tx0) current_octant |= 1;
                    }
                    else
                    {
                        // max(tx0, ty0, tz0) is tz0: entry plane is XY.
                        if (txm < tz0) current_octant |= 4;
                        if (tym < tz0) current_octant |= 2;
                    }
                }
                else
                {
                    if (ty0 > tz0)
                    {
                        // max(tx0, ty0, tz0) is ty0: entry plane is XZ.
                        if (txm < ty0) current_octant |= 4;
                        if (tzm < ty0) current_octant |= 1;
                    }
                    else
                    {
                        // max(tx0, ty0, tz0) is tz0: entry plane is XY.
                        if (txm < tz0) current_octant |= 4;
                        if (tym < tz0) current_octant |= 2;
                    }
                }

                // Visit child nodes.
                const size_t first_child_node_index = node.get_child_node_index();
                const size_t End = 8;
                while (true)
                {
                    switch (current_octant)
                    {
                      case 0:
                        if (intersect_recursive(
                                first_child_node_index + a,
                                org, dir,
                                x0, y0, z0, xm, ym, zm,
                                tx0, ty0, tz0, txm, tym, tzm,
                                a, visitor))
                            return true;
                        current_octant = choose_next_octant(txm, tym, tzm, 4, 2, 1);
                        break;

                      case 1:
                        if (intersect_recursive(
                                first_child_node_index + (1 ^ a),
                                org, dir,
                                x0, y0, zm, xm, ym, z1,
                                tx0, ty0, tzm, txm, tym, tz1,
                                a, visitor))
                            return true;
                        current_octant = choose_next_octant(txm, tym, tz1, 5, 3, End);
                        break;

                      case 2:
                        if (intersect_recursive(
                                first_child_node_index + (2 ^ a),
                                org, dir,
                                x0, ym, z0, xm, y1, zm,
                                tx0, tym, tz0, txm, ty1, tzm,
                                a, visitor))
                            return true;
                        current_octant = choose_next_octant(txm, ty1, tzm, 6, End, 3);
                        break;

                      case 3:
                        if (intersect_recursive(
                                first_child_node_index + (3 ^ a),
                                org, dir,
                                x0, ym, zm, xm, y1, z1,
                                tx0, tym, tzm, txm, ty1, tz1,
                                a, visitor))
                            return true;
                        current_octant = choose_next_octant(txm, ty1, tz1, 7, End, End);
                        break;

                      case 4:
                        if (intersect_recursive(
                                first_child_node_index + (4 ^ a),
                                org, dir,
                                xm, y0, z0, x1, ym, zm,
                                txm, ty0, tz0, tx1, tym, tzm,
                                a, visitor))
                            return true;
                        current_octant = choose_next_octant(tx1, tym, tzm, End, 6, 5);
                        break;

                      case 5:
                        if (intersect_recursive(
                                first_child_node_index + (5 ^ a),
                                org, dir,
                                xm, y0, zm, x1, ym, z1,
                                txm, ty0, tzm, tx1, tym, tz1,
                                a, visitor))
                            return true;
                        current_octant = choose_next_octant(tx1, tym, tz1, End, 7, End);
                        break;

                      case 6:
                        if (intersect_recursive(
                                first_child_node_index + (6 ^ a),
                                org, dir,
                                xm, ym, z0, x1, y1, zm,
                                txm, tym, tz0, tx1, ty1, tzm,
                                a, visitor))
                            return true;
                        current_octant = choose_next_octant(tx1, ty1, tzm, End, End, 7);
                        break;

                      case 7:
                        if (intersect_recursive(
                                first_child_node_index + (7 ^ a),
                                org, dir,
                                xm, ym, zm, x1, y1, z1,
                                txm, tym, tzm, tx1, ty1, tz1,
                                a, visitor))
                            return true;
                        return false;

                      case End:
                        return false;
                    }
                }
#endif
            }

            static size_t choose_next_octant(
                const double            x,
                const double            y,
                const double            z,
                const size_t            a,
                const size_t            b,
                const size_t            c)
            {
                if (x < y)
                {
                    return x < z ? a : c;
                }
                else
                {
                    return y < z ? b : c;
                }
            }
        };

        class Visitor
        {
          public:
            Visitor(
                const asr::ShadingRay&  ray,
                IntersectionResult&     result)
              : m_ray(ray)
              , m_result(result)
            {
            }

            bool visit(
                const asf::AABB3d&      leaf_bbox,
                const double            t_enter,
                const double            t_leave)
            {
                if (t_enter < m_result.m_distance)
                {
                    m_result.m_hit = true;
                    m_result.m_distance = t_enter;
                    m_result.m_geometric_normal = -m_ray.m_dir;
                    m_result.m_shading_normal = -m_ray.m_dir;
                    m_result.m_uv = asr::GVector2(0.0, 0.0);
                    m_result.m_material_slot = 0;
                }

                return m_result.m_hit;
            }

          private:
            struct Vector3iHasher
            {
                size_t operator()(const asf::Vector3i& v) const
                {
                    return asf::mix_uint32(v[0], v[1], v[2]);
                }
            };

            const asr::ShadingRay&      m_ray;
            IntersectionResult&         m_result;
        };

        asr::GScalar                    m_shell_thickness;

        std::unique_ptr<Octree>         m_octree;

        mutable boost::atomic<size_t>   m_cast_rays;
        mutable boost::atomic<size_t>   m_visited_voxels;
        mutable boost::atomic<size_t>   m_intersected_flakes;

        const asr::ObjectInstance* get_uncached_base_object_instance() const
        {
            return static_cast<const asr::ObjectInstance*>(m_inputs.get_entity("base_object_instance"));
        }

        asr::GScalar get_uncached_shell_thickness() const
        {
            return m_params.get_required<asr::GScalar>("shell_thickness", asr::GScalar(0.1));
        }
    };


    //
    // Factory for the new object model.
    //

    class FlakesObjectFactory
      : public asr::IObjectFactory
    {
      public:
        // Delete this instance.
        void release() override
        {
            delete this;
        }

        // Return a string identifying this object model.
        const char* get_model() const override
        {
            return Model;
        }

        // Return metadata for this object model.
        asf::Dictionary get_model_metadata() const override
        {
            return
                asf::Dictionary()
                    .insert("name", Model)
                    .insert("label", "Flakes Object");
        }

        // Return metadata for the inputs of this object model.
        asf::DictionaryArray get_input_metadata() const override
        {
            asf::DictionaryArray metadata;

            metadata.push_back(
                asf::Dictionary()
                    .insert("name", "base_object_instance")
                    .insert("label", "Base Object Instance")
                    .insert("type", "entity")
                    .insert("entity_types",
                        asf::Dictionary().insert("object_insgtance", "Object Instances"))
                    .insert("use", "required"));

            metadata.push_back(
                asf::Dictionary()
                    .insert("name", "shell_thickness")
                    .insert("label", "Shell Thickness")
                    .insert("type", "numeric")
                    .insert("min",
                        asf::Dictionary()
                            .insert("value", "0.0")
                            .insert("type", "hard"))
                    .insert("max",
                        asf::Dictionary()
                            .insert("value", "1.0")
                            .insert("type", "soft"))
                    .insert("use", "required")
                    .insert("default", "0.1"));

            return metadata;
        }

        // Create a new single empty object.
        asf::auto_release_ptr<asr::Object> create(
            const char*                 name,
            const asr::ParamArray&      params) const override
        {
            return asf::auto_release_ptr<asr::Object>(new FlakesObject(name, params));
        }

        // Create objects, potentially from external assets.
        bool create(
            const char*                 name,
            const asr::ParamArray&      params,
            const asf::SearchPaths&     search_paths,
            const bool                  omit_loading_assets,
            asr::ObjectArray&           objects) const override
        {
            objects.push_back(create(name, params).release());
            return true;
        }
    };


    //
    // FlakesObject::Octree::Node class implementation.
    //

    inline void FlakesObject::Octree::Node::make_interior()
    {
        m_info &= 0x7FFFFFFFUL;
    }

    inline void FlakesObject::Octree::Node::make_leaf()
    {
        m_info |= 0x80000000UL;
    }

    inline bool FlakesObject::Octree::Node::is_interior() const
    {
        return (m_info & 0x80000000UL) == 0;
    }

    inline bool FlakesObject::Octree::Node::is_leaf() const
    {
        return (m_info & 0x80000000UL) != 0;
    }

    inline void FlakesObject::Octree::Node::set_solid_bit(const bool solid)
    {
        assert(is_leaf());
        if (solid)
             m_info |= 0x40000000UL;
        else m_info &= 0xBFFFFFFFUL;
    }

    inline bool FlakesObject::Octree::Node::is_empty() const
    {
        assert(is_leaf());
        return (m_info & 0x40000000UL) == 0;
    }

    inline bool FlakesObject::Octree::Node::is_solid() const
    {
        assert(is_leaf());
        return (m_info & 0x40000000UL) != 0;
    }

    inline void FlakesObject::Octree::Node::set_child_node_index(const size_t index)
    {
        assert(is_interior());
        assert(index < (1UL << 30));
        m_info &= 0x80000000UL;
        m_info |= static_cast<asf::uint32>(index);
    }

    inline size_t FlakesObject::Octree::Node::get_child_node_index() const
    {
        assert(is_interior());
        return static_cast<size_t>((m_info & 0x7FFFFFFFUL));
    }
}


//
// Plugin entry point.
//

extern "C"
{
    APPLESEED_DLL_EXPORT asr::IObjectFactory* appleseed_create_object_factory()
    {
        return new FlakesObjectFactory();
    }
}
