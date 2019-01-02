
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
#include "foundation/utility/makevector.h"
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


namespace
{
    //
    // Settings.
    //

    #define BRUTE_FORCE_OCTREE_INTERSECTION


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
        bool on_frame_begin(
            const asr::Project&         project,
            const asr::BaseGroup*       parent,
            asr::OnFrameBeginRecorder&  recorder,
            asf::IAbortSwitch*          abort_switch) override
        {
            if (!asr::ProceduralObject::on_frame_begin(project, parent, recorder, abort_switch))
                return false;

            const asr::OnFrameBeginMessageContext context("object", this);

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

            // Retrieve parameters.
            const std::string mode =
                m_params.get_required<std::string>("mode", "flakes", asf::make_vector("flakes", "voxelization"));
            m_mode = mode == "flakes" ? Mode::Flakes : Mode::Voxelization;
            m_voxel_size = get_uncached_voxel_size();
            m_flake_size = m_params.get_required<float>("flake_size", 0.05f);
            m_flake_size_jitter = m_params.get_required<float>("flake_size_jitter", 0.2f);
            m_flake_center_jitter = m_params.get_required<double>("flake_center_jitter", 0.8);
            m_flakes_per_voxel = m_params.get_required<double>("flakes_per_voxel", 1.0);
            RENDERER_LOG_DEBUG("flakes: parameters:");
            RENDERER_LOG_DEBUG("  voxel size:           %f", m_voxel_size);
            RENDERER_LOG_DEBUG("  flake size:           %f", m_flake_size);
            RENDERER_LOG_DEBUG("  flake size jitter:    %f", m_flake_size_jitter);
            RENDERER_LOG_DEBUG("  flake center jitter:  %f", m_flake_center_jitter);
            RENDERER_LOG_DEBUG("  flakes per voxel:     %f", m_flakes_per_voxel);

            // Build octree.
            const asr::GAABB3 octree_root_aabb = compute_local_bbox();
            m_octree.reset(
                new Octree(
                    octree_root_aabb,
                    *base_object_instance,
                    m_voxel_size));

            // Print octree statistics.
            const size_t node_count = m_octree->get_node_count();
            const asf::uint64 solid_leaf_count = static_cast<asf::uint64>(m_octree->count_solid_leaves());
            const asf::uint64 total_flake_count = static_cast<asf::uint64>(solid_leaf_count * m_flakes_per_voxel);
            RENDERER_LOG_DEBUG("flakes: statistics:");
            RENDERER_LOG_DEBUG("  node count:           %s", asf::pretty_uint(node_count).c_str());
            RENDERER_LOG_DEBUG("  solid leaf count:     %s", asf::pretty_uint(solid_leaf_count).c_str());
            RENDERER_LOG_DEBUG("  total flake count:    %s", asf::pretty_uint(total_flake_count).c_str());

            // Initialize intersection statistics.
            m_cast_rays = 0;
            m_visited_voxels = 0;
            m_intersected_flakes = 0;

            return true;
        }

        void on_frame_end(
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
                const asr::GScalar voxel_size = get_uncached_voxel_size();
                bbox = base_object_instance->compute_parent_bbox();
                bbox.grow(asr::GVector3(voxel_size));
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

            switch (m_mode)
            {
              case Mode::Flakes:
                {
                    FlakesVisitor visitor(
                        result,
                        ray.m_tmin,
                        ray.m_tmax,
                        m_flake_size,
                        m_flake_size_jitter,
                        m_flake_center_jitter,
                        m_flakes_per_voxel);
                    m_octree->intersect(ray, visitor);
                }
                break;

              case Mode::Voxelization:
                {
                    VoxelizationVisitor visitor(
                        result,
                        ray.m_tmin,
                        ray.m_tmax);
                    m_octree->intersect(ray, visitor);
                }
                break;

              assert_otherwise;
            }

#if 0
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
            // todo: optimize.
            IntersectionResult result;
            intersect(ray, result);
            return result.m_hit;
        }

      private:
        class Octree
        {
          public:
            Octree(
                const asr::GAABB3&          root_aabb,
                const asr::ObjectInstance&  object_instance,
                const asr::GScalar          voxel_size)
              : m_root_aabb(root_aabb)
              , m_voxel_size(voxel_size)
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

            size_t get_node_count() const
            {
                return m_nodes.size();
            }

            size_t count_solid_leaves() const
            {
                return count_solid_leaves_recursive(0);
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
            const asr::GScalar  m_voxel_size;

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

                    if (asf::max_value(node_aabb.extent()) > m_voxel_size)
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

            size_t count_solid_leaves_recursive(const size_t node_index) const
            {
                const Node& node = m_nodes[node_index];

                if (node.is_leaf())
                    return node.is_solid() ? 1 : 0;
                else
                {
                    size_t count = 0;

                    const size_t child_node_index = node.get_child_node_index();

                    for (size_t i = 0; i < 8; ++i)
                        count += count_solid_leaves_recursive(child_node_index + i);

                    return count;
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
                    return
                        node.is_solid()
                            ? visitor.visit(org, dir, node_index, node_aabb, t_enter, t_leave)
                            : false;
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

                    const asf::AABB3d node_aabb(
                        asf::Vector3d(x0, y0, z0),
                        asf::Vector3d(x1, y1, z1));

                    const double t_enter = asf::max(tx0, ty0, tz0);
                    const double t_leave = asf::min(tx1, ty1, tz1);

                    return visitor.visit(org, dir, node_index, node_aabb, t_enter, t_leave);
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
                    return x < z ? a : c;
                else return y < z ? b : c;
            }
        };

        //
        // An octree visitor that renders a direct visualization of the octree.
        //

        class VoxelizationVisitor
        {
          public:
            VoxelizationVisitor(
                IntersectionResult&     result,
                const double            ray_tmin,
                const double            ray_tmax)
              : m_result(result)
              , m_ray_tmin(ray_tmin)
              , m_ray_tmax(ray_tmax)
            {
            }

            bool visit(
                const asf::Vector3d&    ray_org,
                const asf::Vector3d&    ray_dir,
                const size_t            leaf_index,
                const asf::AABB3d&      leaf_aabb,
                const double            t_enter,
                const double            t_leave)
            {
                if (t_enter < m_result.m_distance)
                {
                    const asf::Ray3d ray(ray_org, ray_dir, m_ray_tmin, m_ray_tmax);
                    const asf::RayInfo3d ray_info(ray);

                    double distance;
                    asf::Vector3d normal;

                    if (asf::intersect(ray, ray_info, leaf_aabb, distance, normal))
                    {
                        m_result.m_hit = true;
                        m_result.m_distance = distance;
                        m_result.m_geometric_normal = normal;
                        m_result.m_shading_normal = normal;
                        m_result.m_uv = asr::GVector2(0.0, 0.0);
                        m_result.m_material_slot = 0;
                    }
                }

                return m_result.m_hit;
            }

          private:
            IntersectionResult&         m_result;
            const double                m_ray_tmin;
            const double                m_ray_tmax;
        };

        //
        // An octree visitor that renders flakes (randomly oriented quads).
        //

        class FlakesVisitor
        {
          public:
            FlakesVisitor(
                IntersectionResult&     result,
                const double            ray_tmin,
                const double            ray_tmax,
                const float             flake_size,
                const float             flake_size_jitter,
                const double            flake_center_jitter,
                const double            flakes_per_voxel)
              : m_result(result)
              , m_ray_tmin(ray_tmin)
              , m_ray_tmax(ray_tmax)
              , m_flake_size(flake_size)
              , m_flake_size_jitter(flake_size_jitter)
              , m_flake_center_jitter(flake_center_jitter)
              , m_flakes_per_voxel(flakes_per_voxel)
            {
            }

            bool visit(
                const asf::Vector3d&    ray_org,
                const asf::Vector3d&    ray_dir,
                const size_t            leaf_index,
                const asf::AABB3d&      leaf_aabb,
                const double            t_enter,
                const double            t_leave)
            {
                const asf::Ray3d ray(ray_org, ray_dir);
                const asf::Vector3d leaf_center = leaf_aabb.center();
                const asf::Vector3d leaf_extent = leaf_aabb.extent();

                // Initialize RNG for this voxel.
                const asf::uint32 voxel_seed = asf::hash_uint64_to_uint32(leaf_index);
                asf::Xoroshiro128plus rng(voxel_seed, voxel_seed);

                // Force some mixing.
                rng.rand_uint32();

                bool found_hit = false;

                // Instantiate and intersect the flake(s) contained in this voxel.
                const size_t flake_count = asr::stochastic_cast<asf::Xoroshiro128plus, size_t, double>(rng, m_flakes_per_voxel);
                for (size_t flake_index = 0; flake_index < flake_count; ++flake_index)
                {
                    // IMPORTANT: All random numbers must be drawn unconditionally,
                    // regardless of whether the flake has been intersected or not.

                    // Compute the center of this flake.
                    const asf::Vector3d flake_center =
                        leaf_center +
                        leaf_extent * m_flake_center_jitter * asf::rand_vector2<asf::Vector3d>(rng);

                    // Compute the normal vector of this flake.
                    const asf::Vector3d flake_normal =
                        asf::sample_sphere_uniform(asf::rand_vector2<asf::Vector2d>(rng));
                    assert(asf::is_normalized(flake_normal));

                    // Compute the size of this flake.
                    const float flake_size_multiplier = 1.0f - m_flake_size_jitter * asf::rand_float2(rng, -1.0f, +1.0f);
                    const float flake_size = m_flake_size * flake_size_multiplier;
                    const float half_flake_size = 0.5f * flake_size;
                    const float rcp_flake_size = 1.0f / flake_size;
                    assert(flake_size > 0.0f);

                    // Randomly orient the flake around its normal.
                    const double flake_angle = asf::rand_double2(rng, 0.0, asf::TwoPi<double>());

                    // Reject this flake if the ray doesn't intersect its support plane.
                    double t_flake;
                    if (!asf::intersect(ray, flake_center, flake_normal, t_flake))
                        continue;

                    // Reject this flake if it's beyond the closest intersection found so far.
                    if (t_flake >= m_result.m_distance)
                        continue;

                    // Reject this flake if its intersection with the ray is outside the voxel.
                    const asf::Vector3d flake_hit = ray.point_at(t_flake);
                    if (!leaf_aabb.contains(flake_hit))
                        continue;

                    // Build a randomly oriented tangent frame around the flake's normal vector.
                    const asf::Basis3d flake_basis(flake_normal);
                    const asf::Quaterniond q = asf::Quaterniond::make_rotation(flake_normal, flake_angle);
                    const asf::Vector3d flake_tangent_u = asf::rotate(q, flake_basis.get_tangent_u());
                    const asf::Vector3d flake_tangent_v = asf::cross(flake_tangent_u, flake_normal);//asf::rotate(q, flake_basis.get_tangent_v());

                    // Compute UV coordinates of the hit point on the flake.
                    const asf::Vector3d center_to_hit = flake_hit - flake_center;
                    const float u = static_cast<float>(asf::dot(center_to_hit, flake_tangent_u));
                    const float v = static_cast<float>(asf::dot(center_to_hit, flake_tangent_v));

                    // Reject this flake if the hit point is outside its bounds.
                    if (u < -half_flake_size || u > half_flake_size &&
                        v < -half_flake_size || v > half_flake_size)
                        continue;

                    // An intersection with this flake has been found.
                    m_result.m_hit = true;
                    m_result.m_distance = t_flake;
                    m_result.m_geometric_normal = flake_normal;
                    m_result.m_shading_normal = flake_normal;
                    m_result.m_uv[0] = asf::saturate((u + half_flake_size) * rcp_flake_size);
                    m_result.m_uv[1] = asf::saturate((v + half_flake_size) * rcp_flake_size);
                    m_result.m_material_slot = 0;
                }

                return m_result.m_hit;
            }

          private:
            IntersectionResult&         m_result;
            const double                m_ray_tmin;
            const double                m_ray_tmax;
            const float                 m_flake_size;
            const float                 m_flake_size_jitter;
            const double                m_flake_center_jitter;
            const double                m_flakes_per_voxel;
        };

        enum class Mode { Flakes, Voxelization };

        Mode                            m_mode;
        asr::GScalar                    m_voxel_size;
        float                           m_flake_size;
        float                           m_flake_size_jitter;
        double                          m_flake_center_jitter;
        double                          m_flakes_per_voxel;

        std::unique_ptr<Octree>         m_octree;

        mutable boost::atomic<size_t>   m_cast_rays;
        mutable boost::atomic<size_t>   m_visited_voxels;
        mutable boost::atomic<size_t>   m_intersected_flakes;

        const asr::ObjectInstance* get_uncached_base_object_instance() const
        {
            return static_cast<const asr::ObjectInstance*>(m_inputs.get_entity("base_object_instance"));
        }

        asr::GScalar get_uncached_voxel_size() const
        {
            return m_params.get_required<asr::GScalar>("voxel_size", asr::GScalar(0.1));
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
                    .insert("name", "mode")
                    .insert("label", "Mode")
                    .insert("type", "enumeration")
                    .insert("items", asf::Dictionary()
                        .insert("Flakes", "flakes")
                        .insert("Voxelization", "voxelization"))
                    .insert("use", "required")
                    .insert("default", "flakes")
                    .insert("on_change", "rebuild_form"));

            metadata.push_back(
                asf::Dictionary()
                    .insert("name", "voxel_size")
                    .insert("label", "Voxel Size")
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

            metadata.push_back(
                asf::Dictionary()
                    .insert("name", "flake_size")
                    .insert("label", "Flake Size")
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
                    .insert("default", "0.05")
                    .insert("visible_if",
                        asf::Dictionary()
                            .insert("mode", "flakes")));

            metadata.push_back(
                asf::Dictionary()
                    .insert("name", "flake_size_jitter")
                    .insert("label", "Flake Size Jitter")
                    .insert("type", "numeric")
                    .insert("min",
                        asf::Dictionary()
                            .insert("value", "0.0")
                            .insert("type", "hard"))
                    .insert("max",
                        asf::Dictionary()
                            .insert("value", "1.0")
                            .insert("type", "hard"))
                    .insert("use", "required")
                    .insert("default", "0.2")
                    .insert("visible_if",
                        asf::Dictionary()
                            .insert("mode", "flakes")));

            metadata.push_back(
                asf::Dictionary()
                    .insert("name", "flake_center_jitter")
                    .insert("label", "Flake Center Jitter")
                    .insert("type", "numeric")
                    .insert("min",
                        asf::Dictionary()
                            .insert("value", "0.0")
                            .insert("type", "hard"))
                    .insert("max",
                        asf::Dictionary()
                            .insert("value", "1.0")
                            .insert("type", "hard"))
                    .insert("use", "required")
                    .insert("default", "0.8")
                    .insert("visible_if",
                        asf::Dictionary()
                            .insert("mode", "flakes")));

            metadata.push_back(
                asf::Dictionary()
                    .insert("name", "flakes_per_voxel")
                    .insert("label", "Flakes per Voxel")
                    .insert("type", "numeric")
                    .insert("min",
                        asf::Dictionary()
                            .insert("value", "0.0")
                            .insert("type", "hard"))
                    .insert("max",
                        asf::Dictionary()
                            .insert("value", "10.0")
                            .insert("type", "soft"))
                    .insert("use", "required")
                    .insert("default", "1.0"));

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
