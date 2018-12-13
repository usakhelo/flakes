
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

// todo: fix.
#include "renderer/kernel/shading/shadingray.h"

// appleseed.foundation headers.
#include "foundation/math/aabb.h"
#include "foundation/math/basis.h"
#include "foundation/math/hash.h"
#include "foundation/math/intersection/rayplane.h"
#include "foundation/math/ray.h"
#include "foundation/math/rng/distribution.h"
#include "foundation/math/rng/xoroshiro128plus.h"
#include "foundation/math/sampling/mappings.h"
#include "foundation/math/scalar.h"
#include "foundation/math/vector.h"
#include "foundation/platform/atomic.h"
#include "foundation/utility/api/specializedapiarrays.h"
#include "foundation/utility/containers/dictionary.h"
#include "foundation/utility/job/iabortswitch.h"
#include "foundation/utility/string.h"

// appleseed.main headers.
#include "main/dllvisibility.h"

// Standard headers.
#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <utility>

// Forward declarations.
namespace foundation { class SearchPaths; }

namespace asf = foundation;
namespace asr = renderer;

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

            m_radius = get_uncached_radius();
            m_rcp_radius = 1.0 / m_radius;

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
            const auto r = static_cast<asr::GScalar>(get_uncached_radius());
            return asr::GAABB3(asr::GVector3(-r), asr::GVector3(r));
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

            // Flakes are only visible to camera rays.
            if ((ray.m_flags & asr::VisibilityFlags::CameraRay) == 0)
                return;

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
        }

        // Compute the intersection between a ray expressed in object space and
        // the surface of this object and simply return whether there was a hit.
        bool intersect(
            const asr::ShadingRay&  ray) const override
        {
            return false;

            /*const double Epsilon = 1.0e-6;

            const double a = asf::dot(ray.m_org, ray.m_dir);
            const double b = asf::square(a) - dot(ray.m_org, ray.m_org) + asf::square(m_radius);

            if (b < 0.0)
                return false;

            const double c = std::sqrt(b);

            const double t1 = -a - c;
            if (t1 >= std::max(ray.m_tmin, Epsilon) && t1 < ray.m_tmax)
                return true;

            const double t2 = -a + c;
            if (t2 >= std::max(ray.m_tmin, Epsilon) && t2 < ray.m_tmax)
                return true;

            return false;*/
        }

      private:
        struct Vector3iHasher
        {
            size_t operator()(const asf::Vector3i& v) const
            {
                return asf::mix_uint32(v[0], v[1], v[2]);
            }
        };

        double                          m_radius;
        double                          m_rcp_radius;

        mutable boost::atomic<size_t>   m_cast_rays;
        mutable boost::atomic<size_t>   m_visited_voxels;
        mutable boost::atomic<size_t>   m_intersected_flakes;

        double get_uncached_radius() const
        {
            return m_params.get_optional<double>("radius");
        }

        static size_t intersect_sphere(
            const asr::ShadingRay&  ray,
            const double            radius,
            double                  t_out[2])
        {
            const double Epsilon = 1.0e-6;

            size_t hit_count = 0;

            const double a = asf::dot(ray.m_org, ray.m_dir);
            const double b = asf::square(a) - dot(ray.m_org, ray.m_org) + asf::square(radius);

            if (b < 0.0)
                return hit_count;

            const double c = std::sqrt(b);

            double t = -a - c;
            if (t >= std::max(ray.m_tmin, Epsilon) && t < ray.m_tmax)
                t_out[hit_count++] = t;

            t = -a + c;
            if (t >= std::max(ray.m_tmin, Epsilon) && t < ray.m_tmax)
                t_out[hit_count++] = t;

            if (hit_count == 2)
            {
                if (t_out[0] > t_out[1])
                    std::swap(t_out[0], t_out[1]);
            }

            return hit_count;
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
                    .insert("name", "radius")
                    .insert("label", "Radius")
                    .insert("type", "numeric")
                    .insert("min",
                        asf::Dictionary()
                            .insert("value", "0.0")
                            .insert("type", "hard"))
                    .insert("max",
                        asf::Dictionary()
                            .insert("value", "10.0")
                            .insert("type", "soft"))
                    .insert("use", "optional")
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
