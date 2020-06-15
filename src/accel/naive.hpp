#pragma once

#include <limits>
#include <vector>

#include <glm/glm.hpp>

#include "accel/accel_trait.hpp"
#include "accel/geometry.hpp"

namespace iray {

    // Naive "acceleration", just checks against all triangles

    template <>
    struct accelerator<accel_types::naive> {
        std::vector<triangle> triangles;

        accelerator(std::vector<triangle>&& tris) {
            triangles = tris;
            tris.clear();
        }

        bool intersects(ray& ray, intersection_result& res) {
            double min = std::numeric_limits<double>::max();
            bool   hit = false;

            triangle* tri_ptr = nullptr;

            for (auto& tri : triangles) {
                auto v0v1 = tri.p1.pos - tri.p0.pos;
                auto v0v2 = tri.p2.pos - tri.p0.pos;
                auto pvec = glm::cross(ray.direction, v0v2);

                auto det = glm::dot(v0v1, pvec);

                if (std::fabs(det) < 1e-9)
                    continue;

                auto inv_det = 1.0 / det;

                auto tvec = ray.origin - tri.p0.pos;
                auto qvec = glm::cross(tvec, v0v1);

                auto u = glm::dot(tvec, pvec) * inv_det;
                auto v = glm::dot(ray.direction, qvec) * inv_det;

                // bitwise OR so compiler doesnt generate an if statement for every check
                // this improves
                if ((u < 0) | (u > 1) | (v < 0) | (u + v > 1))
                    continue;

                auto t = glm::dot(v0v2, qvec) * inv_det;

                if (t < min) {
                    hit = true;

                    min     = t;
                    tri_ptr = &tri;
                }
            }

            if (!hit) {
                return false;
            }

            res.t         = min;
            res.hitpos    = ray.origin + ray.direction * min;
            res.hitnormal = tri_ptr->p0.normal;
            res.hit       = true;

            return true;
        };
    };

} // namespace iray
