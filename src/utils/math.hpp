#pragma once

#include <cmath>
#include <immintrin.h>
#include <limits>

#include <glm/glm.hpp>

namespace iray {

    const __m256 one_m256       = _mm256_set1_ps(1.0f);
    const __m256 minus_one_m256 = _mm256_set1_ps(-1.0f);
    const __m256 pos_eps_m256   = _mm256_set1_ps(1e-9f);
    const __m256 neg_eps_m256   = _mm256_set1_ps(-1e-9f);
    const __m256 zero_m256      = _mm256_set1_ps(0.0f);
    const __m256 fmax_m256      = _mm256_set1_ps(std::numeric_limits<float>::max());

    bool ray_vs_aabb_avx2(const glm::vec3& orig, const glm::vec3& dir, const glm::vec3& vmin,
                          const glm::vec3& vmax, float& t) {

        // this is borked

        // inv_rd = 1 / rd
        __m256 inv_rd = _mm256_div_ps(
            one_m256, _mm256_set_ps(dir.x, dir.x, dir.y, dir.y, dir.z, dir.z, 1.f, 1.f));

        // vm = [ vmin.x, vmax.x, vmin.y, vmax.y ... ]
        __m256 vm = _mm256_set_ps(vmin.x, vmax.x, vmin.y, vmax.y, vmin.z, vmax.z, 1.f, 1.f);

        // ori = [ orig.x, orig.x, orig.y, orig.y, ...]
        __m256 ori = _mm256_set_ps(orig.x, orig.x, orig.y, orig.y, orig.z, orig.z, 1.f, 1.f);

        // sub = vm - ori
        __m256 sub = _mm256_sub_ps(vm, ori);

        // ts = sub * inv_rd
        __m256 ts = _mm256_mul_ps(sub, inv_rd);

        float __attribute__((aligned(32))) tsf[8];
        _mm256_store_ps(tsf, ts);

        tsf[6] = std::max(std::min(tsf[0], tsf[1]),
                          std::max(std::min(tsf[2], tsf[3]), std::min(tsf[4], tsf[5])));
        tsf[7] = std::min(std::max(tsf[0], tsf[1]),
                          std::min(std::max(tsf[2], tsf[3]), std::max(tsf[4], tsf[5])));

        t = tsf[7];

        return !(tsf[7] < 0.0 || tsf[7] < tsf[6]);
    }

    bool ray_vs_aabb(const glm::dvec3& orig, const glm::dvec3& dir, const glm::dvec3& vmin,
                     const glm::dvec3& vmax, double& t) {

        auto rdx = 1.0 / dir.x;
        auto rdy = 1.0 / dir.y;
        auto rdz = 1.0 / dir.z;

        auto t1 = (vmin.x - orig.x) * rdx;
        auto t2 = (vmax.x - orig.x) * rdx;
        auto t3 = (vmin.y - orig.y) * rdy;
        auto t4 = (vmax.y - orig.y) * rdy;
        auto t5 = (vmin.z - orig.z) * rdz;
        auto t6 = (vmax.z - orig.z) * rdz;

        auto t7 = std::max(std::min(t1, t2), std::max(std::min(t3, t4), std::min(t5, t6)));
        auto t8 = std::min(std::max(t1, t2), std::min(std::max(t3, t4), std::max(t5, t6)));

        if (t8 < 0.0 || t8 < t7)
            return false;

        t = t7;

        return true;
    }

    bool ray_vs_tri(const glm::dvec3& orig, const glm::dvec3& dir, const triangle& tri, double& t) {

        auto v0v1 = tri.p1.pos - tri.p0.pos;
        auto v0v2 = tri.p2.pos - tri.p0.pos;
        auto pvec = glm::cross(dir, v0v2);

        auto det = glm::dot(v0v1, pvec);

        if (std::fabs(det) < 1e-9)
            return false;

        auto inv_det = 1.0 / det;

        auto tvec = orig - tri.p0.pos;
        auto qvec = glm::cross(tvec, v0v1);

        auto u = glm::dot(tvec, pvec) * inv_det;
        auto v = glm::dot(dir, qvec) * inv_det;

        if ((u < 0) | (u > 1) | (v < 0) | (u + v > 1))
            return false;

        t = glm::dot(v0v2, qvec) * inv_det;

        return true;
    }

    // Returns true if the triangle p0 p1 p2 intersects an aabb
    bool tri_intersects_aabb(const glm::dvec3& p0, const glm::dvec3& p1, const glm::dvec3& p2,
                             const glm::dvec3& box_min, const glm::dvec3& box_max) {

        constexpr size_t X = 0;
        constexpr size_t Y = 1;
        constexpr size_t Z = 2;

        auto box_center = (box_min + box_max) * 0.5;
        auto box_extent = glm::abs(box_max - box_min);

        std::cout << box_extent.x << "\n";
        std::cout << box_extent.y << "\n";
        std::cout << box_extent.z << "\n";

        auto v0 = p0 - box_center;
        auto v1 = p1 - box_center;
        auto v2 = p2 - box_center;

        auto f0 = p1 - p0;
        auto f1 = p2 - p1;
        auto f2 = p0 - p2;

        auto axis_test = [box_extent, v0, v1, v2](const glm::dvec3& axis, const glm::dvec3& f,
                                                  const int axis0, const int axis1) {
            double a = glm::dot(v0, axis);
            double b = glm::dot(v1, axis);
            double c = glm::dot(v2, axis);

            auto r =
                box_extent[axis0] * std::abs(f[axis1]) + box_extent[axis1] * std::abs(f[axis0]);

            std::cout << a << "\t" << b << "\t" << c << "\t" << r << "\n";

            return std::max(-std::max(a, std::max(b, c)), std::min(a, std::min(b, c))) > r;
        };

        if (axis_test(glm::dvec3(0, -f0.z, f0.y), f0, Y, Z))
            return false;

        if (axis_test(glm::dvec3(0, -f1.z, f1.y), f1, Y, Z))
            return false;

        if (axis_test(glm::dvec3(0, -f2.z, f2.y), f2, Y, Z))
            return false;

        if (axis_test(glm::dvec3(f0.z, 0, -f0.x), f0, X, Z))
            return false;

        if (axis_test(glm::dvec3(f1.z, 0, -f1.x), f1, X, Z))
            return false;

        if (axis_test(glm::dvec3(f2.x, 0, -f2.x), f2, X, Z))
            return false;

        if (axis_test(glm::dvec3(-f0.y, f0.x, 0), f0, X, Y))
            return false;

        if (axis_test(glm::dvec3(-f1.y, f1.x, 0), f1, X, Y))
            return false;

        if (axis_test(glm::dvec3(-f2.y, f2.x, 0), f2, X, Y))
            return false;

        for (int n = 0; n < 3; n++)
            if (std::max(v0[n], std::max(v1[n], v2[n])) < -box_extent[n] ||
                std::min(v0[n], std::min(v1[n], v2[n])) > box_extent[n])
                return false;

        auto plane_normal = glm::cross(f0, f1);
        auto plane_dist   = glm::dot(plane_normal, v0);

        auto r = glm::dot(box_extent, glm::abs(plane_normal));

        if (plane_dist > r)
            return false;

        return true;
    }

    // Returns -1, 0, 1 if the triangle p0 p1 p2 is on the "left" side, is touching the plane,
    // or is on the "right" side of the plane. If all 3 points lie EXACTLY on the plane and
    // some sort of floating point omen occurs, it returns 1.
    int tri_plane_check(const glm::dvec3& p0, const glm::dvec3& p1, const glm::dvec3& p2,
                        const glm::dvec3& plane_point, const glm::dvec3& plane_normal) {

        auto dot0 = glm::dot(plane_normal, p0 - plane_point);
        auto dot1 = glm::dot(plane_normal, p1 - plane_point);
        auto dot2 = glm::dot(plane_normal, p2 - plane_point);

        auto sgn0 = std::signbit(dot0);
        auto sgn1 = std::signbit(dot1);
        auto sgn2 = std::signbit(dot2);

        if (sgn0 == sgn1 && sgn0 == sgn2 && sgn1 == sgn2)
            return sgn0 ? -1 : 1;

        return 0;
    }

    // Calculates the bounding box of a triangle
    void triangle_bbox(const triangle& tri, glm::dvec3& min, glm::dvec3& max) {
        max.x = std::max(tri.p0.pos.x, std::max(tri.p1.pos.x, tri.p2.pos.x));
        max.y = std::max(tri.p0.pos.y, std::max(tri.p1.pos.y, tri.p2.pos.y));
        max.z = std::max(tri.p0.pos.z, std::max(tri.p1.pos.z, tri.p2.pos.z));
        min.x = std::min(tri.p0.pos.x, std::min(tri.p1.pos.x, tri.p2.pos.x));
        min.y = std::min(tri.p0.pos.y, std::min(tri.p1.pos.y, tri.p2.pos.y));
        min.z = std::min(tri.p0.pos.z, std::min(tri.p1.pos.z, tri.p2.pos.z));
    }

} // namespace iray