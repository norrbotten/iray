#pragma once

#include <immintrin.h>
#include <iostream>
#include <limits>
#include <optional>
#include <vector>

#include <glm/glm.hpp>

#include "accel/accel_trait.hpp"
#include "accel/geometry.hpp"

namespace iray {

    // Naive "accelerator" using AVX2, intersects 8 triangles at once
    // Note that this uses floats so it wont be as precise
    // https://stackoverflow.com/questions/45599766/fast-sse-ray-4-triangle-intersection

    struct avx2_packed_triangles {
        __m256 edge1[3]; // v1 - v0
        __m256 edge2[3]; // v2 - v0
        __m256 vert0[3];
        __m256 inactive;

        // not used in intersection code, but still need normals
        __m256 normal[3];
    };

    struct avx2_packed_ray {
        __m256 origin[3];
        __m256 direction[3];
    };

    inline void avx2_cross(__m256 result[3], const __m256 a[3], const __m256 b[3]) {
        result[0] = _mm256_fmsub_ps(a[1], b[2], _mm256_mul_ps(b[1], a[2]));
        result[1] = _mm256_fmsub_ps(a[2], b[0], _mm256_mul_ps(b[2], a[0]));
        result[2] = _mm256_fmsub_ps(a[0], b[1], _mm256_mul_ps(b[0], a[1]));
    }

    inline __m256 avx2_dot(const __m256 a[3], const __m256 b[3]) {
        return _mm256_fmadd_ps(a[2], b[2], _mm256_fmadd_ps(a[1], b[1], _mm256_mul_ps(a[0], b[0])));
    }

    inline void avx2_sub(__m256 result[3], const __m256 a[3], const __m256 b[3]) {
        result[0] = _mm256_sub_ps(a[0], b[0]);
        result[1] = _mm256_sub_ps(a[1], b[1]);
        result[2] = _mm256_sub_ps(a[2], b[2]);
    }

    inline float avx2_extract(const __m256 x, const int n) {
        return ((float*)(&x))[n];
    }

    const __m256 one_m256       = _mm256_set1_ps(1.0f);
    const __m256 minus_one_m256 = _mm256_set1_ps(-1.0f);
    const __m256 pos_eps_m256   = _mm256_set1_ps(1e-9f);
    const __m256 neg_eps_m256   = _mm256_set1_ps(-1e-9f);
    const __m256 zero_m256      = _mm256_set1_ps(0.0f);

    template <>
    struct accelerator<accel_types::naive_avx2> {
        std::vector<avx2_packed_triangles> packed_triangles;

        accelerator(std::vector<triangle>&& tris) {
            const auto N = tris.size();
            packed_triangles.reserve(N);

            for (unsigned int i = 0; i < N; i += 8) {
                glm::vec3 edge1[8];
                glm::vec3 edge2[8];
                glm::vec3 vert0[8];
                float     inactive[8];

                glm::vec3 normal[8];

                for (unsigned int n = 0; n < 8; n++) {
                    edge1[n] = (i + n) < N ? tris[i + n].p1.pos - tris[i + n].p0.pos
                                           : glm::dvec3(0.0, 0.0, 0.0);

                    edge2[n] = (i + n) < N ? tris[i + n].p2.pos - tris[i + n].p0.pos
                                           : glm::dvec3(0.0, 0.0, 0.0);

                    vert0[n] = (i + n) < N ? tris[i + n].p0.pos : glm::dvec3(0.0, 0.0, 0.0);

                    inactive[n] = (i + n) < N ? 0.f : 1.f;

                    normal[n] = (i + n) < N ? tris[i + n].p0.normal : glm::dvec3(0, 0, 0);
                }

                avx2_packed_triangles tri;

                for (int n = 0; n < 3; n++) {
                    tri.edge1[n] =
                        _mm256_set_ps(edge1[0][n], edge1[1][n], edge1[2][n], edge1[3][n],
                                      edge1[4][n], edge1[5][n], edge1[6][n], edge1[7][n]);

                    tri.edge2[n] =
                        _mm256_set_ps(edge2[0][n], edge2[1][n], edge2[2][n], edge2[3][n],
                                      edge2[4][n], edge2[5][n], edge2[6][n], edge2[7][n]);

                    tri.vert0[n] =
                        _mm256_set_ps(vert0[0][n], vert0[1][n], vert0[2][n], vert0[3][n],
                                      vert0[4][n], vert0[5][n], vert0[6][n], vert0[7][n]);

                    tri.normal[n] =
                        _mm256_set_ps(normal[0][n], normal[1][n], normal[2][n], normal[3][n],
                                      normal[4][n], normal[5][n], normal[6][n], normal[7][n]);
                }

                tri.inactive = _mm256_set_ps(inactive[0], inactive[1], inactive[2], inactive[3],
                                             inactive[4], inactive[5], inactive[6], inactive[7]);

                packed_triangles.push_back(tri);
            }

            tris.clear();
        }

        bool intersects(ray& ray, intersection_result& res) {
            avx2_packed_ray p_ray;

            for (int n = 0; n < 3; n++) {
                p_ray.origin[n]    = _mm256_set1_ps(ray.origin[n]);
                p_ray.direction[n] = _mm256_set1_ps(ray.direction[n]);
            }

            for (auto& packed_tris : packed_triangles) {

                // q = cross(ray.dir, v0v2)
                __m256 q[3];
                avx2_cross(q, p_ray.direction, packed_tris.edge2);

                // a = dot(v0v1, q)
                __m256 a = avx2_dot(packed_tris.edge1, q);

                // f = 1 / a
                __m256 f = _mm256_div_ps(one_m256, a);

                // s = ray.origin - v0
                __m256 s[3];
                avx2_sub(s, p_ray.origin, packed_tris.vert0);

                // u = f * dot(s, q)
                __m256 u = _mm256_mul_ps(f, avx2_dot(s, q));

                // r = cross(s, edge1)
                __m256 r[3];
                avx2_cross(r, s, packed_tris.edge1);

                // v = f * dot(ray.dir, r)
                __m256 v = _mm256_mul_ps(f, avx2_dot(p_ray.direction, r));

                // t = f * dot(v0v2, r)
                __m256 t = _mm256_mul_ps(f, avx2_dot(packed_tris.edge2, r));

                // mask of failed intersections
                __m256 failed;

                // t > eps && t < -eps
                failed = _mm256_and_ps(_mm256_cmp_ps(a, neg_eps_m256, _CMP_GT_OQ),
                                       _mm256_cmp_ps(a, pos_eps_m256, _CMP_LT_OQ));

                // u > 0
                failed = _mm256_or_ps(failed, _mm256_cmp_ps(u, zero_m256, _CMP_LT_OQ));

                // v > 0
                failed = _mm256_or_ps(failed, _mm256_cmp_ps(v, zero_m256, _CMP_LT_OQ));

                // (u + v) < 1
                failed =
                    _mm256_or_ps(failed, _mm256_cmp_ps(_mm256_add_ps(u, v), one_m256, _CMP_GT_OQ));

                // tri.inactive == false
                failed = _mm256_or_ps(failed, packed_tris.inactive);

                // set failed ones to -1
                __m256 t_results = _mm256_blendv_ps(t, minus_one_m256, failed);

                // get failed isect mask as bitset in an int
                int mask = _mm256_movemask_ps(t_results);

                // at least 1 intersection
                if (mask != 0xFF) {
                    for (int n = 0; n < 8; n++) {
                        float val = avx2_extract(t_results, n);

                        if (val > 0.f && val < res.t) {
                            res.t   = val;
                            res.hit = true;

                            res.hitnormal = glm::vec3(avx2_extract(packed_tris.normal[0], n),
                                                      avx2_extract(packed_tris.normal[1], n),
                                                      avx2_extract(packed_tris.normal[2], n));
                        }
                    }
                }
            }

            if (res.hit) {
                res.hitpos = ray.origin + ray.direction * res.t;

                return true;
            }

            return false;
        }
    };

} // namespace iray
