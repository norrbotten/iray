#pragma once

#include <mmintrin.h>

#include <limits>
#include <vector>

#include <glm/glm.hpp>

#include "accel/accel_trait.hpp"
#include "accel/geometry.hpp"

// include naive avx2 acc for structs and crap
#include "accel/naive_avx2.hpp"

#include "utils/math.hpp"

namespace iray {

    constexpr size_t MAX_TRIS_PER_AVX2_NODE = 256;

    struct avx2_aa_bsp_node {
        avx2_aa_bsp_node* left;
        avx2_aa_bsp_node* right;

        // the bsps plane
        glm::dvec3 plane_point;
        glm::dvec3 plane_normal;

        // bounding box containing all triangles, this is required
        // in order to prune branches since otherwise every node
        // would have to be tested
        glm::dvec3 box_min;
        glm::dvec3 box_max;

        // triangles this node contains, if this node is not a leaf, this is empty
        std::vector<triangle> tris;

        std::vector<avx2_packed_triangles> packed_tris;

        ~avx2_aa_bsp_node() {
            if (left != nullptr)
                delete left;

            if (right != nullptr)
                delete right;
        }
    };

    avx2_aa_bsp_node* build_avx2_aa_bsp(std::vector<triangle>&& tris, int splitting_plane = 0) {
        avx2_aa_bsp_node* node = new avx2_aa_bsp_node();

        // find the midpoint
        glm::dvec3 midpoint = glm::dvec3(0.0);

        for (const auto& tri : tris)
            midpoint += (tri.p0.pos + tri.p1.pos + tri.p2.pos);

        midpoint /= (double)(tris.size() * 3);
        node->plane_point = midpoint;

        // get current splitting plane
        constexpr glm::dvec3 splitplanes[3] = {
            glm::dvec3(1, 0, 0),
            glm::dvec3(0, 1, 0),
            glm::dvec3(0, 0, 1),
        };

        node->plane_normal = splitplanes[splitting_plane % 3];

        // find bounding box of all tris
        glm::dvec3 min_vec = glm::dvec3(std::numeric_limits<double>::max());
        glm::dvec3 max_vec = glm::dvec3(std::numeric_limits<double>::lowest());

        for (const auto& tri : tris) {
            glm::dvec3 tri_bbox_min, tri_bbox_max;
            triangle_bbox(tri, tri_bbox_min, tri_bbox_max);

            for (int n = 0; n < 3; n++) {
                min_vec[n] = std::min(min_vec[n], tri_bbox_min[n]);
                max_vec[n] = std::max(max_vec[n], tri_bbox_max[n]);
            }
        }

        node->box_min = min_vec;
        node->box_max = max_vec;

        // classify triangles as left or right
        // triangles that intersect the splitting plane go in both vectors
        // you may think, "why not split the triangle if it intersects?", well,
        // doing that would generate even more triangles since unless its split
        // perfectly on one of its vertices, 3 triangles will be generated.

        std::vector<triangle> left_tris, right_tris;

        for (const auto& tri : tris) {
            int side = tri_plane_check(tri.p0.pos, tri.p1.pos, tri.p2.pos, node->plane_point,
                                       node->plane_normal);

            if (side == -1)
                left_tris.push_back(tri);
            else if (side == 1)
                right_tris.push_back(tri);
            else {
                left_tris.push_back(tri);
                right_tris.push_back(tri);
            }
        }

        node->tris = tris;
        tris.clear();

        // recursively split classified triangles if theres still too many
        if ((left_tris.size() > MAX_TRIS_PER_AVX2_NODE) ||
            (right_tris.size() > MAX_TRIS_PER_AVX2_NODE)) {
            node->tris.clear();

            node->left  = build_avx2_aa_bsp(std::move(left_tris), splitting_plane + 1);
            node->right = build_avx2_aa_bsp(std::move(right_tris), splitting_plane + 1);
        }
        else {
            // pack triangles

            const auto N = node->tris.size();
            node->packed_tris.reserve(N / 8);

            for (unsigned int i = 0; i < N; i += 8) {
                glm::vec3 edge1[8];
                glm::vec3 edge2[8];
                glm::vec3 vert0[8];
                float     inactive[8];

                glm::vec3 normal[8];

                for (unsigned int n = 0; n < 8; n++) {
                    edge1[n] = (i + n) < N ? node->tris[i + n].p1.pos - node->tris[i + n].p0.pos
                                           : glm::dvec3(0.0, 0.0, 0.0);

                    edge2[n] = (i + n) < N ? node->tris[i + n].p2.pos - node->tris[i + n].p0.pos
                                           : glm::dvec3(0.0, 0.0, 0.0);

                    vert0[n] = (i + n) < N ? node->tris[i + n].p0.pos : glm::dvec3(0.0, 0.0, 0.0);

                    inactive[n] = (i + n) < N ? 0.f : 1.f;

                    normal[n] = (i + n) < N ? node->tris[i + n].p0.normal : glm::dvec3(0, 0, 0);
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

                node->packed_tris.push_back(tri);
            }

            // we dont need the original tris anymore
            node->tris.clear();
        }

        return node;
    }

    bool ray_vs_avx2_aa_bsp(const glm::dvec3& orig, const glm::dvec3& dir,
                            const avx2_packed_ray& p_ray, avx2_aa_bsp_node* node, double& t,
                            glm::dvec3& normal) {

        // just some sanity checks
        if (node == nullptr)
            return false;

        double dummy;
        if (!ray_vs_aabb(orig, dir, node->box_min, node->box_max, dummy))
            return false;

        if (node->packed_tris.size() > 0 && node->left == nullptr && node->right == nullptr) {
            // at a leaf node

            bool       hit   = false;
            double     t_min = std::numeric_limits<double>::max();
            glm::dvec3 hit_normal;

            for (auto& packed_tris : node->packed_tris) {
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
                __m256 t_shit = _mm256_mul_ps(f, avx2_dot(packed_tris.edge2, r));

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
                __m256 t_results = _mm256_blendv_ps(t_shit, minus_one_m256, failed);

                // get failed isect mask as bitset in an int
                int mask = _mm256_movemask_ps(t_results);

                // at least 1 intersection
                if (mask != 0xFF) {
                    for (int n = 0; n < 8; n++) {
                        float val = avx2_extract(t_results, n);

                        if (val > 0.f && val < t_min) {
                            t_min = val;
                            hit   = true;

                            hit_normal = glm::dvec3(avx2_extract(packed_tris.normal[0], n),
                                                    avx2_extract(packed_tris.normal[1], n),
                                                    avx2_extract(packed_tris.normal[2], n));
                        }
                    }
                }
            }

            if (t_min < t) {
                normal = hit_normal;
                t      = t_min;
            }

            return hit;
        }
        else {
            auto left  = ray_vs_avx2_aa_bsp(orig, dir, p_ray, node->left, t, normal);
            auto right = ray_vs_avx2_aa_bsp(orig, dir, p_ray, node->right, t, normal);

            return left || right;
        }
    }

    template <>
    struct accelerator<accel_types::axisaligned_bsp_avx2> {
        avx2_aa_bsp_node* root_node = nullptr;

        accelerator(std::vector<triangle>&& tris) {
            root_node = build_avx2_aa_bsp(std::move(tris));
        }

        ~accelerator() {
            if (root_node != nullptr)
                delete root_node;
        }

        bool intersects(ray& ray, intersection_result& res) {
            double     t = std::numeric_limits<double>::max();
            glm::dvec3 normal;

            avx2_packed_ray p_ray;

            for (int n = 0; n < 3; n++) {
                p_ray.origin[n]    = _mm256_set1_ps(ray.origin[n]);
                p_ray.direction[n] = _mm256_set1_ps(ray.direction[n]);
            }

            if (ray_vs_avx2_aa_bsp(ray.origin, ray.direction, p_ray, root_node, t, normal)) {
                res.t         = t;
                res.hitpos    = ray.origin + ray.direction * t;
                res.hitnormal = normal;
                res.hit       = true;

                return true;
            }

            return false;
        }
    };

} // namespace iray
