#pragma once

#include <cmath>
#include <functional>
#include <limits>
#include <unordered_set>

#include <glm/glm.hpp>

#include "accel/accel_trait.hpp"
#include "accel/geometry.hpp"

#include "utils/math.hpp"

namespace iray {

    // Accelerator using a crappy kdtree implementation i wrote in lua years ago
    // its only slightly faster than the naive avx2 "accelerator" with 1 million or so tris
    // 3-4x faster than naive avx2 with 100-200k tris

    // idea: limit subdivision so theres 128-256 tris per leaf, then do those with avx2

    // its also bork and finds the furthest intersection or some shit rn

    inline void kd_triangle_bbox(const triangle& tri, glm::dvec3& min, glm::dvec3& max) {
        max.x = std::max(tri.p0.pos.x, std::max(tri.p1.pos.x, tri.p2.pos.x));
        max.y = std::max(tri.p0.pos.y, std::max(tri.p1.pos.y, tri.p2.pos.y));
        max.z = std::max(tri.p0.pos.z, std::max(tri.p1.pos.z, tri.p2.pos.z));
        min.x = std::min(tri.p0.pos.x, std::min(tri.p1.pos.x, tri.p2.pos.x));
        min.y = std::min(tri.p0.pos.y, std::min(tri.p1.pos.y, tri.p2.pos.y));
        min.z = std::min(tri.p0.pos.z, std::min(tri.p1.pos.z, tri.p2.pos.z));
    }

    inline void kd_triangle_midpoint(const triangle& tri, glm::dvec3& mid) {
        mid = (tri.p0.pos + tri.p1.pos + tri.p2.pos) / 3.0;
    }

    std::size_t kd_triangle_hash(const triangle& tri) {
        // it will be FIIIINEEEEE
        // this entire project is just for fucking around anyways

        auto shit_hash = [](const glm::dvec3& point) -> std::size_t {
            std::size_t res = 0;

            for (int n = 0; n < 3; n++)
                res ^= *(std::size_t*)((void*)(&point[n]));

            return res;
        };

        return shit_hash(tri.p0.pos) ^ shit_hash(tri.p1.pos) ^ shit_hash(tri.p2.pos);
    }

    struct kd_node {
        glm::dvec3 bbox_min = glm::dvec3();
        glm::dvec3 bbox_max = glm::dvec3();

        kd_node* left  = nullptr;
        kd_node* right = nullptr;

        std::vector<triangle> tris;

        ~kd_node() {
            if (left != nullptr)
                delete left;

            if (right != nullptr)
                delete right;
        }
    };

    bool kd_ray_vs_kdtree(const glm::dvec3& orig, const glm::dvec3& dir, kd_node* kd, double& t,
                          triangle** hit_tri) {

        if (kd == nullptr)
            return false;

        double dummy;
        if (ray_vs_aabb(orig, dir, kd->bbox_min, kd->bbox_max, dummy)) {

            if (kd->left != nullptr || kd->right != nullptr) { // not at a leaf
                auto t_left  = kd_ray_vs_kdtree(orig, dir, kd->left, t, hit_tri);
                auto t_right = kd_ray_vs_kdtree(orig, dir, kd->right, t, hit_tri);

                return t_left || t_right;
            }
            else {
                bool      hit     = false;
                double    t_min   = std::numeric_limits<double>::max();
                triangle* tri_ptr = nullptr;

                for (auto& tri : kd->tris) {
                    double t_tri;
                    if (ray_vs_tri(orig, dir, tri, t_tri)) {
                        hit = true;

                        if (t_tri < t_min) {
                            t_min = t_tri;

                            tri_ptr = std::addressof(tri);
                        }
                    }
                }

                if (t_min < t) {
                    *hit_tri = tri_ptr;
                    t        = t_min;
                }

                return hit;
            }
        }

        return false;
    }

    kd_node* build_kd(std::vector<triangle>&& tris, int cur_depth = 0) {
        const auto N = tris.size();

        kd_node* node_ptr = new kd_node();
        node_ptr->tris    = tris;

        tris.clear(); // above assignment will have deep copied, so lets clear this one

        if (N == 0)
            return node_ptr;

        if (N == 1) {
            kd_triangle_bbox(tris[0], node_ptr->bbox_min, node_ptr->bbox_max);
            return node_ptr;
        }

        // find bounding box of all tris
        glm::dvec3 min_vec = glm::dvec3(std::numeric_limits<double>::max());
        glm::dvec3 max_vec = glm::dvec3(std::numeric_limits<double>::lowest());

        for (const auto& tri : node_ptr->tris) {
            glm::dvec3 tri_bbox_min, tri_bbox_max;
            kd_triangle_bbox(tri, tri_bbox_min, tri_bbox_max);

            for (int n = 0; n < 3; n++) {
                min_vec[n] = std::min(min_vec[n], tri_bbox_min[n]);
                max_vec[n] = std::max(max_vec[n], tri_bbox_max[n]);
            }
        }

        node_ptr->bbox_min = min_vec;
        node_ptr->bbox_max = max_vec;

        // find midpoint
        glm::dvec3 midpoint = glm::dvec3(0.0);

        for (auto& tri : node_ptr->tris) {
            glm::dvec3 tmp_midpoint;
            kd_triangle_midpoint(tri, tmp_midpoint);
            midpoint += tmp_midpoint;
        }

        midpoint /= (double)N;

        // find the longest axis
        int    longest_axis = -1;
        double longest_len  = 0;

        for (int n = 0; n < 3; n++) {
            double len = std::abs(node_ptr->bbox_max[n] - node_ptr->bbox_min[0]);
            if (len > longest_len) {
                longest_len  = len;
                longest_axis = 0;
            }
        }

        // split triangles based on whatever side of the midpoint theyre on
        std::vector<triangle> left_tris;
        std::vector<triangle> right_tris;

        for (const auto& tri : node_ptr->tris) {
            glm::dvec3 tri_midpoint;
            kd_triangle_midpoint(tri, tri_midpoint);

            if (midpoint[longest_axis] >= tri_midpoint[longest_axis])
                right_tris.push_back(tri);
            else
                left_tris.push_back(tri);
        }

        // if one of the sides are empty, copy triangles into empty side
        if (left_tris.size() == 0 && right_tris.size() > 0)
            left_tris = right_tris;

        if (right_tris.size() == 0 && left_tris.size() > 0)
            right_tris = left_tris;

        // find number of matching triangles betweem left and right
        std::unordered_set<std::size_t> left_hashes;

        for (const auto& tri : left_tris)
            left_hashes.insert(kd_triangle_hash(tri));

        unsigned int num_matching = 0;
        for (const auto& tri : right_tris)
            if (left_hashes.contains(kd_triangle_hash(tri)))
                num_matching++;

        // if less than half match, keep subdividing
        if (num_matching <= left_tris.size() / 2 && num_matching <= right_tris.size() / 2) {
            node_ptr->left  = build_kd(std::move(left_tris), cur_depth + 1);
            node_ptr->right = build_kd(std::move(right_tris), cur_depth + 1);

            node_ptr->tris.clear();
        }

        return node_ptr;
    }

    template <>
    struct accelerator<accel_types::kdtree> {
        kd_node* root_node = nullptr;

        accelerator(std::vector<triangle>&& tris) {
            root_node = build_kd(std::move(tris));
        }

        ~accelerator() {
            if (root_node != nullptr)
                delete root_node;
        }

        bool intersects(ray& ray, intersection_result& res) {
            double    t   = std::numeric_limits<double>::max();
            triangle* tri = nullptr;

            if (kd_ray_vs_kdtree(ray.origin, ray.direction, root_node, t, &tri)) {
                res.t         = t;
                res.hitpos    = ray.origin + ray.direction * t;
                res.hitnormal = tri->p0.normal;
                res.hit       = true;

                return true;
            }

            return false;
        };
    };

} // namespace iray
