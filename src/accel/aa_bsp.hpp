#pragma once

#include <limits>
#include <vector>

#include <glm/glm.hpp>

#include "accel/accel_trait.hpp"
#include "accel/geometry.hpp"

#include "utils/math.hpp"

namespace iray {

    constexpr size_t MAX_TRIS_PER_NODE = 256;

    // Accelerator using a "bsp"

    struct aa_bsp_node {
        aa_bsp_node* left;
        aa_bsp_node* right;

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

        ~aa_bsp_node() {
            if (left != nullptr)
                delete left;

            if (right != nullptr)
                delete right;
        }
    };

    aa_bsp_node* build_aa_bsp(std::vector<triangle>&& tris, int splitting_plane = 0) {
        aa_bsp_node* node = new aa_bsp_node();

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
        if ((left_tris.size() > MAX_TRIS_PER_NODE) || (right_tris.size() > MAX_TRIS_PER_NODE)) {
            node->tris.clear();

            node->left  = build_aa_bsp(std::move(left_tris), splitting_plane + 1);
            node->right = build_aa_bsp(std::move(right_tris), splitting_plane + 1);
        }

        return node;
    }

    bool ray_vs_aa_bsp(const glm::dvec3& orig, const glm::dvec3& dir, aa_bsp_node* node, double& t,
                       triangle** hit_tri) {

        // just some sanity checks
        if (node == nullptr)
            return false;

        double dummy;
        if (!ray_vs_aabb(orig, dir, node->box_min, node->box_max, dummy))
            return false;

        if (node->tris.size() > 0 && node->left == nullptr && node->right == nullptr) {
            // at a leaf node

            bool      hit     = false;
            double    t_min   = std::numeric_limits<double>::max();
            triangle* tri_ptr = nullptr;

            for (auto& tri : node->tris) {
                double t_tri;
                if (ray_vs_tri(orig, dir, tri, t_tri)) {
                    hit = true;

                    if (t_tri < t_min) {
                        t_min   = t_tri;
                        tri_ptr = &tri;
                    }
                }
            }

            if (t_min < t) {
                *hit_tri = tri_ptr;
                t        = t_min;
            }

            return hit;
        }
        else {
            auto left  = ray_vs_aa_bsp(orig, dir, node->left, t, hit_tri);
            auto right = ray_vs_aa_bsp(orig, dir, node->right, t, hit_tri);

            return left || right;
        }
    }

    template <>
    struct accelerator<accel_types::axisaligned_bsp> {
        aa_bsp_node* root_node = nullptr;

        accelerator(std::vector<triangle>&& tris) {
            root_node = build_aa_bsp(std::move(tris));
        }

        ~accelerator() {
            if (root_node != nullptr)
                delete root_node;
        }

        bool intersects(ray& ray, intersection_result& res) {
            double    t = std::numeric_limits<double>::max();
            triangle* tri;

            if (ray_vs_aa_bsp(ray.origin, ray.direction, root_node, t, &tri)) {
                res.t         = t;
                res.hitpos    = ray.origin + ray.direction * t;
                res.hitnormal = tri->p0.normal;
                res.hit       = true;

                return true;
            }

            return false;
        }
    };

} // namespace iray
