#pragma once

#include <vector>

#include "accel/accel_trait.hpp"
#include "accel/geometry.hpp"

namespace iray {

    template <>
    struct accel<accel_types::naive> {
        std::vector<triangle> triangles;

        accel(std::vector<triangle>&& tris) {
            triangles = tris;
            tris.clear();
        }
    };

} // namespace iray
