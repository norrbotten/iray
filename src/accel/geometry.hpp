#pragma once

#include <array>
#include <limits>
#include <ostream>

#include <glm/glm.hpp>

namespace iray {

    struct intersection_result {
        double     t         = std::numeric_limits<double>::max();
        glm::dvec3 hitpos    = glm::dvec3();
        glm::dvec3 hitnormal = glm::dvec3();
        bool       hit       = false;
    };

    struct vertex {
        glm::dvec3 pos;
        glm::dvec3 normal;
        glm::dvec2 texcoord;
    };

    struct triangle {
        vertex p0, p1, p2;
    };

    struct ray {
        glm::dvec3 origin;
        glm::dvec3 direction;
    };

} // namespace iray
