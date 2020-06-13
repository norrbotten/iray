#pragma once

#include <array>
#include <ostream>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

namespace iray {

    struct vertex {
        glm::vec3 pos;
        glm::vec3 normal;
        glm::vec2 texcoord;
    };

    struct triangle {
        vertex p0, p1, p2;
    };

} // namespace iray
