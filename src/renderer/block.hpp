#pragma once

#include "utils/camera.hpp"
#include "utils/film.hpp"

namespace iray {

    enum class block_status {
        AVAILABLE,
        RENDERING,
        FINISHED,
    };

    struct block {
        int start_x, start_y;
        int end_x, end_y;

        block_status status;
    };

} // namespace iray