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

        block(int sx, int sy, int ex, int ey)
            : start_x(sx)
            , start_y(sy)
            , end_x(ex)
            , end_y(ey)
            , status(block_status::AVAILABLE) {
        }

        block_status status;
    };

} // namespace iray