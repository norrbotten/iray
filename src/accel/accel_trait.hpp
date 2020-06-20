#pragma once

#include <type_traits>
#include <vector>

#include "renderer/scene.hpp"

namespace iray {

    enum class accel_types {
        naive,
        naive_avx2,
        kdtree,
        axisaligned_bsp,
        axisaligned_bsp_avx2,
    };

    template <accel_types Type>
    struct accelerator : std::false_type {
        accelerator(scene* scene_ptr) = delete;
    };

} // namespace iray
