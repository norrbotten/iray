#pragma once

#include <type_traits>

namespace iray {

    enum class accel_types {
        naive,
        naive_avx2,
    };

    template <accel_types Type>
    struct accelerator : std::false_type {
        accelerator() = delete;
    };

} // namespace iray

#include "accel/naive.hpp"
