#pragma once

#include <type_traits>

namespace iray {

    enum class accel_types {
        naive,
    };

    template <accel_types Type>
    struct accel : std::false_type {};

} // namespace iray

#include "accel/naive.hpp"
