#pragma once

#include <type_traits>

#include "accel/accel_trait.hpp"
#include "accel/geometry.hpp"

#include "utils/color.hpp"

namespace iray {

    enum class integrator_types {
        albedo,
    };

    template <integrator_types IntegType, accel_types AccelType>
    struct integrator : std::false_type {
        integrator(accelerator<AccelType>* accel) = delete;
    };

} // namespace iray
