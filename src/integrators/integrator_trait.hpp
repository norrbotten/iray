#pragma once

#include <type_traits>

#include "accel/accel_trait.hpp"

namespace iray {

    enum class integrator_types {
        albedo,
    };

    template <integrator_types Type, accel_types AccelType>
    struct integrator : std::false_type {
        integrator() = delete;
    };

} // namespace iray

#include "integrators/albedo.hpp"
