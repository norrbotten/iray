#pragma once

#include <vector>

#include "accel/accel_trait.hpp"
#include "integrators/integrator_trait.hpp"

#include "accel/geometry.hpp"

namespace iray {

    template <accel_types AccelType>
    struct integrator<integrator_types::albedo, AccelType> {
        accel<AccelType> accelerator;

        integrator(std::vector<triangle>&& tris)
            : accelerator(std::move(tris)) {
        }
    };

} // namespace iray