#pragma once

#include "accel/geometry.hpp"

#include "accel/accel_trait.hpp"
#include "integrators/integrator_trait.hpp"

#include "utils/color.hpp"

namespace iray {

    template <accel_types AccelType>
    struct integrator<integrator_types::albedo, AccelType> {
        accelerator<AccelType>* accel = nullptr;

        integrator(accelerator<AccelType>* accel)
            : accel(accel) {
        }

        color radiance(ray& ray) {
            intersection_result res;

            if (accel->intersects(ray, res)) {
                float col   = (240.f - res.t) / 240.f;
                float shade = glm::dot(res.hitnormal, ray.direction * -1.0);
                return color{col * shade, col * shade, col * shade};
            }

            return color{0, 0, 0};
        }
    };

} // namespace iray