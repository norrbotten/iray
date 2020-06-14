#pragma once

namespace iray {

    struct color {
        float r, g, b;

        color operator/(float rhs) {
            float inv = 1.f / rhs;
            return color{r * inv, g * inv, b * inv};
        }
    };

} // namespace iray