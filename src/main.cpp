
#include <iostream>

#include <mutex>
#include <thread>

#include "accel/naive_avx2.hpp"

#include "integrators/albedo.hpp"

#include "utils/camera.hpp"
#include "utils/film.hpp"
#include "utils/parsers.hpp"

#include "renderer/render_ctx.hpp"

int main() {
    using namespace iray;

    render_settings settings;
    settings.resolution.width  = 1024;
    settings.resolution.height = 1024;

    settings.fov = 70.f;

    settings.cam_pos = glm::vec3(-50.01, -50.0, 40.0);
    settings.cam_aim = glm::vec3(0.0, 0.0, 36.0);

    scene scene;
    scene.models.push_back(model{"data/eiffel.stl", glm::dmat4(1.f)});

    render_ctx<accel_types::naive_avx2, integrator_types::albedo> ctx(&settings, &scene);

    ctx.render(24);
    ctx.save("works.png");

    // 139,448 triangles * 1024^2 pixels = 146,221,826,048 ray vs triangle tests
    // rendered in 9.44851 seconds on 24 threads
    // 146,221,826,048/9.44851 = 15,475,649,181 tests/second
    // 15.5 billion ray/triangle tests a second, without any acceleration structure
}
