
#include <iostream>

#include <mutex>
#include <thread>

#include "accel/aa_bsp.hpp"
#include "accel/aa_bsp_avx2.hpp"
#include "accel/kdtree.hpp"
#include "accel/naive.hpp"
#include "accel/naive_avx2.hpp"

#include "integrators/albedo.hpp"

#include "utils/camera.hpp"
#include "utils/film.hpp"
#include "utils/parsers.hpp"

#include "renderer/render_ctx.hpp"

#include "utils/math.hpp"

int main() {
    using namespace iray;

    render_settings settings;
    settings.resolution.width  = 512;
    settings.resolution.height = 512;

    settings.blocksize = 32;

    settings.fov = 70.f;

    settings.cam_pos = glm::dvec3(-70.0, -70.0, 60.0);
    settings.cam_aim = glm::dvec3(0.0, 0.0, 50.0);

    // settings.cam_pos = glm::dvec3(1.0, 2.0, 1.3);
    // settings.cam_aim = glm::dvec3(0.0, 0.0, 0.0);

    scene scene;
    scene.models.push_back(model{"data/baby_yoda.stl", glm::dmat4(1.f)});

    render_ctx<accel_types::axisaligned_bsp_avx2, integrator_types::albedo> ctx(&settings, &scene);

    ctx.render(24);
    ctx.save("yoda_avx2.png");
}
