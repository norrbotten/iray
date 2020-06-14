
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

    std::vector<triangle> tris;
    if (auto err = parse_stl("data/eiffel.stl", tris); err.has_value()) {
        std::cout << err.value() << "\n";
        return EXIT_FAILURE;
    }

    std::cout << tris.size() << " triangles loaded\n";

    render_settings settings;
    settings.resolution.width  = 512;
    settings.resolution.height = 512;

    settings.fov = 70.f;

    settings.cam_pos = glm::vec3(-40.0, -40.0, 20.0);
    settings.cam_aim = glm::vec3(0.0, 0.0, 20.0);

    auto accel = accelerator<accel_types::naive_avx2>(std::move(tris));
    auto integ = integrator<integrator_types::albedo, accel_types::naive_avx2>(&accel);

    int width  = 1024;
    int height = 1024;

    camera      cam(width, height);
    sample_film film(width, height);

    cam.set_pos(glm::dvec3(-60, -60, 60));
    cam.aim_at(glm::dvec3(0, 0, 48));

    std::mutex cout_mtx;

    auto render_row = [&film, &cam, &accel, &integ, &cout_mtx, width](int y) {
        for (int x = 0; x < width; x++) {
            intersection_result res;

            auto ray = cam.camray(x, y);
            film.splat(x, y, integ.radiance(ray, res));
        }

        std::lock_guard g(cout_mtx);
        std::cout << "line " << y << " finished\n";
    };

    std::vector<std::thread> workers;

    for (int y = 0; y < height; y++)
        workers.push_back(std::thread(render_row, y));

    for (auto& worker : workers)
        worker.join();

    film.save_png("eiffel_avx2.png");
}
