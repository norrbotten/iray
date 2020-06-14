
#include <iostream>

#include <mutex>
#include <thread>

#include "accel/naive.hpp"
#include "accel/naive_avx2.hpp"

#include "utils/camera.hpp"
#include "utils/film.hpp"
#include "utils/parsers.hpp"

int main() {
    using namespace iray;

    std::vector<triangle> tris;
    if (auto err = parse_stl("data/eiffel.stl", tris); err.has_value()) {
        std::cout << err.value() << "\n";
        return EXIT_FAILURE;
    }

    std::cout << tris.size() << " triangles loaded\n";

    auto acc = accelerator<accel_types::naive_avx2>(std::move(tris));

    int width  = 512;
    int height = 512;

    camera cam(width, height);
    film   film(width, height);

    cam.set_pos(glm::dvec3(-60, -60, 60));
    cam.aim_at(glm::dvec3(0, 0, 48));

    std::mutex cout_mtx;

    auto render_row = [&film, &cam, &acc, &cout_mtx, width](int y) {
        for (int x = 0; x < width; x++) {
            intersection_result res;

            auto camray = cam.camray(x, y);
            if (acc.intersects(camray, res)) {
                float col   = (240.f - res.t) / 240.f;
                float shade = glm::dot(res.hitnormal, camray.direction * -1.0);

                film.splat(x, y, color{col * shade, col * shade, col * shade});
            }
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
