#pragma once

#include <iostream>

#include <chrono>
#include <deque>
#include <mutex>
#include <optional>
#include <vector>

#include "renderer/block.hpp"
#include "renderer/scene.hpp"

#include "accel/accel_trait.hpp"
#include "integrators/integrator_trait.hpp"

#include "utils/camera.hpp"
#include "utils/film.hpp"

namespace iray {

    struct render_settings {
        struct {
            int width  = 512;
            int height = 512;
        } resolution;

        int blocksize = 32;

        double fov = 90;

        glm::dvec3 cam_pos = glm::vec3();
        glm::dvec3 cam_aim = glm::vec3();
    };

    template <accel_types AccelType, integrator_types IntegType>
    struct render_ctx {
        render_settings* settings = nullptr;
        scene*           scene_ptr;

        camera      cam;
        sample_film film;

        accelerator<AccelType>*           accel = nullptr;
        integrator<IntegType, AccelType>* integ = nullptr;

        std::mutex        blocks_mtx;
        std::deque<block> blocks;

        render_ctx(render_settings* settings, scene* scene_ptr)
            : settings(settings)
            , scene_ptr(scene_ptr)
            , cam(settings->resolution.width, settings->resolution.height)
            , film(settings->resolution.width, settings->resolution.height) {

            const auto w = settings->resolution.width;
            const auto h = settings->resolution.height;

            std::cerr << "[render_ctx::render_ctx] initializing " << w << " x " << h << " pixels\n";

            cam.set_pos(settings->cam_pos);
            cam.aim_at(settings->cam_aim);

            cam.set_fov(settings->fov);

            // probably unnecessary here
            std::lock_guard g(blocks_mtx);

            for (int y = 0; y < h; y += settings->blocksize)
                for (int x = 0; x < w; x += settings->blocksize)
                    blocks.push_back(block(x, y, std::min(x + settings->blocksize, w),
                                           std::min(y + settings->blocksize, h)));

            std::cerr << "[render_ctx::render_ctx] building scene\n";
            scene_ptr->build();

            std::cerr << "[render_ctx::render_ctx] built scene, " << scene_ptr->triangles.size()
                      << " total triangles"
                      << "\n";

            accel = new accelerator<AccelType>(std::move(scene_ptr->triangles));
            integ = new integrator<IntegType, AccelType>(accel);
        }

        void render(int num_threads) {
            std::cerr << "[render_ctx::render] rendering using " << num_threads << " threads\n";

            std::vector<std::thread> workers;
            workers.reserve(num_threads);

            auto get_block = [this]() -> std::optional<block> {
                std::lock_guard g(blocks_mtx);

                if (blocks.empty())
                    return {};

                auto next_block = blocks.back();
                blocks.pop_back();

                next_block.status = block_status::RENDERING;

                return next_block;
            };

            auto time_start = std::chrono::high_resolution_clock::now();

            for (int i = 0; i < num_threads; i++) {
                workers.push_back(std::thread([this, get_block]() {
                    while (true) {
                        auto maybe_block = get_block();

                        if (!maybe_block.has_value())
                            break;

                        auto block = maybe_block.value();

                        for (int y = block.start_y; y < block.end_y; y++) {
                            for (int x = block.start_x; x < block.end_x; x++) {
                                auto ray = cam.camray(x, y);
                                film.splat(x, y, integ->radiance(ray));
                            }
                        }

                        block.status = block_status::FINISHED;
                    }

                    std::cerr << "[render_ctx::render thread#" << std::this_thread::get_id()
                              << "] finished\n";
                }));
            }

            for (auto& worker : workers)
                worker.join();

            auto time_end = std::chrono::high_resolution_clock::now();
            auto dur      = std::chrono::duration<double>(time_end - time_start);

            std::cerr << "[render_ctx::render] render finished in " << dur.count() << "s\n";
        }

        void save(const char* filename) {
            film.save_png(filename);
        }
    };

} // namespace iray
