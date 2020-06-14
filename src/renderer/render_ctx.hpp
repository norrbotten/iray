#pragma once

#include <mutex>
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
        camera      cam;
        sample_film film;
        scene*      scene_ptr;

        render_settings* settings = nullptr;

        accelerator<AccelType>*           accel = nullptr;
        integrator<IntegType, AccelType>* integ = nullptr;

        std::mutex         blocks_mtx;
        std::vector<block> blocks;
        std::vector<block> blocks_finished;

        render_ctx(render_settings* settings, scene* scene_ptr)
            : settings(settings)
            , scene_ptr(scene_ptr) {

            const auto w = settings->resolution.width;
            const auto h = settings->resolution.height;

            cam  = camera(w, h);
            film = sample_film(w, h);

            cam.set_pos(settings->cam_pos);
            cam.aim_at(settings->cam_aim);

            cam.set_fov(settings->fov);

            // probably unnecessary here
            std::lock_guard g(blocks_mtx);

            for (int y = 0; y < h; y += settings->blocksize)
                for (int x = 0; x < w; x += settings->blocksize)
                    blocks.push_back(block{x, y, std::min(x + settings->blocksize, w),
                                           std::min(y + settings->blocksize, h)});

            scene_ptr->build();
            accel = new accelerator<AccelType>(std::move(scene_ptr->triangles));
            integ = new integrator<IntegType, AccelType>(accel);
        }

        void render(int num_threads) {
        }
    };

} // namespace iray
