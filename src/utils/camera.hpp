#pragma once

#include <cmath>

#include <glm/glm.hpp>

#include "accel/geometry.hpp"

namespace iray {

    class camera {
        float      fov;
        glm::dvec3 origin;

        glm::dvec3 forward;
        glm::dvec3 right;
        glm::dvec3 up;

        int   w, h;
        float aspect;

    public:
        camera(int width, int height)
            : w(width)
            , h(height) {

            fov    = 1.0 / float(std::tan(0.01745329251f * 45.f));
            origin = glm::dvec3(0, 0, 0);

            aspect = float(height) / float(width);

            set_pos(glm::dvec3(0, 0, 0));
            set_dir(glm::dvec3(1, 0, 0));
        }

        void set_pos(glm::dvec3 pos) {
            this->origin = pos;
        }

        void set_dir(glm::dvec3 dir) {
            this->forward = dir;
            this->right   = glm::cross(this->forward, glm::dvec3(0, 0, 1));
            this->up      = glm::cross(this->right, this->forward);
        }

        void aim_at(glm::dvec3 pos) {
            auto dir = glm::normalize(pos - this->origin);
            this->set_dir(dir);
        }

        ray camray(int x, int y) {
            double x_i = (2.0 * float(x) / float(this->w) - 1.0);
            double y_i = (2.0 * float(y) / float(this->h) - 1.0) * this->aspect;

            glm::dvec3 dir = this->forward - this->right * x_i - this->up * y_i;

            return ray{this->origin, glm::normalize(dir)};
        }
    };

} // namespace iray
