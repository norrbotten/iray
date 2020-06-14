#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "accel/geometry.hpp"
#include "utils/parsers.hpp"

namespace iray {

    struct model {
        std::string filename;
        glm::dmat4  transform;
    };

    struct scene {
        std::vector<model>    models;
        std::vector<triangle> triangles;

        void build() {
            for (auto& model : models) {
                std::filesystem::path path(model.filename);

                std::vector<triangle> temp_tris;

                if (path.extension().compare("obj") == 0)
                    parse_obj(model.filename.c_str(), temp_tris);
                else if (path.extension().compare("stl") == 0)
                    parse_stl(model.filename.c_str(), temp_tris);

                for (auto& tri : temp_tris) {
                    tri.p0.pos = model.transform * glm::dvec4(tri.p0.pos, 1.0);
                    tri.p1.pos = model.transform * glm::dvec4(tri.p1.pos, 1.0);
                    tri.p2.pos = model.transform * glm::dvec4(tri.p2.pos, 1.0);

                    tri.p0.normal = model.transform * glm::dvec4(tri.p0.normal, 1.0);
                    tri.p1.normal = model.transform * glm::dvec4(tri.p1.normal, 1.0);
                    tri.p2.normal = model.transform * glm::dvec4(tri.p2.normal, 1.0);

                    triangles.push_back(tri);
                }
            }
        }
    };

} // namespace iray