
#include <iostream>

#include "utils/parsers.hpp"

int main() {
    using namespace iray;

    std::vector<triangle> tris;
    if (auto err = parse_obj("data/gourd.obj", tris); err.has_value()) {
        std::cout << err.value() << "\n";
    }

    for (auto& tri : tris) {
        std::cout << tri.p0.normal.x << " \t" << tri.p0.normal.y << " \t" << tri.p0.normal.z
                  << "\n";
    }
}
