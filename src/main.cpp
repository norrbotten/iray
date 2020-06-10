
#include "integrators/albedo.hpp"

int main() {
    using namespace iray;

    std::vector<triangle> tris;

    auto integ = integrator<integrator_types::albedo, accel_types::naive>(std::move(tris));
}