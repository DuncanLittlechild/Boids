#include <raylib.h>
#include <vector>
#include <cstdint>

#include "Random.h"

using real64 = double;
using real32 = float;
using uint64 = uint64_t;
using uint32 = uint32_t;

namespace Settings {
    // All values marked as max are non-inclusive
    // Max x and y value a boid can have before it wraps around to 0
    real32 MaxX{};
    real32 MaxY{};

    // Ensures that boids will always keep moving slightly 
    real32 MinSpeed{};
    real32 MaxSpeed{};

    real32 MaxOrientation {2 * PI};
}


std::size_t boidNum{100};

// Boids are implemented as the vector indices
int main(){
    std::vector<Vector2> positions(boidNum, Vector2{});

    std::vector<real32> orientation(boidNum, 0);

    std::vector<real32> speed(boidNum, Settings::MinSpeed);

    // Randomly determine the starting position, speed and orientation of all
    // boids.
    for (auto i {0uz}; i < boidNum; ++i){
        positions[i].x = Random::get(0.0f, Settings::MaxX);
        positions[i].y = Random::get(0.0f, Settings::MaxY);

        orientation[i] = Random::get(0.0f, Settings::MaxOrientation);

        speed[i] = Random::get(Settings::MinSpeed, Settings::MaxSpeed);
    }

    return 0;
}