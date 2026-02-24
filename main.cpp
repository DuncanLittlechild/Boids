#include <raylib.h>
#include <vector>
#include <cstdint>
#include <cmath>

#include "Random.h"

using real64 = double;
using real32 = float;
using uint64 = uint64_t;
using uint32 = uint32_t;
using int64 = int64_t;
using int32 = int32_t;
using uint8 = uint8_t;

namespace Settings {
    int32 ScreenWidth {1280};
    int32 ScreenHeight {720};

    // All values marked as max are non-inclusive
    // Max x and y value a boid can have before it wraps around to 0
    real32 MaxX{static_cast<real32>(ScreenWidth};
    real32 MaxY{static_cast<real32>(ScreenHeight};

    // Ensures that boids will always keep moving slightly 
    real32 Min1DVelocity{};
    real32 Max1DVelocity{};

    // The radius used to calculate the size of a boid
    real32 BoidRadius {2.0f};

    // a limit on the number of boids one boid can interact with at any one time
    // Included for performance reasons
    uint32 MaxInteractable {32};

    // The distance at which the separation rule begins to come into effect
    real32 MinSeparation {1.0f};
    // Factor by which separation vectors are multiplied
    real32 SeparationForceFactor {5.0f};
    // Maximum separation force in any one direction
    real32 MaxSeparationForce {5.0f};

    real32 InteractionDistance {10.0f};

    uint32 FramesPerSecond {60};
}

std::size_t boidNum{100};

struct Boids {
    std::size_t number;
    std::vector<Vector2> positions;

    std::vector<real32> orientations;

    std::vector<Vector2> velocities;
    
    Boids(std::size_t num)
        : number {num}
        , positions(num, Vector2{})
        , velocities(num, Vector2{})
    {};
};

Boids GLOBAL_BOIDS{boidNum};

void ZeroVector(std::vector<Vector2>& vector){
    for (auto& vec : vector) {
        vec.x = 0.0f;
        vec.y = 0.0f;
    }
}

real32 GetDistance(const Vector2& a, const Vector2& b){
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

// Gets the vector from the main boid to the centre of the group of boids contained in the inputted array
Vector2 GetCentreOfGroup(std::vector<Vector2>& boidsInRange, Vector2& mainBoidPosition){
    Vector2 centreOfGroup {0.0, 0.0};
    for (auto& i : boidsInRange){
        centreOfGroup.x += GLOBAL_BOIDS.positions[i].x;
        centreOfGroup.y += GLOBAL_BOIDS.positions[i].y;
    }
    
    std::size_t arraySize {boidsInRange.size()};
    if (arraySize){
        centreOfGroup.x /= boidsInRange.size();
        centreOfGroup.y /= boidsInRange.size();
    }
    
    return {centreOfGroup.x - GLOBAL_BOIDS.positions[i].x, centreOfGroup.y - GLOBAL_BOIDS.positions[i].y};
}

// Gets the average velocity vector of an array of vectors
// This is being used as the average heading vector
Vector2 GetAverageHeadingVector(std::vector<Vector2>& boidsInRange){
    Vector2 totalVelocity {0.0f, 0.0f};
    
    for (auto& i : boidsInRange){
        totalVelocity.x += GLOBAL_BOIDS.velocities[i].x;
        totalVelocity.y += GLOBAL_BOIDS.velocities[i].y;
    }
    
    std::size_t arraySize {boidsInRange.size()};
    if (arraySize){
        totalVelocity.x /= boidsInRange.size();
        totalVelocity.y /= boidsInRange.size();
    }
    
    return totalVelocity;
}

//Gets the force with which boids that are too close will repel the main boid
Vector2 GetSeparationForceVector(std::vector<Vector2>& boidsTooClose, Vector2& mainBoidPosition){
    // Get the direction and the magnitude away from the too close boid, and turn that into a velocity vector
    Vector2 separationForceVector {0.0f, 0.0f};
    for (auto& i : boidsTooClose){
        Vector2 diff {mainBoidPosition.x - GLOBAL_BOIDS.positions[i].x, mainBoidPosition.y - GLOBAL_BOIDS.positions[i].y};
        
        separationForceVector.x += Settings::SeparationForceFactor * (Settings::MinSeparation - diff.x);
        separationForceVector.y += Settings::SeparationForceFactor * (Settings::MinSeparation - diff.y);
        
        if (separationForceVector.x >= Settings::MaxSeparationForce) 
            separationForceVector.x = Settings::MaxSeparationForce;
        
        if (separationForceVector.y >= Settings::MaxSeparationForce)
            separationForceVector.y = Settings::MaxSeparationForce;
    }
}

void UpdateBoids() {
    // Apply the rules to the boids
    // TODO: consider memoisation for this
    // This for loop just updates velocity - position is dealt with later
    for (auto i {0uz}; i < GLOBAL_BOIDS.number; ++i){
        std::vector<Vector2> boidsInRange;
        boidsInRange.reserve(Settings:MaxInteractable);
        
        std::vector<Vector2> boidsTooClose;
        boidsTooClose.reserve(16);
        
        for (auto j {iuz}; j < GLOBAL_BOIDS.number; ++j){
            real32 distance {GetDistance(GLOBAL_BOIDS.positions[i], GLOBAL_BOIDS.positions[j])};
            if (distance < Settings::InteractionDistance){
                 boidsInRange.push_back(j);
                if (distance < Settings::MinSeparation){
                    boidsTooClose.push_back(j);
                }
                if (boidsInRange.size() >= Settings::MaxInteractable){
                    break;
                }
            }            
        }
        // Generate vectors to represent each of the forces the boid is under
        Vector2 toCentreOfGroupVector {GetCentreOfGroupVector(boidsInRange, GLOBAL_BOIDS.positions[i])};
        
        Vector2 averageHeadingVector {GetAverageHeadingVector(boidsInRange)};
        
        Vector2 separationForceVector {GetSeparationForceVector(boidsTooClose)};
        
        // Get a resultant vector from this, then alter velocity by a set percentage of this
        Vector2 resultantVector
    }

    // Use velocity to update position
    for(auto i {0uz}; i < GLOBAL_BOIDS.number; ++i){
        GLOBAL_BOIDS.positions[i].x += (GLOBAL_BOIDS.velocities[i].x / Settings::FramesPerSecond) >= Settings::MaxX;
        GLOBAL_BOIDS.positions[i].y += GLOBAL_BOIDS.velocities[i].y / Settings::FramesPerSecond;
    }
}

void DrawBoids() {
    auto& positions {GLOBAL_BOIDS.positions};
    for (auto i {0uz}; i < positions.size(); ++i){
        real32 orientation {atan(positions[i].y/positions[i].x)};
        DrawPoly(positions[i], 3, Settings::BoidRadius, orientation, WHITE);
    }
}

// Boids are implemented as the vector indices
int main(){
    
    auto& positions {GLOBAL_BOIDS.positions};
    auto& orientations {GLOBAL_BOIDS.orientations};
    auto& velocities {GLOBAL_BOIDS.velocities};
    
    // Randomly determine the starting position and velocity of all boids.
    for (auto i {0uz}; i < GLOBAL_BOIDS.number; ++i){
        positions[i].x = Random::get(0.0f, Settings::MaxX);
        positions[i].y = Random::get(0.0f, Settings::MaxY);

        velocities[i].x = Random::get(Settings::Min1DVelocity, Settings::Max1DVelocity);
        velocities[i].y = Random::get(Settings::Min1DVelocity, Settings::Max1DVelocity);
    }

    InitWindow(static_cast<int>(Settings::ScreenWidth, Settings::ScreenHeight, "Boids");
    SetTargetFPS(Settings::FramesPerSecond);

    while (!WindowShouldClose()){
        ClearBackground(BLACK);
        BeginDrawing();
        DrawBoids();
        EndDrawing();
        UpdateBoids();
    }

    return 0;

}
