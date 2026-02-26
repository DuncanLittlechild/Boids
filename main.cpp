#include <raylib.h>
#include <vector>
#include <cmath>

#include "Random.h"
#include "Vector2_operators.h"

namespace Settings {
    int32 ScreenWidth {1280};
    int32 ScreenHeight {720};

    // All values marked as max are non-inclusive
    // Max x and y value a boid can have before it wraps around to 0
    real32 MaxX{static_cast<real32>(ScreenWidth)};
    real32 MaxY{static_cast<real32>(ScreenHeight)};

    // Ensures that boids will always keep moving slightly 
    real32 Min1DVelocity{-5.0f};
    real32 Max1DVelocity{5.0f};

    // The radius used to calculate the size of a boid
    real32 BoidRadius {8.0f};

    // a limit on the number of boids one boid can interact with at any one time
    // Included for performance reasons
    uint32 MaxInteractable {32};
    real32 InteractionDistance {10.0f};

    // The distance at which the separation rule begins to come into effect
    real32 MinSeparation {1.0f};
    // Factor by which separation vectors are multiplied
    real32 SeparationForceFactor {5.0f};
    // Maximum separation force in any one direction
    real32 MaxSeparationForce {5.0f};

    real32 SeparationVectorMod {1.0f};
    real32 AlignmentVectorMod {1.0f};
    real32 CohesionVectorMod {1.0f};
    real32 ResultantForceMod {1.0f};

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

template <typename T>
T ClampTo(T num, T limit){
    return num <= limit ? num : limit;
}

void ClampVec(Vector2& vec, real32 lowerLimit, real32 upperLimit){
    vec.x = vec.x >= lowerLimit ? (vec.x < upperLimit ? vec.x : upperLimit) : lowerLimit;
    vec.y = vec.y >= lowerLimit ? (vec.y < upperLimit ? vec.y : upperLimit) : lowerLimit;
}

// Wraps a number on a target, ensuring it stays in the range 0 <= x < wrapOn
real32 WrapOn(real32 toWrap, real32 wrapOn) {
    return toWrap < wrapOn ? (toWrap >= 0 ? toWrap : wrapOn - toWrap) : toWrap - wrapOn;
}

real32 GetDistance(const Vector2& a, const Vector2& b){
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

// Gets the vector from the main boid to the centre of the group of boids contained in the inputted array
Vector2 GetCentreOfGroupVector(std::vector<std::size_t>& boidsInRange, Vector2& mainBoidPosition){
    Vector2 centreOfGroup {0.0, 0.0};
    if (!boidsInRange.size())
        return centreOfGroup;

    for (auto& i : boidsInRange){
        centreOfGroup.x += GLOBAL_BOIDS.positions[i].x;
        centreOfGroup.y += GLOBAL_BOIDS.positions[i].y;
    }
    
    std::size_t arraySize {boidsInRange.size()};
    if (arraySize){
        centreOfGroup.x /= boidsInRange.size();
        centreOfGroup.y /= boidsInRange.size();
    }
    
    return {centreOfGroup.x - mainBoidPosition.x, centreOfGroup.y - mainBoidPosition.y};
}

// Gets the average velocity vector of an array of vectors
// This is being used as the average heading vector
Vector2 GetAverageHeadingVector(std::vector<std::size_t>& boidsInRange){
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
Vector2 GetSeparationForceVector(std::vector<std::size_t>& boidsTooClose, Vector2& mainBoidPosition){
    // Get the direction and the magnitude away from the too close boid, and turn that into a velocity vector
    Vector2 separationForceVector {0.0f, 0.0f};
    for (auto& i : boidsTooClose){
        Vector2 diff {mainBoidPosition.x - GLOBAL_BOIDS.positions[i].x, mainBoidPosition.y - GLOBAL_BOIDS.positions[i].y};
        
        separationForceVector.x += ClampTo(Settings::MinSeparation - diff.x, Settings::MaxSeparationForce);
        separationForceVector.y += ClampTo(Settings::MinSeparation - diff.y, Settings::MaxSeparationForce);
    }

    return separationForceVector;
}

void UpdateBoids(real32 deltaTime) {
    // Apply the rules to the boids
    // TODO: consider memoisation for this
    // This for loop just updates velocity - position is dealt with later
    for (auto i {0uz}; i < GLOBAL_BOIDS.number; ++i){
        std::vector<std::size_t> boidsInRange;
        boidsInRange.reserve(Settings::MaxInteractable);
        
        std::vector<std::size_t> boidsTooClose;
        boidsTooClose.reserve(16);
        
        for (auto j {0uz}; j < GLOBAL_BOIDS.number; ++j){
            if (i == j)
                continue;
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
        // Generate vectors to represent each of the forces the boid is under. This obtained by getting relevant vectors then subtracting the boids current velocity from this vector
        Vector2 resultantForceVector{0.0f, 0.0f};
        if (boidsInRange.size()){
            Vector2 cohesionForceVector {0.0f, 0.0f};
            Vector2 alignmentForceVector {0.0f, 0.0f};
            Vector2 separationForceVector{0.0f, 0.0f};
            cohesionForceVector = GetCentreOfGroupVector(boidsInRange, GLOBAL_BOIDS.positions[i]);

            alignmentForceVector = GetAverageHeadingVector(boidsInRange);
            DrawLine(GLOBAL_BOIDS.positions[i].x, GLOBAL_BOIDS.positions[i].y, GLOBAL_BOIDS.positions[i].x + cohesionForceVector.x * 10, GLOBAL_BOIDS.positions[i].y +cohesionForceVector.y, GREEN);
            DrawLine(GLOBAL_BOIDS.positions[i].x, GLOBAL_BOIDS.positions[i].y, GLOBAL_BOIDS.positions[i].x + alignmentForceVector.x * 10, GLOBAL_BOIDS.positions[i].y + alignmentForceVector.y , PURPLE);

            separationForceVector = GetSeparationForceVector(boidsTooClose, GLOBAL_BOIDS.positions[i]);
        
            DrawLine(GLOBAL_BOIDS.positions[i].x, GLOBAL_BOIDS.positions[i].y, GLOBAL_BOIDS.positions[i].x + separationForceVector.x * 10, GLOBAL_BOIDS.positions[i].y + separationForceVector.y * 10, YELLOW);
            // Get a resultant vector from this
            resultantForceVector =
                (cohesionForceVector * Settings::SeparationVectorMod 
                + alignmentForceVector * Settings::AlignmentVectorMod 
                + separationForceVector * Settings::SeparationVectorMod)
                - GLOBAL_BOIDS.velocities[i];
        }

        // Draws resultant force
        DrawLine(GLOBAL_BOIDS.positions[i].x, GLOBAL_BOIDS.positions[i].y, GLOBAL_BOIDS.positions[i].x + resultantForceVector.x * 10, GLOBAL_BOIDS.positions[i].y + resultantForceVector.y * 10, BLUE);
        // Modify the boid's direction towards the target
        ClampVec(GLOBAL_BOIDS.velocities[i] += resultantForceVector * (1.0f/60.0f) * Settings::ResultantForceMod, 
            Settings::Min1DVelocity, 
            Settings::Max1DVelocity);
    }

    // Use velocity to update position. If they would move off the screen, wrap around to the other side
    for(auto i {0uz}; i < GLOBAL_BOIDS.number; ++i){
        GLOBAL_BOIDS.positions[i].x = WrapOn(GLOBAL_BOIDS.positions[i].x + GLOBAL_BOIDS.velocities[i].x, Settings::ScreenWidth);
        GLOBAL_BOIDS.positions[i].y = WrapOn(GLOBAL_BOIDS.positions[i].y + GLOBAL_BOIDS.velocities[i].y, Settings::ScreenHeight);
    }
}

void DrawBoids() {
    auto& positions {GLOBAL_BOIDS.positions};
    auto& velocities {GLOBAL_BOIDS.velocities};
    for (auto i {0uz}; i < positions.size(); ++i){
        real32 orientationDeg {static_cast<real32>(atan(velocities[i].y/velocities[i].x)) * (180 / PI)};
        DrawPoly(positions[i], 3, Settings::BoidRadius, orientationDeg, WHITE);
        DrawLine(positions[i].x, positions[i].y, positions[i].x + velocities[i].x * 10, positions[i].y + velocities[i].y * 10, RED);
    }
}

// Boids are implemented as the vector indices
int main(){
    
    auto& positions {GLOBAL_BOIDS.positions};
    auto& velocities {GLOBAL_BOIDS.velocities};
    
    // Randomly determine the starting position and velocity of all boids.
    for (auto i {0uz}; i < GLOBAL_BOIDS.number; ++i){
        positions[i].x = Random::get(0.0f, Settings::MaxX);
        positions[i].y = Random::get(0.0f, Settings::MaxY);

        velocities[i].x = Random::get(Settings::Min1DVelocity, Settings::Max1DVelocity);
        velocities[i].y = Random::get(Settings::Min1DVelocity, Settings::Max1DVelocity);
    }

    InitWindow(Settings::ScreenWidth, Settings::ScreenHeight, "Boids");
    SetTargetFPS(Settings::FramesPerSecond);

    while (!WindowShouldClose()){
        ClearBackground(BLACK);
        BeginDrawing();
        DrawBoids();
        EndDrawing();
        real32 deltaTime{GetFrameTime()};
        UpdateBoids(deltaTime);
    }

    return 0;

}