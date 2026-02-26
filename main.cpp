#include <raylib.h>
#include <raymath.h>
#include <vector>
#include <cmath>

#define RAYGUI_IMPLEMENTATION
#include "Random.h"
#include "Type_definitions.h"
#include"raygui.h"


constexpr int32 ScreenWidth {1280};
constexpr int32 ScreenHeight {720};

// All values marked as max are non-inclusive
// Max x and y value a boid can have before it wraps around to 0
constexpr real32 MaxX{static_cast<real32>(ScreenWidth)};
constexpr real32 MaxY{static_cast<real32>(ScreenHeight)};

// Struct to hold all the parameters needed for/used in the simulation
struct SimulationSettings {
    // Ensures that boids will always keep moving slightly 
    real32 MinSpeed{1.0f};
    real32 MaxSpeed{5.0f};

    // The radius used to calculate the size of a boid
    real32 BoidRadius {4.0f};

    // a limit on the number of boids one boid can interact with at any one time
    // Included for performance reasons
    uint32 MaxInteractable {32};
    real32 InteractionDistance {50.0f};

    // The distance at which the separation rule begins to come into effect
    real32 MinSeparation {7.0f};

    real32 SeparationVectorMod {2.0f};
    real32 AlignmentVectorMod {0.5f};
    real32 CohesionVectorMod {0.5f};
    real32 ResultantForceMod {1.0f};

    uint32 FramesPerSecond {60};
};

SimulationSettings GLOBAL_SETTINGS{};

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
    vec.x = Clamp(vec.x, lowerLimit, upperLimit);
    vec.y = Clamp(vec.y, lowerLimit, upperLimit);
}

void ClampSpeed(Vector2& velocity, real32 minSpeed, real32 maxSpeed){
    real32 speed {Vector2Length(velocity)};
    if (speed > 1.0e-5){
        if (speed < minSpeed){
            velocity.x = (velocity.x / speed) * minSpeed;
            velocity.y = (velocity.y / speed) * minSpeed;
        }
        else if (speed >= maxSpeed){
            velocity.x = (velocity.x / speed) * maxSpeed;
            velocity.y = (velocity.y / speed) * maxSpeed;
        }
    }
}

// Wraps a number on a target, ensuring it stays in the range 0 <= x < wrapOn
real32 WrapOn(real32 toWrap, real32 wrapOn) {
    return toWrap < wrapOn ? (toWrap >= 0 ? toWrap : wrapOn - toWrap) : toWrap - wrapOn;
}

void DrawSimulationGUI() {
    const int sidebarX = 1000;
    const int sidebarWidth = 200;
    const int startY = 20;
    const int spacing = 40;
    
    DrawRectangle(sidebarX, 0, sidebarWidth, GetScreenHeight(), 
                  Fade(LIGHTGRAY, 0.9f));
    
    real32 y = static_cast<real32>(startY);
    
    GuiGroupBox((Rectangle){sidebarX + 5, y, sidebarWidth - 10, 300}, 
                "Simulation Controls");
    y += 30;
    
    GuiLabel((Rectangle){sidebarX + 10, y, 180, 20}, "Separation Force Mod");
    y += 20;
    GuiSliderBar((Rectangle){sidebarX + 10, y, 180, 20}, 
                 NULL, TextFormat("%.2f", GLOBAL_SETTINGS.SeparationVectorMod),
                 &GLOBAL_SETTINGS.MinSeparation, 0.0f, 5.0f);
    y += spacing;
    
    GuiLabel((Rectangle){sidebarX + 10, y, 180, 20}, "Alignment Force Mod");
    y += 20;
    GuiSliderBar((Rectangle){sidebarX + 10, y, 180, 20}, 
                 NULL, TextFormat("%.2f", GLOBAL_SETTINGS.AlignmentVectorMod),
                 &GLOBAL_SETTINGS.AlignmentVectorMod, 0.0f, 5.0f);
    y += spacing;
    
    GuiLabel((Rectangle){sidebarX + 10, y, 180, 20}, "Cohesion Force Mod");
    y += 20;
    GuiSliderBar((Rectangle){sidebarX + 10, y, 180, 20}, 
                 NULL, TextFormat("%.2f", GLOBAL_SETTINGS.CohesionVectorMod),
                 &GLOBAL_SETTINGS.CohesionVectorMod, 0.0f, 5.0f);
    y += spacing;
    /*
    GuiLabel((Rectangle){sidebarX + 10, y, 180, 20}, "Max Speed");
    y += 20;
    GuiSliderBar((Rectangle){sidebarX + 10, y, 180, 20}, 
                 NULL, TextFormat("%.0f", GLOBAL_SETTINGS.maxSpeed),
                 &GLOBAL_SETTINGS.maxSpeed, 50.0f, 500.0f);
    y += spacing;
    
    GuiLabel((Rectangle){sidebarX + 10, y, 180, 20}, "Perception Radius");
    y += 20;
    GuiSliderBar((Rectangle){sidebarX + 10, y, 180, 20}, 
                 NULL, TextFormat("%.0f", GLOBAL_SETTINGS.perceptionRadius),
                 &GLOBAL_SETTINGS.perceptionRadius, 20.0f, 200.0f);
    y += spacing + 10;
    
    if (GuiButton((Rectangle){sidebarX + 10, y, 180, 30}, 
                  GLOBAL_SETTINGS.isPaused ? "#131#Resume" : "#132#Pause")) {
        GLOBAL_SETTINGS.isPaused = !GLOBAL_SETTINGS.isPaused;
    }
    y += 40;
    
    if (GuiButton((Rectangle){sidebarX + 10, y, 180, 30}, 
                  "#77#Reset")) {
        GLOBAL_SETTINGS = SimulationSettings{};  // Reset to defaults
    }
        */
}

real32 GetDistance(const Vector2& a, const Vector2& b){
    return Vector2Length(Vector2{a.x-b.x, a.y-b.y});
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
        totalVelocity.x /= arraySize;
        totalVelocity.y /= arraySize;
    }
    
    return totalVelocity;
}

//Gets the force with which boids that are too close will repel the main boid
Vector2 GetSeparationForceVector(std::vector<std::size_t>& boidsTooClose, Vector2& mainBoidPosition){
    // Get the direction and the magnitude away from the too close boid, and turn that into a velocity vector
    Vector2 separationForceVector {0.0f, 0.0f};
    for (auto& i : boidsTooClose){
        Vector2 diff {mainBoidPosition.x - GLOBAL_BOIDS.positions[i].x, mainBoidPosition.y - GLOBAL_BOIDS.positions[i].y};
        
        separationForceVector.x += GLOBAL_SETTINGS.MinSeparation - diff.x;
        separationForceVector.y += GLOBAL_SETTINGS.MinSeparation - diff.y;
    }

    return separationForceVector;
}

void UpdateBoids(real32 deltaTime) {
    // Apply the rules to the boids
    // TODO: consider memoisation for this
    // This for loop just updates velocity - position is dealt with later
    for (auto i {0uz}; i < GLOBAL_BOIDS.number; ++i){
        std::vector<std::size_t> boidsInRange;
        boidsInRange.reserve(GLOBAL_SETTINGS.MaxInteractable);
        
        std::vector<std::size_t> boidsTooClose;
        boidsTooClose.reserve(16);
        
        for (auto j {0uz}; j < GLOBAL_BOIDS.number; ++j){
            if (i == j)
                continue;
            real32 distance {GetDistance(GLOBAL_BOIDS.positions[i], GLOBAL_BOIDS.positions[j])};
            if (distance < GLOBAL_SETTINGS.InteractionDistance){
                 boidsInRange.push_back(j);
                if (distance < GLOBAL_SETTINGS.MinSeparation){
                    boidsTooClose.push_back(j);
                }
                if (boidsInRange.size() >= GLOBAL_SETTINGS.MaxInteractable){
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
            DrawLine(GLOBAL_BOIDS.positions[i].x, GLOBAL_BOIDS.positions[i].y, GLOBAL_BOIDS.positions[i].x + cohesionForceVector.x * 5, GLOBAL_BOIDS.positions[i].y +cohesionForceVector.y, GREEN);
            DrawLine(GLOBAL_BOIDS.positions[i].x, GLOBAL_BOIDS.positions[i].y, GLOBAL_BOIDS.positions[i].x + alignmentForceVector.x * 5, GLOBAL_BOIDS.positions[i].y + alignmentForceVector.y , PURPLE);

            separationForceVector = GetSeparationForceVector(boidsTooClose, GLOBAL_BOIDS.positions[i]);
        
            DrawLine(GLOBAL_BOIDS.positions[i].x, GLOBAL_BOIDS.positions[i].y, GLOBAL_BOIDS.positions[i].x + separationForceVector.x * 5, GLOBAL_BOIDS.positions[i].y + separationForceVector.y * 10, YELLOW);
            // Get a resultant vector from this
            resultantForceVector =
                (cohesionForceVector * GLOBAL_SETTINGS.SeparationVectorMod 
                + alignmentForceVector * GLOBAL_SETTINGS.AlignmentVectorMod 
                + separationForceVector * GLOBAL_SETTINGS.SeparationVectorMod);
        }

        // Draws resultant force
        DrawLine(GLOBAL_BOIDS.positions[i].x, GLOBAL_BOIDS.positions[i].y, GLOBAL_BOIDS.positions[i].x + resultantForceVector.x * 5, GLOBAL_BOIDS.positions[i].y + resultantForceVector.y * 10, BLUE);
        // Modify the boid's direction towards the target
        GLOBAL_BOIDS.velocities[i] += resultantForceVector * (1.0f/60.0f) * GLOBAL_SETTINGS.ResultantForceMod;
        ClampSpeed(GLOBAL_BOIDS.velocities[i], 
            GLOBAL_SETTINGS.MinSpeed, 
            GLOBAL_SETTINGS.MaxSpeed);
    }

    // Use velocity to update position. If they would move off the screen, wrap around to the other side
    for(auto i {0uz}; i < GLOBAL_BOIDS.number; ++i){
        GLOBAL_BOIDS.positions[i].x = WrapOn(GLOBAL_BOIDS.positions[i].x + GLOBAL_BOIDS.velocities[i].x, ScreenWidth);
        GLOBAL_BOIDS.positions[i].y = WrapOn(GLOBAL_BOIDS.positions[i].y + GLOBAL_BOIDS.velocities[i].y, ScreenHeight);
    }
}

void DrawBoids() {
    auto& positions {GLOBAL_BOIDS.positions};
    auto& velocities {GLOBAL_BOIDS.velocities};
    for (auto i {0uz}; i < positions.size(); ++i){
        real32 orientationDeg {static_cast<real32>(atan(velocities[i].y/velocities[i].x)) * (180 / PI)};
        DrawPoly(positions[i], 3, GLOBAL_SETTINGS.BoidRadius, orientationDeg, WHITE);
        DrawLine(positions[i].x, positions[i].y, positions[i].x + velocities[i].x * 10, positions[i].y + velocities[i].y * 10, RED);
    }
}

// Boids are implemented as the vector indices
int main(){
    
    auto& positions {GLOBAL_BOIDS.positions};
    auto& velocities {GLOBAL_BOIDS.velocities};
    
    // Randomly determine the starting position and velocity of all boids.
    for (auto i {0uz}; i < GLOBAL_BOIDS.number; ++i){
        positions[i].x = Random::get(0.0f, MaxX);
        positions[i].y = Random::get(0.0f, MaxY);

        real32 angle {Random::get(-1.0f, 1.0f)};
        real32 speed {Random::get(GLOBAL_SETTINGS.MinSpeed, GLOBAL_SETTINGS.MaxSpeed)};



        velocities[i].x = cosf(angle * PI) * speed;
        velocities[i].y = sinf(angle * PI) * speed;
    }

    InitWindow(ScreenWidth, ScreenHeight, "Boids");
    SetTargetFPS(GLOBAL_SETTINGS.FramesPerSecond);

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