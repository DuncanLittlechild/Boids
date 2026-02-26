#include <raylib.h>
#include <raymath.h>
#include <vector>
#include <cmath>
#include <iostream>

#define RAYGUI_IMPLEMENTATION
#include "Random.h"
#include "Type_definitions.h"
#include"raygui.h"


constexpr int32 ScreenWidth {680};
constexpr int32 ScreenHeight {400};

// All values marked as max are non-inclusive
// Max x and y value a boid can have before it wraps around to 0
constexpr real32 MaxX{static_cast<real32>(ScreenWidth)};
constexpr real32 MaxY{static_cast<real32>(ScreenHeight)};

// Struct to hold all the parameters needed for/used in the simulation
struct SimulationSettings {
    // Ensures that boids will always keep moving slightly 
    real32 MinSpeed{2.0f};
    real32 MaxSpeed{10.0f};

    // The radius used to calculate the size of a boid
    real32 BoidRadius {4.0f};

    // a limit on the number of boids one boid can interact with at any one time
    // Included for performance reasons
    uint32 MaxInteractable {32};
    real32 InteractionDistance {100.0f};

    // The distance at which the separation rule begins to come into effect
    real32 MinSeparation {20.0f};

    real32 SeparationVectorMod {1.5f};
    real32 AlignmentVectorMod {1.0f};
    real32 CohesionVectorMod {1.0f};
    real32 ResultantForceMod {1.0f};

    uint32 FramesPerSecond {60};

    bool IsPaused {false};
};

SimulationSettings GLOBAL_SETTINGS{};

std::size_t boidNum{100};

struct Boids {
    std::size_t number;
    std::vector<Vector2> positions;
    std::vector<Vector2> velocities;
    std::vector<Vector2> alignment;
    std::vector<Vector2> cohesion;
    std::vector<Vector2> separation;
    
    Boids(std::size_t num)
        : number {num}
        , positions(num, Vector2{})
        , velocities(num, Vector2{})
        , alignment(num, Vector2{})
        , cohesion(num, Vector2{})
        , separation(num, Vector2{})
    {};
};

Boids GLOBAL_BOIDS{boidNum};

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
    return toWrap < wrapOn ? (toWrap >= 0 ? toWrap : wrapOn + toWrap) : toWrap - wrapOn;
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
                 &GLOBAL_SETTINGS.SeparationVectorMod, 0.0f, 5.0f);
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
    
    GuiLabel((Rectangle){sidebarX + 10, y, 180, 20}, "Separation Distance");
    y += 20;
    GuiSliderBar((Rectangle){sidebarX + 10, y, 180, 20}, 
                 NULL, TextFormat("%.0f", GLOBAL_SETTINGS.MinSeparation),
                 &GLOBAL_SETTINGS.MinSeparation, 5.0f, 500.0f);
    y += spacing;
    
    GuiLabel((Rectangle){sidebarX + 10, y, 180, 20}, "Interaction Radius");
    y += 20;
    GuiSliderBar((Rectangle){sidebarX + 10, y, 180, 20}, 
                 NULL, TextFormat("%.0f", GLOBAL_SETTINGS.InteractionDistance),
                 &GLOBAL_SETTINGS.InteractionDistance, 20.0f, 200.0f);
    y += spacing + 10;
    
    if (GuiButton((Rectangle){sidebarX + 10, y, 180, 30}, 
                  GLOBAL_SETTINGS.IsPaused ? "#131#Resume" : "#132#Pause")) {
        GLOBAL_SETTINGS.IsPaused = !GLOBAL_SETTINGS.IsPaused;
    }
    y += 40;
    
    if (GuiButton((Rectangle){sidebarX + 10, y, 180, 30}, 
                  "#77#Reset")) {
        GLOBAL_SETTINGS = SimulationSettings{};  // Reset to defaults
    }
}

// Gets the distance between two points, checking if wrapping the values makes them closer
// If it does, uses the wrapped distance
real32 GetWrappedDistance(const Vector2& a, const Vector2& b){
    real32 dX {abs(a.x - b.x)};
    real32 dY {abs(a.y - b.y)};

    if (dX > ScreenWidth / 2.0f) {
        dX = ScreenWidth - dX;
    }
    if (dY > ScreenHeight / 2.0f){
        dY = ScreenHeight - dY;
    }

    return sqrt(dX * dX + dY * dY);
}

// Gets the vector from the main boid to the centre of the group of boids contained in the inputted array
Vector2 GetCentreOfGroupVector(std::vector<std::size_t>& boidsInRange, Vector2& mainBoidPosition){
    Vector2 centreOfGroup {0.0, 0.0};
    if (!boidsInRange.size())
        return centreOfGroup;

    for (auto& i : boidsInRange){
        centreOfGroup += GLOBAL_BOIDS.positions[i];
    }

    centreOfGroup /= boidsInRange.size();
    centreOfGroup -= mainBoidPosition;
    
    return Vector2Normalize(centreOfGroup);
}

// Gets the average velocity vector of an array of vectors
// This is being used as the average heading vector
Vector2 GetAverageHeadingVector(std::vector<std::size_t>& boidsInRange){
    Vector2 totalVelocity {0.0f, 0.0f};
    if (!boidsInRange.size()){
        return totalVelocity;
    }
    
    for (auto& i : boidsInRange){
        totalVelocity += GLOBAL_BOIDS.velocities[i];
    }

    Vector2 averageVelocity {totalVelocity / boidsInRange.size()};
    
    return averageVelocity;
}

//Gets the force with which boids that are too close will repel the main boid
Vector2 GetSeparationVector(std::vector<std::size_t>& boidsTooClose, Vector2& mainBoidPosition){
    // Get the direction and the magnitude away from the too close boid, and turn that into a velocity vector
    Vector2 separationForceVector {0.0f, 0.0f};
    if (!boidsTooClose.size()){
        return separationForceVector;
    }
    for (auto& i : boidsTooClose){
        Vector2 diff {mainBoidPosition - GLOBAL_BOIDS.positions[i]};
        real32 distance {Vector2Length(diff)};
        if (distance > 0.0f){
            diff = Vector2Normalize(diff);
            diff /= distance;
            separationForceVector += diff;
        }
    }

    return separationForceVector;
}

Vector2 GetRForceVector(Vector2& cohesionForceVector, Vector2& alignmentForceVector, Vector2& separationForceVector){
     // Get a resultant vector from this
    return (cohesionForceVector * GLOBAL_SETTINGS.CohesionVectorMod 
            + alignmentForceVector * GLOBAL_SETTINGS.AlignmentVectorMod 
            + separationForceVector * GLOBAL_SETTINGS.SeparationVectorMod);
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
            real32 distance {GetWrappedDistance(GLOBAL_BOIDS.positions[i], GLOBAL_BOIDS.positions[j])};
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
        GLOBAL_BOIDS.cohesion[i] = GetCentreOfGroupVector(boidsInRange, GLOBAL_BOIDS.positions[i]);

        GLOBAL_BOIDS.alignment[i] = GetAverageHeadingVector(boidsInRange);
        GLOBAL_BOIDS.separation[i] = GetSeparationVector(boidsTooClose, GLOBAL_BOIDS.positions[i]);
    
           
        Vector2 resultantForceVector {GetRForceVector(GLOBAL_BOIDS.cohesion[i], GLOBAL_BOIDS.alignment[i], GLOBAL_BOIDS.separation[i])};
        // Draws resultant force
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
        real32 orientationDeg {atan2f(velocities[i].y, velocities[i].x) * (180 / PI)};
        Vector2 resultantForceVector {GetRForceVector(GLOBAL_BOIDS.cohesion[i], GLOBAL_BOIDS.alignment[i], GLOBAL_BOIDS.separation[i])};
        // Draw boid
        DrawPoly(positions[i], 3, GLOBAL_SETTINGS.BoidRadius, orientationDeg, WHITE);
        // Velocity Vector
        DrawLine(positions[i].x, positions[i].y, positions[i].x + velocities[i].x, positions[i].y + velocities[i].y, RED);
        // Iteraction distance 
        DrawCircleLines(positions[i].x, positions[i].y, GLOBAL_SETTINGS.InteractionDistance, Color{255,255,255,100});
        // Cohesion Force
        DrawLine(GLOBAL_BOIDS.positions[i].x, GLOBAL_BOIDS.positions[i].y, GLOBAL_BOIDS.positions[i].x + GLOBAL_BOIDS.cohesion[i].x, GLOBAL_BOIDS.positions[i].y +GLOBAL_BOIDS.cohesion[i].y, GREEN);
        // Alignment force
        DrawLine(GLOBAL_BOIDS.positions[i].x, GLOBAL_BOIDS.positions[i].y, GLOBAL_BOIDS.positions[i].x + GLOBAL_BOIDS.alignment[i].x, GLOBAL_BOIDS.positions[i].y + GLOBAL_BOIDS.alignment[i].y , PURPLE);
        // Separation force
        DrawLine(GLOBAL_BOIDS.positions[i].x, GLOBAL_BOIDS.positions[i].y, GLOBAL_BOIDS.positions[i].x + GLOBAL_BOIDS.separation[i].x, GLOBAL_BOIDS.positions[i].y + GLOBAL_BOIDS.separation[i].y, YELLOW);
        // Resultant force
        DrawLine(GLOBAL_BOIDS.positions[i].x, GLOBAL_BOIDS.positions[i].y, GLOBAL_BOIDS.positions[i].x + resultantForceVector.x, GLOBAL_BOIDS.positions[i].y + resultantForceVector.y, BLUE);
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
        DrawSimulationGUI();
        DrawBoids();
        EndDrawing();
        real32 deltaTime{GetFrameTime()};
        if (!GLOBAL_SETTINGS.IsPaused){
            UpdateBoids(deltaTime);
        }
    }

    return 0;

}