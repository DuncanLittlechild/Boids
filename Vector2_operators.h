#ifndef VECTOR2_OPERATORS_H
#define VECTOR2_OPERATORS_H
#include <raylib.h>

#include "Type_definitions.h"

inline Vector2 operator*(Vector2 vec, real32 mod){
    return Vector2{vec.x * mod, vec.y * mod};
}

inline Vector2 operator+(Vector2 vec1, Vector2 vec2){
    return Vector2{vec1.x + vec2.x, vec1.y + vec2.y};
}

inline Vector2 operator-(Vector2 vec1, Vector2 vec2){
    return Vector2{vec1.x - vec2.x, vec1.y - vec2.y};
}

inline Vector2& operator+=(Vector2& vec1, Vector2 vec2){
    vec1.x += vec2.x;
    vec1.y += vec2.y;
    return vec1;
}
#endif