#pragma once
#include <cmath>
 
struct Coord
{
    float x;
    float y;
 
    Coord operator+(const Coord& c) const { return { x + c.x, y + c.y }; }
    Coord operator-(const Coord& c) const { return { x - c.x, y - c.y }; }
    Coord operator*(float s)        const { return { x * s, y * s }; }
    Coord operator/(float s)        const { return { x / s, y / s }; }
    bool  operator==(const Coord& c) const { return x == c.x && y == c.y; }
};
 
inline float length(Coord c)
{
    return std::hypot(c.x, c.y);
}
 
inline Coord normalize(Coord c)
{
    float len = length(c);
    if (len == 0.0f) return { 0.0f, 0.0f };
    return { c.x / len, c.y / len };
}

// Heading (radians) of the vector c, measured from the +x axis.
inline float angleOf(Coord c)
{
    return std::atan2(c.y, c.x);
}

// Wrap an angle to the range (-pi, pi].
inline float normalizeAngle(float a)
{
    constexpr float pi  = 3.14159265358979323846f;
    constexpr float tau = 2.0f * pi;
    while (a >  pi) a -= tau;
    while (a < -pi) a += tau;
    return a;
}

// Unit direction vector for a heading (radians).
inline Coord dirVec(float angle)
{
    return { std::cos(angle), std::sin(angle) };
}