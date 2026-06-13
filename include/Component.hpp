#pragma once

#include <array>
#include <cmath>
#include <string>

namespace genesis {

enum class ComponentTopology {
    PrimitiveSolid,
    PiercedHollow
};

struct Vec2f {
    float x = 0.0f;
    float y = 0.0f;
};

inline Vec2f operator+(const Vec2f& lhs, const Vec2f& rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y};
}

inline Vec2f operator-(const Vec2f& lhs, const Vec2f& rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y};
}

inline Vec2f operator*(const Vec2f& value, float scalar) {
    return {value.x * scalar, value.y * scalar};
}

inline Vec2f operator*(float scalar, const Vec2f& value) {
    return value * scalar;
}

inline Vec2f& operator+=(Vec2f& lhs, const Vec2f& rhs) {
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    return lhs;
}

inline Vec2f& operator-=(Vec2f& lhs, const Vec2f& rhs) {
    lhs.x -= rhs.x;
    lhs.y -= rhs.y;
    return lhs;
}

inline float Length(const Vec2f& value) {
    return std::sqrt(value.x * value.x + value.y * value.y);
}

struct MaterialProperties {
    std::array<float, 3> vector{0.0f, 0.0f, 0.0f};
    float density = 0.0f;
    float hardness = 0.0f;
    float elasticity = 0.0f;
};

struct Component {
    std::string name;
    MaterialProperties material;
    Vec2f position{0.0f, 0.0f};
    Vec2f velocity{0.0f, 0.0f};
    Vec2f size{110.0f, 54.0f};
    std::array<float, 3> dimensions{1.0f, 1.0f, 1.0f};
    ComponentTopology topology = ComponentTopology::PrimitiveSolid;
    bool anchored = false;

    float Volume() const;
    float Mass() const;
    bool IsPiercedHollow() const;
};

}  // namespace genesis
