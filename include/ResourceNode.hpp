#pragma once

#include <string>

#include "box2d/box2d.h"
#include "raylib.h"

#include "Component.hpp"

namespace genesis {

class ResourceNode {
public:
    ResourceNode() = default;
    ResourceNode(std::string name, float hardness, float durability, Color color, Vec2f position, Vec2f size);

    const std::string& Name() const;
    float Hardness() const;
    float Durability() const;
    bool IsHarvested() const;
    const Color& DisplayColor() const;
    const Vec2f& Position() const;
    const Vec2f& Size() const;
    b2BodyId PhysicsBody() const;
    void SetPhysicsBody(b2BodyId bodyId);

    float ApplyDamage(float amount);
    void MarkHarvested();
    bool ContainsPoint(const Vec2f& point) const;

private:
    std::string name_;
    float hardness_ = 0.0f;
    float durability_ = 0.0f;
    b2BodyId body_ = b2_nullBodyId;
    Color color_{};
    Vec2f position_{};
    Vec2f size_{};
    bool harvested_ = false;
};

}  // namespace genesis
