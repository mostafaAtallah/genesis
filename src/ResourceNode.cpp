#include "ResourceNode.hpp"

#include <algorithm>
#include <utility>

namespace genesis {

ResourceNode::ResourceNode(std::string name, float hardness, float durability, Color color, Vec2f position, Vec2f size)
    : name_(std::move(name)),
      hardness_(hardness),
      durability_(durability),
      body_(b2_nullBodyId),
      color_(color),
      position_(position),
      size_(size) {}

const std::string& ResourceNode::Name() const {
    return name_;
}

float ResourceNode::Hardness() const {
    return hardness_;
}

float ResourceNode::Durability() const {
    return durability_;
}

bool ResourceNode::IsHarvested() const {
    return harvested_;
}

const Color& ResourceNode::DisplayColor() const {
    return color_;
}

const Vec2f& ResourceNode::Position() const {
    return position_;
}

const Vec2f& ResourceNode::Size() const {
    return size_;
}

b2BodyId ResourceNode::PhysicsBody() const {
    return body_;
}

void ResourceNode::SetPhysicsBody(b2BodyId bodyId) {
    body_ = bodyId;
}

float ResourceNode::ApplyDamage(float amount) {
    if (harvested_) {
        return 0.0f;
    }

    durability_ = std::max(0.0f, durability_ - std::max(0.0f, amount));
    return durability_;
}

void ResourceNode::MarkHarvested() {
    harvested_ = true;
    body_ = b2_nullBodyId;
}

bool ResourceNode::ContainsPoint(const Vec2f& point) const {
    const float half_width = size_.x * 0.5f;
    const float half_height = size_.y * 0.5f;
    return point.x >= position_.x - half_width &&
           point.x <= position_.x + half_width &&
           point.y >= position_.y - half_height &&
           point.y <= position_.y + half_height;
}

}  // namespace genesis
