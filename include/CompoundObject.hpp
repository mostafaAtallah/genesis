#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "Component.hpp"

namespace genesis {

struct StructuralJoint {
    std::size_t first = 0;
    std::size_t second = 0;
    Vec2f local_offset{0.0f, 0.0f};
    float max_shear_force = 1200.0f;
    float stress_load = 0.0f;
    bool broken = false;
    bool isInterlocking = false;
};

using JointLink = StructuralJoint;

struct CollisionEvent {
    std::size_t component_index = 0;
    Vec2f impulse{0.0f, 0.0f};
    float magnitude = 0.0f;
};

class CompoundObject {
public:
    CompoundObject() = default;
    explicit CompoundObject(std::string label);

    std::size_t AddComponent(std::shared_ptr<Component> component);
    void Connect(std::size_t first, std::size_t second, float max_shear_force = 1200.0f);
    int HitTest(const Vec2f& point) const;
    void BeginDrag(std::size_t component_index, const Vec2f& mouse_position);
    void UpdateDrag(const Vec2f& mouse_position, float dt);
    void EndDrag();
    void Step(float dt, float floor_y, float left_x, float right_x);
    void ProcessCollisionImpact(std::size_t component_index, const Vec2f& impulse);
    std::vector<CollisionEvent> ConsumeCollisionEvents();
    std::vector<CompoundObject> ValidateStructuralIntegrity();
    std::vector<CompoundObject> ConsumeOrphanedObjects();
    void ApplyStrain(float delta);
    std::size_t Size() const;
    std::size_t JointCount() const;
    std::size_t BrokenJointCount() const;
    float StructuralStrain() const;
    bool IsSplit() const;
    const std::vector<std::shared_ptr<Component>>& Components() const;
    const std::vector<JointLink>& Joints() const;
    const std::string& Label() const;
    std::size_t ActiveComponentIndex() const;
    float ActiveJointStress() const;
    float ActiveJointMaxShear() const;
    bool ActiveJointIsInterlocking() const;
    std::string ActiveJointTypeLabel() const;
    bool HasJoints() const;
    void SetWorkbenchActive(bool value);
    bool WorkbenchActive() const;
    void SetSwinging(bool value);
    bool Swinging() const;
    void SetActiveResourceTarget(std::string name, float hardness, float durability);
    void ClearActiveResourceTarget();
    bool HasActiveResourceTarget() const;
    const std::string& ActiveResourceName() const;
    float ActiveResourceHardness() const;
    float ActiveResourceDurability() const;
    float LastImpactForce() const;
    float LastJointStress() const;
    void AddFeedbackStress(float magnitude);
    void MarkJointInterlocking(std::size_t joint_index, bool value);
    bool HasOrphanedObjects() const;

private:
    friend void ProcessCollisionImpact(CompoundObject* tool, float normalImpulse);

    std::string label_ = "compound";
    std::vector<std::shared_ptr<Component>> components_;
    std::vector<JointLink> joints_;
    std::vector<CollisionEvent> collision_events_;
    std::vector<CompoundObject> orphan_buffer_;
    std::size_t active_joint_index_ = 0;
    std::size_t dragged_component_ = static_cast<std::size_t>(-1);
    Vec2f drag_offset_{0.0f, 0.0f};
    Vec2f drag_velocity_{0.0f, 0.0f};
    Vec2f previous_drag_mouse_{0.0f, 0.0f};
    float structural_strain_ = 0.0f;
    bool workbench_active_ = false;
    bool swinging_ = false;
    std::string active_resource_name_;
    float active_resource_hardness_ = 0.0f;
    float active_resource_durability_ = 0.0f;
    float last_impact_force_ = 0.0f;
    float last_joint_stress_ = 0.0f;
    float simulation_time_ = 0.0f;
};

void ProcessCollisionImpact(CompoundObject* tool, float normalImpulse);

}  // namespace genesis
