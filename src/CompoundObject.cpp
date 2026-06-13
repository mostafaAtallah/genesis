#include "CompoundObject.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <queue>
#include <utility>

namespace genesis {

float Component::Volume() const {
    return dimensions[0] * dimensions[1] * dimensions[2];
}

float Component::Mass() const {
    return std::max(0.25f, Volume() * std::max(0.1f, material.density));
}

bool Component::IsPiercedHollow() const {
    return topology == ComponentTopology::PiercedHollow;
}

namespace {

struct Bounds {
    float left = 0.0f;
    float right = 0.0f;
    float top = 0.0f;
    float bottom = 0.0f;
};

Bounds ComputeBounds(const Component& component) {
    const float half_width = component.size.x * 0.5f;
    const float half_height = component.size.y * 0.5f;
    return Bounds{
        component.position.x - half_width,
        component.position.x + half_width,
        component.position.y - half_height,
        component.position.y + half_height
    };
}

bool IsFullyContained(const Component& inner, const Component& outer) {
    const Bounds inner_bounds = ComputeBounds(inner);
    const Bounds outer_bounds = ComputeBounds(outer);
    return inner_bounds.left >= outer_bounds.left &&
           inner_bounds.right <= outer_bounds.right &&
           inner_bounds.top >= outer_bounds.top &&
           inner_bounds.bottom <= outer_bounds.bottom;
}

bool CanInterlock(const Component& first, const Component& second) {
    if (first.IsPiercedHollow() == second.IsPiercedHollow()) {
        return false;
    }

    const Component& solid = first.IsPiercedHollow() ? second : first;
    const Component& hollow = first.IsPiercedHollow() ? first : second;
    return solid.topology == ComponentTopology::PrimitiveSolid &&
           hollow.topology == ComponentTopology::PiercedHollow &&
           IsFullyContained(solid, hollow);
}

}  // namespace

CompoundObject::CompoundObject(std::string label) : label_(std::move(label)) {}

std::size_t CompoundObject::AddComponent(std::shared_ptr<Component> component) {
    if (!component) {
        return components_.size();
    }

    components_.push_back(std::move(component));
    return components_.size() - 1U;
}

void CompoundObject::Connect(std::size_t first, std::size_t second, float max_shear_force) {
    if (first >= components_.size() || second >= components_.size() || first == second) {
        return;
    }

    const Vec2f offset = components_[second]->position - components_[first]->position;
    const bool interlocking = CanInterlock(*components_[first], *components_[second]);
    const float shear_force = std::max(10.0f, max_shear_force * (interlocking ? 5.0f : 1.0f));
    joints_.push_back(JointLink{first, second, offset, shear_force, 0.0f, false, interlocking});
}

int CompoundObject::HitTest(const Vec2f& point) const {
    for (std::size_t i = components_.size(); i > 0; --i) {
        const std::size_t index = i - 1U;
        const auto& component = *components_[index];
        const float half_width = component.size.x * 0.5f;
        const float half_height = component.size.y * 0.5f;
        const bool inside_x = point.x >= component.position.x - half_width && point.x <= component.position.x + half_width;
        const bool inside_y = point.y >= component.position.y - half_height && point.y <= component.position.y + half_height;
        if (inside_x && inside_y) {
            return static_cast<int>(index);
        }
    }

    return -1;
}

void CompoundObject::BeginDrag(std::size_t component_index, const Vec2f& mouse_position) {
    if (component_index >= components_.size()) {
        return;
    }

    dragged_component_ = component_index;
    drag_offset_ = components_[component_index]->position - mouse_position;
    previous_drag_mouse_ = mouse_position;
    drag_velocity_ = {0.0f, 0.0f};
    components_[component_index]->velocity = {0.0f, 0.0f};
}

void CompoundObject::UpdateDrag(const Vec2f& mouse_position, float dt) {
    if (dragged_component_ >= components_.size()) {
        return;
    }

    auto& component = *components_[dragged_component_];
    const Vec2f target = mouse_position + drag_offset_;
    const Vec2f delta = mouse_position - previous_drag_mouse_;

    component.velocity = dt > 0.0f ? delta * (1.0f / dt) : Vec2f{0.0f, 0.0f};
    component.position = target;

    drag_velocity_ = component.velocity;
    previous_drag_mouse_ = mouse_position;
}

void CompoundObject::EndDrag() {
    if (dragged_component_ < components_.size()) {
        components_[dragged_component_]->velocity = drag_velocity_;
    }

    dragged_component_ = static_cast<std::size_t>(-1);
}

void CompoundObject::Step(float dt, float floor_y, float left_x, float right_x) {
    collision_events_.clear();
    simulation_time_ += dt;

    constexpr float gravity = 1600.0f;
    constexpr float air_damping = 0.995f;
    constexpr float restitution = 0.18f;

    for (std::size_t i = 0; i < components_.size(); ++i) {
        auto& component = *components_[i];

        if (component.anchored || i == dragged_component_) {
            continue;
        }

        if (swinging_ && i + 1U == components_.size()) {
            component.velocity.x += std::sin(simulation_time_ * 8.0f) * 22.0f * dt;
            component.velocity.y += std::cos(simulation_time_ * 6.0f) * 14.0f * dt;
        }

        component.velocity.y += gravity * dt;
        component.position += component.velocity * dt;
        component.velocity.x *= air_damping;
        component.velocity.y *= air_damping;

        const float half_width = component.size.x * 0.5f;
        const float half_height = component.size.y * 0.5f;

        if (component.position.x - half_width < left_x) {
            component.position.x = left_x + half_width;
            component.velocity.x = std::abs(component.velocity.x) * restitution;
        } else if (component.position.x + half_width > right_x) {
            component.position.x = right_x - half_width;
            component.velocity.x = -std::abs(component.velocity.x) * restitution;
        }

        if (component.position.y + half_height > floor_y) {
            const float impact_speed = std::max(0.0f, component.velocity.y);
            const float impulse_magnitude = component.Mass() * impact_speed * (1.0f + restitution);
            component.position.y = floor_y - half_height;
            component.velocity.y = -impact_speed * restitution;

            const Vec2f impulse{0.0f, impulse_magnitude};
            ProcessCollisionImpact(i, impulse);
            collision_events_.push_back(CollisionEvent{i, impulse, impulse_magnitude});
        }
    }

    if (dragged_component_ < components_.size()) {
        auto& component = *components_[dragged_component_];
        const float half_width = component.size.x * 0.5f;
        const float half_height = component.size.y * 0.5f;
        component.position.x = std::clamp(component.position.x, left_x + half_width, right_x - half_width);
        component.position.y = std::min(component.position.y, floor_y - half_height);
    }

    for (auto& joint : joints_) {
        if (joint.broken) {
            continue;
        }

        auto& first = *components_[joint.first];
        auto& second = *components_[joint.second];
        const Vec2f desired_second = first.position + joint.local_offset;
        const Vec2f error = second.position - desired_second;

        if (!first.anchored && joint.first != dragged_component_) {
            first.position += error * 0.25f;
            first.velocity += error * (-4.0f);
        }

        if (!second.anchored && joint.second != dragged_component_) {
            second.position -= error * 0.75f;
            second.velocity += error * (4.0f);
        }
    }
}

void CompoundObject::ProcessCollisionImpact(std::size_t component_index, const Vec2f& impulse) {
    const float magnitude = Length(impulse);
    last_impact_force_ = magnitude;
    structural_strain_ += magnitude * 0.0002f;

    for (auto& joint : joints_) {
        if (joint.broken) {
            continue;
        }

        if (joint.first == component_index || joint.second == component_index) {
            joint.stress_load += magnitude;
            last_joint_stress_ = joint.stress_load;
            if (magnitude >= joint.max_shear_force) {
                joint.broken = true;
                structural_strain_ += 0.5f;
            }
        }
    }
}

std::vector<CollisionEvent> CompoundObject::ConsumeCollisionEvents() {
    std::vector<CollisionEvent> events;
    events.swap(collision_events_);
    return events;
}

std::vector<CompoundObject> CompoundObject::ValidateStructuralIntegrity() {
    std::vector<CompoundObject> orphaned;

    if (components_.empty()) {
        return orphaned;
    }

    auto build_group = [&](const std::vector<std::size_t>& group, const std::string& suffix) {
        CompoundObject container(label_ + suffix);
        for (std::size_t original_index : group) {
            auto copy = std::make_shared<Component>(*components_[original_index]);
            copy->anchored = false;
            container.AddComponent(copy);
        }

        for (const auto& joint : joints_) {
            if (joint.broken) {
                continue;
            }

            const auto first_it = std::find(group.begin(), group.end(), joint.first);
            const auto second_it = std::find(group.begin(), group.end(), joint.second);
            if (first_it != group.end() && second_it != group.end()) {
                const std::size_t first = static_cast<std::size_t>(std::distance(group.begin(), first_it));
                const std::size_t second = static_cast<std::size_t>(std::distance(group.begin(), second_it));
                container.Connect(first, second, joint.max_shear_force);
            }
        }

        return container;
    };

    std::vector<bool> visited(components_.size(), false);

    auto bfs_group = [&](std::size_t seed) {
        std::vector<std::size_t> group;
        std::queue<std::size_t> pending;
        pending.push(seed);
        visited[seed] = true;

        while (!pending.empty()) {
            const std::size_t current = pending.front();
            pending.pop();
            group.push_back(current);

            for (const auto& joint : joints_) {
                if (joint.broken) {
                    continue;
                }

                std::size_t next = static_cast<std::size_t>(-1);
                if (joint.first == current) {
                    next = joint.second;
                } else if (joint.second == current) {
                    next = joint.first;
                }

                if (next < components_.size() && !visited[next]) {
                    visited[next] = true;
                    pending.push(next);
                }
            }
        }

        return group;
    };

    const std::vector<std::size_t> root_group = bfs_group(0U);
    std::vector<std::shared_ptr<Component>> retained_components;
    retained_components.reserve(root_group.size());
    for (std::size_t index : root_group) {
        retained_components.push_back(components_[index]);
    }

    if (root_group.size() != components_.size()) {
        for (std::size_t index = 0; index < components_.size(); ++index) {
            if (!visited[index]) {
                const std::vector<std::size_t> group = bfs_group(index);
                if (!group.empty()) {
                    orphaned.push_back(build_group(group, "_orphan"));
                }
            }
        }
    }

    CompoundObject retained = build_group(root_group, "");
    retained.components_ = std::move(retained_components);
    components_ = std::move(retained.components_);
    joints_ = std::move(retained.joints_);
    if (!orphaned.empty()) {
        orphan_buffer_.insert(orphan_buffer_.end(), std::make_move_iterator(orphaned.begin()), std::make_move_iterator(orphaned.end()));
    }
    dragged_component_ = static_cast<std::size_t>(-1);
    return orphaned;
}

std::vector<CompoundObject> CompoundObject::ConsumeOrphanedObjects() {
    std::vector<CompoundObject> orphaned;
    orphaned.swap(orphan_buffer_);
    return orphaned;
}

void CompoundObject::ApplyStrain(float delta) {
    structural_strain_ = std::max(0.0f, structural_strain_ + delta);
}

std::size_t CompoundObject::Size() const {
    return components_.size();
}

std::size_t CompoundObject::JointCount() const {
    return joints_.size();
}

std::size_t CompoundObject::BrokenJointCount() const {
    return static_cast<std::size_t>(std::count_if(joints_.begin(), joints_.end(), [](const JointLink& joint) {
        return joint.broken;
    }));
}

float CompoundObject::StructuralStrain() const {
    return structural_strain_;
}

bool CompoundObject::IsSplit() const {
    return BrokenJointCount() > 0U || structural_strain_ > 1.2f;
}

const std::vector<std::shared_ptr<Component>>& CompoundObject::Components() const {
    return components_;
}

const std::vector<JointLink>& CompoundObject::Joints() const {
    return joints_;
}

const std::string& CompoundObject::Label() const {
    return label_;
}

std::size_t CompoundObject::ActiveComponentIndex() const {
    return dragged_component_ < components_.size() ? dragged_component_ : 0U;
}

float CompoundObject::ActiveJointStress() const {
    if (joints_.empty()) {
        return 0.0f;
    }

    const std::size_t index = std::min(active_joint_index_, joints_.size() - 1U);
    return joints_[index].stress_load;
}

float CompoundObject::ActiveJointMaxShear() const {
    if (joints_.empty()) {
        return 0.0f;
    }

    const std::size_t index = std::min(active_joint_index_, joints_.size() - 1U);
    return joints_[index].max_shear_force;
}

bool CompoundObject::ActiveJointIsInterlocking() const {
    if (joints_.empty()) {
        return false;
    }

    const std::size_t index = std::min(active_joint_index_, joints_.size() - 1U);
    return joints_[index].isInterlocking;
}

std::string CompoundObject::ActiveJointTypeLabel() const {
    if (joints_.empty()) {
        return "UNASSEMBLED";
    }

    return ActiveJointIsInterlocking() ? "SOCKET" : "WRAP";
}

bool CompoundObject::HasJoints() const {
    return !joints_.empty();
}

void CompoundObject::SetWorkbenchActive(bool value) {
    workbench_active_ = value;
}

bool CompoundObject::WorkbenchActive() const {
    return workbench_active_;
}

void CompoundObject::SetSwinging(bool value) {
    swinging_ = value;
}

bool CompoundObject::Swinging() const {
    return swinging_;
}

void CompoundObject::SetActiveResourceTarget(std::string name, float hardness, float durability) {
    active_resource_name_ = std::move(name);
    active_resource_hardness_ = hardness;
    active_resource_durability_ = durability;
}

void CompoundObject::ClearActiveResourceTarget() {
    active_resource_name_.clear();
    active_resource_hardness_ = 0.0f;
    active_resource_durability_ = 0.0f;
}

bool CompoundObject::HasActiveResourceTarget() const {
    return !active_resource_name_.empty();
}

const std::string& CompoundObject::ActiveResourceName() const {
    return active_resource_name_;
}

float CompoundObject::ActiveResourceHardness() const {
    return active_resource_hardness_;
}

float CompoundObject::ActiveResourceDurability() const {
    return active_resource_durability_;
}

void CompoundObject::AddFeedbackStress(float magnitude) {
    structural_strain_ += magnitude * 0.0005f;
    for (auto& joint : joints_) {
        if (!joint.broken) {
            joint.stress_load += magnitude * 0.5f;
            last_joint_stress_ = joint.stress_load;
        }
    }
}

void CompoundObject::MarkJointInterlocking(std::size_t joint_index, bool value) {
    if (joint_index >= joints_.size()) {
        return;
    }

    joints_[joint_index].isInterlocking = value;
    if (value) {
        joints_[joint_index].max_shear_force = std::max(10.0f, joints_[joint_index].max_shear_force * 5.0f);
    }
}

bool CompoundObject::HasOrphanedObjects() const {
    return !orphan_buffer_.empty();
}

float CompoundObject::LastImpactForce() const {
    return last_impact_force_;
}

float CompoundObject::LastJointStress() const {
    return last_joint_stress_;
}

void ProcessCollisionImpact(CompoundObject* tool, float normalImpulse) {
    if (!tool || tool->joints_.empty()) {
        return;
    }

    const float impact = std::max(0.0f, normalImpulse);
    tool->last_impact_force_ = impact;

    bool joint_broken = false;
    for (int index = static_cast<int>(tool->joints_.size()) - 1; index >= 0; --index) {
        auto& joint = tool->joints_[static_cast<std::size_t>(index)];
        if (joint.broken) {
            continue;
        }

        tool->active_joint_index_ = static_cast<std::size_t>(index);
        const float jointStress = impact * (joint.isInterlocking ? 0.25f : 1.30f);
        joint.stress_load += jointStress;
        tool->last_joint_stress_ = joint.stress_load;

        if (jointStress > joint.max_shear_force) {
            joint.broken = true;
            tool->structural_strain_ += 0.5f;
            joint_broken = true;
            break;
        }
    }

    if (joint_broken) {
        (void)tool->ValidateStructuralIntegrity();
    }
}

}  // namespace genesis
