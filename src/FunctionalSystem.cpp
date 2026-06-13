#include "FunctionalSystem.hpp"

namespace genesis {

DeductionResult FunctionalSystem::Analyze(const CompoundObject& object) const {
    DeductionResult result;

    if (object.Size() == 0U) {
        result.action = "inactive";
        return result;
    }

    const float component_count = static_cast<float>(object.Size());
    const float joint_count = static_cast<float>(object.JointCount());
    const float broken_joint_count = static_cast<float>(object.BrokenJointCount());
    const float strain = object.StructuralStrain();

    if (!object.HasJoints()) {
        result.lever_advantage = component_count;
        result.effective_hardness = 0.0f;
        result.can_trigger_action = false;
        result.action = "unassembled";
        return result;
    }

    result.lever_advantage = component_count / (joint_count + 1.0f);
    result.effective_hardness = (1.0f + joint_count - broken_joint_count) - strain;
    result.can_trigger_action = result.effective_hardness > 0.5f && !object.IsSplit();
    result.action = result.can_trigger_action ? "tool_action" : "structural_failure";
    return result;
}

}  // namespace genesis
