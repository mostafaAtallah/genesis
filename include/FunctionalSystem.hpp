#pragma once

#include <string>

#include "CompoundObject.hpp"

namespace genesis {

struct DeductionResult {
    bool can_trigger_action = false;
    float lever_advantage = 0.0f;
    float effective_hardness = 0.0f;
    std::string action;
};

class FunctionalSystem {
public:
    DeductionResult Analyze(const CompoundObject& object) const;
};

}  // namespace genesis
