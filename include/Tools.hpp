#pragma once

#include <string>

#include "box2d/box2d.h"
#include "raylib.h"
#include "raymath.h"
#include "memory"
#include "Component.hpp"
#include "ResourceNode.hpp"

namespace genesis{
    struct BodyTag {
    enum class Kind {
        ToolPart,
        Resource,
        Floor,
        Cursor
    };

    Kind kind = Kind::Floor;
    std::size_t index = 0;
};
struct ToolPartRuntime {
    std::shared_ptr<genesis::Component> component;
    b2BodyId bodyId = b2_nullBodyId;
    b2ShapeId shapeId = b2_nullShapeId;
    BodyTag tag{};
    float displayDepth = 0.0f;
    Mesh mesh{0};
    bool ownsMesh = false;
};

struct ToolLinkRuntime {
    b2JointId jointId = b2_nullJointId;
    std::size_t first = 0;
    std::size_t second = 0;
    float maxShearForce = 1200.0f;
    bool broken = false;
    bool interlocking = false;
};

struct ResourceRuntime {
    genesis::ResourceNode node;
    b2BodyId bodyId = b2_nullBodyId;
    b2ShapeId shapeId = b2_nullShapeId;
    BodyTag tag{};
    bool pendingDestroy = false;
    float displayDepth = 0.0f;
};

}