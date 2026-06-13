#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"
#include "box2d/box2d.h"
#include "box2d/collision.h"
#include "Tools.hpp"
#include "CompoundObject.hpp"
#include "FunctionalSystem.hpp"
#include "ResourceNode.hpp"
#include "Constants.hpp"
//constants header
int main() {
    using namespace genesis;

    SimulationState sim;
    BuildSimulation(sim);

    InitWindow(kScreenWidth, kScreenHeight, "Genesis");
    SetTargetFPS(60);

    Camera3D camera = { 0 };
    camera.position = Vector3{10.0f, 8.6f, 20.0f};
    camera.target = Vector3{10.0f, 2.8f, 0.0f};
    camera.up = Vector3{0.0f, 1.0f, 0.0f};
    camera.fovy = 38.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    while (!WindowShouldClose()) {
        const float dt = 1.0f / 60.0f;
        sim.simulationTime += dt;
        const Vector2 mouseScreen = Vector2{static_cast<float>(GetMouseX()), static_cast<float>(GetMouseY())};
        const b2Vec2 mouseWorld = ToWorld(mouseScreen, camera);
        const Rectangle restartButton{760.0f, 666.0f, 190.0f, 38.0f};
        const bool restartHovered = CheckCollisionPointRec(mouseScreen, restartButton);
        bool restartRequested = restartHovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);

        camera.position.z -= GetMouseWheelMove() * 0.8f;
        camera.position.x += (IsKeyDown(KEY_D) ? 0.04f : 0.0f) - (IsKeyDown(KEY_A) ? 0.04f : 0.0f);
        camera.position.y += (IsKeyDown(KEY_E) ? 0.04f : 0.0f) - (IsKeyDown(KEY_Q) ? 0.04f : 0.0f);

        if (IsKeyPressed(KEY_SPACE)) {
            sim.swinging = !sim.swinging;
        }

        if (IsKeyPressed(KEY_ENTER)) {
            sim.workbenchActive = !sim.workbenchActive;
            if (!sim.workbenchActive) {
                ClearWorkbenchSelection(sim);
            } else if (sim.workbenchSelection < 0 && !sim.parts.empty()) {
                SelectWorkbenchPart(sim, 0);
            }
        }

        if (restartRequested) {
            DestroySimulation(sim);
            BuildSimulation(sim);
        } else if (!sim.workbenchActive && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            int hitIndex = FindWorkbenchPartAtMouse(sim, mouseWorld);
            for (int i = static_cast<int>(sim.parts.size()) - 1; i >= 0; --i) {
                if (i == hitIndex) {
                    sim.dragging = true;
                    sim.grabbedPart = i;

                    b2BodyDef cursorDef = b2DefaultBodyDef();
                    cursorDef.type = b2_kinematicBody;
                    cursorDef.position = mouseWorld;
                    sim.cursorBodyId = b2CreateBody(sim.worldId, &cursorDef);
                    b2Body_SetUserData(sim.cursorBodyId, &sim.cursorTag);

                    b2WeldJointDef dragDef = b2DefaultWeldJointDef();
                    dragDef.base.bodyIdA = sim.cursorBodyId;
                    dragDef.base.bodyIdB = sim.parts[static_cast<std::size_t>(i)].bodyId;
                    dragDef.base.localFrameA.p = b2Vec2{0.0f, 0.0f};
                    dragDef.base.localFrameB.p = b2Body_GetLocalPoint(sim.parts[static_cast<std::size_t>(i)].bodyId, mouseWorld);
                    dragDef.linearHertz = 20.0f;
                    dragDef.linearDampingRatio = 0.85f;
                    dragDef.angularHertz = 20.0f;
                    dragDef.angularDampingRatio = 0.85f;
                    sim.dragJointId = b2CreateWeldJoint(sim.worldId, &dragDef);
                    break;
                }
            }
        }

        if (sim.workbenchActive) {
            if (IsKeyPressed(KEY_TAB)) {
                const int nextIndex = sim.parts.empty() ? -1 : ((sim.workbenchSelection + 1) % static_cast<int>(sim.parts.size()));
                SelectWorkbenchPart(sim, nextIndex);
            }

            if (IsKeyPressed(KEY_C)) {
                CutSelectedWorkbenchPart(sim, Vector2{mouseWorld.x, mouseWorld.y});
            }

            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                const int hitIndex = FindWorkbenchPartAtMouse(sim, mouseWorld);
                if (hitIndex >= 0) {
                    SelectWorkbenchPart(sim, hitIndex);
                } else if (!sim.parts.empty()) {
                    const int nextIndex = sim.workbenchSelection < 0 ? 0 : ((sim.workbenchSelection + 1) % static_cast<int>(sim.parts.size()));
                    SelectWorkbenchPart(sim, nextIndex);
                }
            }

            MoveSelectedWorkbenchPart(sim, mouseWorld, IsMouseButtonDown(MOUSE_BUTTON_LEFT), dt);
        }

        if (sim.dragging && b2Body_IsValid(sim.cursorBodyId)) {
            b2Transform target = {mouseWorld, b2Rot_identity};
            b2Body_SetTargetTransform(sim.cursorBodyId, target, dt, true);

            if (sim.swinging && sim.grabbedPart >= 0 && sim.grabbedPart < static_cast<int>(sim.parts.size())) {
                const float swingVelocity = std::sin(sim.simulationTime * 10.0f) * 18.0f;
                b2Body_SetAngularVelocity(sim.parts[static_cast<std::size_t>(sim.grabbedPart)].bodyId, swingVelocity);
            }
        }

        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            DestroyDragJoint(sim);
        }

        sim.lastImpactForce = 0.0f;
        sim.lastJointStress = 0.0f;
        sim.activeResourceName.clear();
        sim.activeResourceHardness = 0.0f;
        sim.activeResourceDurability = 0.0f;

        b2World_Step(sim.worldId, dt, 4);

        std::vector<b2BodyId> bodiesToDestroy;

        const b2ContactEvents contactEvents = b2World_GetContactEvents(sim.worldId);
        for (int i = 0; i < contactEvents.hitCount; ++i) {
            const b2ContactHitEvent& hit = contactEvents.hitEvents[i];
            if (!b2Contact_IsValid(hit.contactId)) {
                continue;
            }

            const b2ContactData contactData = b2Contact_GetData(hit.contactId);
            float normalImpulse = 0.0f;
            for (int pointIndex = 0; pointIndex < contactData.manifold.pointCount; ++pointIndex) {
                normalImpulse += contactData.manifold.points[pointIndex].totalNormalImpulse;
            }
            const float impactMagnitude = std::max(normalImpulse, hit.approachSpeed * 160.0f);

            const b2BodyId bodyA = b2Shape_GetBody(hit.shapeIdA);
            const b2BodyId bodyB = b2Shape_GetBody(hit.shapeIdB);
            auto* tagA = static_cast<BodyTag*>(b2Body_GetUserData(bodyA));
            auto* tagB = static_cast<BodyTag*>(b2Body_GetUserData(bodyB));

            auto handleResourceImpact = [&](BodyTag* toolTag, BodyTag* resourceTag, float impulse) {
                if (!toolTag || !resourceTag || toolTag->kind != BodyTag::Kind::ToolPart || resourceTag->kind != BodyTag::Kind::Resource) {
                    return;
                }

                auto& toolPart = sim.parts[toolTag->index];
                auto& resource = sim.resources[resourceTag->index];
                if (resource.node.IsHarvested()) {
                    return;
                }

                sim.activeResourceName = resource.node.Name();
                sim.activeResourceHardness = resource.node.Hardness();
                sim.activeResourceDurability = resource.node.Durability();

                const bool toolHardEnough = toolPart.component->material.hardness >= resource.node.Hardness();
                if (toolHardEnough) {
                    const float damage = std::max(0.0f, impulse * 0.12f);
                    const float remaining = resource.node.ApplyDamage(damage);
                    if (remaining <= 0.0f) {
                        std::printf("[SYSTEM] Resource Harvested!\n");
                        sim.lastDroppedItem = resource.node.Name() + " drop";
                        resource.node.MarkHarvested();
                        bodiesToDestroy.push_back(resource.bodyId);
                    }
                    sim.lastImpactForce = impulse;
                } else {
                    sim.lastImpactForce = impulse;
                    sim.toolModel.ApplyStrain(impulse * 0.004f);
                    ApplyImpactToTool(sim, toolTag->index, impulse);
                }
            };

            if (tagA && tagB) {
                if (tagA->kind == BodyTag::Kind::ToolPart && tagB->kind == BodyTag::Kind::Resource) {
                    handleResourceImpact(tagA, tagB, impactMagnitude);
        } else if (tagB->kind == BodyTag::Kind::ToolPart && tagA->kind == BodyTag::Kind::Resource) {
                    handleResourceImpact(tagB, tagA, impactMagnitude);
                } else if ((tagA->kind == BodyTag::Kind::ToolPart && tagB->kind == BodyTag::Kind::Floor) ||
                           (tagB->kind == BodyTag::Kind::ToolPart && tagA->kind == BodyTag::Kind::Floor)) {
                    const std::size_t toolIndex = tagA->kind == BodyTag::Kind::ToolPart ? tagA->index : tagB->index;
                    ApplyImpactToTool(sim, toolIndex, impactMagnitude);
                }
            }

            if (tagA && tagA->kind == BodyTag::Kind::ToolPart) {
                genesis::ProcessCollisionImpact(&sim.toolModel, impactMagnitude);
            } else if (tagB && tagB->kind == BodyTag::Kind::ToolPart) {
                genesis::ProcessCollisionImpact(&sim.toolModel, impactMagnitude);
            }
        }

        for (b2BodyId bodyId : bodiesToDestroy) {
            if (b2Body_IsValid(bodyId)) {
                b2DestroyBody(bodyId);
            }
        }

        for (auto& resource : sim.resources) {
            if (b2Body_IsValid(resource.bodyId)) {
                resource.node.SetPhysicsBody(resource.bodyId);
            } else {
                resource.node.MarkHarvested();
            }
        }

        for (auto& part : sim.parts) {
            if (b2Body_IsValid(part.bodyId)) {
                SyncComponentVisual(*part.component, part.bodyId);
            }
        }

        const int brokenJoints = static_cast<int>(std::count_if(sim.joints.begin(), sim.joints.end(), [](const auto& joint) {
            return joint.broken;
        }));
        const float leverAdvantage = static_cast<float>(sim.parts.size()) / (static_cast<float>(sim.joints.size()) + 1.0f);
        const float effectiveHardness = 1.0f + static_cast<float>(sim.joints.size() - brokenJoints) - sim.toolModel.StructuralStrain();
        const bool canTriggerAction = sim.toolModel.HasJoints() && effectiveHardness > 0.5f && brokenJoints == 0;

        BeginDrawing();
        ClearBackground(Color{18, 20, 28, 255});

        DrawText("Genesis", 32, 28, 32, RAYWHITE);
        DrawText("Space toggles swing, Enter toggles workbench mode, Tab/click selects, mouse drag moves the piece, move the mouse then press C to cut.", 32, 66, 18, Color{180, 190, 210, 255});

        BeginMode3D(camera);
        DrawPlane(Vector3{10.0f, 0.0f, 0.0f}, Vector2{22.0f, 12.0f}, Color{88, 104, 90, 255});
        DrawCubeV(Vector3{10.0f, -0.25f, 0.0f}, Vector3{20.5f, 0.5f, 8.0f}, Color{76, 84, 96, 255});
        DrawCubeWiresV(Vector3{10.0f, -0.25f, 0.0f}, Vector3{20.5f, 0.5f, 8.0f}, Color{100, 112, 128, 255});
        DrawCubeV(Vector3{-0.25f, 6.0f, 0.0f}, Vector3{0.5f, 12.0f, 8.0f}, Color{52, 58, 72, 255});
        DrawCubeV(Vector3{20.25f, 6.0f, 0.0f}, Vector3{0.5f, 12.0f, 8.0f}, Color{52, 58, 72, 255});
        DrawSphere(Vector3{16.0f, 10.0f, -6.0f}, 0.9f, Color{255, 235, 180, 255});
        DrawSphereWires(Vector3{16.0f, 10.0f, -6.0f}, 0.9f, 8, 8, Color{255, 248, 220, 100});

        for (const auto& resource : sim.resources) {
            DrawResourceNode(resource, camera);
        }

        for (std::size_t i = 0; i < sim.parts.size(); ++i) {
            const auto& part = sim.parts[i];
            const bool selected = sim.dragging && sim.grabbedPart >= 0 && i == static_cast<std::size_t>(sim.grabbedPart);
            const bool workbenchSelected = sim.workbenchActive && sim.workbenchSelection == static_cast<int>(i);
            DrawToolPart(part, selected || workbenchSelected, camera);
        }

        for (const auto& joint : sim.joints) {
            if (joint.broken || !b2Joint_IsValid(joint.jointId)) {
                continue;
            }

            const Vector3 start = ToRender3D(b2Body_GetPosition(sim.parts[joint.first].bodyId), 0.2f);
            const Vector3 end = ToRender3D(b2Body_GetPosition(sim.parts[joint.second].bodyId), 0.2f);
            DrawLine3D(start, end, Color{210, 220, 235, 255});
        }

        if (b2Body_IsValid(sim.cursorBodyId)) {
            const Vector3 cursorPos = ToRender3D(b2Body_GetPosition(sim.cursorBodyId), 0.4f);
            DrawCubeWiresV(cursorPos, Vector3{0.35f, 0.35f, 0.35f}, Color{255, 214, 102, 255});
        }

        EndMode3D();

        DrawText(TextFormat("Components: %i", static_cast<int>(sim.parts.size())), 760, 170, 22, RAYWHITE);
        DrawText(TextFormat("Joints: %i", static_cast<int>(sim.joints.size())), 760, 204, 22, RAYWHITE);
        DrawText(TextFormat("Broken joints: %i", brokenJoints), 760, 238, 22, RAYWHITE);
        DrawText(TextFormat("Workbench: %s", sim.workbenchActive ? "ON" : "OFF"), 760, 272, 22, sim.workbenchActive ? Color{110, 220, 150, 255} : Color{240, 190, 100, 255});
        DrawText(TextFormat("Swinging: %s", sim.swinging ? "YES" : "NO"), 760, 306, 22, sim.swinging ? Color{110, 220, 150, 255} : Color{240, 190, 100, 255});
        DrawText(TextFormat("Lever advantage: %.2f", leverAdvantage), 760, 340, 22, RAYWHITE);
        DrawText(TextFormat("Effective hardness: %.2f", effectiveHardness), 760, 374, 22, RAYWHITE);
        DrawText(TextFormat("Outcome: %s", canTriggerAction ? "tool_action" : "structural_failure"), 760, 408, 22, canTriggerAction ? Color{110, 220, 150, 255} : Color{240, 110, 110, 255});

        DrawText(TextFormat("Active Resource: %s", sim.activeResourceName.empty() ? "None" : sim.activeResourceName.c_str()), 760, 462, 18, Color{180, 190, 210, 255});
        DrawText(TextFormat("Resource Hardness: %.2f", sim.activeResourceHardness), 760, 490, 18, Color{180, 190, 210, 255});
        DrawText(TextFormat("Resource Durability: %.1f", sim.activeResourceDurability), 760, 518, 18, Color{180, 190, 210, 255});
        DrawText(TextFormat("Active Joint Connection Type: %s", sim.toolModel.ActiveJointTypeLabel().c_str()), 760, 556, 18, Color{180, 190, 210, 255});
        DrawText(TextFormat("Current Contact Force: %.1f N", sim.toolModel.LastImpactForce()), 760, 584, 18, Color{180, 190, 210, 255});
        DrawText(TextFormat("Joint Stress Capacity: %.1f / %.1f N", sim.toolModel.ActiveJointStress(), sim.toolModel.ActiveJointMaxShear()), 760, 612, 18, Color{180, 190, 210, 255});
        const char* workbenchSelectionName = "None";
        if (sim.workbenchSelection >= 0 && sim.workbenchSelection < static_cast<int>(sim.parts.size())) {
            workbenchSelectionName = sim.parts[static_cast<std::size_t>(sim.workbenchSelection)].component->name.c_str();
        }
        DrawText(TextFormat("Workbench Selection: %s", workbenchSelectionName), 760, 640, 18, Color{170, 180, 200, 255});
        DrawText("Tip: click or Tab to select, move the mouse to aim the cut, then press C.", 760, 666, 18, Color{170, 180, 200, 255});

        DrawRectangleRounded(restartButton, 0.2f, 8, restartHovered ? Color{86, 102, 140, 255} : Color{52, 60, 82, 255});
        DrawRectangleRoundedLinesEx(restartButton, 0.2f, 8, 2.0f, Color{200, 210, 230, 255});
        DrawText("Start Again", static_cast<int>(restartButton.x + 34.0f), static_cast<int>(restartButton.y + 10.0f), 18, RAYWHITE);

        if (!sim.lastDroppedItem.empty()) {
            DrawText(sim.lastDroppedItem.c_str(), 760, 668, 18, Color{240, 220, 120, 255});
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
