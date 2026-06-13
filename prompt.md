# Stage 1

The current Raylib simulation window is executing correctly and displaying the static object graph metrics, but it is completely non-interactive. Update the codebase (`src/main.cpp` and `src/CompoundObject.cpp`) to implement real-time player mouse interactions and dynamic stress testing using Raylib and Box2D.

Implement the following features exactly:

1. MOUSE DRAGGING INTERACTION (b2MouseJoint):
   - When the player left-clicks and holds over a component fixture, create a temporary `b2MouseJoint` bound to the current mouse position coordinates.
   - On mouse release, destroy the joint. This allows the player to grab, fling, and violently shake the compound object around the screen using natural screen physics.

2. IMPACT INTERACTION TESTING (The Stress Test):
   - Add a static, solid rectangular wall or floor entity into the Box2D world space (e.g., positioned at the bottom of the viewport window).
   - Enable your ContactListener to listen for collision contact events between the compound object parts and this static barrier.

3. DYNAMIC DISASSEMBLY (Joint Shearing):
   - When the user flings the compound object into the boundary wall, read the normal impulse force vector from the contact solver data.
   - Pass this force vector into `ProcessCollisionImpact()` to evaluate stress thresholds. If the impulse force exceeds `maxShearForce` for the joint, destroy the `b2WeldJoint` immediately so that the parts break apart dynamically in real-time.

Keep the headless metric log calculations running on screen in text format while updating the coordinates dynamically every frame step. Retain the modular class architecture layout.


# Stage 2
Please implement Stage 2 of our dynamic crafting survival engine. Maintain the current Raylib rendering loop, Box2D world configuration, mouse-dragging mechanics, and Spacebar/Enter workbench state tracking. Implement the following game logic classes and algorithms:

1. RESOURCE NODES (ResourceNode.hpp / .cpp):
   - Create a `ResourceNode` class containing fields for `name`, `hardness` (0.0f to 1.0f), `durability` (float), and a pointer to a static Box2D body (`b2Body*`).
   - Spawn two static instances directly in the game world:
     * A "Pine Tree" (Hardness: 0.35, Durability: 100.0f) colored Green in Raylib.
     * A "Granite Boulder" (Hardness: 0.75, Durability: 250.0f) colored Light Gray in Raylib.

2. SYSTEMIC HARVESTING CONTACT LISTENER:
   - Update your Box2D ContactListener setup. When a component of an actively swung CompoundObject collides with a ResourceNode, extract the contact normal impulse force.
   - Run a material evaluation check: Compare the hitting component's `material.hardness` against the target `ResourceNode::hardness`.
   - If Tool Hardness >= Resource Hardness: Subtract durability points from the node proportional to the normal impact force. If durability reaches <= 0, print "[SYSTEM] Resource Harvested!" to the terminal, trigger an item drop, and safely destroy the node's static physics body.
   - If Tool Hardness < Resource Hardness: Refuse to damage the node. Transfer 100% of the impact force directly back into the tool's joint network as feedback stress.

3. GRAPH ISOLATION & SUB-GRAPH PARTITIONING:
   - Update `CompoundObject::ValidateStructuralIntegrity()`. If a joint's stress threshold is exceeded and `world->DestroyJoint()` is executed, run a connectivity graph traversal (BFS or DFS) starting from the primary grip node.
   - Isolate any unvisited component sub-graphs that are no longer physically linked to the handle.
   - Instatiate a new, independent `CompoundObject` container for those orphaned parts, allowing them to drop and behave as standalone physical clutter in the environment space.

Ensure all real-time text telemetry logs on the right side of the screen update to show active Resource Node stats if the player is hitting one.



# Stage 3
1. DYNAMIC TOPOLOGY DETECTION:
   - Enhance the assembly tracking logic inside `src/CompoundObject.cpp`. When the user confirms an assembly configuration, analyze the overlapping fixtures.
   - If a PRIMITIVE_SOLID component penetrates completely inside the bounding boundaries of a PIERCED_HOLLOW component, explicitly set `StructuralJoint::isInterlocking = true`. Otherwise, default to false (Friction Surface Wrap).
   - Interlocking joints must automatically scale up their `maxShearForce` value by a factor of 5x to simulate the structural advantage of mechanical pinning.

2. SHOCKWAVE STRESS PROPAGATION:
   - Implement `void ProcessCollisionImpact(CompoundObject* tool, float normalImpulse)` in `src/CompoundObject.cpp`.
   - Iterate backward through the tool's joint vector from the contact point down to the grip node. For each joint, compute the local stress tensor: `float jointStress = normalImpulse * (joint.isInterlocking ? 0.25f : 1.30f);`.
   - If `jointStress > joint.maxShearForce`, call the Box2D solver to instantly destroy that joint constraint, then invoke our Stage 2 sub-graph partitioning routine to split the orphaned components into standalone physics bodies.

3. SCREEN TELEMETRY:
   - Update the Raylib `DrawText()` rendering loop block in `src/main.cpp` to output live structural metrics on the translucent sidebar: Active Joint Connection Type (SOCKET vs WRAP), Current Contact Force (Newtons), and Joint Stress Capacity ratios.

Provide only the clean, complete C++ source code updates for our source and include files without extra conversational commentary."



# additional points :
1. DYNAMIC MESH SLICING LOGIC:
   - Implement a function `void SliceComponentMesh(Raylib::Mesh& mesh, Vector3 planePoint, Vector3 planeNormal, Raylib::Mesh& outMeshA, Raylib::Mesh& outMeshB)`.
   - This function must iterate through the triangles of the active 3D model, evaluate which side of the cutting plane they reside on, and separate them into two independent vertex arrays.
   - Caps or seals the newly created exposed inner faces of both meshes so they remain solid, watertight 3D volumes.

2. PROCEDURAL BULLET PHYSICS REGENERATION:
   - When a component is sliced or chopped in WORKBENCH mode, safely remove its old `btRigidBody` and `btBoxShape` from the Bullet dynamics world.
   - For both newly generated sub-meshes, extract their raw vertex arrays and instantiate two new dynamic `btConvexHullShape` physics bodies.
   - Calculate their new mass and center of gravity based on their reduced volumes, then re-insert them into the active physics loop so they can tumble independently.

3. WORKBENCH CUTTING INTERACTION:
   - Update the WORKBENCH keyboard logic. When a user selects a piece and presses 'C' (Chop/Cut), project a 3D cutting plane line based on the camera view axis.
   - Slice the selected component right through that plane vector.
   - If the player cuts a piece thin enough, dynamically update its system attributes (e.g., if a brown block is carved into a long, thin cylinder, its material identity transitions from 'Log' to 'Handle Shaft').

Provide only the clean, refactored C++17 source code additions for this procedural sculpting engine architecture."