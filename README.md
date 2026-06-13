# Unified Engineering Blueprint: Dynamic Physics & Emergent Crafting Engine

This document serves as the master specification for implementing a boundary-free sandbox survival game in **modern C++ (C++17)** using **Raylib** for graphics/windowing and **Box2D** for rigid-body physics. 

## 1. Project Philosophy & Core Rules
1. **Zero Predefined Item IDs:** Items do not possess static identifiers like `item_axe` or `item_sword`. Object capability is *derived dynamically* from its physical structure, material properties, and geometry layout.
2. **Structural Realism:** All assemblies rely on real-time physics engine constraints (`b2WeldJoint`). Excessive recoil forces, high torque, or hard impacts will dynamically shear these bonds, shattering custom tools back into raw fragments.
3. **Emergent Engineering:** If a player wedges a tapered rod into a pierced stone block, the physical topology restricts motion—creating a socketed tool through mechanical interlocking rather than a recipe toggle.

---

## 2. Directory Architecture Layout

The codebase must follow this strict, modular topology:
```text
project_root/
├── CMakeLists.txt             # Project orchestrator
├── include/
│   ├── Component.hpp          # Base entity definition & material vector metadata
│   ├── CompoundObject.hpp     # Physics graph tracking interconnected components
│   └── FunctionalSystem.hpp   # Engine rules parsing topology to utility actions
└── src/
    ├── CompoundObject.cpp     # Joint tracking, structural strain, and graph splitting
    ├── FunctionalSystem.cpp   # Lever advantages, CoM math, and capability deduction
    └── main.cpp               # Raylib initialization, camera tracking, and the Box2D loop