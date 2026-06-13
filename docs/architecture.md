# Architecture

This project is organized around a physical deduction pipeline:

1. `Component` stores a part, its material vector, and geometry metadata.
2. `CompoundObject` tracks connected components, joints, and structural strain.
3. `FunctionalSystem` inspects topology and produces an action/failure outcome.
4. `main.cpp` demonstrates the current simulation scaffold and will later host the Raylib and Box2D loop.

The README describes the intended runtime behavior: structural interaction is
emergent, not recipe-driven, so the code should keep topology and material
properties at the center of every deduction.
