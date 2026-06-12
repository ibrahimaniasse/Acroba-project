# Design Decisions

This document outlines the rationale behind key architectural choices in the Acroba framework.

## 1. Manager-Plugin Pattern vs Behavior Trees (BT.CPP)

**Decision:** Build a custom Manager-Plugin orchestrator instead of using standard Behavior Trees (like `behaviortree.cpp` + Navigation2).

**Rationale:**
- **Simplicity for definition:** BTs require complex XML definitions for even simple sequences. Our approach allows a planner to trigger single behaviors sequentially via standard ROS Actions.
- **State sharing:** The `BehaviorContext` allows sharing state between ticks seamlessly without complex blackboard mapping.
- **Scope:** This framework focuses on providing robust *primitives* (the behaviors themselves). A BT or state machine could easily be layered *on top* of the Behavior Manager to sequence these primitives.

## 2. The Python / C++ Split

**Decision:** Implement high-level, sequential logic in Python, and low-level, reactive logic in C++.

**Rationale:**
- **Python (Sequential/Stateful):** Behaviors like `search_pattern` or `patrol_loop` involve generating waypoints, maintaining indices, and managing multi-step state machines. Python is excellent for rapid development of this logic, and since navigation updates at ~10-20Hz, Python's performance is not a bottleneck. Furthermore, pure Python logic can be unit-tested instantly without compilation.
- **C++ (Reactive/Safety):** Behaviors like `avoid_obstacle` or `emergency_stop` require processing dense arrays (e.g., 360+ LiDAR points) and reacting in milliseconds to prevent physical damage. C++ provides the deterministic memory management and raw execution speed necessary for these safety-critical loops.

## 3. Action-based Interfaces

**Decision:** Use ROS 2 Actions for triggering behaviors instead of Services or custom Topics.

**Rationale:**
- Behaviors are, by definition, long-running tasks.
- Actions provide built-in semantics for:
  - **Goal acceptance/rejection** (e.g., rejecting an unknown behavior name).
  - **Continuous feedback** (e.g., progress percentage, distance to target).
  - **Preemption/Cancellation** (critical for interrupting a patrol to execute a docking sequence).
- Services block the caller, and custom topics require reinventing state-tracking logic.

## 4. Parameter Passing via String Arrays

**Decision:** In the `ExecuteBehavior` action, parameters are passed as `string[] names` and `string[] values`.

**Rationale:**
- Normally, ROS 2 parameters are set on the Node. However, Python behaviors are *plugins*, not nodes. 
- We want to be able to override a behavior's default parameters *per execution* (e.g., executing `move_to_waypoint` with speed 0.5, and later with speed 1.0) without mutating global node parameters.
- Passing them as generic string arrays allows the interface to remain fixed while supporting arbitrary parameters for new behaviors. The behaviors cast the strings to `float`, `int`, etc., internally.
