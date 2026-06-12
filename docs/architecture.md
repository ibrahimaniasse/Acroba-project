# Architecture Details

The Acroba framework uses a Manager-Plugin pattern, implemented via ROS 2 Action servers.

## Core Components

### 1. The Behavior Manager (`acroba_manager`)
The central orchestration node. It is responsible for:
- **Registry Loading**: Reading `behaviors.yaml` and dynamically loading Python classes at startup.
- **Action Server**: Exposing the `/acroba/execute_behavior` action server.
- **Context Management**: Subscribing to `/odom` and `/scan`, and maintaining a shared `BehaviorContext` object.
- **Tick Loop**: Running a fixed-frequency timer (default 20Hz) that calls `on_tick()` on the currently active Python behavior.
- **Preemption**: Handling cancellation requests and safely transitioning the active behavior through its `on_exit()` method.

### 2. Python Plugins (`acroba_behaviors_py`)
These are pure-Python classes that inherit from `BaseBehavior`. 
- They do **not** inherit from `rclpy.Node`.
- They do **not** contain their own callback queues.
- They are entirely passive, driven by the Manager's tick loop.
- **Advantage**: They are extremely easy to unit-test without a running ROS ecosystem.

### 3. C++ Reactive Nodes (`acroba_behaviors_cpp`)
These are standalone C++ nodes (`rclcpp::Node`) that each host their own Action Server.
- They interface directly with sensors (e.g., subscribing to `/scan`).
- They publish directly to `/cmd_vel`.
- When the Manager needs to execute a C++ behavior, it acts as an Action *Client*, forwarding the goal to the C++ node.
- **Advantage**: They bypass the Python GIL and Manager tick-rate limits, allowing for high-frequency control loops (e.g., 50Hz+ for collision avoidance).

## Interfaces (`acroba_msgs`)

### `ExecuteBehavior.action`
The universal contract for triggering a behavior.
```yaml
# Goal
string behavior_name
string[] parameter_names
string[] parameter_values
---
# Result
bool success
string message
float64 execution_time
---
# Feedback
string current_status
float32 progress
string detail
```
*Note on parameters: We use string arrays for parameters to allow dynamic overrides without needing a strongly typed schema for every behavior.*

### `BehaviorStatus.msg`
Continuously published by the Manager (and C++ nodes) for external monitoring and UI integration.
```yaml
string behavior_name
uint8 status
string detail
builtin_interfaces/Time stamp
```

## Node Graph Overview

```mermaid
graph LR
    subgraph Sensors
        Odom[/odom]
        Scan[/scan]
    end
    
    subgraph Actuators
        CmdVel[/cmd_vel]
    end
    
    Odom --> Manager[Behavior Manager]
    Scan --> Manager
    Manager --> CmdVel
    
    Scan --> Avoid[Avoid Obstacle C++]
    Avoid --> CmdVel
    
    Scan --> Wall[Follow Wall C++]
    Wall --> CmdVel
    
    Client((External Client)) -- ExecuteBehavior --> Manager
```
