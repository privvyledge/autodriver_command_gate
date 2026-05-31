# command_gate

A ROS 2 Humble safety gate package for F1/10 autonomous racing on Jetson Orin. It relays N command channels from input topics to output topics only when the gate is "open". Gate state is driven by a heartbeat watchdog and/or an explicit enable signal.

Both a C++ composable node and a functionally identical Python node are provided, sharing the same parameters, topics, and services.

## Gate Logic

```
gate_open = (heartbeat_ok OR !require_heartbeat) AND (enabled OR !require_enable)
```

When the gate closes, each channel publishes its buffered fallback message, then a zero-initialized message, then warns once if neither is available. The `warned_drop` flag resets when the gate reopens.

State transitions are logged at INFO level (e.g., `"Gate OPEN — heartbeat received"`, `"Gate CLOSED — heartbeat timeout"`).

## Dependencies

| Category | Packages |
|----------|----------|
| Build | `ament_cmake`, `ament_cmake_python`, `rclcpp`, `rclcpp_components` |
| Runtime | `rclpy`, `rosidl_runtime_py`, `std_msgs`, `std_srvs`, `geometry_msgs`, `ackermann_msgs`, `launch_ros` |
| Test | `ament_lint_auto`, `ament_lint_common`, `python3-pytest` |

## Build

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select command_gate

# Verify C++ component registration
ros2 component types   # should list: command_gate::CommandGateNode
```

## Configuration

Edit `config/command_gate.yaml` before launching or pass a custom config via the `config` launch argument.

### Global Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `num_channels` | int | `2` | Number of relay channels |
| `require_heartbeat` | bool | `true` | Gate closes on heartbeat timeout |
| `heartbeat_timeout` | float (s) | `1.0` | Seconds before heartbeat is considered stale |
| `heartbeat_topic` | string | `~/heartbeat` | Topic to watch for heartbeat pulses |
| `heartbeat_type` | string | `std_msgs/msg/Empty` | ROS 2 type of the heartbeat topic; any type works — only message arrival matters |
| `require_enable` | bool | `false` | Gate also requires explicit enable signal |
| `enable_topic` | string | `~/enable` | Topic for enable state (`std_msgs/Bool`) |

> The heartbeat check timer fires at `heartbeat_timeout / 4` for prompt timeout detection.

### Per-Channel Parameters (`channel_N`)

| Parameter | Required | Description |
|-----------|----------|-------------|
| `input_topic` | yes | Topic to subscribe to |
| `output_topic` | yes | Topic to publish to when gate is open |
| `message_type` | yes | ROS 2 type string, e.g. `ackermann_msgs/msg/AckermannDriveStamped` |
| `fallback_topic` | no | Topic whose last message is published when gate closes |

### Example Config

```yaml
command_gate:
  ros__parameters:
    num_channels: 2
    require_heartbeat: true
    heartbeat_timeout: 1.0
    require_enable: false

    channel_0:
      input_topic: steering_in
      output_topic: steering_out
      message_type: std_msgs/msg/Float64

    channel_1:
      input_topic: drive_in
      output_topic: drive_out
      message_type: ackermann_msgs/msg/AckermannDriveStamped
      fallback_topic: safe_drive_cmd
```

## ROS 2 Interface

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `~/heartbeat` | (configured via `heartbeat_type`) | Heartbeat pulses; any message arrival resets the watchdog |
| `~/enable` | `std_msgs/Bool` | Enable/disable the gate via topic |
| `<input_topic>` × N | (configured type) | Command inputs for each channel |
| `<fallback_topic>` × N | (configured type) | Fallback sources buffered per channel |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `<output_topic>` × N | (configured type) | Relayed or fallback output per channel |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `~/set_enabled` | `std_srvs/SetBool` | Persistent enable/disable (preferred over the enable topic) |

## Launch

```bash
# C++ standalone (default)
ros2 launch command_gate command_gate.launch.py

# Python standalone
ros2 launch command_gate command_gate.launch.py impl:=python

# C++ in a ComposableNodeContainer
ros2 launch command_gate command_gate.launch.py impl:=cpp use_composition:=true

# Custom config
ros2 launch command_gate command_gate.launch.py config:=/path/to/my_config.yaml
```

**Launch arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `impl` | `cpp` | `cpp` or `python` |
| `config` | package yaml | Path to parameter YAML |
| `use_composition` | `false` | Load C++ node into a component container |

## Integration Test

```bash
# Terminal 1 — launch
ros2 launch command_gate command_gate.launch.py impl:=cpp

# Terminal 2 — heartbeat at 5 Hz
ros2 topic pub /command_gate/heartbeat std_msgs/msg/Empty '{}' -r 5

# Terminal 3 — drive commands
ros2 topic pub /drive_in ackermann_msgs/msg/AckermannDriveStamped \
  '{header: {}, drive: {speed: 1.0}}' -r 10

# Terminal 4 — observe output (relays while heartbeat alive; zeros ~1 s after stopping)
ros2 topic echo /drive_out

# Test enable service
ros2 service call /command_gate/set_enabled std_srvs/srv/SetBool '{data: false}'
ros2 service call /command_gate/set_enabled std_srvs/srv/SetBool '{data: true}'
```

## Architecture

### Dual Implementations

| | C++ | Python |
|--|-----|--------|
| Entry point | `command_gate_node` (via `rclcpp_components`) | `command_gate_node_py` (via `setup.py`) |
| Key file | `src/command_gate_node.cpp` | `command_gate/command_gate_node.py` |
| Generic pub/sub | `rclcpp::GenericPublisher/Subscription` | `rclpy` raw subscriptions (`raw=True`) |
| Zero-init | Compile-time type list (`TRY` macro) | Runtime via `get_message()` + `serialize_message()` |
| Composition | Yes — registered as `command_gate::CommandGateNode` | No |

Supported zero-init types for C++: `geometry_msgs/Twist`, `geometry_msgs/TwistStamped`, `ackermann_msgs/AckermannDrive`, `ackermann_msgs/AckermannDriveStamped`, `std_msgs/Float64`. Unsupported types log a warning and skip zero-init; the Python node handles any type.

### Thread Safety

Both implementations assume a **single-threaded executor**. If loaded into a multi-threaded container, `heartbeat_ok_`, `enabled_`, and per-channel fallback buffers require mutex protection.