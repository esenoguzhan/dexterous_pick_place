# LARA5 grasp simulation bringup (ROS 2 Humble)

Start the complete tested simulation in one command:

```bash
ros2 launch lara5_grasp_bringup simulation.launch.py
```

This launches MuJoCo with its viewer, simulated external/wrist cameras, robot TF,
controller manager and all three controllers, native safety/planning backend,
`object_approach`, `hand_grasp`, `object_lift`, and one RViz window with the grasp/path displays.
The backend is explicitly simulation mode with the hand and safety filter on.
It does not connect to physical robot hardware.

Startup leaves all three tasks idle. Supply the object-centre pose manually on
`/arm/approach/target`, wait for verified positioning, then explicitly trigger
`/arm/grasp/target`. After closure, explicitly request `/arm/lift/target`. No command messages are published by this launch. The
independent task launches remain available. Execution supplies the RViz preview
markers/paths; there is no competing planning-preview node.

[Complete two-terminal Docker/UI instructions](../../../docs/grasp_bringup_simulation.md).

## Options

| Argument | Default | Purpose |
| --- | --- | --- |
| `sim_headless` | `false` | Hide MuJoCo viewer when true |
| `rviz` | `true` | Start exactly one RViz window |
| `sim_camera_backend` | `egl` | EGL cameras with the viewer; `osmesa` is headless only |
| `sim_config` | Installed `lara5_mujoco_sim/config/mujoco_sim.yaml` | Physics, objects and simulated cameras |
| `sim_initial_positions` | `0 0 1.5708 0 1.5708 0` | Initial six arm joints in radians |
| `approach_config` | Installed `lara5_object_approach/config/approach.yaml` | Positioning/grasp-frame parameters |
| `grasp_config` | Installed `lara5_grasp/config/grasp.yaml` | Closure/contact parameters |
| `lift_config` | Installed `lara5_object_lift/config/lift.yaml` | Verified lift parameters |
| `rviz_config` | Installed `lara5_grasp_bringup/rviz/grasp_lift.rviz` | Approach and magenta lift paths |

Examples:

```bash
# No GUI, same ROS interfaces:
ros2 launch lara5_grasp_bringup simulation.launch.py sim_headless:=true rviz:=false
# Full custom task YAML files:
ros2 launch lara5_grasp_bringup simulation.launch.py approach_config:=/path/approach.yaml grasp_config:=/path/grasp.yaml
```

Each included launch has its own configuration scope, so their generic
`config_file` arguments cannot overwrite one another. The backend and approach
child RViz instances are disabled; this launch owns the single RViz instance.

Wait for active controllers and `Safety filter ready` before sending inputs.
Ctrl+C stops the complete launch, including the simulation backend and viewer.
Use one backend/bringup on a ROS domain and one motion client per controller.
Closure contact evidence alone is not verified retention. The explicit lift task
verifies physical rise and retained grip; no automatic opening is added.

## Validation

The package startup test launches a real headless backend in an isolated ROS
domain, verifies active controllers, all three idle tasks, camera/force feedback,
unique command subscribers and correct separate YAML overrides, then shuts down.
It sends no arm or hand goals. A private-display opt-in also verifies that exactly
one RViz instance starts:

```bash
colcon test --packages-select lara5_grasp_bringup --event-handlers console_direct+
colcon test-result --test-result-base build/lara5_grasp_bringup/test_results --verbose
```
