# Robot Grinding Testbed ROS 2

Cell configuration, tool changes, and grinding examples for the UR20 and Yaskawa NEX10 testbed at the Manufacturing Futures Institute.

## Repository Structure

| Package | Contents |
| --- | --- |
| `rgt_manager` | Tool-management node, cell/tool transforms, YAML configuration, Xacro, and meshes |
| `rgt_interfaces` | Tool-management service definitions |
| `rgt_scripts` | World/tray calibration and SpaceMouse teleoperation |
| `rgt_examples` | Tool-change, grinding, sanding, and force-contact examples |

## Dependencies

### Other repositories

Place these alongside this repository in your workspace's `src/` directory:

- [robot_manager_ros2](https://github.com/cmu-mfi/robot_manager_ros2): common robot-manager interfaces.
- [ur_ros2](https://github.com/cmu-mfi/ur_ros2): UR20 driver bringup and robot manager.
- [ynx_ros2](https://github.com/cmu-mfi/ynx_ros2): NEX10 driver and robot manager.
- [ati_netft_ros2](https://github.com/cmu-mfi/ati_netft_ros2): needed when enabling the NEX10's external force/torque sensor.

Follow each robot repository's hardware setup instructions, including the UR factory calibration file.

### Using rosdep

Install declared dependencies, build, and source the workspace:

```zsh
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.zsh
```

For using the spacemouse:
```zsh
pip install pyspacemouse==1.1.5 --break-system-packages
```

## Configuration

Edit [rgt_manager/config/config.yaml](rgt_manager/config/config.yaml). Positions (`x`, `y`, `z`) are in metres; orientations (`ax`, `ay`, `az`, `w`) are quaternion components. Omitted components default to zero, except `w`, which defaults to one.

### Add a new tool

1. Put `my_tool.glb` in [rgt_manager/meshes](rgt_manager/meshes/). Model it in metres, with its origin and axes aligned to the tool-side connector frame. Then compress the glb files [here](https://gltf.report/)
2. Copy [grinder.macro.xacro](rgt_manager/urdf/grinder.macro.xacro) to `rgt_manager/urdf/my_tool.macro.xacro`. Change the macro name and default link name to `my_tool`, and the mesh URI to `package://rgt_manager/meshes/my_tool.glb`.
3. Optionally add `my_tool_collision.glb` and a `<collision>` element, following [ur20_tc_master.macro.xacro](rgt_manager/urdf/ur20_tc_master.macro.xacro). RGT publishes the description and TF; it does not add tools or collision objects to MoveIt planning scenes.
4. Include and instantiate the macro in [rgt.xacro](rgt_manager/urdf/rgt.xacro):

   ```xml
   <xacro:include filename="$(find rgt_manager)/urdf/my_tool.macro.xacro" />
   <xacro:my_tool />
   ```

5. Add an entry under `tools` in `config.yaml`, using a free tray and a measured TCP offset:

   ```yaml
   my_tool:
     location: tray
     tray_name: tray_5
     child_transforms:
       my_tool_tcp:
         z: -0.15  # Example only: replace with the measured offset.
   ```

The tool name must match its Xacro link. `location` is `tray` or the carrying robot (`ur20` / `nex10`); each robot using the tool needs a calibrated reference for its tray. A new tray also needs a matching `<xacro:tray name="<robot>_tray_N" />` instance in `rgt.xacro`.

### Calibrate the system

Start the robot bringups and `rgt_manager` as described under Usage. Enable `launch_servo:=true` for calibration. Confirm the robot's `tool0_transforms` match the fitted tool changer and calibration tool.

Both acquisition scripts use the SpaceMouse: the left button cycles fast, slow, and translation-only modes; the right button prints YAML. They command the standalone Servo node, so run calibration separately from other motion clients.

#### Calibrate world

Fit the calibration tool and run this for each robot (`ns:=ur20` or `ns:=nex10`):

```zsh
ros2 run rgt_scripts find_world_reference --ros-args -p ns:=ur20
```

Jog until `<robot>_calibration_tool` coincides with the intended world origin **and orientation**, then press the right button. The script prints the robot's `mount_point` pose for `robots.<robot>.mount_point`.

#### Calibrate trays

Run for each robot that will access a tray:

```zsh
ros2 run rgt_scripts find_tray_reference --ros-args -p ns:=nex10
```

Align `<robot>_tc_hook` with the tray's docking reference origin and axes, then press the right button. Save the printed robot pose under `trays.<tray_name>.references.<robot>`. Repeat for every required robot/tray pair.

#### Save configuration

Copy the printed values into the source `config.yaml`, preserving the surrounding `robots` / `trays` structure. The scripts only print values; there is no save service. Rebuild `rgt_manager`, source the overlay, and restart it:

```zsh
colcon build --symlink-install --packages-select rgt_manager
source install/setup.zsh
ros2 run rgt_manager rgt_manager
```

After reviewing their motion paths, check the saved references with:

```zsh
ros2 run rgt_scripts test_world_reference --ros-args -p ns:=ur20
ros2 run rgt_scripts test_tray_reference --ros-args -p ns:=nex10 -p tc:=nex10_tray_1
```

These checks move the robot; they are calibration utilities, not automated tests.

## Usage

### Launch the cell

In each terminal, enter the container, source Jazzy, and source `install/setup.zsh` from the workspace root. Run each command in a separate terminal:

```zsh
ros2 launch ur_bringup bringup.launch.py ns:=ur20
ros2 launch ynx_bringup bringup.launch.py ns:=nex10
ros2 run rgt_manager rgt_manager
```

Bringups use real hardware by default; add `use_mock_hardware:=true` for mock operation. Add `launch_servo:=true` for calibration or standalone Servo examples. RGT publishes cell/tool TF and `/rgt/robot_description` for visualization.

### Interact with rgt_manager

Use `change_tool` for normal tool changes. It returns the currently held tool before picking the requested one; an empty tool name requests return only:

```zsh
ros2 service call /rgt_manager/change_tool rgt_interfaces/srv/ChangeTool "{robot: nex10, tool: grinder}"
ros2 service call /rgt_manager/change_tool rgt_interfaces/srv/ChangeTool "{robot: nex10, tool: ''}"
```

Direct pickup/return and manual state correction are also available:

```zsh
ros2 service call /rgt_manager/take_tool rgt_interfaces/srv/TakeTool "{robot: nex10, tray: tray_2}"
ros2 service call /rgt_manager/return_tool rgt_interfaces/srv/ReturnTool "{robot: nex10, tray: tray_2}"
ros2 service call /rgt_manager/override_tool_location rgt_interfaces/srv/OverrideToolLocation "{tool: grinder, location: tray}"
```

Each response contains `success`. Overrides update in-memory ownership/TF without motion; restarting restores YAML state. Match overrides to the physical tool location. Send one tool operation at a time: the manager does not serialize cell-wide requests. Pickup/return sequences currently contain no latch IO command.

### rgt_examples

Review the source's tool assignments, poses, and force settings before running; these examples move hardware.

| Executable | Behavior / prerequisites |
| --- | --- |
| `tool_change_example` | Changes and returns tools on both robots. Update its UR20/sander request first: the supplied config has no UR20 reference for tray_1. |
| `grind` | Grinding pass using `grinder_tooltip` and manager admittance control; assumes the tool is mounted and overrides ownership. |
| `sand` | Sanding pass using `sander_tooltip` and manager admittance control; assumes the tool is mounted and overrides ownership. |
| `move_till_force` | Moves along tool Z until the force threshold; requires standalone Servo and `/<robot>/force_torque_sensor_broadcaster/wrench_filtered`. |

For example, with the grinder mounted and settings reviewed:

```zsh
ros2 run rgt_examples grind --ros-args -p ns:=nex10
```
