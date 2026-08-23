# Validation Report

Package: `picka_description`  
Target: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress  
Validation date: 2026-07-29

## Summary

| Status | Count |
|---|---:|
| PASS | 12 |
| FAIL | 0 |
| NOT RUN | 8 |
| NEEDS USER CONFIRMATION | 1 |

The package passed all static checks available in the conversion environment.
ROS 2 Humble and Gazebo Fortress were not installed in that environment, so this
report does not claim runtime confirmation.

## PASS

1. **Archive safety and integrity** — 76 original ZIP entries inspected; no
   absolute paths, parent traversal, or symlink entries; CRC test passed.
2. **XML well-formedness** — `package.xml`, all Xacro/include files, and the
   Fortress world SDF parsed successfully.
3. **Python syntax** — both launch files and `setup.py` compiled with
   `python3 -m py_compile`.
4. **YAML syntax** — controller and bridge configurations parsed successfully.
5. **Kinematic tree** — 14 unique links and 13 unique joints form one connected,
   acyclic tree with one root named `world`; every non-root link has one parent.
6. **ROS-safe names** — all link, joint, controller, resource, and package names
   are valid and consistent; exporter names containing spaces were removed.
7. **Mesh references** — all 13 `package://picka_description/meshes/...` visual
   references resolve to existing files.
8. **STL integrity and scale** — every STL is a finite, correctly sized binary
   STL; `0.001` scale is consistent with millimetre CAD geometry.
9. **Numerical validity** — all joint axes are finite and normalized, all lower
   limits are less than or equal to upper limits, and every dynamic link has a
   positive mass and positive-definite inertia satisfying triangle inequalities.
10. **Control consistency** — five controller joints match five position command
    interfaces; the passive left finger has state interfaces only.
11. **Gripper mimic consistency** — `left_gripper_joint` mimics
    `right_gripper_joint` with multiplier `-1`, matching the mirrored limits.
12. **Geometry transform check** — all revolute origins lie inside the parent and
    child mesh bounds; the zero pose and a small multi-joint moved pose remained
    connected in an independent mesh-transform rendering.

## FAIL

None in the checks that were run.

## NOT RUN — target tool or dependency unavailable

1. Official `xacro` executable expansion.
2. Official `check_urdf` parser.
3. Clean `colcon build`.
4. `ros2 pkg prefix` check against the installed package.
5. Headless Fortress launch and robot entity creation.
6. ROS topic, TF, and controller-manager runtime checks.
7. Controller activation and hardware-interface claim checks.
8. Live safe trajectory, physics stability, and Gazebo GUI inspection.

Run the commands in `README_GAZEBO.md` on Ubuntu 22.04 to complete these checks.

## NEEDS USER CONFIRMATION

**Mechanical joint intent.** The ZIP did not include CAD screenshots or design
dimensions that independently prove the intended shaft centers. The original
origins and axes were therefore preserved. Static geometry evidence is internally
consistent, but visually confirm these joints in Gazebo:

- `base_joint`
- `shoulder_joint`
- `elbow_joint`
- `wrist_joint`
- `right_gripper_joint`
- `left_gripper_joint`

Rotate one joint at a time through a small angle. Confirm that the shaft stays
inside the bracket/horn and that no child link jumps or orbits around a remote
point. Confirm that the fingers open symmetrically without collision.

## Approximations

| Item | Method |
|---|---|
| `forearm_horn_1` inertia | Solid box using original mass and STL AABB |
| `wrist_horn_1` inertia | Solid box using original mass and STL AABB |
| All collision geometry | One axis-aligned box per scaled STL |
| Fixed mounting height | Base-link mesh minimum Z, giving `-0.02 m` mount offset |

