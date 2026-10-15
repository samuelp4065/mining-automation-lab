---
title: Kinematics
layout: default
nav_order: 4
---

# The arm solve

`excavator_kinematics` is the single source of truth for the arm, and the only package
with no ROS and no hardware imports. Everything else — the ROS nodes, the simulator, the
REPL — imports it.

{: .note }
> An earlier version of this solver existed in **five copies** that had drifted apart on
> elbow convention. Consolidating it was a correctness fix, not housekeeping. Do not
> reintroduce a local copy.

---

## The model

The arm is a **3-link planar manipulator** in the vertical x–z plane.

- `theta1` is absolute from +x
- `theta2` and `theta3` are relative to the previous link
- The tool angle is the joint sum: `phi = theta1 + theta2 + theta3`

That joint sum is what makes the solve closed-form:

1. Back the wrist off the tip by `L3` along `phi`
2. Solve boom and stick with the law of cosines
3. Recover `theta3 = phi - (theta1 + theta2)`

No iteration, no Jacobian, no convergence failures.

---

## Two behaviours the tests pin down

**Reachability is checked on the wrist, not the tip.** A tip well inside the workspace
still raises an error if an aggressive tool angle throws the wrist outside it. This
surprises people, and it is correct — the wrist is what has to physically get there.

**Elbow-up is the default.** Every piece of hardware code has always used elbow-up. The
legacy solver defaulted to elbow-down and never drove the machine.

---

## Units

**Centimetres and radians inside the solver.** Degrees appear only at UI boundaries:
simulator sliders, REPL input, and the calibration functions.

---

## Calibration

Per-joint affine maps convert a joint angle to an actuator command in the range
**[−90, 90]**:

```
cmd = (angle_deg + OFFSET) * SLOPE
```

That range corresponds to the actuator's physical travel, so it implies hard mechanical
limits per joint. Those limits are enforced before any command is issued — the solver will
happily return kinematically valid poses the machine cannot physically hold, and without
the check those saturate silently at the end stops with the bucket somewhere other than
the commanded point.

Negative slopes on stick and bucket mean those actuators travel backwards relative to the
angle convention. A sign error in one of them inverts exactly one joint on the real
machine.

{: .warning }
> **Open item.** Boom and stick calibration agrees with the IK angle convention — roughly
> 61–64% of sampled workspace poses fall inside travel, as expected. The bucket does not:
> only **12.3%**, and its IK range barely overlaps its travel band. Either the bucket
> constant was derived against a different zero, or its convention differs. This needs one
> measurement on the machine. It must not be "fixed" by loosening the limit check, which is
> currently the only thing preventing silent saturation.

---

## Testing

32 tests, no ROS and no hardware required, running in under a second:

```bash
cd src/excavator_kinematics && python3 -m pytest test/ -q
```

Because the package has no hardware dependency, these run anywhere — including CI and a
development laptop.

---

## Trying it without hardware

```bash
ros2 run excavator_kinematics ik_repl        # pose -> angles -> servo commands
python3 tools/simulator/arm_simulator.py     # interactive Matplotlib simulator
```

The simulator takes real centimetres.
