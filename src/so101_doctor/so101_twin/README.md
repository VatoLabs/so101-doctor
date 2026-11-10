# SO-101 twin for MuJoCo

Open `scene.xml` (arm on a floor) or `so101_twin.xml` (arm only) in MuJoCo 3.17 (tested on 3.17.0 only):
`python -m mujoco.viewer --mjcf scene.xml`, the `simulate` app, or `mujoco.MjModel.from_xml_path`.
Drive it with `ctrl` = joint goal [rad] for the five arm actuators (as the servo's goal register) and the gripper.

The five arm servos are modelled as STS3215 with the registers P 16, I 0, Acceleration 254,
Maximum_Acceleration 254 (LeRobot's defaults) and a 12 V supply. The numbers below were measured once on one
bench STS3215 servo (12 V); the file is less accurate when the arm's registers or supply differ. The gripper is not
part of the twin: it keeps Menagerie's actuator.

## What changed from Menagerie (Apache-2.0 section 4)
Bodies, meshes, inertias, joint axes and ranges, collision geoms and the gripper actuator are Menagerie's
(robotstudio_so101, commit f3b6013), except that the optional wrist-camera mount (body `camera_mount`, its `wrist_cam`
camera and its mesh) is left out. Changed: `option` (timestep 0.0005 s, noslip_iterations 20), joint class
`sts3215_twin` on the five arm joints, and the five arm actuators, now native `dcmotor` servos (input `pos`). Each
computes, per step: `V = clip(kp/(K/R) * (ctrl_slewed - q), +-Vmax)`, `torque = K/R * V - K^2/R * qdot`: the servo's
P loop on PWM duty, its 12 V torque limit and the motor's back-EMF (K = motorconst, R = resistance).
The servo torque law is BAM's (Rhoban, arXiv 2410.08650, github.com/Rhoban/bam).

## Numbers (one value for all five arm joints)

| value | number | source |
|---|---|---|
| motorconst | 2.48816299915482 | measured once on one bench STS3215 servo (12 V) |
| resistance | 4.0497918040595 | measured once on one bench STS3215 servo (12 V) |
| kp | 21.6116926281351 | motorconst/resistance * supply * P * 0.166 (BAM's STS3215 error gain) * 1.10365 (measured on the same servo) |
| kd | 1.52870947690627 | motorconst^2/resistance: cancels dcmotor's back-EMF compensation term (the STS3215 drives voltage) |
| slewmax | 2.87138447595858 | stand-in for the firmware acceleration profile at Acceleration = Maximum_Acceleration = 254 (no native element limits setpoint acceleration); set on the same bench servo |
| Vmax | 11.64 | supply * 0.97 (BAM's STS3215 duty limit) |
| armature | 0.0239212809068759 | measured once on one bench STS3215 servo (12 V) |
| damping | 0.0328657598915 | measured once on one bench STS3215 servo (12 V) |
| frictionloss | 0.190454935353648 | one constant for the load- and speed-dependent gearbox friction; set on the same bench servo |

Not mapped: D (the model has no D term) and the load- and speed-dependent parts of the friction (folded into the frictionloss constant).
Initial state: each arm actuator has one state in `data.act`, the slewed setpoint the P loop sees. It starts at 0
after a reset; when you start the arm elsewhere, set it to the starting joint angles together with `qpos`.

## License
Derived from the SO-101 model in MuJoCo Menagerie (google-deepmind/mujoco_menagerie, robotstudio_so101, itself
derived from The Robot Studio's SO-ARM100 MJCF), released under the Apache License 2.0: LICENSE, copied unchanged.
The meshes in assets/ are Menagerie's, unchanged (the camera-mount mesh is not included).
