# so101-doctor

Checks how the six servos of an SO-101 arm are set up, and writes a MuJoCo twin of the arm built for the servo gain
your arm runs. Read only: it cannot move the arm, change a setting or switch torque. Six servos take under a second.
Step by step, with fixes: https://vatolabs.com/doctor. If it is useful, a star helps others find it.

## Install and run

Needs Python 3.10 or newer (3.9 is enough for the check alone, without `mujoco==3.17.0`).

**macOS / Linux**
```
python3 -m venv ~/so101
~/so101/bin/pip install https://github.com/VatoLabs/so101-doctor/archive/refs/heads/main.zip mujoco==3.17.0
~/so101/bin/so101-doctor                                  # check the arm; --demo shows example output without one
~/so101/bin/so101-doctor --export-mujoco ~/so101_twin     # write the twin; works without an arm
~/so101/bin/python -m mujoco.viewer --mjcf ~/so101_twin/scene.xml
```

**Windows (PowerShell)**
```
py -m venv "$HOME\so101"
& "$HOME\so101\Scripts\pip" install https://github.com/VatoLabs/so101-doctor/archive/refs/heads/main.zip mujoco==3.17.0
& "$HOME\so101\Scripts\so101-doctor"
& "$HOME\so101\Scripts\so101-doctor" --export-mujoco "$HOME\so101_twin"
& "$HOME\so101\Scripts\python" -m mujoco.viewer --mjcf "$HOME\so101_twin\scene.xml"
```

Before the check: power the servos, plug in the board, close LeRobot. The usual servo board (USB vendor 1a86) is
found automatically; with several boards, or another make, pass `--port`. "Permission denied" on Linux:
`sudo usermod -aG dialout $USER`, then log out and back in.

Checked by us: every step on Ubuntu 24.04 (Python 3.10; the viewer window opened with the twin, its sliders and keys
were not tried; install, demo and export also on 3.12) and the register check on macOS. Not yet checked: Windows, and
the export and viewer on macOS.

## What it prints

One row per servo (firmware, voltage, temperature, gains, the acceleration limit the firmware applies), then
warnings in plain words.

```
so101-doctor: 6 of 6 servos read on DEMO (example values, not a real arm) (scan complete)

id  joint         fw    volts temp  P/D/I      acc max85  accel rad/s2  60 deg min  note
 1  shoulder_pan  3.10   12.0  30C  16/32/0    254   254          39.0      0.33 s
 3  elbow_flex    3.10   12.0  30C  16/32/0    254    50           7.7      0.74 s  capped by Maximum_Acceleration 50
 6  gripper       3.10   12.0  30C  16/32/0      0   254           7.7      0.74 s  Acceleration 0 acts as 50

WARNING  servo 3 (elbow_flex): Acceleration 254 asks for 39.0 rad/s^2 but Maximum_Acceleration (register 85) = 50
         caps it at 7.7 rad/s^2: 5.1x lower, so a move from rest takes up to 2.3x longer
```
(`--demo` output, shortened: three of the six rows, one of the three messages, long lines wrapped, the legend left out.)

Just powered the arm? Every joint reads Acceleration 0. That is the power-on state, not a fault: connect with LeRobot
once (it writes 254), close it, and read again.

## The twin

`--export-mujoco DIR` writes Menagerie's SO-101 with the five arm servos replaced by native STS3215 models, for
LeRobot's registers (I 0, Acceleration 254, Maximum_Acceleration 254, 12 V) and the P gain your arm runs. `DIR` must
be new or empty; nothing is overwritten.

- Arm plugged in, joints 1-5 all at P 16 (what LeRobot writes) or all at P 32 (factory): the twin for that P. It also
  checks servos 1-5 against the twin's values and names what differs.
- No arm: the P 16 twin; add `--twin-p 32` for an arm at 32.
- P 16 on some joints and 32 on others: the P 16 twin and a note (mixed gains are not tested).

Drive it like the servo: `ctrl` = joint goal in rad. The gripper keeps Menagerie's actuator.

How close it is, on one arm with nothing in the gripper, not yet replicated. Same commands to the real arm and the
simulation; distance between the two gripper tips at rest, mean over the holds:

| arm set to | twin | stock Menagerie | holds |
|---|---|---|---|
| P 16 | 6.4 mm | 15.2 mm | 180 (8 Nov 2026) |
| P 32 | 3.7 mm | 7.7 mm | 164 (10 Nov 2026) |

In motion at P 16, replaying five teleoperation recordings, it tracked the joints to 1.25 deg RMSE against 2.23 deg
for stock Menagerie, closer on every joint of every recording (same arm, recorded 2 Nov 2026, not yet replicated).
Not checked: other arms, a load in the gripper, the 7.4 V servo version, fast teleoperation at P 32. Scored on MuJoCo
3.17.0 (only the 2.23 deg was computed on 4.0.0); on 4.0.0 the file loads and one 3 s test move ended at the same
joint angles.

## Why the acceleration matters

The STS3215 moves an internal target toward the goal at a constant acceleration,
**a = 100 steps/s^2 x min(Acceleration, Maximum_Acceleration)** (Acceleration 0 acts as 50), and its position loop
follows that target, not the goal. On one bench servo at LeRobot's settings, adding this profile to a fitted servo
model cut its position error from 18.1 to 7.1 mrad (stock MuJoCo Menagerie: 68.3 mrad). Measurements, method and
credits: [VatoLabs/STS3215_profile](https://github.com/VatoLabs/STS3215_profile).

```python
from so101_doctor.profile import profile_accel, internal_target
a = profile_accel(254, 254)                 # 38.96 rad/s^2
target = internal_target(goals, a, dt=0.02)  # what the servo's position loop really tracks
```

## Options, scripts and AI agents

`--port PORT`, `--ids 1-6` (IDs 0-253, ranges and lists), `--baud 1000000`, `--json`, `--list-ports`, `--strict`,
`--demo`, `--export-mujoco DIR`, `--twin-p {16,32}`.

Exit codes: 0 a report was printed (it may hold warnings or be incomplete), 1 nothing could be read (port not found
or busy, no servo answered; with `--export-mujoco` and no port found the twin is still written and the code is 0),
2 bad command line or an export folder that holds other files or the other P's twin, 3 with `--strict` when the
report has a warning or the scan is incomplete.

`--json` prints one object: `version`, `source`, `complete`, `bus_error`, `servos`, `findings` (level and text) and,
with `--export-mujoco`, `twin` (`exported`, `p`, `p_source`, `checked`, `reason`, `match`, `mismatches`, `mixed_p`,
`assumes`). Full instructions for an AI agent: https://vatolabs.com/llms.txt. Do not change servo registers to make
the check pass without asking the owner.

## Coming soon

Per-arm calibration: a twin fitted to your own arm from a short recording session. On our arm, with 200 g at the
jaw, it cut the tip error 5.1x (measured by camera at a tag on the gripper; one arm, one session, 9 Nov 2026, not yet
replicated). After that: payload and contact force from the servos' own load readings, with no added sensor.

## Licence

Apache-2.0 (LICENSE, NOTICE). The bundled twin folders derive from MuJoCo Menagerie (Apache-2.0); each folder's
README.md says what was changed.
