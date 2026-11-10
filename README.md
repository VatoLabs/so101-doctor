# so101-doctor

Reads the configuration and status registers of every servo on an SO-101 arm and tells you how they are set up
and what that does to its motion. It can also write a twin: a simulated copy of the arm for the MuJoCo simulator,
built for the servo gain your arm runs. It only reads: it cannot move the arm, change a setting or switch torque on
or off. Six servos take under a second. Step by step for beginners: https://vatolabs.com/doctor

Install (needs Python 3.10 or newer and internet; Python 3.9 is enough for the check alone, without `mujoco==3.17.0`):

**macOS / Linux**, in a terminal:
```
python3 -m venv ~/so101
~/so101/bin/pip install https://github.com/VatoLabs/so101-doctor/archive/refs/heads/main.zip mujoco==3.17.0
~/so101/bin/so101-doctor --demo     # example output, no arm needed
```

**Windows**, in PowerShell:
```
py -m venv "$HOME\so101"
& "$HOME\so101\Scripts\pip" install https://github.com/VatoLabs/so101-doctor/archive/refs/heads/main.zip mujoco==3.17.0
& "$HOME\so101\Scripts\so101-doctor" --demo
```

Then plug in the arm's servo board, power the servos, close LeRobot if it is running, and run the same command
without `--demo`. It finds the usual servo board (USB vendor 1a86) by itself; with several boards, or a board of
another make, it lists the ports and you pass one with `--port`. On Linux, "Permission denied" means your user
may not open serial ports: `sudo usermod -aG dialout $USER`, then log out and back in.

Checked by us: every step on Ubuntu 24.04 (Python 3.10; the viewer window opened with the twin, its sliders and keys
were not tried; install, demo and export also on 3.12) and the register check on macOS. Not yet checked by us: Windows, and the export and viewer on macOS.

What you get, per servo:

- the acceleration limit its firmware puts on a move, from registers 41 and 85, and the shortest time that allows
  for a 60 deg move;
- firmware, supply voltage against the servo's own limits, temperature, gains, faults;
- warnings when something is off: register 85 capping the acceleration you asked for, joints set up differently,
  a supply above a servo's voltage setting, a missing servo, a fault bit, a position outside one turn.

```
so101-doctor: 6 of 6 servos read on DEMO (example values, not a real arm) (scan complete)

id  joint         fw    volts temp  P/D/I      acc max85  accel rad/s2  60 deg min  note
 1  shoulder_pan  3.10   12.0  30C  16/32/0    254   254          39.0      0.33 s
 3  elbow_flex    3.10   12.0  30C  16/32/0    254    50           7.7      0.74 s  capped by Maximum_Acceleration 50
 6  gripper       3.10   12.0  30C  16/32/0      0   254           7.7      0.74 s  Acceleration 0 acts as 50

WARNING  servo 3 (elbow_flex): Acceleration 254 asks for 39.0 rad/s^2 but Maximum_Acceleration (register 85) = 50
         caps it at 7.7 rad/s^2: 5.1x lower, so a move from rest takes up to 2.3x longer
note     Acceleration (register 41) reads 0 on servo 6. The register is not stored: it goes back to 0 each time
         the servo is powered. LeRobot writes 254 when it connects, so this is the arm as it is now, not as
         LeRobot runs it.
```
(`--demo` output, shortened: three of the six rows, long lines wrapped, the legend left out.)

**Just powered the arm on?** Then every joint reads Acceleration 0 and 7.7 rad/s^2. That is the power-on state,
not a fault: connect with LeRobot once (it writes 254), close it, and read again to see the arm as LeRobot runs it.

## Why the acceleration matters

The STS3215 does not drive straight to the goal you send. It moves an internal target toward the goal at a constant
acceleration, braking in time to stop on it, and the position loop follows that target:
**a = 100 steps/s^2 x min(Acceleration, Maximum_Acceleration)**, Acceleration 0 acting as 50. A simulator without
this profile has the servo chase the goal itself, which the real one does not do. On one bench servo at LeRobot's
settings, adding the profile to a fitted servo model cut its position error from 18.1 to 7.1 mrad (stock MuJoCo
Menagerie: 68.3 mrad). Measurements, method and credits: [VatoLabs/STS3215_profile](https://github.com/VatoLabs/STS3215_profile).

Use the profile in your own code:

```python
from so101_doctor.profile import profile_accel, internal_target
a = profile_accel(254, 254)                 # 38.96 rad/s^2
target = internal_target(goals, a, dt=0.02)  # what the servo's position loop really tracks
```

## SO-101 twin for MuJoCo

`so101-doctor --export-mujoco DIR` writes a MuJoCo model of the SO-101 into `DIR` (a new or empty folder, it never
overwrites other files; works offline, no mujoco import). It is Menagerie's SO-101 with the five arm servos replaced by native STS3215 models, for
LeRobot's registers (I 0, Acceleration 254, Maximum_Acceleration 254, 12 V) and the P gain your arm runs:

- arm plugged in, joints 1-5 all at P 16 (what LeRobot writes) or all at P 32 (the servo's factory value): you get
  the twin built for that P, and the last lines say so, e.g. `Exported the SO-101 MuJoCo twin for P 16 (read from your arm)`;
- no arm plugged in: the P 16 twin; add `--twin-p 32` for an arm set to 32;
- P 16 on some joints and 32 on others: the P 16 twin and a note (mixed gains are not tested).

With an arm read in the same run it also checks servos 1-5 against the twin's values and names what differs. The
gripper is not part of the twin and is not judged.

Open it: `~/so101/bin/python -m mujoco.viewer --mjcf DIR/scene.xml` (needs mujoco 3.17.0, Python 3.10+).
Scored on MuJoCo 3.17.0 (only the 2.23 deg below was computed on 4.0.0); on 4.0.0 the file loads and one 3 s test move ended at the same joint angles. Drive it
like the servo: `ctrl` = joint goal in rad.

How close it is, on one arm (bare, nothing in the gripper), not yet replicated. Same commands to the real arm and to
the simulation; distance between the two gripper tips once both have come to rest, mean over the holds:

| arm set to | twin | stock Menagerie | holds |
|---|---|---|---|
| P 16 | 6.4 mm | 15.2 mm | 180 (8 Nov 2026) |
| P 32 | 3.7 mm | 7.7 mm | 164 (10 Nov 2026) |

In motion at P 16, replaying five teleoperation recordings, it tracked the joints to 1.25 deg RMSE against 2.23 deg
for stock Menagerie, and was closer on every joint of every recording (same arm, recorded 2 Nov 2026, not yet replicated). Not checked: other arms, a load in the gripper, the 7.4 V servo version, fast teleoperation at P 32.
Licence of the exported folder: Apache-2.0 (Menagerie's), see its README.md for what changed.

## Options

`--port PORT` (default: the only servo board plugged in), `--ids 1-6` (IDs 0-253, ranges and lists), `--baud 1000000`,
`--json`, `--list-ports`, `--strict`, `--demo` (example values, no arm), `--export-mujoco DIR`, `--twin-p {16,32}`.
If LeRobot or another script has the port open, close it first.

Exit codes, for scripts: 0 a report was printed (it may hold warnings or be incomplete), 1 nothing could be read
(port not found or busy, no servo answered; with `--export-mujoco` and no port found the twin is still written and
the exit code is 0), 2 bad command line or an export folder that holds other files or the other P's twin, 3 with
`--strict` when the report has a warning or the scan is incomplete.

## For AI agents and scripts

If you are an assistant setting this up for someone, this is the whole job (macOS/Linux paths; on Windows use
`py -m venv "$HOME\so101"` and `"$HOME\so101\Scripts\..."` with the PowerShell call operator `&`):

```
python3 -m venv ~/so101                                   # Python 3.10+
~/so101/bin/pip install https://github.com/VatoLabs/so101-doctor/archive/refs/heads/main.zip mujoco==3.17.0
~/so101/bin/so101-doctor --json                           # read the arm (read only); --demo if no arm is connected
~/so101/bin/so101-doctor --export-mujoco ~/so101_twin     # write the twin for the arm's P (16 or 32); works with no arm
~/so101/bin/python -m mujoco.viewer --mjcf ~/so101_twin/scene.xml   # macOS scripts with launch_passive: use mjpython
```

Facts to rely on: the tool only sends PING and READ packets, so it cannot move the arm or change a register. It needs
the port to itself (quit LeRobot first). `--json` prints one JSON object: `version`, `source`, `complete`,
`bus_error`, `servos`, `findings` (level and text) and, with `--export-mujoco`, `twin` (`exported`, `p`, `p_source`,
`checked`, `reason`, `match`, `mismatches`, `mixed_p`, `assumes`); `p_source` is `arm`, `option`, `default` or `demo`.
The export never overwrites other files: pass a new or empty folder. The same text, with the Windows lines, is at
https://vatolabs.com/llms.txt. A freshly powered arm reads Acceleration 0 until LeRobot has connected
once; that is not a fault. Do not change servo registers to make the check pass without asking the owner.

## Coming soon

This is the first step. Next, per-arm calibration: a twin fitted to your own arm from a short recording session.
On our arm, with 200 g at the jaw, it cut the tip error 5.1x (measured by camera at a tag on the gripper; one arm, one
session, 9 Nov 2026, not yet replicated). After that: payload and contact force from the servos' own load readings,
with no added sensor.

## Licence

Apache-2.0 (LICENSE, NOTICE): free for any use, companies included. The bundled twin folders derive from MuJoCo
Menagerie (Apache-2.0); each folder's README.md says what was changed.
