# Reference: the STS3215 (firmware 3.10) acceleration profile, measured on a bench (Vato Labs, Oct-Nov 2026).
# The servo's position loop tracks an internal target that moves toward your goal at constant acceleration and
# brakes in time to stop on it. Feed the goals you send; simulate the servo tracking the returned target.
import math

UNIT = 100 * 2 * math.pi / 4096  # [rad/s^2] per register unit (100 encoder steps/s^2)


def profile_accel(acceleration, maximum_acceleration):
    """Registers 41 (Acceleration) and 85 (Maximum_Acceleration, factory 50) -> rad/s^2, or None when 85 is 0.
    Measured: Acceleration 0 acts as 50 (at 85 = 50 and 254), 85 caps 41, 85 = 0 switches the profile off, and a
    write to 85 takes effect only once 41 is written after it (the last two first reported in Rhoban/bam PR #20).
    So pass the 85 that was in place at the last write of 41; LeRobot writes 85 then 41 on every connect.
    Both must be whole numbers 0-254 (what LeRobot writes at most; 255 was never measured). Acceleration 0 acting
    as 50 was measured at caps 50 and 254 only."""
    for r in (acceleration, maximum_acceleration):
        if isinstance(r, bool) or r != int(r) or not 0 <= r <= 254:
            raise ValueError(f"register value {r!r}: expected a whole number 0-254")
    if maximum_acceleration == 0:
        return None
    return UNIT * min(acceleration or 50, maximum_acceleration)


def internal_target(goals, accel, dt):
    """Goals [rad] sent every dt [s] -> the target the position loop tracks, one per goal (accel None: the goals)."""
    if accel is None or len(goals) == 0:
        return list(goals)
    x, v, out = goals[0], 0.0, []
    for g in goals:
        e = g - x
        brake = accel * dt * (math.sqrt(0.25 + 2 * abs(e) / (accel * dt * dt)) - 0.5)  # fastest speed that still stops
        v += max(-accel * dt, min(accel * dt, math.copysign(brake, e) - v))
        x += v * dt
        out.append(x)
    return out
