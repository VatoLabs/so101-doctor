"""What the registers mean: per-servo derived values and findings."""
from __future__ import annotations

import math

from .profile import UNIT, profile_accel
from .registers import JOINTS, STS3215

MOVE_DEG = 60.0
ARM_IDS = (1, 2, 3, 4, 5)  # the gripper is often set up differently on purpose


def accel(r: dict):
    """(rad/s^2 or None if the profile is off, note). Registers above 254 were never measured."""
    acc, cap = r["acceleration"], r["maximum_acceleration"]
    if acc > 254 or cap > 254:
        return None, "unknown: a register above 254 was never measured"
    a = profile_accel(acc, cap)
    if a is None:
        return None, "profile off (Maximum_Acceleration 0)"
    if acc == 0:
        return a, "Acceleration 0 acts as 50"
    if acc > cap:
        return a, f"capped by Maximum_Acceleration {cap}"
    return a, ""


def derived(r: dict) -> dict:
    a, note = accel(r)
    t = 2 * math.sqrt(math.radians(MOVE_DEG) / a) if a else None
    return {"joint": JOINTS.get(r["id"], "?"), "firmware": f"{r['firmware_major_version']}.{r['firmware_minor_version']}",
            "volts": r["present_voltage"] / 10, "max_volts": r["max_voltage_limit"] / 10,
            "temperature_c": r["present_temperature"], "accel_rad_s2": a, "accel_note": note, "move_60deg_min_s": t}


# Servo status (register 65) and the reply error byte share these bits (ST memory map, Waveshare docs)
FAULTS = {0: "voltage out of range", 1: "magnetic encoder fault", 2: "overheating", 3: "overcurrent",
          5: "overload"}


def fault_names(v: int) -> str:
    names = [n for b, n in FAULTS.items() if v >> b & 1]
    other = v & ~sum(1 << b for b in FAULTS)
    return ", ".join(names + ([f"unknown bits 0b{other:08b}"] if other else []))


def read_ok(servos: dict) -> dict:
    return {i: s for i, s in servos.items() if "registers" in s}


def findings(servos: dict, ids) -> list:
    """[(level, text)], level 'warn' or 'info'."""
    out = []
    missing = [i for i in ids if i not in servos]
    if missing:
        out.append(("warn", f"no reply from servo id {', '.join(map(str, missing))}: check the cable, power and IDs"))
    for i, s in sorted(servos.items()):
        if "failed" in s:
            out.append(("warn", f"servo {i}: answered but its registers could not be read ({s['failed']}); "
                                "results below do not cover it"))
    servos = read_ok(servos)
    for i, s in sorted(servos.items()):
        r, d, name = s["registers"], s["derived"], f"servo {i} ({s['derived']['joint']})"
        if r["model_number"] != STS3215:
            out.append(("warn", f"{name}: model {r['model_number']}, not an STS3215 (777); the checks below assume one"))
        if d["firmware"] != "3.10":
            out.append(("info", f"{name}: firmware {d['firmware']}; the acceleration rule was measured on 3.10 only"))
        acc, cap = r["acceleration"], r["maximum_acceleration"]
        if 0 < cap < acc and d["accel_rad_s2"]:
            asked = UNIT * min(acc, 254)
            ratio = asked / d["accel_rad_s2"]
            out.append(("warn", f"{name}: Acceleration {acc} asks for {asked:.1f} rad/s^2 but Maximum_Acceleration "
                                f"(register 85) = {cap} caps it at {d['accel_rad_s2']:.1f} rad/s^2: {ratio:.1f}x "
                                f"lower, so a move from rest takes up to {math.sqrt(ratio):.1f}x longer"))
        if cap == 0:
            out.append(("info", f"{name}: Maximum_Acceleration 0: no acceleration profile, the target jumps to the goal"))
        if d["accel_note"].startswith("unknown"):
            out.append(("warn", f"{name}: acceleration not assessed: Acceleration {acc}, Maximum_Acceleration {cap} "
                                "(values above 254 were never measured)"))
        if s["error"]:
            out.append(("warn", f"{name}: the servo reported a fault while it was being read: "
                                f"{fault_names(s['error'])}"))
        if r["status"]:
            out.append(("warn", f"{name}: fault status: {fault_names(r['status'])} (voltage {d['volts']:.1f} V, "
                                f"{r['present_temperature']} C)"))
        if r["operating_mode"] != 0:
            out.append(("warn", f"{name}: operating mode {r['operating_mode']}, not position mode (0) as LeRobot sets "
                                "it; the acceleration and move times above assume position mode"))
        if r["phase"] & 0x10:
            out.append(("info", f"{name}: Phase bit 4 is set; LeRobot clears it on connect to keep positions "
                                "within 0-4095"))
        if d["volts"] > d["max_volts"]:
            out.append(("warn", f"{name}: supply {d['volts']:.1f} V is above this servo's max voltage setting "
                                f"{d['max_volts']:.1f} V: maybe a 7.4 V servo on a 12 V supply. Power off and check."))
        elif d["volts"] < r["min_voltage_limit"] / 10:
            out.append(("warn", f"{name}: supply {d['volts']:.1f} V is below its min voltage setting "
                                f"{r['min_voltage_limit'] / 10:.1f} V"))
        if r["present_temperature"] >= r["max_temperature_limit"] - 15:
            out.append(("warn", f"{name}: {r['present_temperature']} C, within 15 C of its "
                                f"{r['max_temperature_limit']} C shutdown limit"))
        lo, hi, p = r["min_position_limit"], r["max_position_limit"], r["present_position"]
        if lo >= hi:
            out.append(("warn", f"{name}: position limits {lo}-{hi} leave no range (LeRobot refuses equal limits): "
                                "recalibrate this joint"))
        elif not 0 <= p <= 4095:
            out.append(("warn", f"{name}: position {p} is outside one turn (0-4095): check Phase and the homing "
                                "offset, LeRobot expects positions within one turn"))
        elif not lo <= p <= hi:
            over = max(lo - p, p - hi)
            out.append(("info", f"{name}: position {p} is {over} step{'s' * (over > 1)} outside its limits {lo}-{hi}. "
                                "The limits clamp the goal, not the joint: it can sit past them when moved by hand "
                                "or pushed by a load. Far outside means the joint needs recalibrating."))
    zero = [i for i, s in sorted(servos.items()) if s["registers"]["acceleration"] == 0]
    if zero:
        out.append(("info", f"Acceleration (register 41) reads 0 on servo {', '.join(map(str, zero))}. The register "
                            "is not stored: it goes back to 0 each time the servo is powered. LeRobot writes 254 when "
                            "it connects, so this is the arm as it is now, not as LeRobot runs it."))
    arm = [servos[i]["registers"] for i in ARM_IDS if i in servos]
    for reg in ("acceleration", "maximum_acceleration", "p_coefficient", "d_coefficient"):
        vals = sorted({r[reg] for r in arm})
        if len(vals) > 1:
            out.append(("warn", f"joints 1-5 differ in {reg}: {vals}. Joints set up differently move differently."))
    if any(s["registers"]["torque_enable"] for s in servos.values()):
        on = [i for i, s in sorted(servos.items()) if s["registers"]["torque_enable"]]
        out.append(("info", f"torque is on for servo {', '.join(map(str, on))} (so101-doctor left it as it was)"))
    return out


def table(servos: dict) -> str:
    servos = read_ok(servos)
    head = f"{'id':>2}  {'joint':13} {'fw':5} {'volts':>5} {'temp':>4}  {'P/D/I':9} {'acc':>4} {'max85':>5}" \
           f"  {'accel rad/s2':>12}  {'60 deg min':>10}  note"
    rows = [head, "-" * len(head)]
    for i, s in sorted(servos.items()):
        r, d = s["registers"], s["derived"]
        a = f"{d['accel_rad_s2']:.1f}" if d["accel_rad_s2"] else "?" if d["accel_note"].startswith("unknown") else "off"
        t = "-" if d["move_60deg_min_s"] is None else f"{d['move_60deg_min_s']:.2f} s"
        pdi = f"{r['p_coefficient']}/{r['d_coefficient']}/{r['i_coefficient']}"
        rows.append(f"{i:>2}  {d['joint']:13} {d['firmware']:5} {d['volts']:>5.1f} {d['temperature_c']:>3}C  {pdi:9} "
                    f"{r['acceleration']:>4} {r['maximum_acceleration']:>5}  {a:>12}  {t:>10}  {d['accel_note']}")
    return "\n".join(rows)


def text(servos: dict, ids, source: str, f: list) -> str:
    done = len(read_ok(servos))
    state = "complete" if done == len(ids) else "INCOMPLETE"
    lines = [f"so101-doctor: {done} of {len(ids)} servos read on {source} (scan {state})", "", table(servos), "",
             "accel = the acceleration limit the firmware puts on its internal target, from registers 41 and 85: "
             "100 steps/s^2 x min(Acceleration, Maximum_Acceleration), Acceleration 0 acting as 50 "
             "(a change to 85 counts from the next write of 41; LeRobot writes both on connect).",
             f"60 deg min = the shortest time that acceleration allows for a {MOVE_DEG:.0f} deg move "
             "(speed limits only make it longer).", ""]
    warns = [t for lvl, t in f if lvl == "warn"]
    infos = [t for lvl, t in f if lvl == "info"]
    lines += [f"WARNING  {t}" for t in warns] + [f"note     {t}" for t in infos]
    if not warns:
        lines.append("No problems found in the registers.")
    lines += ["", "Read only: nothing was written to the servos. Method and measurements: "
              "https://github.com/VatoLabs/STS3215_profile"]
    return "\n".join(lines)
