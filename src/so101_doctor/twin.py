"""The bundled SO-101 MuJoCo twin: copy it out, and check an arm's registers against what it assumes."""
from __future__ import annotations

import json
import shutil
from importlib import resources
from pathlib import Path

from .registers import JOINTS
from .report import ARM_IDS, read_ok

PS = (16, 32)  # P gains with a twin file: 16 is LeRobot's write, 32 the servo's factory value
VOLTS = (11.0, 13.0)  # the twin is a 12 V model
REGS = {"p_coefficient": ("P", "p"), "i_coefficient": ("I", "i"), "acceleration": ("Acceleration", "acceleration"),
        "maximum_acceleration": ("Maximum_Acceleration", "maximum_acceleration")}


def bundled():
    return resources.files("so101_doctor").joinpath("so101_twin")


def overlay():
    return resources.files("so101_doctor").joinpath("so101_twin_p32")


def files(root, prefix=""):
    """{relative path: Traversable} of every file under root."""
    out = {}
    for e in root.iterdir():
        if e.is_dir():
            out.update(files(e, f"{prefix}{e.name}/"))
        else:
            out[prefix + e.name] = e
    return out


def open_command(dest) -> str:
    scene = str(Path(dest) / "scene.xml")
    return "python -m mujoco.viewer --mjcf " + (f'"{scene}"' if " " in scene else scene)  # quotes: sh, PowerShell, cmd


def variant(p=16) -> dict:
    """{relative path: Traversable} of the twin file built for P 16 (the bundled folder) or P 32 (plus the overlay)."""
    if p not in PS:
        raise ValueError(f"no twin file for P {p}")
    out = files(bundled())
    if p == 32:
        out.update(files(overlay()))
    return out


def pick(servos: dict, option=None):
    """(P of the twin file to use, source): the option wins; else the P every readable arm servo reads, if 16 or 32."""
    if option:
        return option, "option"
    ps = {v["registers"]["p_coefficient"] for i, v in read_ok(servos).items() if i in ARM_IDS}
    return (ps.pop(), "arm") if len(ps) == 1 and ps <= set(PS) else (16, "default")


def why(p, source, arm_read, demo=False) -> str:
    if source == "arm":
        return "read from the demo values, not a real arm" if demo else "read from your arm"
    if source == "option":
        return "chosen with --twin-p"
    return ("LeRobot's default; the arm's P gains are not all 16 or all 32" if arm_read else
            "LeRobot's default; no arm read, pass --twin-p 32 for an arm set to the factory gain")


def assumed(p=16) -> dict:
    return json.loads(variant(p)["values.json"].read_text())["registers"]


def dest_state(dest, p=16) -> str:
    """'new' (absent or empty), 'same' (already holds the export for P p) or 'busy' (anything else)."""
    d = Path(dest)
    if not d.exists() or (d.is_dir() and not any(d.iterdir())):
        return "new"
    if not d.is_dir():
        return "busy"
    have = {str(p.relative_to(d)).replace("\\", "/") for p in d.rglob("*") if p.is_file()}
    want = variant(p)
    same = have == set(want) and all((d / k).read_bytes() == v.read_bytes() for k, v in want.items())
    return "same" if same else "busy"


def holds_other(dest, p) -> bool:
    return dest_state(dest, 48 - p) == "same"


def export(dest, p=16) -> None:
    for k, v in variant(p).items():
        t = Path(dest) / k
        t.parent.mkdir(parents=True, exist_ok=True)
        with v.open("rb") as src, open(t, "wb") as out:
            shutil.copyfileobj(src, out)


def check(servos: dict, p=16) -> dict:
    """Servos 1-5 against the assumptions of the twin for P p: {"match": {id: bool or None}, "mismatches", "assumes"}."""
    want, ok = assumed(p), read_ok(servos)
    match, bad = {}, []
    for i in ARM_IDS:
        if i not in ok:
            match[i] = None
            continue
        r, v, joint = ok[i]["registers"], ok[i]["derived"]["volts"], JOINTS[i]
        before = len(bad)
        for reg, (name, key) in REGS.items():
            if r[reg] == want[key]:
                continue
            fix = ""
            if reg == "acceleration" and r[reg] == 0:
                text = "Acceleration (register 41) reads 0: not set since power-on"
                fix = "connect once with LeRobot, which sets 254"
            else:
                text = f"{name} is {r[reg]}, the twin assumes {want[key]}"
            bad.append({"servo": i, "joint": joint, "register": reg, "reads": r[reg], "assumes": want[key],
                        "text": text, "fix": fix})
        if not VOLTS[0] <= v <= VOLTS[1]:
            bad.append({"servo": i, "joint": joint, "register": "present_voltage", "reads": v,
                        "assumes": want["vin"], "text": f"supply is {v:.1f} V, the twin assumes {want['vin']:g} V "
                        f"(within {VOLTS[0]:g} to {VOLTS[1]:g})", "fix": ""})
        match[i] = len(bad) == before
    ps = {ok[i]["registers"]["p_coefficient"] for i in ARM_IDS if i in ok}
    return {"match": match, "mismatches": bad, "assumes": want, "mixed_p": len(ps) > 1}


def text(c: dict) -> str:
    w = c["assumes"]
    head = (f"Twin check (servos 1-5; the twin assumes P {w['p']}, I {w['i']}, Acceleration {w['acceleration']}, "
            f"Maximum_Acceleration {w['maximum_acceleration']}, {w['vin']:g} V). The gripper (id 6) is not part of "
            "the twin and is not judged.")
    lines = [head]
    for m in c["mismatches"]:
        fix = f"; fix: {m['fix']}" if m["fix"] else ""
        lines.append(f"  servo {m['servo']} ({m['joint']}): {m['text']}{fix}")
    unread = [str(i) for i, m in c["match"].items() if m is None]
    if unread:
        lines.append(f"  servo {', '.join(unread)}: not read, not checked")
    if c.get("mixed_p"):
        lines.append("  Mixed P gains are not tested with the twin: set all five arm servos to 16 or to 32.")
    if c["mismatches"]:
        lines.append("The twin will be less accurate until the arm matches these values.")
    elif not unread:
        lines.append("All five arm servos match what the twin assumes.")
    return "\n".join(lines)
