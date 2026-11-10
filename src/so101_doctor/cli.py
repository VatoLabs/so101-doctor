"""so101-doctor: read the registers of every servo on an SO-101 arm and say what they do. Read only."""
from __future__ import annotations

import argparse
import json
import sys
from importlib import resources
from pathlib import Path

from . import __version__, twin
from .bus import BAUD, WCH, Bus, BusError, usb_ports
from .registers import read_all
from .report import derived, findings, read_ok, text

EXIT_CODES = """exit codes:
  0  a report was printed (it may hold warnings or be incomplete: see --strict)
  1  nothing could be read: port not found or busy, or no servo answered
     (with --export-mujoco and no port found, the twin is still written: exit 0)
  2  bad command line, or an export folder that is not empty
  3  --strict only: the report has a warning or the scan is incomplete"""


def parse_ids(s: str):
    """'1-6' or '1,2,5' -> sorted IDs without repeats. Servo IDs are 0-253 (254 is the broadcast address)."""
    out = set()
    try:
        for part in s.split(","):
            a, _, b = part.partition("-")
            lo, hi = int(a), int(b or a)
            if not 0 <= lo <= hi <= 253:
                raise ValueError
            out.update(range(lo, hi + 1))
    except ValueError:
        raise argparse.ArgumentTypeError(f"{s!r}: give IDs from 0 to 253, such as 1-6 or 1,2,5") from None
    return sorted(out)


class NoPort(SystemExit):
    """No servo board to read: exits with the message, unless --export-mujoco goes on without an arm."""


def pick_port():
    """The servo board's port when exactly one is plugged in. Packets are never sent to a port picked by guess:
    a USB serial device of another make (an Arduino, a printer) is only listed."""
    ports = usb_ports()
    boards = [p for p in ports if p.vid == WCH]
    if len(boards) == 1:
        return boards[0].device
    if not ports:
        raise NoPort("No USB serial port found. Plug in the arm's servo board (and power it), or pass --port.")
    listed = "\n".join(f"  {p.device}  {p.description}" for p in ports)
    what = "Several servo boards" if boards else "No port looks like the usual servo board (USB vendor 1a86)"
    raise NoPort(f"{what}; pass yours with --port:\n{listed}")


def scan(port, ids, baud):
    """{id: {"registers", "error"} or {"failed"}} and a bus-wide error message (None if the bus held up).
    A servo that answers PING but fails a read is recorded and the scan goes on; a serial error stops it."""
    import serial

    servos = {}
    with Bus(port, baud) as bus:
        for i in ids:
            try:
                if not bus.ping(i):
                    continue
                regs, err = read_all(bus, i)
                servos[i] = {"registers": regs, "error": err}
            except BusError as e:
                servos[i] = {"failed": str(e)}
            except (serial.SerialException, OSError) as e:
                return servos, f"the serial connection failed at servo {i}: {e}. Check the USB cable and power."
    return servos, None


def main(argv=None):
    ap = argparse.ArgumentParser(prog="so101-doctor", description=__doc__, epilog=EXIT_CODES,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port of the servo board (default: the only servo board plugged in)")
    ap.add_argument("--ids", type=parse_ids, default="1-6", help="servo IDs to read, e.g. 1-6 or 1,2,3 (default 1-6)")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--json", action="store_true", help="print everything as JSON")
    ap.add_argument("--demo", action="store_true", help="no arm needed: run on built-in example values")
    ap.add_argument("--strict", action="store_true", help="exit 3 on any warning or an incomplete scan (for scripts)")
    ap.add_argument("--export-mujoco", metavar="DIR", help="copy the SO-101 MuJoCo twin to DIR (an empty or new "
                    "folder; the P 16 file unless --twin-p says 32 or the arm reads 32); with an arm read in the same run, "
                    "check its registers against the twin")
    ap.add_argument("--twin-p", type=int, choices=twin.PS, help="P gain the exported twin is built for: 16 (LeRobot's "
                    "write) or 32 (the servo's factory value). Default: 32 if every arm servo read reads P 32, else 16")
    ap.add_argument("--list-ports", action="store_true", help="list USB serial ports and exit")
    ap.add_argument("--version", action="version", version=f"so101-doctor {__version__}")
    a = ap.parse_args(argv)
    if a.list_ports:
        for p in usb_ports():
            print(f"{p.device}  {p.description}  {p.vid:04x}:{p.pid:04x}")
        return 0
    ids, bus_error, skipped, no_port = a.ids, None, None, False
    if a.export_mujoco:
        a.export_mujoco = str(Path(a.export_mujoco).expanduser())  # PowerShell passes ~ through

    def refuse(p):
        other = twin.holds_other(a.export_mujoco, p)
        print(f"so101-doctor: {a.export_mujoco} " + (f"holds the twin for P {48 - p}, not P {p}: pass a new folder."
              if other else "exists and is not empty (nor an identical export): pass a new or empty folder."),
              file=sys.stderr)
        return 2

    if a.export_mujoco:  # before the scan, refuse only what no variant could use
        for p in ([a.twin_p] if a.twin_p else twin.PS):
            if twin.dest_state(a.export_mujoco, p) != "busy":
                break
        else:
            return refuse(a.twin_p or 16)
    if a.demo:
        raw = json.loads(resources.files("so101_doctor").joinpath("demo.json").read_text())
        servos = {int(k): {"registers": v, "error": 0} for k, v in raw["servos"].items() if int(k) in ids}
        source = "DEMO (example values, not a real arm)"
    else:
        servos, source = {}, None
        try:
            source = a.port or pick_port()
            servos, bus_error = scan(source, ids, a.baud)
        except NoPort as e:
            if not a.export_mujoco:
                raise
            skipped, no_port = str(e).splitlines()[0], True
        except BusError as e:
            if not a.export_mujoco:
                sys.exit(f"so101-doctor: {e}")
            skipped = f"{e}"
    for s in servos.values():
        if "registers" in s:
            s["derived"] = derived(s["registers"])
    found = findings(servos, ids)
    complete = len(read_ok(servos)) == len(ids)
    t = None
    if a.export_mujoco:
        why = skipped or (None if read_ok(servos) else "no servo answered")
        tp, tsrc = twin.pick(servos, a.twin_p)
        state = twin.dest_state(a.export_mujoco, tp)
        if state == "busy":
            return refuse(tp)
        t = {"exported": None, "checked": not why, "reason": why, "p": tp,
             "p_source": "demo" if a.demo and tsrc == "arm" else tsrc,
             **(twin.check(servos, tp) if not why else {})}
        try:
            if state == "new":
                twin.export(a.export_mujoco, tp)
        except OSError as e:
            sys.exit(f"so101-doctor: could not write {a.export_mujoco}: {e}")
        t["exported"] = str(a.export_mujoco)
    if a.json:
        out = {"version": __version__, "source": source, "complete": complete, "bus_error": bus_error,
               "servos": servos, "findings": [{"level": lvl, "text": x} for lvl, x in found]}
        if t:
            out["twin"] = t
        print(json.dumps(out, indent=1, allow_nan=False))
    else:
        if not skipped:
            print(text(servos, ids, source, found))
        if bus_error:
            print(f"\nScan stopped early: {bus_error}")
        if t:
            d = t["exported"]
            print(("\n" if not skipped else "") + (twin.text(t) if t["checked"] else f"Twin check: arm not checked "
                  f"({t['reason']})."))
            print(f"\n{'Already exported' if state == 'same' else 'Exported'} the SO-101 MuJoCo twin for P {tp} "
                  f"({twin.why(tp, tsrc, bool(read_ok(servos)), a.demo)}): {d}\n"
                  f"Open it: {twin.open_command(d)}\n"
                  "Needs mujoco 3.17 (pip install mujoco==3.17.0, Python 3.10+).")
    if no_port:
        return 0
    if skipped:
        return 1
    if not read_ok(servos):
        what = "No servo could be read" if servos else "No servo answered"
        print(f"\n{what}. Check power (the servos need their own supply), the cable, --baud and --ids.",
              file=sys.stderr)
        return 1
    if a.strict and (not complete or any(lvl == "warn" for lvl, _ in found)):
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
