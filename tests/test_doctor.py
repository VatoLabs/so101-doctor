import json
import re
from importlib import resources
from pathlib import Path

import pytest

from so101_doctor import bus as busmod
from so101_doctor import cli, twin
from so101_doctor.cli import main
from so101_doctor.registers import MAP, SIGN_BIT, read_all

DEMO = json.loads(resources.files("so101_doctor").joinpath("demo.json").read_text())["servos"]


def memory(regs):
    mem = bytearray(256)
    for name, (addr, n) in MAP.items():
        v = regs[name]
        if v < 0:  # sign-magnitude
            v = -v | 1 << SIGN_BIT[name]
        mem[addr:addr + n] = v.to_bytes(n, "little")
    return mem


class FakeSerial:
    """Servos on a bus: answers PING and READ, records every packet sent; optional echo and line noise (lead:
    before the echo, noise: after it), an error byte on ping replies, and a cap on the bytes one read returns."""

    def __init__(self, servos, echo=False, noise=b"", errors=None, mute_reads=(), die_at=None, lead=b"",
                 ping_error=0, chunk=256):
        self.mem = {i: memory(r) for i, r in servos.items()}
        self.echo, self.noise, self.sent, self.out = echo, noise, [], b""
        self.lead, self.ping_error, self.chunk = lead, ping_error, chunk
        self.errors = errors or {}  # id -> list of error bytes for successive READ replies
        self.mute_reads, self.die_at = mute_reads, die_at  # ids that answer PING but never READ; id that kills the bus

    def reset_input_buffer(self):
        self.out = b""

    def flush(self):
        pass

    def close(self):
        pass

    def write(self, tx):
        self.sent.append(bytes(tx))
        sid, inst = tx[2], tx[4]
        if sid == self.die_at:
            raise OSError(5, "Input/output error")
        self.out += self.lead + (bytes(tx) if self.echo else b"") + self.noise
        if sid not in self.mem or (inst == busmod.READ and sid in self.mute_reads):
            return
        data = b"" if inst == busmod.PING else bytes(self.mem[sid][tx[5]:tx[5] + tx[6]])
        err = self.errors.get(sid, []).pop(0) if inst == busmod.READ and self.errors.get(sid) else 0
        err = self.ping_error if inst == busmod.PING else err
        body = bytes([sid, len(data) + 2, err]) + data
        self.out += b"\xff\xff" + body + bytes([~sum(body) & 0xFF])

    def read(self, n):
        n = min(n, self.chunk)
        r, self.out = self.out[:n], self.out[n:]
        return r


def fake_bus(ser):
    b = busmod.Bus.__new__(busmod.Bus)
    b.ser, b.timeout, b.retries = ser, 0.05, 3
    return b


def servos():
    return {int(k): dict(v) for k, v in DEMO.items()}  # copies: tests change values


@pytest.mark.parametrize("echo,noise", [(False, b""), (True, b""), (False, b"\x00\xff\x13"), (True, b"\xff")])
def test_read_all_roundtrip(echo, noise):
    ser = FakeSerial(servos(), echo, noise)
    regs, err = read_all(fake_bus(ser), 3)
    assert err == 0 and regs == DEMO["3"]


@pytest.mark.parametrize("lead", [b"\x00", b"\xff", b"\xff\xff", b"\x13\xff"])
def test_echo_after_a_stray_byte_is_not_taken_for_the_reply(lead):
    # the echo of the 2-byte read at 85 has the shape of its reply (error 2, data 85, 2)
    ser = FakeSerial(servos(), echo=True, lead=lead)
    regs, err = read_all(fake_bus(ser), 3)
    assert err == 0 and regs == DEMO["3"]
    assert not fake_bus(FakeSerial({}, echo=True, lead=lead)).ping(3)  # an echo alone is not a servo


@pytest.mark.parametrize("echo", [False, True])
def test_ping_reply_equal_to_the_ping_is_still_a_servo(echo):
    # a servo whose only error bit is voltage answers a ping with the ping's own bytes
    assert fake_bus(FakeSerial(servos(), echo=echo, ping_error=1)).ping(3)
    assert not fake_bus(FakeSerial({}, echo=echo)).ping(3)


def test_reply_arriving_a_byte_at_a_time():
    b = fake_bus(FakeSerial(servos(), echo=True, chunk=1))
    assert b.ping(3) and read_all(b, 3) == (DEMO["3"], 0)


def test_only_ping_and_read_are_ever_sent():
    ser = FakeSerial(servos())
    b = fake_bus(ser)
    for i in range(1, 8):
        if b.ping(i):
            read_all(b, i)
    assert ser.sent and {p[4] for p in ser.sent} <= {busmod.PING, busmod.READ}
    for inst in (0x03, 0x04, 0x05, 0x06, 0x83, 0x82):
        with pytest.raises(ValueError):
            busmod.packet(1, inst, b"\x28\x00")


def test_source_has_no_write_path():
    src = "".join(p.read_text() for p in Path(busmod.__file__).parent.glob("*.py"))
    assert not re.search(r"0x0[3-6]\b|0x8[23]\b|def write_|sync_write", src)


def patch_bus(monkeypatch, s, **kw):
    monkeypatch.setattr(busmod.Bus, "__init__", lambda self, *a, **k: setattr(self, "ser", FakeSerial(s, **kw))
                        or setattr(self, "timeout", 0.02) or setattr(self, "retries", 2))


def test_missing_servo_and_cap_warning(monkeypatch, capsys):
    s = servos()
    del s[5]
    patch_bus(monkeypatch, s)
    assert main(["--port", "fake"]) == 0
    out = capsys.readouterr().out
    assert "5 of 6 servos" in out and "no reply from servo id 5" in out
    assert "register 85) = 50 caps it at 7.7 rad/s^2" in out


def test_demo_json(capsys):
    assert main(["--demo", "--json"]) == 0
    d = json.loads(capsys.readouterr().out)
    assert abs(d["servos"]["1"]["derived"]["accel_rad_s2"] - 38.96) < 0.01
    assert d["servos"]["6"]["derived"]["accel_note"] == "Acceleration 0 acts as 50"
    assert any("reads 0 on servo 6" in f["text"] and f["level"] == "info" for f in d["findings"])


def test_ids_are_checked_and_demo_follows_them(capsys):
    for bad in ("1-300", "a", "-1", "6-1", "254", "", "1,,2"):
        with pytest.raises(SystemExit) as e:
            main(["--demo", "--ids", bad])
        assert e.value.code == 2
    capsys.readouterr()
    assert main(["--demo", "--ids", "2,1,2"]) == 0
    out = capsys.readouterr().out
    assert "2 of 2 servos read" in out and "elbow_flex" not in out


def test_exit_codes(monkeypatch, capsys):
    assert main(["--demo"]) == 0 and main(["--demo", "--strict"]) == 3  # the demo holds a warning
    assert main(["--demo", "--strict", "--ids", "1,2"]) == 0
    s = servos()
    s[3]["maximum_acceleration"] = 254
    patch_bus(monkeypatch, s, mute_reads=(2,))
    assert main(["--port", "fake", "--strict"]) == 3  # incomplete scan
    patch_bus(monkeypatch, {})
    assert main(["--port", "fake"]) == 1
    assert "No servo answered" in capsys.readouterr().err


def test_error_byte_kept_from_every_reply():
    ser = FakeSerial(servos(), errors={2: [4, 0, 0]})
    assert read_all(fake_bus(ser), 2)[1] == 4


def test_failed_read_keeps_the_rest(monkeypatch, capsys):
    patch_bus(monkeypatch, servos(), mute_reads=(2,))
    assert main(["--port", "fake"]) == 0
    out = capsys.readouterr().out
    assert "5 of 6 servos read" in out and "INCOMPLETE" in out and "servo 2: answered but" in out
    assert "\n 3  elbow_flex" in out and "\n 6  gripper" in out
    patch_bus(monkeypatch, servos(), mute_reads=(2,))
    main(["--port", "fake", "--json"])
    d = json.loads(capsys.readouterr().out)
    assert d["complete"] is False and "failed" in d["servos"]["2"] and "derived" in d["servos"]["6"]


def test_bus_dies_mid_scan(monkeypatch, capsys):
    patch_bus(monkeypatch, servos(), die_at=4)
    assert main(["--port", "fake"]) == 0
    out = capsys.readouterr().out
    assert "3 of 6 servos read" in out and "Scan stopped early: the serial connection failed at servo 4" in out


def run_regs(monkeypatch, capsys, **changes):
    s = servos()
    for k, v in changes.items():
        s[3][k] = v
    patch_bus(monkeypatch, s)
    main(["--port", "fake"])
    return capsys.readouterr().out


def test_config_checks(monkeypatch, capsys):
    out = run_regs(monkeypatch, capsys, min_position_limit=2048, max_position_limit=2048, present_position=2048)
    assert "servo 3 (elbow_flex): position limits 2048-2048 leave no range" in out
    assert "operating mode 1, not position mode" in run_regs(monkeypatch, capsys, operating_mode=1)
    assert "Phase bit 4 is set" in run_regs(monkeypatch, capsys, phase=16 | 12)
    out = run_regs(monkeypatch, capsys, status=0b100101)
    assert "fault status: voltage out of range, overheating, overload" in out
    out = run_regs(monkeypatch, capsys, present_position=DEMO["3"]["max_position_limit"] + 1)
    assert "note     servo 3 (elbow_flex): position" in out and "is 1 step outside its limits" in out
    out = run_regs(monkeypatch, capsys, present_position=-5)  # sign-magnitude, bit 15
    assert "WARNING  servo 3 (elbow_flex): position -5 is outside one turn" in out


def test_unknown_acceleration_is_flagged_and_json_strict(monkeypatch, capsys):
    out = run_regs(monkeypatch, capsys, maximum_acceleration=255)
    assert "acceleration not assessed" in out and "No problems found" not in out
    for v in range(256):
        for reg in ("acceleration", "maximum_acceleration"):
            s = servos()
            s[1][reg] = v
            patch_bus(monkeypatch, s)
            main(["--port", "fake", "--json"])
            json.loads(capsys.readouterr().out, parse_constant=lambda c: pytest.fail(f"{reg}={v}: {c} in JSON"))


def stock():
    s = servos()
    for r in s.values():
        r["maximum_acceleration"] = 254
    s[6]["acceleration"] = 254
    return s


def checked(s):
    return twin.check({i: {"registers": r, "derived": cli.derived(r)} for i, r in s.items()})


def test_export_writes_the_bundled_files(tmp_path, capsys):
    d = tmp_path / "twin"
    assert main(["--demo", "--export-mujoco", str(d)]) == 0
    out = capsys.readouterr().out
    assert "python -m mujoco.viewer --mjcf" in out and "mujoco==3.17.0" in out
    assert "twin for P 16 (read from the demo values, not a real arm)" in out and "your arm" not in out.split("Exported")[1]
    want = twin.files(twin.bundled())
    assert {"scene.xml", "so101_twin.xml", "LICENSE", "README.md", "values.json"} <= set(want)
    assert {str(p.relative_to(d)) for p in d.rglob("*") if p.is_file()} == set(want)
    assert main(["--demo", "--json", "--export-mujoco", str(d)]) == 0  # same P again: nothing rewritten
    assert json.loads(capsys.readouterr().out)["twin"]["p_source"] == "demo"
    assert all((d / k).read_bytes() == v.read_bytes() for k, v in want.items())
    assert main(["--demo", "--export-mujoco", str(d)]) == 0  # an identical export is not a conflict
    assert "Already exported" in capsys.readouterr().out


def test_export_refuses_a_busy_folder(tmp_path, capsys):
    d = tmp_path / "mine"
    d.mkdir()
    (d / "notes.txt").write_text("keep")
    assert main(["--demo", "--export-mujoco", str(d)]) == 2
    assert "not empty" in capsys.readouterr().err and [p.name for p in d.iterdir()] == ["notes.txt"]
    (d / "notes.txt").unlink()
    assert main(["--demo", "--export-mujoco", str(d)]) == 0  # an empty folder is fine
    (d / "README.md").write_text("changed")
    assert main(["--demo", "--export-mujoco", str(d)]) == 2


def test_twin_check_passes_on_stock_and_judges_five_servos():
    c = checked(stock())
    assert c["mismatches"] == [] and c["match"] == {i: True for i in range(1, 6)}
    s = stock()
    s[6].update(p_coefficient=40, i_coefficient=9, maximum_acceleration=0, present_voltage=40)
    assert checked(s)["mismatches"] == []  # the gripper is not judged
    assert "gripper (id 6) is not part of the twin" in twin.text(c)


def test_twin_check_flags_p_acceleration_and_voltage():
    s = stock()
    s[2]["p_coefficient"] = 32  # one servo at 32 is a mixed arm
    s[3]["acceleration"] = 0
    s[4]["present_voltage"] = 74
    c = checked(s)
    assert c["match"] == {1: True, 2: False, 3: False, 4: False, 5: True}
    t = twin.text(c)
    assert "servo 2 (shoulder_lift): P is 32, the twin assumes 16" in t
    assert "servo 3 (elbow_flex): Acceleration (register 41) reads 0: not set since power-on; fix: connect once with LeRobot" in t
    assert "servo 4 (wrist_flex): supply is 7.4 V, the twin assumes 12 V" in t
    assert "less accurate until the arm matches" in t
    assert "Mixed P gains are not tested with the twin: set all five arm servos to 16 or to 32." in t


def all_p(p):
    s = stock()
    for r in s.values():
        r["p_coefficient"] = p
    return s


def exported(d):
    kp = re.search(r'controller="([\d.]+) ', (d / "so101_twin.xml").read_text())
    return json.loads((d / "values.json").read_text())["registers"]["p"], kp


def test_uniform_p32_arm_matches_and_exports_the_p32_file(monkeypatch, capsys, tmp_path):
    patch_bus(monkeypatch, all_p(32))
    assert main(["--port", "fake", "--json", "--export-mujoco", str(tmp_path / "t")]) == 0
    d = json.loads(capsys.readouterr().out)["twin"]
    assert d["p"] == 32 and d["p_source"] == "arm" and d["mismatches"] == [] and d["assumes"]["p"] == 32
    p, kp = exported(tmp_path / "t")
    assert p == 32 and kp.group(1).startswith("43.22")
    assert {k: v.read_bytes() for k, v in twin.variant(32).items()} == {
        k: (tmp_path / "t" / k).read_bytes() for k in twin.variant(32)}
    patch_bus(monkeypatch, all_p(32))
    assert main(["--port", "fake", "--export-mujoco", str(tmp_path / "u")]) == 0
    out = capsys.readouterr().out
    assert "twin for P 32 (read from your arm)" in out and "All five arm servos match" in out
    assert main(["--port", "fake", "--export-mujoco", str(tmp_path / "u")]) == 0  # same variant again, no re-read needed
    assert "Already exported the SO-101 MuJoCo twin for P 32" in capsys.readouterr().out


def test_mixed_arm_gets_the_p16_file_and_the_mixed_line(monkeypatch, capsys, tmp_path):
    s = stock()
    s[2]["p_coefficient"] = 32
    patch_bus(monkeypatch, s)
    assert main(["--port", "fake", "--json", "--export-mujoco", str(tmp_path / "t")]) == 0
    d = json.loads(capsys.readouterr().out)["twin"]
    assert d["p"] == 16 and d["p_source"] == "default" and d["mixed_p"] is True
    assert [m["servo"] for m in d["mismatches"]] == [2] and exported(tmp_path / "t")[0] == 16
    patch_bus(monkeypatch, s)
    assert main(["--port", "fake", "--export-mujoco", str(tmp_path / "u")]) == 0
    out = capsys.readouterr().out
    assert "servo 2 (shoulder_lift): P is 32, the twin assumes 16" in out and "Mixed P gains are not tested" in out


def test_twin_p_option(monkeypatch, capsys, tmp_path):
    monkeypatch.setattr(cli, "usb_ports", list)
    assert main(["--twin-p", "32", "--json", "--export-mujoco", str(tmp_path / "t")]) == 0
    d = json.loads(capsys.readouterr().out)["twin"]
    assert d["p"] == 32 and d["p_source"] == "option" and d["checked"] is False and exported(tmp_path / "t")[0] == 32
    assert main(["--export-mujoco", str(tmp_path / "u")]) == 0
    out = capsys.readouterr().out
    assert "twin for P 16 (LeRobot's default; no arm read, pass --twin-p 32" in out and exported(tmp_path / "u")[0] == 16
    patch_bus(monkeypatch, all_p(32))
    assert main(["--port", "fake", "--twin-p", "16", "--export-mujoco", str(tmp_path / "v")]) == 0
    out = capsys.readouterr().out
    assert "servo 1 (shoulder_pan): P is 32, the twin assumes 16" in out and "Mixed" not in out
    assert exported(tmp_path / "v")[0] == 16


def test_folder_holding_the_other_variant_is_refused(monkeypatch, capsys, tmp_path):
    d = tmp_path / "t"
    assert main(["--demo", "--export-mujoco", str(d)]) == 0
    capsys.readouterr()
    assert main(["--demo", "--twin-p", "32", "--export-mujoco", str(d)]) == 2
    assert "holds the twin for P 16, not P 32: pass a new folder" in capsys.readouterr().err
    patch_bus(monkeypatch, all_p(32))  # chosen only after the scan: still refused, nothing overwritten
    assert main(["--port", "fake", "--export-mujoco", str(d)]) == 2
    assert "holds the twin for P 16, not P 32" in capsys.readouterr().err and exported(d)[0] == 16


def test_twin_check_in_a_scan_and_in_json(monkeypatch, capsys, tmp_path):
    s = stock()
    s[1]["p_coefficient"] = 20
    del s[5]
    patch_bus(monkeypatch, s)
    assert main(["--port", "fake", "--export-mujoco", str(tmp_path / "t")]) == 0
    out = capsys.readouterr().out
    assert "servo 1 (shoulder_pan): P is 20" in out and "servo 5: not read, not checked" in out
    patch_bus(monkeypatch, stock())
    assert main(["--port", "fake", "--json", "--export-mujoco", str(tmp_path / "t2")]) == 0
    d = json.loads(capsys.readouterr().out, parse_constant=lambda c: pytest.fail(c))
    assert d["twin"]["checked"] and d["twin"]["exported"] == str(tmp_path / "t2") and d["twin"]["mismatches"] == []
    assert d["twin"]["match"] == {str(i): True for i in range(1, 6)} and d["twin"]["assumes"]["p"] == 16
    main(["--demo", "--json"])
    assert "twin" not in json.loads(capsys.readouterr().out)


def test_export_without_an_arm(monkeypatch, capsys, tmp_path):
    monkeypatch.setattr(cli, "usb_ports", list)
    assert main(["--export-mujoco", str(tmp_path / "t")]) == 0
    out = capsys.readouterr().out
    assert "arm not checked (No USB serial port found" in out and (tmp_path / "t" / "scene.xml").exists()
    assert "so101-doctor:" not in out
    assert main(["--json", "--export-mujoco", str(tmp_path / "t2")]) == 0
    d = json.loads(capsys.readouterr().out)
    assert d["twin"]["checked"] is False and "No USB serial port" in d["twin"]["reason"] and d["servos"] == {}
    with pytest.raises(SystemExit):  # without --export-mujoco a missing port still stops the run
        main([])
    patch_bus(monkeypatch, {})
    assert main(["--port", "fake", "--export-mujoco", str(tmp_path / "t3")]) == 1
    assert "arm not checked (no servo answered)" in capsys.readouterr().out


def test_twin_loads_and_steps_in_mujoco(tmp_path):
    mujoco = pytest.importorskip("mujoco")
    for p in (16, 32):
        main(["--demo", "--twin-p", str(p), "--export-mujoco", str(tmp_path / f"t{p}")])
        m = mujoco.MjModel.from_xml_path(str(tmp_path / f"t{p}" / "scene.xml"))
        d = mujoco.MjData(m)
        for _ in range(200):
            mujoco.mj_step(m, d)
        assert m.nu == 6 and all(abs(x) < 10 for x in d.qpos)


def test_twin_p_is_read_from_a_uniform_arm_only():
    def arm(ps):
        return {i: {"registers": {"p_coefficient": v}} for i, v in zip(range(1, 6), ps)}

    assert twin.pick(arm([16] * 5)) == (16, "arm") and twin.pick(arm([32] * 5)) == (32, "arm")
    assert twin.pick(arm([16, 32, 16, 16, 16])) == (16, "default") and twin.pick(arm([24] * 5)) == (16, "default")
    assert twin.pick({}) == (16, "default") and twin.pick(arm([16] * 5), 32) == (32, "option")
