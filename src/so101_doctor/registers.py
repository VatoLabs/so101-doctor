"""STS3215 register map (Feetech STS memory table, the names LeRobot uses) and decoding."""
from __future__ import annotations

# name: (address, bytes)
MAP = {
    "firmware_major_version": (0, 1), "firmware_minor_version": (1, 1), "model_number": (3, 2), "id": (5, 1),
    "baud_rate": (6, 1), "return_delay_time": (7, 1), "min_position_limit": (9, 2), "max_position_limit": (11, 2),
    "max_temperature_limit": (13, 1), "max_voltage_limit": (14, 1), "min_voltage_limit": (15, 1),
    "max_torque_limit": (16, 2), "phase": (18, 1), "unloading_condition": (19, 1), "p_coefficient": (21, 1),
    "d_coefficient": (22, 1), "i_coefficient": (23, 1), "minimum_startup_force": (24, 2), "cw_dead_zone": (26, 1),
    "ccw_dead_zone": (27, 1), "protection_current": (28, 2), "homing_offset": (31, 2), "operating_mode": (33, 1),
    "protective_torque": (34, 1), "protection_time": (35, 1), "overload_torque": (36, 1),
    "over_current_protection_time": (38, 1), "torque_enable": (40, 1), "acceleration": (41, 1),
    "goal_position": (42, 2), "goal_velocity": (46, 2), "torque_limit": (48, 2), "lock": (55, 1),
    "present_position": (56, 2), "present_velocity": (58, 2), "present_load": (60, 2), "present_voltage": (62, 1),
    "present_temperature": (63, 1), "status": (65, 1), "moving": (66, 1), "present_current": (69, 2),
    "maximum_acceleration": (85, 1), "acceleration_multiplier": (86, 1),
}
# sign-magnitude registers and their sign bit (positions: bit 15, as LeRobot decodes them; set only past one turn)
SIGN_BIT = {"homing_offset": 11, "goal_position": 15, "present_position": 15, "goal_velocity": 15,
            "present_velocity": 15, "present_load": 10}
BLOCKS = ((0, 41), (41, 31), (85, 2))  # three reads cover every register above
STS3215 = 777
JOINTS = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex", 4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}


def sign_magnitude(v: int, bit: int) -> int:
    return -(v & ((1 << bit) - 1)) if v >> bit & 1 else v


def decode(mem: dict) -> dict:
    """{address: byte} -> {name: value} (little-endian, sign-magnitude where the servo uses it)."""
    out = {}
    for name, (addr, n) in MAP.items():
        v = mem[addr] if n == 1 else mem[addr] | mem[addr + 1] << 8
        out[name] = sign_magnitude(v, SIGN_BIT[name]) if name in SIGN_BIT else v
    return out


def read_all(bus, servo_id: int):
    """All registers of one servo. Returns (registers, the error bits seen in any of its replies)."""
    mem, err = {}, 0
    for addr, n in BLOCKS:
        e, data = bus.read(servo_id, addr, n)
        err |= e
        mem.update({addr + k: b for k, b in enumerate(data)})
    return decode(mem), err
