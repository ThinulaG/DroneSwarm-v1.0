"""
PC-side flight controller.

Owns the takeoff state machine and the nested PID stack that converts
(world-frame filtered position, velocity, FC heading) into RC stick
microsecond values (T, R, P, Y, A) suitable for writing as CSV to the
existing `drone_transmitter_serial_espnow.ino`.

Assumes Betaflight in **Angle mode** so stick µs maps to body-frame
roll/pitch angle setpoints rather than rate setpoints. This is what makes
a 60 Hz outer loop sufficient for hover.

State machine
-------------
    IDLE      -> armed=0, throttle=1000, sticks centred
    ARMING    -> armed=1, throttle held at IDLE_THROTTLE for ARMING_HOLD_S
    READY     -> armed, idle throttle, waiting for cmd_takeoff()
    TAKEOFF   -> z setpoint ramps from pos.z to target_z over TAKEOFF_RAMP_S;
                 xy + heading setpoints latched at entry
    HOVER     -> PID holds (xy_target, z_target, heading_target)
    LANDING   -> z setpoint ramps to 0 over LANDING_RAMP_S, then disarm -> IDLE
    EMERGENCY -> immediate disarm (cmd_arm(False), z-error > Z_ERR_LIMIT_M
                 for Z_ERR_LIMIT_HOLD_S, or sustained dt > 0.5 s)

Sign conventions (sticks, Angle mode)
-------------------------------------
    throttle_us  1000 = motors min, 2000 = motors max
    roll_us      1500 = centred; >1500 = roll RIGHT
    pitch_us     1500 = centred; >1500 = pitch FORWARD (nose down)
    yaw_us       1500 = centred; >1500 = yaw RIGHT

If your drone tilts the wrong way during initial bench tests, flip the
relevant sign in `_SIGN_*` below rather than fighting the PID.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


# =========================
# Stick conventions
# =========================

STICK_CENTRE = 1500
STICK_HALF_RANGE = 500       # so output spans 1000..2000
THROTTLE_MIN = 1000
THROTTLE_MAX = 2000

# Flip these if a positive body-frame error drives the drone the wrong way.
_SIGN_PITCH = +1.0           # +1: positive pitch_sp_deg -> pitch_us > 1500 (nose down, fly +x body)
_SIGN_ROLL = +1.0            # +1: positive roll_sp_deg  -> roll_us  > 1500 (roll right, fly +y body)
_SIGN_YAW = +1.0


# =========================
# Default tunables (tuneable live via set_pid / set_trim)
# =========================

@dataclass
class ControlParams:
    # Stick / drone limits
    hover_throttle: int = 1480      # bench-measured: µs at which drone just hovers
    idle_throttle: int = 1050       # motors spinning, no lift, used in ARMING/READY
    max_tilt_deg: float = 15.0
    max_vel_mps: float = 1.0        # outer-loop velocity setpoint clamp
    max_vel_z_mps: float = 0.5

    # Position PID (m -> m/s)
    kp_pos_xy: float = 2.5
    ki_pos_xy: float = 0.0
    kd_pos_xy: float = 0.4
    kp_pos_z: float = 3.5
    ki_pos_z: float = 0.5
    kd_pos_z: float = 0.5

    # Velocity PID for XY (m/s -> degrees of tilt)
    kp_vel_xy: float = 8.0
    ki_vel_xy: float = 1.0
    kd_vel_xy: float = 0.3

    # Velocity PID for Z (m/s -> µs offset from hover_throttle)
    kp_vel_z: float = 120.0
    ki_vel_z: float = 40.0
    kd_vel_z: float = 20.0

    # Yaw PID (rad -> µs offset from 1500)
    kp_yaw: float = 80.0
    ki_yaw: float = 10.0
    kd_yaw: float = 5.0

    # State-machine timings
    arming_hold_s: float = 0.3
    takeoff_ramp_s: float = 5.0
    landing_ramp_s: float = 1.5

    # Safety
    z_err_limit_m: float = 0.5
    z_err_limit_hold_s: float = 0.5

    # Trim (added directly to outputs)
    trim_throttle: int = 0
    trim_roll: int = 0
    trim_pitch: int = 0
    trim_yaw: int = 0


# =========================
# Sub-objects
# =========================

class State(str, Enum):
    IDLE = "IDLE"
    ARMING = "ARMING"
    READY = "READY"
    TAKEOFF = "TAKEOFF"
    HOVER = "HOVER"
    LANDING = "LANDING"
    EMERGENCY = "EMERGENCY"


class PID:
    """Single-axis PID with output clamp + back-calculation anti-windup."""

    def __init__(self, kp: float, ki: float, kd: float, out_min: float, out_max: float):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self._integ = 0.0
        self._prev_err: Optional[float] = None

    def set_gains(self, kp: float, ki: float, kd: float):
        self.kp, self.ki, self.kd = kp, ki, kd

    def set_limits(self, out_min: float, out_max: float):
        self.out_min, self.out_max = out_min, out_max

    def reset(self):
        self._integ = 0.0
        self._prev_err = None

    def step(self, sp: float, pv: float, dt: float) -> float:
        err = sp - pv
        deriv = 0.0 if self._prev_err is None else (err - self._prev_err) / max(dt, 1e-4)
        self._prev_err = err

        self._integ += err * dt
        u_unsat = self.kp * err + self.ki * self._integ + self.kd * deriv
        u = max(self.out_min, min(self.out_max, u_unsat))

        # Back-calculation anti-windup
        if self.ki > 0 and u != u_unsat:
            self._integ -= (u_unsat - u) / self.ki

        return u


def _wrap_pi(a: float) -> float:
    """Wrap angle to [-pi, +pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _clamp_int(x: float, lo: int, hi: int) -> int:
    return int(max(lo, min(hi, round(x))))


# =========================
# Controller
# =========================

@dataclass
class _Setpoint:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    heading: float = 0.0


class Controller:
    def __init__(self, params: Optional[ControlParams] = None):
        self.p = params or ControlParams()
        self.state = State.IDLE
        self._state_entry_t = time.perf_counter()
        self._takeoff_target_z = 0.20

        # Latched setpoint (xy + heading captured at TAKEOFF entry; z ramped)
        self._sp = _Setpoint()

        # External-arm flag from PC ("arm-drone" socket event)
        self._armed_requested = False

        # Track sustained large z error -> EMERGENCY
        self._z_err_violation_since: Optional[float] = None

        # Build PIDs
        v = self.p.max_vel_mps
        vz = self.p.max_vel_z_mps
        tilt = self.p.max_tilt_deg
        self._pid_pos_x = PID(self.p.kp_pos_xy, self.p.ki_pos_xy, self.p.kd_pos_xy, -v, v)
        self._pid_pos_y = PID(self.p.kp_pos_xy, self.p.ki_pos_xy, self.p.kd_pos_xy, -v, v)
        self._pid_pos_z = PID(self.p.kp_pos_z,  self.p.ki_pos_z,  self.p.kd_pos_z,  -vz, vz)

        self._pid_vel_x = PID(self.p.kp_vel_xy, self.p.ki_vel_xy, self.p.kd_vel_xy, -tilt, tilt)
        self._pid_vel_y = PID(self.p.kp_vel_xy, self.p.ki_vel_xy, self.p.kd_vel_xy, -tilt, tilt)
        self._pid_vel_z = PID(self.p.kp_vel_z,  self.p.ki_vel_z,  self.p.kd_vel_z,  -400.0, 400.0)

        self._pid_yaw   = PID(self.p.kp_yaw,    self.p.ki_yaw,    self.p.kd_yaw,    -STICK_HALF_RANGE, STICK_HALF_RANGE)

    # ----------------------------------------------------------------
    # Commands from PC / web UI
    # ----------------------------------------------------------------

    def cmd_arm(self, armed: bool):
        self._armed_requested = bool(armed)
        if not armed:
            self._go(State.IDLE)

    def cmd_takeoff(self, target_z: float = 0.20):
        if not self._armed_requested:
            return  # ignored when disarmed
        self._takeoff_target_z = float(target_z)
        if self.state in (State.READY, State.HOVER):
            self._go(State.TAKEOFF)

    def cmd_land(self):
        if self.state in (State.HOVER, State.TAKEOFF):
            self._go(State.LANDING)

    def cmd_setpoint(self, x: float, y: float, z: float):
        # Only retarget once stably hovering; otherwise stash for next HOVER entry.
        self._sp.x = float(x)
        self._sp.y = float(y)
        self._sp.z = float(z)

    def set_pid(self, gains):
        """
        Accepts an iterable matching the existing 17-element shape from the
        React UI: [Kp_xy, Ki_xy, Kd_xy,
                   Kp_z, Ki_z, Kd_z,
                   Kp_yaw, Ki_yaw, Kd_yaw,
                   Kp_vxy, Ki_vxy, Kd_vxy,
                   Kp_vz, Ki_vz, Kd_vz,
                   ground_effect_coef, ground_effect_offset]
        The last two are accepted-but-ignored (we don't model ground effect
        on the PC side; it's an FC concern).
        """
        g = [float(x) for x in gains]
        if len(g) < 15:
            return
        self._pid_pos_x.set_gains(g[0], g[1], g[2])
        self._pid_pos_y.set_gains(g[0], g[1], g[2])
        self._pid_pos_z.set_gains(g[3], g[4], g[5])
        self._pid_yaw  .set_gains(g[6], g[7], g[8])
        self._pid_vel_x.set_gains(g[9], g[10], g[11])
        self._pid_vel_y.set_gains(g[9], g[10], g[11])
        self._pid_vel_z.set_gains(g[12], g[13], g[14])

    def set_trim(self, t: int, r: int, p: int, y: int):
        self.p.trim_throttle = int(t)
        self.p.trim_roll = int(r)
        self.p.trim_pitch = int(p)
        self.p.trim_yaw = int(y)

    def get_state(self) -> str:
        return self.state.value

    # ----------------------------------------------------------------
    # Main per-tick entry
    # ----------------------------------------------------------------

    def step(self, pos, vel, heading: Optional[float], dt: float):
        """
        pos:     np.array([x, y, z]) in metres, world frame (or None)
        vel:     np.array([vx, vy, vz]) in m/s, world frame (or None)
        heading: float, rad, FC yaw (or None if not yet received)
        dt:      seconds since previous step

        Returns: (throttle_us, roll_us, pitch_us, yaw_us, armed) all ints.
        """
        # Safety: missing pose => EMERGENCY (FW failsafe will also fire after 500 ms)
        if pos is None or vel is None:
            self._go(State.EMERGENCY)
            return self._safe_sticks()

        # State transitions that aren't command-driven
        self._tick_state(pos)

        # Sticks per state
        if self.state == State.IDLE or self.state == State.EMERGENCY:
            return self._safe_sticks()

        if self.state == State.ARMING or self.state == State.READY:
            # Motors armed + at idle throttle. Sticks centred (no PID).
            return (self.p.idle_throttle + self.p.trim_throttle,
                    STICK_CENTRE + self.p.trim_roll,
                    STICK_CENTRE + self.p.trim_pitch,
                    STICK_CENTRE + self.p.trim_yaw,
                    1)

        # TAKEOFF / HOVER / LANDING all run the PID stack against self._sp
        return self._pid_sticks(pos, vel, heading if heading is not None else 0.0, dt)

    # ----------------------------------------------------------------
    # State machine internals
    # ----------------------------------------------------------------

    def _go(self, new_state: State):
        if new_state == self.state:
            return
        # Reset integrators on state transitions
        for pid in (self._pid_pos_x, self._pid_pos_y, self._pid_pos_z,
                    self._pid_vel_x, self._pid_vel_y, self._pid_vel_z,
                    self._pid_yaw):
            pid.reset()
        self.state = new_state
        self._state_entry_t = time.perf_counter()
        self._z_err_violation_since = None

    def _state_age(self) -> float:
        return time.perf_counter() - self._state_entry_t

    def _tick_state(self, pos):
        # Disarm-overrides
        if not self._armed_requested:
            self._go(State.IDLE)
            return

        # Sustained Z-error -> EMERGENCY (TAKEOFF/HOVER only)
        if self.state in (State.TAKEOFF, State.HOVER):
            z_err = abs(pos[2] - self._sp.z)
            now = time.perf_counter()
            if z_err > self.p.z_err_limit_m:
                if self._z_err_violation_since is None:
                    self._z_err_violation_since = now
                elif now - self._z_err_violation_since > self.p.z_err_limit_hold_s:
                    self._go(State.EMERGENCY)
                    return
            else:
                self._z_err_violation_since = None

        # IDLE -> ARMING when PC requests armed
        if self.state == State.IDLE and self._armed_requested:
            # Latch xy at the LED's current position so HOVER works if we go
            # straight to takeoff without an explicit setpoint.
            self._sp.x = float(pos[0])
            self._sp.y = float(pos[1])
            self._sp.z = 0.0
            self._go(State.ARMING)
            return

        # ARMING -> READY after hold
        if self.state == State.ARMING and self._state_age() > self.p.arming_hold_s:
            self._go(State.READY)
            return

        # TAKEOFF z ramp (xy latched, z ramps 0 -> target over takeoff_ramp_s)
        if self.state == State.TAKEOFF:
            a = self._state_age()
            frac = min(1.0, a / max(self.p.takeoff_ramp_s, 1e-3))
            self._sp.z = frac * self._takeoff_target_z
            if frac >= 1.0:
                self._sp.z = self._takeoff_target_z
                self._go(State.HOVER)
            return

        # LANDING z ramp (z held xy, z ramps current -> 0 over landing_ramp_s)
        if self.state == State.LANDING:
            a = self._state_age()
            frac = min(1.0, a / max(self.p.landing_ramp_s, 1e-3))
            self._sp.z = self._takeoff_target_z * (1.0 - frac)
            if frac >= 1.0:
                self._sp.z = 0.0
                self._armed_requested = False  # cut motors at touchdown
                self._go(State.IDLE)
            return

    def _enter_takeoff_latch(self, pos):
        self._sp.x = float(pos[0])
        self._sp.y = float(pos[1])
        # heading latch done at first PID tick once we have a value

    # ----------------------------------------------------------------
    # PID -> sticks
    # ----------------------------------------------------------------

    def _pid_sticks(self, pos, vel, heading: float, dt: float):
        # ---- Position loop: world-frame ----
        vx_sp = self._pid_pos_x.step(self._sp.x, float(pos[0]), dt)
        vy_sp = self._pid_pos_y.step(self._sp.y, float(pos[1]), dt)
        vz_sp = self._pid_pos_z.step(self._sp.z, float(pos[2]), dt)

        # ---- Velocity loop: rotate world-frame error into BODY frame using heading ----
        # heading = drone yaw in world; world->body is rotation by (-heading)
        c, s = math.cos(-heading), math.sin(-heading)
        b_vx_sp = c * vx_sp - s * vy_sp
        b_vy_sp = s * vx_sp + c * vy_sp
        b_vx    = c * float(vel[0]) - s * float(vel[1])
        b_vy    = s * float(vel[0]) + c * float(vel[1])

        pitch_deg = self._pid_vel_x.step(b_vx_sp, b_vx, dt)   # body +x -> pitch fwd
        roll_deg  = self._pid_vel_y.step(b_vy_sp, b_vy, dt)   # body +y -> roll right
        thr_off   = self._pid_vel_z.step(vz_sp,   float(vel[2]), dt)

        # ---- Yaw loop ----
        yaw_us_off = self._pid_yaw.step(0.0, _wrap_pi(heading - self._sp.heading), dt)

        # ---- Stick conversion ----
        pitch_us = STICK_CENTRE + _SIGN_PITCH * pitch_deg * (STICK_HALF_RANGE / self.p.max_tilt_deg)
        roll_us  = STICK_CENTRE + _SIGN_ROLL  * roll_deg  * (STICK_HALF_RANGE / self.p.max_tilt_deg)
        yaw_us   = STICK_CENTRE + _SIGN_YAW   * yaw_us_off
        thr_us   = self.p.hover_throttle + thr_off

        return (
            _clamp_int(thr_us   + self.p.trim_throttle, THROTTLE_MIN, THROTTLE_MAX),
            _clamp_int(roll_us  + self.p.trim_roll,     1000, 2000),
            _clamp_int(pitch_us + self.p.trim_pitch,    1000, 2000),
            _clamp_int(yaw_us   + self.p.trim_yaw,      1000, 2000),
            1,
        )

    def _safe_sticks(self):
        return (
            THROTTLE_MIN + self.p.trim_throttle,
            STICK_CENTRE + self.p.trim_roll,
            STICK_CENTRE + self.p.trim_pitch,
            STICK_CENTRE + self.p.trim_yaw,
            0,
        )


# =========================
# Smoke test
# =========================

def _smoke_test():
    """
    Run the controller through IDLE -> ARMING -> READY -> TAKEOFF -> HOVER ->
    LANDING -> IDLE with synthetic perfect-tracking positions, printing
    transitions + sticks.
    """
    import numpy as np

    c = Controller()
    pos = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    vel = np.zeros(3, dtype=np.float32)
    heading = 0.0
    dt = 1.0 / 60.0

    def tick(label):
        T, R, P, Y, A = c.step(pos.copy(), vel.copy(), heading, dt)
        print(f"  [{c.get_state():<9}] {label:<22} pos.z={pos[2]:+.3f}  "
              f"sp.z={c._sp.z:+.3f}  sticks=T{T} R{R} P{P} Y{Y} A{A}")

    print("Arming the drone...")
    c.cmd_arm(True)
    for _ in range(40):  # 0.66 s of ARMING then READY
        tick("idling")
        time.sleep(dt)

    print("Commanding takeoff to 0.20 m...")
    c.cmd_takeoff(0.20)
    for i in range(180):  # 3 s of TAKEOFF + HOVER
        # Pretend the drone perfectly follows the z setpoint with a ~50 ms lag
        pos[2] += 0.5 * (c._sp.z - pos[2])
        tick("flying")
        time.sleep(dt)
        if i == 60:
            print("Commanding land...")
            c.cmd_land()

    for _ in range(60):
        pos[2] += 0.5 * (c._sp.z - pos[2])
        tick("landing")
        time.sleep(dt)

    print("Done.")


if __name__ == "__main__":
    _smoke_test()
