"""
Screws tilt helper using Cartographer TOUCH mode.

Why:
 - Cartographer v3 can make the builtin SCREWS_TILT_CALCULATE unusable.
 - Using scan-mode PROBE for screw points can be unsafe if the scan model/range is off.
 - Touch mode provides a bounded, sample-filtered contact measurement.

Config:
  [screws_tilt_touch]
  screw1: X,Y
  screw1_name: ...
  ...
  horizontal_move_z: 10
  speed: 150
  screw_thread: CW-M4
  pitch: 0.70

GCode:
  SCREWS_TILT_TOUCH_CALCULATE [BASE=AVERAGE|SCREW1] [TOL=0.02] [SPEED=150] [Z=10] [SAMPLES=1]
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class ScrewPoint:
    name: str
    label: str
    x: float
    y: float


@dataclass
class _SimpleBounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def is_within(self, *, x: float, y: float) -> bool:
        # Match Cartographer's small epsilon behavior
        eps = 0.01
        return (self.min_x - eps) <= x <= (self.max_x + eps) and (self.min_y - eps) <= y <= (self.max_y + eps)


def _parse_xy(val: str) -> tuple[float, float]:
    parts = [p.strip() for p in val.split(",")]
    if len(parts) != 2:
        raise ValueError(f"Invalid screw coordinate '{val}', expected 'X,Y'")
    return (float(parts[0]), float(parts[1]))


def _format_turns(minutes_total: float) -> str:
    # 1 turn = 60 minutes
    turns = int(minutes_total // 60)
    mins = int(round(minutes_total - (turns * 60)))
    if mins == 60:
        turns += 1
        mins = 0
    return f"{turns} turns + {mins} min"


def _solve_plane(points: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    """
    Fit plane z = a*x + b*y + c to points using normal equations.
    Returns (a, b, c).
    """
    n = float(len(points))
    if n < 3:
        raise ValueError("Need at least 3 points to fit a plane")

    s_x = s_y = s_z = 0.0
    s_xx = s_yy = s_xy = 0.0
    s_xz = s_yz = 0.0
    for x, y, z in points:
        s_x += x
        s_y += y
        s_z += z
        s_xx += x * x
        s_yy += y * y
        s_xy += x * y
        s_xz += x * z
        s_yz += y * z

    # Solve:
    # [s_xx s_xy s_x] [a] = [s_xz]
    # [s_xy s_yy s_y] [b]   [s_yz]
    # [s_x  s_y  n ] [c]   [s_z ]
    A11, A12, A13 = s_xx, s_xy, s_x
    A21, A22, A23 = s_xy, s_yy, s_y
    A31, A32, A33 = s_x, s_y, n
    B1, B2, B3 = s_xz, s_yz, s_z

    det = (A11 * (A22 * A33 - A23 * A32)) - (A12 * (A21 * A33 - A23 * A31)) + (A13 * (A21 * A32 - A22 * A31))
    if abs(det) < 1e-12:
        raise ValueError("Plane fit is singular (points may be collinear)")

    det_a = (B1 * (A22 * A33 - A23 * A32)) - (A12 * (B2 * A33 - A23 * B3)) + (A13 * (B2 * A32 - A22 * B3))
    det_b = (A11 * (B2 * A33 - A23 * B3)) - (B1 * (A21 * A33 - A23 * A31)) + (A13 * (A21 * B3 - B2 * A31))
    det_c = (A11 * (A22 * B3 - B2 * A32)) - (A12 * (A21 * B3 - B2 * A31)) + (B1 * (A21 * A32 - A22 * A31))

    a = det_a / det
    b = det_b / det
    c = det_c / det
    return (a, b, c)


def load_config(config):
    return ScrewsTiltTouch(config)


class ScrewsTiltTouch:
    def __init__(self, config):
        self.printer = config.get_printer()
        # NOTE: At config load time, some core objects (notably toolhead) may not
        # be available yet. Defer lookups until klippy:connect.
        self.gcode = self.printer.lookup_object("gcode")
        self.toolhead = None

        self.horizontal_move_z = config.getfloat("horizontal_move_z", default=10.0, above=0.0)
        self.speed_mm_s = config.getfloat("speed", default=150.0, above=1.0)
        self.screw_thread = config.get("screw_thread", default="CW-M4")
        self.pitch = config.getfloat("pitch", default=0.70, above=0.0)

        # Travel routing to avoid wiping/nozzle-clean area:
        # - travel_via: "none" (direct), "center" (via axis center), "custom" (via travel_via_x/y)
        self.travel_via = (config.get("travel_via", default="center") or "center").lower()
        self.travel_via_x = config.get("travel_via_x", default=None)
        self.travel_via_y = config.get("travel_via_y", default=None)

        # If Cartographer touch triggers immediately ("Probe triggered prior to movement"),
        # do a small recovery lift and retry.
        self.pretrigger_retries = config.getint("pretrigger_retries", default=2, minval=0)
        self.pretrigger_lift = config.getfloat("pretrigger_lift", default=5.0, above=0.0)
        self.pretrigger_dwell_ms = config.getint("pretrigger_dwell_ms", default=200, minval=0)

        # After a successful touch, lift a bit to ensure we are not still "pressed".
        self.post_probe_lift = config.getfloat("post_probe_lift", default=2.0, above=0.0)
        self.post_probe_dwell_ms = config.getint("post_probe_dwell_ms", default=100, minval=0)

        # Optional: temporarily override Cartographer touch boundaries for this command only.
        self.override_touch_bounds = bool(config.getint("override_touch_bounds", default=0, minval=0))
        self.override_touch_bounds_margin = config.getfloat("override_touch_bounds_margin", default=1.0, above=0.0)

        self.screws: list[ScrewPoint] = []
        for i in range(1, 7):
            key = f"screw{i}"
            if not config.get(key, default=None):
                continue
            x, y = _parse_xy(config.get(key))
            label = config.get(f"{key}_name", default=key)
            self.screws.append(ScrewPoint(name=key, label=label, x=x, y=y))

        self.gcode.register_command(
            "SCREWS_TILT_TOUCH_CALCULATE",
            self.cmd_SCREWS_TILT_TOUCH_CALCULATE,
            desc="Auto screws tilt helper using Cartographer TOUCH probe.",
        )
        self.printer.register_event_handler("klippy:connect", self._handle_connect)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")

    def _get_cartographer_touch(self):
        carto = self.printer.lookup_object("cartographer", None)
        if carto is None:
            raise self.printer.command_error("Cartographer object not found (missing [cartographer]?)")
        touch = getattr(carto, "touch_mode", None)
        if touch is None:
            raise self.printer.command_error("Cartographer touch_mode not available")
        return touch

    def _raise_dir(self) -> str:
        s = (self.screw_thread or "").upper()
        return "CCW" if "CCW" in s else "CW"

    def _nozzle_to_probe_xy(self, touch, nx: float, ny: float) -> tuple[float, float]:
        cfg = getattr(touch, "_config")
        x_offset = float(cfg.x_offset)
        y_offset = float(cfg.y_offset)
        return (nx + x_offset, ny + y_offset)

    def _get_travel_via_xy(self) -> Optional[tuple[float, float]]:
        if self.travel_via == "none":
            return None
        if self.travel_via == "custom":
            if self.travel_via_x is None or self.travel_via_y is None:
                raise self.printer.command_error(
                    "travel_via=custom requires travel_via_x and travel_via_y in [screws_tilt_touch]"
                )
            return (float(self.travel_via_x), float(self.travel_via_y))

        assert self.toolhead is not None
        curtime = self.printer.get_reactor().monotonic()
        st = self.toolhead.get_status(curtime)
        amin = st.get("axis_minimum", [0.0, 0.0, 0.0])
        amax = st.get("axis_maximum", [0.0, 0.0, 0.0])
        xmid = (float(amin[0]) + float(amax[0])) / 2.0
        ymid = (float(amin[1]) + float(amax[1])) / 2.0
        return (xmid, ymid)

    def _touch_once(self, touch) -> float:
        fn = getattr(touch, "_perform_single_probe", None)
        if callable(fn):
            return float(fn())
        return float(touch.perform_probe())

    def _probe_at(self, touch, x: float, y: float, z_safe: float, speed: float, boundaries, samples: int):
        if boundaries is not None and not boundaries.is_within(x=x, y=y):
            raise self.printer.command_error(
                f"Target nozzle position ({x:.2f},{y:.2f}) is outside touch boundaries. "
                f"Valid range: X=[{boundaries.min_x:.2f}, {boundaries.max_x:.2f}], "
                f"Y=[{boundaries.min_y:.2f}, {boundaries.max_y:.2f}]. "
                "This means the Cartographer coil would be outside the configured mesh bounds at that nozzle XY. "
                "Pick a different probe point or expand [bed_mesh] mesh_min/mesh_max in cartographer.cfg."
            )

        self._move_xy_safe(x, y, z_safe, speed)

        collected: list[float] = []
        for _i in range(max(1, int(samples))):
            attempt = 0
            while True:
                try:
                    z = float(self._touch_once(touch))
                    if not math.isfinite(z):
                        raise self.printer.command_error("Touch probe returned non-finite result")
                    collected.append(z)
                    break
                except Exception as e:
                    msg = str(e)
                    if "Probe triggered prior to movement" in msg and attempt < self.pretrigger_retries:
                        attempt += 1
                        self.gcode.respond_info(
                            f"SCREWS_TILT_TOUCH: pre-trigger at ({x:.2f},{y:.2f}); "
                            f"lifting Z by {self.pretrigger_lift:.2f}mm, dwell {self.pretrigger_dwell_ms}ms, "
                            f"retry ({attempt}/{self.pretrigger_retries})"
                        )
                        fz = 5.0 * 60.0
                        dwell_line = f"G4 P{int(self.pretrigger_dwell_ms)}\n" if self.pretrigger_dwell_ms else ""
                        script = (
                            "G91\n"
                            f"G1 Z{self.pretrigger_lift:.3f} F{fz:.0f}\n"
                            "G90\n"
                            f"{dwell_line}"
                            "M400\n"
                        )
                        self.gcode.run_script_from_command(script)
                        continue
                    raise

        z_out = float(sorted(collected)[len(collected) // 2])

        if self.post_probe_lift > 0.0:
            fz = 5.0 * 60.0
            dwell_line = f"G4 P{int(self.post_probe_dwell_ms)}\n" if self.post_probe_dwell_ms else ""
            script = (
                "G91\n"
                f"G1 Z{self.post_probe_lift:.3f} F{fz:.0f}\n"
                "G90\n"
                f"{dwell_line}"
                "M400\n"
            )
            self.gcode.run_script_from_command(script)
        return z_out

    def _move_xy_safe(self, x: float, y: float, z: float, speed_mm_s: float):
        assert self.toolhead is not None
        pos = self.toolhead.get_position()
        cur_z = float(pos[2])
        z_target = max(cur_z, float(z))
        f_xy = float(speed_mm_s) * 60.0
        f_z = 5.0 * 60.0
        via = self._get_travel_via_xy()
        script = "G90\n" f"G1 Z{z_target:.3f} F{f_z:.0f}\n"
        if via is not None:
            vx, vy = via
            script += f"G1 X{vx:.3f} Y{vy:.3f} F{f_xy:.0f}\n"
        script += f"G1 X{x:.3f} Y{y:.3f} F{f_xy:.0f}\n"
        script += "M400\n"
        self.gcode.run_script_from_command(script)

    def cmd_SCREWS_TILT_TOUCH_CALCULATE(self, gcmd):
        if self.toolhead is None:
            raise self.printer.command_error("Not connected yet - try again after RESTART")
        if len(self.screws) < 3:
            raise self.printer.command_error("Need at least 3 screws defined in [screws_tilt_touch]")

        base = gcmd.get("BASE", "AVERAGE").upper()
        tol = gcmd.get_float("TOL", 0.02, minval=0.0)
        speed = gcmd.get_float("SPEED", self.speed_mm_s, above=1.0)
        z_safe = gcmd.get_float("Z", self.horizontal_move_z, above=0.0)
        samples = gcmd.get_int("SAMPLES", 1, minval=1)

        curtime = self.printer.get_reactor().monotonic()
        homed_axes = (self.toolhead.get_status(curtime).get("homed_axes") or "")
        if not all(a in homed_axes for a in ("x", "y", "z")):
            raise self.printer.command_error(
                f"Home XYZ before running SCREWS_TILT_TOUCH_CALCULATE (homed_axes='{homed_axes}')"
            )

        touch = self._get_cartographer_touch()
        original_boundaries = getattr(touch, "boundaries", None)
        boundaries = original_boundaries

        try:
            max_temp = touch._config.max_touch_temperature  # type: ignore[attr-defined]
        except Exception:
            max_temp = None
        extr = self.printer.lookup_object("extruder", None)
        if max_temp is not None and extr is not None:
            temp = extr.get_status(self.printer.get_reactor().monotonic())["temperature"]
            if temp > max_temp + 2:
                raise self.printer.command_error(
                    f"Nozzle too hot for touch probing ({temp:.1f}C > {max_temp}C). Cool down first."
                )

        self.gcode.respond_info(
            f"SCREWS_TILT_TOUCH: probing {len(self.screws)} screws with Cartographer TOUCH (pitch {self.pitch}mm/turn, thread {self.screw_thread})"
        )
        self.gcode.respond_info(
            f"SCREWS_TILT_TOUCH: pretrigger_retries={self.pretrigger_retries} pretrigger_lift={self.pretrigger_lift:.2f}mm pretrigger_dwell_ms={self.pretrigger_dwell_ms}"
        )
        self.gcode.respond_info(
            f"SCREWS_TILT_TOUCH: post_probe_lift={self.post_probe_lift:.2f}mm post_probe_dwell_ms={self.post_probe_dwell_ms}"
        )
        self.gcode.respond_info(f"SCREWS_TILT_TOUCH: samples_per_point={samples}")
        if boundaries is not None:
            self.gcode.respond_info(
                f"SCREWS_TILT_TOUCH: touch bounds X=[{boundaries.min_x:.2f},{boundaries.max_x:.2f}] Y=[{boundaries.min_y:.2f},{boundaries.max_y:.2f}]"
            )

        cfg = getattr(touch, "_config")
        x_offset = float(cfg.x_offset)
        y_offset = float(cfg.y_offset)

        probe_plan: list[tuple[ScrewPoint, float, float, float, float]] = []
        for sp in self.screws:
            screw_nx, screw_ny = float(sp.x), float(sp.y)
            probe_nx = screw_nx - x_offset
            probe_ny = screw_ny - y_offset
            probe_plan.append((sp, screw_nx, screw_ny, probe_nx, probe_ny))

        restored = False
        try:
            if self.override_touch_bounds and original_boundaries is not None:
                needs = any(
                    not original_boundaries.is_within(x=pnx, y=pny) for (_sp, _sx, _sy, pnx, pny) in probe_plan
                )
                if needs:
                    curtime = self.printer.get_reactor().monotonic()
                    st = self.toolhead.get_status(curtime)
                    amin = st.get("axis_minimum", [0.0, 0.0, 0.0])
                    amax = st.get("axis_maximum", [0.0, 0.0, 0.0])

                    xs = [pnx for (_sp, _sx, _sy, pnx, _pny) in probe_plan]
                    ys = [pny for (_sp, _sx, _sy, _pnx, pny) in probe_plan]
                    m = float(self.override_touch_bounds_margin)
                    min_x = max(float(amin[0]), min(xs) - m)
                    max_x = min(float(amax[0]), max(xs) + m)
                    min_y = max(float(amin[1]), min(ys) - m)
                    max_y = min(float(amax[1]), max(ys) + m)

                    touch.boundaries = _SimpleBounds(min_x=min_x, max_x=max_x, min_y=min_y, max_y=max_y)
                    boundaries = touch.boundaries
                    restored = True
                    self.gcode.respond_info(
                        f"SCREWS_TILT_TOUCH: overriding touch bounds for this command to "
                        f"X=[{min_x:.2f},{max_x:.2f}] Y=[{min_y:.2f},{max_y:.2f}] (margin {m:.2f})"
                    )

            probed_points: list[tuple[float, float, float]] = []
            for (sp, screw_nx, screw_ny, probe_nx, probe_ny) in probe_plan:
                coil_x, coil_y = self._nozzle_to_probe_xy(touch, probe_nx, probe_ny)
                self.gcode.respond_info(
                    f"{sp.label}: screw_nozzle=({screw_nx:.2f},{screw_ny:.2f}) "
                    f"probe_nozzle=({probe_nx:.2f},{probe_ny:.2f}) coil=({coil_x:.2f},{coil_y:.2f})"
                )
                z = self._probe_at(touch, probe_nx, probe_ny, z_safe, speed, boundaries, samples)
                probed_points.append((probe_nx, probe_ny, z))
                self.gcode.respond_info(f"{sp.label}: z@probe= {z:.6f}")
        finally:
            if restored:
                touch.boundaries = original_boundaries

        a, b, c = _solve_plane(probed_points)
        self.gcode.respond_info(f"SCREWS_TILT_TOUCH: fitted plane z = {a:.6e}*x + {b:.6e}*y + {c:.6f}")

        results: list[tuple[ScrewPoint, float]] = []
        for sp in self.screws:
            screw_nx, screw_ny = float(sp.x), float(sp.y)
            z_screw = (a * screw_nx) + (b * screw_ny) + c
            results.append((sp, z_screw))
            self.gcode.respond_info(f"{sp.label}: z@screw= {z_screw:.6f}")

        if base == "SCREW1":
            z_ref: Optional[float] = None
            for sp, z in results:
                if sp.name == "screw1":
                    z_ref = z
                    break
            if z_ref is None:
                raise self.printer.command_error("BASE=SCREW1 but screw1 is not defined")
        else:
            z_ref = sum(z for _sp, z in results) / len(results)

        raise_dir = self._raise_dir()
        self.gcode.respond_info(f"Reference z={z_ref:.6f} (BASE={base})")

        for sp, z in results:
            delta = z - z_ref
            adelta = abs(delta)
            if adelta <= tol:
                self.gcode.respond_info(f"{sp.label}: OK (Δ={delta:+.4f}mm)")
                continue

            # High point (delta>0) -> lower bed at that screw (opposite of raise_dir)
            if delta > 0:
                direction = "CCW" if raise_dir == "CW" else "CW"
            else:
                direction = raise_dir

            turns = adelta / self.pitch
            minutes_total = turns * 60.0
            self.gcode.respond_info(f"{sp.label}: Δ={delta:+.4f}mm -> {direction} {_format_turns(minutes_total)}")

