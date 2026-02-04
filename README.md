# Screws Tilt Touch (Cartographer v3 + Klipper)

This repo contains a Klipper `extras/` module and an example config that provide a `SCREWS_TILT_CALCULATE`-like workflow **using Cartographer TOUCH**.

It was created to work around situations where Klipper’s builtin `[screws_tilt_adjust]` / `SCREWS_TILT_CALCULATE` is not usable together with Cartographer v3.

## What you get

- **Klipper module**: `klippy/extras/screws_tilt_touch.py`
  - Registers the command: **`SCREWS_TILT_TOUCH_CALCULATE`**
  - Probes multiple screw points using Cartographer **TOUCH** (nozzle contact).
  - Fits a plane to probed points and prints screw adjustment guidance (turns/minutes).
  - Routes moves via a configurable waypoint to avoid a nozzle-wipe zone.
  - Has safety/recovery for the common Cartographer error: **`Probe triggered prior to movement`**.
  - Uses safe bounds derived from `[bed_mesh]` + Cartographer offsets, with optional per-axis clamp control.

- **Example config**: `config/screws-tilt-adjust.cfg`
  - A ready-to-include `[screws_tilt_touch]` section.
  - A convenience macro `SCREWS_CALIBRATION`.

## Install

### 1) Copy the module into Klipper

Copy:

- `klippy/extras/screws_tilt_touch.py` → `~/klipper/klippy/extras/screws_tilt_touch.py`

Then restart the Klipper **service** (not just `RESTART`), so Python modules reload.

### 2) Add the config

Copy:

- `config/screws-tilt-adjust.cfg` → `~/printer_data/config/config_packages/macro/screws-tilt-adjust.cfg`

And include it from your `printer.cfg`:

```ini
[include config_packages/macro/screws-tilt-adjust.cfg]
```

Restart Klipper.

## Usage

### Basic

```gcode
SCREWS_TILT_TOUCH_CALCULATE
```

### Common parameters

- `BASE=AVERAGE|SCREW1`
- `TOL=0.02`
- `SPEED=150`
- `Z=10`
- `SAMPLES=1` (recommended default)
- `CLAMP_X=1|0` (default 1)
- `CLAMP_Y=1|0` (default 1)

Example:

```gcode
SCREWS_TILT_TOUCH_CALCULATE BASE=AVERAGE TOL=0.02 SPEED=150 Z=10 SAMPLES=1
SCREWS_TILT_TOUCH_CALCULATE BASE=AVERAGE TOL=0.02 SPEED=150 Z=10 SAMPLES=1 CLAMP_Y=0
```

### Convenience macro

```gcode
SCREWS_CALIBRATION
```

## Configuration reference (`[screws_tilt_touch]`)

### Screw points

`screwN` are **nozzle/toolhead XY** positions of the screw center (normal Klipper X/Y).

Cartographer coil XY is:

`coil_xy = nozzle_xy + (x_offset, y_offset)`

The module probes at the **nozzle screw coordinates** by default. If a screw is outside
the safe bounds (derived from `[bed_mesh]` + offsets), it clamps **only the necessary
axis** (X and/or Y) to the nearest safe point.

### Move routing (avoid nozzle-wipe zones)

- `travel_via: center|none|custom`
- `travel_via_x`, `travel_via_y` (if `custom`)

### Touch bounds and clamping

The module computes **safe nozzle bounds** from `[bed_mesh]` and Cartographer offsets,
then applies them to Cartographer’s touch boundaries for the duration of the command.

You can disable clamping per axis:

- `CLAMP_X=0` to allow X outside safe bounds
- `CLAMP_Y=0` to allow Y outside safe bounds

### “Probe triggered prior to movement” recovery

- `pretrigger_retries` (default 2)
- `pretrigger_lift` (default 5.0mm)
- `pretrigger_dwell_ms` (default 200)
- `post_probe_lift` (default 2.0mm)
- `post_probe_dwell_ms` (default 100)

## Moonraker update manager

Once this repo is cloned on your printer, you can track updates via Moonraker with an entry like:

```ini
[update_manager screws_tilt_touch]
type: git_repo
path: /home/kiroru/klipper-screws-tilt-touch
origin: https://github.com/<you>/<repo>.git
primary_branch: main
managed_services: klipper
install_script: install.sh
```

## License

TBD (choose MIT / GPLv3 / etc.)
