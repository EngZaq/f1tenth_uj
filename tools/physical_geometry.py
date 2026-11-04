"""Shared physical geometry for offline planning only."""

from pathlib import Path
import math
import yaml

CONFIG = (
    Path(__file__).resolve().parents[1]
    / "src/f1tenth_bringup/config/physical_vehicle.yaml"
)

data = yaml.safe_load(CONFIG.read_text())
vehicle = data["vehicle"]

WHEELBASE = float(vehicle["wheelbase_m"])
BODY_FRONT = float(vehicle["body_front_x_m"])
BODY_REAR = float(vehicle["body_rear_x_m"])
BODY_HALFWIDTH = float(vehicle["body_half_width_m"])

LASER = {
    "rear_axle_to_body_center_x_m":
        (BODY_FRONT + BODY_REAR) / 2
}

values = (WHEELBASE, BODY_FRONT, BODY_REAR, BODY_HALFWIDTH)

if not all(math.isfinite(v) for v in values):
    raise ValueError("Nonfinite physical geometry")

if not (
    WHEELBASE > 0
    and BODY_FRONT > BODY_REAR
    and BODY_HALFWIDTH > 0
):
    raise ValueError("Invalid physical vehicle dimensions")
