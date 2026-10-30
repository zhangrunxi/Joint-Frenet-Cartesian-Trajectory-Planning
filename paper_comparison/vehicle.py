"""The test vehicle shared by every method, the evaluator and the scenarios.

The manuscript's vehicle is the bus of Table 1. The SUV is a representative
mid-size passenger vehicle, added because the manuscript states that the
method is not specific to buses; its values were fixed before any SUV run.

The vehicle is selected once per process with ``BENCH_VEHICLE`` (``bus`` by
default), so that a run cannot mix geometries between methods.
"""

import os
from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleGeometry:
    name: str
    length: float
    width: float
    wheelbase: float
    rear_overhang: float
    max_steering_deg: float
    max_steering_rate_degps: float

    @property
    def half_width(self):
        return self.width / 2

    @property
    def front_extent(self):
        """Distance from the rear axle to the front bumper."""
        return self.length - self.rear_overhang


BUS = VehicleGeometry("bus", 6.50, 2.30, 4.32, 1.00, 29.32, 24.55)
SUV = VehicleGeometry("suv", 4.80, 1.90, 2.85, 1.00, 35.0, 30.0)

VEHICLES = {"bus": BUS, "suv": SUV}
GEOMETRY = VEHICLES[os.environ.get("BENCH_VEHICLE", "bus")]
