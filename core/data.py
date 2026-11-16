"""Loads the shared motor/car presets (also read by the MATLAB side)."""
import json
from pathlib import Path

DATA_DIR = Path(__file__).resolve().parent.parent / "data"


def load_motors():
    with open(DATA_DIR / "motors.json") as f:
        return json.load(f)["motors"]


def load_cars():
    with open(DATA_DIR / "cars.json") as f:
        return json.load(f)["cars"]
