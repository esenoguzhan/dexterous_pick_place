#!/usr/bin/env python3
"""Simulation-only, contact-verified NEURA IK pick and place."""

from lara5_sim_approach.approach import main
from lara5_sim_approach.pick_place import PickPlace


if __name__ == "__main__":
    main(PickPlace)
