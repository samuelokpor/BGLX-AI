#!/usr/bin/env python3

import math


LOCATIONS = {
    'HOME': (
        0.000,
        0.000,
    ),

    'PICKUP_A': (
        5.514,
        0.098,
    ),

    'DELIVERY_A': (
        4.969,
        5.492,
    ),
}


def get_location(name):

    if name not in LOCATIONS:
        available = ', '.join(
            sorted(LOCATIONS.keys())
        )

        raise ValueError(
            f"Unknown mission location '{name}'. "
            f"Available: {available}"
        )

    return LOCATIONS[name]


def arrival_pose(
    source_name,
    target_name
):

    sx, sy = get_location(
        source_name
    )

    tx, ty = get_location(
        target_name
    )

    yaw = math.atan2(
        ty - sy,
        tx - sx
    )

    return (
        tx,
        ty,
        yaw,
    )


def build_delivery_route(
    home_name,
    pickup_name,
    delivery_name
):

    return {
        'pickup': arrival_pose(
            home_name,
            pickup_name
        ),

        'delivery': arrival_pose(
            pickup_name,
            delivery_name
        ),

        'home': arrival_pose(
            delivery_name,
            home_name
        ),
    }
