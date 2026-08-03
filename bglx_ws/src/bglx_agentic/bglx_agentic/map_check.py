"""Compare the static map against live sensing.

A saved map is a photograph of one moment. The world moves on: a van that was
parked during mapping drives away, and a van that was not there parks in the
service lane. Both cases are invisible to a planner that trusts the map, and
they fail in opposite directions.

  GHOST OBSTACLE   map says occupied, the scan sees through that cell.
                   Something has LEFT. The planner routes around empty air,
                   or refuses a route that is actually open. Wasteful, safe.

  GHOST FREE SPACE map says clear, the scan reports a hit. Something has
                   ARRIVED. The global planner commits to a corridor that is
                   blocked. On a tricycle with a 0.77m turning radius and no
                   rear sensing, discovering this mid-corridor is how the
                   vehicle gets stranded. This is the dangerous direction.

Nav2 handles neither well. Costmap layers combine by maximum cost, so the
static layer always wins: a ghost obstacle in the saved map is never truly
erased. And the global costmap only clears where the sensor currently looks.

CONFIDENCE COUPLING
A map/scan mismatch has two possible causes, and they demand opposite
responses:
    tight covariance + mismatch -> the WORLD changed. Trust the sensor.
    loose covariance + mismatch -> the ROBOT is lost. Trust neither; the
                                   scan is being compared against the wrong
                                   part of the map.
So localisation confidence is reported alongside, never separately.
"""

import math

# AMCL covariance thresholds, set against a converged baseline of
# x 0.086, y 0.133, yaw 0.022 measured on the Oxford map.
COV_GOOD = 0.5          # m^2, position variance below this is trustworthy
COV_POOR = 2.0          # m^2, above this the pose is not to be trusted
COV_YAW_POOR = 0.25     # rad^2, about 29 degrees of standard deviation

# A single stray ray is noise. This many agreeing rays is a real object.
MIN_RUN = 4

# A scan hit is only "unmapped" if NOTHING occupied lies within this radius of
# it in the static map. Two reasons for the slack: AMCL pose error is a few
# tens of centimetres, which at 0.05m resolution is several cells; and a
# SLAM-built map has soft wall edges carrying intermediate values rather than
# a crisp occupied/free boundary. Without this, every wall in the map reports
# as an unmapped obstacle.
SEARCH_CELLS = 8        # 0.4m at 0.05m/cell
MAPPED_OCCUPIED = 50    # >= this counts as "the map knows something is here"


def _world_to_cell(mx, my, info):
    ox = info.origin.position.x
    oy = info.origin.position.y
    cx = int((mx - ox) / info.resolution)
    cy = int((my - oy) / info.resolution)
    if 0 <= cx < info.width and 0 <= cy < info.height:
        return cx, cy
    return None


def _cell_value(grid, info, cx, cy):
    return grid[cy * info.width + cx]


def confidence_note(cov):
    """Turn an AMCL covariance into a sentence about how lost the robot is."""
    if cov is None:
        return None, "Localisation confidence unknown (no AMCL pose yet)."
    vx, vy, vyaw = cov[0], cov[7], cov[35]
    pos = max(vx, vy)
    sd = math.sqrt(pos)
    yaw_sd = math.degrees(math.sqrt(vyaw)) if vyaw > 0 else 0.0

    if pos > COV_POOR or vyaw > COV_YAW_POOR:
        return False, (
            "LOCALISATION LOST: the pose estimate has spread to +/-%.1fm and "
            "+/-%.0f degrees. Coordinates are NOT reliable. Do not plan long "
            "routes and do not trust any map comparison - a mismatch now most "
            "likely means the robot is looking at the wrong part of the map, "
            "not that the world has changed. Drive slowly toward a known "
            "landmark until confidence recovers." % (sd, yaw_sd))
    if pos > COV_GOOD:
        return None, (
            "Localisation is DEGRADED: +/-%.1fm, +/-%.0f degrees. Treat "
            "coordinates and any map mismatch with caution." % (sd, yaw_sd))
    return True, ("Localisation confident: +/-%.2fm, +/-%.0f degrees."
                  % (sd, yaw_sd))


def compare(scan, grid, info, robot_xy, robot_yaw, cov=None,
            max_range=15.0):
    """Static map vs live scan. Returns a paragraph, not a number.

    scan      LaserScan-like
    grid      flat occupancy list from the static map (-1 unknown, 0..100)
    info      the map's MapMetaData
    robot_xy  (x, y) in the map frame
    robot_yaw radians
    """
    ok, conf = confidence_note(cov)
    if scan is None or grid is None or info is None:
        return conf + " Map comparison unavailable (no map or no scan)."

    rx, ry = robot_xy
    arrived = []      # (bearing_deg, range) where the map said clear
    departed = 0      # rays passing through cells the map calls occupied
    checked = 0

    n = len(scan.ranges)
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        if r > max_range:
            continue
        checked += 1
        ang = scan.angle_min + i * scan.angle_increment + robot_yaw
        hx = rx + r * math.cos(ang)
        hy = ry + r * math.sin(ang)

        cell = _world_to_cell(hx, hy, info)
        if cell is None:
            continue

        # Search a neighbourhood, not one cell. If anything occupied or
        # unknown is nearby, the map already knows about this return.
        cx0, cy0 = cell
        near_known = False
        for dy in range(-SEARCH_CELLS, SEARCH_CELLS + 1):
            for dx in range(-SEARCH_CELLS, SEARCH_CELLS + 1):
                nx, ny = cx0 + dx, cy0 + dy
                if not (0 <= nx < info.width and 0 <= ny < info.height):
                    near_known = True      # map edge: do not claim novelty
                    break
                nv = _cell_value(grid, info, nx, ny)
                if nv < 0 or nv >= MAPPED_OCCUPIED:
                    near_known = True
                    break
            if near_known:
                break

        # A hit with nothing known anywhere near it: something ARRIVED.
        if not near_known:
            bearing = math.degrees(scan.angle_min + i * scan.angle_increment)
            bearing = (bearing + 180.0) % 360.0 - 180.0
            arrived.append((bearing, r))

        # Sample a point 60% along the ray. If the map calls it occupied but
        # the beam passed through, something LEFT.
        mx = rx + 0.6 * r * math.cos(ang)
        my = ry + 0.6 * r * math.sin(ang)
        mid = _world_to_cell(mx, my, info)
        if mid is not None and _cell_value(grid, info, *mid) >= 90:
            departed += 1

    if checked == 0:
        return conf + " Map comparison unavailable (no usable scan returns)."

    lines = [conf]

    # Group arrivals into contiguous bearing runs so one stray ray is not
    # reported as an obstacle.
    if arrived:
        arrived.sort()
        runs, cur = [], [arrived[0]]
        for prev, item in zip(arrived, arrived[1:]):
            if item[0] - prev[0] <= 2.0:
                cur.append(item)
            else:
                runs.append(cur)
                cur = [item]
        runs.append(cur)
        real = [g for g in runs if len(g) >= MIN_RUN]

        if real:
            real.sort(key=lambda g: min(x[1] for x in g))
            parts = []
            for g in real[:3]:
                a0, a1 = g[0][0], g[-1][0]
                d = min(x[1] for x in g)
                width = 2.0 * d * math.sin(math.radians(abs(a1 - a0) / 2.0))
                parts.append("%.1fm away at bearing %.0f to %.0f degrees "
                             "(roughly %.1fm wide)" % (d, a0, a1, width))
            lines.append(
                "UNMAPPED OBSTACLE PRESENT: the map shows clear ground, but "
                "the scan reports something " + "; ".join(parts) + ". "
                "Something is here that was not here when the map was made - "
                "a parked vehicle, a delivery, a barrier. The global planner "
                "does NOT know about it and may route straight through it. "
                "Use look to find out WHAT it is before deciding: a vehicle "
                "unloading will move, a barrier will not.")

    if departed > checked * 0.15:
        lines.append(
            "MAPPED OBSTACLE GONE: %d%% of scan rays pass straight through "
            "cells the map records as occupied. Something that was here when "
            "the map was made has left. Routes the planner currently refuses "
            "may in fact be open."
            % int(100.0 * departed / checked))

    if len(lines) == 1:
        lines.append("Map and sensor agree: nothing present that the map does "
                     "not already know about.")

    return " ".join(lines)