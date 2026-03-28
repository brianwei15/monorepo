import math
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV


# ----------------------------------------------------------------
# Search pattern parameters (body frame: fwd = forward, right = right)
# ----------------------------------------------------------------
# Per-drone search altitudes (NED z, negative = up). Add/remove entries to match drone count.
SEARCH_ALTITUDES = [-5.0, -8.0, -11.0]  # Drone 1: 5m, Drone 2: 8m, Drone 3: 11m AGL

STRIP_WIDTH  = 5.0   # metres between parallel snake passes
FORWARD_DIST = 15.0  # metres forward per pass
N_STRIPS     = 3     # number of forward/back passes

MARGIN = 1.5         # metres — arrival tolerance


def _make_snake_waypoints_body(alt: float) -> list:
    """
    Lawnmower waypoints in the drone's *body frame*:
      fwd   = forward (along initial heading)
      right = right (perpendicular, CW from forward)

    Returns a list of (fwd, right, z) tuples.
    The pattern starts at the drone's spawn and sweeps forward
    FORWARD_DIST metres, stepping right by STRIP_WIDTH each pass.
    """
    wps = []
    for i in range(N_STRIPS):
        right_pos = i * STRIP_WIDTH
        if i % 2 == 0:                      # outward pass
            wps.append((0.0,          right_pos, alt))
            wps.append((FORWARD_DIST, right_pos, alt))
        else:                               # return pass
            wps.append((FORWARD_DIST, right_pos, alt))
            wps.append((0.0,          right_pos, alt))
        if i < N_STRIPS - 1:               # lateral step to next strip
            next_right = (i + 1) * STRIP_WIDTH
            end_fwd = FORWARD_DIST if i % 2 == 0 else 0.0
            wps.append((end_fwd, next_right, alt))
    return wps


def _body_to_ned(fwd: float, right: float, yaw_ned: float) -> tuple:
    """
    Rotate a body-frame offset (fwd, right) to NED (north, east) using
    the drone's NED heading yaw_ned (radians, 0 = North, π/2 = East).

    NED rotation:
        north = fwd * cos(yaw) - right * sin(yaw)
        east  = fwd * sin(yaw) + right * cos(yaw)
    """
    north = fwd * math.cos(yaw_ned) - right * math.sin(yaw_ned)
    east  = fwd * math.sin(yaw_ned) + right * math.cos(yaw_ned)
    return north, east


class SwarmSearchMode(Mode):
    """
    Snake (lawnmower) search for every drone in the swarm.

    Waypoints are defined in each drone's body frame (forward / right relative
    to its spawn position) and then rotated into NED using the reference heading
    captured by SwarmTakeoffMode from the middle drone's initial yaw.

    This means the search pattern follows the physical placement direction of the
    drones, regardless of where PX4 happens to be pointing them at startup.

    Complete when all drones have visited every waypoint.
    """

    def __init__(self, node: Node, uav: UAV):
        super().__init__(node, uav)
        self.uavs = [getattr(node, f"uav{i}") for i in range(1, len(node._all_uavs) + 1)]
        # Waypoints are built in on_enter so that the reference yaw captured by
        # SwarmTakeoffMode.on_enter() is already available on the node.
        self._waypoints = []
        self._wp_index = [0] * len(self.uavs)

    def on_enter(self) -> None:
        # Read the reference yaw written by SwarmTakeoffMode; default to 0 (North) if missing.
        ref_yaw = getattr(self.node, "swarm_reference_yaw", 0.0)
        self.log(f"Using reference yaw: {ref_yaw:.3f} rad ({math.degrees(ref_yaw):.1f}°)")

        # Rotate body-frame waypoints to NED for every drone.
        # Each drone flies at its own altitude; same XY pattern translated by spawn.
        self._waypoints = []
        for i in range(len(self.uavs)):
            alt = SEARCH_ALTITUDES[i] if i < len(SEARCH_ALTITUDES) else SEARCH_ALTITUDES[-1]
            body_wps = _make_snake_waypoints_body(alt)
            ned_wps = [
                (*_body_to_ned(fwd, right, ref_yaw), z)
                for (fwd, right, z) in body_wps
            ]
            self._waypoints.append(ned_wps)

        self._wp_index = [0] * len(self.uavs)

    def on_update(self, time_delta: float) -> None:
        for i, uav in enumerate(self.uavs):
            waypoints = self._waypoints[i]
            if self._wp_index[i] >= len(waypoints):
                # Finished — hold current position
                if uav.local_position:
                    uav.publish_position_setpoint(
                        (uav.local_position.x, uav.local_position.y, uav.local_position.z),
                        lock_yaw=True,
                    )
                continue

            wp = waypoints[self._wp_index[i]]
            uav.publish_position_setpoint(wp)

            if uav.local_position:
                dist = math.sqrt(
                    (uav.local_position.x - wp[0]) ** 2
                    + (uav.local_position.y - wp[1]) ** 2
                    + (uav.local_position.z - wp[2]) ** 2
                )
                if dist < MARGIN:
                    self.log(
                        f"Drone {i + 1}: reached waypoint {self._wp_index[i] + 1}/{len(waypoints)}"
                    )
                    self._wp_index[i] += 1

    def check_status(self) -> str:
        if all(
            self._wp_index[i] >= len(self._waypoints[i])
            for i in range(len(self.uavs))
        ):
            self.log("All drones completed search path.")
            return "complete"
        return "continue"
