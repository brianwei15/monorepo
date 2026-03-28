import math
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV


# Search altitudes per drone (NED z, negative = up). Each drone flies a different layer.
# Drone 1: 5m AGL, Drone 2: 8m AGL, Drone 3: 11m AGL
DRONE_ALTITUDES = [-5.0, -8.0, -11.0]

# Snake (lawnmower) waypoints in LOCAL NED, relative to each drone's own spawn origin.
# Covers a 15m North × 10m East area with 3 N-S passes spaced 5m apart.
# NED: x = North, y = East, z = Down (negative = up)
def _make_snake_waypoints(alt: float):
    return [
        (15.0,  0.0, alt),  # north pass 1
        (15.0,  5.0, alt),  # step east
        (0.0,   5.0, alt),  # south pass 2
        (0.0,  10.0, alt),  # step east
        (15.0, 10.0, alt),  # north pass 3
    ]

MARGIN = 1.5  # meters — arrival tolerance


class SwarmSearchMode(Mode):
    """
    Snake (lawnmower) search path for all drones in the swarm.
    Each drone independently navigates its own waypoints at a different altitude,
    covering parallel layers of the search volume.
    Complete when all drones have visited every waypoint.
    """

    def __init__(self, node: Node, uav: UAV):
        super().__init__(node, uav)
        self.uavs = [getattr(node, f"uav{i}") for i in range(1, len(node._all_uavs) + 1)]
        self._waypoints = [
            _make_snake_waypoints(DRONE_ALTITUDES[i] if i < len(DRONE_ALTITUDES) else DRONE_ALTITUDES[-1])
            for i in range(len(self.uavs))
        ]
        self._wp_index = [0] * len(self.uavs)

    def on_update(self, time_delta: float) -> None:
        for i, uav in enumerate(self.uavs):
            waypoints = self._waypoints[i]
            if self._wp_index[i] >= len(waypoints):
                # Drone finished — hover in place
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
                        f"Drone {i+1}: reached waypoint {self._wp_index[i]+1}/{len(waypoints)}"
                    )
                    self._wp_index[i] += 1

    def check_status(self) -> str:
        if all(self._wp_index[i] >= len(self._waypoints[i]) for i in range(len(self.uavs))):
            self.log("All drones completed search path.")
            return "complete"
        return "continue"
