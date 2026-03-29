import math
from geometry_msgs.msg import PointStamped
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV
import numpy as np


def _make_snake_waypoints_body(
    alt: float,
    right_start: float,
    n_strips: int,
    strip_width: float,
    forward_dist: float,
) -> list:
    wps = []
    for j in range(n_strips):
        right_pos = right_start + j * strip_width
        if j % 2 == 0:
            wps.append((0.0, right_pos, alt))
            wps.append((forward_dist, right_pos, alt))
        else:
            wps.append((forward_dist, right_pos, alt))
            wps.append((0.0, right_pos, alt))
        if j < n_strips - 1:
            next_right = right_start + (j + 1) * strip_width
            end_fwd = forward_dist if j % 2 == 0 else 0.0
            wps.append((end_fwd, next_right, alt))
    return wps


def _body_to_ned(fwd: float, right: float, yaw_ned: float) -> tuple:
    north = fwd * math.cos(yaw_ned) - right * math.sin(yaw_ned)
    east = fwd * math.sin(yaw_ned) + right * math.cos(yaw_ned)
    return north, east


class SwarmSearchMode(Mode):
    """
    Snake (lawnmower) search for every drone in the swarm.
    All parameters are read from the mission YAML params block.

    While searching, each drone's raft detection topic is monitored.
    On the first confirmed detection, all drones abort their search paths
    and converge on the raft's estimated world position (the detecting
    drone's NED XY), each maintaining its own search altitude.
    Returns "raft_found" once all drones are within `margin` of the
    convergence point, or "complete" if the search paths finish first.
    """

    def __init__(
        self,
        node: Node,
        uav: UAV,
        initial_search_heading_deg: float = 0.0,
        search_altitudes: list = None,
        strip_width: float = 5.0,
        forward_dist: float = 15.0,
        n_strips: int = 3,
        path_segments=[],
        drone_spacing: float = 6.0,
        margin: float = 1.5,
        detection_topics: list = None,
    ):
        super().__init__(node, uav)
        self.uavs = [
            getattr(node, f"uav{i}") for i in range(1, len(node._all_uavs) + 1)
        ]

        self._heading_deg = initial_search_heading_deg
        self._altitudes = (
            search_altitudes if search_altitudes is not None else [-5.0, -8.0, -11.0]
        )
        self._strip_width = strip_width
        self._forward_dist = forward_dist
        self._n_strips = n_strips
        self._drone_spacing = drone_spacing
        self._margin = margin

        self._waypoints = []
        self._wp_index = [0] * len(self.uavs)
        self.curr_heading = self.ned_deg_to_unit_vector(self._heading_deg)

        # Raft detection state
        self._raft_detected = False
        self._converge_xy = None  # (north, east) NED when raft found

        # Subscribe to one detection topic per drone (or a shared topic).
        # Each topic's callback records which UAV index reported the sighting
        # so we can use that drone's NED position as the raft world location.
        topics = detection_topics or ["/raft_detection/rgb"]
        self._detection_subs = []
        for i, topic in enumerate(topics):
            drone_idx = i if i < len(self.uavs) else 0
            self._detection_subs.append(
                node.create_subscription(
                    PointStamped,
                    topic,
                    lambda msg, idx=drone_idx: self._detection_callback(msg, idx),
                    10,
                )
            )
        self.log(
            f"Subscribed to {len(self._detection_subs)} raft detection topic(s): {topics}"
        )

    # ------------------------------------------------------------------
    # ROS callback
    # ------------------------------------------------------------------

    def _detection_callback(self, msg: PointStamped, drone_idx: int) -> None:
        """Called when a raft detection is published on any subscribed topic."""
        if self._raft_detected:
            return
        # z == -1 means "not detected"; z > 0 is the confidence score
        if msg.point.z <= 0.0:
            return
        uav = self.uavs[drone_idx]
        if uav.local_position is None:
            return
        self._converge_xy = (uav.local_position.x, uav.local_position.y)
        self._raft_detected = True
        self.log(
            f"Raft detected by drone {drone_idx + 1} "
            f"(score={msg.point.z:.2f}) — "
            f"converging all drones to NED "
            f"({self._converge_xy[0]:.1f}, {self._converge_xy[1]:.1f})"
        )

    # ------------------------------------------------------------------
    # Mode lifecycle
    # ------------------------------------------------------------------

    def on_enter(self) -> None:
        self.log(
            f"NE UNIT VECTOR for {self._heading_deg} {self.ned_deg_to_unit_vector(self._heading_deg)}"
        )
        ref_yaw = math.radians(self._heading_deg)
        self.log(f"Search heading: {self._heading_deg}° → {ref_yaw:.3f} rad NED")

        self._waypoints = self.create_triangle_pattern(self._heading_deg, self._forward_dist)
        self._wp_index = [0] * len(self.uavs)
        self._raft_detected = False
        self._converge_xy = None

    def on_update(self, time_delta: float) -> None:
        if self._raft_detected:
            self._update_converge()
        else:
            self._update_search()

    def check_status(self) -> str:
        if self._raft_detected:
            # All drones converged in XY?
            if self._converge_xy is not None and all(
                self._drone_xy_dist(uav) < self._margin
                for uav in self.uavs
                if uav.local_position is not None
            ):
                self.log("All drones converged on raft location.")
                return "raft_found"
            return "continue"

        if all(
            self._wp_index[i] >= len(self._waypoints[i]) for i in range(len(self.uavs))
        ):
            self.log("All drones completed search path.")
            return "complete"
        return "continue"

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _update_search(self) -> None:
        """Advance each drone along its search waypoints."""
        for i, uav in enumerate(self.uavs):
            waypoints = self._waypoints[i]
            if self._wp_index[i] >= len(waypoints):
                if uav.local_position:
                    uav.publish_position_setpoint(
                        (
                            uav.local_position.x,
                            uav.local_position.y,
                            uav.local_position.z,
                        ),
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
                if dist < self._margin:
                    self.log(
                        f"Drone {i + 1}: reached waypoint {self._wp_index[i] + 1}/{len(waypoints)}"
                    )
                    self._wp_index[i] += 1

    def _update_converge(self) -> None:
        """Fly every drone to the raft XY while keeping its current altitude."""
        cx, cy = self._converge_xy
        for uav in self.uavs:
            if uav.local_position is None:
                continue
            uav.publish_position_setpoint(
                (cx, cy, uav.local_position.z),
                lock_yaw=True,
            )

    def _drone_xy_dist(self, uav) -> float:
        """Horizontal distance from a drone to the convergence point."""
        cx, cy = self._converge_xy
        return math.sqrt(
            (uav.local_position.x - cx) ** 2
            + (uav.local_position.y - cy) ** 2
        )

    # ------------------------------------------------------------------
    # Pattern generators
    # ------------------------------------------------------------------

    def create_circle_pattern(self, initial_heading, radius):
        path_segments = [
            [float(initial_heading + i), float(radius / 360)] for i in range(360)
        ]

        waypoints = [[], [], []]
        curr_heading = 0.0
        curr_pos = np.array(
            [self.uavs[0].local_position.x, self.uavs[0].local_position.y]
        )
        for rel_heading, distance in path_segments:
            self.log(f"{rel_heading, distance}")
            curr_heading += rel_heading
            unit_dir = self.ned_deg_to_unit_vector(curr_heading)
            next_xy = distance * unit_dir + curr_pos
            curr_pos = next_xy
            d1_next_wp = np.array([next_xy[0], next_xy[1], self._altitudes[0]])
            rel_right = self.ned_deg_to_unit_vector(curr_heading + 90.0)
            right_xy = next_xy + (self._drone_spacing * rel_right)
            rel_left = self.ned_deg_to_unit_vector(curr_heading - 90.0)
            left_xy = next_xy + (self._drone_spacing * rel_left)
            d2_next_wp = np.array([right_xy[0], right_xy[1], self._altitudes[1]])
            d3_next_wp = np.array([left_xy[0], left_xy[1], self._altitudes[1]])
            waypoints[0].append(d1_next_wp)
            waypoints[1].append(d2_next_wp)
            waypoints[2].append(d3_next_wp)

        return waypoints

    def create_triangle_pattern(self, initial_heading, dist):
        path_segments = [
            [float(initial_heading), float(dist)],  # initial heading is NED
            [120.0, float(dist) / 3],
            [120.0, float(dist)],
            [120.0, float(dist) / 3],
            [120.0, float(dist)],
            # then it returns to local origin
        ]

        return self._pattern_helper(path_segments)

    def _pattern_helper(self, path_segments):
        waypoints = [[], [], []]
        curr_heading = 0.0
        curr_pos = np.array(
            [self.uavs[0].local_position.x, self.uavs[0].local_position.y]
        )
        for rel_heading, distance in path_segments:
            self.log(f"{rel_heading, distance}")
            curr_heading += rel_heading
            unit_dir = self.ned_deg_to_unit_vector(curr_heading)
            next_xy = distance * unit_dir + curr_pos
            curr_pos = next_xy
            d1_next_wp = np.array([next_xy[0], next_xy[1], self._altitudes[0]])
            rel_right = self.ned_deg_to_unit_vector(curr_heading + 90.0)
            right_xy = next_xy + (self._drone_spacing * rel_right)
            rel_left = self.ned_deg_to_unit_vector(curr_heading - 90.0)
            left_xy = next_xy + (self._drone_spacing * rel_left)
            d2_next_wp = np.array([right_xy[0], right_xy[1], self._altitudes[1]])
            d3_next_wp = np.array([left_xy[0], left_xy[1], self._altitudes[2]])
            waypoints[0].append(d1_next_wp)
            waypoints[1].append(d2_next_wp)
            waypoints[2].append(d3_next_wp)

        return waypoints

    def ned_deg_to_unit_vector(self, heading_deg):
        """
        Convert a NED heading (degrees, 0=North, 90=East, clockwise)
        to a unit vector (x=North, y=East).
        """
        rad = math.radians(heading_deg)
        x = math.cos(rad)  # North component
        y = math.sin(rad)  # East component
        return np.array([x, y])
