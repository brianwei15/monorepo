import math
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
    ):
        super().__init__(node, uav)
        self.uavs = [
            getattr(node, f"uav{i}") for i in range(1, len(node._all_uavs) + 1)
        ]

        self._heading_deg = initial_search_heading_deg
        self.log(f"{initial_search_heading_deg}")
        self.log(f"{initial_search_heading_deg}")
        self.log(f"{initial_search_heading_deg}")
        self.log(f"{initial_search_heading_deg}")
        self.log(f"{initial_search_heading_deg}")
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

    def on_enter(self) -> None:
        self.log(
            f"NE UNIT VECTOR for {self._heading_deg} {self.ned_deg_to_unit_vector(self._heading_deg)}"
        )
        ref_yaw = math.radians(self._heading_deg)
        self.log(f"Search heading: {self._heading_deg}° → {ref_yaw:.3f} rad NED")

        self._waypoints = self.create_triangle_pattern(self._heading_deg, self._forward_dist)
        self._wp_index = [0] * len(self.uavs)

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
            [float(initial_heading), float(dist)], #initial heading is NED
            [120.0, float(dist)/3],
            [120.0, float(dist)],
            [120.0, float(dist)/3],
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
            d3_next_wp = np.array([left_xy[0], left_xy[1], self._altitudes[1]])
            waypoints[0].append(d1_next_wp)
            waypoints[1].append(d2_next_wp)
            waypoints[2].append(d3_next_wp)

        return waypoints

    def on_update(self, time_delta: float) -> None:
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

    def check_status(self) -> str:
        if all(
            self._wp_index[i] >= len(self._waypoints[i]) for i in range(len(self.uavs))
        ):
            self.log("All drones completed search path.")
            return "complete"
        return "continue"

    def ned_deg_to_unit_vector(self, heading_deg):
        """
        Convert a NED heading (degrees, 0=North, 90=East, clockwise)
        to a unit vector (x=North, y=East).
        """
        rad = math.radians(heading_deg)
        x = math.cos(rad)  # North component
        y = math.sin(rad)  # East component
        return np.array([x, y])
