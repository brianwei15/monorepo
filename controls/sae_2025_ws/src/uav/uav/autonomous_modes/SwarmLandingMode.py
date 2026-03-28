import math
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV
from px4_msgs.msg import VehicleStatus


RETURN_ALTITUDE = -5.0  # NED z — altitude to hold while returning to origin (5m AGL)
RETURN_MARGIN_XY = 0.5  # metres — horizontal arrival tolerance before landing
RETURN_MARGIN_Z  = 1.5  # metres — vertical tolerance (looser, altitude varies more)


class SwarmLandingMode(Mode):
    """
    Return all drones to their spawn origin, then land.
    Phase 0 (returning): each drone flies back to (0, 0, RETURN_ALTITUDE) in its local frame.
    Phase 1 (landing):   once at origin, command AUTO_LAND.
    Complete (terminate) when all drones are disarmed and out of AUTO_LAND.
    """

    def __init__(self, node: Node, uav: UAV):
        super().__init__(node, uav)
        self.uavs = [getattr(node, f"uav{i}") for i in range(1, len(node._all_uavs) + 1)]
        # "returning" → fly back to origin; "landing" → execute land
        self._phase = ["returning"] * len(self.uavs)

    def on_update(self, time_delta: float) -> None:
        for i, uav in enumerate(self.uavs):
            if self._phase[i] == "returning":
                origin = (0.0, 0.0, RETURN_ALTITUDE)
                uav.publish_position_setpoint(origin)
                if uav.local_position:
                    xy_dist = math.sqrt(
                        uav.local_position.x ** 2 + uav.local_position.y ** 2
                    )
                    z_dist = abs(uav.local_position.z - RETURN_ALTITUDE)
                    if xy_dist < RETURN_MARGIN_XY and z_dist < RETURN_MARGIN_Z:
                        self.log(f"Drone {i+1}: at origin, initiating land")
                        self._phase[i] = "landing"
            else:
                # Maintain current position setpoint while transitioning to AUTO_LAND
                if uav.local_position:
                    uav.publish_position_setpoint(
                        (uav.local_position.x, uav.local_position.y, uav.local_position.z),
                        lock_yaw=True,
                    )
                if uav.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                    uav.land()

    def check_status(self) -> str:
        all_landed = all(
            uav.arm_state == VehicleStatus.ARMING_STATE_DISARMED
            and uav.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND
            for uav in self.uavs
        )
        if all_landed:
            self.log("All drones landed and disarmed.")
            return "terminate"
        return "continue"
