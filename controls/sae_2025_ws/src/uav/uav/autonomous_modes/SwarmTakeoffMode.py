from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV
from px4_msgs.msg import VehicleStatus


class SwarmTakeoffMode(Mode):
    """
    Vertical takeoff for all drones in the swarm.
    Commands takeoff on each drone independently; complete when all reach OFFBOARD mode.
    """

    def __init__(self, node: Node, uav: UAV):
        super().__init__(node, uav)
        self.uavs = [getattr(node, f"uav{i}") for i in range(1, len(node._all_uavs) + 1)]
        self._commanded = [False] * len(self.uavs)
        self._elapsed = [0.0] * len(self.uavs)

    def on_update(self, time_delta: float) -> None:
        for i, uav in enumerate(self.uavs):
            self._elapsed[i] += time_delta
            if uav.local_position is None or uav.global_position is None:
                continue
            if not self._commanded[i]:
                uav.takeoff()
                self._commanded[i] = True
                self._elapsed[i] = 0.0
                self.log(f"Drone {i+1}: takeoff commanded")
            elif (
                self._elapsed[i] >= 1.0
                and uav.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
            ):
                uav.engage_offboard_mode()
                self.log(f"Drone {i+1}: engaging offboard mode")

    def check_status(self) -> str:
        if all(
            uav.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            for uav in self.uavs
        ):
            self.log("All drones in OFFBOARD mode. Takeoff complete.")
            return "complete"
        return "continue"
