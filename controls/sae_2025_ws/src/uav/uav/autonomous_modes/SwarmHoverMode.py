from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV


class SwarmHoverMode(Mode):
    """
    Hold every drone in the swarm at the position it occupied when this
    mode was entered.  Never transitions out on its own (returns "continue"
    indefinitely), so it acts as an open-ended loiter after raft detection.
    """

    def __init__(self, node: Node, uav: UAV):
        super().__init__(node, uav)
        self.uavs = [
            getattr(node, f"uav{i}") for i in range(1, len(node._all_uavs) + 1)
        ]
        self._hold_positions: dict = {}  # drone index → (x, y, z)

    def on_enter(self) -> None:
        self._hold_positions.clear()
        for i, uav in enumerate(self.uavs):
            if uav.local_position is not None:
                self._hold_positions[i] = (
                    uav.local_position.x,
                    uav.local_position.y,
                    uav.local_position.z,
                )
                self.log(
                    f"Drone {i + 1}: holding at "
                    f"({uav.local_position.x:.1f}, "
                    f"{uav.local_position.y:.1f}, "
                    f"{uav.local_position.z:.1f})"
                )

    def on_update(self, time_delta: float) -> None:
        for i, uav in enumerate(self.uavs):
            if i not in self._hold_positions:
                # Position wasn't available at on_enter; latch it now
                if uav.local_position is not None:
                    self._hold_positions[i] = (
                        uav.local_position.x,
                        uav.local_position.y,
                        uav.local_position.z,
                    )
            if i in self._hold_positions:
                uav.publish_position_setpoint(
                    self._hold_positions[i], lock_yaw=True
                )

    def check_status(self) -> str:
        return "continue"
