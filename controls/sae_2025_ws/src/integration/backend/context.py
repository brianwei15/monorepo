from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from pathlib import Path

from .config import DRONE_FLEET, RuntimeConfig
from .ssh import SSHExecutor
from .state import MissionStateMachine


@dataclass(slots=True)
class DroneContext:
    drone_id: str
    name: str
    base_dir: Path
    config: RuntimeConfig
    ssh: SSHExecutor
    mission_state: MissionStateMachine

    def as_app_context(self) -> "AppContext":
        return AppContext(
            base_dir=self.base_dir,
            config=self.config,
            ssh=self.ssh,
            mission_state=self.mission_state,
            fleet={},
        )


@dataclass(slots=True)
class AppContext:
    base_dir: Path
    config: RuntimeConfig
    ssh: SSHExecutor
    mission_state: MissionStateMachine
    fleet: dict[str, DroneContext]


def create_context(base_dir: Path) -> AppContext:
    config = RuntimeConfig.from_env(base_dir)
    ssh = SSHExecutor(config)
    mission_state = MissionStateMachine.create()
    fleet = _create_fleet(config, base_dir)
    return AppContext(
        base_dir=base_dir,
        config=config,
        ssh=ssh,
        mission_state=mission_state,
        fleet=fleet,
    )


def _create_fleet(base_config: RuntimeConfig, base_dir: Path) -> dict[str, DroneContext]:
    fleet = {}
    for drone in DRONE_FLEET:
        drone_config = dataclasses.replace(base_config, pi_host=drone["host"])
        fleet[drone["id"]] = DroneContext(
            drone_id=drone["id"],
            name=drone["name"],
            base_dir=base_dir,
            config=drone_config,
            ssh=SSHExecutor(drone_config),
            mission_state=MissionStateMachine.create(),
        )
    return fleet
