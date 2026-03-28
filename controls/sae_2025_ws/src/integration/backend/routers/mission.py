from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Form, Query

from ..context import AppContext
from ..models import (
    LaunchParamsResponse,
    MessageResponse,
    MissionFileResponse,
    MissionLaunchStatusResponse,
    MissionLogsResponse,
    MissionNameOptionsResponse,
    MissionStateResponse,
)
from ..services import local_mission

_DRONE_ID = "main"


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(tags=["mission"])

    @router.get("/api/mission/state", response_model=MissionStateResponse)
    async def mission_state() -> MissionStateResponse:
        return MissionStateResponse.model_validate(
            await local_mission.mission_state(ctx, _DRONE_ID)
        )

    @router.get(
        "/api/mission/launch/status", response_model=MissionLaunchStatusResponse
    )
    async def mission_launch_status() -> MissionLaunchStatusResponse:
        return MissionLaunchStatusResponse.model_validate(
            await local_mission.refresh_runtime_state(ctx, _DRONE_ID)
        )

    @router.get("/api/mission/launch/logs", response_model=MissionLogsResponse)
    async def mission_launch_logs(
        lines: Annotated[int, Query()] = 200,
        offset: Annotated[int | None, Query()] = None,
        inode: Annotated[int | None, Query()] = None,
    ) -> MissionLogsResponse:
        return MissionLogsResponse.model_validate(
            await local_mission.launch_logs(
                ctx,
                _DRONE_ID,
                lines=lines,
                offset=offset,
                inode=inode,
            )
        )

    @router.post(
        "/api/mission/prepare", response_model=MessageResponse | MissionLogsResponse
    )
    async def prepare_mission():
        result = await local_mission.prepare_mission(ctx, _DRONE_ID)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return MissionLogsResponse.model_validate(
            {**result, "running": False, "logs": ""}
        )

    @router.post(
        "/api/mission/stop", response_model=MessageResponse | MissionLogsResponse
    )
    async def stop_mission():
        result = await local_mission.stop_mission(ctx, _DRONE_ID)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return MissionLogsResponse.model_validate(
            {**result, "running": False, "logs": ""}
        )

    @router.post(
        "/api/mission/start", response_model=MessageResponse | MissionLogsResponse
    )
    async def start_mission():
        result = await local_mission.start_mission(ctx, _DRONE_ID)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return MissionLogsResponse.model_validate(
            {**result, "running": False, "logs": ""}
        )

    @router.post("/api/failsafe", response_model=MessageResponse | MissionLogsResponse)
    async def trigger_failsafe():
        result = await local_mission.trigger_failsafe(ctx, _DRONE_ID)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return MissionLogsResponse.model_validate(
            {**result, "running": False, "logs": ""}
        )

    @router.get("/api/mission/launch-params", response_model=LaunchParamsResponse)
    async def get_launch_params() -> LaunchParamsResponse:
        return LaunchParamsResponse.model_validate(
            await local_mission.get_launch_params(ctx, _DRONE_ID)
        )

    @router.get("/api/mission/mission-names", response_model=MissionNameOptionsResponse)
    async def get_mission_names() -> MissionNameOptionsResponse:
        return MissionNameOptionsResponse.model_validate(
            await local_mission.list_mission_names(ctx, _DRONE_ID)
        )

    @router.post("/api/mission/launch-params", response_model=LaunchParamsResponse)
    async def set_launch_params(content: Annotated[str, Form(...)]):
        return LaunchParamsResponse.model_validate(
            await local_mission.set_launch_params(ctx, _DRONE_ID, content=content)
        )

    @router.get("/api/mission/mission-file", response_model=MissionFileResponse)
    async def get_mission_file(name: Annotated[str, Query(...)]) -> MissionFileResponse:
        return MissionFileResponse.model_validate(
            await local_mission.get_mission_file(ctx, _DRONE_ID, name=name)
        )

    @router.post("/api/mission/mission-file", response_model=MissionFileResponse)
    async def set_mission_file(
        name: Annotated[str, Form(...)], content: Annotated[str, Form(...)]
    ) -> MissionFileResponse:
        return MissionFileResponse.model_validate(
            await local_mission.set_mission_file(
                ctx, _DRONE_ID, name=name, content=content
            )
        )

    return router
