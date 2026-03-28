from __future__ import annotations

import asyncio
import logging
from typing import Annotated

from fastapi import APIRouter, Form, Query, WebSocket

log = logging.getLogger(__name__)

from ..config import DRONE_FLEET
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
from ..services import local_mission as mission_service


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(tags=["drones"])

    # ── Fleet status ──────────────────────────────────────────────────────────

    @router.get("/api/drones")
    async def list_drones():
        log.info("/api/drones called")
        async def _ping(drone_id: str, name: str, host: str):
            drone = ctx.fleet.get(drone_id)
            if not drone:
                log.warning("No fleet entry for %s", drone_id)
                return {"id": drone_id, "name": name, "host": host, "connected": False}
            cmd = drone.ssh.build_ssh_cmd("echo ok")
            log.info("Pinging %s (%s): %s", drone_id, host, " ".join(cmd))
            try:
                result = await asyncio.wait_for(drone.ssh.run("echo ok", timeout=5), timeout=6)
                connected = result.returncode == 0
                if connected:
                    log.info("SSH ping OK for %s (%s)", drone_id, host)
                else:
                    log.warning("SSH ping failed for %s (%s) rc=%s stderr=%r", drone_id, host, result.returncode, result.stderr.strip())
            except asyncio.TimeoutError:
                log.warning("SSH ping timed out (thread pool likely saturated) for %s (%s)", drone_id, host)
                connected = False
            except Exception as exc:
                log.warning("SSH ping exception for %s (%s): %s", drone_id, host, exc)
                connected = False
            return {"id": drone_id, "name": name, "host": host, "connected": connected}

        results = await asyncio.gather(
            *[_ping(d["id"], d["name"], d["host"]) for d in DRONE_FLEET]
        )
        return {"success": True, "drones": list(results)}

    # ── Per-drone helpers ─────────────────────────────────────────────────────

    def _drone(drone_id: str):
        return ctx.fleet.get(drone_id)

    def _missing(drone_id: str):
        return {"success": False, "error": f"Unknown drone: {drone_id}"}

    # ── Mission state & logs ──────────────────────────────────────────────────

    @router.get("/api/drones/{drone_id}/state", response_model=MissionStateResponse)
    async def drone_state(drone_id: str):
        d = _drone(drone_id)
        if not d:
            return _missing(drone_id)
        return MissionStateResponse.model_validate(
            await mission_service.mission_state(d.as_app_context(), drone_id)
        )

    @router.get("/api/drones/{drone_id}/launch/status", response_model=MissionLaunchStatusResponse)
    async def drone_launch_status(drone_id: str):
        d = _drone(drone_id)
        if not d:
            return _missing(drone_id)
        return MissionLaunchStatusResponse.model_validate(
            await mission_service.refresh_runtime_state(d.as_app_context(), drone_id)
        )

    @router.get("/api/drones/{drone_id}/launch/logs", response_model=MissionLogsResponse)
    async def drone_launch_logs(
        drone_id: str,
        lines: Annotated[int, Query()] = 200,
        offset: Annotated[int | None, Query()] = None,
        inode: Annotated[int | None, Query()] = None,
    ):
        d = _drone(drone_id)
        if not d:
            return _missing(drone_id)
        return MissionLogsResponse.model_validate(
            await mission_service.launch_logs(
                d.as_app_context(), drone_id, lines=lines, offset=offset, inode=inode
            )
        )

    # ── Mission actions ───────────────────────────────────────────────────────

    @router.post("/api/drones/{drone_id}/prepare")
    async def drone_prepare(drone_id: str):
        d = _drone(drone_id)
        if not d:
            return _missing(drone_id)
        result = await mission_service.prepare_mission(d.as_app_context(), drone_id)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return MissionLogsResponse.model_validate({**result, "running": False, "logs": ""})

    @router.post("/api/drones/{drone_id}/stop")
    async def drone_stop(drone_id: str):
        d = _drone(drone_id)
        if not d:
            return _missing(drone_id)
        result = await mission_service.stop_mission(d.as_app_context(), drone_id)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return MissionLogsResponse.model_validate({**result, "running": False, "logs": ""})

    @router.post("/api/drones/{drone_id}/start")
    async def drone_start(drone_id: str):
        d = _drone(drone_id)
        if not d:
            return _missing(drone_id)
        result = await mission_service.start_mission(d.as_app_context(), drone_id)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return MissionLogsResponse.model_validate({**result, "running": False, "logs": ""})

    @router.post("/api/drones/{drone_id}/failsafe")
    async def drone_failsafe(drone_id: str):
        d = _drone(drone_id)
        if not d:
            return _missing(drone_id)
        result = await mission_service.trigger_failsafe(d.as_app_context(), drone_id)
        if result.get("success"):
            return MessageResponse.model_validate(result)
        return MissionLogsResponse.model_validate({**result, "running": False, "logs": ""})

    # ── Launch params & mission files ─────────────────────────────────────────

    @router.get("/api/drones/{drone_id}/launch-params", response_model=LaunchParamsResponse)
    async def drone_get_launch_params(drone_id: str):
        d = _drone(drone_id)
        if not d:
            return _missing(drone_id)
        return LaunchParamsResponse.model_validate(
            await mission_service.get_launch_params(d.as_app_context(), drone_id)
        )

    @router.post("/api/drones/{drone_id}/launch-params", response_model=LaunchParamsResponse)
    async def drone_set_launch_params(drone_id: str, content: Annotated[str, Form(...)]):
        d = _drone(drone_id)
        if not d:
            return _missing(drone_id)
        return LaunchParamsResponse.model_validate(
            await mission_service.set_launch_params(
                d.as_app_context(), drone_id, content=content
            )
        )

    @router.get("/api/drones/{drone_id}/mission-names", response_model=MissionNameOptionsResponse)
    async def drone_mission_names(drone_id: str):
        d = _drone(drone_id)
        if not d:
            return _missing(drone_id)
        return MissionNameOptionsResponse.model_validate(
            await mission_service.list_mission_names(d.as_app_context(), drone_id)
        )

    @router.get("/api/drones/{drone_id}/mission-file", response_model=MissionFileResponse)
    async def drone_get_mission_file(drone_id: str, name: Annotated[str, Query(...)]):
        d = _drone(drone_id)
        if not d:
            return _missing(drone_id)
        return MissionFileResponse.model_validate(
            await mission_service.get_mission_file(d.as_app_context(), drone_id, name=name)
        )

    @router.post("/api/drones/{drone_id}/mission-file", response_model=MissionFileResponse)
    async def drone_set_mission_file(
        drone_id: str,
        name: Annotated[str, Form(...)],
        content: Annotated[str, Form(...)],
    ):
        d = _drone(drone_id)
        if not d:
            return _missing(drone_id)
        return MissionFileResponse.model_validate(
            await mission_service.set_mission_file(
                d.as_app_context(), drone_id, name=name, content=content
            )
        )

    # ── Terminal WebSocket ────────────────────────────────────────────────────

    @router.websocket("/ws/drones/{drone_id}/terminal")
    async def drone_terminal(
        drone_id: str,
        websocket: WebSocket,
        offset: int = Query(default=0),
        inode: int = Query(default=0),
    ):
        d = _drone(drone_id)
        if not d:
            await websocket.close(code=4004)
            return
        await mission_service.stream_terminal(
            d.as_app_context(), drone_id, websocket, offset=offset, inode=inode
        )

    return router
