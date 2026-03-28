from __future__ import annotations

import asyncio

from fastapi import APIRouter, Query, WebSocket

from ..config import DRONE_FLEET
from ..context import AppContext
from ..services import pi_setup


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(tags=["pi_connections"])

    @router.get("/api/pi-connections")
    async def list_pi_connections():
        async def _status(drone_id: str, name: str, host: str):
            drone = ctx.fleet.get(drone_id)
            if not drone:
                return {"id": drone_id, "name": name, "host": host,
                        "connected": False, "state": "offline"}
            try:
                ping = await drone.ssh.run("echo ok", timeout=3)
                if ping.returncode != 0:
                    return {"id": drone_id, "name": name, "host": host,
                            "connected": False, "state": "offline"}
            except Exception:
                return {"id": drone_id, "name": name, "host": host,
                        "connected": False, "state": "offline"}
            status = await pi_setup.probe_status(drone.as_app_context())
            return {
                "id": drone_id,
                "name": name,
                "host": host,
                "connected": True,
                "state": status.get("state", "unknown"),
                "running": status.get("running", False),
                "pid": status.get("pid"),
            }

        results = await asyncio.gather(
            *[_status(d["id"], d["name"], d["host"]) for d in DRONE_FLEET]
        )
        return {"success": True, "connections": list(results)}

    @router.get("/api/pi-connections/{drone_id}/status")
    async def pi_connection_status(drone_id: str):
        drone = ctx.fleet.get(drone_id)
        if not drone:
            return {"success": False, "error": f"Unknown drone: {drone_id}"}
        try:
            ping = await drone.ssh.run("echo ok", timeout=3)
            if ping.returncode != 0:
                return {"success": True, "connected": False, "state": "offline"}
        except Exception:
            return {"success": True, "connected": False, "state": "offline"}
        status = await pi_setup.probe_status(drone.as_app_context())
        return {**status, "connected": True}

    @router.post("/api/pi-connections/{drone_id}/launch")
    async def pi_launch(drone_id: str):
        drone = ctx.fleet.get(drone_id)
        if not drone:
            return {"success": False, "error": f"Unknown drone: {drone_id}"}
        return await pi_setup.launch(drone.as_app_context(), drone_id)

    @router.get("/api/pi-connections/{drone_id}/logs")
    async def pi_logs(
        drone_id: str,
        offset: int = Query(default=None),
        inode: int = Query(default=None),
    ):
        drone = ctx.fleet.get(drone_id)
        if not drone:
            return {"success": False, "error": f"Unknown drone: {drone_id}"}
        return await pi_setup.get_logs(
            drone.as_app_context(),
            offset=offset,
            inode=inode,
        )

    @router.post("/api/pi-connections/{drone_id}/stop")
    async def pi_stop(drone_id: str):
        drone = ctx.fleet.get(drone_id)
        if not drone:
            return {"success": False, "error": f"Unknown drone: {drone_id}"}
        return await pi_setup.stop(drone.as_app_context())

    @router.websocket("/ws/pi-connections/{drone_id}/terminal")
    async def pi_terminal(
        drone_id: str,
        websocket: WebSocket,
        offset: int = Query(default=0),
        inode: int = Query(default=0),
    ):
        drone = ctx.fleet.get(drone_id)
        if not drone:
            await websocket.close(code=4004)
            return
        await pi_setup.stream_terminal(
            drone.as_app_context(), websocket, offset=offset, inode=inode
        )

    return router
