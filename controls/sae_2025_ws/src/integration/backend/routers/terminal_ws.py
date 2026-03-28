from __future__ import annotations

from fastapi import APIRouter, Query, WebSocket

from ..context import AppContext
from ..services import local_mission

_DRONE_ID = "main"


def build_router(ctx: AppContext) -> APIRouter:
    router = APIRouter(tags=["terminal"])

    @router.websocket("/ws/mission/terminal")
    async def mission_terminal_stream(
        websocket: WebSocket,
        offset: int = Query(default=0),
        inode: int = Query(default=0),
    ):
        await local_mission.stream_terminal(
            ctx,
            _DRONE_ID,
            websocket,
            offset=offset,
            inode=inode,
        )

    return router
