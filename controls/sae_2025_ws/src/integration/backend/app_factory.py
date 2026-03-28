from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from .context import create_context
from .routers import (
    config_router,
    connection_router,
    deploy_router,
    drones_router,
    mission_router,
    pi_connections_router,
    stream_router,
    terminal_ws_router,
    wifi_router,
)
from .services import ros_stream


def create_app(base_dir: Path) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        loop = asyncio.get_event_loop()
        ros_stream.ros_init(loop)
        yield
        ros_stream.ros_shutdown()

    app = FastAPI(title="PennAiR Auton Deploy", lifespan=lifespan)

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    ctx = create_context(base_dir)

    app.include_router(config_router(ctx))
    app.include_router(connection_router(ctx))
    app.include_router(wifi_router(ctx))
    app.include_router(deploy_router(ctx))
    app.include_router(mission_router(ctx))
    app.include_router(terminal_ws_router(ctx))
    app.include_router(drones_router(ctx))
    app.include_router(pi_connections_router(ctx))
    app.include_router(stream_router())

    frontend_dist = base_dir / "frontend" / "dist"
    if frontend_dist.exists():
        app.mount(
            "/", StaticFiles(directory=str(frontend_dist), html=True), name="frontend"
        )

    return app
