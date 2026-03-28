from .config import build_router as config_router
from .connection import build_router as connection_router
from .deploy import build_router as deploy_router
from .drones import build_router as drones_router
from .mission import build_router as mission_router
from .pi_connections import build_router as pi_connections_router
from .stream import build_router as stream_router
from .terminal_ws import build_router as terminal_ws_router
from .wifi import build_router as wifi_router

__all__ = [
    "config_router",
    "connection_router",
    "wifi_router",
    "deploy_router",
    "drones_router",
    "mission_router",
    "pi_connections_router",
    "stream_router",
    "terminal_ws_router",
]
