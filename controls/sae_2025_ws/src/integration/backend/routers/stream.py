from __future__ import annotations

from fastapi import APIRouter, Query, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, StreamingResponse

from ..services import ros_stream


def build_router() -> APIRouter:
    router = APIRouter(tags=["stream"])

    @router.get("/api/stream/status")
    async def stream_status() -> JSONResponse:
        return JSONResponse({"available": ros_stream.is_available()})

    @router.get("/api/stream/video")
    async def video_stream(topic: str = Query(...)) -> StreamingResponse:
        if not ros_stream.is_available():
            return JSONResponse(
                {"error": "ROS 2 not available on this host"},
                status_code=503,
            )

        async def _mjpeg():
            async for frame in ros_stream.stream_frames(topic):
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + frame
                    + b"\r\n"
                )

        return StreamingResponse(
            _mjpeg(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    @router.websocket("/ws/stream/video")
    async def video_stream_ws(websocket: WebSocket, topic: str = Query(...)):
        await websocket.accept()
        try:
            async for frame in ros_stream.stream_frames(topic):
                await websocket.send_bytes(frame)
        except WebSocketDisconnect:
            pass
        except Exception:
            pass

    return router
