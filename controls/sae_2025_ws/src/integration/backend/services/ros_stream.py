from __future__ import annotations

import asyncio
import logging
import threading
from collections import defaultdict
from typing import AsyncIterator

logger = logging.getLogger(__name__)

_node = None
_executor_thread: threading.Thread | None = None
_main_loop: asyncio.AbstractEventLoop | None = None
_lock = threading.Lock()
# topic -> list of asyncio.Queue[bytes | None]
_queues: dict[str, list[asyncio.Queue]] = defaultdict(list)
# topic -> subscription handle
_subs: dict[str, object] = {}
# topic -> frame count (for diagnostics)
_frame_counts: dict[str, int] = defaultdict(int)
_available = False


def ros_init(loop: asyncio.AbstractEventLoop) -> None:
    global _node, _executor_thread, _main_loop, _available
    _main_loop = loop
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        rclpy.init()
        _node = rclpy.create_node("integration_stream")
        executor = SingleThreadedExecutor()
        executor.add_node(_node)

        def _spin():
            try:
                executor.spin()
            except Exception:
                pass

        _executor_thread = threading.Thread(target=_spin, daemon=True)
        _executor_thread.start()
        _available = True
        logger.info("ROS 2 streaming node initialized")
    except Exception as exc:
        logger.warning("ROS 2 not available, camera streams disabled: %s", exc)


def ros_shutdown() -> None:
    global _node, _available
    _available = False
    if _node is not None:
        try:
            import rclpy

            _node.destroy_node()
            rclpy.try_shutdown()
        except Exception:
            pass


def is_available() -> bool:
    return _available


def _dispatch(topic: str, frame: bytes) -> None:
    loop = _main_loop
    if loop is None:
        return
    with _lock:
        queues = list(_queues.get(topic, []))
    for q in queues:
        try:
            # Drop oldest frame if queue is full to avoid stale frames
            if q.full():
                try:
                    q.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            loop.call_soon_threadsafe(q.put_nowait, frame)
        except Exception:
            pass


def _make_compressed_callback(topic: str):
    def _callback(msg) -> None:
        try:
            data = bytes(msg.data)
            _frame_counts[topic] += 1
            if _frame_counts[topic] == 1:
                logger.info("First frame received on %s (format=%s, size=%d bytes)", topic, getattr(msg, 'format', '?'), len(data))
            elif _frame_counts[topic] % 100 == 0:
                logger.info("Frame count on %s: %d", topic, _frame_counts[topic])
            _dispatch(topic, data)
        except Exception as exc:
            logger.warning("Compressed image dispatch failed for %s: %s", topic, exc)

    return _callback


def _ensure_subscription(topic: str) -> None:
    global _node
    if _node is None:
        return
    with _lock:
        if topic in _subs:
            return
        try:
            from sensor_msgs.msg import CompressedImage

            sub = _node.create_subscription(
                CompressedImage, topic, _make_compressed_callback(topic), 5
            )
            _subs[topic] = sub
            logger.info("Subscribed to ROS topic: %s", topic)
        except Exception as exc:
            logger.warning("Could not subscribe to %s: %s", topic, exc)


async def stream_frames(topic: str) -> AsyncIterator[bytes]:
    """Async generator yielding JPEG bytes for each incoming ROS image."""
    _ensure_subscription(topic)
    queue: asyncio.Queue[bytes] = asyncio.Queue(maxsize=4)
    with _lock:
        _queues[topic].append(queue)
    try:
        while True:
            try:
                frame = await asyncio.wait_for(queue.get(), timeout=5.0)
                yield frame
            except asyncio.TimeoutError:
                # No frame yet — keep waiting (connection stays open)
                continue
    finally:
        with _lock:
            try:
                _queues[topic].remove(queue)
            except ValueError:
                pass
