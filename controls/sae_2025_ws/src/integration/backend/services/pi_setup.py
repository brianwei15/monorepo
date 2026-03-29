from __future__ import annotations

import asyncio
import base64
import logging
import re
from contextlib import suppress

logger = logging.getLogger(__name__)

from fastapi import WebSocket, WebSocketDisconnect

from ..context import AppContext
from ..models import TerminalChunkMessage, TerminalInfoMessage

REMOTE_WS = "~/monorepo/controls/sae_2025_ws"


def _px4_id(drone_id: str) -> str:
    return drone_id.split("_")[-1] if "_" in drone_id else "1"


def _launch_paths(ctx: AppContext) -> dict[str, str]:
    d = ctx.config.remote_dir
    return {
        "log":  f"{d}/.pi_setup_launch.log",
        "pid":  f"{d}/.pi_setup_launch.pid",
        "pgid": f"{d}/.pi_setup_launch.pgid",
    }

# Keep old name as alias so callers don't break
_paths = _launch_paths


def _build_paths(ctx: AppContext) -> dict[str, str]:
    d = ctx.config.remote_dir
    return {
        "log":  f"{d}/.pi_setup_build.log",
        "pid":  f"{d}/.pi_setup_build.pid",
        "pgid": f"{d}/.pi_setup_build.pgid",
    }


def _is_offline(msg: str) -> bool:
    lower = (msg or "").lower()
    return "cannot find " in lower or "cannot reach " in lower or "timed out" in lower


async def _probe(ctx: AppContext, paths: dict) -> dict:
    """Generic probe: check if the process tracked by paths is running."""
    cmd = f"""
        pid_file={ctx.ssh.q(paths["pid"])}
        log_file={ctx.ssh.q(paths["log"])}
        if [ -f "$pid_file" ]; then
            pid="$(cat "$pid_file" 2>/dev/null)"
            if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                echo "RUNNING:$pid"
                exit 0
            fi
            echo "STOPPED"
            exit 0
        fi
        if [ -f "$log_file" ]; then echo "STOPPED"; else echo "NOT_STARTED"; fi
    """
    result = await ctx.ssh.run(cmd, timeout=8)
    if result.returncode != 0:
        error = ctx.ssh.friendly_error(result.stderr)
        return {
            "success": False,
            "running": False,
            "state": "offline" if _is_offline(error) else "error",
            "error": error,
        }
    out = (result.stdout or "").strip()
    if out.startswith("RUNNING:"):
        return {"success": True, "running": True, "state": "running", "pid": out.split(":", 1)[1]}
    if out == "NOT_STARTED":
        return {"success": True, "running": False, "state": "not_started"}
    return {"success": True, "running": False, "state": "stopped"}


async def _stop_process(ctx: AppContext, paths: dict, label: str) -> dict:
    """Generic stop: kill the process tracked by paths."""
    cmd = f"""
        pid_file={ctx.ssh.q(paths["pid"])}
        pgid_file={ctx.ssh.q(paths["pgid"])}
        if [ ! -f "$pid_file" ]; then echo "NOT_RUNNING"; exit 0; fi
        pid="$(cat "$pid_file" 2>/dev/null || true)"
        pgid=""
        if [ -f "$pgid_file" ]; then pgid="$(cat "$pgid_file" 2>/dev/null || true)"; fi
        target=""
        if [ -n "$pgid" ] && kill -0 "-$pgid" 2>/dev/null; then target="-$pgid";
        elif [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then target="$pid";
        else rm -f "$pid_file" "$pgid_file"; echo "NOT_RUNNING"; exit 0; fi
        kill -INT "$target" 2>/dev/null || true
        for _ in 1 2 3 4 5; do
            if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then sleep 1;
            else rm -f "$pid_file" "$pgid_file"; echo "STOPPED"; exit 0; fi
        done
        kill -KILL "$target" 2>/dev/null || true
        rm -f "$pid_file" "$pgid_file"
        echo "STOPPED"
    """
    result = await ctx.ssh.run(f"bash -lc {ctx.ssh.q(cmd)}", timeout=20)
    if result.returncode != 0:
        return {"success": False, "error": ctx.ssh.format_remote_error(result.stderr, f"Stop {label} failed")}
    return {"success": True, "output": f"{label} stopped"}


async def _start_detached(ctx: AppContext, script_src: str, paths: dict, label: str) -> dict:
    """Write script to Pi, kill any existing process tracked by paths, then start it detached."""
    script_path = paths["log"].replace(".log", ".sh")
    encoded = base64.b64encode(script_src.encode()).decode()

    write_cmd = (
        f"printf '%s' {ctx.ssh.q(encoded)} | base64 -d > {ctx.ssh.q(script_path)}"
        f" && chmod +x {ctx.ssh.q(script_path)}"
    )
    wr = await ctx.ssh.run(write_cmd, timeout=10)
    if wr.returncode != 0:
        return {"success": False, "error": ctx.ssh.format_remote_error(wr.stderr, f"Failed to write {label} script")}

    cmd = f"""
        set -e
        pid_file={ctx.ssh.q(paths["pid"])}
        pgid_file={ctx.ssh.q(paths["pgid"])}
        log_file={ctx.ssh.q(paths["log"])}

        if [ -f "$pid_file" ]; then
            old_pid="$(cat "$pid_file" 2>/dev/null || true)"
            old_pgid=""
            if [ -f "$pgid_file" ]; then
                old_pgid="$(cat "$pgid_file" 2>/dev/null || true)"
            fi
            if [ -z "$old_pgid" ] && [ -n "$old_pid" ]; then
                old_pgid="$(ps -o pgid= "$old_pid" 2>/dev/null | tr -d ' ' || true)"
            fi
            target=""
            if [ -n "$old_pgid" ] && kill -0 "-$old_pgid" 2>/dev/null; then
                target="-$old_pgid"
            elif [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null; then
                target="$old_pid"
            fi
            if [ -n "$target" ]; then
                kill -INT "$target" 2>/dev/null || true
                for _ in 1 2 3; do
                    if [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null; then sleep 1; else break; fi
                done
                if [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null; then
                    kill -KILL "$target" 2>/dev/null || true
                fi
            fi
            rm -f "$pid_file"
        fi
        rm -f "$pgid_file"

        : > "$log_file"

        nohup {ctx.ssh.q(script_path)} >> "$log_file" 2>&1 < /dev/null &

        new_pid=$!
        new_pgid="$(ps -o pgid= "$new_pid" 2>/dev/null | tr -d ' ' || true)"
        echo "$new_pid" > "$pid_file"
        if [ -n "$new_pgid" ]; then echo "$new_pgid" > "$pgid_file"; fi
        sleep 1

        if kill -0 "$new_pid" 2>/dev/null; then
            echo "STARTED:$new_pid"
            exit 0
        fi
        echo "FAILED"
        exit 1
    """

    result = await ctx.ssh.run(f"bash -lc {ctx.ssh.q(cmd)}", timeout=20)
    stdout = (result.stdout or "").strip()
    stderr = (result.stderr or "").strip()
    combined = "\n".join(p for p in (stdout, stderr) if p)
    match = re.search(r"STARTED\s*:\s*([0-9]+)", combined)
    if match:
        return {"success": True, "output": f"{label} started", "running": True, "pid": match.group(1)}
    error = ctx.ssh.format_remote_error(result.stderr or result.stdout, f"{label} failed to start")
    return {"success": False, "error": error}


async def probe_status(ctx: AppContext) -> dict:
    return await _probe(ctx, _launch_paths(ctx))


async def probe_build_status(ctx: AppContext) -> dict:
    return await _probe(ctx, _build_paths(ctx))


async def build(ctx: AppContext, drone_id: str) -> dict:
    """Run only the colcon build step (no launch)."""
    px4_id = _px4_id(drone_id)
    logger.info("[pi_setup.build] drone_id=%r px4_id=%r", drone_id, px4_id)
    paths = _build_paths(ctx)

    script_src = f"""\
#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd ~/monorepo/controls/sae_2025_ws
echo "=== Building uav package (px4_id={px4_id}) ==="
rm -rf build/uav install/uav
colcon build --packages-select uav
if [ $? -ne 0 ]; then
    echo "=== BUILD FAILED ==="
    exit 1
fi
echo "=== Build complete ==="
"""
    result = await _start_detached(ctx, script_src, paths, "build")
    if result.get("success"):
        result["output"] = f"uav build started (px4_id={px4_id})"
    return result


async def launch(ctx: AppContext, drone_id: str, config=None) -> dict:
    """Run only the ros2 launch step (assumes package is already built)."""
    from ..models import PiLaunchConfig
    if config is None:
        config = PiLaunchConfig()
    px4_id = config.px4_id if config.px4_id else _px4_id(drone_id)
    thermal = config.thermal if config.thermal is not None else (px4_id == "1")
    thermal_flag = "true" if thermal else "false"
    use_camera_flag = "true" if config.use_camera else "false"
    use_mavproxy_flag = "true" if config.use_mavproxy else "false"
    use_xrce_flag = "true" if config.use_xrce else "false"
    telem = config.telem or "telem2"
    proxy_ip = config.proxy_ip or "10.42.0.1"
    udp_port = int(config.udp_port or 14550)
    logger.info(
        "[pi_setup.launch] drone_id=%r px4_id=%r telem=%r proxy_ip=%r udp_port=%r thermal=%r use_camera=%r use_mavproxy=%r use_xrce=%r",
        drone_id, px4_id, telem, proxy_ip, udp_port, thermal_flag, use_camera_flag, use_mavproxy_flag, use_xrce_flag,
    )
    paths = _launch_paths(ctx)

    script_src = f"""\
#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd ~/monorepo/controls/sae_2025_ws
if [ ! -f install/uav/share/uav/package.xml ]; then
    echo "=== ERROR: uav package not built — run Build first ==="
    exit 1
fi
echo "=== Launching pi-setup (px4_id={px4_id} telem={telem} proxy={proxy_ip}:{udp_port}) ==="
source install/setup.bash
exec ros2 launch uav pi-setup.launch.py px4_id:={px4_id} telem:={telem} proxy_ip:={proxy_ip} udp_port:={udp_port} thermal:={thermal_flag} use_camera:={use_camera_flag} use_mavproxy:={use_mavproxy_flag} use_xrce:={use_xrce_flag}
"""
    result = await _start_detached(ctx, script_src, paths, "launch")
    if result.get("success"):
        result["output"] = f"pi-setup launched (px4_id={px4_id})"
    return result


async def stop(ctx: AppContext) -> dict:
    return await _stop_process(ctx, _launch_paths(ctx), "launch")


async def stop_build(ctx: AppContext) -> dict:
    return await _stop_process(ctx, _build_paths(ctx), "build")


async def get_logs(
    ctx: AppContext,
    *,
    offset: int | None = None,
    inode: int | None = None,
    log_type: str = "launch",
) -> dict:
    paths = _build_paths(ctx) if log_type == "build" else _launch_paths(ctx)
    log_file = paths["log"]

    if offset is not None:
        start = max(0, int(offset))
        inode_value = max(0, int(inode or 0))
        cmd = f"""
            log_file={ctx.ssh.q(log_file)}
            start={start}
            req_inode={inode_value}
            if [ ! -f "$log_file" ]; then echo "__META__:0:0:0:1"; exit 0; fi
            inode_now="$(stat -c %i "$log_file" 2>/dev/null || stat -f %i "$log_file" 2>/dev/null || echo 0)"
            size="$(wc -c < "$log_file" | tr -d ' ')"
            reset=0
            if [ "$req_inode" -gt 0 ] && [ "$inode_now" -ne "$req_inode" ]; then start=0; reset=1; fi
            if [ "$start" -gt "$size" ]; then start=0; reset=1; fi
            echo "__META__:$size:$reset:$inode_now:0"
            if [ "$size" -gt "$start" ]; then tail -c +$((start + 1)) "$log_file"; fi
        """
        result = await ctx.ssh.run(cmd, timeout=10)
        if result.returncode != 0:
            return {"success": False, "error": ctx.ssh.friendly_error(result.stderr)}
        out = result.stdout or ""
        first_line, sep, rest = out.partition("\n")
        meta_match = re.match(r"^__META__:(\d+):([01]):(\d+):([01])$", first_line.strip())
        if not meta_match:
            return {"success": False, "error": "Failed to parse log metadata"}
        return {
            "success": True,
            "logs": rest if sep else "",
            "next_offset": int(meta_match.group(1)),
            "inode": int(meta_match.group(3)),
            "reset": meta_match.group(2) == "1",
        }

    cmd = f"""
        log_file={ctx.ssh.q(log_file)}
        if [ -f "$log_file" ]; then tail -n 500 "$log_file"; fi
    """
    result = await ctx.ssh.run(cmd, timeout=10)
    if result.returncode != 0:
        return {"success": False, "error": ctx.ssh.friendly_error(result.stderr)}
    return {"success": True, "logs": result.stdout or ""}


async def stream_terminal(
    ctx: AppContext,
    websocket: WebSocket,
    *,
    offset: int = 0,
    inode: int = 0,
    log_type: str = "launch",
) -> None:
    await websocket.accept()
    current_offset = max(0, offset)
    current_inode = max(0, inode)

    try:
        while True:
            chunk = await get_logs(ctx, offset=current_offset, inode=current_inode, log_type=log_type)
            if not chunk.get("success"):
                msg = TerminalInfoMessage(
                    type="error",
                    message=chunk.get("error") or "Terminal stream failed",
                )
                await websocket.send_text(msg.model_dump_json())
                if _is_offline(chunk.get("error") or ""):
                    break
                await asyncio.sleep(0.5)
                continue

            text = chunk.get("logs") or ""
            next_offset = int(chunk.get("next_offset") or current_offset)
            next_inode = int(chunk.get("inode") or current_inode)
            reset = bool(chunk.get("reset"))

            if text or reset:
                payload = TerminalChunkMessage(
                    data=text,
                    next_offset=next_offset,
                    inode=next_inode,
                    reset=reset,
                )
                await websocket.send_text(payload.model_dump_json())

            current_offset = next_offset
            current_inode = next_inode

            with suppress(asyncio.TimeoutError):
                await asyncio.wait_for(websocket.receive_text(), timeout=0.01)
            await asyncio.sleep(0.5)

    except WebSocketDisconnect:
        return
    except Exception as exc:
        with suppress(Exception):
            msg = TerminalInfoMessage(type="error", message=ctx.ssh.friendly_error(str(exc)))
            await websocket.send_text(msg.model_dump_json())
    finally:
        with suppress(Exception):
            await websocket.close()
