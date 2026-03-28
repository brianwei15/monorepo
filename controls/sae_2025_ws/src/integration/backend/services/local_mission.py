from __future__ import annotations

import asyncio
import os
import re
import shlex
from contextlib import suppress
from pathlib import Path

from fastapi import WebSocket, WebSocketDisconnect

from ..context import AppContext
from ..models import TerminalChunkMessage, TerminalInfoMessage


MISSION_NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")


def _normalize_mission_name(name: str) -> str:
    mission = (name or "").strip()
    if not mission:
        raise ValueError("Mission name is required")
    if not MISSION_NAME_RE.fullmatch(mission):
        raise ValueError(
            "Invalid mission name. Use letters, numbers, underscores, or dashes."
        )
    return mission


def _local_ws(ctx: AppContext) -> Path:
    """Local ROS2 workspace root — uses local_ws_dir config if set, otherwise auto-detects."""
    if ctx.config.local_ws_dir:
        return Path(ctx.config.local_ws_dir)
    return ctx.base_dir.parent.parent


def _local_paths(ctx: AppContext, drone_id: str) -> dict[str, str]:
    ws = _local_ws(ctx)
    return {
        "log": str(ws / f".mission_{drone_id}.log"),
        "pid": str(ws / f".mission_{drone_id}.pid"),
        "pgid": str(ws / f".mission_{drone_id}.pgid"),
        "launch_params": str(ws / "src/uav/launch/launch_params.yaml"),
        "missions_dir": str(ws / "src/uav/uav/missions"),
    }


async def _run_local(cmd: str, timeout: float = 15.0) -> tuple[int, str, str]:
    try:
        proc = await asyncio.create_subprocess_shell(
            cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        try:
            stdout_b, stderr_b = await asyncio.wait_for(
                proc.communicate(), timeout=timeout
            )
        except asyncio.TimeoutError:
            with suppress(Exception):
                proc.kill()
                await proc.communicate()
            return 1, "", "Command timed out"
        return (
            proc.returncode or 0,
            stdout_b.decode(errors="replace"),
            stderr_b.decode(errors="replace"),
        )
    except Exception as exc:
        return 1, "", str(exc)


def _pid_alive(pid_str: str) -> bool:
    try:
        os.kill(int(pid_str), 0)
        return True
    except (OSError, ValueError):
        return False


async def probe_launch_status(ctx: AppContext, drone_id: str) -> dict:
    paths = _local_paths(ctx, drone_id)
    pid_file = Path(paths["pid"])
    log_file = Path(paths["log"])

    if pid_file.exists():
        try:
            pid = pid_file.read_text().strip()
            if pid and _pid_alive(pid):
                return {
                    "success": True,
                    "running": True,
                    "state": "running",
                    "pid": pid,
                }
        except Exception:
            pass
        return {"success": True, "running": False, "state": "stopped"}

    if log_file.exists():
        return {"success": True, "running": False, "state": "stopped"}

    return {"success": True, "running": False, "state": "not_prepared"}


async def refresh_runtime_state(ctx: AppContext, drone_id: str) -> dict:
    status = await probe_launch_status(ctx, drone_id)
    await ctx.mission_state.apply_launch_status(
        success=status.get("success", False),
        state=status.get("state", "error"),
        running=status.get("running", False),
        pid=status.get("pid"),
        error=status.get("error"),
    )
    return status


async def mission_state(ctx: AppContext, drone_id: str) -> dict:
    await refresh_runtime_state(ctx, drone_id)
    state = await ctx.mission_state.snapshot()
    return {"success": True, "state": state.model_dump()}


async def launch_logs(
    ctx: AppContext,
    drone_id: str,
    *,
    lines: int = 200,
    offset: int | None = None,
    inode: int | None = None,
) -> dict:
    try:
        status = await refresh_runtime_state(ctx, drone_id)
        log_file = Path(_local_paths(ctx, drone_id)["log"])

        if offset is not None:
            start = max(0, int(offset))
            inode_value = max(0, int(inode or 0))

            if not log_file.exists():
                return {
                    "success": True,
                    "running": status.get("running", False),
                    "logs": "",
                    "next_offset": 0,
                    "inode": 0,
                    "reset": False,
                }

            stat = log_file.stat()
            inode_now = stat.st_ino
            size = stat.st_size
            reset = False

            if inode_value > 0 and inode_now != inode_value:
                start = 0
                reset = True
            if start > size:
                start = 0
                reset = True

            data = ""
            if size > start:
                with open(log_file, "rb") as f:
                    f.seek(start)
                    data = f.read(size - start).decode(errors="replace")

            return {
                "success": True,
                "running": status.get("running", False),
                "logs": data,
                "next_offset": size,
                "inode": inode_now,
                "reset": reset,
            }

        if not log_file.exists():
            return {
                "success": True,
                "running": status.get("running", False),
                "logs": "",
            }

        line_count = max(20, min(lines, 20000))
        rc, stdout, _ = await _run_local(
            f"tail -n {line_count} {shlex.quote(str(log_file))}",
            timeout=10,
        )
        return {
            "success": True,
            "running": status.get("running", False),
            "logs": stdout,
        }
    except Exception as exc:
        return {"success": False, "running": False, "error": str(exc)}


async def stream_terminal(
    ctx: AppContext,
    drone_id: str,
    websocket: WebSocket,
    *,
    offset: int = 0,
    inode: int = 0,
) -> None:
    await websocket.accept()
    current_offset = max(0, offset)
    current_inode = max(0, inode)

    try:
        while True:
            chunk = await launch_logs(
                ctx,
                drone_id,
                offset=current_offset,
                inode=current_inode,
            )
            if not chunk.get("success"):
                message = TerminalInfoMessage(
                    type="error",
                    message=chunk.get("error") or "Terminal stream failed",
                )
                await websocket.send_text(message.model_dump_json())
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
            message = TerminalInfoMessage(type="error", message=str(exc))
            await websocket.send_text(message.model_dump_json())
    finally:
        with suppress(Exception):
            await websocket.close()


async def prepare_mission(ctx: AppContext, drone_id: str) -> dict:
    await ctx.mission_state.set(
        phase="preparing",
        launch_state="running",
        running=False,
        message="Starting mission launch",
        error=None,
    )
    try:
        paths = _local_paths(ctx, drone_id)
        ws = shlex.quote(str(_local_ws(ctx)))
        pid_file = shlex.quote(paths["pid"])
        pgid_file = shlex.quote(paths["pgid"])
        log_file = shlex.quote(paths["log"])

        launch_extra_args = ""
        px4_path = ctx.config.px4_path
        if not px4_path:
            import subprocess as _sp
            try:
                result = _sp.run(
                    ["find", os.path.expanduser("~"), "-maxdepth", "6",
                     "-name", "PX4-Autopilot", "-type", "d"],
                    capture_output=True, text=True, timeout=10,
                )
                found = result.stdout.strip().splitlines()
                if found:
                    px4_path = found[0]
            except Exception:
                pass
        if px4_path:
            launch_extra_args += f" px4_path:={shlex.quote(px4_path)}"

        cmd = f"""
            set -e
            cd {ws}
            source /opt/ros/humble/setup.bash
            source install/setup.bash

            pid_file={pid_file}
            pgid_file={pgid_file}
            log_file={log_file}
            had_previous=0

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
                    had_previous=1
                    kill -INT "$target" 2>/dev/null || true
                    for _ in 1 2 3 4 5; do
                        if [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null; then
                            sleep 1
                        else
                            break
                        fi
                    done
                    if [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null; then
                        kill -TERM "$target" 2>/dev/null || true
                        sleep 1
                    fi
                    if [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null; then
                        kill -KILL "$target" 2>/dev/null || true
                    fi
                fi

                rm -f "$pid_file"
            fi
            rm -f "$pgid_file"

            : > "$log_file"
            nohup setsid ros2 launch uav main.launch.py{launch_extra_args} >> "$log_file" 2>&1 < /dev/null &
            new_pid=$!
            new_pgid="$(ps -o pgid= "$new_pid" 2>/dev/null | tr -d ' ' || true)"
            echo "$new_pid" > "$pid_file"
            if [ -n "$new_pgid" ]; then
                echo "$new_pgid" > "$pgid_file"
            fi
            sleep 1

            if kill -0 "$new_pid" 2>/dev/null; then
                if [ "$had_previous" -eq 1 ]; then
                    echo "RESTARTED:$new_pid"
                else
                    echo "STARTED:$new_pid"
                fi
                exit 0
            fi

            echo "FAILED"
            exit 1
        """
        rc, stdout, stderr = await _run_local(
            f"bash -lc {shlex.quote(cmd)}", timeout=30
        )
        combined = "\n".join(p for p in (stdout.strip(), stderr.strip()) if p)
        match = re.search(r"(STARTED|RESTARTED)\s*:\s*([0-9]+)", combined)
        if match:
            pid = match.group(2)
            await ctx.mission_state.set(
                phase="running",
                launch_state="running",
                running=True,
                pid=pid,
                message="Mission launch running",
                error=None,
            )
            label = "restarted" if match.group(1) == "RESTARTED" else "started"
            return {
                "success": True,
                "output": f"Mission launch {label}",
                "running": True,
                "pid": pid,
            }

        error = f"Prepare mission failed: {combined or 'launch did not start'}"
        await ctx.mission_state.set(
            phase="error",
            launch_state="error",
            running=False,
            pid=None,
            error=error,
        )
        return {"success": False, "error": error}
    except Exception as exc:
        error = str(exc)
        await ctx.mission_state.set(
            phase="error",
            launch_state="error",
            running=False,
            pid=None,
            error=error,
        )
        return {"success": False, "error": error}


async def stop_mission(ctx: AppContext, drone_id: str) -> dict:
    await ctx.mission_state.set(
        phase="stopping",
        launch_state="stopped",
        running=True,
        message="Stopping mission launch",
        error=None,
    )
    try:
        paths = _local_paths(ctx, drone_id)
        pid_file = shlex.quote(paths["pid"])
        pgid_file = shlex.quote(paths["pgid"])

        cmd = f"""
            pid_file={pid_file}
            pgid_file={pgid_file}

            if [ ! -f "$pid_file" ]; then
                echo "NOT_RUNNING"
                exit 0
            fi

            pid="$(cat "$pid_file" 2>/dev/null || true)"
            pgid=""
            if [ -f "$pgid_file" ]; then
                pgid="$(cat "$pgid_file" 2>/dev/null || true)"
            fi

            target=""
            if [ -n "$pgid" ] && kill -0 "-$pgid" 2>/dev/null; then
                target="-$pgid"
            elif [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                target="$pid"
            else
                rm -f "$pid_file" "$pgid_file"
                echo "NOT_RUNNING"
                exit 0
            fi

            kill -INT "$target" 2>/dev/null || true
            for _ in 1 2 3 4 5; do
                if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                    sleep 1
                else
                    rm -f "$pid_file" "$pgid_file"
                    echo "STOPPED"
                    exit 0
                fi
            done

            kill -TERM "$target" 2>/dev/null || true
            sleep 1
            if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                kill -KILL "$target" 2>/dev/null || true
            fi
            rm -f "$pid_file" "$pgid_file"
            echo "STOPPED"
        """
        rc, stdout, stderr = await _run_local(
            f"bash -lc {shlex.quote(cmd)}", timeout=20
        )
        if rc != 0:
            error = (stderr or stdout or "Stop mission failed").strip()
            await ctx.mission_state.set(
                phase="error",
                launch_state="error",
                running=False,
                pid=None,
                error=error,
            )
            return {"success": False, "error": error}

        await ctx.mission_state.set(
            phase="idle",
            launch_state="stopped",
            running=False,
            pid=None,
            message="Mission launch stopped",
            error=None,
        )
        if "NOT_RUNNING" in stdout:
            return {"success": True, "output": "Mission launch is not running"}
        return {"success": True, "output": "Mission launch stopped"}
    except Exception as exc:
        error = str(exc)
        await ctx.mission_state.set(
            phase="error",
            launch_state="error",
            running=False,
            pid=None,
            error=error,
        )
        return {"success": False, "error": error}


async def start_mission(ctx: AppContext, drone_id: str) -> dict:
    try:
        ws = shlex.quote(str(_local_ws(ctx)))
        cmd = (
            f"cd {ws} && "
            "source /opt/ros/humble/setup.bash && "
            "source install/setup.bash && "
            'ros2 service call /mode_manager/start_mission std_srvs/srv/Trigger "{}"'
        )
        rc, stdout, stderr = await _run_local(
            f"bash -lc {shlex.quote(cmd)}", timeout=20
        )
        if rc != 0:
            error = (stderr or stdout or "Start mission failed").strip()
            await ctx.mission_state.set(
                phase="error",
                launch_state="error",
                running=False,
                pid=None,
                error=error,
            )
            return {"success": False, "error": error}

        await refresh_runtime_state(ctx, drone_id)
        return {
            "success": True,
            "output": stdout.strip() or "Start mission service called",
        }
    except Exception as exc:
        error = str(exc)
        await ctx.mission_state.set(
            phase="error",
            launch_state="error",
            running=False,
            pid=None,
            error=error,
        )
        return {"success": False, "error": error}


async def trigger_failsafe(ctx: AppContext, drone_id: str) -> dict:
    try:
        ws = shlex.quote(str(_local_ws(ctx)))
        cmd = (
            f"cd {ws} && "
            "source /opt/ros/humble/setup.bash && "
            "source install/setup.bash && "
            'ros2 service call /mode_manager/failsafe std_srvs/srv/Trigger "{}"'
        )
        rc, stdout, stderr = await _run_local(
            f"bash -lc {shlex.quote(cmd)}", timeout=15
        )
        if rc not in (0, 124):
            return {"success": False, "error": (stderr or "Failsafe command failed").strip()}
        return {"success": True, "output": "Failsafe triggered"}
    except Exception as exc:
        return {"success": False, "error": str(exc)}


async def get_launch_params(ctx: AppContext, drone_id: str) -> dict:
    try:
        params_path = Path(_local_paths(ctx, drone_id)["launch_params"])
        if not params_path.exists():
            return {
                "success": False,
                "error": f"launch_params.yaml not found at {params_path}",
            }
        return {"success": True, "content": params_path.read_text()}
    except Exception as exc:
        return {"success": False, "error": str(exc)}


async def set_launch_params(ctx: AppContext, drone_id: str, *, content: str) -> dict:
    try:
        params_path = Path(_local_paths(ctx, drone_id)["launch_params"])
        params_path.parent.mkdir(parents=True, exist_ok=True)
        params_path.write_text(content)
        return {"success": True, "output": "launch_params.yaml updated"}
    except Exception as exc:
        return {"success": False, "error": str(exc)}


async def list_mission_names(ctx: AppContext, drone_id: str) -> dict:
    try:
        missions_dir = Path(_local_paths(ctx, drone_id)["missions_dir"])
        if not missions_dir.exists():
            return {
                "success": False,
                "missions": [],
                "error": f"Mission directory not found: {missions_dir}",
            }
        names = sorted({
            p.stem
            for p in missions_dir.iterdir()
            if p.suffix in (".yaml", ".yml")
        })
        return {"success": True, "missions": names}
    except Exception as exc:
        return {"success": False, "missions": [], "error": str(exc)}


async def get_mission_file(ctx: AppContext, drone_id: str, *, name: str) -> dict:
    try:
        mission_name = _normalize_mission_name(name)
    except ValueError as exc:
        return {"success": False, "mission": None, "error": str(exc)}
    try:
        missions_dir = Path(_local_paths(ctx, drone_id)["missions_dir"])
        for ext in (".yaml", ".yml"):
            p = missions_dir / f"{mission_name}{ext}"
            if p.exists():
                return {
                    "success": True,
                    "mission": mission_name,
                    "path": str(p),
                    "content": p.read_text(),
                }
        return {
            "success": False,
            "mission": mission_name,
            "error": f"Mission file '{mission_name}.yaml' not found",
        }
    except Exception as exc:
        return {"success": False, "mission": mission_name, "error": str(exc)}


async def set_mission_file(
    ctx: AppContext, drone_id: str, *, name: str, content: str
) -> dict:
    try:
        mission_name = _normalize_mission_name(name)
    except ValueError as exc:
        return {"success": False, "mission": None, "error": str(exc)}
    try:
        missions_dir = Path(_local_paths(ctx, drone_id)["missions_dir"])
        missions_dir.mkdir(parents=True, exist_ok=True)

        target = missions_dir / f"{mission_name}.yaml"
        yml_path = missions_dir / f"{mission_name}.yml"
        if yml_path.exists() and not target.exists():
            target = yml_path

        target.write_text(content)
        return {
            "success": True,
            "mission": mission_name,
            "path": str(target),
            "output": f"Mission YAML updated ({mission_name})",
        }
    except Exception as exc:
        return {"success": False, "mission": mission_name, "error": str(exc)}
