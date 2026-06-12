"""Web UI teleop for Unitree Go2 — hold buttons / keys for continuous motion.

Not exposed to LLM or the service API. Opens a local browser panel on 127.0.0.1.

Usage:
    python -m examples.run_unitree_teleop_web --transport fake
    RDB_UNITREE_ENABLE_MOTION=true python -m examples.run_unitree_teleop_web \\
        --transport webrtc --live --strong
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import sys
import webbrowser
from typing import Any

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse

from config.settings import Settings
from examples.run_unitree_smoke import create_transport
from examples.run_unitree_teleop import (
    POSTURES,
    TELEOP_CONFIRMATION,
    _CAR_NUDGES,
    _CAR_STRONG_NUDGES,
    _DEFAULT_NUDGES,
    _SPORT_MODE_LABELS,
    _STRONG_NUDGES,
    prep_locomotion,
)
from robot_brain.actuation.unitree import UnitreeRobot

# Max seconds per single hold gesture (safety cap).
_MAX_HOLD_SECONDS = 5.0
_DRIVE_KEYS = frozenset({"w", "s", "a", "d", "q", "e"})


def drive_channel_label(vx: float, vy: float, vyaw: float, *, omni: bool) -> str:
    """Human-readable channel hint matching transport hybrid routing."""
    if vyaw != 0.0:
        return "joystick (arc)"
    if vy != 0.0 and not omni:
        return "move(1008) strafe"
    if vy != 0.0:
        return "move(1008)"
    return "move(1008) forward"


def combine_nudge_keys(
    keys: set[str], vectors: dict[str, dict[str, float]]
) -> tuple[float, float, float]:
    """Sum per-key nudge vectors (e.g. W+D → forward + turn in car mode)."""
    vx = vy = vyaw = 0.0
    for key in keys:
        if key not in _DRIVE_KEYS:
            continue
        recipe = vectors.get(key, {})
        vx += float(recipe.get("vx", 0))
        vy += float(recipe.get("vy", 0))
        vyaw += float(recipe.get("vyaw", 0))
    return vx, vy, vyaw

_TELEOP_HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no"/>
<title>Go2 Teleop</title>
<style>
  * { box-sizing: border-box; touch-action: manipulation; user-select: none; }
  body {
    margin: 0; min-height: 100dvh; font-family: system-ui, sans-serif;
    background: #0f1419; color: #e7ecf3; display: flex; flex-direction: column;
  }
  header { padding: 12px 16px; border-bottom: 1px solid #243041; }
  header h1 { margin: 0; font-size: 1.1rem; font-weight: 600; }
  #status { margin-top: 6px; font-size: 0.85rem; color: #8fa3b8; min-height: 1.2em; }
  #status.err { color: #ff7b72; }
  main { flex: 1; display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 16px; padding: 16px; }
  .grid { display: grid; grid-template-columns: repeat(3, 72px); grid-template-rows: repeat(3, 72px); gap: 10px; }
  .btn {
    border: none; border-radius: 14px; background: #1f2a38; color: #e7ecf3;
    font-size: 1.1rem; font-weight: 600; cursor: pointer;
    box-shadow: 0 2px 0 #0b1018;
  }
  .btn:active, .btn.active { background: #3d7eff; transform: translateY(1px); box-shadow: none; }
  .btn.stop { width: min(100%, 240px); height: 56px; background: #8b2635; font-size: 1rem; margin-top: 4px; }
  .btn.posture { width: 100px; height: 44px; font-size: 0.85rem; background: #2a3544; }
  .row { display: flex; gap: 10px; flex-wrap: wrap; justify-content: center; }
  .hint { font-size: 0.78rem; color: #6d8299; text-align: center; max-width: 320px; line-height: 1.4; }
  .dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%; margin-right: 6px; background: #555; }
  .dot.on { background: #3d7eff; }
</style>
</head>
<body>
<header>
  <h1><span id="connDot" class="dot"></span>Go2 Web Teleop</h1>
  <div id="status">连接中…</div>
</header>
<main>
  <div class="grid" id="drivePad">
    <span></span><button type="button" class="btn" data-cmd="w">W</button><span></span>
    <button type="button" class="btn" data-cmd="a">A</button>
    <button type="button" class="btn" data-cmd="s">S</button>
    <button type="button" class="btn" data-cmd="d">D</button>
    <button type="button" class="btn" data-cmd="q">Q</button>
    <span></span>
    <button type="button" class="btn" data-cmd="e">E</button>
  </div>
  <button type="button" class="btn stop" id="stopBtn" data-cmd="stop">STOP / 空格</button>
  <div class="row">
    <button type="button" class="btn posture" data-posture="stand_up">站立 U</button>
    <button type="button" class="btn posture" data-posture="balance_stand">平衡 B</button>
    <button type="button" class="btn posture" data-posture="free_walk">行走 F</button>
    <button type="button" class="btn posture" data-posture="stand_down">趴下 L</button>
  </div>
  <p class="hint">车式操控：W/S 前后，A/D 转弯，Q/E 平移。W+A 或 W+D 可边走边转。松开即停。仅 localhost。</p>
</main>
<script>
const statusEl = document.getElementById("status");
const connDot = document.getElementById("connDot");
const KEY_MAP = { w:"w", s:"s", a:"a", d:"d", q:"q", e:"e", " ":"stop" };
let ws;
const held = new Set();
const ptrCmd = new Map();

function setStatus(text, err=false) {
  statusEl.textContent = text;
  statusEl.classList.toggle("err", err);
}

function send(msg) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg));
}

function refreshActiveButtons() {
  document.querySelectorAll("[data-cmd]").forEach((btn) => {
    const cmd = btn.dataset.cmd;
    if (cmd && cmd !== "stop") btn.classList.toggle("active", held.has(cmd));
  });
}

function syncDrive() {
  refreshActiveButtons();
  if (held.size === 0) {
    send({ type: "release" });
    return;
  }
  send({ type: "drive_mix", keys: [...held] });
}

function pressCmd(cmd) {
  if (cmd === "stop") {
    held.clear();
    ptrCmd.clear();
    refreshActiveButtons();
    send({ type: "stop" });
    return;
  }
  held.add(cmd);
  syncDrive();
}

function releaseCmd(cmd) {
  if (!held.has(cmd)) return;
  held.delete(cmd);
  syncDrive();
}

function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onopen = () => { connDot.classList.add("on"); setStatus("已连接 — 按住方向键移动"); send({ type: "status" }); };
  ws.onclose = () => { connDot.classList.remove("on"); setStatus("连接断开", true); setTimeout(connect, 1500); };
  ws.onmessage = (ev) => {
    try {
      const m = JSON.parse(ev.data);
      if (m.type === "status") setStatus(m.text);
      if (m.type === "drive_ack") setStatus(`drive ${m.channel} vx=${m.vx} vy=${m.vy} vyaw=${m.vyaw}`);
      if (m.type === "error") setStatus(m.text, true);
    } catch (_) {}
  };
}

document.getElementById("drivePad").addEventListener("pointerdown", (ev) => {
  const btn = ev.target.closest("[data-cmd]");
  if (!btn || btn.dataset.cmd === "stop") return;
  ev.preventDefault();
  const cmd = btn.dataset.cmd;
  ptrCmd.set(ev.pointerId, cmd);
  btn.setPointerCapture(ev.pointerId);
  pressCmd(cmd);
});
document.getElementById("drivePad").addEventListener("pointerup", (ev) => {
  const cmd = ptrCmd.get(ev.pointerId);
  ptrCmd.delete(ev.pointerId);
  if (cmd) releaseCmd(cmd);
});
document.getElementById("drivePad").addEventListener("pointercancel", (ev) => {
  const cmd = ptrCmd.get(ev.pointerId);
  ptrCmd.delete(ev.pointerId);
  if (cmd) releaseCmd(cmd);
});
document.getElementById("stopBtn").addEventListener("pointerdown", (ev) => {
  ev.preventDefault();
  pressCmd("stop");
});

document.querySelectorAll("[data-posture]").forEach((btn) => {
  btn.addEventListener("click", () => send({ type: "posture", name: btn.dataset.posture }));
});

window.addEventListener("keydown", (ev) => {
  const cmd = KEY_MAP[ev.key.toLowerCase()];
  if (!cmd || ev.repeat) return;
  ev.preventDefault();
  pressCmd(cmd);
});
window.addEventListener("keyup", (ev) => {
  const cmd = KEY_MAP[ev.key.toLowerCase()];
  if (!cmd || cmd === "stop") return;
  releaseCmd(cmd);
});

window.addEventListener("blur", () => {
  held.clear();
  ptrCmd.clear();
  refreshActiveButtons();
  send({ type: "release" });
});
connect();
</script>
</body>
</html>
"""


class HoldController:
    """Stream drive while keys are held; one continuous lease, live chord updates."""

    def __init__(self, robot: UnitreeRobot) -> None:
        self._robot = robot
        self._task: asyncio.Task[None] | None = None
        self._lock = asyncio.Lock()
        self._target_vx = 0.0
        self._target_vy = 0.0
        self._target_vyaw = 0.0

    async def set_velocity(self, vx: float, vy: float, vyaw: float) -> None:
        """Update target velocity; start/stop stream as needed."""
        async with self._lock:
            self._target_vx, self._target_vy, self._target_vyaw = vx, vy, vyaw
            moving = bool(vx or vy or vyaw)
            if not moving:
                await self._cancel_task(release=True)
                return
            if self._task is None or self._task.done():
                self._task = asyncio.create_task(self._loop())

    async def start(self, *, vx: float, vy: float, vyaw: float) -> None:
        await self.set_velocity(vx, vy, vyaw)

    async def release(self) -> None:
        """Pointer/key up — zero joystick only (DimOS semantics)."""
        async with self._lock:
            self._target_vx = self._target_vy = self._target_vyaw = 0.0
            await self._cancel_task(release=True)

    async def estop(self) -> None:
        """STOP button — release joystick then StopMove."""
        async with self._lock:
            self._target_vx = self._target_vy = self._target_vyaw = 0.0
            await self._cancel_task(release=True)
            await self._robot.stop("web teleop e-stop")

    async def _cancel_task(self, *, release: bool) -> None:
        task = self._task
        self._task = None
        if task is not None and not task.done():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        if release:
            await self._robot.release_drive("web teleop release")

    async def _loop(self) -> None:
        loop = asyncio.get_event_loop()

        def live_velocity() -> tuple[float, float, float]:
            return self._target_vx, self._target_vy, self._target_vyaw

        try:
            while self._target_vx or self._target_vy or self._target_vyaw:
                session_end = loop.time() + _MAX_HOLD_SECONDS
                await self._robot.stream_hold(
                    live_velocity,
                    session_deadline=session_end,
                )
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            logging.warning("Hold stream stopped: %s", exc)
        finally:
            if self._task is asyncio.current_task():
                self._task = None
            if not (self._target_vx or self._target_vy or self._target_vyaw):
                await self._robot.release_drive("web teleop hold end")


def _schedule_hold(
    coro: Any,
    websocket: WebSocket | None = None,
) -> None:
    """Run hold/robot I/O off the WebSocket receive loop so the UI stays responsive."""

    async def _runner() -> None:
        try:
            await coro
        except Exception as exc:
            logging.exception("Hold command failed")
            if websocket is not None:
                try:
                    await websocket.send_json({"type": "error", "text": str(exc)})
                except Exception:
                    pass

    asyncio.create_task(_runner())


def build_app(
    robot: UnitreeRobot,
    hold: HoldController,
    vectors: dict[str, dict[str, float]],
    *,
    omni: bool = False,
) -> FastAPI:
    app = FastAPI(title="Go2 Web Teleop", docs_url=None, redoc_url=None)

    @app.get("/", response_class=HTMLResponse)
    async def index() -> str:
        return _TELEOP_HTML

    @app.websocket("/ws")
    async def ws_endpoint(websocket: WebSocket) -> None:
        await websocket.accept()
        try:
            while True:
                raw = await websocket.receive_text()
                msg = json.loads(raw)
                kind = msg.get("type")

                if kind == "drive_mix":
                    raw_keys = msg.get("keys", [])
                    keys = {str(k) for k in raw_keys if str(k) in _DRIVE_KEYS}
                    vx, vy, vyaw = combine_nudge_keys(keys, vectors)
                    channel = drive_channel_label(vx, vy, vyaw, omni=omni)
                    await websocket.send_json(
                        {
                            "type": "drive_ack",
                            "vx": round(vx, 3),
                            "vy": round(vy, 3),
                            "vyaw": round(vyaw, 3),
                            "channel": channel,
                            "keys": sorted(keys),
                        }
                    )
                    _schedule_hold(
                        hold.set_velocity(vx=vx, vy=vy, vyaw=vyaw), websocket
                    )

                elif kind == "drive":
                    cmd = str(msg.get("cmd", ""))
                    if cmd in _DRIVE_KEYS:
                        vx, vy, vyaw = combine_nudge_keys({cmd}, vectors)
                        _schedule_hold(
                            hold.set_velocity(vx=vx, vy=vy, vyaw=vyaw), websocket
                        )

                elif kind == "stop":
                    _schedule_hold(hold.estop(), websocket)

                elif kind == "release":
                    _schedule_hold(hold.release(), websocket)

                elif kind == "posture":
                    name = str(msg.get("name", ""))
                    if name not in POSTURES.values():
                        await websocket.send_json(
                            {"type": "error", "text": f"unknown posture: {name}"}
                        )
                        continue

                    async def _posture() -> None:
                        await hold.release()
                        await robot.set_posture(name)
                        await websocket.send_json(
                            {"type": "status", "text": f"姿态: {name}"}
                        )

                    _schedule_hold(_posture(), websocket)

                elif kind == "status":

                    async def _status() -> None:
                        await hold.release()
                        await robot.get_state()
                        raw_state = robot.action_history[-1].get("raw", {})
                        mode = raw_state.get("sport_mode")
                        label = (
                            _SPORT_MODE_LABELS.get(mode, "?")
                            if mode is not None
                            else "?"
                        )
                        text = (
                            f"sport_mode={mode} ({label}) "
                            f"battery={raw_state.get('battery_level', '?')}% "
                            f"error={raw_state.get('error_code', 0)}"
                        )
                        await websocket.send_json({"type": "status", "text": text})

                    _schedule_hold(_status(), websocket)

        except WebSocketDisconnect:
            await hold.release()
        except Exception as exc:
            logging.exception("WebSocket error")
            await hold.release()
            try:
                await websocket.send_json({"type": "error", "text": str(exc)})
            except Exception:
                pass

    return app


async def run_server(app: FastAPI, host: str, port: int) -> None:
    config = uvicorn.Config(app, host=host, port=port, log_level="warning")
    server = uvicorn.Server(config)
    await server.serve()


async def main() -> None:
    logging.basicConfig(level=logging.WARNING)
    logging.getLogger("robot_brain").setLevel(logging.INFO)

    parser = argparse.ArgumentParser(description="Go2 web teleop (hold-to-move panel)")
    parser.add_argument("--transport", choices=["fake", "webrtc"], default="fake")
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--robot-ip", default="")
    parser.add_argument("--prep-stand", action="store_true", default=None)
    parser.add_argument("--no-prep-stand", action="store_true")
    parser.add_argument("--strong", action="store_true")
    parser.add_argument(
        "--omni",
        action="store_true",
        help="Omni keymap: A/D strafe, Q/E yaw (default is car: A/D turn, Q/E strafe)",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    if args.no_prep_stand:
        prep_stand_flag = False
    elif args.prep_stand:
        prep_stand_flag = True
    else:
        prep_stand_flag = args.live

    if args.transport == "webrtc" and not args.live:
        print("[ERROR] webrtc web teleop requires --live")
        sys.exit(1)

    settings_kwargs: dict = dict(
        robot_backend="unitree",
        unitree_transport=args.transport,
        unitree_dry_run=not args.live,
    )
    if args.robot_ip:
        settings_kwargs["unitree_robot_ip"] = args.robot_ip
    if args.strong and args.live:
        settings_kwargs.update(
            unitree_max_speed=0.35,
            unitree_max_yaw_speed=0.35,
            unitree_max_drive_duration=0.8,
        )
    if args.live:
        # Hold-to-move streams up to _MAX_HOLD_SECONDS per gesture.
        settings_kwargs["unitree_max_drive_duration"] = max(
            float(settings_kwargs.get("unitree_max_drive_duration", 0.5)),
            _MAX_HOLD_SECONDS,
        )
    settings = Settings(**settings_kwargs)
    if args.omni:
        vectors = _STRONG_NUDGES if args.strong else _DEFAULT_NUDGES
    else:
        vectors = _CAR_STRONG_NUDGES if args.strong else _CAR_NUDGES

    if args.transport == "webrtc" and args.live and not settings.unitree_enable_motion:
        print("[ERROR] Live webrtc requires RDB_UNITREE_ENABLE_MOTION=true")
        sys.exit(1)

    if args.live:
        print("[WARNING] Real motion — flat ground, clear space, hand on STOP.")
        confirm = input(f"Type '{TELEOP_CONFIRMATION}' to proceed: ").strip()
        if confirm != TELEOP_CONFIRMATION:
            print("[ABORT]")
            sys.exit(1)

    try:
        transport = await create_transport(args.transport, settings)
    except (RuntimeError, ConnectionError, TimeoutError) as exc:
        print(f"[ERROR] Connect failed: {exc}")
        sys.exit(1)

    robot = UnitreeRobot(transport, settings)
    hold = HoldController(robot)

    if args.live and prep_stand_flag:
        await prep_locomotion(robot)

    mode_label = "omni" if args.omni else "car (A/D turn)"
    url = f"http://{args.host}:{args.port}/"
    app = build_app(robot, hold, vectors, omni=args.omni)
    print(f"[Web Teleop] {mode_label} — open {url} — hold keys, release to stop.")
    if not args.no_browser:
        webbrowser.open(url)

    try:
        await run_server(app, args.host, args.port)
    finally:
        await hold.release()
        await robot.stop("web teleop shutdown")
        await transport.disconnect()
        print("[OK] Web teleop ended.")


if __name__ == "__main__":
    asyncio.run(main())
