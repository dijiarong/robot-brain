"""Cloud WebSocket signaling client + orchestration.

Optimizations:
- Shared _Go2MediaCache: transceiver handles resolved once, reused across
  all browser peer attaches.
- Reconnection: when Go2 WebRTC drops, the gateway logs the event and closes
  browser peers gracefully. A future iteration can add full reconnect-with-
  reattach (the transport already supports it).
"""
from __future__ import annotations

import asyncio
import json
import logging
import ssl
from typing import Any

from config.settings import Settings
from robot_brain.actuation.unitree_webrtc import UnitreeWebRTCTransport
from robot_brain.gateway.browser_peer import BrowserPeerManager
from robot_brain.gateway.media_bridge import _Go2MediaCache
from robot_brain.gateway.teleop_bridge import TeleopBridge
from robot_brain.teleop.session import TeleopSession

logger = logging.getLogger(__name__)

# WebSocket signaling reconnect delay (seconds).
_WS_RECONNECT_DELAY = 3.0


def _ws_connect_kwargs(url: str, settings: Settings) -> dict[str, Any]:
    kwargs: dict[str, Any] = {"proxy": None}
    if url.startswith("wss://") and settings.gateway_signaling_insecure_ssl:
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        kwargs["ssl"] = ctx
    return kwargs


class RobotGateway:
    """Single process: Go2 WebRTC + browser WebRTC + TeleopSession safety.

    Lifecycle:
    1. Connect Go2 WebRTC (provided externally, already connected).
    2. Connect cloud WebSocket signaling.
    3. Accept browser offers → create peer → relay Go2 media + teleop.
    4. On Go2 disconnect: close browser peers, signal error.
    """

    def __init__(
        self,
        transport: UnitreeWebRTCTransport,
        session: TeleopSession,
        settings: Settings,
    ) -> None:
        self._transport = transport
        self._session = session
        self._settings = settings
        self._peers: BrowserPeerManager | None = None
        self._media_cache: _Go2MediaCache | None = None
        self._ws: Any = None
        self._main_loop: asyncio.AbstractEventLoop | None = None
        self._out_queue: asyncio.Queue[tuple[str, str, Any]] = asyncio.Queue()

    async def run(self) -> None:
        import websockets

        self._main_loop = asyncio.get_running_loop()
        url = self._settings.gateway_signaling_url
        robot_id = self._settings.gateway_robot_id
        logger.info("[Gateway] connecting signaling %s as %s", url, robot_id)

        # Bypass system HTTP/SOCKS proxy — LAN/cloud signaling must connect directly.
        async with websockets.connect(url, **_ws_connect_kwargs(url, self._settings)) as ws:
            self._ws = ws
            await ws.send(
                json.dumps(
                    {"type": "login", "source": robot_id, "target": "", "data": None}
                )
            )
            ack = json.loads(await ws.recv())
            logger.info("[Gateway] signaling login ack: %s", ack.get("type"))

            go2_conn = self._transport.webrtc_conn
            if go2_conn is None:
                raise RuntimeError("Go2 WebRTC not connected")

            # Shared media cache: resolve transceivers once, reuse across attaches.
            self._media_cache = _Go2MediaCache(go2_conn)
            self._media_cache.resolve()

            turn = self._settings.gateway_turn_url or "(none — LAN only)"
            logger.info("[Gateway] browser ICE: STUN + TURN %s", turn)

            self._peers = BrowserPeerManager(
                go2_conn=go2_conn,
                media_cache=self._media_cache,
                teleop_factory=lambda uid: TeleopBridge(self._session, uid),
                settings=self._settings,
                on_signal_out=self._signal_out,
            )

            sender = asyncio.create_task(self._signal_sender())
            try:
                async for raw in ws:
                    await self._handle_signal(json.loads(raw))
            finally:
                sender.cancel()
                if self._peers is not None:
                    await self._transport.run_on_conn_loop(
                        self._peers.close_all(),
                        timeout=15.0,
                    )

    async def _signal_out(self, msg_type: str, target: str, data: Any) -> None:
        """Enqueue outbound signaling; safe from Go2 background loop or main loop."""
        loop = asyncio.get_running_loop()
        if self._main_loop is not None and loop is not self._main_loop:
            await asyncio.wrap_future(
                asyncio.run_coroutine_threadsafe(
                    self._enqueue_signal(msg_type, target, data),
                    self._main_loop,
                )
            )
            return
        await self._enqueue_signal(msg_type, target, data)

    async def _enqueue_signal(self, msg_type: str, target: str, data: Any) -> None:
        await self._out_queue.put((msg_type, target, data))

    async def _signal_sender(self) -> None:
        robot_id = self._settings.gateway_robot_id
        while True:
            msg_type, target, data = await self._out_queue.get()
            if self._ws is None:
                continue
            try:
                await self._ws.send(
                    json.dumps(
                        {
                            "type": msg_type,
                            "source": robot_id,
                            "target": target,
                            "data": data,
                        }
                    )
                )
            except Exception as exc:
                logger.warning("[Gateway] signal send failed: %s", exc)

    async def _handle_signal(self, msg: dict[str, Any]) -> None:
        if self._peers is None:
            return
        msg_type = msg.get("type")
        source = msg.get("source") or ""
        data = msg.get("data")

        if msg_type == "offer":
            await self._transport.run_on_conn_loop(
                self._peers.handle_offer(source, data),
                timeout=60.0,
            )
        elif msg_type == "candidate" and data:
            await self._transport.run_on_conn_loop(
                self._peers.handle_candidate(source, data),
                timeout=10.0,
            )
