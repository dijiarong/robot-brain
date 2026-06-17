"""Browser-facing aiortc peer (answers offers from the cloud signaling hub).

Optimizations:
- Connection timeout: peers stuck in "connecting" > 30s are closed.
- Lock-guarded close to prevent races with ICE/datachannel callbacks.
- Peer creation timestamp for observability.
- Graceful shutdown with configurable drain timeout.
"""
from __future__ import annotations

import asyncio
import json
import logging
import time
from typing import Any, Callable, Awaitable

from aiortc import RTCIceServer, RTCPeerConnection, RTCSessionDescription, RTCConfiguration

from robot_brain.gateway.media_bridge import _Go2MediaCache, attach_go2_to_browser
from robot_brain.gateway.teleop_bridge import TeleopBridge

logger = logging.getLogger(__name__)

OnSignalOut = Callable[[str, str, Any], Awaitable[None]]

# Peer stuck in "connecting" longer than this is considered stale and closed.
_PEER_CONNECT_TIMEOUT = 30.0  # seconds
# Grace period after close() before the RTCPeerConnection is considered done.
_PEER_CLOSE_DRAIN = 2.0  # seconds


def _rtc_configuration(settings: Any) -> RTCConfiguration:
    servers = [RTCIceServer(urls="stun:stun.l.google.com:19302")]
    turn = getattr(settings, "gateway_turn_url", "") or ""
    if turn:
        servers.append(
            RTCIceServer(
                urls=turn,
                username=getattr(settings, "gateway_turn_user", "") or None,
                credential=getattr(settings, "gateway_turn_pass", "") or None,
            )
        )
    return RTCConfiguration(iceServers=servers)


def _parse_remote_candidate(data: dict[str, Any]) -> Any | None:
    """Parse browser trickle-ICE payload into aiortc RTCIceCandidate."""
    from aiortc.sdp import candidate_from_sdp

    cand_str = (data.get("candidate") or "").strip()
    if not cand_str:
        return None
    cand = candidate_from_sdp(cand_str)
    cand.sdpMid = data.get("sdpMid")
    cand.sdpMLineIndex = data.get("sdpMLineIndex")
    return cand


async def _wait_ice_gathering(pc: RTCPeerConnection, *, timeout: float = 15.0) -> None:
    if pc.iceGatheringState == "complete":
        return
    loop = asyncio.get_running_loop()
    done: asyncio.Future[None] = loop.create_future()

    @pc.on("icegatheringstatechange")
    async def _on_gathering() -> None:
        if pc.iceGatheringState == "complete" and not done.done():
            done.set_result(None)

    try:
        await asyncio.wait_for(done, timeout=timeout)
    except asyncio.TimeoutError:
        logger.warning(
            "[Gateway] ICE gathering timed out (state=%s)",
            pc.iceGatheringState,
        )


class BrowserPeerManager:
    """Manages browser-facing RTCPeerConnection instances.

    Each browser gets one peer. Media tracks from Go2 are relayed to each
    browser peer via in-process ``RelayStreamTrack`` (no ffmpeg/UDP).

    Lifecycle:
    - ``handle_offer`` creates a new peer (closing any existing one for the user).
    - ``handle_candidate`` applies trickle-ICE candidates.
    - ``close_peer`` / ``close_all`` tear down peers and associated teleop bridges.
    - A background task prunes peers stuck in "connecting" longer than timeout.
    """

    def __init__(
        self,
        *,
        go2_conn: Any,
        media_cache: _Go2MediaCache,
        teleop_factory: Callable[[str], TeleopBridge],
        settings: Any,
        on_signal_out: OnSignalOut,
    ) -> None:
        self._go2_conn = go2_conn
        self._media_cache = media_cache
        self._teleop_factory = teleop_factory
        self._settings = settings
        self._on_signal_out = on_signal_out
        self._peers: dict[str, RTCPeerConnection] = {}
        self._teleop: dict[str, TeleopBridge] = {}
        self._close_locks: dict[str, asyncio.Lock] = {}
        self._created_at: dict[str, float] = {}
        self._peer_tokens: dict[str, int] = {}
        self._pruner_task: asyncio.Task[None] | None = None

    async def handle_offer(self, user_id: str, offer_data: dict[str, Any]) -> None:
        """Process an offer from *user_id*, creating or replacing the peer."""
        existing = self._peers.get(user_id)
        if existing is not None:
            state = getattr(existing, "connectionState", "")
            age = time.time() - self._created_at.get(user_id, 0.0)
            if state in ("connecting", "connected") and age < 10.0:
                logger.info(
                    "[Gateway] ignoring duplicate offer for %s (state=%s, %.1fs)",
                    user_id,
                    state,
                    age,
                )
                return

        token = self._peer_tokens.get(user_id, 0) + 1
        self._peer_tokens[user_id] = token
        await self._tear_down_peer(user_id)

        if self._peer_tokens.get(user_id) != token:
            return

        pc = RTCPeerConnection(_rtc_configuration(self._settings))
        self._peers[user_id] = pc
        self._close_locks.setdefault(user_id, asyncio.Lock())
        self._created_at[user_id] = time.time()
        bridge = self._teleop_factory(user_id)
        self._teleop[user_id] = bridge

        def _is_current() -> bool:
            return (
                self._peer_tokens.get(user_id) == token
                and self._peers.get(user_id) is pc
            )

        # Start the stale-peer pruner if not already running.
        if self._pruner_task is None or self._pruner_task.done():
            self._pruner_task = asyncio.create_task(self._prune_stale_peers())

        @pc.on("icecandidate")
        async def on_ice(candidate: Any) -> None:
            if candidate is not None and _is_current():
                await self._on_signal_out("candidate", user_id, candidate.to_dict())

        @pc.on("connectionstatechange")
        async def on_state() -> None:
            if not _is_current():
                return
            state = pc.connectionState
            logger.info("[Gateway] browser peer %s state=%s", user_id, state)
            if state == "failed":
                await self._tear_down_peer(user_id, pc=pc, token=token)

        @pc.on("iceconnectionstatechange")
        async def on_ice_state() -> None:
            if not _is_current():
                return
            logger.info(
                "[Gateway] browser peer %s ice=%s gathering=%s",
                user_id,
                pc.iceConnectionState,
                pc.iceGatheringState,
            )
            if pc.iceConnectionState == "failed":
                await self._tear_down_peer(user_id, pc=pc, token=token)

        @pc.on("datachannel")
        def on_datachannel(channel: Any) -> None:
            if channel.label != "control":
                return

            @channel.on("message")
            def on_message(message: Any) -> None:
                try:
                    raw = message if isinstance(message, str) else message.decode("utf-8")
                    msg = json.loads(raw)
                except (UnicodeDecodeError, json.JSONDecodeError):
                    return
                move = TeleopBridge.parse_move(msg)
                if move is not None:
                    bridge.set_joystick(*move)

        offer = RTCSessionDescription(sdp=offer_data["sdp"], type=offer_data["type"])
        try:
            await pc.setRemoteDescription(offer)
            if not _is_current():
                await _close_pc_safe(pc)
                return

            await attach_go2_to_browser(
                self._go2_conn, pc, media_cache=self._media_cache
            )
            if not _is_current():
                await _close_pc_safe(pc)
                return

            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
            await _wait_ice_gathering(pc)
            if not _is_current():
                await _close_pc_safe(pc)
                return

            await self._on_signal_out(
                "answer",
                user_id,
                {"type": pc.localDescription.type, "sdp": pc.localDescription.sdp},
            )
            logger.info("[Gateway] sent WebRTC answer to %s", user_id)
        except Exception as exc:
            logger.warning("[Gateway] offer handling failed for %s: %s", user_id, exc)
            if _is_current():
                await self._tear_down_peer(user_id, pc=pc, token=token)
            else:
                await _close_pc_safe(pc)

    async def handle_candidate(self, user_id: str, candidate: dict[str, Any]) -> None:
        """Apply a trickle-ICE candidate from *user_id*."""
        pc = self._peers.get(user_id)
        if pc is None:
            return

        ice = _parse_remote_candidate(candidate)
        if ice is None:
            return

        try:
            await pc.addIceCandidate(ice)
        except Exception as exc:
            logger.warning(
                "[Gateway] ICE candidate for %s ignored (%s): %s",
                user_id,
                getattr(ice, "type", "?"),
                exc,
            )

    async def close_peer(self, user_id: str) -> None:
        """Close the browser peer and its teleop bridge for *user_id*."""
        await self._tear_down_peer(user_id)

    async def _tear_down_peer(
        self,
        user_id: str,
        *,
        pc: RTCPeerConnection | None = None,
        token: int | None = None,
    ) -> None:
        lock = self._close_locks.setdefault(user_id, asyncio.Lock())

        async with lock:
            if token is not None and self._peer_tokens.get(user_id) != token:
                return
            if pc is not None and self._peers.get(user_id) is not pc:
                return

            teleop = self._teleop.pop(user_id, None)
            if teleop is not None:
                try:
                    await asyncio.wait_for(teleop.stop(), timeout=3.0)
                except asyncio.TimeoutError:
                    logger.warning("[Gateway] teleop stop timed out for %s", user_id)
                except Exception as exc:
                    logger.debug("[Gateway] teleop stop error for %s: %s", user_id, exc)

            current = self._peers.pop(user_id, None)
            self._created_at.pop(user_id, None)

            if current is not None:
                try:
                    await asyncio.wait_for(_close_pc_safe(current), timeout=_PEER_CLOSE_DRAIN)
                except asyncio.TimeoutError:
                    logger.warning("[Gateway] peer close timed out for %s", user_id)
                except Exception as exc:
                    logger.debug("[Gateway] peer close error for %s: %s", user_id, exc)

        if user_id not in self._peers:
            logger.info("[Gateway] browser peer %s fully closed", user_id)

    async def close_all(self) -> None:
        """Close all browser peers concurrently with a global timeout."""
        if self._pruner_task and not self._pruner_task.done():
            self._pruner_task.cancel()
            try:
                await self._pruner_task
            except asyncio.CancelledError:
                pass

        user_ids = list(self._peers)
        if not user_ids:
            return

        tasks = [asyncio.create_task(self.close_peer(uid)) for uid in user_ids]
        done, pending = await asyncio.wait(tasks, timeout=10.0)
        for t in pending:
            t.cancel()
        logger.info("[Gateway] all browser peers closed (%d closed, %d timed out)",
                    len(done), len(pending))

    async def _prune_stale_peers(self) -> None:
        """Periodically close peers stuck in 'connecting' state too long."""
        while True:
            await asyncio.sleep(10.0)
            now = time.time()
            stale = [
                uid
                for uid, created in list(self._created_at.items())
                if (now - created) > _PEER_CONNECT_TIMEOUT
                and self._peers.get(uid) is not None
                and getattr(self._peers[uid], "connectionState", "") == "connecting"
            ]
            for uid in stale:
                pc = self._peers.get(uid)
                if pc is None:
                    continue
                logger.warning(
                    "[Gateway] pruning stale connecting peer %s (%.0fs)",
                    uid,
                    now - self._created_at.get(uid, 0),
                )
                await self._tear_down_peer(uid, pc=pc, token=self._peer_tokens.get(uid))


async def _close_pc_safe(pc: RTCPeerConnection) -> None:
    """Close an RTCPeerConnection, catching and logging any errors."""
    try:
        await pc.close()
    except Exception as exc:
        logger.debug("[Gateway] pc.close() error: %s", exc)
