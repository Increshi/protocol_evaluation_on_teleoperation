#!/usr/bin/env python3
"""
WebRTC Signaling Server
========================
Lightweight WebSocket server that brokers the SDP offer/answer and ICE candidate
exchange between the Isaac Sim server (Python) and the Unity client (C#).

Run this on a machine reachable by BOTH Isaac Sim and the Quest headset.
It can be the same laptop that previously ran forwarding.py.

Dependencies:
    pip install websockets

Usage:
    python webrtc_signaling_server.py
"""

import asyncio
import json
import logging
import argparse
from typing import Optional

try:
    import websockets
    from websockets.server import serve
except ImportError:
    print("❌  Missing dependency. Run:  pip install websockets")
    raise SystemExit(1)

# ==============================================================================
# Configuration
# ==============================================================================
DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8765

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("signaling")

# ==============================================================================
# Signaling Server
# ==============================================================================
class SignalingServer:
    """
    Keeps track of exactly two peers:
      • "isaac"  – the Isaac Sim side (Python WebRTC peer)
      • "unity"  – the Unity / Quest side (C# WebRTC peer)

    Any message from one peer is forwarded to the other.
    """

    def __init__(self):
        self.peers: dict[str, websockets.WebSocketServerProtocol] = {}
        self.lock = asyncio.Lock()

    async def register(self, ws: websockets.WebSocketServerProtocol, role: str):
        async with self.lock:
            if role in self.peers:
                old = self.peers[role]
                log.warning(f"⚠️  Role '{role}' reconnected – closing old socket.")
                try:
                    await old.close()
                except Exception:
                    pass
            self.peers[role] = ws
            log.info(f"✅  '{role}' registered  ({ws.remote_address})")

    async def unregister(self, role: str):
        async with self.lock:
            self.peers.pop(role, None)
            log.info(f"🔌  '{role}' disconnected")

    def other_role(self, role: str) -> str:
        return "unity" if role == "isaac" else "isaac"

    async def relay(self, sender_role: str, message: str):
        target_role = self.other_role(sender_role)
        async with self.lock:
            target_ws = self.peers.get(target_role)
        if target_ws is None:
            log.warning(f"⏳  No '{target_role}' connected yet – dropping message.")
            return
        try:
            await target_ws.send(message)
            log.debug(f"📨  {sender_role} → {target_role}  ({len(message)} bytes)")
        except websockets.ConnectionClosed:
            log.warning(f"📴  '{target_role}' connection lost while relaying.")

    async def handler(self, ws: websockets.WebSocketServerProtocol):
        """Handle a single WebSocket connection."""
        role: Optional[str] = None
        try:
            # First message MUST be a registration: {"register": "isaac"} or {"register": "unity"}
            raw = await asyncio.wait_for(ws.recv(), timeout=10.0)
            msg = json.loads(raw)
            role = msg.get("register")
            if role not in ("isaac", "unity"):
                await ws.close(1008, "First message must be {\"register\": \"isaac\" | \"unity\"}")
                return
            await self.register(ws, role)

            # Notify the peer that registration succeeded
            await ws.send(json.dumps({"registered": role}))

            # Relay loop
            async for raw in ws:
                # Check for "bye" — relay it, then treat as disconnect
                try:
                    parsed = json.loads(raw)
                    if parsed.get("bye"):
                        log.info(f"👋  '{role}' sent bye — relaying and disconnecting")
                        await self.relay(role, raw)
                        # Don't break here; let the finally block unregister.
                        # Just close our end cleanly.
                        await ws.close(1000, "bye")
                        return
                except (json.JSONDecodeError, AttributeError):
                    pass
                await self.relay(role, raw)

        except asyncio.TimeoutError:
            log.warning("Timeout waiting for registration message.")
        except websockets.ConnectionClosed:
            pass
        except Exception as e:
            log.error(f"Handler error: {e}")
        finally:
            if role:
                await self.unregister(role)


async def main(host: str, port: int):
    server = SignalingServer()
    log.info(f"🚀  Signaling server starting on ws://{host}:{port}")
    async with serve(server.handler, host, port):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WebRTC Signaling Server")
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"Bind address (default {DEFAULT_HOST})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"Port (default {DEFAULT_PORT})")
    args = parser.parse_args()
    asyncio.run(main(args.host, args.port))
