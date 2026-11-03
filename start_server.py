#!/usr/bin/env python3
"""
CARLA Live Server — persistent multi-client simulation host.

Usage (demo / frontend-only mode):
    python start_server.py

Usage (live server mode — requires a running CARLA instance):
    python start_server.py \\
        --carla-host 192.168.1.110 \\
        --admin-token mysecret \\
        --live

Options
-------
--live              Start the persistent CARLA world immediately.
--carla-host HOST   IP of the CARLA server  (default: 192.168.1.110)
--carla-port PORT   CARLA server port        (default: 2000)
--admin-token TOKEN Admin API token          (default: read from ADMIN_TOKEN env)
--port PORT         Web server port          (default: 8000)
--no-v2v            Disable V2V network
--v2v-range M       V2V cooperative range in metres (default: 75)
"""

import argparse
import logging
import os
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Ensure project root is on sys.path so 'src' imports work from anywhere
# ---------------------------------------------------------------------------
_project_root = Path(__file__).resolve().parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.server.app import run_server, set_persistent_world  # noqa: E402

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(name)s — %(message)s",
)
logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="CARLA Live Server",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--live", action="store_true",
                   help="Start persistent CARLA world on launch")
    p.add_argument("--carla-host", default="192.168.1.110",
                   help="CARLA server IP address")
    p.add_argument("--carla-port", type=int, default=2000,
                   help="CARLA server port")
    p.add_argument("--admin-token", default=None,
                   help="Secret token for the /admin API (overrides ADMIN_TOKEN env var)")
    p.add_argument("--port", type=int, default=8000,
                   help="Web server listen port")
    p.add_argument("--host", default="0.0.0.0",
                   help="Web server bind address")
    p.add_argument("--no-v2x", action="store_true",
                   help="Disable V2X network")
    p.add_argument("--v2x-range", type=float, default=75.0,
                   help="V2X cooperative range in metres")
    p.add_argument("--tm-port", type=int, default=8001,
                   help="CARLA Traffic Manager port")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    args = _parse_args()

    # --- Admin token --------------------------------------------------------
    token = args.admin_token or os.environ.get("ADMIN_TOKEN", "")
    if token:
        os.environ["ADMIN_TOKEN"] = token
    else:
        # Generate a random token so the server isn't accidentally open
        import secrets
        token = secrets.token_urlsafe(16)
        os.environ["ADMIN_TOKEN"] = token
        logger.warning("No --admin-token provided; generated: %s", token)

    # --- Banner -------------------------------------------------------------
    sep = "=" * 72
    print(sep)
    print("  CARLA Live Server")
    print(sep)
    print(f"  Web server   : http://{args.host}:{args.port}")
    print(f"  Admin panel  : http://localhost:{args.port}/admin   (token required)")
    print(f"  Client join  : http://localhost:{args.port}/client  (open)")
    print(f"  Admin token  : {token}")
    if args.live:
        print(f"  CARLA host   : {args.carla_host}:{args.carla_port}")
        print(f"  V2X          : {'disabled' if args.no_v2x else f'enabled, range={args.v2x_range:.0f} m'}")
    else:
        print("  Mode         : frontend-only (use --live to start simulation)")
    print(sep)
    print("  Press Ctrl+C to stop")
    print(sep)

    # --- Persistent world ---------------------------------------------------
    world = None
    if args.live:
        try:
            from src.server.persistent_world import PersistentCARLAServer
            from src.server.world_config import WorldConfig
            world = PersistentCARLAServer(WorldConfig(
                host=args.carla_host,
                port=args.carla_port,
                v2x_enabled=not args.no_v2x,
                v2x_range_m=args.v2x_range,
                tm_port=args.tm_port,
                web_port=args.port,
            ))
            world.start()
            set_persistent_world(world)
            logger.info("Persistent CARLA world started")
        except Exception as exc:
            logger.error("Failed to start persistent world: %s", exc)
            logger.error("Falling back to frontend-only mode")

    # --- Web server ---------------------------------------------------------
    try:
        run_server(host=args.host, port=args.port)
    except KeyboardInterrupt:
        print("\nShutting down…")
    finally:
        if world is not None:
            world.stop()
        logger.info("Server stopped")


if __name__ == "__main__":
    main()

