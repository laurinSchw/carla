#!/usr/bin/env python3
"""
CARLA Live Server — persistent multi-client simulation host.

Usage (demo / frontend-only mode):
    python start_server.py

Usage (live server mode — requires a running CARLA instance):
    python start_server.py \\
        --carla-host <carla-pc-ip> \\
        --admin-token mysecret \\
        --live

Options
-------
--live              Start the persistent CARLA world immediately.
--carla-host HOST   IP of the CARLA server  (default: env CARLA_HOST; required with --live)
--carla-port PORT   CARLA server port        (default: env CARLA_PORT, else 2000)
--admin-token TOKEN Admin API token          (default: read from ADMIN_TOKEN env)
--port PORT         Web server port          (default: 8000)
--no-v2x            Disable the V2X network
--fake              No CARLA needed: run the real server on a deterministic synthetic world (implies --live).
                    For frontend work and smoke tests. Not a simulator: see src/testing/fake_carla.
--fake-vehicles N   Background vehicles spawned at start in --fake mode (default: 12)
--fake-pedestrians N  Pedestrians spawned at start in --fake mode (default: 6)
--fake-seed N       Seed of the synthetic world's own randomness (default: 0; the simulation seed is --seed)
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
    p.add_argument("--carla-host", default=None,
                   help="CARLA server IP address (default: env CARLA_HOST; required with --live)")
    p.add_argument("--carla-port", type=int, default=None,
                   help="CARLA server RPC port (default: env CARLA_PORT, else 2000)")
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
    p.add_argument("--seed", type=int, default=42,
                   help="Master random seed (spawns, Traffic Manager, pedestrians, drivers, channel)")
    p.add_argument("--no-drivers", action="store_true", help="Disable seeded driver profiles (plain Traffic Manager)")
    p.add_argument("--v2x-advisory", action="store_true", help="V2X-assisted slow-down for threatened vehicles")
    p.add_argument("--v2x-rate", type=float, default=2.0,
                   help="V2X update rate in Hz; 1/(rate*0.05 s) must be a whole number of frames")
    p.add_argument("--v2x-penetration", type=float, default=1.0,
                   help="Share (0..1) of background vehicles that are V2X stations")
    p.add_argument("--tm-port", type=int, default=8001,
                   help="CARLA Traffic Manager port")
    p.add_argument("--fake", action="store_true",
                   help="Run on a deterministic synthetic world instead of CARLA (implies --live)")
    p.add_argument("--fake-vehicles", type=int, default=12, help="Background vehicles at start (--fake)")
    p.add_argument("--fake-pedestrians", type=int, default=6, help="Pedestrians at start (--fake)")
    p.add_argument("--fake-seed", type=int, default=0, help="Seed of the synthetic world (--fake)")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def resolve_carla_endpoint(host_arg, port_arg, env):
    """CARLA host and port: command line first, then CARLA_HOST / CARLA_PORT, port finally CARLA's own default 2000.
    There is no default host (it differs per lab); ``host`` is None when neither is given. Raises ValueError for a bad port."""
    host = host_arg or env.get("CARLA_HOST") or None
    raw = port_arg if port_arg is not None else (env.get("CARLA_PORT") or 2000)
    try:
        port = int(raw)
    except (TypeError, ValueError):
        raise ValueError(f"CARLA port must be a number, got {raw!r}")
    if not 1 <= port <= 65535:
        raise ValueError(f"CARLA port out of range: {port}")
    return host, port


def main() -> None:
    args = _parse_args()
    try:
        args.carla_host, args.carla_port = resolve_carla_endpoint(args.carla_host, args.carla_port, os.environ)
    except ValueError as exc:
        raise SystemExit(f"error: {exc}")
    if args.fake:
        # must happen before anything imports ``carla``; production paths never import src.testing
        from src.testing.fake_carla import install
        install(args.fake_seed)
        args.live = True
        args.carla_host = "fake"
    if args.live and not args.carla_host:
        raise SystemExit("error: --live needs a CARLA host: pass --carla-host HOST or set CARLA_HOST (port: --carla-port / CARLA_PORT, default 2000)")
    from src.server.app import run_server, set_persistent_world

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
    print(f"  Client join  : http://localhost:{args.port}/vehicle (open),  /observer")
    print(f"  Admin token  : {token}")
    if args.fake:
        print("  CARLA        : FAKE - a synthetic world, not a simulator (see src/testing/fake_carla)")
    elif args.live:
        print(f"  CARLA host   : {args.carla_host}:{args.carla_port}")
    if args.live:
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
            from src.traffic.advisory import AdvisoryConfig
            world = PersistentCARLAServer(WorldConfig(
                host=args.carla_host,
                port=args.carla_port,
                v2x_enabled=not args.no_v2x,
                v2x_range_m=args.v2x_range,
                v2x_penetration_rate=args.v2x_penetration,
                v2x_update_rate_hz=args.v2x_rate,
                seed=args.seed,
                drivers_enabled=not args.no_drivers,
                advisory=AdvisoryConfig(enabled=args.v2x_advisory),
                tm_port=args.tm_port,
                web_port=args.port,
            ))
            world.start()
            set_persistent_world(world)
            if args.fake:
                world.submit(world.spawn_background_traffic, args.fake_vehicles).result(30)
                world.submit(world.spawn_pedestrians, args.fake_pedestrians).result(30)
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

