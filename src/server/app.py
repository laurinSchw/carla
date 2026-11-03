"""FastAPI app for the persistent CARLA server: admin and client routes plus static pages.

Replaces the plane-B wiring that lived in ``src/visualization/lidar/server.py``.
"""

import sys
from pathlib import Path

from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

import src.server.admin_routes as _admin_mod
import src.server.client_routes as _client_mod

WEB_DIR = Path(__file__).parent.parent / "visualization" / "web"

app = FastAPI(title="CARLA V2X + LiDAR Server")
app.include_router(_admin_mod.router)
app.include_router(_client_mod.router)

if (WEB_DIR / "static").exists():
    app.mount("/static", StaticFiles(directory=str(WEB_DIR / "static")), name="static")
if (WEB_DIR / "assets").exists():
    app.mount("/assets", StaticFiles(directory=str(WEB_DIR / "assets")), name="assets")


def set_persistent_world(world) -> None:
    """Register the PersistentCARLAServer with the admin and client routes."""
    _admin_mod.set_persistent_world(world)
    _client_mod.set_persistent_world(world)


# /admin/connect creates a world after startup and propagates it through this module.
_admin_mod.set_server_module(sys.modules[__name__])


def _page(name: str, missing: str) -> HTMLResponse:
    page = WEB_DIR / name
    if page.exists():
        return HTMLResponse(content=page.read_text())
    return HTMLResponse(content=f"<h1>{missing} not found</h1>", status_code=404)


@app.get("/", include_in_schema=False)
async def root():
    # Docker healthcheck requests "/" and expects 200.
    return HTMLResponse(
        "<!doctype html><title>CARLA V2X + LiDAR Server</title>"
        "<h1>CARLA V2X + LiDAR Server</h1>"
        "<ul><li><a href='/admin'>Admin panel</a> (token required)</li>"
        "<li><a href='/client'>Join as a vehicle</a></li></ul>"
    )


@app.get("/admin", include_in_schema=False)
async def admin_panel():
    return _page("admin_panel.html", "Admin panel")


@app.get("/client", include_in_schema=False)
async def client_join():
    return _page("client_join.html", "Client join page")


def run_server(host: str = "0.0.0.0", port: int = 8000):
    import uvicorn

    uvicorn.run(app, host=host, port=port)
