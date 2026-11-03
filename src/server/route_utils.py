"""Helpers shared by the route modules."""

import asyncio
from typing import Any, Callable

from fastapi import HTTPException


async def on_sim_thread(world, fn: Callable[..., Any], *args: Any) -> Any:
    """Run ``fn(*args)`` on the world's simulation thread and await the result.

    The event loop stays free while CARLA works (defect C5). If the simulation is not
    running the request fails with 503 instead of touching CARLA from the wrong thread.
    """
    try:
        return await asyncio.wrap_future(world.submit(fn, *args))
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc))
