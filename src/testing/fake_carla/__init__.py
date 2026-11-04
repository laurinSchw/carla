"""``install()`` makes ``import carla`` resolve to the deterministic fake (``start_server.py --fake``, headless
scenarios, frontend work). Never imported by a production path: only ``--fake`` / ``fake: true`` reach it."""

import logging
import sys
import types

from .sim import build_module

logger = logging.getLogger(__name__)


def install(seed: int = 0) -> types.ModuleType:
    """Register the fake as the ``carla`` module and return it. Refuses to replace a real, already imported CARLA.

    Idempotent: a second call returns the module already installed (modules that did ``import carla`` keep their
    reference, so swapping it would split the process between two fakes). ``seed`` only seeds the walkers' navigation
    targets until the server calls ``world.set_pedestrians_seed(master seed)``, which it does at start.
    """
    existing = sys.modules.get("carla")
    if existing is not None:
        if not getattr(existing, "IS_FAKE", False):
            raise RuntimeError("the real carla module is already imported; --fake must be chosen before it is")
        return existing
    module = build_module(seed)
    sys.modules["carla"] = module
    logger.warning("carla is the FAKE synthetic world (src/testing/fake_carla): results are not CARLA results")
    return module
