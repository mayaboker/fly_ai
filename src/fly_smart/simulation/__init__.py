"""PyBullet-based simulation adapters for the Fly Smart mission core."""

__all__ = ("StrikeResult", "StrikeSimulation")


def __getattr__(name: str):
    if name in __all__:
        from .runner import StrikeResult, StrikeSimulation

        return {"StrikeResult": StrikeResult, "StrikeSimulation": StrikeSimulation}[name]
    raise AttributeError(name)
