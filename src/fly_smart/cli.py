"""Compatibility entry point for the simulation command."""

from .simulation.cli import main, self_check

__all__ = ("main", "self_check")
