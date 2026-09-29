"""
Public LQR API owned by this package.

The validated TVLQR implementation remains import-compatible with the legacy
``gim_control.lqi_node`` while the runtime is migrated to the common runner.
"""

from gim_control.tvlqr_controller import TvlqrController, TvlqrWeights


class LqrController(TvlqrController):
    pass


__all__ = ["LqrController", "TvlqrWeights"]
