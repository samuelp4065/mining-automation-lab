"""Proximity decision logic, with no ROS and no camera.

Kept separate from the node so the part that decides whether the machine may
move can be tested exhaustively at the desk. Every interesting case here --
hysteresis around the threshold, a patch full of dropouts, a stale signal --
is one you would otherwise have to reproduce by walking in front of a moving
excavator.
"""

from dataclasses import dataclass

import numpy as np

__all__ = ["ProximityConfig", "ProximityState", "patch_distance", "evaluate"]


@dataclass(frozen=True)
class ProximityConfig:
    """Thresholds for the interlock.

    ``release_above`` must exceed ``stop_below``: with a single threshold the
    machine chatters -- stop, creep forward, clear, stop -- when an obstacle
    sits exactly at the limit. The gap is the hysteresis band.

    ``min_valid_fraction`` guards against deciding on almost no data. A patch
    that is 95% dropout can produce a confident-looking median from a handful
    of noisy pixels, and treating that as a clear path is the dangerous
    direction to be wrong in.
    """

    stop_below_m: float = 1.00
    release_above_m: float = 1.20
    min_valid_fraction: float = 0.25

    def __post_init__(self):
        if self.release_above_m <= self.stop_below_m:
            raise ValueError(
                "release_above_m must exceed stop_below_m, or the interlock "
                "will chatter at the threshold"
            )


@dataclass(frozen=True)
class ProximityState:
    clear: bool           # may the machine drive forward?
    distance_m: float     # nearest measured distance, NaN if unknown
    valid_fraction: float # share of the patch that returned anything
    reason: str


def patch_distance(depth_m, centre, size):
    """Median distance over a square patch, ignoring dropouts.

    Median rather than minimum: a single hot pixel reading 0.2 m would stop
    the machine every frame it appears. Median rather than mean: a mean is
    dragged by the far background showing through a gap in a near object.

    Args:
        depth_m: 2-D array of metres, NaN where there was no return.
        centre: (u, v) pixel to sample around.
        size: patch edge length in pixels.

    Returns:
        ``(distance, valid_fraction)``; distance is NaN if nothing returned.
    """
    height, width = depth_m.shape
    half = max(size // 2, 1)
    u, v = int(np.clip(centre[0], 0, width - 1)), int(np.clip(centre[1], 0, height - 1))

    patch = depth_m[max(v - half, 0):v + half + 1, max(u - half, 0):u + half + 1]
    if patch.size == 0:
        return float("nan"), 0.0

    valid = patch[np.isfinite(patch)]
    fraction = valid.size / patch.size
    if valid.size == 0:
        return float("nan"), 0.0
    return float(np.median(valid)), float(fraction)


def evaluate(distance_m, valid_fraction, config, previously_clear):
    """Decide whether forward motion is permitted.

    ``previously_clear`` carries the hysteresis: which threshold applies
    depends on which side the machine is currently on.

    An unusable reading -- no returns, or too few to trust -- is reported as
    NOT clear. The alternative is to let the machine drive on the strength of
    data that is missing, and the most likely reason for a sudden loss of
    returns is something close and dark filling the frame.
    """
    if not np.isfinite(distance_m) or valid_fraction < config.min_valid_fraction:
        return ProximityState(
            clear=False,
            distance_m=distance_m,
            valid_fraction=valid_fraction,
            reason=f"insufficient data ({valid_fraction:.0%} of patch valid)",
        )

    threshold = config.release_above_m if not previously_clear else config.stop_below_m
    clear = distance_m >= threshold

    if clear:
        reason = f"clear at {distance_m:.2f} m"
    else:
        reason = f"obstacle at {distance_m:.2f} m (limit {threshold:.2f} m)"

    return ProximityState(
        clear=clear,
        distance_m=distance_m,
        valid_fraction=valid_fraction,
        reason=reason,
    )
