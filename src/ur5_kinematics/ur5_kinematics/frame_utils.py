"""Static UR frame helpers (``base`` vs REP-103 ``base_link``)."""

from __future__ import annotations

import numpy as np


def transform_point(point: np.ndarray, from_frame: str, to_frame: str) -> np.ndarray:
    """Transform a 3D point between supported UR visualization frames."""
    point = np.asarray(point, dtype=float).reshape(3)
    if from_frame == to_frame:
        return point.copy()

    if from_frame == 'base' and to_frame == 'base_link':
        return np.array([-point[0], -point[1], point[2]], dtype=float)

    if from_frame == 'base_link' and to_frame == 'base':
        return np.array([-point[0], -point[1], point[2]], dtype=float)

    raise ValueError(
        f'unsupported frame transform {from_frame!r} -> {to_frame!r} '
        '(supported: base <-> base_link)'
    )


__all__ = ['transform_point']