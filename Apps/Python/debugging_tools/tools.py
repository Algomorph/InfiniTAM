import numpy as np
from typing import Mapping, Tuple


class Segment:


def segment_sign(segment: Tuple[np.ndarray, np.ndarray]):
    direction = segment[1] - segment[0]
    return (1.0 / direction) < 0.0


def SegmentIntersectsGridAlignedBox3D(segment: Tuple[np.ndarray, np.ndarray],
                                      boxMin: Mapping[int, float], boxMax: Mapping[int, float]):
    tmin = tmax = tymin = tymax = tzmin = tzmax = 0.
    bounds = (boxMin, boxMax)
    sign = segment_sign(segment)

    tmin = (bounds[sign[0]].x - segment.origin.x) * segment.inverseDirection.x;
    tmax = (bounds[1 - sign[0]].x - segment.origin.x) * segment.inverseDirection.x;
    tymin = (bounds[sign[1]].y - segment.origin.y) * segment.inverseDirection.y;
    tymax = (bounds[1 - sign[1]].y - segment.origin.y) * segment.inverseDirection.y;

    if ((tmin > tymax) or (tymin > tmax)):
        return False

    if (tymin > tmin):
        tmin = tymin
    if (tymax < tmax):
        tmax = tymax

    tzmin = (bounds[segment.sign[2]].z - segment.origin.z) * segment.inverseDirection.z;
    tzmax = (bounds[1 - segment.sign[2]].z - segment.origin.z) * segment.inverseDirection.z;

    if ((tmin > tzmax) | | (tzmin > tmax)):
        return False

    if (tzmin > tmin):
        tmin = tzmin

    if (tzmax < tmax):
        tmax = tzmax
    return not (tmax < 0.0 or tmin > 1.0)
