import numpy as np


def medfilt1d_same(x: np.ndarray, k: int) -> np.ndarray:
    if k <= 1:
        return x
    k = int(k) | 1
    pad = k // 2
    xp = np.pad(x, (pad, pad), mode='edge')
    shape = (x.size, k)
    strides = (xp.strides[0], xp.strides[0])
    as_strided = np.lib.stride_tricks.as_strided
    win = as_strided(xp, shape=shape, strides=strides)
    return np.partition(win, pad, axis=1)[:, pad]


def longest_true_run(mask: np.ndarray):
    if mask.size == 0:
        return 0, 0
    m = mask.astype(np.int8)
    dm = np.diff(np.r_[0, m, 0])
    starts = np.flatnonzero(dm == 1)
    ends = np.flatnonzero(dm == -1)
    if starts.size == 0:
        return 0, 0
    lengths = ends - starts
    i = np.argmax(lengths)
    return int(starts[i]), int(lengths[i])


def lpf(prev: float, new: float, alpha: float) -> float:
    return (1.0 - alpha) * prev + alpha * new


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class RateLimiter:
    def __init__(self, dv_max: float):
        self.dv_max = abs(float(dv_max))
        self.prev = None
    def step(self, target: float) -> float:
        if self.prev is None:
            self.prev = target
            return target
        d = target - self.prev
        if d > self.dv_max:
            d = self.dv_max
        elif d < -self.dv_max:
            d = -self.dv_max
        self.prev += d
        return self.prev