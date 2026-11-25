"""solid.py -- containment and penetration depth for closed triangle meshes (numpy + scipy only).

`Solid(V, F)` answers, for many query points at once:
  winding(X)  generalised winding number from signed vertical ray crossings (1 inside, 0 outside a closed,
              outward-oriented mesh; robust to small gaps because crossings are summed with their orientation)
  inside(X)   winding >= 0.5
  depth(X)    penetration depth: distance from an inside point to the nearest point of the surface (dense surface
              samples, no distance cut-off), 0 outside
  band_depth  the old estimate (nearest sample within 2.5 mm, projected on its normal): kept only to show what the
              previous fit check missed

How containment works: the mesh is cut by vertical lines on an `h` grid (column centres jittered by a fraction of a
cell so they never run exactly along an edge). For every column the heights where it crosses a triangle are stored
with the triangle's orientation (+1 if its normal points up). A point's winding number is the sum of the orientations
of the crossings above it. The point is snapped to its column, so the containment boundary is exact in z and within
h / sqrt(2) in x/y. Depth comes from the surface samples, so it is exact to about half the sample spacing.

Used by check_v4_fit.py; tests in tests/test_fit_solid.py.
"""
import numpy as np
from scipy.spatial import cKDTree

_JITTER = (0.1371, 0.2927)          # column-centre offsets in cells: irrational-ish, avoids edge/vertex hits


def sample_surface(V, F, n, seed=0):
    """`n` area-weighted random points on the triangles (and the triangle normals)."""
    rng = np.random.default_rng(seed)
    T = V[F]
    cr = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    area = 0.5 * np.linalg.norm(cr, axis=1)
    keep = area > 0
    T, cr, area = T[keep], cr[keep], area[keep]
    i = rng.choice(len(T), size=n, p=area / area.sum())
    u, v = rng.random(n), rng.random(n)
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    P = T[i, 0] + u[:, None] * (T[i, 1] - T[i, 0]) + v[:, None] * (T[i, 2] - T[i, 0])
    N = cr[i] / np.linalg.norm(cr[i], axis=1, keepdims=True)
    return P, N


class Solid:
    def __init__(self, V, F, h=None, n_samples=None, seed=0):
        V = np.asarray(V, float); F = np.asarray(F, np.int64)
        self.V, self.F = V, F
        self.lo, self.hi = V.min(0), V.max(0)
        ext = self.hi - self.lo
        self.h = float(h if h is not None else max(2.5e-4, min(1e-3, ext[:2].max() / 400)))
        T = V[F]
        cr = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        self.area = float(0.5 * np.linalg.norm(cr, axis=1).sum())
        self.volume = float(np.einsum("ij,ij->i", T[:, 0], np.cross(T[:, 1], T[:, 2])).sum() / 6.0)
        self._orient = 1.0 if self.volume >= 0 else -1.0          # inside-out meshes still give winding +1 inside
        if n_samples is None:                                     # ~0.3 mm spacing, capped
            n_samples = int(np.clip(self.area / (3e-4 ** 2), 2000, 150000))
        self.P, self.N = sample_surface(V, F, n_samples, seed)
        self.tree = cKDTree(self.P)
        self.spacing = float(np.sqrt(self.area / n_samples))
        self._build_columns(T, cr)
        self.centre = 0.5 * (self.lo + self.hi)
        self.radius = float(np.linalg.norm(V - self.centre, axis=1).max())

    # ---- column index ------------------------------------------------------------------------
    def _build_columns(self, T, cr):
        h = self.h
        self.x0 = self.lo[0] - h * (1 + _JITTER[0]); self.y0 = self.lo[1] - h * (1 + _JITTER[1])
        self.nx = int(np.ceil((self.hi[0] - self.x0) / h)) + 2
        self.ny = int(np.ceil((self.hi[1] - self.y0) / h)) + 2
        nz = cr[:, 2]
        keep = np.abs(nz) > 1e-18                                  # vertical triangles have no crossings
        T, nz = T[keep], nz[keep]
        # column i has its centre at x0 + (i + 0.5) h
        i0 = np.ceil((T[:, :, 0].min(1) - self.x0) / h - 0.5).astype(np.int64)
        i1 = np.floor((T[:, :, 0].max(1) - self.x0) / h - 0.5).astype(np.int64)
        j0 = np.ceil((T[:, :, 1].min(1) - self.y0) / h - 0.5).astype(np.int64)
        j1 = np.floor((T[:, :, 1].max(1) - self.y0) / h - 0.5).astype(np.int64)
        ni, nj = np.maximum(i1 - i0 + 1, 0), np.maximum(j1 - j0 + 1, 0)
        cnt = ni * nj
        keys, zs, sg = [], [], []
        tri_idx = np.arange(len(T))
        for chunk in np.array_split(tri_idx, max(1, int(cnt.sum() // 4_000_000) + 1)):
            c = cnt[chunk]
            t = np.repeat(chunk, c)
            if len(t) == 0:
                continue
            start = np.repeat(np.cumsum(c) - c, c)
            k = np.arange(len(t)) - start
            ii = i0[t] + k // nj[t]; jj = j0[t] + k % nj[t]
            px = self.x0 + (ii + 0.5) * h; py = self.y0 + (jj + 0.5) * h
            a, b, cc = T[t, 0], T[t, 1], T[t, 2]
            d = (b[:, 0] - a[:, 0]) * (cc[:, 1] - a[:, 1]) - (cc[:, 0] - a[:, 0]) * (b[:, 1] - a[:, 1])
            w1 = ((px - a[:, 0]) * (cc[:, 1] - a[:, 1]) - (cc[:, 0] - a[:, 0]) * (py - a[:, 1])) / d
            w2 = ((b[:, 0] - a[:, 0]) * (py - a[:, 1]) - (px - a[:, 0]) * (b[:, 1] - a[:, 1])) / d
            w0 = 1 - w1 - w2
            hit = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
            z = w0 * a[:, 2] + w1 * b[:, 2] + w2 * cc[:, 2]
            keys.append((ii * self.ny + jj)[hit]); zs.append(z[hit]); sg.append(np.sign(nz[t])[hit])
        col = np.concatenate(keys) if keys else np.zeros(0, np.int64)
        z = np.concatenate(zs) if zs else np.zeros(0)
        s = np.concatenate(sg) if sg else np.zeros(0)
        self._zspan = float(self.hi[2] - self.lo[2]) + 1.0
        key = col * self._zspan + (z - self.lo[2])
        o = np.argsort(key, kind="stable")
        self._key = key[o]
        self._csum = np.concatenate([[0.0], np.cumsum(s[o])])
        self.n_crossings = len(o)

    def winding(self, X):
        X = np.atleast_2d(np.asarray(X, float))
        out = np.zeros(len(X))
        i = np.floor((X[:, 0] - self.x0) / self.h).astype(np.int64)
        j = np.floor((X[:, 1] - self.y0) / self.h).astype(np.int64)
        ok = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny) & (X[:, 2] >= self.lo[2]) & (X[:, 2] <= self.hi[2])
        if not ok.any():
            return out
        col = i[ok] * self.ny + j[ok]
        q = np.searchsorted(self._key, col * self._zspan + (X[ok, 2] - self.lo[2]), side="right")
        e = np.searchsorted(self._key, (col + 1) * self._zspan, side="left")
        out[ok] = (self._csum[e] - self._csum[q]) * self._orient
        return out

    def inside(self, X):
        return self.winding(X) >= 0.5

    def depth(self, X):
        """Penetration depth (m) of each point: distance to the surface if inside, else 0."""
        X = np.atleast_2d(np.asarray(X, float))
        out = np.zeros(len(X))
        near = np.all((X >= self.lo) & (X <= self.hi), axis=1)
        if not near.any():
            return out
        idx = np.flatnonzero(near)
        ins = self.inside(X[idx])
        if ins.any():
            k = idx[ins]
            out[k] = self.tree.query(X[k])[0]
        return out

    def band_depth(self, X, band=0.0025):
        """The previous estimate: nearest surface sample within `band`, depth along its normal (misses deep points)."""
        X = np.atleast_2d(np.asarray(X, float))
        dist, i = self.tree.query(X, distance_upper_bound=band)
        ok = np.isfinite(dist); out = np.zeros(len(X))
        out[ok] = np.maximum(0.0, -np.einsum("ij,ij->i", X[ok] - self.P[i[ok]], self.N[i[ok]]))
        return out


def box_mesh(lo, hi):
    """Closed, outward-oriented box (V, F) between corners lo and hi."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    V = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    F = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                  [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    return V, F


def mutual_depth(A, PA, B, PB):
    """Deepest penetration between two solids, both directions: samples of A inside B and samples of B inside A.
    PA / PB are A's / B's surface samples already expressed in the other solid's frame."""
    return max(float(B.depth(PA).max(initial=0.0)), float(A.depth(PB).max(initial=0.0)))
