
def _raycast_polyline(self, angle_rad: float):
    """Sugár–polyline metszés.
    Visszaad: (r, nx, ny) ahol r a távolság, (nx,ny) az egységnyi fal-normál,
    amely a robot felé mutat. Ha nincs metszés: None.
    """
    poly = self.poly_left if self.side == 'left' else self.poly_right
    if len(poly) < self.min_poly_points:
        return None

    dx = math.cos(angle_rad)
    dy = math.sin(angle_rad)

    best = None  # (t, nx, ny)
    for i in range(len(poly) - 1):
        x1, y1 = poly[i]
        x2, y2 = poly[i + 1]
        sx = x2 - x1
        sy = y2 - y1

        # sugár [0,0]+t[dx,dy], szakasz [x1,y1]+u[sx,sy]
        det = (-dx * sy + dy * sx)
        if abs(det) < 1e-9:
            continue  # párhuzamos

        u = (-dy * x1 + dx * y1) / det

        if abs(dx) > abs(dy):
            t = (x1 + u * sx) / (dx if abs(dx) > 1e-9 else 1e-9)
        else:
            t = (y1 + u * sy) / (dy if abs(dy) > 1e-9 else 1e-9)

        if not (0.0 <= u <= 1.0 and t is not None and t >= self.valid_min):
            continue

        # metszéspont (falon)
        px = dx * t
        py = dy * t

        # szakasz tangense
        tx = sx
        ty = sy
        tn = math.hypot(tx, ty)
        if tn < 1e-6:
            continue
        tx /= tn
        ty /= tn

        # két lehetséges normál: (-ty, tx) és (ty, -tx)
        nx = -ty
        ny = tx

        # irány kiválasztása: a robot (0,0) felé mutasson
        # azaz a normál és (-p) skalárszorzata legyen pozitív
        if nx * (-px) + ny * (-py) < 0.0:
            nx = -nx
            ny = -ny

        if best is None or t < best[0]:
            best = (t, nx, ny)

    if best is None:
        return None

    t, nx, ny = best
    r = min(t, self.range_clip_max)
    return r, nx, ny


    def _get_range_follow(self, scan: LaserScan, angle_rad: float):
    """Távolság + normál a follow-szögben.
    Először a polyline-t próbáljuk, ha nincs találat, LiDAR-ból becsülünk.
    Visszaad: (r, nx, ny)
    """
    # Polyline elsődleges
    if self._poly_active():
        res = self._raycast_polyline(angle_rad)
        if res is not None:
            return res  # (r, nx, ny)

    # Fallback: LiDAR – normál becslés -p/|p|
    r = self._get_range_at_scan(scan, angle_rad, half_window=1)
    if not math.isfinite(r) or r >= self.range_clip_max:
        return self.range_clip_max, 0.0, 0.0

    px = r * math.cos(angle_rad)
    py = r * math.sin(angle_rad)
    norm = math.hypot(px, py)
    if norm < 1e-6:
        return r, 0.0, 0.0

    nx = -px / norm
    ny = -py / norm
    return r, nx, ny


    def _goalpoint_from_follow(self, scan: LaserScan):
    a = math.radians(self.sign * self.follow_angle_deg)
    r, nx, ny = self._get_range_follow(scan, a)

    if not math.isfinite(r) or r >= self.range_clip_max or (nx == 0.0 and ny == 0.0):
        return None

    # falon lévő metszéspont a sugár mentén
    px = r * math.cos(a)
    py = r * math.sin(a)

    # a fal normálja felé toljuk (nx,ny), ami a robot felé mutat
    gx = px + self.desired_current * nx + self.lookahead_forward
    gy = py + self.desired_current * ny
    return gx, gy