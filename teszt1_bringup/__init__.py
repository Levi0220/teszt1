
        # outlier limit a goalpontra
        self.max_goal_jump = float(self.declare_parameter('max_goal_jump', 2.0).value)
        # goalpoint ne mehessen a fal túloldalára (offset <= ratio * r)
        self.max_goal_offset_ratio = float(self.declare_parameter('max_goal_offset_ratio', 0.8).value)
    # ------------ Follow-hit: távolság + normál ------------
    def _get_follow_hit(self, scan: LaserScan, angle_rad: float):
        """
        Polyline esetén: (r, nx, ny) a legközelebbi fal-szakasz normáljával.
        Ha nincs polyline, LiDAR-ból számolunk egy "radialis" normált.
        """
        if self._poly_active():
            res = self._raycast_polyline(angle_rad)
            if res is not None:
                return res  # (r, nx, ny)

        # --- fallback: nyers LiDAR ---
        r = self._get_range_at_scan(scan, angle_rad, half_window=1)
        if not math.isfinite(r) or r >= self.range_clip_max:
            return None

        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)
        px = r * dx
        py = r * dy
        norm = math.hypot(px, py)
        if norm < 1e-6:
            return None

        # régi logika: robot <- falpont irány (nem igazi falnormál, de jobb mint a semmi)
        nx = -px / norm
        ny = -py / norm
        return r, nx, ny


    def _goalpoint_from_follow(self, scan: LaserScan):
        a = math.radians(self.sign * self.follow_angle_deg)

        hit = self._get_follow_hit(scan, a)
        if hit is None:
            return None

        r, nx, ny = hit
        if not math.isfinite(r) or r >= self.range_clip_max:
            return None

        # falpont a follow-sugár mentén
        px = r * math.cos(a)
        py = r * math.sin(a)

        # normált biztos ami biztos egységre normáljuk
        n_len = math.hypot(nx, ny)
        if n_len < 1e-6:
            return None
        nx /= n_len
        ny /= n_len

        # ne mehessünk a fal túloldalára: offset <= max_goal_offset_ratio * r
        max_offset = self.max_goal_offset_ratio * r
        offset = min(self.desired_current, max_offset)

        # goalpont = falpont + offset * n + előretolás
        gx = px + offset * nx + self.lookahead_forward
        gy = py + offset * ny
        return gx, gy
