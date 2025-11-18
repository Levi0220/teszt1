    # ------------ Raycast polyline mentén ------------
    def _raycast_polyline(self, angle_rad: float):
        """
        Sugár: r(t) = t * d, d = (cos(a), sin(a)), t >= 0
        Szakasz: p(u) = p1 + u * s, s = p2 - p1, 0 <= u <= 1

        Visszatérés:
          (dist, nx, ny)   ha találtunk legközelebbi metszést
          None             ha nincs metszés
        ahol (nx, ny) a fal helyi normálvektora, a robot felé mutatva.
        """
        poly = self.poly_left if self.side == 'left' else self.poly_right
        if len(poly) < self.min_poly_points:
            return None

        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)

        best_t = None
        best_nx = None
        best_ny = None

        for i in range(len(poly) - 1):
            x1, y1 = poly[i]
            x2, y2 = poly[i + 1]
            sx = x2 - x1
            sy = y2 - y1

            # ray [0,0]+t[dx,dy], seg [x1,y1]+u[sx,sy]
            det = (-dx * sy + dy * sx)
            if abs(det) < 1e-9:
                continue  # párhuzamos, kihagyjuk

            u = (-dy * x1 + dx * y1) / det

            if abs(dx) > abs(dy):
                t = (x1 + u * sx) / (dx if abs(dx) > 1e-9 else 1e-9)
            else:
                t = (y1 + u * sy) / (dy if abs(dy) > 1e-9 else 1e-9)

            if not (0.0 <= u <= 1.0 and t is not None and t >= self.valid_min):
                continue

            # itt van érvényes metszés
            px = t * dx
            py = t * dy

            # szakasz iránya (tangens)
            seg_len = math.hypot(sx, sy)
            if seg_len < 1e-6:
                continue
            tx = sx / seg_len
            ty = sy / seg_len

            # két lehetséges normál: bal / jobb
            nx1, ny1 = -ty, tx
            nx2, ny2 = ty, -tx

            # a robot a (0,0)-ban van, a falpont p = (px,py)
            # falpont -> robot vektor: (-px, -py)
            # válasszuk azt a normált, amelyik inkább a robot felé mutat:
            dot1 = nx1 * (-px) + ny1 * (-py)
            dot2 = nx2 * (-px) + ny2 * (-py)
            if dot1 > dot2:
                nx, ny = nx1, ny1
            else:
                nx, ny = nx2, ny2

            # csak a legközelebbit tartjuk meg
            if best_t is None or t < best_t:
                best_t = t
                best_nx = nx
                best_ny = ny

        if best_t is not None and math.isfinite(best_t):
            return min(best_t, self.range_clip_max), best_nx, best_ny

        return None