import numpy as np

def in_los(buildings, r, p1, p2, c, x_min, x_max, y_min, y_max):
    def point_line_distance(px, py, x1, y1, x2, y2):
        line_mag = np.hypot(x2 - x1, y2 - y1)
        if line_mag == 0:
            return np.hypot(px - x1, py - y1)

        u = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / (line_mag ** 2)
        u = max(min(u, 1), 0)
        closest_x = x1 + u * (x2 - x1)
        closest_y = y1 + u * (y2 - y1)
        return np.hypot(px - closest_x, py - closest_y)

    x1, y1 = p1
    x2, y2 = p2
    if(np.hypot(x2 - x1, y2 - y1) > 2.5):
        return False
    for bx, by in buildings:
        if(bx >= x_max or bx <= x_min or by >= y_max or by <= y_min):
            continue
        distance = point_line_distance(bx, by, x1, y1, x2, y2)
        if distance <= r + c:
            return False
    return True

def create_matrix(world_width, world_height, buildings, edge, r, c):
    m = (int)(world_width / edge) + 1
    n = (int)(world_height / edge) + 1
    matrix = np.zeros((m, n, m, n), dtype=bool)

    for i1 in range(m):
        for j1 in range(n):
            for i2 in range(i1, m):
                for j2 in range(n):
                    x1, y1 = i1 * edge, j1 * edge
                    x2, y2 = i2 * edge, j2 * edge
                    matrix[i1][j1][i2][j2] = in_los(buildings, r, (x1, y1), (x2, y2), c, min(x1, x2) - edge, max(x1, x2) + edge, min(y1, y2) - edge, max(y1, y2) + edge)
                    matrix[i2][j2][i1][j1] = matrix[i1][j1][i2][j2]
    return matrix
