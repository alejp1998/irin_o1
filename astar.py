
"""
astar.py — the 8-direction A* planner from iri1controller.cpp (IRIN O1, 2019),
extracted as a reusable, testable module.

Faithful port of the controller's pathFind():
- 8 directions with straight-line bias (nextLevel: straight = 0, diagonal = 2)
- priority = level + estimate * 10
- route returned as a string of direction indices (same encoding as the C++)

Grids are row-major: grid[y][x], 0 = free, 1 = obstacle.
"""

from __future__ import annotations

from heapq import heappop, heappush

DIR = 8
DX = [1, 1, 0, -1, -1, -1, 0, 1]
DY = [0, 1, 1, 1, 0, -1, -1, -1]


def _estimate(x: int, y: int, x_dest: int, y_dest: int) -> int:
    """Chebyshev-like distance used by the controller's node::estimate."""
    return max(abs(x - x_dest), abs(y - y_dest))


def next_level(level: int, i: int) -> int:
    """Straight moves cost 0 extra, diagonals cost 2 (controller's nextLevel)."""
    return level + (0 if i in (0, 2, 4, 6) else 2)


def path_find(
    grid: list[list[int]],
    x_start: int,
    y_start: int,
    x_finish: int,
    y_finish: int,
) -> str:
    """A* path from (xStart,yStart) to (xFinish,yFinish) on a row-major 0/1 grid.

    grid[y][x]: 0 = free, 1 = obstacle. Returns the route as a string of
    direction indices (each char '0'..'7', dx/dy lookup), or '' if
    unreachable — like the controller's pathFind().
    """
    n = len(grid[0]) if grid else 0  # columns (x range)
    m = len(grid)  # rows (y range)
    if not (0 <= x_start < n and 0 <= y_start < m and 0 <= x_finish < n and 0 <= y_finish < m):
        return ""
    if grid[y_start][x_start] == 1 or grid[y_finish][x_finish] == 1:
        return ""

    closed = [[0] * n for _ in range(m)]
    open_map = [[0] * n for _ in range(m)]
    dir_map = [[0] * n for _ in range(m)]
    pq: list[tuple[int, int, int, int]] = []  # (priority, level, x, y)

    heappush(pq, (_estimate(x_start, y_start, x_finish, y_finish) * 10, 0, x_start, y_start))
    open_map[y_start][x_start] = _estimate(x_start, y_start, x_finish, y_finish) * 10

    while pq:
        priority, level, x, y = heappop(pq)
        # stale entry: a better priority for this node was found since it was pushed
        if open_map[y][x] != priority:
            continue
        open_map[y][x] = 0
        closed[y][x] = 1

        if x == x_finish and y == y_finish:
            # reconstruct by following the stored BACKWARD directions toward the
            # start (dir_map stores (i+4)%8, exactly like the controller); the
            # route chars are flipped back to the forward directions
            path = ""
            while not (x == x_start and y == y_start):
                j = dir_map[y][x]
                path = str((j + DIR // 2) % DIR) + path
                x = x + DX[j]
                y = y + DY[j]
            return path

        for i in range(DIR):
            xdx = x + DX[i]
            ydy = y + DY[i]
            if (
                0 <= xdx < n
                and 0 <= ydy < m
                and grid[ydy][xdx] == 0
                and closed[ydy][xdx] == 0
            ):
                m0_level = next_level(level, i)
                m0_priority = m0_level + _estimate(xdx, ydy, x_finish, y_finish) * 10
                if open_map[ydy][xdx] == 0 or open_map[ydy][xdx] > m0_priority:
                    open_map[ydy][xdx] = m0_priority
                    dir_map[ydy][xdx] = (i + DIR // 2) % DIR
                    heappush(pq, (m0_priority, m0_level, xdx, ydy))

    return ""


def grid_from_string(rows: list[str], obstacle: str = "#") -> list[list[int]]:
    """Parse ASCII rows (top = y=0) into a row-major 0/1 grid ('#' = obstacle)."""
    return [[1 if ch == obstacle else 0 for ch in row] for row in rows]
