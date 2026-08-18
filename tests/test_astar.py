"""Tests for the A* planner ported from iri1controller.cpp (IRIN O1)."""

import astar


def make_grid(rows):
    return astar.grid_from_string(rows)


def test_finds_straight_path():
    g = make_grid(["........", "........", "........"])
    route = astar.path_find(g, 0, 0, 7, 0)
    assert route != ""
    # follow the route and check it lands exactly on the goal
    x, y = 0, 0
    for ch in route:
        i = int(ch)
        x += astar.DX[i]
        y += astar.DY[i]
    assert (x, y) == (7, 0)
    assert len(route) == 7  # optimal straight run


def test_avoids_obstacle():
    g = make_grid(
        [
            "........",
            "........",
            "..####..",
            "........",
        ]
    )
    route = astar.path_find(g, 0, 0, 7, 3)
    assert route != ""
    x, y = 0, 0
    for ch in route:
        i = int(ch)
        x += astar.DX[i]
        y += astar.DY[i]
        assert g[y][x] == 0, "route must never step on an obstacle"
    assert (x, y) == (7, 3)


def test_unreachable_returns_empty():
    g = make_grid(
        [
            "......",
            ".####.",
            ".#..#.",
            ".#..#.",
            ".####.",
        ]
    )
    route = astar.path_find(g, 0, 0, 3, 2)  # goal sealed inside walls
    assert route == ""


def test_goal_equals_start():
    g = make_grid(["....."])
    assert astar.path_find(g, 2, 0, 2, 0) == ""


def test_start_or_goal_on_obstacle_returns_empty():
    g = make_grid(["....#"])
    assert astar.path_find(g, 4, 0, 0, 0) == ""
    assert astar.path_find(g, 0, 0, 4, 0) == ""


def test_out_of_bounds_returns_empty():
    g = make_grid(["....."])
    assert astar.path_find(g, -1, 0, 4, 0) == ""
    assert astar.path_find(g, 0, 0, 9, 9) == ""


def test_diagonal_shortcut_preferred_over_staircase():
    # With the controller's costs (straight +0, diagonal +2) the diagonal
    # shortcut reaches (2,2) at priority 4 while the straight staircase only
    # arrives later (its first step pops at priority 20) — so the A* returns
    # the 2 diagonal moves, exactly like the C++.
    g = make_grid(["...", "...", "..."])
    route = astar.path_find(g, 0, 0, 2, 2)
    assert route == "11"  # two diagonal moves
    x, y = 0, 0
    for ch in route:
        i = int(ch)
        x += astar.DX[i]
        y += astar.DY[i]
    assert (x, y) == (2, 2)


def test_next_level_straight_bias():
    assert astar.next_level(0, 0) == 0  # straight
    assert astar.next_level(0, 2) == 0
    assert astar.next_level(0, 1) == 2  # diagonal
    assert astar.next_level(0, 7) == 2
