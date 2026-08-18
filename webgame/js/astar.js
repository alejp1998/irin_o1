/**
 * astar.js — 8-direction A* ported from iri1controller.cpp (IRIN O1, 2019).
 * 1:1 port of the Python module astar.py. Pure JS (no DOM).
 */
(function (root, factory) {
  if (typeof module === "object" && module.exports) module.exports = factory();
  else root.AStar = factory();
})(typeof self !== "undefined" ? self : this, function () {
  "use strict";

  var DIR = 8;
  var DX = [1, 1, 0, -1, -1, -1, 0, 1];
  var DY = [0, 1, 1, 1, 0, -1, -1, -1];

  function estimate(x, y, xDest, yDest) {
    return Math.max(Math.abs(x - xDest), Math.abs(y - yDest));
  }

  function nextLevel(level, i) {
    return level + (i % 2 === 0 ? 0 : 2); // straight 0, diagonal 2
  }

  /**
   * A* on a row-major grid grid[y][x] (0 free, 1 obstacle).
   * Returns { route, order, closed } where route is the direction-index string
   * ('' if unreachable), order is the exploration order (cells expanded) and
   * closed marks every closed cell — used by the visualizer.
   */
  function pathFind(grid, xStart, yStart, xFinish, yFinish) {
    var n = grid[0] ? grid[0].length : 0;
    var m = grid.length;
    var closed = [];
    var openMap = [];
    var dirMap = [];
    var order = [];
    var i, x, y, xdx, ydy, j;

    for (y = 0; y < m; y++) {
      closed.push(new Array(n).fill(0));
      openMap.push(new Array(n).fill(0));
      dirMap.push(new Array(n).fill(0));
    }

    if (
      xStart < 0 ||
      xStart >= n ||
      yStart < 0 ||
      yStart >= m ||
      xFinish < 0 ||
      xFinish >= n ||
      yFinish < 0 ||
      yFinish >= m
    )
      return { route: "", order: order, closed: closed };
    if (grid[yStart][xStart] === 1 || grid[yFinish][xFinish] === 1) {
      return { route: "", order: order, closed: closed };
    }

    // priority queue: [priority, level, x, y]
    var pq = [];
    function push(prio, lvl, px, py) {
      var k = pq.length;
      pq.push([prio, lvl, px, py]);
      while (k > 0) {
        var p = (k - 1) >> 1;
        if (pq[p][0] <= pq[k][0]) break;
        var tmp = pq[p];
        pq[p] = pq[k];
        pq[k] = tmp;
        k = p;
      }
    }
    function pop() {
      var top = pq[0];
      var last = pq.pop();
      if (pq.length) {
        pq[0] = last;
        var k = 0;
        for (;;) {
          var l = 2 * k + 1,
            r = 2 * k + 2,
            s = k;
          if (l < pq.length && pq[l][0] < pq[s][0]) s = l;
          if (r < pq.length && pq[r][0] < pq[s][0]) s = r;
          if (s === k) break;
          var t = pq[s];
          pq[s] = pq[k];
          pq[k] = t;
          k = s;
        }
      }
      return top;
    }

    push(estimate(xStart, yStart, xFinish, yFinish) * 10, 0, xStart, yStart);
    openMap[yStart][xStart] = estimate(xStart, yStart, xFinish, yFinish) * 10;

    while (pq.length) {
      var node = pop();
      var priority = node[0],
        level = node[1];
      x = node[2];
      y = node[3];
      if (openMap[y][x] !== priority) continue; // stale entry
      openMap[y][x] = 0;
      closed[y][x] = 1;
      order.push([x, y]);

      if (x === xFinish && y === yFinish) {
        var route = "";
        while (!(x === xStart && y === yStart)) {
          j = dirMap[y][x];
          route = String((j + DIR / 2) % DIR) + route;
          x = x + DX[j];
          y = y + DY[j];
        }
        return { route: route, order: order, closed: closed };
      }

      for (i = 0; i < DIR; i++) {
        xdx = x + DX[i];
        ydy = y + DY[i];
        if (
          xdx >= 0 &&
          xdx < n &&
          ydy >= 0 &&
          ydy < m &&
          grid[ydy][xdx] === 0 &&
          closed[ydy][xdx] === 0
        ) {
          var m0Level = nextLevel(level, i);
          var m0Priority = m0Level + estimate(xdx, ydy, xFinish, yFinish) * 10;
          if (openMap[ydy][xdx] === 0 || openMap[ydy][xdx] > m0Priority) {
            openMap[ydy][xdx] = m0Priority;
            dirMap[ydy][xdx] = (i + DIR / 2) % DIR;
            push(m0Priority, m0Level, xdx, ydy);
          }
        }
      }
    }
    return { route: "", order: order, closed: closed };
  }

  function gridFromString(rows, obstacle) {
    obstacle = obstacle || "#";
    return rows.map(function (row) {
      return row.split("").map(function (ch) {
        return ch === obstacle ? 1 : 0;
      });
    });
  }

  return {
    DIR: DIR,
    DX: DX,
    DY: DY,
    pathFind: pathFind,
    gridFromString: gridFromString,
  };
});
