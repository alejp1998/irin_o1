/**
 * Node tests: JS A* port must behave exactly like the Python module.
 */
const { test } = require("node:test");
const assert = require("node:assert");

const A = require("../js/astar.js");

function follow(route, x0, y0) {
  let x = x0,
    y = y0;
  for (const ch of route) {
    const i = Number(ch);
    x += A.DX[i];
    y += A.DY[i];
  }
  return [x, y];
}

test("finds straight path on open grid", () => {
  const g = A.gridFromString(["........", "........", "........"]);
  const r = A.pathFind(g, 0, 0, 7, 0);
  assert.strictEqual(r.route, "0000000");
  assert.deepStrictEqual(follow(r.route, 0, 0), [7, 0]);
});

test("avoids obstacles", () => {
  const g = A.gridFromString(["........", "........", "..####..", "........"]);
  const r = A.pathFind(g, 0, 0, 7, 3);
  assert.ok(r.route.length > 0);
  let x = 0,
    y = 0;
  for (const ch of r.route) {
    const i = Number(ch);
    x += A.DX[i];
    y += A.DY[i];
    assert.strictEqual(g[y][x], 0, "route must not step on an obstacle");
  }
  assert.deepStrictEqual([x, y], [7, 3]);
});

test("unreachable goal returns empty route", () => {
  const g = A.gridFromString([".####.", ".#..#.", ".#..#.", ".####."]);
  const r = A.pathFind(g, 0, 0, 2, 2);
  assert.strictEqual(r.route, "");
});

test("diagonal shortcut preferred (same as C++)", () => {
  const g = A.gridFromString(["...", "...", "..."]);
  const r = A.pathFind(g, 0, 0, 2, 2);
  assert.strictEqual(r.route, "11");
});

test("goal equals start -> empty route, no expansion", () => {
  const g = A.gridFromString(["....."]);
  const r = A.pathFind(g, 2, 0, 2, 0);
  assert.strictEqual(r.route, "");
  assert.strictEqual(r.order.length, 1); // start popped, goal check
});

test("out of bounds / obstacle start returns empty", () => {
  const g = A.gridFromString(["....#", "....."]);
  assert.strictEqual(A.pathFind(g, -1, 0, 4, 1).route, "");
  assert.strictEqual(A.pathFind(g, 4, 0, 0, 0).route, "");
});

test("exploration order ends at the goal", () => {
  const g = A.gridFromString([".....", ".....", "....."]);
  const r = A.pathFind(g, 0, 0, 4, 2);
  assert.deepStrictEqual(r.order[r.order.length - 1], [4, 2]);
});
