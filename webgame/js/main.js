/**
 * IRIN O1 — A* visualizer: 20x20 grid, paint walls, animated search.
 */
(function () {
  "use strict";

  var A = window.AStar;
  var canvas = document.getElementById("view");
  var ctx = canvas.getContext("2d");
  var $id = function (id) {
    return document.getElementById(id);
  };

  var N = 20;
  var grid = [];
  for (var y = 0; y < N; y++) grid.push(new Array(N).fill(0));

  var start = [1, 18];
  var goal = [18, 1];

  var PAL = {
    dark: {
      bg: "#0b0f19",
      free: "#101a2c",
      wall: "#334155",
      wallEdge: "#475569",
      grid: "rgba(148,163,184,0.14)",
      open: "#f59e0b",
      closed: "#2563eb",
      route: "#22c55e",
      start: "#34d399",
      goal: "#f87171",
      text: "#e2e8f0",
      muted: "#94a3b8",
      panel: "#111c30",
      border: "rgba(255,255,255,0.1)",
    },
    light: {
      bg: "#f1f5f9",
      free: "#ffffff",
      wall: "#334155",
      wallEdge: "#94a3b8",
      grid: "rgba(15,23,42,0.08)",
      open: "#f59e0b",
      closed: "#3b82f6",
      route: "#059669",
      start: "#10b981",
      goal: "#ef4444",
      text: "#0f172a",
      muted: "#475569",
      panel: "#ffffff",
      border: "rgba(15,23,42,0.12)",
    },
  };
  function pal() {
    var t = document.documentElement.getAttribute("data-theme");
    return PAL[t === "dark" ? "dark" : "light"];
  }

  // ---------------------------------------------------------------- state
  var playing = false;
  var anim = null; // { order, closed, route, idx, routeSet }
  var mode = "start"; // start | goal | wall

  function log(msg) {
    var box = $id("log");
    var div = document.createElement("div");
    div.textContent = "› " + msg;
    box.appendChild(div);
    while (box.children.length > 60) box.removeChild(box.firstChild);
    box.scrollTop = box.scrollHeight;
  }

  // ---------------------------------------------------------------- render
  function sizeCanvas() {
    var panel = $id("stage-panel");
    var w = panel.clientWidth;
    var h = panel.clientHeight;
    var dpr = Math.max(1, window.devicePixelRatio || 1);
    canvas.width = Math.floor(w * dpr);
    canvas.height = Math.floor(h * dpr);
    canvas.style.width = w + "px";
    canvas.style.height = h + "px";
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  function render() {
    var p = pal();
    var w = canvas.width / Math.max(1, window.devicePixelRatio || 1);
    var h = canvas.height / Math.max(1, window.devicePixelRatio || 1);
    ctx.fillStyle = p.bg;
    ctx.fillRect(0, 0, w, h);

    var side = Math.min(w, h) - 48;
    var ox = (w - side) / 2;
    var oy = (h - side) / 2;
    var cell = side / N;

    // cells
    for (var y = 0; y < N; y++) {
      for (var x = 0; x < N; x++) {
        var col = p.free;
        if (grid[y][x] === 1) col = p.wall;
        else if (anim && anim.closed[y][x]) col = p.closed;
        ctx.fillStyle = col;
        ctx.fillRect(ox + x * cell, oy + y * cell, cell, cell);
        if (anim && anim.idx > 0 && !anim.closed[y][x]) {
          // cells currently in the open set get the amber tint
          var inOpen = anim.openSet && anim.openSet[y][x];
          if (inOpen) {
            ctx.fillStyle = p.open;
            ctx.globalAlpha = 0.75;
            ctx.fillRect(ox + x * cell, oy + y * cell, cell, cell);
            ctx.globalAlpha = 1;
          }
        }
      }
    }

    // grid lines
    ctx.strokeStyle = p.grid;
    ctx.lineWidth = 1;
    for (var i = 0; i <= N; i++) {
      ctx.beginPath();
      ctx.moveTo(ox + i * cell, oy);
      ctx.lineTo(ox + i * cell, oy + side);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(ox, oy + i * cell);
      ctx.lineTo(ox + side, oy + i * cell);
      ctx.stroke();
    }

    // route
    if (anim && anim.route && anim.routeSet) {
      var x = start[0],
        y = start[1];
      ctx.strokeStyle = p.route;
      ctx.lineWidth = Math.max(3, cell * 0.3);
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      ctx.beginPath();
      ctx.moveTo(ox + (x + 0.5) * cell, oy + (y + 0.5) * cell);
      var steps = anim.route.slice(0, anim.routeSet).split("");
      for (var k = 0; k < steps.length; k++) {
        var di = Number(steps[k]);
        x += A.DX[di];
        y += A.DY[di];
        ctx.lineTo(ox + (x + 0.5) * cell, oy + (y + 0.5) * cell);
      }
      ctx.stroke();
    }

    // start / goal markers
    drawMarker(start, p.start, "S");
    drawMarker(goal, p.goal, "G");

    function drawMarker(pt, color, label) {
      var cx = ox + (pt[0] + 0.5) * cell;
      var cy = oy + (pt[1] + 0.5) * cell;
      ctx.fillStyle = color;
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(cx, cy, cell * 0.36, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.fillStyle = "#ffffff";
      ctx.font = "700 " + Math.max(9, cell * 0.32) + "px system-ui";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(label, cx, cy + 1);
    }

    // legend
    var lx = 14;
    var ly = 14;
    ctx.fillStyle = p.panel + "dd";
    ctx.strokeStyle = p.border;
    ctx.beginPath();
    ctx.roundRect(lx, ly, 200, 26, 8);
    ctx.fill();
    ctx.stroke();
    var legend = [
      [p.open, "open"],
      [p.closed, "closed"],
      [p.route, "route"],
      [p.start, "start"],
      [p.goal, "goal"],
    ];
    var lx2 = lx + 12;
    ctx.font = "10px system-ui";
    ctx.textBaseline = "middle";
    legend.forEach(function (item) {
      ctx.fillStyle = item[0];
      ctx.beginPath();
      ctx.arc(lx2, ly + 13, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = p.muted;
      ctx.textAlign = "left";
      ctx.fillText(item[1], lx2 + 8, ly + 14);
      lx2 += 12 + ctx.measureText(item[1]).width + 14;
    });
  }

  // ---------------------------------------------------------------- animation
  function run() {
    if (grid[start[1]][start[0]] === 1) {
      log("⚠️ Start sits on a wall!");
      return;
    }
    if (grid[goal[1]][goal[0]] === 1) {
      log("⚠️ Goal sits on a wall!");
      return;
    }
    var res = A.pathFind(grid, start[0], start[1], goal[0], goal[1]);
    anim = {
      order: res.order,
      closed: res.closed,
      route: res.route,
      idx: 0,
      routeSet: 0,
    };
    playing = true;
    $id("hud-exp").textContent = "0";
    $id("hud-len").textContent = "—";
    log("▶ A* running…");
  }

  var speed = 12;
  function tick() {
    if (!anim) return;
    var advance = Math.max(1, Math.round(speed / 4));
    anim.idx = Math.min(anim.order.length, anim.idx + advance);
    var n = anim.order.length;
    var cur = anim.order.slice(0, anim.idx);
    anim.closed = [];
    for (var y = 0; y < N; y++) anim.closed.push(new Array(N).fill(0));
    cur.forEach(function (c) {
      anim.closed[c[1]][c[0]] = 1;
    });
    // open set: last frontier (the newest cells not yet closed)
    var frontier = cur.slice(-Math.min(cur.length, 40));
    anim.openSet = [];
    for (var yy = 0; yy < N; yy++) anim.openSet.push(new Array(N).fill(0));
    frontier.forEach(function (c) {
      anim.openSet[c[1]][c[0]] = 1;
    });
    $id("hud-exp").textContent = String(anim.idx);

    if (anim.idx >= n) {
      anim.openSet = null;
      if (anim.route) {
        var perStep = Math.max(1, Math.round(speed / 2));
        anim.routeSet = Math.min(
          anim.route.length,
          (anim.routeSet || 0) + perStep,
        );
        if (anim.routeSet >= anim.route.length) {
          playing = false;
          $id("hud-len").textContent = String(anim.route.length) + " steps";
          log(
            "✅ Route found: " +
              anim.route.length +
              " steps (" +
              anim.idx +
              " expanded).",
          );
          setResult(
            "✅ Route found!",
            anim.route.length + " steps · " + anim.idx + " cells expanded",
            "win",
          );
        }
      } else {
        playing = false;
        log(
          "❌ No route — goal unreachable (" + anim.idx + " cells expanded).",
        );
        setResult(
          "❌ Unreachable!",
          "No path exists with these walls.",
          "fail",
        );
      }
    }
  }

  function setResult(title, sub, cls) {
    var r = $id("hud-result");
    r.className = "hud-result " + cls;
    r.innerHTML =
      '<div class="hud-result-title">' +
      title +
      "</div>" +
      '<div class="hud-result-sub">' +
      sub +
      "</div>";
  }

  // ---------------------------------------------------------------- input
  function cellAt(e) {
    var r = canvas.getBoundingClientRect();
    var side = Math.min(r.width, r.height) - 48;
    var ox = (r.width - side) / 2;
    var oy = (r.height - side) / 2;
    var cell = side / N;
    var x = Math.floor((e.clientX - r.left - ox) / cell);
    var y = Math.floor((e.clientY - r.top - oy) / cell);
    if (x < 0 || x >= N || y < 0 || y >= N) return null;
    return [x, y];
  }

  var painting = false;

  function wireCanvas() {
    canvas.addEventListener("pointerdown", function (e) {
      var c = cellAt(e);
      if (!c) return;
      playing = false;
      anim = null;
      $id("hud-len").textContent = "—";
      $id("hud-exp").textContent = "—";
      if (e.button === 2) {
        mode = "wall";
        painting = true;
        grid[c[1]][c[0]] = 1;
      } else if (e.shiftKey) {
        goal = c;
      } else {
        start = c;
      }
      render();
    });
    canvas.addEventListener("pointermove", function (e) {
      if (!painting) return;
      var c = cellAt(e);
      if (c) {
        grid[c[1]][c[0]] = 1;
        render();
      }
    });
    canvas.addEventListener("pointerup", function () {
      painting = false;
    });
    canvas.addEventListener("contextmenu", function (e) {
      e.preventDefault();
    });
  }

  // ---------------------------------------------------------------- wiring
  function wire() {
    $id("btn-run").addEventListener("click", function () {
      run();
      var r = $id("hud-result");
      r.className = "hud-result hidden";
    });

    $id("btn-clear").addEventListener("click", function () {
      grid = [];
      for (var y = 0; y < N; y++) grid.push(new Array(N).fill(0));
      anim = null;
      playing = false;
      $id("hud-len").textContent = "—";
      $id("hud-exp").textContent = "—";
      $id("hud-result").className = "hud-result hidden";
      log("🧹 Walls cleared.");
      render();
    });

    $id("btn-arena").addEventListener("click", function () {
      // an arena inspired by the O1 Webots map (walls + blocks)
      grid = [];
      for (var y = 0; y < N; y++) grid.push(new Array(N).fill(0));
      var walls = [
        [4, 2, 6, 4],
        [14, 2, 4, 5],
        [4, 14, 5, 4],
        [13, 14, 6, 3],
        [9, 7, 2, 6],
        [2, 8, 2, 4],
      ];
      walls.forEach(function (wr) {
        for (var yy = wr[1]; yy < wr[1] + wr[3]; yy++)
          for (var xx = wr[0]; xx < wr[0] + wr[2]; xx++)
            if (xx >= 0 && xx < N && yy >= 0 && yy < N) grid[yy][xx] = 1;
      });
      start = [1, 18];
      goal = [18, 1];
      anim = null;
      playing = false;
      log("🏟️ O1 arena preset loaded (20×20).");
      render();
    });

    $id("speed").addEventListener("input", function () {
      speed = Number(this.value);
      $id("speed-v").textContent = this.value + "×";
    });

    $id("btn-theme").addEventListener("click", function () {
      var t =
        document.documentElement.getAttribute("data-theme") === "dark"
          ? "light"
          : "dark";
      document.documentElement.setAttribute("data-theme", t);
      try {
        localStorage.setItem("theme", t);
      } catch (e) {}
      applyTheme();
      render();
    });
    window
      .matchMedia("(prefers-color-scheme: dark)")
      .addEventListener("change", function (ev) {
        if (localStorage.getItem("theme")) return;
        document.documentElement.setAttribute(
          "data-theme",
          ev.matches ? "dark" : "light",
        );
        applyTheme();
        render();
      });
    window.addEventListener("resize", function () {
      sizeCanvas();
      render();
    });
  }

  function applyTheme() {
    var t = document.documentElement.getAttribute("data-theme");
    $id("btn-theme").textContent = t === "dark" ? "☀️" : "🌙";
  }

  // guide
  var guideOpen = false;
  function wireGuide() {
    var guide = $id("guide");
    function open() {
      guideOpen = true;
      guide.classList.remove("hidden");
    }
    function close() {
      guideOpen = false;
      guide.classList.add("hidden");
    }
    $id("btn-guide").addEventListener("click", open);
    guide.querySelectorAll("[data-close-guide]").forEach(function (el) {
      el.addEventListener("click", close);
    });
    document.addEventListener("keydown", function (e) {
      if (e.code === "Escape" && guideOpen) close();
    });
  }

  // ---------------------------------------------------------------- loop
  function init() {
    applyTheme();
    wireGuide();
    wire();
    wireCanvas();
    sizeCanvas();
    // default arena on load
    $id("btn-arena").click();
    log("🎯 Left-click start · shift-click goal · right-click walls.");
    render();

    function loop() {
      if (playing) tick();
      render();
      requestAnimationFrame(loop);
    }
    requestAnimationFrame(loop);
  }

  init();
})();
