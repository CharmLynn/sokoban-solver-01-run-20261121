// Sokoban solver: room decomposition + push-optimal A* per room.
//
// A generated level is a one-square-wide hallway with one to four small rooms
// off it, each room holding 3-4 boxes and as many goals, joined to the hallway
// by a single door. A box pushed out of a door can never come back, so the
// rooms are independent: the level's push optimum is the sum of the rooms'.
// This solver therefore splits a level into rooms and solves each room
// push-optimally on its own, then stitches the rooms' move strings together
// with the player's walks between them.
//
// Each room is solved with A* over pushes. A state is the sorted box squares
// plus the player's reachable region (named by its lowest square, which is a
// canonical name for the region). The heuristic is the cheapest assignment of
// boxes to goals under shortest-path distances that ignore the other boxes,
// which never overestimates the pushes still needed, so the first solved
// state popped is push-optimal. Two deadlocks are pruned, both of which can
// never occur on a solvable path: a box on a dead square (a square from
// which no sequence of pushes, other boxes aside, reaches any goal) and a
// box in a frozen 2x2 block of walls and boxes with a box off its goal.
//
// Budgets are counted in work (expansions / states), never in wall-clock
// time, so two runs on the same set write the same bytes. If the optimal
// search ever gives up on a room, a breadth-first push search and then a
// weighted A* are tried, so a level still gets a (possibly non-optimal)
// solution rather than nothing.
//
// Local run: mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Work budgets (expansions / states), deterministic: never the clock.
const ASTAR_CAP = 150_000;
const BFS_CAP = 250_000;
const WEIGHTED_CAP = 150_000;
const GLOBAL_CAP = 4_000_000;

let workUsed = 0;

const DIRS = [
  [0, -1, 'u'],
  [0, 1, 'd'],
  [-1, 0, 'l'],
  [1, 0, 'r'],
];

// ---------------------------------------------------------------- parsing

function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals = [];
  const boxes = [];
  let player = -1;
  for (let y = 0; y < h; y++) {
    const row = rows[y];
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  }
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
    }
  }
  return { w, h, wall, goals, boxes: boxes.sort((a, b) => a - b), player };
}

// ------------------------------------------------------- board primitives

// Squares from which a lone box can be pushed onto some goal: flood
// backwards from the goals, a pull at a time, over open floor.
function liveSquares(w, wall, goals) {
  const live = new Uint8Array(wall.length);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi];
    for (let k = 0; k < 4; k++) {
      const d = DIRS[k][1] * w + DIRS[k][0];
      const y = x - d;
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

// Shortest walking distances from each goal over open floor, ignoring
// boxes: dist[g][cell]. Used by the assignment heuristic.
function goalDistances(w, wall, goals) {
  const dist = goals.map(() => new Int32Array(wall.length).fill(-1));
  goals.forEach((g, gi) => {
    const queue = [g];
    dist[gi][g] = 0;
    for (let qi = 0; qi < queue.length; qi++) {
      const p = queue[qi];
      for (let k = 0; k < 4; k++) {
        const q = p + DIRS[k][1] * w + DIRS[k][0];
        if (!wall[q] && dist[gi][q] < 0) {
          dist[gi][q] = dist[gi][p] + 1;
          queue.push(q);
        }
      }
    }
  });
  return dist;
}

// Flood the player's reachable region around walls and boxes; returns its
// lowest square (the region's canonical name) and marks the region in
// `seen` under `stamp`.
function floodRegion(w, wall, occupied, from, seen, stamp, stack) {
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n];
    if (p < low) low = p;
    for (let k = 0; k < 4; k++) {
      const q = p + DIRS[k][1] * w + DIRS[k][0];
      if (!wall[q] && !occupied[q] && seen[q] !== stamp) {
        seen[q] = stamp;
        stack[n++] = q;
      }
    }
  }
  return low;
}

// Whether the box just pushed to `t` is caught in a frozen 2x2 block:
// four squares of walls and boxes holding a box off its goal.
function frozenBlock(w, wall, occupied, goal, t) {
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (!(wall[a] || occupied[a])) continue;
    if (!(wall[b] || occupied[b])) continue;
    if (!(wall[c] || occupied[c])) continue;
    if (!(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goal[a]) || (occupied[b] && !goal[b]) || (occupied[c] && !goal[c]) || (occupied[d] && !goal[d])) return true;
  }
  return false;
}

// The shortest walk from `from` to `to` around walls and boxes, in lower
// LURD letters; '' when already there. Throws when unreachable.
function walkTo(w, wall, occupied, from, to) {
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi];
    if (p === to) break;
    for (let k = 0; k < 4; k++) {
      const q = p + DIRS[k][1] * w + DIRS[k][0];
      if (wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = DIRS[k][2];
      queue.push(q);
    }
  }
  if (prev[to] === -1) throw new Error('walkTo: target out of reach');
  let s = '';
  for (let p = to; p !== from; p = prev[p]) s = how[p] + s;
  return s;
}

// ------------------------------------------------------------ push search

// A state's key: its region's lowest square and its boxes' squares. Exact
// as a number while size ** (boxes + 1) stays under 2 ** 53 (rooms hold at
// most 4 boxes on boards this generator draws).
function keyOf(boxes, low, size) {
  let k = low;
  for (const b of boxes) k = k * size + b;
  return k;
}

// Cheapest assignment of boxes to goals under the precomputed distances;
// Infinity when some box cannot reach any goal. With more than 4 boxes the
// full assignment is too big, so the sum of each box's nearest goal is used
// instead (still a lower bound on the pushes left).
function assignmentCost(boxes, goals, dist) {
  const n = boxes.length;
  if (n > 4) {
    let s = 0;
    for (const b of boxes) {
      let m = Infinity;
      for (let gi = 0; gi < goals.length; gi++) {
        const d = dist[gi][b];
        if (d >= 0 && d < m) m = d;
      }
      if (m === Infinity) return Infinity;
      s += m;
    }
    return s;
  }
  const order = [...goals.keys()];
  let best = Infinity;
  // All permutations of the goals (n <= 4, so at most 24).
  const permute = (i) => {
    if (i === n) {
      let s = 0;
      for (let j = 0; j < n; j++) {
        const d = dist[order[j]][boxes[j]];
        if (d < 0) return;
        s += d;
      }
      if (s < best) best = s;
      return;
    }
    for (let j = i; j < n; j++) {
      [order[i], order[j]] = [order[j], order[i]];
      permute(i + 1);
      [order[i], order[j]] = [order[j], order[i]];
    }
  };
  permute(0);
  return best;
}

// Push search over one board: {w, wall, goals, boxes, player}. Returns the
// list of pushes ([from, dirIndex]) from the start to a solved state, or
// null. mode 'astar' is optimal (admissible assignment heuristic);
// 'bfs' expands in push order (also optimal); 'weighted' multiplies the
// heuristic (fast, not optimal) as a last resort.
function pushSearch(board, mode, cap) {
  const { w, wall, goals } = board;
  const size = wall.length;
  const n = board.boxes.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(w, wall, goals);
  const dist = mode === 'bfs' ? null : goalDistances(w, wall, goals);
  const weight = mode === 'weighted' ? 2 : 1;
  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;

  const solved = (boxes) => boxes.every((b) => goal[b]);
  if (board.boxes.some((b) => !live[b])) return null;

  // States, in discovery order: boxes, the player's square after the push
  // that made them, parent index, the push (box's old square, dir), and
  // the region's lowest square (its canonical name, for the state's key).
  const sBoxes = [board.boxes.slice()];
  const sPlayer = [board.player];
  const sParent = [-1];
  const sFrom = [-1];
  const sDir = [-1];
  const sG = [0];
  const sF = [0];
  const sSeq = [0];
  const sLow = [-1];
  let seq = 1;

  // A state's key: its region's lowest square and its boxes' squares. A
  // number while the board is small and the box count low (rooms), a
  // string otherwise, so a whole-level search never collides.
  const numericKey = n <= 4 && Math.pow(size, n + 1) < 2 ** 53;
  const key = (boxes, low) => {
    if (numericKey) return keyOf(boxes, low, size);
    return low + '|' + boxes.join(',');
  };
  const closed = new Map();
  for (const b of board.boxes) occupied[b] = 1;
  sLow[0] = floodRegion(w, wall, occupied, board.player, seen, ++stamp, stack);
  for (const b of board.boxes) occupied[b] = 0;
  closed.set(key(board.boxes, sLow[0]), 0);
  sF[0] = mode === 'bfs' ? 0 : assignmentCost(board.boxes, goals, dist);

  // Binary heap of state indices, ordered by (f, deeper g first, discovery).
  const heap = [0];
  const less = (a, b) =>
    sF[a] < sF[b] ||
    (sF[a] === sF[b] && (sG[a] > sG[b] || (sG[a] === sG[b] && sSeq[a] < sSeq[b])));
  const heapPush = (i) => {
    let c = heap.length;
    heap.push(i);
    while (c > 0) {
      const p = (c - 1) >> 1;
      if (!less(heap[c], heap[p])) break;
      [heap[c], heap[p]] = [heap[p], heap[c]];
      c = p;
    }
  };
  const heapPop = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
        if (!less(heap[c], heap[i])) break;
        [heap[i], heap[c]] = [heap[c], heap[i]];
        i = c;
      }
    }
    return top;
  };

  let expanded = 0;
  while (heap.length > 0) {
    const s = heapPop();
    // A stale entry: this state was reached again with fewer pushes since.
    if (closed.get(key(sBoxes[s], sLow[s])) !== sG[s]) continue;
    const boxes = sBoxes[s];
    if (solved(boxes)) {
      const pushes = [];
      for (let t = s; t > 0; t = sParent[t]) pushes.push([sFrom[t], sDir[t]]);
      pushes.reverse();
      return pushes;
    }
    if (++expanded > cap) return null;
    workUsed++;
    // The player's reachable region from this state.
    for (const b of boxes) occupied[b] = 1;
    const regionStamp = ++stamp;
    floodRegion(w, wall, occupied, sPlayer[s], seen, regionStamp, stack);
    for (let bi = 0; bi < n; bi++) {
      const b = boxes[bi];
      for (let k = 0; k < 4; k++) {
        const d = DIRS[k][1] * w + DIRS[k][0];
        const t = b + d;
        // The player must stand on the far side of the box to push it.
        if (seen[b - d] !== regionStamp) continue;
        if (wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(w, wall, occupied, goal, t);
        let next = null;
        let nk = -1;
        let nlow = -1;
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((a, b2) => a - b2);
          nlow = floodRegion(w, wall, occupied, b, mark, ++stamp, stack);
          nk = key(next, nlow);
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen || nk < 0) continue;
        const g = sG[s] + 1;
        const prev = closed.get(nk);
        if (prev !== undefined && prev <= g) continue;
        closed.set(nk, g);
        const h = mode === 'bfs' ? 0 : assignmentCost(next, goals, dist) * weight;
        if (h === Infinity) continue;
        const idx = sBoxes.length;
        sBoxes.push(next);
        sPlayer.push(b);
        sParent.push(s);
        sFrom.push(b);
        sDir.push(k);
        sG.push(g);
        sF.push(g + h);
        sSeq.push(seq++);
        sLow.push(nlow);
        heapPush(idx);
      }
    }
    for (const b of boxes) occupied[b] = 0;
  }
  return null;
}

// Turns a push list into a move string on `board`, walking the player to
// each push square first. Returns {moves, endPlayer}.
function pushesToMoves(board, pushes) {
  const { w, wall } = board;
  const occupied = new Uint8Array(wall.length);
  let player = board.player;
  let boxes = board.boxes.slice();
  let moves = '';
  for (const [from, k] of pushes) {
    const d = DIRS[k][1] * w + DIRS[k][0];
    for (const b of boxes) occupied[b] = 1;
    moves += walkTo(w, wall, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    moves += DIRS[k][2];
    player = from;
    boxes = boxes.map((b) => (b === from ? from + d : b)).sort((a, b) => a - b);
  }
  return { moves, endPlayer: player };
}

// ------------------------------------------------------------------ rooms

// A level's rooms: cut the dead squares of the player's row (the hallway:
// a box there reaches no goal) out of the floor; each remaining part that
// holds a box is a room. Null when the level is no set of rooms.
function roomsOf(level) {
  const { w, wall, goals, boxes, player } = level;
  const live = liveSquares(w, wall, goals);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) {
    if (!wall[i] && !live[i]) cut[i] = 1;
  }
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (let k = 0; k < 4; k++) {
        const q = squares[i] + DIRS[k][1] * w + DIRS[k][0];
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ squares, boxes: [], goals: [] });
  }
  for (const b of boxes) rooms[part[b]].boxes.push(b);
  for (const g of goals) {
    if (part[g] >= 0) rooms[part[g]].goals.push(g);
  }
  if (goals.some((g) => part[g] < 0)) return null;
  if (rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

// Solves one room push-optimally (with fallbacks), the other rooms' boxes
// standing as walls. Returns {moves, endPlayer, boxes} or null.
function solveRoom(level, room, otherBoxes, player) {
  const wall = level.wall.slice();
  for (const b of otherBoxes) wall[b] = 1;
  const board = {
    w: level.w,
    wall,
    goals: room.goals.slice().sort((a, b) => a - b),
    boxes: room.boxes.slice().sort((a, b) => a - b),
    player,
  };
  const caps = workUsed > GLOBAL_CAP
    ? [['weighted', 60_000]]
    : [
        ['astar', ASTAR_CAP],
        ['bfs', BFS_CAP],
        ['weighted', WEIGHTED_CAP],
      ];
  for (const [mode, cap] of caps) {
    const pushes = pushSearch(board, mode, cap);
    if (pushes === null) continue;
    const { moves, endPlayer } = pushesToMoves(board, pushes);
    let bs = board.boxes.slice();
    for (const [from, k] of pushes) {
      const d = DIRS[k][1] * level.w + DIRS[k][0];
      bs = bs.map((b) => (b === from ? from + d : b)).sort((a, b) => a - b);
    }
    return { moves, endPlayer, boxes: bs };
  }
  return null;
}

// ------------------------------------------------------------------ solve

function solveLevel(text) {
  const level = parseLevel(text);
  const rooms = roomsOf(level);
  // No room structure: fall back to a push search over the whole level.
  if (rooms === null) {
    const board = { w: level.w, wall: level.wall, goals: level.goals, boxes: level.boxes, player: level.player };
    for (const [mode, cap] of [['astar', ASTAR_CAP], ['bfs', BFS_CAP], ['weighted', WEIGHTED_CAP]]) {
      const pushes = pushSearch(board, mode, cap);
      if (pushes !== null) return pushesToMoves(board, pushes).moves;
    }
    return '';
  }
  let player = level.player;
  let moves = '';
  // The rooms' current boxes, updated as each room is solved.
  const currentBoxes = rooms.map((r) => r.boxes.slice().sort((a, b) => a - b));
  for (let ri = 0; ri < rooms.length; ri++) {
    const others = [];
    for (let oj = 0; oj < rooms.length; oj++) {
      if (oj !== ri) others.push(...currentBoxes[oj]);
    }
    const room = { ...rooms[ri], boxes: currentBoxes[ri] };
    const solved = solveRoom(level, room, others, player);
    if (solved === null) return '';
    moves += solved.moves;
    player = solved.endPlayer;
    currentBoxes[ri] = solved.boxes;
  }
  return moves;
}

for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
