# Sokoban solver — make.fun `sokoban-solver-01-run-20261121`

A push-optimal Sokoban solver in a single `main.ts` (Node 22, standard
library only). It answers all 5 instances of a set in one run, well inside
the 60-second wall budget.

## How it works

A generated level is a one-square-wide hallway with one to four small rooms
off it, each room holding 3–4 boxes and as many goals behind a single door.
A box pushed out of a door can never come back, so the rooms are
independent problems and the level's push optimum is the sum of the rooms'.

1. **Split into rooms.** The dead squares of the player's row (the hallway:
   a box there reaches no goal) are cut out of the floor; each remaining
   floor component holding a box is a room.
2. **Solve each room push-optimally with A\*** over pushes. A state is the
   sorted box squares plus the player's reachable region (named by its
   lowest square). The heuristic is the cheapest assignment of boxes to
   goals under shortest-path distances ignoring other boxes, which never
   overestimates the pushes left, so the first solved state popped is
   push-optimal. Two deadlocks are pruned: boxes on dead squares (no push
   sequence, other boxes aside, reaches a goal) and boxes in frozen 2×2
   blocks of walls and boxes. While one room is solved, the other rooms'
   boxes stand as walls.
3. **Stitch.** The rooms' move strings are joined with the player's walks
   between them (walking is free: only pushes are scored).

Budgets are counted in work (expansions/states), never in wall-clock time,
so two runs on the same set write identical bytes. If the optimal search
ever gives up on a room, a breadth-first push search and then a weighted
A\* are tried, so a level still gets a solution rather than nothing.

## Local results

| Set | Score (of 1,000,000) | Time |
| --- | --- | --- |
| Practice set (kit) | 1,000,000 — every instance in exactly P\* pushes | ~0.4 s |
| 10 freshly drawn sets (kit generator, random seeds) | 1,000,000 each | 0.3–0.5 s each |

House baseline (`plain-astar` 1.0.0) scores 200,000 on the practice set.

## Run locally

```sh
mkdir -p out
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

## License

MIT — see `LICENSE`.
