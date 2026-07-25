# Eternity II solver

C++ backtracking solver for [Eternity II](https://en.wikipedia.org/wiki/Eternity_II_puzzle)
that reaches **465 of 480 matched edges with all five official clue pieces forced
into their published positions**.
At the time of writing (2026-07-25) this is the record for the five official clues score.
Repo contains:
- The source code
- The board
- The exact command that produced it

```
$ ./build.sh && ./reproduce_465.sh
== searching (seed 784944885, 15 threads, solver ./bw)
ESCALATED COMPLETION: 465/480 thread 3 seed 784944890497614205 rung 1 rewind 236 spent 12 nodes 1655742
== raw board vs recorded board_465_rot90_raw.txt
   IDENTICAL
== independent rescore
   matched edges : 465/480  (15 mismatches)
   valid board   : OK
== REPRODUCED: 465/480 with all five official clues
```

Roughly 4 seconds on a Ryzen 9 with 32 threads.

[View the board](
https://e2.bucas.name/#puzzle=Clues&board_w=16&board_h=16&board_edges=acdaacocadgcabmdabgbacsbacpcaepcaeteaeueadweadsdacidafvcadofaabddteaovntgwwvmsowpgmssqugphjqptwhtkmtuvtkwppvsuvpiowuvuwooujubafueseannnswlmnoiqwwiliujsijrijwvlrmtrvtlqtprilvmkrwuhmwswujuhsfafuepbansvpmwkskgrjlgmtsjwgiggjlorgrrmoqunrisouknhshlsnwmulhgimfafgboeavjqokqtjrqpqmqgqwsoqgvgsrnkvmpjnnigpomnihmrmsommurhoiqorfafqeqbaqwtqtnlkplsnqjklovmjgtqvksntjgisgkogntkkrwhtmqhwhmkqonpmfacnbteatpjtloupsnqokolnmnloqgtnnkpgigmkolugkvulhnlvhhunkrphphhrcabhendavoknuvgovlgvlkpllqiktjoqpjijmlljulpluqrllhwqunvhpgnnhhlgbachdidakkgigtrkgiwtqooiiisoommiijjmlthjplmtrlslwvwlvitvnqwilhuqcafhdpcagtnprgrttgigoslgskssmnrkjronhmormupmsgruwstgthuswjhhumhjfaemcqeangwqringiwgilrqwsmtrrwmmoklwokvkppskrhkptvphusuvhwushqkweafqeibagqiinqoqgmpqqntmtqvnmokqllwovmqlsutmksiuphssukjhuwokkuhwfadubveaijjlosujpwnstwvwvvlwkvwvwkrvqjsktrujijurshvjjnthouqnhhrudadhencajminujvmnvijvijvliriwquirpnqstopusrtuhlsvrkhtrqrqvprrtkvdaftcvbaiwpmvjgwistjjprsroppunkonvjnomjvrgtmllogkillqphippjpkrwpfacrbeaapdaegdadtcadrbacpcabkfacjbafjbabtfaboeaflfaehbafjfabwdafcaad)


## What is here

| path | |
|---|---|
| `bw.cpp` | the whole solver in one file; no dependencies beyond libstdc++ and pthreads |
| `build.sh` | plain `-O3` build, or a profile-guided (PGO) clang build |
| `reproduce_465.sh` | replays the winning seed and verifies the board three ways |
| `run_465eg.sh` | the campaign wrapper: restart loop, seeding, stop-on-target |
| `256pieces.txt` | the puzzle: one line per piece, `top bottom left right` colours, 0 = grey edge |
| `hints.txt` | the five official clue placements, official coordinates |
| `hints_rot90.txt`, `hints_rot270.txt` | the same clues for the instance rotated 90°/270° clockwise |
| `results_465/` | the board in raw and official coordinates, plus its metadata sidecar and rescore |
| `tools/rotate_board.py` | turn a board between rotated and official coordinates |
| `tools/to_bucas.py` | independent rescore + validity check, emits a bucas viewer URL |

`board_465_rot90_raw.txt.meta` is the engine's own sidecar: score, thread,
attempt seed, which fill indices actually spent a break, and two viewer URLs.


## Build

```bash
./build.sh          # clang -O3 -march=native          -> ./bw
./build.sh gcc      # g++ -O3 -march=native            -> ./bw
./build.sh pgo      # clang, profile-guided (~2 min)   -> ./bw_eg
```

PGO is profile-guided optimization: `build.sh pgo` compiles an instrumented
binary, runs it for 90 seconds on this puzzle to record which branches and
functions are actually hot, then recompiles using that profile. Worth about
+11.6% nodes/s here (measured against gcc on identical seeds).

## How the solver works

The engine fills cells in a fixed order and keeps, for every `(west, south)`
colour key, a precomputed table of every piece/rotation that can go there. A
**break schedule** names the fill indices at which one mismatched edge may be
spent, so a completed board has a score known in advance
(`480 - number of breaks`).

The additions that produced the 465:

* **`--exact-endgame 244:50000000`** - stop the hot depth-first search at depth 244 and finish
  the last 12 cells with a minimum-cost branch and bound (~100 nodes per call)
  that can use ring mismatches and the whole unspent budget wherever it likes.
* **`--endgame-probe 4`** - when the exact endgame fails, report *by how much*.
  A failure becomes a distance, which is what the ladder consumes.
* **`--endgame-ladder 3:236:1e9,2:230:4e9,1:224:8e9`** - escalation. A prefix
  that missed by ≤3 earns a 20-cell exact re-solve; ≤2 earns 26 cells; ≤1 earns
  32. Those are rare, so the wide searches stay affordable. **This is the stage
  that found the board**: the winning prefix reached depth 244 having spent only
  12 of its 15 breaks, the cheap 12-cell endgame could not close it, and rung 1
  did, in 1.66M nodes.
* **rotated instances** - `hints_rot90.txt` searches the same puzzle rotated 90°
  clockwise. Under a fixed corner-first fill order the rotations are genuinely
  different searches; rot90/rot270 produce 4–8× the near misses of the official
  orientation. A board found there is rotated back afterwards.
* **`--ring-breaks 236`** - from fill depth 236 on, a border cell may also
  mismatch a ring colour. Without it, 78% of deep prefixes cannot be completed
  at all, at any budget.

Every completion is independently rescored by the engine's own `verifyBoard`
before it is allowed to be saved, and again offline by `tools/to_bucas.py`.


## Previous work

`bw.cpp` is a port of [Joshua Blackwood's public solver](https://github.com/jblackwood345/EternityII_Solver)
- the engine behind the 468/469/470 unconstrained records - extended here with:

* all five clue pieces forced (single-piece tables plus neighbour-filtered tables),
* a configurable break-index schedule instead of a fixed one,
* ring-colour breaks, an exact minimum-cost endgame, and an escalation ladder
  (see below),
* a per-thread restart campaign with node caps, atomic saves, and independent
  re-validation of every board before it is written.


## Running a fresh hunt

```bash
./build.sh pgo
LADDER="3:236:1000000000,2:230:4000000000,1:224:8000000000" \
THREADS=15 CAMPAIGN_DIR=campaign_rot90 HINTS_FILE=hints_rot90.txt \
    ./run_465eg.sh
```

`run_465eg.sh` loops: draw a wall-clock seed, run the engine, restart if it dies,
stop as soon as `best_score.txt` in the campaign directory reaches `TARGET`.
Every knob is an environment variable (`THREADS`, `SOLVER`, `HINTS_FILE`,
`TARGET`, `MIN_SAVE`, `BREAKS`, `ENDGAME`, `PROBE`, `LADDER`, `RING`,
`HEUR_SCALE`, `PRUNE`, `EXTRA`, `SEED_OFFSET`). **Log the seed** — it is the only
thing that makes a find reproducible.

Boards land in the campaign directory as `board_<score>_<epoch>_t<thread>.txt`
with a `.meta` sidecar carrying the score, thread, attempt seed, breaks used and
two viewer URLs. Progress goes to `bw.log` there.

To aim at a different score, change the number of breaks: 15 releases means a
completion scores 465, 14 means 466, and so on. Each step is exponentially
harder.

### Converting a rotated find

```bash
python3 tools/rotate_board.py <board> 3 board_official.txt --hints hints.txt   # rot90 find
python3 tools/rotate_board.py <board> 1 board_official.txt --hints hints.txt   # rot270 find
python3 tools/to_bucas.py board_official.txt
```

`rotate_board.py` exits non-zero if the rotated board does not put all five
clues exactly where the official puzzle requires.

## Board file format

Row-major, 16 lines of 16 `pieceID/rotation` tokens, row 0 = top row of the
physical board. `pieceID` is 1-based and matches the line number in
`256pieces.txt`; `rotation` is the number of 90° clockwise turns from that
file's canonical orientation.

Internally `bw.cpp` works in "bu" coordinates (row 0 = bottom), matching
Blackwood's fill order, and converts on save.

## Coordinates and colour labels

`256pieces.txt` uses Blackwood's colour labels, so his heuristic sides
`{13,16,10}` and ring colours `{1,5,9,13,17}` apply unchanged; this was verified
piece-by-piece rather than assumed. The five clues in `hints.txt` were checked
against the bucas Clues board - applying each listed rotation to each canonical
piece reproduces the published placed edges under one global colour relabelling.

## Thanks

Thanks to [Daan van den Berg](https://scholar.google.com/citations?hl=en&user=0LBRFAcAAAAJ&view_op=list_works&sortby=pubdate) for the joint research on Eternity II.


## License

GPL-3.0, inherited rather than chosen: `bw.cpp` is a port of Joshua Blackwood's
[EternityII_Solver](https://github.com/jblackwood345/EternityII_Solver), which
is published under the GNU General Public License version 3, so this derived
work carries the same terms. See `LICENSE`.

The puzzle data (`256pieces.txt`, the clue files) is not code — it is the
published Eternity II piece set, as circulated in the solver community.
