#!/bin/bash
# Replay the exact run that produced the 465/480 five-clue board and check the
# result byte-for-byte against results_465/board_465_rot90_raw.txt.
#
# The seed below is the one the original campaign drew from the wall clock and
# logged; thread 3 finds the board on its very first attempt (~2 s on an idle
# 9950X, longer on a busy machine). Everything after that is verification.
#
#   ./reproduce_465.sh [solver] [outdir]
#
# Defaults: solver ./bw (or ./bw_eg if present), outdir ./replay.
set -eu
cd "$(dirname "$0")"

SOLVER=${1:-}
if [ -z "$SOLVER" ]; then
    if [ -x ./bw_eg ]; then SOLVER=./bw_eg; else SOLVER=./bw; fi
fi
OUT=${2:-replay}
TIMEOUT=${TIMEOUT:-900}

[ -x "$SOLVER" ] || { echo "no solver at $SOLVER — run ./build.sh first" >&2; exit 2; }

rm -rf "$OUT"
mkdir -p "$OUT"

echo "== searching (seed 784944885, 15 threads, solver $SOLVER)"
"$SOLVER" 256pieces.txt hints_rot90.txt \
    --threads 15 --seconds 0 --seed 784944885 \
    --node-cap 2000000000 --min-save 465 --save-dir "$OUT" \
    --prune 150000000:205 --heur-scale 0.97 --double-breaks \
    --ring-breaks 236 --exact-endgame 244:50000000 --endgame-probe 4 \
    --endgame-ladder 3:236:1000000000,2:230:4000000000,1:224:8000000000 \
    --breaks 192,197,201,206,211,216,221,225,229,233,237,241,246,250,253 &
BW=$!

# Stop the engine the moment it banks a 465; give up after TIMEOUT seconds.
WAITED=0
while kill -0 "$BW" 2>/dev/null; do
    if [ "$(cat "$OUT/best_score.txt" 2>/dev/null || echo 0)" -ge 465 ]; then
        kill -TERM "$BW" 2>/dev/null || true
        break
    fi
    if [ "$WAITED" -ge "$TIMEOUT" ]; then
        echo "!! no 465 after ${TIMEOUT}s — killing" >&2
        kill -TERM "$BW" 2>/dev/null || true
        wait "$BW" 2>/dev/null || true
        exit 1
    fi
    sleep 2
    WAITED=$((WAITED + 2))
done
wait "$BW" 2>/dev/null || true

FOUND=$(ls "$OUT"/board_4*_*_t*.txt 2>/dev/null | head -1 || true)
[ -n "$FOUND" ] || { echo "!! engine exited without saving a 465 board" >&2; exit 1; }
echo "== engine saved $FOUND"
grep -E 'COMPLETION: 465' "$OUT/bw.log" || true

echo
echo "== raw board vs recorded board_465_rot90_raw.txt"
if diff -q "$FOUND" results_465/board_465_rot90_raw.txt >/dev/null; then
    echo "   IDENTICAL"
else
    echo "   DIFFERS — replay did not reproduce the recorded board" >&2
    diff "$FOUND" results_465/board_465_rot90_raw.txt | head -20
    exit 1
fi

echo
echo "== rotating back to official coordinates (3 quarter-turns)"
python3 tools/rotate_board.py "$FOUND" 3 "$OUT/board_465_official.txt" --hints hints.txt
if diff -q "$OUT/board_465_official.txt" results_465/board_465_official.txt >/dev/null; then
    echo "   IDENTICAL to results_465/board_465_official.txt (all five clues in place)"
else
    echo "   DIFFERS from results_465/board_465_official.txt" >&2
    exit 1
fi

echo
echo "== independent rescore"
python3 tools/to_bucas.py "$OUT/board_465_official.txt"

echo
echo "== REPRODUCED: 465/480 with all five official clues"
