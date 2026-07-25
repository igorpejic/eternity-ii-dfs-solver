#!/bin/bash
# Strict five-clue 465 hunt: reserve-schedule DFS + exact minimum-cost endgame.
#
# The engine stops the hot DFS at --exact-endgame depth and completes the
# remaining suffix with an exact branch-and-bound that may also use ring-colour
# mismatches (78% of deep prefixes cannot be completed at all without one) and
# the whole unspent budget at any depth. Completions are therefore found by the
# exact stage, and every one of them is independently rescored before it is
# banked. The schedule keeps only 12 releases below the cutoff so arrivals carry
# a real reserve: measured, tightening to 12 costs nothing in high-slack
# arrivals and buys ~22% more nodes/s versus the old 14-release shape.
#
# Watch: "cost funnel" (arrivals deep with budget unspent) and "exact endgame"
# (arrival count per slack and the cheapest exact completion cost of each).
set -u
cd "$(dirname "$0")"

DIR=${CAMPAIGN_DIR:-campaign_465eg}
THREADS=${THREADS:-30}
SOLVER=${SOLVER:-./bw_eg}
HINTS_FILE=${HINTS_FILE:-hints.txt}
TARGET=${TARGET:-465}
MIN_SAVE=${MIN_SAVE:-465}
BREAKS=${BREAKS:-192,197,201,206,211,216,221,225,229,233,237,241,246,250,253}
ENDGAME=${ENDGAME:-244:50000000}
# Cascade: the cheap 12-cell exact endgame runs with a probe ceiling so it can
# report how close it came; a prefix that lands within GAP of the budget earns a
# 20-cell exact re-solve. Measured: 20% of near misses improve by two whole
# mismatches that way, for ~4% of nodes.
PROBE=${PROBE:-4}
# Ladder rungs: gap:rewind:nodecap. A prefix climbs to the next rung only when the
# previous exact pass left it that close to the budget, so the wide searches are
# rare. 236 = 20 free cells, 230 = 26, 224 = 32, 216 = 40 (a state within one
# mismatch of the budget is rare enough to deserve an hour of one thread).
LADDER=${LADDER:-4:236:1000000000,3:230:6000000000,2:224:8000000000,1:216:10000000000}
RING=${RING:-236}
HEUR_SCALE=${HEUR_SCALE:-0.97}
PRUNE=${PRUNE:-150000000:205}
EXTRA=${EXTRA:-}
mkdir -p "$DIR"

echo "$$" > "$DIR/campaign.pid"
MON_PID=""
BW_PID=""
cleanup() {
    if [ -n "$MON_PID" ]; then kill "$MON_PID" 2>/dev/null || true; fi
    if [ -n "$BW_PID" ]; then kill -TERM "$BW_PID" 2>/dev/null || true; fi
    rm -f "$DIR/bw.pid" "$DIR/campaign.pid"
}
trap cleanup EXIT

echo "[$(date '+%F %T')] 465 endgame-exact wrapper started (pid $$), threads=$THREADS," \
     "breaks=$BREAKS, endgame=$ENDGAME, ring=$RING"
while true; do
    SEED=$(( ($(date +%s) % 1000000000 + ${SEED_OFFSET:-0}) % 1000000000 ))
    echo "[$(date '+%F %T')] launching $SOLVER, seed $SEED (best: $(cat "$DIR/best_score.txt" 2>/dev/null || echo 464))"
    # shellcheck disable=SC2086
    "$SOLVER" 256pieces.txt "$HINTS_FILE" \
        --threads "$THREADS" --seconds 0 --seed "$SEED" \
        --node-cap 2000000000 --min-save "$MIN_SAVE" --save-dir "$DIR" \
        --prune "$PRUNE" --heur-scale "$HEUR_SCALE" --double-breaks \
        --ring-breaks "$RING" --exact-endgame "$ENDGAME" \
        --endgame-probe "$PROBE" --endgame-ladder "$LADDER" \
        $EXTRA --breaks "$BREAKS" &
    BW_PID=$!
    echo "$BW_PID" > "$DIR/bw.pid"

    # Stop the campaign the moment the engine has banked a verified target board.
    (
        while kill -0 "$BW_PID" 2>/dev/null; do
            BEST=$(cat "$DIR/best_score.txt" 2>/dev/null || echo 0)
            if [ "$BEST" -ge "$TARGET" ]; then
                echo "[$(date '+%F %T')] TARGET REACHED: $BEST/480; stopping finder"
                kill -TERM "$BW_PID" 2>/dev/null || true
                exit 0
            fi
            sleep 5
        done
    ) &
    MON_PID=$!
    wait "$BW_PID"
    RC=$?
    kill "$MON_PID" 2>/dev/null || true
    wait "$MON_PID" 2>/dev/null || true
    MON_PID=""
    if [ "$RC" -eq 0 ] || [ "$RC" -eq 130 ] || [ "$RC" -eq 143 ]; then
        echo "[$(date '+%F %T')] clean exit (rc=$RC)"
        break
    fi
    echo "[$(date '+%F %T')] finder died (rc=$RC); restarting in 5s"
    sleep 5
done
