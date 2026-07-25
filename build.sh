#!/bin/bash
# Build the solver.
#
#   ./build.sh          plain -O3 build            -> ./bw
#   ./build.sh pgo      clang PGO build            -> ./bw_eg   (~+11.6% nodes/s)
#   ./build.sh gcc      plain -O3 build with g++   -> ./bw
#
# PGO = profile-guided optimization: compile an instrumented binary, run it on a
# representative workload to record which branches and functions are hot, then
# recompile against that profile.
#
# Both binaries search identically: the attempt seed fully determines the
# result, so a board found by one is reproducible by the other. PGO only buys
# throughput.
set -eu
cd "$(dirname "$0")"

MODE=${1:-plain}
FLAGS="-O3 -march=native -std=c++17 -pthread"

case "$MODE" in
plain)
    clang++ $FLAGS bw.cpp -o bw
    echo "built ./bw (clang -O3)"
    ;;
gcc)
    g++ $FLAGS bw.cpp -o bw
    echo "built ./bw (g++ -O3)"
    ;;
pgo)
    WORK=$(mktemp -d)
    trap 'rm -rf "$WORK"' EXIT
    echo "[1/3] instrumented build"
    clang++ $FLAGS -fprofile-instr-generate bw.cpp -o "$WORK/inst"
    echo "[2/3] profile run (90s, 2 threads)"
    LLVM_PROFILE_FILE="$WORK/p_%p.profraw" "$WORK/inst" 256pieces.txt hints_rot90.txt \
        --threads 2 --seconds 90 --seed 5150 --node-cap 2000000000 --min-save 999 \
        --save-dir "$WORK/train" --prune 150000000:205 --heur-scale 0.97 --double-breaks \
        --ring-breaks 236 --exact-endgame 244:50000000 --endgame-probe 4 \
        --endgame-ladder 4:236:1000000000,3:230:6000000000,2:224:8000000000 \
        --breaks 192,197,201,206,211,216,221,225,229,233,237,241,246,250,253
    llvm-profdata merge -output="$WORK/bw.profdata" "$WORK"/p_*.profraw
    echo "[3/3] optimised build"
    clang++ $FLAGS -fprofile-instr-use="$WORK/bw.profdata" bw.cpp -o bw_eg
    echo "built ./bw_eg (clang PGO)"
    ;;
*)
    echo "usage: $0 [plain|gcc|pgo]" >&2
    exit 2
    ;;
esac
