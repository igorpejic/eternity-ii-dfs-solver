// Blackwood-style break-index backtracker for the 5-clue Eternity II record hunt.
//
// Copyright (C) 2026 Igor Pejic
//
// Port of Joshua Blackwood's public solver (github.com/jblackwood345/EternityII_Solver,
// the engine behind the 468/469/470 records), which is GPL-3.0. Modified in 2026,
// extended with:
//   * all 5 clue pieces forced (single-piece tables + neighbor-filtered tables)
//   * configurable break indices (default 19 breaks -> completed board = 461 = new clue record)
//   * per-thread restart campaign with node cap, heartbeat stats, atomic saves,
//     independent board re-validation before any save, signal-safe shutdown.
//
// Internal coordinates are "bu" (row 0 = bottom row of the physical board), matching
// Blackwood's fill order. Our project files (256pieces.txt, hints.txt, saved boards)
// are "td" (row 0 = top). bu_row = 15 - td_row. Piece rotations are physical
// (# of 90-degree CW turns) and identical in both frames.
//
// Verified 2026-07-04 (derive_mapping.py): our 256pieces.txt uses Blackwood's exact
// color labels, so heuristic_sides {13,16,10} and ring colors {1,5,9,13,17} apply as-is.
//
// Build:  g++ -O3 -march=native -std=c++17 -pthread bw.cpp -o bw
// Run:    ./bw ../256pieces.txt ../hints.txt --threads 32 --seconds 0 --seed 1

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <csignal>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <climits>
#include <mutex>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

// ---------------------------------------------------------------- basic types

struct Piece {          // physical sides, clockwise cycle from top
    uint8_t n, e, s, w; // north, east, south, west in canonical (file) orientation
};

struct RotatedPiece {   // one placement candidate, 8 bytes
    uint16_t pieceNumber;   // 1..256, 0 = empty
    uint8_t rot;            // CW quarter turns from canonical
    uint8_t top;            // color facing north after rotation
    uint8_t right;          // color facing east after rotation
    uint8_t breakCount;     // 0 or 1 mismatches vs the (west,south) key
    uint8_t hsc;            // heuristic side count of the piece (rotation-invariant)
    uint8_t pad;
};
static_assert(sizeof(RotatedPiece) == 8, "RotatedPiece must be 8 bytes");

struct SortEntry { RotatedPiece rp; int key; int32_t sortKey; };

// A candidate table: for key = west*23 + south, a slice of candidates.
struct Table {
    std::vector<RotatedPiece> pool;   // sorted per restart, slices contiguous per key
    uint32_t off[530] = {0};          // off[k]..off[k+1] is the slice for key k
    std::vector<SortEntry> base;      // unsorted master copy (rebuilt slices from this)
};

static constexpr int NCOLORS = 23;
// Exhaustion-schedule colors. Blackwood's hand-tuned default {13,16,10};
// overridable with --heur-colors "13,16,10,11,12" (learned from pool telemetry).
// The ramp auto-scales by the chosen set's total edge count so the default
// reproduces the original array exactly.
static std::vector<int> HEUR_COLORS = {13, 16, 10};
static double HEUR_EDGE_TOTAL = 150.0;   // recomputed after pieces load
static constexpr bool IS_RING_COLOR[NCOLORS] = {
    false, true, false, false, false, true, false, false, false, true,
    false, false, false, true,  false, false, false, true, false, false,
    false, false, false}; // {1,5,9,13,17}

// Two fill orders, selected at startup by clue mode (zero hot-loop cost: the DFS
// only ever reads the derived SEQ_ROW/SEQ_COL/FILL_IDX arrays).
//
// CLUED (5 hints): Blackwood's order with one change — the 3x3 corner containing
// clue 181 at bu(2,2) is filled FIRST (indices 0-8), then rows 0-2 sweep
// left-to-right. Raster order put the clue corner's dead-ends behind 13+ cells of
// row-0 tail that are independent of the corner, so DFS re-solved the same dead
// corner for millions of irrelevant tail configs (measured: never passed depth 20).
// Clue fill indices: 181@8, 249@45, 139@119, 208@188, 255@247.
//
// RASTER (center clue only): Blackwood's original, unmodified — the configuration
// behind every 468-470 record. No corner clues, so no corner-first fix needed.
static const int BOARD_ORDER_CLUED[16][16] = {
    {196, 197, 198, 199, 200, 205, 210, 215, 220, 225, 230, 235, 243, 249, 254, 255},
    {191, 192, 193, 194, 195, 204, 209, 214, 219, 224, 229, 234, 242, 248, 252, 253},
    {186, 187, 188, 189, 190, 203, 208, 213, 218, 223, 228, 233, 241, 247, 250, 251},
    {181, 182, 183, 184, 185, 202, 207, 212, 217, 222, 227, 232, 240, 244, 245, 246},
    {176, 177, 178, 179, 180, 201, 206, 211, 216, 221, 226, 231, 236, 237, 238, 239},
    {160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175},
    {144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159},
    {128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143},
    {112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127},
    { 96,  97,  98,  99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111},
    { 80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95},
    { 64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79},
    { 48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63},
    {  6,   7,   8,  35,  36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47},
    {  3,   4,   5,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34},
    {  0,   1,   2,   9,  10,  11,  12,  13,  14,  15,  16,  17,  18,  19,  20,  21}};

static const int BOARD_ORDER_RASTER[16][16] = {
    {196, 197, 198, 199, 200, 205, 210, 215, 220, 225, 230, 235, 243, 249, 254, 255},
    {191, 192, 193, 194, 195, 204, 209, 214, 219, 224, 229, 234, 242, 248, 252, 253},
    {186, 187, 188, 189, 190, 203, 208, 213, 218, 223, 228, 233, 241, 247, 250, 251},
    {181, 182, 183, 184, 185, 202, 207, 212, 217, 222, 227, 232, 240, 244, 245, 246},
    {176, 177, 178, 179, 180, 201, 206, 211, 216, 221, 226, 231, 236, 237, 238, 239},
    {160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175},
    {144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159},
    {128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143},
    {112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127},
    { 96,  97,  98,  99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111},
    { 80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95},
    { 64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79},
    { 48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63},
    { 32,  33,  34,  35,  36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47},
    { 16,  17,  18,  19,  20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31},
    {  0,   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12,  13,  14,  15}};

// Chosen at startup: CLUED when any non-center hint exists, else RASTER.
static const int (*BOARD_ORDER)[16] = BOARD_ORDER_CLUED;

// Default budget release schedules per mode (both overridable with --breaks).
// Clued: ours from the 461-463 hunt. Center-only: Blackwood's canonical 470-run
// set — for a realistic ladder start at 466/467, pass a 13-14 slot schedule.
static const char* DEFAULT_BREAKS =
    "177,182,187,192,197,201,206,211,216,221,225,229,233,237,239,241,244,250,253";
static const char* DEFAULT_BREAKS_CENTER_ONLY =
    "201,206,211,216,221,225,229,233,237,239";

// -------------------------------------------------------------------- globals

struct Config {
    std::string piecesFile, hintsFile;
    int threads = 32;
    long seconds = 0;              // 0 = run until signal
    uint64_t seed = 1;
    uint64_t nodeCap = 50000000000ULL;
    double heurScale = 1.0;
    int minSave = 458;             // save completed boards scoring >= this
    std::string saveDir = "campaign_bw";
    uint64_t pruneNodes = 0;       // 0 = prune-back off
    int pruneDepth = 0;
    bool bench = false;
    bool stats = false;            // per-depth node counters (debug)
    std::string breaks = DEFAULT_BREAKS;
    bool breaksSet = false;        // --breaks given (else mode default applies)
    bool heurSet = false;          // --heur-scale given (else mode default applies)
    std::string poolLog;           // telemetry: leftover-pool mask at depth 176 +
                                   // final attempt depth, one line per deep attempt
    std::string heurColors;        // --heur-colors "13,16,10,11,12" (learned set)
    bool sdPrune = false;          // --sd-prune: color supply/demand necessary
                                   // condition (piece-theft early detection)
    bool isolatedBreaks = false;   // --isolated-breaks: Blackwood's discipline —
                                   // no two broken seams may share a cell
    int cellBreaks = 1;            // --double-breaks: allow 2 mismatches at one
                                   // placement (REPLAY class expansion)
    std::string fillOrder;         // --fill-order raster|colband|comb (center-only;
                                   // GAUNTLET: scan order unlocks disjoint
                                   // reachable-completion slices)
    int tailDepth = 0;             // --tail-mrv DEPTH:NODES: once per attempt,
    uint64_t tailNodes = 0;        // solve the remaining cells with exact MRV
    int tailTargetScore = -1;      // --tail-target-score SCORE: exact-tail target;
                                   // default inherits the generator's final budget
    uint64_t tailPeriod = 0;       // --tail-period N: sample another prefix
                                   // after N additional hot DFS nodes
    bool tailAdaptive = false;     // --tail-adaptive: progressively rewind a
                                   // wider suffix after depth 248/250/252/253
    int ringDepth = 0;             // --ring-breaks DEPTH: border cells filled at
                                   // or after DEPTH may mismatch a ring colour
                                   // (0 = Blackwood's ban, the historical default)
    int endgameDepth = 0;          // --exact-endgame DEPTH:NODES: at EVERY prefix
    uint64_t endgameNodes = 0;     // reaching DEPTH, replace the hot DFS's search
                                   // of the remaining cells with an exact
                                   // minimum-cost completion, and backtrack at
                                   // once when it exhausts. Affordable because
                                   // only ~22k prefixes per 80B nodes reach 244,
                                   // and it buys ring-colour mismatches, the full
                                   // remaining budget at any depth, and MRV order.
    uint64_t yieldBase = 0;        // --yield-budget BASE:PER — spend nodes on an
    uint64_t yieldPer = 0;         // attempt in proportion to what it produces:
                                   // the attempt starts with BASE nodes and earns
                                   // PER more for every exact-endgame arrival it
                                   // reaches. Measured: 40% of attempts reach the
                                   // cutoff zero times while the top few supply
                                   // ~40% of all arrivals, so a flat node cap
                                   // spends most of its time on dead bottoms.
    int escGap = 0;                // --endgame-escalate GAP:REWIND:NODES — when the
    int escRewind = 0;             // cheap 12-cell exact endgame lands within GAP of
    uint64_t escNodes = 0;         // the budget, re-solve a much wider suffix from
                                   // REWIND exactly. Near misses are rare, so the
                                   // wide search is nearly free, and it is applied
                                   // to a population already known to be good —
                                   // unlike simply lowering the cutoff, which
                                   // spends the wide search on everything and
                                   // measured strictly worse.
    // --endgame-ladder "G1:R1:N1,G2:R2:N2,..." — successive exact re-solves of an
    // ever wider suffix. A prefix enters rung k only when rung k-1 left its
    // cheapest completion within Gk of the budget, so each more expensive search
    // runs on a population the cheaper one already proved good, and the rungs stay
    // rare. Rewinds must strictly deepen. --endgame-escalate/2 append one rung
    // each and remain valid spellings.
    struct EscRung { int gap; int rewind; uint64_t nodes; };
    std::vector<EscRung> ladder;
    int endgameSample = 0;         // --endgame-sample N: apply the wide probe to
                                   // every Nth call only, so the cost
                                   // distribution can be measured without paying
                                   // for it on every arrival (the probe changes
                                   // per-call cost by orders of magnitude at
                                   // shallow cutoffs, which distorts exactly the
                                   // comparison it is meant to inform).
    int endgameProbeExtra = 0;     // --endgame-probe EXTRA: widen the exact search
                                   // to budget+EXTRA so the telemetry shows how
                                   // far the endgame misses (measurement mode).
    int tailMinSlack = 0;          // --tail-min-slack S: only spend exact-endgame
                                   // work on states with >= S budget unspent.
                                   // Measured: the endgame of a banked depth-250
                                   // prefix needs 4-10 further mismatches, so
                                   // zero-slack states are hopeless by ~5.
};

static Config cfg;
static Piece PIECES[257];                 // 1-based
static uint8_t PIECE_HSC[257];
static int PIECE_CLASS[257];              // 0 middle, 1 side, 2 corner
static bool IS_CLUE[257];
struct Hint { int piece, tdRow, tdCol, rot; };
static std::vector<Hint> HINTS;

// Hot solver tables packed into one cache-aligned block so code/data layout
// changes elsewhere can't shift them across cache lines (a 1KB rodata addition
// once cost a measurable ~3% nodes/s purely from layout drift).
struct alignas(64) HotTables {
    uint8_t seqRow[256];   // bu row per solve index
    uint8_t seqCol[256];   // bu col per solve index
    uint8_t breakArray[256];
    int heurArray[256];
};
static HotTables HOT;
#define SEQ_ROW HOT.seqRow
#define SEQ_COL HOT.seqCol
#define BREAK_ARRAY HOT.breakArray
#define HEUR_ARRAY HOT.heurArray
static int FILL_IDX[256];                  // bu cellIdx -> solve index
static int FIRST_BREAK_IDX = 256;
static constexpr int MAX_HEUR_IDX = 160;
// Cost-funnel depth thresholds, and the total mismatch budget of the schedule.
static constexpr int FUNNEL_TH[4] = {244, 248, 250, 252};
static int TOTAL_BUDGET = 0;               // BREAK_ARRAY[255], or the tail target

static std::atomic<bool> g_stop{false};
static std::atomic<int> g_bestScore{0};
static std::mutex g_saveMutex;
static FILE* g_log = nullptr;

struct alignas(64) ThreadStats {
    std::atomic<uint64_t> nodes{0}, placements{0}, restarts{0}, completions{0};
    std::atomic<uint64_t> tailCalls{0}, tailNodes{0};
    std::atomic<uint64_t> adaptiveCalls[4] = {};
    std::atomic<uint64_t> adaptiveNodes[4] = {};
    std::atomic<uint64_t> adaptiveCapped[4] = {};
    std::atomic<int> maxDepthEver{0};
    std::atomic<uint64_t> depthHist[257] = {}; // attempt-end max depth histogram
    // Cost funnel: [threshold][slack] = attempts that reached depth >= FUNNEL_TH[k]
    // while still holding >= slack budget in reserve. Raw depth is a mirage —
    // every banked depth-250 prefix had spent its whole budget and still needed
    // 4-10 more mismatches. Reaching depth 248 with 3 unspent is the event that
    // actually precedes a completion; this is the A/B fitness signal.
    std::atomic<uint64_t> costFunnel[4][6] = {};
    // exact-endgame accounting. endgameGrid[slack][cost] counts calls that
    // arrived with `slack` budget unspent and whose cheapest exact filling of the
    // suffix cost `cost` further mismatches (cost 15 = none under the ceiling).
    // This joint distribution is what decides the schedule: a completion needs
    // cost <= slack, and P(cost <= s) rises steeply in s, so it prices "one more
    // unit of reserve" against the arrival rate that buying it costs.
    std::atomic<uint64_t> endgameCalls{0}, endgameCapped{0}, endgameNodes{0};
    std::atomic<uint64_t> endgameGrid[8][16] = {};
    // attemptYield[b] = attempts whose useful-arrival count fell in bucket b
    // (0, 1, 2-3, 4-7, ... 4096+). If yield is heavy-tailed then bottom halves
    // differ in kind, not luck, and re-exploring a good one beats resampling.
    std::atomic<uint64_t> attemptYield[14] = {};
    // ring-colour share of the cheapest completion each endgame call found
    std::atomic<uint64_t> endgameRing[16] = {};
    // escalated (wide-rewind) results: [gap] where gap = cost - budget, 15 = none
    static constexpr int MAX_RUNGS = 4;
    std::atomic<uint64_t> escCalls[MAX_RUNGS] = {}, escCapped[MAX_RUNGS] = {};
    std::atomic<uint64_t> escNodes[MAX_RUNGS] = {};
    std::atomic<uint64_t> escGapHist[MAX_RUNGS][16] = {};
    uint64_t depthNodes[257] = {};             // nodes spent at each depth (--stats)
    uint64_t depthFits[257] = {};              // perfect placements per depth (--stats)
    uint64_t depthHalf[257] = {};              // break placements per depth (--stats)
};
static std::vector<ThreadStats> g_stats;

// --------------------------------------------------------------------- logging

static void logf(const char* fmt, ...) {
    char buf[2048];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    time_t t = time(nullptr);
    struct tm tmv;
    localtime_r(&t, &tmv);
    char ts[32];
    strftime(ts, sizeof ts, "%Y-%m-%d %H:%M:%S", &tmv);
    std::lock_guard<std::mutex> lk(g_saveMutex);
    printf("[%s] %s\n", ts, buf);
    fflush(stdout);
    if (g_log) { fprintf(g_log, "[%s] %s\n", ts, buf); fflush(g_log); }
}

[[noreturn]] static void die(const char* fmt, ...) {
    char buf[2048];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    fprintf(stderr, "FATAL: %s\n", buf);
    exit(1);
}

// ------------------------------------------------------------------------ rng

struct Rng { // xoshiro256++, split-seeded
    uint64_t s[4];
    explicit Rng(uint64_t seed) {
        uint64_t x = seed;
        for (auto& v : s) { // splitmix64
            x += 0x9e3779b97f4a7c15ULL;
            uint64_t z = x;
            z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
            z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
            v = z ^ (z >> 31);
        }
    }
    static uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
    uint64_t next() {
        uint64_t r = rotl(s[0] + s[3], 23) + s[0];
        uint64_t t = s[1] << 17;
        s[2] ^= s[0]; s[3] ^= s[1]; s[1] ^= s[2]; s[0] ^= s[3]; s[2] ^= t;
        s[3] = rotl(s[3], 45);
        return r;
    }
    uint32_t below(uint32_t n) { return (uint32_t)(next() % n); }
};

// ------------------------------------------------------------------- geometry

static inline uint8_t sideAfterRot(const Piece& p, int rot, int face) {
    // face: 0=N 1=E 2=S 3=W; rotated cycle: out[i] = canon[(i - rot) mod 4]
    const uint8_t c[4] = {p.n, p.e, p.s, p.w};
    return c[((face - rot) % 4 + 4) % 4];
}

// ------------------------------------------------------------- input handling

static void loadPieces(const std::string& path) {
    FILE* f = fopen(path.c_str(), "r");
    if (!f) die("cannot open pieces file %s", path.c_str());
    int t, b, l, r, cnt = 0;
    while (fscanf(f, "%d %d %d %d", &t, &b, &l, &r) == 4) {
        if (cnt >= 256) die("more than 256 pieces");
        ++cnt;
        PIECES[cnt] = {(uint8_t)t, (uint8_t)r, (uint8_t)b, (uint8_t)l}; // file: T B L R
    }
    fclose(f);
    if (cnt != 256) die("expected 256 pieces, got %d", cnt);

    int nc = 0, ns = 0, nm = 0;
    for (int i = 1; i <= 256; i++) {
        const Piece& p = PIECES[i];
        int greys = (p.n == 0) + (p.e == 0) + (p.s == 0) + (p.w == 0);
        PIECE_CLASS[i] = greys == 2 ? 2 : (greys == 1 ? 1 : 0);
        (greys == 2 ? nc : greys == 1 ? ns : nm)++;
    }
    if (nc != 4 || ns != 56 || nm != 196)
        die("piece classes wrong: %d corners %d sides %d middles", nc, ns, nm);
}

// Compute per-piece heuristic side counts for the active color set (called after
// arg parsing so --heur-colors is honored). Also fixes the ramp scale base.
static void computeHsc() {
    // Blackwood's own set {13,16,10} totals 122 edges (13 is a ring color with
    // only 24) — the ramp scale is relative to THAT, so the default reproduces
    // his array exactly (scale 1.0).
    double blackwoodTotal = 0, total = 0;
    for (int i = 1; i <= 256; i++) {
        const Piece& p = PIECES[i];
        uint8_t h = 0;
        for (int hs : HEUR_COLORS)
            h += (p.n == hs) + (p.e == hs) + (p.s == hs) + (p.w == hs);
        PIECE_HSC[i] = h;
        total += h;
        for (int hs : {13, 16, 10})
            blackwoodTotal += (p.n == hs) + (p.e == hs) + (p.s == hs) + (p.w == hs);
    }
    HEUR_EDGE_TOTAL = 150.0 * total / blackwoodTotal;   // buildHeurArray divides by 150
    std::string cs;
    for (int c : HEUR_COLORS) cs += (cs.empty() ? "" : ",") + std::to_string(c);
    logf("heuristic colors: {%s}, total edges %.0f (ramp scale x%.3f)",
         cs.c_str(), total, total / blackwoodTotal);
}

static void loadHints(const std::string& path) {
    FILE* f = fopen(path.c_str(), "r");
    if (!f) die("cannot open hints file %s", path.c_str());
    char line[256];
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        Hint h;
        if (sscanf(line, "%d %d %d %d", &h.piece, &h.tdRow, &h.tdCol, &h.rot) == 4)
            HINTS.push_back(h);
    }
    fclose(f);
    if (HINTS.empty() || HINTS.size() > 5)
        die("expected 1-5 hints, got %zu", HINTS.size());
    bool haveCenter = false;
    for (const Hint& h : HINTS) {
        if (PIECE_CLASS[h.piece] != 0) die("clue piece %d is not interior", h.piece);
        if (IS_CLUE[h.piece]) die("duplicate hint for piece %d", h.piece);
        IS_CLUE[h.piece] = true;
        if (h.tdRow == 8 && h.tdCol == 7) haveCenter = true;
    }
    // The one-clue record-line mode is defined by the official centre clue.
    // Full-clue runs may rotate the entire instance to search the same puzzle
    // from another corner; on an even board that moves the centre clue among
    // the four central cells, while all five clues remain forced.
    if (HINTS.size() == 1 && !haveCenter)
        die("no hint at td(8,7): the mandatory center piece must always be fixed "
            "(all record-line configurations keep it)");
}

// Select fill order + config defaults by clue mode. Startup-only; the DFS reads
// the derived SEQ/FILL arrays, so this has zero effect on placements/sec.
static int ORDER_CUSTOM[16][16];

// Alternative center-only fill orders (GAUNTLET portfolio). Both keep the
// west-and-south-first invariant the DFS requires; the startup validator checks.
static void buildCustomOrder(const std::string& name) {
    auto set = [](int fi, int buRow, int col) { ORDER_CUSTOM[15 - buRow][col] = fi; };
    if (name == "colband") {          // columns left->right, each bottom->top
        for (int fi = 0; fi < 256; fi++) set(fi, fi & 15, fi >> 4);
    } else if (name == "comb") {      // bottom 8 rows raster, then 8-cell teeth
        for (int fi = 0; fi < 128; fi++) set(fi, fi >> 4, fi & 15);
        for (int fi = 128; fi < 256; fi++)
            set(fi, 8 + ((fi - 128) & 7), (fi - 128) >> 3);
    } else {
        die("unknown --fill-order %s (raster|colband|comb)", name.c_str());
    }
    BOARD_ORDER = ORDER_CUSTOM;
}

static void applyClueMode() {
    bool centerOnly = HINTS.size() == 1;
    BOARD_ORDER = centerOnly ? BOARD_ORDER_RASTER : BOARD_ORDER_CLUED;
    if (!cfg.fillOrder.empty() && cfg.fillOrder != "raster") {
        if (!centerOnly) die("--fill-order variants are center-only for now");
        buildCustomOrder(cfg.fillOrder);
        logf("fill order: %s (GAUNTLET portfolio)", cfg.fillOrder.c_str());
    }
    if (!cfg.breaksSet && centerOnly) cfg.breaks = DEFAULT_BREAKS_CENTER_ONLY;
    if (!cfg.heurSet) cfg.heurScale = centerOnly ? 1.0 : 0.97;
    logf("clue mode: %s (%zu hints) -> %s fill order, heurScale %.3f%s",
         centerOnly ? "CENTER-ONLY" : "FULL-CLUES", HINTS.size(),
         centerOnly ? "raster" : "corner-first", cfg.heurScale,
         cfg.heurSet ? " (explicit)" : " (mode default)");
}

// ------------------------------------------------------- candidate enumeration

// Enumerate placement candidates of one piece into `base`.
//   rotFilter   : -1 any rotation, else exactly that rotation
//   maxBr       : max mismatches per placement (0 exact, 1 single, 2 double —
//                 REPLAY finding: community 460-class boards need double-break
//                 cells that single-break engines cannot represent)
//   allowRing   : also emit mismatches on a ring colour. Ring colours occur only
//                 on the lateral edges of border pieces, so Blackwood's ban keeps
//                 the 60-seam frame cycle perfect; measured on 300 banked
//                 depth-250 prefixes, 78% of them cannot be completed AT ALL
//                 under that ban, at any budget. Ring mismatches must come in
//                 pairs (each ring colour has exactly 24 edges, so a single
//                 mismatch leaves an odd count that cannot pair up), hence they
//                 cost at least 2 of the budget and belong in the reserve.
//   reqTop/reqRight : -1 none, else rotated piece must show this color N / E
static void enumeratePiece(std::vector<SortEntry>& base, int pieceNo,
                           int rotFilter, int maxBr, bool allowRing = false,
                           int reqTop = -1, int reqRight = -1) {
    const Piece& p = PIECES[pieceNo];
    for (int rot = 0; rot < 4; rot++) {
        if (rotFilter >= 0 && rot != rotFilter) continue;
        uint8_t nn = sideAfterRot(p, rot, 0), ee = sideAfterRot(p, rot, 1),
                ss = sideAfterRot(p, rot, 2), ww = sideAfterRot(p, rot, 3);
        if (reqTop >= 0 && nn != reqTop) continue;
        if (reqRight >= 0 && ee != reqRight) continue;
        for (int left = 0; left < NCOLORS; left++) {
            for (int bottom = 0; bottom < NCOLORS; bottom++) {
                int breaks = 0;
                bool ringBreak = false;
                if (ww != left) { breaks++; ringBreak |= IS_RING_COLOR[ww]; }
                if (ss != bottom) { breaks++; ringBreak |= IS_RING_COLOR[ss]; }
                if (breaks > maxBr || (ringBreak && !allowRing)) continue;
                SortEntry e;
                e.rp = {(uint16_t)pieceNo, (uint8_t)rot, nn, ee,
                        (uint8_t)breaks, PIECE_HSC[pieceNo], 0};
                e.key = left * NCOLORS + bottom;
                e.sortKey = 0;
                base.push_back(e);
            }
        }
    }
}

// Tables owned per thread (sorted with fresh jitter each restart).
struct Tables {
    Table corners, leftSides, bottomSides, topSides;
    Table rightNoBrk, rightBrk, midNoBrk, midBrk;
    // --ring-breaks variants: same piece classes, but a lateral (ring-colour)
    // seam may mismatch. Only cells whose fill index is >= cfg.ringDepth point
    // at these, so the hot loop pays nothing for the wider space — the cell ->
    // table map is fixed at startup exactly like the late/early break split.
    Table cornersRing, leftSidesRing, bottomSidesRing, topSidesRing, rightBrkRing;
    std::vector<Table> special;          // clue + neighbor-filtered tables
    const Table* lookup[256] = {nullptr}; // bu cellIdx -> table
};

static void finalizeTable(Table& t) {
    std::stable_sort(t.base.begin(), t.base.end(),
                     [](const SortEntry& a, const SortEntry& b) { return a.key < b.key; });
    uint32_t cnt[530] = {0};
    for (const auto& e : t.base) cnt[e.key + 1]++;
    for (int k = 1; k < 530; k++) cnt[k] += cnt[k - 1];
    memcpy(t.off, cnt, sizeof t.off);
    t.pool.resize(t.base.size());
}

static void sortTableForRestart(Table& t, Rng& rng) {
    for (auto& e : t.base)
        e.sortKey = -100000 * (int)e.rp.breakCount + 100 * (int)e.rp.hsc +
                    (int)rng.below(100);
    // per-key slices: stable partition by key already; sort each slice by sortKey desc
    size_t n = t.base.size();
    for (int k = 0; k < 529; k++) {
        uint32_t a = t.off[k], b = t.off[k + 1];
        if (b - a > 1)
            std::sort(t.base.begin() + a, t.base.begin() + b,
                      [](const SortEntry& x, const SortEntry& y) {
                          return x.sortKey > y.sortKey;
                      });
    }
    for (size_t i = 0; i < n; i++) t.pool[i] = t.base[i].rp;
}

// The bu row0 table gets Blackwood's special first-row ordering.
static void sortBottomRowForRestart(Table& t, Rng& rng) {
    for (auto& e : t.base)
        e.sortKey = (e.rp.hsc > 0 ? 100 : 0) + (int)rng.below(100);
    for (int k = 0; k < 529; k++) {
        uint32_t a = t.off[k], b = t.off[k + 1];
        if (b - a > 1)
            std::sort(t.base.begin() + a, t.base.begin() + b,
                      [](const SortEntry& x, const SortEntry& y) {
                          return x.sortKey > y.sortKey;
                      });
    }
    for (size_t i = 0; i < t.base.size(); i++) t.pool[i] = t.base[i].rp;
}

static void buildTables(Tables& T) {
    // generic pools ------------------------------------------------------
    for (int i = 1; i <= 256; i++) {
        switch (PIECE_CLASS[i]) {
            case 2: enumeratePiece(T.corners.base, i, -1, 0); break;
            case 1: {
                const Piece& p = PIECES[i];
                // canonical side piece has grey south; rot k puts grey at S,W,N,E
                for (int rot = 0; rot < 4; rot++) {
                    if (sideAfterRot(p, rot, 2) == 0)       // grey south -> bottom row
                        enumeratePiece(T.bottomSides.base, i, rot, 0);
                    if (sideAfterRot(p, rot, 3) == 0)       // grey west -> left col
                        enumeratePiece(T.leftSides.base, i, rot, 0);
                    if (sideAfterRot(p, rot, 0) == 0)       // grey north -> top row
                        enumeratePiece(T.topSides.base, i, rot, cfg.cellBreaks);
                    if (sideAfterRot(p, rot, 1) == 0) {     // grey east -> right col
                        enumeratePiece(T.rightNoBrk.base, i, rot, 0);
                        enumeratePiece(T.rightBrk.base, i, rot, cfg.cellBreaks);
                    }
                }
                break;
            }
            case 0:
                if (!IS_CLUE[i]) {
                    enumeratePiece(T.midNoBrk.base, i, -1, 0);
                    enumeratePiece(T.midBrk.base, i, -1, cfg.cellBreaks);
                }
                break;
        }
    }
    // ring-widened border pools (only built when the feature is on) ------
    // For a border cell the two seams closed at placement time are one frame
    // side (always grey, always matched) and one lateral ring seam, so under the
    // ban these tables hold no break candidates at all for their ring seam:
    // corners/leftSides/bottomSides are strictly exact today, and topSides /
    // rightBrk can only break on their single interior-facing seam.
    if (cfg.ringDepth) {
        for (int i = 1; i <= 256; i++) {
            const Piece& p = PIECES[i];
            switch (PIECE_CLASS[i]) {
                case 2: enumeratePiece(T.cornersRing.base, i, -1, cfg.cellBreaks, true); break;
                case 1:
                    for (int rot = 0; rot < 4; rot++) {
                        if (sideAfterRot(p, rot, 2) == 0)
                            enumeratePiece(T.bottomSidesRing.base, i, rot, cfg.cellBreaks, true);
                        if (sideAfterRot(p, rot, 3) == 0)
                            enumeratePiece(T.leftSidesRing.base, i, rot, cfg.cellBreaks, true);
                        if (sideAfterRot(p, rot, 0) == 0)
                            enumeratePiece(T.topSidesRing.base, i, rot, cfg.cellBreaks, true);
                        if (sideAfterRot(p, rot, 1) == 0)
                            enumeratePiece(T.rightBrkRing.base, i, rot, cfg.cellBreaks, true);
                    }
                    break;
                default: break;
            }
        }
        finalizeTable(T.cornersRing);
        finalizeTable(T.leftSidesRing);
        finalizeTable(T.bottomSidesRing);
        finalizeTable(T.topSidesRing);
        finalizeTable(T.rightBrkRing);
    }
    finalizeTable(T.corners);
    finalizeTable(T.leftSides);
    finalizeTable(T.bottomSides);
    finalizeTable(T.topSides);
    finalizeTable(T.rightNoBrk);
    finalizeTable(T.rightBrk);
    finalizeTable(T.midNoBrk);
    finalizeTable(T.midBrk);

    // default per-cell assignment ----------------------------------------
    for (int ci = 0; ci < 256; ci++) {
        int row = ci / 16, col = ci % 16, fi = FILL_IDX[ci];
        bool late = fi >= FIRST_BREAK_IDX;
        bool ring = cfg.ringDepth && fi >= cfg.ringDepth;
        if (row == 0)
            T.lookup[ci] = (col == 15) ? (ring ? &T.cornersRing : &T.corners)
                                       : (ring ? &T.bottomSidesRing : &T.bottomSides);
        else if (row == 15)
            T.lookup[ci] = (col == 0 || col == 15)
                               ? (ring ? &T.cornersRing : &T.corners)
                               : (ring ? &T.topSidesRing : &T.topSides);
        else if (col == 0)
            T.lookup[ci] = ring ? &T.leftSidesRing : &T.leftSides;
        else if (col == 15)
            T.lookup[ci] = ring ? &T.rightBrkRing
                                : (late ? &T.rightBrk : &T.rightNoBrk);
        else
            T.lookup[ci] = late ? &T.midBrk : &T.midNoBrk;
    }

    // clue cells + neighbor-filtered cells --------------------------------
    // reserve so pointers into T.special stay valid
    T.special.reserve(HINTS.size() * 3);
    for (const Hint& h : HINTS) {
        int bur = 15 - h.tdRow, buc = h.tdCol;
        int ci = bur * 16 + buc;
        const Piece& p = PIECES[h.piece];
        uint8_t clueW = sideAfterRot(p, h.rot, 3), clueS = sideAfterRot(p, h.rot, 2);
        uint8_t clueN = sideAfterRot(p, h.rot, 0), clueE = sideAfterRot(p, h.rot, 1);
        (void)clueN; (void)clueE;

        // the clue cell itself: single piece, fixed rotation, exact match only
        T.special.emplace_back();
        enumeratePiece(T.special.back().base, h.piece, h.rot, 0);
        finalizeTable(T.special.back());
        T.lookup[ci] = &T.special.back();

        // west neighbor: must show east == clue's west
        int wci = ci - 1;
        bool wLate = FILL_IDX[wci] >= FIRST_BREAK_IDX;
        T.special.emplace_back();
        for (int i = 1; i <= 256; i++)
            if (PIECE_CLASS[i] == 0 && !IS_CLUE[i])
                enumeratePiece(T.special.back().base, i, -1, wLate ? cfg.cellBreaks : 0,
                               false, -1, clueW);
        finalizeTable(T.special.back());
        T.lookup[wci] = &T.special.back();

        // south neighbor: must show north == clue's south
        int sci = ci - 16;
        bool sLate = FILL_IDX[sci] >= FIRST_BREAK_IDX;
        T.special.emplace_back();
        for (int i = 1; i <= 256; i++)
            if (PIECE_CLASS[i] == 0 && !IS_CLUE[i])
                enumeratePiece(T.special.back().base, i, -1, sLate ? cfg.cellBreaks : 0,
                               false, clueS, -1);
        finalizeTable(T.special.back());
        T.lookup[sci] = &T.special.back();
    }
    static std::atomic<bool> dumped{false};
    bool expect = false;
    if (dumped.compare_exchange_strong(expect, true)) {
        logf("table sizes: corners %zu leftSides %zu bottomSides %zu topSides %zu "
             "rightNoBrk %zu rightBrk %zu midNoBrk %zu midBrk %zu",
             T.corners.base.size(), T.leftSides.base.size(), T.bottomSides.base.size(),
             T.topSides.base.size(), T.rightNoBrk.base.size(), T.rightBrk.base.size(),
             T.midNoBrk.base.size(), T.midBrk.base.size());
        if (cfg.ringDepth) {
            int ringCells = 0;
            for (int ci = 0; ci < 256; ci++) {
                int row = ci / 16, col = ci % 16;
                if ((row == 0 || row == 15 || col == 0 || col == 15) &&
                    FILL_IDX[ci] >= cfg.ringDepth)
                    ringCells++;
            }
            logf("ring-break tables (from fill depth %d, %d border cells): corners %zu "
                 "leftSides %zu bottomSides %zu topSides %zu rightBrk %zu",
                 cfg.ringDepth, ringCells, T.cornersRing.base.size(),
                 T.leftSidesRing.base.size(), T.bottomSidesRing.base.size(),
                 T.topSidesRing.base.size(), T.rightBrkRing.base.size());
        }
        for (size_t i = 0; i < T.special.size(); i++)
            logf("special[%zu] (%s of clue %d): %zu entries",
                 i, i % 3 == 0 ? "cell" : (i % 3 == 1 ? "west-nb" : "south-nb"),
                 HINTS[i / 3].piece, T.special[i].base.size());
    }
}

// --------------------------------------------------------------- verification

// Independent rescoring of a completed board. Returns matched edges (of 480),
// or -1 if the board is structurally invalid (bad frame / piece reuse / clue moved).
static int verifyBoard(const RotatedPiece* board) {
    bool used[257] = {false};
    for (int ci = 0; ci < 256; ci++) {
        int pn = board[ci].pieceNumber;
        if (pn < 1 || pn > 256 || used[pn]) return -1;
        used[pn] = true;
    }
    for (const Hint& h : HINTS) {
        int ci = (15 - h.tdRow) * 16 + h.tdCol;
        if (board[ci].pieceNumber != h.piece || board[ci].rot != h.rot) return -1;
    }
    int matched = 0;
    for (int r = 0; r < 16; r++) {
        for (int c = 0; c < 16; c++) {
            int ci = r * 16 + c;
            const Piece& p = PIECES[board[ci].pieceNumber];
            int rot = board[ci].rot;
            uint8_t nn = sideAfterRot(p, rot, 0), ee = sideAfterRot(p, rot, 1),
                    ss = sideAfterRot(p, rot, 2), ww = sideAfterRot(p, rot, 3);
            if (r == 0 && ss != 0) return -1;   // frame must be grey
            if (r == 15 && nn != 0) return -1;
            if (c == 0 && ww != 0) return -1;
            if (c == 15 && ee != 0) return -1;
            if (c < 15) { // east seam
                const Piece& q = PIECES[board[ci + 1].pieceNumber];
                if (ee == sideAfterRot(q, board[ci + 1].rot, 3)) matched++;
            }
            if (r < 15) { // north seam (bu)
                const Piece& q = PIECES[board[ci + 16].pieceNumber];
                if (nn == sideAfterRot(q, board[ci + 16].rot, 2)) matched++;
            }
        }
    }
    return matched;
}

// internal color -> official `jef` motif letter (piecewise reflection, verified
// against the canonical Clues board_edges via the 5 forced clue pieces)
static inline char jefLetter(int c) {
    int i = c == 0 ? 0 : c <= 8 ? 9 - c : c <= 16 ? 25 - c : 39 - c;
    return (char)('a' + i);
}

static void saveBoard(const RotatedPiece* board, int score, int threadId,
                      uint64_t attemptSeed) {
    std::lock_guard<std::mutex> lk(g_saveMutex);
    char path[512], tmp[520];
    time_t t = time(nullptr);
    snprintf(path, sizeof path, "%s/board_%d_%ld_t%d.txt", cfg.saveDir.c_str(),
             score, (long)t, threadId);
    snprintf(tmp, sizeof tmp, "%s.tmp", path);
    FILE* f = fopen(tmp, "w");
    if (!f) { fprintf(stderr, "SAVE FAILED: %s\n", tmp); return; }
    // td row-major pieceID/rot — same format as the existing campaign boards
    for (int tdr = 0; tdr < 16; tdr++) {
        int bur = 15 - tdr;
        for (int c = 0; c < 16; c++) {
            const RotatedPiece& rp = board[bur * 16 + c];
            fprintf(f, "%d/%d%s", rp.pieceNumber, rp.rot, c == 15 ? "\n" : " ");
        }
    }
    fclose(f);
    rename(tmp, path);

    // sidecar metadata + bucas viewer URL (our colors == Blackwood's labels)
    snprintf(tmp, sizeof tmp, "%s.meta", path);
    f = fopen(tmp, "w");
    if (f) {
        fprintf(f, "score %d/480\nthread %d\nattempt_seed %" PRIu64 "\nbreaks %s\n",
                score, threadId, attemptSeed, cfg.breaks.c_str());
        // fill indices that actually consumed a break (for schedule tuning)
        fprintf(f, "breaks_used");
        for (int fi = 0; fi < 256; fi++) {
            int ci = SEQ_ROW[fi] * 16 + SEQ_COL[fi];
            if (board[ci].breakCount) fprintf(f, " %d", fi);
        }
        fprintf(f, "\n");
        fprintf(f, "https://e2.bucas.name/#puzzle=Joshua_Blackwood&board_w=16&board_h=16&board_edges=");
        for (int tdr = 0; tdr < 16; tdr++) {
            int bur = 15 - tdr;
            for (int c = 0; c < 16; c++) {
                const RotatedPiece& rp = board[bur * 16 + c];
                const Piece& p = PIECES[rp.pieceNumber];
                for (int face = 0; face < 4; face++)
                    fputc('a' + sideAfterRot(p, rp.rot, face), f);
            }
        }
        fprintf(f, "&motifs_order=jblackwood\n");
        // same board in the standard jef/Clues space (renders official motif art)
        fprintf(f, "https://e2.bucas.name/#puzzle=Clues&board_w=16&board_h=16&board_edges=");
        for (int tdr = 0; tdr < 16; tdr++) {
            int bur = 15 - tdr;
            for (int c = 0; c < 16; c++) {
                const RotatedPiece& rp = board[bur * 16 + c];
                const Piece& p = PIECES[rp.pieceNumber];
                for (int face = 0; face < 4; face++)
                    fputc(jefLetter(sideAfterRot(p, rp.rot, face)), f);
            }
        }
        fprintf(f, "&board_pieces=");
        for (int tdr = 0; tdr < 16; tdr++)
            for (int c = 0; c < 16; c++)
                fprintf(f, "%03d", board[(15 - tdr) * 16 + c].pieceNumber);
        fprintf(f, "&motifs_order=jef\n");
        fclose(f);
    }

    FILE* bs = fopen((cfg.saveDir + "/best_score.txt.tmp").c_str(), "w");
    if (bs) {
        fprintf(bs, "%d\n", g_bestScore.load());
        fclose(bs);
        rename((cfg.saveDir + "/best_score.txt.tmp").c_str(),
               (cfg.saveDir + "/best_score.txt").c_str());
    }
}

// Preserve exceptionally deep incomplete paths for stronger offline endgame
// work.  Depth 250 supplies the exact finishers, while the much rarer 252+
// records retain the precise paths that made those funnel hits valuable.  Zero
// placements are explicit as 0/0.
static void saveDeepPrefix(const RotatedPiece* board, int depth, int breaksUsed,
                           int threadId, uint64_t attemptSeed) {
    std::lock_guard<std::mutex> lk(g_saveMutex);
    char path[512], tmp[520];
    snprintf(path, sizeof path, "%s/prefix_%d_seed%" PRIu64 "_t%d.txt",
             cfg.saveDir.c_str(), depth, attemptSeed, threadId);
    snprintf(tmp, sizeof tmp, "%s.tmp", path);
    FILE* f = fopen(tmp, "w");
    if (!f) { fprintf(stderr, "PREFIX SAVE FAILED: %s\n", tmp); return; }
    for (int tdr = 0; tdr < 16; tdr++) {
        int bur = 15 - tdr;
        for (int c = 0; c < 16; c++) {
            const RotatedPiece& rp = board[bur * 16 + c];
            fprintf(f, "%d/%d%s", rp.pieceNumber, rp.rot,
                    c == 15 ? "\n" : " ");
        }
    }
    fclose(f);
    rename(tmp, path);
    snprintf(tmp, sizeof tmp, "%s.meta", path);
    f = fopen(tmp, "w");
    if (f) {
        fprintf(f, "depth %d/256\nbreaks_used %d\nthread %d\nattempt_seed %" PRIu64
                   "\nbreaks %s\ntail_target_score %d\n",
                depth, breaksUsed, threadId, attemptSeed, cfg.breaks.c_str(),
                cfg.tailTargetScore >= 0
                    ? cfg.tailTargetScore
                    : 480 - BREAK_ARRAY[255]);
        fclose(f);
    }
}

// Exact cold-path solver for the last few cells.  The main engine deliberately
// uses one extremely cheap topological order; once per restart, at --tail-mrv
// depth, this routine tries the same leftover pool with a most-constrained-cell
// order.  It scores every newly closed seam exactly, so unlike the fast tables
// it may also use a ring-colour mismatch when that still fits the target budget.
// Any result is independently checked by verifyBoard before it can be saved.
struct TailMrvState {
    RotatedPiece* board;
    bool* used;
    bool inTail[256] = {};
    bool placed[256] = {};
    int cells[256] = {};
    int nCells = 0;
    int cluePiece[256] = {};
    int clueRot[256] = {};
    uint64_t nodes = 0, nodeCap = 0;
    int budget = 0;
    int solvedScore = -1;
    bool capped = false;
    // Branch and bound on total mismatches: bestCost is the cheapest complete
    // filling found so far (budget + 1 while none is known), and bestBoard keeps
    // it. Searching for the minimum rather than the first admissible filling
    // costs nothing extra — the incumbent replaces the fixed budget as the
    // pruning bound — and it means a call can return 466 where the old
    // first-solution search would have stopped at 465.
    int bestCost = 0;
    int sup[NCOLORS] = {0};         // colour supply of the still-unplaced pieces
    int bestRing = -1;              // ring-colour mismatches in the incumbent
    RotatedPiece bestBoard[256] = {};
};

struct AdaptiveTailTier {
    int triggerDepth;
    int rewindDepth;
    uint64_t nodeCap;
};

// A deeper main-DFS record earns a wider exact neighbourhood.  Each rewind
// depth is unique, so these calls do not deterministically repeat the ordinary
// depth-236 sampler or one another.
static constexpr AdaptiveTailTier ADAPTIVE_TAIL_TIERS[4] = {
    {248, 235,   2000000ULL},
    {250, 234,   5000000ULL},
    {252, 232,  50000000ULL},
    // Depth 253 is extremely rare (three hits in the previous 6.5T-node run)
    // and every 100M-node rewind capped.  A 1B cap costs negligible campaign
    // throughput at that frequency and gives the fertile 26-cell suffix a
    // meaningful exact pass.
    {253, 230, 1000000000ULL},
};

static inline int tailFace(const RotatedPiece& rp, int face) {
    if (face == 0) return rp.top;
    if (face == 1) return rp.right;
    return sideAfterRot(PIECES[rp.pieceNumber], rp.rot, face);
}

static inline bool tailFrameOK(int ci, int piece, int rot) {
    int r = ci >> 4, c = ci & 15;
    bool n = sideAfterRot(PIECES[piece], rot, 0) == 0;
    bool e = sideAfterRot(PIECES[piece], rot, 1) == 0;
    bool s = sideAfterRot(PIECES[piece], rot, 2) == 0;
    bool w = sideAfterRot(PIECES[piece], rot, 3) == 0;
    return n == (r == 15) && e == (c == 15) &&
           s == (r == 0) && w == (c == 0);
}

// How many of a complete board's mismatched seams sit on a ring colour, i.e. on
// the 60-seam frame cycle. Tells us whether an endgame's cost is dominated by the
// frame (a border-piece pool problem) or by the interior (a middle-pool problem).
static int countRingMismatches(const RotatedPiece* board) {
    int ring = 0;
    for (int r = 0; r < 16; r++)
        for (int c = 0; c < 16; c++) {
            int ci = r * 16 + c;
            const Piece& p = PIECES[board[ci].pieceNumber];
            int rot = board[ci].rot;
            uint8_t ee = sideAfterRot(p, rot, 1), nn = sideAfterRot(p, rot, 0);
            if (c < 15) {
                const Piece& q = PIECES[board[ci + 1].pieceNumber];
                uint8_t ww = sideAfterRot(q, board[ci + 1].rot, 3);
                if (ee != ww && (IS_RING_COLOR[ee] || IS_RING_COLOR[ww])) ring++;
            }
            if (r < 15) {
                const Piece& q = PIECES[board[ci + 16].pieceNumber];
                uint8_t ss = sideAfterRot(q, board[ci + 16].rot, 2);
                if (nn != ss && (IS_RING_COLOR[nn] || IS_RING_COLOR[ss])) ring++;
            }
        }
    return ring;
}

__attribute__((cold, noinline))
static bool tailMrvDfs(TailMrvState& q, int depth, int running) {
    if (running >= q.bestCost || g_stop.load(std::memory_order_relaxed))
        return false;
    if (q.nodes >= q.nodeCap) {
        q.capped = true;
        return false;
    }
    q.nodes++;
    if (depth == q.nCells) {
        int score = verifyBoard(q.board);
        if (score < 0 || 480 - score >= q.bestCost) return false;
        q.bestCost = 480 - score;          // new incumbent tightens the bound
        q.bestRing = countRingMismatches(q.board);
        q.solvedScore = score;
        memcpy(q.bestBoard, q.board, sizeof q.bestBoard);
        return false;                      // keep searching for a cheaper filling
    }

    static constexpr int DR[4] = {1, 0, -1, 0};
    static constexpr int DC[4] = {0, 1, 0, -1};
#ifdef BW_TAIL_TRUE_MRV
    // True MRV: choose the cell with the fewest currently legal
    // (piece,rotation) values.  The former topological proxy only maximized the
    // number of determined neighbours; it could therefore branch on a cell
    // with dozens of values while another live cell already had a singleton or
    // an empty domain.  This is a cold-path endgame solver, so spending a scan
    // here is cheap relative to the subtree it avoids.  Determined-neighbour
    // count remains the stable tie-break.
    int ci = -1, bestDomain = INT_MAX, bestDet = -1;
    for (int k = 0; k < q.nCells; k++) {
        int x = q.cells[k];
        if (q.placed[x]) continue;
        int r = x >> 4, c = x & 15, det = 0;
        for (int f = 0; f < 4; f++) {
            int rr = r + DR[f], cc = c + DC[f];
            if (rr < 0 || rr >= 16 || cc < 0 || cc >= 16) { det++; continue; }
            int nb = rr * 16 + cc;
            if (!q.inTail[nb] || q.placed[nb]) det++;
        }
        int domain = 0;
        for (int p = 1; p <= 256; p++) {
            if (q.used[p]) continue;
            if (IS_CLUE[p] && q.cluePiece[x] != p) continue;
            if (q.cluePiece[x] && q.cluePiece[x] != p) continue;
            for (int rot = 0; rot < 4; rot++) {
                if (q.cluePiece[x] && q.clueRot[x] != rot) continue;
                if (!tailFrameOK(x, p, rot)) continue;
                RotatedPiece rp = {(uint16_t)p, (uint8_t)rot,
                                   sideAfterRot(PIECES[p], rot, 0),
                                   sideAfterRot(PIECES[p], rot, 1), 0,
                                   PIECE_HSC[p], 0};
                int add = 0;
                for (int f = 0; f < 4; f++) {
                    int rr = r + DR[f], cc = c + DC[f];
                    if (rr < 0 || rr >= 16 || cc < 0 || cc >= 16) continue;
                    int nb = rr * 16 + cc;
                    if (q.inTail[nb] && !q.placed[nb]) continue;
                    const RotatedPiece& other = q.board[nb];
                    if (!other.pieceNumber ||
                        tailFace(rp, f) != tailFace(other, (f + 2) & 3))
                        add++;
                }
                domain += running + add < q.bestCost;
            }
        }
        if (domain == 0) return false;
        if (domain < bestDomain || (domain == bestDomain && det > bestDet)) {
            bestDomain = domain;
            bestDet = det;
            ci = x;
        }
    }
#else
    // Production baseline: a cheap MRV proxy that maximizes already determined
    // neighbours.  Frame sides count because they restrict the piece class.
    //
    // The same scan collects the colour demand of every open seam, giving an
    // admissible bound: a demand edge of colour c can only be matched by one of
    // the sup[c] colour-c edges still in hand, so at least
    // sum_c max(0, demand[c] - sup[c]) further seams must mismatch. Cheap here
    // (one pass over a handful of cells) and it is what makes a shallower
    // cutoff — a wider exact region — affordable.
    int ci = -1, bestDet = -1;
    int dem[NCOLORS] = {0};
    for (int k = 0; k < q.nCells; k++) {
        int x = q.cells[k];
        if (q.placed[x]) continue;
        int r = x >> 4, c = x & 15, det = 0;
        for (int f = 0; f < 4; f++) {
            int rr = r + DR[f], cc = c + DC[f];
            if (rr < 0 || rr >= 16 || cc < 0 || cc >= 16) { det++; continue; }
            int nb = rr * 16 + cc;
            if (!q.inTail[nb] || q.placed[nb]) {
                det++;
                const RotatedPiece& o = q.board[nb];
                if (o.pieceNumber) dem[tailFace(o, (f + 2) & 3)]++;
            }
        }
        if (det > bestDet) { bestDet = det; ci = x; }
    }
    int lb = 0;
    for (int c2 = 1; c2 < NCOLORS; c2++)
        if (dem[c2] > q.sup[c2]) lb += dem[c2] - q.sup[c2];
    if (running + lb >= q.bestCost) return false;
#endif

    struct Cand { RotatedPiece rp; uint8_t add; };
    Cand cand[1024];
    int nc = 0, r = ci >> 4, c = ci & 15;
    for (int p = 1; p <= 256; p++) {
        if (q.used[p]) continue;
        if (IS_CLUE[p] && q.cluePiece[ci] != p) continue;
        if (q.cluePiece[ci] && q.cluePiece[ci] != p) continue;
        for (int rot = 0; rot < 4; rot++) {
            if (q.cluePiece[ci] && q.clueRot[ci] != rot) continue;
            if (!tailFrameOK(ci, p, rot)) continue;
            RotatedPiece rp = {(uint16_t)p, (uint8_t)rot,
                               sideAfterRot(PIECES[p], rot, 0),
                               sideAfterRot(PIECES[p], rot, 1), 0,
                               PIECE_HSC[p], 0};
            int add = 0;
            for (int f = 0; f < 4; f++) {
                int rr = r + DR[f], cc = c + DC[f];
                if (rr < 0 || rr >= 16 || cc < 0 || cc >= 16) continue;
                int nb = rr * 16 + cc;
                if (q.inTail[nb] && !q.placed[nb]) continue;
                const RotatedPiece& other = q.board[nb];
                if (!other.pieceNumber || tailFace(rp, f) != tailFace(other, (f + 2) & 3))
                    add++;
            }
            if (running + add < q.bestCost)
                cand[nc++] = {rp, (uint8_t)add};
        }
    }
    // With <=8 cells, insertion sort is cheaper and simpler than allocating.
    for (int a = 1; a < nc; a++) {
        Cand v = cand[a]; int b = a - 1;
        while (b >= 0 && cand[b].add > v.add) { cand[b + 1] = cand[b]; b--; }
        cand[b + 1] = v;
    }

    q.placed[ci] = true;
    for (int i = 0; i < nc; i++) {
        RotatedPiece rp = cand[i].rp;
        rp.breakCount = cand[i].add; // metadata only; verifier owns the score
        q.board[ci] = rp;
        q.used[rp.pieceNumber] = true;
        const Piece& pl = PIECES[rp.pieceNumber];
        q.sup[pl.n]--; q.sup[pl.e]--; q.sup[pl.s]--; q.sup[pl.w]--;
        tailMrvDfs(q, depth + 1, running + cand[i].add);
        q.sup[pl.n]++; q.sup[pl.e]++; q.sup[pl.s]++; q.sup[pl.w]++;
        if (q.nodes >= q.nodeCap || g_stop.load(std::memory_order_relaxed)) {
            q.used[rp.pieceNumber] = false;
            q.board[ci] = {};
            q.placed[ci] = false;
            return false;
        }   // supply already restored above
        q.used[rp.pieceNumber] = false;
        q.board[ci] = {};
    }
    q.placed[ci] = false;
    return false;
}

// Exactly complete the fill-order suffix from startDepth, minimizing total
// mismatches.  Returns the best score reachable within `budgetOverride` (or the
// schedule's total budget), or -1 when no filling that cheap exists.  On success
// `board` holds the winning arrangement.  `costOut` reports the cheapest cost
// seen even when it misses the budget, which is what the endgame telemetry
// needs: the interesting question is not "did it fit" but "by how much did it
// miss".
__attribute__((cold, noinline))
static int solveTailMrv(RotatedPiece* board, bool* used, int startDepth,
                        int breaksUsed, uint64_t nodeCap, uint64_t* nodesOut,
                        bool* cappedOut = nullptr, int* costOut = nullptr,
                        int budgetOverride = -1, int acceptCost = INT_MAX,
                        int* ringOut = nullptr) {
    TailMrvState q;
    q.board = board;
    q.used = used;
    q.nodeCap = nodeCap;
    q.budget = budgetOverride >= 0 ? budgetOverride : TOTAL_BUDGET;
    q.bestCost = q.budget + 1;
    for (const Hint& h : HINTS) {
        int ci = (15 - h.tdRow) * 16 + h.tdCol;
        q.cluePiece[ci] = h.piece;
        q.clueRot[ci] = h.rot;
    }
    for (int d = startDepth; d < 256; d++) {
        int ci = SEQ_ROW[d] * 16 + SEQ_COL[d];
        q.inTail[ci] = true;
        q.cells[q.nCells++] = ci;
    }
    for (int p = 1; p <= 256; p++) {
        if (used[p]) continue;
        const Piece& pc = PIECES[p];
        q.sup[pc.n]++; q.sup[pc.e]++; q.sup[pc.s]++; q.sup[pc.w]++;
    }
    tailMrvDfs(q, 0, breaksUsed);
    if (nodesOut) *nodesOut = q.nodes;
    if (cappedOut) *cappedOut = q.capped;
    if (costOut) *costOut = q.bestCost;
    if (ringOut) *ringOut = q.bestRing;
    // A filling that misses acceptCost is telemetry only: leave board/used
    // exactly as the caller had them so the live DFS can carry on.
    if (q.solvedScore < 0 || q.bestCost > acceptCost) return -1;
    memcpy(board, q.bestBoard, sizeof q.bestBoard);   // restore the incumbent
    return q.solvedScore;
}

// Temporarily unplace a suffix of the live Blackwood path, solve that larger
// constrained sub-puzzle, then restore the exact DFS state if it fails.  Full
// snapshots make restoration independent of how/where MRV stopped (exhausted,
// capped, or signal) and keep used[] consistent with board[].
__attribute__((cold, noinline))
static int solveTailMrvRewind(RotatedPiece* board, bool* used,
                              int currentDepth, int rewindDepth,
                              int breaksUsed, uint64_t nodeCap,
                              uint64_t* nodesOut, bool* cappedOut,
                              int* costOut = nullptr, int budgetOverride = -1,
                              int acceptCost = INT_MAX) {
    RotatedPiece savedBoard[256];
    bool savedUsed[257];
    memcpy(savedBoard, board, sizeof savedBoard);
    memcpy(savedUsed, used, sizeof savedUsed);

    for (int d = rewindDepth; d < currentDepth; d++) {
        int ci = SEQ_ROW[d] * 16 + SEQ_COL[d];
        if (board[ci].pieceNumber)
            used[board[ci].pieceNumber] = false;
        board[ci] = {};
    }

    int score = solveTailMrv(board, used, rewindDepth, breaksUsed,
                             nodeCap, nodesOut, cappedOut, costOut,
                             budgetOverride, acceptCost);
    if (score < 0) {
        memcpy(board, savedBoard, sizeof savedBoard);
        memcpy(used, savedUsed, sizeof savedUsed);
    }
    return score;
}

// ------------------------------------------------------------------ the solver

static void solverThread(int threadId) {
#ifdef BW_COMPILED_TAIL_DEPTH
    static constexpr int HOT_TAIL_DEPTH = BW_COMPILED_TAIL_DEPTH;
#else
    const int HOT_TAIL_DEPTH = cfg.tailDepth;
#endif
    Tables T;
    buildTables(T);
    FILE* poolF = nullptr;                 // pool telemetry sink (per thread)
    std::string poolBuf;
    ThreadStats& st = g_stats[threadId];
    auto tStart = std::chrono::steady_clock::now();

    uint64_t attempt = 0;
    while (!g_stop.load(std::memory_order_relaxed)) {
        attempt++;
        uint64_t attemptSeed =
            cfg.seed * 1000000007ULL + (uint64_t)threadId * 1000003ULL + attempt;
        Rng rng(attemptSeed);
        for (auto* t : {&T.corners, &T.leftSides, &T.topSides, &T.rightNoBrk,
                        &T.rightBrk, &T.midNoBrk, &T.midBrk})
            sortTableForRestart(*t, rng);
        sortBottomRowForRestart(T.bottomSides, rng);
        if (cfg.ringDepth) {
            for (auto* t : {&T.cornersRing, &T.leftSidesRing, &T.topSidesRing,
                            &T.rightBrkRing})
                sortTableForRestart(*t, rng);
            sortBottomRowForRestart(T.bottomSidesRing, rng);
        }
        for (auto& sp : T.special) sortTableForRestart(sp, rng);

        // ---- one restart attempt -----------------------------------------
        RotatedPiece board[256] = {};
        bool used[257] = {false};
        uint8_t cumBreaks[256] = {0};
        uint8_t cumHeur[256] = {0};
        uint16_t tryNext[256] = {0};

        // supply/demand state (--sd-prune): S[c] = edges of color c on unplaced
        // pieces; D[c] = edges of color c exposed by placed cells toward empty
        // in-board neighbors. Necessary condition for completability: S >= D.
        int16_t S[NCOLORS] = {0}, D[NCOLORS] = {0};
        // side color of a placed cell facing a given direction (0=N 1=E 2=S 3=W)
        auto faceColor = [&](const RotatedPiece& rp, int face) -> int {
            if (face == 0) return rp.top;
            if (face == 1) return rp.right;
            return sideAfterRot(PIECES[rp.pieceNumber], rp.rot, face);
        };
        // apply placement/removal effects on S/D. dir: +1 place, -1 remove.
        auto sdApply = [&](int ci, const RotatedPiece& rp, int dir) {
            const Piece& p = PIECES[rp.pieceNumber];
            S[p.n] -= dir; S[p.e] -= dir; S[p.s] -= dir; S[p.w] -= dir;
            const int r = ci >> 4, c = ci & 15;
            const int nb[4] = { r < 15 ? ci + 16 : -1, c < 15 ? ci + 1 : -1,
                                r > 0 ? ci - 16 : -1, c > 0 ? ci - 1 : -1 };
            for (int f = 0; f < 4; f++) {   // f: my face N,E,S,W toward nb[f]
                if (nb[f] < 0) continue;    // frame side: no demand
                const RotatedPiece& o = board[nb[f]];
                if (o.pieceNumber) D[faceColor(o, (f + 2) & 3)] -= dir; // seam closed/reopened
                else D[faceColor(rp, f)] += dir;                       // exposed/unexposed
            }
        };
        auto sdViolated = [&]() {
            for (int c2 = 1; c2 < NCOLORS; c2++)
                if (D[c2] > S[c2]) return true;
            return false;
        };

        // seed cell 0 = bu(0,0): random corner showing grey west+south
        {
            uint32_t a = T.corners.off[0], b = T.corners.off[0 + 1];
            if (b <= a) die("no corner candidates for cell 0");
            const RotatedPiece& c0 = T.corners.pool[a + rng.below(b - a)];
            board[0] = c0;
            used[c0.pieceNumber] = true;
            cumBreaks[0] = 0;
            cumHeur[0] = c0.hsc;
            if (cfg.sdPrune) {
                for (int p = 1; p <= 256; p++) {
                    if (p == c0.pieceNumber) continue;
                    S[PIECES[p].n]++; S[PIECES[p].e]++; S[PIECES[p].s]++; S[PIECES[p].w]++;
                }
                D[faceColor(c0, 0)]++;      // exposes north toward bu(1,0)
                D[faceColor(c0, 1)]++;      // exposes east toward bu(0,1)
            }
        }

        int si = 1, maxSi = 1;
        uint64_t nodes = 0;
        uint64_t placements = 0, lastFlushedPlacements = 0;
        uint64_t nodesAtDepthCheck = 0;
        bool snapped = false;              // pool telemetry: mask at depth 176
        uint8_t snapMask[32];
        uint8_t brokenAt[256] = {0};       // --isolated-breaks: incident broken seams
        int16_t brokeNbAt[256];            // per-depth: other cell of the broken seam
        if (cfg.isolatedBreaks) memset(brokeNbAt, 0xFF, sizeof brokeNbAt); // -1
        bool tailCalled = false;
        uint64_t nextTailAt = 0;
        // deepest cell index reached while holding each cumulative spend
        uint16_t depthAtSpend[32] = {0};
        uint64_t attemptArrivals = 0;   // exact-endgame calls this attempt made
        uint64_t endgameSeq = 0;        // call counter driving --endgame-sample
        uint64_t attemptCap = cfg.yieldBase ? cfg.yieldBase : cfg.nodeCap;

        while (true) {
            nodes++;
            if (cfg.stats) st.depthNodes[si]++;
            if ((nodes & 0xFFFFF) == 0) { // every ~1M nodes: time/stop checks
                if (g_stop.load(std::memory_order_relaxed)) break;
                if (cfg.seconds > 0) {
                    auto el = std::chrono::duration_cast<std::chrono::seconds>(
                                  std::chrono::steady_clock::now() - tStart).count();
                    if (el >= cfg.seconds) { g_stop = true; break; }
                }
                st.nodes.fetch_add(0x100000, std::memory_order_relaxed);
                st.placements.fetch_add(placements - lastFlushedPlacements,
                                        std::memory_order_relaxed);
                lastFlushedPlacements = placements;
            }
            if (si > maxSi) {
                maxSi = si;
                nodesAtDepthCheck = nodes;
                if ((si == 250 || si >= 252) &&
                    TOTAL_BUDGET - (int)cumBreaks[si - 1] >= cfg.tailMinSlack)
                    saveDeepPrefix(board, si, cumBreaks[si - 1],
                                   threadId, attemptSeed);
                if (!snapped && si >= 176 && !cfg.poolLog.empty()) {
                    snapped = true;        // cold path: first crossing only
                    memset(snapMask, 0, sizeof snapMask);
                    for (int p = 1; p <= 256; p++)
                        if (used[p]) snapMask[(p - 1) >> 3] |= (uint8_t)(1u << ((p - 1) & 7));
                }
                // A newly established near-miss is disproportionately valuable.
                // Retry the exact official-score finisher at every deeper record
                // so depth-252/254 prefixes cannot miss MRV merely because the
                // periodic depth-236 sampler was not due on their ancestor.
                if (cfg.tailDepth && si > cfg.tailDepth && si < 256 &&
                    TOTAL_BUDGET - (int)cumBreaks[si - 1] >= cfg.tailMinSlack) {
                    uint64_t tn = 0;
                    int score = solveTailMrv(board, used, si, cumBreaks[si - 1],
                                             cfg.tailNodes, &tn);
                    st.tailCalls.fetch_add(1, std::memory_order_relaxed);
                    st.tailNodes.fetch_add(tn, std::memory_order_relaxed);
                    if (score >= 0) {
                        st.completions.fetch_add(1, std::memory_order_relaxed);
                        int prev = g_bestScore.load();
                        while (score > prev &&
                               !g_bestScore.compare_exchange_weak(prev, score)) {}
                        logf("TAIL-MRV COMPLETION: %d/480 thread %d seed %" PRIu64
                             " depth %d nodes %" PRIu64 "%s",
                             score, threadId, attemptSeed, si, tn,
                             score > prev ? "  *** NEW BEST ***" : "");
                        if (score >= cfg.minSave)
                            saveBoard(board, score, threadId, attemptSeed);
                        break;
                    }
                }
                // A record at 248+ is evidence that this attempt's earlier
                // prefix is unusually fertile.  Spend progressively more exact
                // work by unplacing a wider, previously untested suffix.  The
                // cheap exact-current-suffix call above still refutes the exact
                // 8/6/4/2-cell dead end first.
                if (cfg.tailAdaptive && si < 256 &&
                    TOTAL_BUDGET - (int)cumBreaks[si - 1] >= cfg.tailMinSlack) {
                    int tier = -1;
                    for (int k = 0; k < 4; k++)
                        if (si == ADAPTIVE_TAIL_TIERS[k].triggerDepth) tier = k;
                    if (tier >= 0) {
                        const AdaptiveTailTier& at = ADAPTIVE_TAIL_TIERS[tier];
                        uint64_t tn = 0;
                        bool capped = false;
                        int score = solveTailMrvRewind(
                            board, used, si, at.rewindDepth,
                            cumBreaks[at.rewindDepth - 1], at.nodeCap,
                            &tn, &capped);
                        st.tailCalls.fetch_add(1, std::memory_order_relaxed);
                        st.tailNodes.fetch_add(tn, std::memory_order_relaxed);
                        st.adaptiveCalls[tier].fetch_add(1, std::memory_order_relaxed);
                        st.adaptiveNodes[tier].fetch_add(tn, std::memory_order_relaxed);
                        if (capped)
                            st.adaptiveCapped[tier].fetch_add(1, std::memory_order_relaxed);
                        if (score >= 0) {
                            st.completions.fetch_add(1, std::memory_order_relaxed);
                            int prev = g_bestScore.load();
                            while (score > prev &&
                                   !g_bestScore.compare_exchange_weak(prev, score)) {}
                            logf("ADAPTIVE-MRV COMPLETION: %d/480 thread %d seed %" PRIu64
                                 " record %d rewind %d nodes %" PRIu64 "%s",
                                 score, threadId, attemptSeed, si, at.rewindDepth, tn,
                                 score > prev ? "  *** NEW BEST ***" : "");
                            if (score >= cfg.minSave)
                                saveBoard(board, score, threadId, attemptSeed);
                            break;
                        }
                    }
                }
                if (si == 256) { // completed board
                    int breaks = cumBreaks[255];
                    int score = 480 - breaks;
                    st.completions.fetch_add(1, std::memory_order_relaxed);
                    int v = verifyBoard(board);
                    if (v != score) {
                        logf("CRITICAL BUG: engine score %d != verified %d — not saving",
                             score, v);
                    } else {
                        int prev = g_bestScore.load();
                        while (score > prev && !g_bestScore.compare_exchange_weak(prev, score)) {}
                        logf("COMPLETION: %d/480 (%d breaks) thread %d seed %" PRIu64 "%s%s",
                             score, breaks, threadId, attemptSeed,
                             score > prev ? "  *** NEW BEST ***" : "",
                             score == 480 ? "  ***** FULL SOLUTION — ETERNITY II SOLVED *****" : "");
                        if (score >= cfg.minSave) saveBoard(board, score, threadId, attemptSeed);
                    }
                    break; // restart with fresh ordering
                }
            }
            if (nodes > attemptCap) break;
            if (cfg.pruneNodes && nodes - nodesAtDepthCheck > cfg.pruneNodes &&
                maxSi < cfg.pruneDepth)
                break; // prune-back: stalled shallow, restart early

            int row = SEQ_ROW[si], col = SEQ_COL[si];
            int ci = row * 16 + col;

            if (board[ci].pieceNumber) {
                if (cfg.sdPrune) sdApply(ci, board[ci], -1);
                if (cfg.isolatedBreaks && brokeNbAt[si] >= 0) {
                    brokenAt[ci]--;
                    brokenAt[brokeNbAt[si]]--;
                    brokeNbAt[si] = -1;
                }
                used[board[ci].pieceNumber] = false;
                board[ci].pieceNumber = 0;
            }

            const Table* tab = T.lookup[ci];
            int left = col ? board[ci - 1].right : 0;
            int bottom = row ? board[ci - 16].top : 0;
            int key = left * NCOLORS + bottom;
            const RotatedPiece* cand = tab->pool.data() + tab->off[key];
            int nCand = (int)(tab->off[key + 1] - tab->off[key]);

            int allowedBreaks = BREAK_ARRAY[si] - cumBreaks[si - 1];
            bool found = false;
            for (int i = tryNext[si]; i < nCand; i++) {
                const RotatedPiece c = cand[i];
                if ((int)c.breakCount > allowedBreaks) break; // sorted: breaks last
                if (used[c.pieceNumber]) continue;
                if (si <= MAX_HEUR_IDX &&
                    (int)cumHeur[si - 1] + (int)c.hsc < HEUR_ARRAY[si])
                    break; // sorted by hsc desc within break class
                board[ci] = c;
                int isoNb = -1;                // --isolated-breaks bookkeeping
                if (cfg.isolatedBreaks && c.breakCount) {
                    int westC = sideAfterRot(PIECES[c.pieceNumber], c.rot, 3);
                    isoNb = (westC != left) ? ci - 1 : ci - 16;
                    if (brokenAt[ci] || brokenAt[isoNb]) {  // would touch a break
                        board[ci].pieceNumber = 0;
                        continue;
                    }
                }
                if (cfg.sdPrune) {
                    sdApply(ci, c, +1);
                    if (sdViolated()) {       // pool provably can't feed the frontier
                        sdApply(ci, c, -1);
                        board[ci].pieceNumber = 0;
                        continue;
                    }
                }
                if (isoNb >= 0) {
                    brokenAt[ci]++;
                    brokenAt[isoNb]++;
                    brokeNbAt[si] = (int16_t)isoNb;
                }
                if (cfg.stats) (c.breakCount ? st.depthHalf : st.depthFits)[si]++;
                used[c.pieceNumber] = true;
                cumBreaks[si] = (uint8_t)(cumBreaks[si - 1] + c.breakCount);
                cumHeur[si] = (uint8_t)(cumHeur[si - 1] + c.hsc);
                tryNext[si] = (uint16_t)(i + 1);
                si++;
                found = true;
                break;
            }
            if (found) {
                placements++;
                // cost funnel: only the deep region can satisfy any threshold,
                // so the tracking cost stays off the shallow hot path
                if (__builtin_expect(si >= FUNNEL_TH[0], false)) {
                    uint8_t sp = cumBreaks[si - 1];
                    if (sp < 32 && si > depthAtSpend[sp]) depthAtSpend[sp] = (uint16_t)si;
                }
                // exact minimum-cost endgame, once per distinct prefix of this depth
                if (__builtin_expect(cfg.endgameDepth && si == cfg.endgameDepth, false)) {
                    int spent = (int)cumBreaks[si - 1];
                    if (TOTAL_BUDGET - spent >= cfg.tailMinSlack) {
                        uint64_t tn = 0;
                        bool capped = false;
                        endgameSeq++;
                        bool probeThis = cfg.endgameProbeExtra &&
                            (!cfg.endgameSample || endgameSeq % cfg.endgameSample == 0);
                        int ceiling = TOTAL_BUDGET + (probeThis ? cfg.endgameProbeExtra : 0);
                        int cost = ceiling + 1;
                        int ringMis = -1;
                        int score = solveTailMrv(
                            board, used, si, spent, cfg.endgameNodes, &tn, &capped,
                            &cost, ceiling, TOTAL_BUDGET, &ringMis);
                        if (ringMis >= 0)
                            st.endgameRing[ringMis < 15 ? ringMis : 15].fetch_add(
                                1, std::memory_order_relaxed);
                        st.endgameCalls.fetch_add(1, std::memory_order_relaxed);
                        st.endgameNodes.fetch_add(tn, std::memory_order_relaxed);
                        if (capped) st.endgameCapped.fetch_add(1, std::memory_order_relaxed);
                        if (cfg.endgameSample && !probeThis) {
                            // unsampled call: counted, but kept out of the
                            // distribution it would bias
                            if (score < 0 && !capped) { si--; continue; }
                        }
                        int slackIdx = TOTAL_BUDGET - spent;
                        slackIdx = slackIdx < 0 ? 0 : (slackIdx > 7 ? 7 : slackIdx);
                        int egCost = cost - spent;   // sentinel cost stays > probe
                        egCost = egCost < 0 ? 0 : (egCost > 15 ? 15 : egCost);
                        st.endgameGrid[slackIdx][egCost].fetch_add(
                            1, std::memory_order_relaxed);
                        attemptArrivals++;
                        attemptCap += cfg.yieldPer;   // productive attempts earn more nodes
                        if (score >= 0) {
                            st.completions.fetch_add(1, std::memory_order_relaxed);
                            int v = verifyBoard(board);
                            if (v != score) {
                                logf("CRITICAL BUG: endgame score %d != verified %d "
                                     "— not saving", score, v);
                            } else {
                                int prev = g_bestScore.load();
                                while (score > prev &&
                                       !g_bestScore.compare_exchange_weak(prev, score)) {}
                                logf("ENDGAME COMPLETION: %d/480 thread %d seed %" PRIu64
                                     " depth %d spent %d nodes %" PRIu64 "%s",
                                     score, threadId, attemptSeed, si, spent, tn,
                                     score > prev ? "  *** NEW BEST ***" : "");
                                if (score >= cfg.minSave)
                                    saveBoard(board, score, threadId, attemptSeed);
                            }
                            break;
                        }
                        // Ladder: each rung re-solves an ever wider suffix
                        // exactly, entered only when the previous pass left this
                        // prefix within that rung's gap of the budget.
                        bool laddered = false;
                        if (score < 0 && !capped && !cfg.ladder.empty()) {
                            int curCost = cost;
                            for (size_t rk = 0; rk < cfg.ladder.size(); rk++) {
                                const Config::EscRung& rung = cfg.ladder[rk];
                                if (curCost > TOTAL_BUDGET + rung.gap) break;
                                if (rung.rewind >= si) break;
                                uint64_t tnr = 0;
                                bool cappedr = false;
                                int costr = TOTAL_BUDGET + rung.gap + 1;
                                int sr = solveTailMrvRewind(
                                    board, used, si, rung.rewind,
                                    (int)cumBreaks[rung.rewind - 1], rung.nodes,
                                    &tnr, &cappedr, &costr,
                                    TOTAL_BUDGET + rung.gap, TOTAL_BUDGET);
                                st.escCalls[rk].fetch_add(1, std::memory_order_relaxed);
                                st.escNodes[rk].fetch_add(tnr, std::memory_order_relaxed);
                                if (cappedr)
                                    st.escCapped[rk].fetch_add(1, std::memory_order_relaxed);
                                int gr = costr - TOTAL_BUDGET;
                                st.escGapHist[rk][gr < 0 ? 0 : (gr > 15 ? 15 : gr)]
                                    .fetch_add(1, std::memory_order_relaxed);
                                if (sr >= 0) {
                                    st.completions.fetch_add(1, std::memory_order_relaxed);
                                    int vr = verifyBoard(board);
                                    if (vr != sr) {
                                        logf("CRITICAL BUG: ladder score %d != verified"
                                             " %d — not saving", sr, vr);
                                    } else {
                                        int prevr = g_bestScore.load();
                                        while (sr > prevr &&
                                               !g_bestScore.compare_exchange_weak(prevr, sr)) {}
                                        logf("ESCALATED COMPLETION: %d/480 thread %d seed %"
                                             PRIu64 " rung %zu rewind %d spent %d nodes %"
                                             PRIu64 "%s", sr, threadId, attemptSeed,
                                             rk + 1, rung.rewind, spent, tnr,
                                             sr > prevr ? "  *** NEW BEST ***" : "");
                                        if (sr >= cfg.minSave)
                                            saveBoard(board, sr, threadId, attemptSeed);
                                    }
                                    laddered = true;
                                    break;
                                }
                                if (costr < curCost) curCost = costr;
                            }
                        }
                        if (laddered) break;
                        // Exhausted with nothing cheap enough: the whole subtree
                        // under this prefix is refuted, including the fillings the
                        // hot tables cannot even represent. Retry this cell.
                        if (!capped) { si--; continue; }
                    }
                }
                // We only arrive at a new prefix after a successful placement,
                // so keep the cold hybrid check off the backtracking half of
                // the hot loop.  The first depth crossing is always tested;
                // --tail-period sparsely samples later distinct prefixes.
                if (__builtin_expect(HOT_TAIL_DEPTH && si == HOT_TAIL_DEPTH &&
                        (!tailCalled || (cfg.tailPeriod && nodes >= nextTailAt)), false)) {
                    uint64_t tn = 0;
                    int score = solveTailMrv(board, used, si, cumBreaks[si - 1],
                                             cfg.tailNodes, &tn);
                    tailCalled = true;
                    nextTailAt = nodes + cfg.tailPeriod;
                    st.tailCalls.fetch_add(1, std::memory_order_relaxed);
                    st.tailNodes.fetch_add(tn, std::memory_order_relaxed);
                    if (score >= 0) {
                        st.completions.fetch_add(1, std::memory_order_relaxed);
                        int prev = g_bestScore.load();
                        while (score > prev &&
                               !g_bestScore.compare_exchange_weak(prev, score)) {}
                        logf("TAIL-MRV COMPLETION: %d/480 thread %d seed %" PRIu64
                             " nodes %" PRIu64 "%s",
                             score, threadId, attemptSeed, tn,
                             score > prev ? "  *** NEW BEST ***" : "");
                        if (score >= cfg.minSave)
                            saveBoard(board, score, threadId, attemptSeed);
                        break;
                    }
                }
                continue;
            }
            tryNext[si] = 0;
            si--;
            if (si <= 0) break; // exhausted the whole tree under cell 0's corner
        }

        st.nodes.fetch_add(nodes & 0xFFFFF, std::memory_order_relaxed);
        st.placements.fetch_add(placements - lastFlushedPlacements,
                                std::memory_order_relaxed);
        st.restarts.fetch_add(1, std::memory_order_relaxed);
        st.depthHist[maxSi].fetch_add(1, std::memory_order_relaxed);
        if (cfg.endgameDepth) {
            int b = 0;
            while (b < 13 && attemptArrivals >= (uint64_t)(1u << b)) b++;
            st.attemptYield[b].fetch_add(1, std::memory_order_relaxed);
        }
        {   // cost funnel: deepest cell reached per remaining budget
            uint16_t fr = 0;
            for (int sp = 0; sp <= TOTAL_BUDGET && sp < 32; sp++) {
                if (depthAtSpend[sp] > fr) fr = depthAtSpend[sp];
                int slack = TOTAL_BUDGET - sp;
                if (slack > 5) continue;
                for (int k = 0; k < 4; k++)
                    if (fr >= FUNNEL_TH[k])
                        st.costFunnel[k][slack].fetch_add(1, std::memory_order_relaxed);
            }
        }
        int prev = st.maxDepthEver.load();
        while (maxSi > prev && !st.maxDepthEver.compare_exchange_weak(prev, maxSi)) {}
        if (snapped) {                     // pool telemetry: "depth hex64(mask@176)"
            char head[16];
            poolBuf.append(head, (size_t)snprintf(head, sizeof head, "%d ", maxSi));
            for (int b = 0; b < 32; b++) {
                char h[3];
                snprintf(h, sizeof h, "%02x", snapMask[b]);
                poolBuf.append(h, 2);
            }
            poolBuf += '\n';
            if (poolBuf.size() > 16384) {
                if (!poolF)
                    poolF = fopen((cfg.poolLog + ".t" + std::to_string(threadId)).c_str(), "a");
                if (poolF) { fwrite(poolBuf.data(), 1, poolBuf.size(), poolF); fflush(poolF); }
                poolBuf.clear();
            }
        }
    }
    if (!poolBuf.empty()) {
        if (!poolF)
            poolF = fopen((cfg.poolLog + ".t" + std::to_string(threadId)).c_str(), "a");
        if (poolF) fwrite(poolBuf.data(), 1, poolBuf.size(), poolF);
    }
    if (poolF) fclose(poolF);
}

// ------------------------------------------------------------------ heartbeat

static void heartbeat() {
    uint64_t lastNodes = 0, lastPlace = 0;
    auto last = std::chrono::steady_clock::now();
    while (!g_stop.load()) {
        for (int i = 0; i < 30 && !g_stop.load(); i++)
            std::this_thread::sleep_for(std::chrono::seconds(1));
        uint64_t n = 0, p = 0, r = 0, c = 0;
        int md = 0;
        // near-miss funnel: attempts whose max depth reached these thresholds
        static const int TH[6] = {240, 244, 248, 250, 252, 254};
        uint64_t fun[6] = {0, 0, 0, 0, 0, 0};
        uint64_t ac[4] = {}, ax[4] = {}, an[4] = {};
        uint64_t cf[4][6] = {};
        uint64_t eg[8][16] = {}, egCalls = 0, egCapped = 0, egNodes = 0;
        for (auto& s : g_stats) {
            for (int k = 0; k < 4; k++)
                for (int sl = 0; sl < 6; sl++) cf[k][sl] += s.costFunnel[k][sl].load();
            if (cfg.endgameDepth) {
                egCalls += s.endgameCalls.load();
                egCapped += s.endgameCapped.load();
                egNodes += s.endgameNodes.load();
                for (int sl = 0; sl < 8; sl++)
                    for (int c2 = 0; c2 < 16; c2++)
                        eg[sl][c2] += s.endgameGrid[sl][c2].load();
            }
            n += s.nodes.load();
            p += s.placements.load();
            r += s.restarts.load();
            c += s.completions.load();
            md = std::max(md, s.maxDepthEver.load());
            if (cfg.tailAdaptive) {
                for (int k = 0; k < 4; k++) {
                    ac[k] += s.adaptiveCalls[k].load();
                    ax[k] += s.adaptiveCapped[k].load();
                    an[k] += s.adaptiveNodes[k].load();
                }
            }
            for (int d = TH[0]; d <= 256; d++) {
                uint64_t v = s.depthHist[d].load();
                if (!v) continue;
                for (int k = 0; k < 6; k++)
                    if (d >= TH[k]) fun[k] += v;
            }
        }
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last).count();
        logf("nodes %.1fB (%.1fM/s) placements %.1fB (%.1fM/s) restarts %" PRIu64
             " completions %" PRIu64 " maxDepth %d best %d funnel"
             " %" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64
             " (≥240/244/248/250/252/254)",
             n / 1e9, (n - lastNodes) / dt / 1e6, p / 1e9,
             (p - lastPlace) / dt / 1e6, r, c, md, g_bestScore.load(),
             fun[0], fun[1], fun[2], fun[3], fun[4], fun[5]);
        // The event that precedes a completion under budget: arriving deep with
        // budget still in hand. Each group is slack 0/1/2/3/4/5+.
        logf("cost funnel d244 %" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64
             " d248 %" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64
             " d250 %" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64
             " d252 %" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64
             " (slack 0/1/2/3/4/5)",
             cf[0][0], cf[0][1], cf[0][2], cf[0][3], cf[0][4], cf[0][5],
             cf[1][0], cf[1][1], cf[1][2], cf[1][3], cf[1][4], cf[1][5],
             cf[2][0], cf[2][1], cf[2][2], cf[2][3], cf[2][4], cf[2][5],
             cf[3][0], cf[3][1], cf[3][2], cf[3][3], cf[3][4], cf[3][5]);
        if (cfg.endgameDepth) {
            uint64_t ay[14] = {}, ar[16] = {};
            for (auto& s : g_stats) {
                for (int b = 0; b < 14; b++) ay[b] += s.attemptYield[b].load();
                for (int b = 0; b < 16; b++) ar[b] += s.endgameRing[b].load();
            }
            std::string rh;
            for (int b = 0; b < 16; b++)
                if (ar[b]) rh += " " + std::to_string(b) + ":" + std::to_string(ar[b]);
            if (!rh.empty()) logf("  cheapest-completion ring mismatches:%s", rh.c_str());
            for (size_t rk = 0; rk < cfg.ladder.size(); rk++) {
                uint64_t rc = 0, rx = 0, rn = 0, rg[16] = {};
                for (auto& s3 : g_stats) {
                    rc += s3.escCalls[rk].load();
                    rx += s3.escCapped[rk].load();
                    rn += s3.escNodes[rk].load();
                    for (int b = 0; b < 16; b++) rg[b] += s3.escGapHist[rk][b].load();
                }
                if (!rc) continue;
                std::string gh;
                for (int b = 0; b < 16; b++)
                    if (rg[b]) gh += " +" + std::to_string(b) + ":" + std::to_string(rg[b]);
                logf("  ladder rung %zu (rewind %d): calls %" PRIu64 " capped %" PRIu64
                     " nodes %.1fM (%.0f/call) gap-to-budget%s", rk + 1,
                     cfg.ladder[rk].rewind, rc, rx, rn / 1e6,
                     rc ? (double)rn / rc : 0.0, gh.c_str());
            }
        }
        if (cfg.tailAdaptive)
            logf("adaptive calls %" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64
                 " capped %" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64
                 " nodes %.1fM/%.1fM/%.1fM/%.1fM (248/250/252/253)",
                 ac[0], ac[1], ac[2], ac[3], ax[0], ax[1], ax[2], ax[3],
                 an[0] / 1e6, an[1] / 1e6, an[2] / 1e6, an[3] / 1e6);
        lastNodes = n;
        lastPlace = p;
        last = now;
    }
}

// ----------------------------------------------------------------------- main

// heuristic ramp (Blackwood's 470-run array, scaled)
static void buildHeurArray() {
    for (int i = 0; i < 256; i++) {
        double v = 0;
        if (i <= 16) v = 0;
        else if (i <= 26) v = (i - 16) * 2.8;
        else if (i <= 56) v = (i - 26) * 1.43333 + 28;
        else if (i <= 76) v = (i - 56) * 0.9 + 71;
        else if (i <= 102) v = (i - 76) * 0.6538 + 89;
        else if (i <= MAX_HEUR_IDX) v = (i - 102) / 4.4615 + 106;
        HEUR_ARRAY[i] = (int)(v * cfg.heurScale * (HEUR_EDGE_TOTAL / 150.0));
    }
}

static void buildStaticArrays() {
    // fill order
    bool seen[256] = {false};
    for (int pr = 0; pr < 16; pr++) {
        for (int c = 0; c < 16; c++) {
            int fi = BOARD_ORDER[pr][c];
            int bur = 15 - pr;
            if (fi < 0 || fi > 255 || seen[fi]) die("bad BOARD_ORDER");
            seen[fi] = true;
            SEQ_ROW[fi] = (uint8_t)bur;
            SEQ_COL[fi] = (uint8_t)c;
            FILL_IDX[bur * 16 + c] = fi;
        }
    }
    // west & south neighbors must be filled before each cell
    for (int fi = 0; fi < 256; fi++) {
        int r = SEQ_ROW[fi], c = SEQ_COL[fi];
        if (c > 0 && FILL_IDX[r * 16 + c - 1] >= fi) die("fill order: west not before %d", fi);
        if (r > 0 && FILL_IDX[(r - 1) * 16 + c] >= fi) die("fill order: south not before %d", fi);
    }

    // break indices
    std::vector<int> breaks;
    {
        std::string s = cfg.breaks;
        for (char& ch : s) if (ch == ',') ch = ' ';
        char* end = nullptr;
        const char* q = s.c_str();
        while (*q) {
            long v = strtol(q, &end, 10);
            if (end == q) break;
            breaks.push_back((int)v);
            q = end;
        }
    }
    if (breaks.empty()) {
        // ZERO-BREAK exact mode: no budget ever releases — perfect placements
        // only, every prefix exhaustively refuted or completed. Target: 480.
        FIRST_BREAK_IDX = 256;
        memset(BREAK_ARRAY, 0, sizeof(BREAK_ARRAY));
        TOTAL_BUDGET = 0;
        logf("ZERO-BREAK exact mode: perfect placements only, target 480");
        buildHeurArray();
        return;
    }
    std::sort(breaks.begin(), breaks.end());
    FIRST_BREAK_IDX = breaks.front();
    if (FIRST_BREAK_IDX <= MAX_HEUR_IDX)
        die("first break index %d must exceed heuristic range %d", FIRST_BREAK_IDX, MAX_HEUR_IDX);
    int cum = 0, bi = 0;
    for (int i = 0; i < 256; i++) {
        while (bi < (int)breaks.size() && breaks[bi] == i) { cum++; bi++; }
        BREAK_ARRAY[i] = (uint8_t)cum;
        // reject duplicates / out-of-range
    }
    if (bi != (int)breaks.size()) die("break index out of range 0..255");
    // eligibility: not row0/col0/corners/clue cells. Those cells close one frame
    // side plus one lateral ring seam, so a release landing there is unspendable
    // — unless --ring-breaks has opened that cell's ring seam.
    for (int fi2 : breaks) {
        int r = SEQ_ROW[fi2], c = SEQ_COL[fi2];
        bool ringOpen = cfg.ringDepth && fi2 >= cfg.ringDepth;
        if (!ringOpen && (r == 0 || c == 0 || (r == 15 && (c == 0 || c == 15))))
            die("break index %d targets a no-break cell (bu r%d c%d)", fi2, r, c);
        for (const Hint& h : HINTS)
            if (15 - h.tdRow == r && h.tdCol == c)
                die("break index %d targets clue cell", fi2);
    }
    TOTAL_BUDGET = cfg.tailTargetScore >= 0 ? 480 - cfg.tailTargetScore
                                            : (int)BREAK_ARRAY[255];
    logf("breaks (%zu total, target completion score %d): %s",
         breaks.size(), 480 - (int)breaks.size(), cfg.breaks.c_str());
    buildHeurArray();
}

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr,
                "usage: %s <pieces.txt> <hints.txt> [--threads N] [--seconds S] "
                "[--seed X] [--breaks \"i,j,...\"] [--node-cap N] [--heur-scale F] "
                "[--min-save S] [--save-dir D] [--prune NODES:DEPTH] "
                "[--tail-mrv DEPTH:NODES] [--tail-target-score SCORE] "
                "[--tail-period N] [--tail-adaptive] [--ring-breaks DEPTH] "
                "[--tail-min-slack S] [--exact-endgame DEPTH:NODES] [--endgame-probe E] "
                "[--bench]\n",
                argv[0]);
        return 1;
    }
    cfg.piecesFile = argv[1];
    cfg.hintsFile = argv[2];
    for (int i = 3; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&]() -> const char* {
            if (++i >= argc) die("missing value for %s", a.c_str());
            return argv[i];
        };
        if (a == "--threads") cfg.threads = atoi(next());
        else if (a == "--seconds") cfg.seconds = atol(next());
        else if (a == "--seed") cfg.seed = strtoull(next(), nullptr, 10);
        else if (a == "--breaks") { cfg.breaks = next(); cfg.breaksSet = true; }
        else if (a == "--node-cap") cfg.nodeCap = (uint64_t)atof(next());
        else if (a == "--heur-scale") { cfg.heurScale = atof(next()); cfg.heurSet = true; }
        else if (a == "--min-save") cfg.minSave = atoi(next());
        else if (a == "--save-dir") cfg.saveDir = next();
        else if (a == "--prune") {
            const char* v = next();
            cfg.pruneNodes = (uint64_t)atof(v);
            const char* colon = strchr(v, ':');
            if (!colon) die("--prune wants NODES:DEPTH");
            cfg.pruneDepth = atoi(colon + 1);
        } else if (a == "--bench") { cfg.bench = true; }
        else if (a == "--stats") { cfg.stats = true; }
        else if (a == "--pool-log") cfg.poolLog = next();
        else if (a == "--heur-colors") cfg.heurColors = next();
        else if (a == "--sd-prune") cfg.sdPrune = true;
        else if (a == "--isolated-breaks") cfg.isolatedBreaks = true;
        else if (a == "--double-breaks") cfg.cellBreaks = 2;
        else if (a == "--fill-order") cfg.fillOrder = next();
        else if (a == "--tail-mrv") {
            const char* v = next();
            const char* colon = strchr(v, ':');
            if (!colon) die("--tail-mrv wants DEPTH:NODES");
            cfg.tailDepth = atoi(v);
            cfg.tailNodes = (uint64_t)atof(colon + 1);
        }
        else if (a == "--tail-target-score") cfg.tailTargetScore = atoi(next());
        else if (a == "--tail-period") cfg.tailPeriod = (uint64_t)atof(next());
        else if (a == "--tail-adaptive") cfg.tailAdaptive = true;
        else if (a == "--ring-breaks") cfg.ringDepth = atoi(next());
        else if (a == "--tail-min-slack") cfg.tailMinSlack = atoi(next());
        else if (a == "--exact-endgame") {
            const char* v = next();
            const char* colon = strchr(v, ':');
            if (!colon) die("--exact-endgame wants DEPTH:NODES");
            cfg.endgameDepth = atoi(v);
            cfg.endgameNodes = (uint64_t)atof(colon + 1);
        }
        else if (a == "--endgame-probe") cfg.endgameProbeExtra = atoi(next());
        else if (a == "--endgame-sample") cfg.endgameSample = atoi(next());
        else if (a == "--endgame-ladder") {
            std::string spec = next();
            for (char& ch : spec) if (ch == ',') ch = ' ';
            const char* q = spec.c_str();
            while (*q) {
                while (*q == ' ') q++;
                if (!*q) break;
                int g = 0, rw = 0; double nd = 0;
                if (sscanf(q, "%d:%d:%lf", &g, &rw, &nd) != 3)
                    die("--endgame-ladder wants GAP:REWIND:NODES[,GAP:REWIND:NODES...]");
                cfg.ladder.push_back({g, rw, (uint64_t)nd});
                while (*q && *q != ' ') q++;
            }
        }
        else if (a == "--endgame-escalate" || a == "--endgame-escalate2") {
            const char* v = next();
            int g = 0, rw = 0; double nd = 0;
            if (sscanf(v, "%d:%d:%lf", &g, &rw, &nd) != 3)
                die("%s wants GAP:REWIND:NODES", a.c_str());
            cfg.ladder.push_back({g, rw, (uint64_t)nd});
        }
        else if (a == "--yield-budget") {
            const char* v = next();
            const char* colon = strchr(v, ':');
            if (!colon) die("--yield-budget wants BASE:PER");
            cfg.yieldBase = (uint64_t)atof(v);
            cfg.yieldPer = (uint64_t)atof(colon + 1);
        }
        else die("unknown arg %s", a.c_str());
    }
    if (cfg.bench) { cfg.threads = 1; if (!cfg.seconds) cfg.seconds = 20; }

    if (cfg.isolatedBreaks && cfg.cellBreaks > 1)
        die("--isolated-breaks and --double-breaks are mutually exclusive");
    if (cfg.tailDepth && (cfg.tailDepth < 1 || cfg.tailDepth > 255 || !cfg.tailNodes))
        die("--tail-mrv depth must be 1..255 and nodes must be positive");
    if (cfg.tailTargetScore < -1 || cfg.tailTargetScore > 480)
        die("--tail-target-score must be 0..480");
    if (cfg.tailTargetScore >= 0 && !cfg.tailDepth)
        die("--tail-target-score requires --tail-mrv");
    if (cfg.tailAdaptive && !cfg.tailDepth)
        die("--tail-adaptive requires --tail-mrv");
    if (cfg.ringDepth && (cfg.ringDepth <= MAX_HEUR_IDX || cfg.ringDepth > 255))
        die("--ring-breaks depth must be %d..255 (the exhaustion ramp owns the "
            "earlier cells)", MAX_HEUR_IDX + 1);
    if (cfg.tailMinSlack < 0 || cfg.tailMinSlack > 20)
        die("--tail-min-slack must be 0..20");
    if (cfg.endgameDepth &&
        (cfg.endgameDepth < 200 || cfg.endgameDepth > 255 || !cfg.endgameNodes))
        die("--exact-endgame depth must be 200..255 with a positive node cap");
    if (cfg.endgameProbeExtra < 0 || cfg.endgameProbeExtra > 20)
        die("--endgame-probe must be 0..20");
    if (cfg.endgameProbeExtra && !cfg.endgameDepth)
        die("--endgame-probe requires --exact-endgame");
    if (cfg.endgameSample < 0) die("--endgame-sample must be >= 0");
    if (cfg.endgameSample && !cfg.endgameProbeExtra)
        die("--endgame-sample requires --endgame-probe");
    if (!cfg.ladder.empty()) {
        if (!cfg.endgameDepth) die("--endgame-ladder requires --exact-endgame");
        if ((int)cfg.ladder.size() > ThreadStats::MAX_RUNGS)
            die("--endgame-ladder supports at most %d rungs", ThreadStats::MAX_RUNGS);
        if (cfg.endgameSample)
            die("--endgame-ladder and --endgame-sample are mutually exclusive");
        int prevRewind = cfg.endgameDepth;
        for (const Config::EscRung& r : cfg.ladder) {
            if (cfg.endgameProbeExtra < r.gap)
                die("ladder rung gap %d needs --endgame-probe >= %d so the cheaper "
                    "pass can see how close it came", r.gap, r.gap);
            if (r.rewind < 180 || r.rewind >= prevRewind || !r.nodes)
                die("ladder rewinds must strictly deepen (180..%d) with a positive "
                    "node cap", prevRewind - 1);
            prevRewind = r.rewind;
        }
    }
    mkdir(cfg.saveDir.c_str(), 0755);
    g_log = fopen((cfg.saveDir + "/bw.log").c_str(), "a");

    loadPieces(cfg.piecesFile);
    if (!cfg.heurColors.empty()) {
        HEUR_COLORS.clear();
        std::string s = cfg.heurColors;
        for (char& ch : s) if (ch == ',') ch = ' ';
        const char* q = s.c_str();
        char* end = nullptr;
        while (*q) {
            long v = strtol(q, &end, 10);
            if (end == q) break;
            if (v < 1 || v >= NCOLORS) die("bad heuristic color %ld", v);
            HEUR_COLORS.push_back((int)v);
            q = end;
        }
        if (HEUR_COLORS.empty()) die("no heuristic colors parsed");
    }
    computeHsc();
    loadHints(cfg.hintsFile);
    applyClueMode();
    buildStaticArrays();
    if (cfg.sdPrune) logf("supply/demand prune: ON");
    if (cfg.tailDepth) {
        if (cfg.tailTargetScore >= 0)
            logf("tail MRV: depth %d (%d cells), node cap %.1fM, period %.1fM hot "
                 "nodes, target %d, official-score seams", cfg.tailDepth,
                 256 - cfg.tailDepth, cfg.tailNodes / 1e6,
                 cfg.tailPeriod / 1e6, cfg.tailTargetScore);
        else
            logf("tail MRV: depth %d (%d cells), node cap %.1fM, period %.1fM hot "
                 "nodes, target generator-budget, official-score seams", cfg.tailDepth,
                 256 - cfg.tailDepth, cfg.tailNodes / 1e6,
                 cfg.tailPeriod / 1e6);
    }
    if (cfg.tailAdaptive)
        logf("adaptive MRV: %d->%d@%.0fM, %d->%d@%.0fM, "
             "%d->%d@%.0fM, %d->%d@%.0fM",
             ADAPTIVE_TAIL_TIERS[0].triggerDepth, ADAPTIVE_TAIL_TIERS[0].rewindDepth,
             ADAPTIVE_TAIL_TIERS[0].nodeCap / 1e6,
             ADAPTIVE_TAIL_TIERS[1].triggerDepth, ADAPTIVE_TAIL_TIERS[1].rewindDepth,
             ADAPTIVE_TAIL_TIERS[1].nodeCap / 1e6,
             ADAPTIVE_TAIL_TIERS[2].triggerDepth, ADAPTIVE_TAIL_TIERS[2].rewindDepth,
             ADAPTIVE_TAIL_TIERS[2].nodeCap / 1e6,
             ADAPTIVE_TAIL_TIERS[3].triggerDepth, ADAPTIVE_TAIL_TIERS[3].rewindDepth,
             ADAPTIVE_TAIL_TIERS[3].nodeCap / 1e6);

    // resume best score
    {
        FILE* f = fopen((cfg.saveDir + "/best_score.txt").c_str(), "r");
        if (f) {
            int b = 0;
            if (fscanf(f, "%d", &b) == 1) g_bestScore = b;
            fclose(f);
        }
    }

    logf("bw starting: %d threads, seed %" PRIu64 ", nodeCap %.1fB, heurScale %.3f, "
         "minSave %d, resume best %d, seconds %ld%s",
         cfg.threads, cfg.seed, cfg.nodeCap / 1e9, cfg.heurScale, cfg.minSave,
         g_bestScore.load(), cfg.seconds, cfg.bench ? " [BENCH]" : "");
    for (const Hint& h : HINTS)
        logf("clue: piece %d td(%d,%d) rot %d -> bu cell (%d,%d) fill idx %d",
             h.piece, h.tdRow, h.tdCol, h.rot, 15 - h.tdRow, h.tdCol,
             FILL_IDX[(15 - h.tdRow) * 16 + h.tdCol]);

    signal(SIGINT, [](int) { g_stop = true; });
    signal(SIGTERM, [](int) { g_stop = true; });

    g_stats = std::vector<ThreadStats>(cfg.threads);
    std::vector<std::thread> threads;
    for (int i = 0; i < cfg.threads; i++) threads.emplace_back(solverThread, i);
    std::thread hb(heartbeat);
    for (auto& t : threads) t.join();
    g_stop = true;
    hb.join();

    // final report
    uint64_t n = 0, p = 0, r = 0, c = 0, tc = 0, tn = 0;
    uint64_t ac[4] = {}, an[4] = {}, ax[4] = {};
    for (auto& s : g_stats) {
        n += s.nodes.load();
        p += s.placements.load();
        r += s.restarts.load();
        c += s.completions.load();
        tc += s.tailCalls.load();
        tn += s.tailNodes.load();
        for (int k = 0; k < 4; k++) {
            ac[k] += s.adaptiveCalls[k].load();
            an[k] += s.adaptiveNodes[k].load();
            ax[k] += s.adaptiveCapped[k].load();
        }
    }
    uint64_t hist[257] = {0};
    for (auto& s : g_stats)
        for (int d = 0; d < 257; d++) hist[d] += s.depthHist[d].load();
    logf("FINAL: nodes %.2fB placements %.2fB restarts %" PRIu64 " completions %" PRIu64
         " best %d", n / 1e9, p / 1e9, r, c, g_bestScore.load());
    if (cfg.tailDepth)
        logf("tail MRV FINAL: calls %" PRIu64 " nodes %.2fM (%.0f/call)",
             tc, tn / 1e6, tc ? (double)tn / tc : 0.0);
    if (cfg.tailAdaptive) {
        for (int k = 0; k < 4; k++) {
            const AdaptiveTailTier& at = ADAPTIVE_TAIL_TIERS[k];
            logf("adaptive MRV %d->%d FINAL: calls %" PRIu64
                 " capped %" PRIu64 " nodes %.2fM (%.0f/call)",
                 at.triggerDepth, at.rewindDepth, ac[k], ax[k], an[k] / 1e6,
                 ac[k] ? (double)an[k] / ac[k] : 0.0);
        }
    }
    // depth distribution of attempts (only non-zero buckets)
    std::string dist = "attempt max-depth histogram:";
    for (int d = 0; d < 257; d++)
        if (hist[d]) dist += " " + std::to_string(d) + ":" + std::to_string(hist[d]);
    logf("%s", dist.c_str());
    if (cfg.stats) {
        std::string dn = "nodes per depth (nonzero):";
        std::string df = "fits per depth:";
        std::string dh = "halffits per depth:";
        for (int d = 0; d < 257; d++) {
            uint64_t v = 0, f = 0, h2 = 0;
            for (auto& s : g_stats) {
                v += s.depthNodes[d];
                f += s.depthFits[d];
                h2 += s.depthHalf[d];
            }
            if (v) dn += " " + std::to_string(d) + ":" + std::to_string(v);
            if (f) df += " " + std::to_string(d) + ":" + std::to_string(f);
            if (h2) dh += " " + std::to_string(d) + ":" + std::to_string(h2);
        }
        // logf truncates at 2KB — write the full arrays directly
        for (const std::string* s2 : {&dn, &df, &dh}) {
            printf("%s\n", s2->c_str());
            if (g_log) fprintf(g_log, "%s\n", s2->c_str());
        }
        fflush(stdout);
        if (g_log) fflush(g_log);
    }
    if (g_log) fclose(g_log);
    return 0;
}
