#!/usr/bin/env python3
"""Validate a saved board file and transform it to the e2.bucas.name URL format
(Jef Bucas's JS viewer: board_edges/board_pieces/board_types parameters).

Reimplements the viewer's own logic from its puzzle JS:
  - board_edges: 4 chars per space, edge order UP,RIGHT,DOWN,LEFT, 'a'=motif 0,
    spaces in reading order (space s = x + y*w, y=0 = top row).
  - initBoardConflicts(): counts mismatched internal seams -> score = 480 - conflicts.
  - piece verification: canonical (min) rotation of each space's 4-char edge string,
    sorted board-wide -> must equal the constant for the piece set.
  - center clue check: board_edges.substr(135*4, 4) must be "sggl" (jblackwood order).
  - board_types: a:None b:Corner c:Border d:Center e:Fixed (clues marked 'e').
  - board_pieces: 3-digit piece number per space (we OMIT it by default: the viewer
    derives piece numbers from its own table; our file has pieceIDs 112/113 swapped
    vs Blackwood's table, so supplying ours could disagree with the viewer's numbers
    for those two pieces).

Usage: python3 to_bucas.py <board_file> [--with-pieces]
"""
import sys
from pathlib import Path

HERE = Path(__file__).parent
W = H = 16
HINTS = [(139, 8, 7, 2), (181, 13, 2, 3), (208, 2, 2, 3), (249, 13, 13, 0), (255, 2, 13, 3)]


def load_pieces():
    pieces = {}
    for i, line in enumerate((HERE.parent / "256pieces.txt").read_text().split("\n")):
        if not line.strip():
            continue
        t, b, l, r = map(int, line.split())
        pieces[i + 1] = (t, r, b, l)  # clockwise cycle (N,E,S,W)
    assert len(pieces) == 256
    return pieces


def side(pieces, pid, rot, face):  # face 0=N 1=E 2=S 3=W, rot = CW quarter turns
    return pieces[pid][(face - rot) % 4]


def load_board(path):
    toks = []
    for line in Path(path).read_text().split("\n"):
        if line.startswith("#") or not line.strip():
            continue
        toks += line.split()
    assert len(toks) == 256, f"expected 256 cells, got {len(toks)}"
    board = {}
    for idx, tok in enumerate(toks):
        pid, rot = map(int, tok.split("/"))
        board[(idx // 16, idx % 16)] = (pid, rot)  # td coords, row 0 = top
    return board


def validate(pieces, board):
    errors = []
    ids = [pr[0] for pr in board.values()]
    if sorted(ids) != list(range(1, 257)):
        errors.append("piece set is not exactly 1..256 used once")
    for pid, r, c, rot in HINTS:
        if board[(r, c)] != (pid, rot):
            errors.append(f"clue {pid} not at td({r},{c}) rot {rot}: found {board[(r,c)]}")
    matched, mm = 0, []
    for r in range(H):
        for c in range(W):
            pid, rot = board[(r, c)]
            if r == 0 and side(pieces, pid, rot, 0) != 0: errors.append(f"frame N td({r},{c})")
            if r == 15 and side(pieces, pid, rot, 2) != 0: errors.append(f"frame S td({r},{c})")
            if c == 0 and side(pieces, pid, rot, 3) != 0: errors.append(f"frame W td({r},{c})")
            if c == 15 and side(pieces, pid, rot, 1) != 0: errors.append(f"frame E td({r},{c})")
            if c < 15:
                q, qr = board[(r, c + 1)]
                if side(pieces, pid, rot, 1) == side(pieces, q, qr, 3): matched += 1
                else: mm.append(f"H({r},{c})")
            if r < 15:
                q, qr = board[(r + 1, c)]
                if side(pieces, pid, rot, 2) == side(pieces, q, qr, 0): matched += 1
                else: mm.append(f"V({r},{c})")
    return matched, mm, errors


def board_edges_string(pieces, board, mapfn=lambda c: chr(ord("a") + c)):
    # viewer space s = x + y*w with y=0 top; edge order UP,RIGHT,DOWN,LEFT
    out = []
    for r in range(H):
        for c in range(W):
            pid, rot = board[(r, c)]
            for face in (0, 1, 2, 3):  # N,E,S,W == UP,RIGHT,DOWN,LEFT
                out.append(mapfn(side(pieces, pid, rot, face)))
    return "".join(out)


def jef_letter(c):
    # internal color -> official Eternity II `jef` motif letter. Piecewise
    # reflection, derived+verified by aligning the 5 forced clues against the
    # official Clues board_edges (rgou/rtrj/vddo/jdso/fskn).
    if c == 0:      i = 0
    elif c <= 8:    i = 9 - c
    elif c <= 16:   i = 25 - c
    else:           i = 39 - c   # 17..22
    return chr(ord("a") + i)


def js_conflicts(edges):
    """Exact reimplementation of the viewer's initBoardConflicts()."""
    EDGE_UP, EDGE_RIGHT, EDGE_DOWN, EDGE_LEFT = 0, 1, 2, 3
    e = [ord(ch) - ord("a") for ch in edges]
    nb = 0
    for y in range(H):
        for x in range(W):
            s = x + y * W
            if x != W - 1:
                if e[s * 4 + EDGE_RIGHT] != e[(s + 1) * 4 + EDGE_LEFT]: nb += 1
                elif e[s * 4 + EDGE_RIGHT] == 0: nb += 1
            if y != H - 1:
                if e[s * 4 + EDGE_DOWN] != e[(s + W) * 4 + EDGE_UP]: nb += 1
                elif e[s * 4 + EDGE_DOWN] == 0: nb += 1
    return nb


def js_piece_verify(edges):
    """The viewer's canonical-rotation sort (board_verify string)."""
    canon = []
    for s in range(W * H):
        r0 = edges[s * 4 : s * 4 + 4]
        rots = [r0]
        for _ in range(3):
            rots.append(rots[-1][3] + rots[-1][:3])
        canon.append(min(rots))
    canon.sort()
    return ",".join(canon)


def board_types_string(board):
    types = []
    for r in range(H):
        for c in range(W):
            if any((hr, hc) == (r, c) for _, hr, hc, _ in HINTS): t = "e"  # Fixed
            elif (r in (0, 15)) and (c in (0, 15)): t = "b"                # Corner
            elif r in (0, 15) or c in (0, 15): t = "c"                     # Border
            else: t = "d"                                                  # Center
            types.append(t)
    return "".join(types)


def jef_url(pieces, board):
    """Official Eternity II `Clues`/`jef` viewer URL (matches the canonical
    5-clue reference; renders with the standard E2 motif art)."""
    edges = board_edges_string(pieces, board, jef_letter)
    ids = "".join(f"{board[(r, c)][0]:03d}" for r in range(H) for c in range(W))
    return ("https://e2.bucas.name/#puzzle=Clues&board_w=16&board_h=16"
            f"&board_edges={edges}&board_pieces={ids}&motifs_order=jef")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = {a for a in sys.argv[1:] if a.startswith("--")}
    if not args:
        sys.exit("usage: to_bucas.py <board_file...> [--with-pieces] [--jblackwood] [--quiet]")
    pieces = load_pieces()
    ref = js_piece_verify(board_edges_string(pieces, {  # full E2 set, identity layout
        (i // 16, i % 16): (i + 1, 0) for i in range(256)}))

    for path in args:
        board = load_board(path)
        matched, mm, errors = validate(pieces, board)
        edges = board_edges_string(pieces, board)  # jblackwood-order for the JS checks
        clue = edges[135 * 4 : 135 * 4 + 4]
        ok = (not errors
              and 480 - js_conflicts(edges) == matched
              and clue in ("sggl", "vddo", "ijjm")
              and js_piece_verify(edges) == ref)

        if "--quiet" not in flags:
            print(f"== {Path(path).name}")
            print(f"   matched edges : {matched}/480  ({480-matched} mismatches)")
            print(f"   valid board   : {'OK' if ok else 'PROBLEM'}"
                  + (f"  {errors}" if errors else ""))
            if mm:
                print(f"   mismatch seams: {' '.join(mm)}")

        print(jef_url(pieces, board))
        if "--jblackwood" in flags:
            url = ("https://e2.bucas.name/#puzzle=Eternity_II&board_w=16&board_h=16"
                   f"&board_edges={edges}&board_types={board_types_string(board)}")
            if "--with-pieces" in flags:
                url += "&board_pieces=" + "".join(
                    f"{board[(r,c)][0]:03d}" for r in range(H) for c in range(W))
            print(url + "&motifs_order=jblackwood")
        print()


if __name__ == "__main__":
    main()
