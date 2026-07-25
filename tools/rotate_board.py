#!/usr/bin/env python3
"""Rotate a saved td-format board by clockwise quarter-turns.

Piece rotations advance with the board rotation.  This is used to convert
solutions from the rotational fill-order portfolio back to the official clue
coordinates without reflecting (flipping) any piece.
"""

from __future__ import annotations

import argparse
from pathlib import Path


def load_board(path: Path) -> list[tuple[int, int]]:
    tokens: list[str] = []
    for line in path.read_text().splitlines():
        if not line.startswith("#"):
            tokens.extend(line.split())
    if len(tokens) != 256:
        raise SystemExit(f"{path}: expected 256 placements, got {len(tokens)}")
    board: list[tuple[int, int]] = []
    for token in tokens:
        piece, rotation = token.split("/", 1)
        board.append((int(piece), int(rotation)))
    return board


def rotate(board: list[tuple[int, int]], turns: int) -> list[tuple[int, int]]:
    turns %= 4
    result = board[:]
    for _ in range(turns):
        nxt: list[tuple[int, int]] = [(0, 0)] * 256
        for row in range(16):
            for col in range(16):
                piece, rotation = result[row * 16 + col]
                new_row, new_col = col, 15 - row
                nxt[new_row * 16 + new_col] = (piece, (rotation + 1) % 4)
        result = nxt
    return result


def check_hints(board: list[tuple[int, int]], path: Path) -> None:
    checked = 0
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        piece, row, col, rotation = map(int, line.split())
        actual = board[row * 16 + col]
        if actual != (piece, rotation):
            raise SystemExit(
                f"{path}: clue {piece} at ({row},{col})/{rotation}, found {actual}"
            )
        checked += 1
    if checked != 5:
        raise SystemExit(f"{path}: expected five clues, checked {checked}")


def format_board(board: list[tuple[int, int]]) -> str:
    rows = []
    for row in range(16):
        rows.append(" ".join(f"{piece}/{rotation}" for piece, rotation in board[row * 16 : (row + 1) * 16]))
    return "\n".join(rows) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("board", type=Path)
    parser.add_argument("turns", type=int, help="clockwise quarter-turns")
    parser.add_argument("output", type=Path)
    parser.add_argument("--hints", type=Path, help="check the rotated placements against this hint file")
    args = parser.parse_args()

    result = rotate(load_board(args.board), args.turns)
    if args.hints:
        check_hints(result, args.hints)
    args.output.write_text(format_board(result))


if __name__ == "__main__":
    main()
