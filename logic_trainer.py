#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GateLab - Interactive Logic Gate, K-Map & Memory Trainer Board
================================================================
Single-file PyQt6 + PySerial desktop application.

Features
  * Landing page + live console (dark instrument UI, Simulithic-inspired accents)
  * Asynchronous serial engine (QThread) @ 115200 8-N-1, JSON input frames in,
    "CMD:<L0>,<L1>,<L2>,<L3>:<LCD1>:<LCD2>\\n" frames out
  * Native Quine-McCluskey + Petrick minimiser (no external boolean libraries)
  * Full adder / subtractor, 74138 decoder, 74153 dual mux simulators
  * 4-bit shift register (SISO / SIPO / PISO / PIPO) built from 7474 D-flip-flops
  * QPainter canvases: interactive K-Map, auto-synthesised schematic,
    register timing diagram, device (IC) view

Requires:  pip install PyQt6 pyserial
Run:       python logic_trainer.py
"""

from __future__ import annotations

import json
import math
import random
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, Dict, Iterable, List, Optional, Sequence, Set, Tuple

import serial
import serial.tools.list_ports
from PyQt6.QtCore import (
    QEasingCurve, QPointF, QPropertyAnimation, QRectF, QSettings, QSize, Qt, QThread, QTimer,
    QVariantAnimation, pyqtSignal,
)
from PyQt6.QtGui import (
    QColor, QFont, QLinearGradient, QPainter, QPainterPath,
    QPen, QPolygonF, QRadialGradient, QTransform, QFontMetrics,
)
from PyQt6.QtWidgets import (
    QAbstractItemView, QApplication, QButtonGroup, QCheckBox, QComboBox, QDialog,
    QDoubleSpinBox, QFrame, QGraphicsOpacityEffect, QHBoxLayout, QHeaderView, QLabel,
    QLineEdit, QListWidget, QListWidgetItem, QMainWindow, QPlainTextEdit,
    QPushButton, QScrollArea, QSizePolicy, QSpinBox, QSplitter, QStackedWidget, QTabWidget,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

APP_NAME = "GateLab"
APP_TAGLINE = "Logic Gate, K-Map & Memory Trainer"
BAUD_RATE = 115200
VARS = "ABCD"
GRAY = (0, 1, 3, 2)          # Gray-code order of 2-bit values: 00, 01, 11, 10
GRAY_LABELS = ("00", "01", "11", "10")


# ======================================================================================
#  THEME
# ======================================================================================
class Theme:
    # Landing page (Simulithic-style light card on navy)
    NAVY = "#0B1F3A"
    PAPER = "#FFFFFF"
    PAPER_2 = "#F6F8FB"
    INK = "#0F1720"
    INK_2 = "#4B5563"
    LINE = "#E3E8EF"
    BLUE = "#0A84FF"
    BLUE_D = "#0066D6"
    # Console (dark instrument)
    GRAPHITE = "#1E1E1E"
    CHARCOAL = "#252526"
    PANEL = "#2A2A2C"
    BORDER = "#3C3C3F"
    TEXT = "#E6E6E6"
    MUTED = "#9DA1A6"
    CYAN = "#007ACC"
    CYAN_L = "#3AA0FF"
    EMERALD = "#2ECC71"
    AMBER = "#F5B041"
    RED = "#E74C3C"
    LIVE = "#00FF66"
    DEAD = "#444444"
    MONO = "Consolas"
    SANS = "Segoe UI"

    GROUP_COLORS = ["#3AA0FF", "#2ECC71", "#F5B041", "#E74C3C", "#B388FF", "#26C6DA",
                    "#FF7043", "#EC407A", "#9CCC65", "#FFD54F"]


def mono(size: int = 10, bold: bool = False) -> QFont:
    f = QFont(Theme.MONO, size)
    f.setStyleHint(QFont.StyleHint.Monospace)
    f.setBold(bold)
    return f


def sans(size: int = 10, bold: bool = False, weight: Optional[int] = None) -> QFont:
    f = QFont(Theme.SANS, size)
    f.setBold(bold)
    if weight is not None:
        f.setWeight(QFont.Weight(weight))
    return f


def qc(hex_or_color, alpha: Optional[int] = None) -> QColor:
    c = QColor(hex_or_color)
    if alpha is not None:
        c.setAlpha(alpha)
    return c


CONSOLE_QSS = f"""
* {{ font-family: '{Theme.SANS}'; font-size: 10pt; color: {Theme.TEXT}; }}
QMainWindow, QWidget#console {{ background: {Theme.GRAPHITE}; }}
QWidget#topbar {{ background: {Theme.CHARCOAL}; border-bottom: 1px solid {Theme.BORDER}; }}
QWidget#modebar {{ background: {Theme.GRAPHITE}; border-bottom: 1px solid {Theme.BORDER}; }}
QFrame#card {{ background: {Theme.CHARCOAL}; border: 1px solid {Theme.BORDER}; border-radius: 10px; }}
QLabel {{ background: transparent; }}
QLabel#h {{ font-size: 8pt; font-weight: 600; color: {Theme.MUTED}; letter-spacing: 1px; }}
QLabel#mono {{ font-family: '{Theme.MONO}'; }}
QLabel#expr {{ font-family: '{Theme.MONO}'; font-size: 15pt; font-weight: 600; color: {Theme.EMERALD}; }}
QLabel#pill {{ background: {Theme.GRAPHITE}; border: 1px solid {Theme.BORDER}; border-radius: 10px;
              padding: 2px 8px; font-size: 8.5pt; color: {Theme.MUTED}; }}
QPushButton {{ background: {Theme.PANEL}; border: 1px solid {Theme.BORDER}; border-radius: 6px;
              padding: 5px 12px; }}
QPushButton:hover {{ border-color: {Theme.CYAN_L}; background: #323236; }}
QPushButton:pressed {{ background: {Theme.CYAN}; }}
QPushButton:disabled {{ color: #6b6b6b; border-color: #333; }}
QPushButton#primary {{ background: {Theme.BLUE}; border: 1px solid {Theme.BLUE}; color: white; font-weight: 600; }}
QPushButton#primary:hover {{ background: #2E96FF; }}
QPushButton#danger {{ background: #5a2320; border-color: {Theme.RED}; color: #ffd9d5; }}
QPushButton#mode {{ background: transparent; border: 1px solid transparent; border-radius: 14px;
                   padding: 5px 16px; color: {Theme.MUTED}; font-weight: 600; letter-spacing: 0.5px; }}
QPushButton#mode:hover {{ color: {Theme.TEXT}; background: #2d2d30; }}
QPushButton#mode:checked {{ background: {Theme.CYAN}; color: white; border-color: {Theme.CYAN_L}; }}
QPushButton#ghost {{ background: transparent; border: 1px solid {Theme.BORDER}; }}
QPushButton#seg {{ border-radius: 0px; padding: 4px 10px; }}
QPushButton#seg:checked {{ background: {Theme.CYAN}; color: white; }}
QComboBox, QLineEdit, QSpinBox, QDoubleSpinBox {{ background: {Theme.GRAPHITE}; border: 1px solid {Theme.BORDER};
              border-radius: 6px; padding: 4px 8px; selection-background-color: {Theme.CYAN}; }}
QComboBox:hover, QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {{ border-color: {Theme.CYAN_L}; }}
QComboBox QAbstractItemView {{ background: {Theme.CHARCOAL}; border: 1px solid {Theme.BORDER};
              selection-background-color: {Theme.CYAN}; outline: 0; }}
QComboBox::drop-down {{ border: none; width: 18px; }}
QLineEdit#mono {{ font-family: '{Theme.MONO}'; }}
QCheckBox {{ spacing: 8px; }}
QCheckBox::indicator {{ width: 16px; height: 16px; border-radius: 4px; border: 1px solid {Theme.BORDER}; background: {Theme.GRAPHITE}; }}
QCheckBox::indicator:checked {{ background: {Theme.CYAN}; border-color: {Theme.CYAN_L}; }}
QTabWidget::pane {{ border: 1px solid {Theme.BORDER}; border-radius: 10px; background: {Theme.CHARCOAL}; top: -1px; }}
QTabBar::tab {{ background: transparent; color: {Theme.MUTED}; padding: 8px 18px; margin-right: 2px;
               border-top-left-radius: 8px; border-top-right-radius: 8px; font-weight: 600; }}
QTabBar::tab:selected {{ color: white; background: {Theme.CHARCOAL}; border: 1px solid {Theme.BORDER}; border-bottom-color: {Theme.CHARCOAL}; }}
QTabBar::tab:hover:!selected {{ color: {Theme.TEXT}; }}
QTableWidget {{ background: {Theme.GRAPHITE}; border: 1px solid {Theme.BORDER}; border-radius: 8px;
               gridline-color: #333336; font-family: '{Theme.MONO}'; selection-background-color: transparent; }}
QHeaderView::section {{ background: {Theme.PANEL}; color: {Theme.MUTED}; border: none; border-bottom: 1px solid {Theme.BORDER};
               padding: 4px; font-weight: 600; }}
QListWidget {{ background: {Theme.GRAPHITE}; border: 1px solid {Theme.BORDER}; border-radius: 8px;
              font-family: '{Theme.MONO}'; outline: 0; }}
QListWidget::item {{ padding: 4px 6px; }}
QListWidget::item:selected {{ background: #20405e; }}
QPlainTextEdit {{ background: #171718; border: 1px solid {Theme.BORDER}; border-radius: 8px;
                 font-family: '{Theme.MONO}'; font-size: 9pt; }}
QScrollArea {{ border: none; background: transparent; }}
QScrollBar:vertical {{ background: transparent; width: 10px; margin: 0; }}
QScrollBar::handle:vertical {{ background: #44444a; border-radius: 5px; min-height: 30px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; width: 0; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; }}
QScrollBar::handle:horizontal {{ background: #44444a; border-radius: 5px; }}
QSplitter::handle {{ background: {Theme.GRAPHITE}; }}
QStatusBar {{ background: {Theme.CHARCOAL}; border-top: 1px solid {Theme.BORDER}; }}
QStatusBar::item {{ border: none; }}
QToolTip {{ background: {Theme.CHARCOAL}; color: {Theme.TEXT}; border: 1px solid {Theme.BORDER}; padding: 4px; }}
"""


# ======================================================================================
#  CORE ALGORITHM A - QUINE-McCLUSKEY + PETRICK
# ======================================================================================
def cell_to_minterm(row: int, col: int) -> int:
    """K-map cell (row=AB gray index, col=CD gray index) -> minterm index (A=MSB)."""
    return (GRAY[row % 4] << 2) | GRAY[col % 4]


def minterm_to_cell(m: int) -> Tuple[int, int]:
    return GRAY.index((m >> 2) & 3), GRAY.index(m & 3)


def parse_index_list(text: str, limit: int = 16) -> List[int]:
    """Parse '1, 3, 5-7, m9' into a sorted unique list. Raises ValueError on bad tokens."""
    out: Set[int] = set()
    cleaned = text.replace(";", ",").replace(" ", ",")
    for tok in cleaned.split(","):
        tok = tok.strip().lower().lstrip("md")
        if not tok:
            continue
        if "-" in tok:
            lo_s, hi_s = tok.split("-", 1)
            lo, hi = int(lo_s), int(hi_s)
            if lo > hi:
                lo, hi = hi, lo
            rng = range(lo, hi + 1)
        else:
            rng = [int(tok)]
        for v in rng:
            if not 0 <= v < limit:
                raise ValueError(f"index {v} outside 0..{limit - 1}")
            out.add(v)
    return sorted(out)


@dataclass(frozen=True)
class Implicant:
    """A product term over 4 variables. `mask` bit = 1 marks an eliminated ('-') variable."""
    value: int
    mask: int
    minterms: frozenset

    NVARS = 4

    def literals(self) -> List[Tuple[int, bool]]:
        """[(variable index 0..3 (A..D), complemented?)] ordered A..D."""
        lits = []
        for i in range(4):
            bit = 3 - i
            if (self.mask >> bit) & 1:
                continue
            lits.append((i, ((self.value >> bit) & 1) == 0))
        return lits

    def literal_count(self) -> int:
        return len(self.literals())

    def to_string(self) -> str:
        lits = self.literals()
        if not lits:
            return "1"
        return "".join(VARS[i] + ("'" if comp else "") for i, comp in lits)

    def pattern(self) -> str:
        return "".join("-" if (self.mask >> b) & 1 else str((self.value >> b) & 1) for b in (3, 2, 1, 0))

    def evaluate(self, a: int, b: int, c: int, d: int) -> int:
        idx = (a << 3) | (b << 2) | (c << 1) | d
        return 1 if (idx & ~self.mask) == (self.value & ~self.mask) else 0

    def cell_set(self) -> Tuple[List[int], List[int]]:
        rows = sorted({minterm_to_cell(m)[0] for m in self.minterms})
        cols = sorted({minterm_to_cell(m)[1] for m in self.minterms})
        return rows, cols

    @staticmethod
    def _cyclic_interval(idx: List[int]) -> Tuple[int, int]:
        """Return (start, length) of a cyclic run on a ring of 4 (rows/cols are toroidal)."""
        s = set(idx)
        if len(s) == 4:
            return 0, 4
        for start in idx:
            if (start - 1) % 4 not in s:
                return start, len(s)
        return idx[0], len(s)

    def grid_rect(self) -> Tuple[int, int, int, int]:
        """Bounding grid rectangle (row0, height, col0, width) in unwrapped cell units.
        row0+height / col0+width may exceed 4 when the group wraps across a border."""
        rows, cols = self.cell_set()
        r0, rh = self._cyclic_interval(rows)
        c0, cw = self._cyclic_interval(cols)
        return r0, rh, c0, cw


@dataclass
class MinimizationResult:
    minterms: List[int]
    dontcares: List[int]
    maxterms: List[int]
    primes: List[Implicant]
    essentials: List[Implicant]
    cover: List[Implicant]
    expression: str                 # e.g. "F = A'B + CD"
    sop: str                        # RHS only
    canonical: str                  # canonical SOP expression
    sigma: str                      # F(A,B,C,D) = Σm(...) + d(...)
    constant: Optional[int]         # 0 / 1 when F is constant
    trace: List[str] = field(default_factory=list)
    cyclic_used: bool = False

    def evaluate(self, idx: int) -> int:
        a, b, c, d = (idx >> 3) & 1, (idx >> 2) & 1, (idx >> 1) & 1, idx & 1
        if self.constant is not None:
            return self.constant
        return 1 if any(t.evaluate(a, b, c, d) for t in self.cover) else 0

    def group_rects(self, which: Optional[Sequence[Implicant]] = None):
        return [(imp, imp.grid_rect()) for imp in (self.cover if which is None else which)]


class QuineMcCluskey:
    """Pure-python 4-variable Quine-McCluskey minimiser with Petrick's method."""

    def __init__(self, nvars: int = 4):
        self.n = nvars
        self.full = (1 << nvars) - 1

    # -- Phase 1 --------------------------------------------------------------------
    def prime_implicants(self, terms: Iterable[int], trace: List[str]) -> List[Implicant]:
        # item: (value, mask, frozenset(minterms))
        current: Dict[Tuple[int, int], Set[int]] = {(t, 0): {t} for t in terms}
        primes: List[Implicant] = []
        round_no = 0
        while current:
            round_no += 1
            groups: Dict[int, List[Tuple[int, int]]] = {}
            for (val, mask) in current:
                groups.setdefault(bin(val).count("1"), []).append((val, mask))
            trace.append(f"-- Combination round {round_no} --")
            for ones in sorted(groups):
                for (val, mask) in sorted(groups[ones]):
                    trace.append(f"  [{ones} ones] {self._pat(val, mask)}  m{sorted(current[(val, mask)])}")
            used: Set[Tuple[int, int]] = set()
            nxt: Dict[Tuple[int, int], Set[int]] = {}
            for ones in sorted(groups):
                if ones + 1 not in groups:
                    continue
                for (v1, m1) in groups[ones]:
                    for (v2, m2) in groups[ones + 1]:
                        if m1 != m2:
                            continue
                        diff = v1 ^ v2
                        if diff and (diff & (diff - 1)) == 0:          # exactly one bit differs
                            key = (v1 & ~diff, m1 | diff)
                            nxt.setdefault(key, set()).update(current[(v1, m1)] | current[(v2, m2)])
                            used.add((v1, m1))
                            used.add((v2, m2))
            for key, mts in current.items():
                if key not in used:                                      # unchecked -> prime implicant
                    primes.append(Implicant(key[0], key[1], frozenset(mts)))
                    trace.append(f"  PRIME  {self._pat(*key)}  covers m{sorted(mts)}")
            current = nxt
        return primes

    def _pat(self, val: int, mask: int) -> str:
        return "".join("-" if (mask >> b) & 1 else str((val >> b) & 1) for b in range(self.n - 1, -1, -1))

    # -- Phase 3b: Petrick ----------------------------------------------------------
    @staticmethod
    def _absorb(products: Set[frozenset]) -> Set[frozenset]:
        ordered = sorted(products, key=len)
        kept: List[frozenset] = []
        for p in ordered:
            if not any(k <= p for k in kept):
                kept.append(p)
        return set(kept)

    def petrick(self, remaining: List[int], cover_map: Dict[int, List[int]], primes: List[Implicant],
                trace: List[str]) -> Set[int]:
        products: Set[frozenset] = {frozenset()}
        for m in remaining:
            clause = cover_map[m]
            nxt = {p | {pi} for p in products for pi in clause}
            products = self._absorb(nxt)
        trace.append(f"  Petrick: {len(products)} irredundant cover(s) found")
        best = min(products, key=lambda s: (len(s), sum(primes[i].literal_count() for i in s), sorted(s)))
        return set(best)

    # -- Full pipeline --------------------------------------------------------------
    def minimize(self, minterms: Iterable[int], dontcares: Iterable[int] = (),
                 maxterms: Iterable[int] = ()) -> MinimizationResult:
        m = set(minterms)
        d = set(dontcares)
        M = set(maxterms)
        space = set(range(1 << self.n))
        for name, s in (("minterm", m), ("don't-care", d), ("maxterm", M)):
            bad = [x for x in s if x not in space]
            if bad:
                raise ValueError(f"{name} index out of range: {bad}")
        if m & d:
            raise ValueError(f"minterm and don't-care overlap: {sorted(m & d)}")
        if m & M:
            raise ValueError(f"minterm and maxterm overlap: {sorted(m & M)}")
        if M & d:
            raise ValueError(f"maxterm and don't-care overlap: {sorted(M & d)}")
        if M and not m:
            m = space - M - d                     # function specified by its maxterms (POS form)
        trace: List[str] = []
        trace.append(f"Minterms     m = {sorted(m)}")
        trace.append(f"Don't cares  d = {sorted(d)}")
        trace.append(f"Maxterms     M = {sorted(space - m - d)}")
        trace.append("")

        sigma = "F(A,B,C,D) = Σm(" + ",".join(map(str, sorted(m))) + ")"
        if d:
            sigma += " + Σd(" + ",".join(map(str, sorted(d))) + ")"
        canon_terms = ["".join(VARS[i] + ("" if (t >> (3 - i)) & 1 else "'") for i in range(4)) for t in sorted(m)]
        canonical = "F = " + (" + ".join(canon_terms) if canon_terms else "0")

        if not m:
            trace.append("No required minterms -> F = 0")
            return MinimizationResult(sorted(m), sorted(d), sorted(space - m - d), [], [], [], "F = 0", "0",
                                      canonical, sigma, 0, trace)
        if (m | d) == space:
            trace.append("Every cell is 1 or X -> F = 1")
            full = Implicant(0, self.full, frozenset(space))
            return MinimizationResult(sorted(m), sorted(d), [], [full], [full], [full], "F = 1", "1",
                                      canonical, sigma, 1, trace)

        # Phase 1 - prime implicants over m + d
        trace.append("PHASE 1: group by popcount and combine single-bit differences")
        primes = self.prime_implicants(m | d, trace)
        primes.sort(key=lambda p: (p.literal_count(), p.value, p.mask))

        # Phase 2 - chart (required minterms only)
        trace.append("")
        trace.append("PHASE 2: prime implicant chart (required minterms only)")
        req = sorted(m)
        header = "  PI            " + " ".join(f"{x:>2}" for x in req)
        trace.append(header)
        cover_map: Dict[int, List[int]] = {x: [] for x in req}
        for i, p in enumerate(primes):
            row = []
            for x in req:
                hit = x in p.minterms
                row.append(" X" if hit else " .")
                if hit:
                    cover_map[x].append(i)
            trace.append(f"  {p.to_string():<6}{p.pattern():<8}" + " ".join(row))

        # Phase 3 - essentials + Petrick
        trace.append("")
        trace.append("PHASE 3: essential prime implicants")
        chosen: Set[int] = set()
        for x in req:
            if len(cover_map[x]) == 1:
                chosen.add(cover_map[x][0])
        essentials = [primes[i] for i in sorted(chosen)]
        for i in sorted(chosen):
            trace.append(f"  EPI {primes[i].to_string()} ({primes[i].pattern()})")
        if not chosen:
            trace.append("  (none)")
        covered = set()
        for i in chosen:
            covered |= primes[i].minterms
        remaining = [x for x in req if x not in covered]
        cyclic = False
        if remaining:
            cyclic = True
            trace.append(f"  Uncovered after EPIs: {remaining} -> cyclic chart, applying Petrick's method")
            chosen |= self.petrick(remaining, cover_map, primes, trace)
        else:
            trace.append("  EPIs cover every required minterm - no Petrick needed")
        cover = [primes[i] for i in sorted(chosen)]
        cover.sort(key=lambda p: (-len(p.minterms), p.value))

        sop = " + ".join(p.to_string() for p in cover)
        trace.append("")
        trace.append(f"PHASE 4: minimal SOP  ->  F = {sop}")
        return MinimizationResult(sorted(m), sorted(d), sorted(space - m - d), primes, essentials, cover,
                                  f"F = {sop}", sop, canonical, sigma, None, trace, cyclic)


# ======================================================================================
#  CORE ALGORITHM B - COMBINATIONAL ENGINES
# ======================================================================================
class FullAdder:
    @staticmethod
    def evaluate(a: int, b: int, cin: int) -> Tuple[int, int]:
        s = a ^ b ^ cin
        cout = (a & b) | (cin & (a ^ b))
        return s, cout


class FullSubtractor:
    @staticmethod
    def evaluate(a: int, b: int, bin_: int) -> Tuple[int, int]:
        diff = a ^ b ^ bin_
        borrow = ((1 - a) & b) | ((1 - (a ^ b)) & bin_)
        return diff, borrow


class Decoder74138:
    """3-to-8 decoder. A is LSB, C is MSB. Outputs are active-low."""

    @staticmethod
    def enabled(g1: int, g2a_n: int, g2b_n: int) -> bool:
        return g1 == 1 and g2a_n == 0 and g2b_n == 0

    @classmethod
    def evaluate(cls, a: int, b: int, c: int, g1: int, g2a_n: int, g2b_n: int) -> List[int]:
        outs = [1] * 8
        if cls.enabled(g1, g2a_n, g2b_n):
            outs[(c << 2) | (b << 1) | a] = 0
        return outs


class Mux74153:
    """One half of a 74153 dual 4-to-1 multiplexer (active-low strobe)."""

    @staticmethod
    def evaluate(s0: int, s1: int, inputs: Sequence[int], strobe_n: int = 0) -> int:
        if strobe_n:
            return 0
        return int(inputs[(s1 << 1) | s0])


# ======================================================================================
#  CORE ALGORITHM C - SEQUENTIAL ENGINE (7474 D-FF, 4-bit shift register)
# ======================================================================================
class DFlipFlop:
    """Positive-edge-triggered D flip-flop with active-low async preset/clear (7474)."""

    def __init__(self) -> None:
        self.q = 0

    def async_set(self, pre_n: int, clr_n: int) -> None:
        if clr_n == 0:
            self.q = 0
        elif pre_n == 0:
            self.q = 1

    def clock(self, d: int) -> None:
        self.q = 1 if d else 0


class ShiftRegister:
    MODES = ("SISO", "SIPO", "PISO", "PIPO")

    def __init__(self) -> None:
        self.ffs = [DFlipFlop() for _ in range(4)]
        self.mode = "SISO"
        self.ext = {"si": 0, "p": [0, 0, 0, 0], "load": 1}
        self._ext_prev = {"si": 0, "p": [0, 0, 0, 0], "load": 1}
        self._ext_change_t = 0.0
        self.setup_s = 0.0                  # setup window (seconds); 0 disables the check
        self.clk_level = 0
        self.edge_count = 0
        self.violations = 0
        self.last_violation = False
        # history entries: dict(edge, si, d, q, violation)
        self.history: Deque[dict] = deque(maxlen=64)

    # -- state ----------------------------------------------------------------------
    @property
    def q(self) -> List[int]:
        return [ff.q for ff in self.ffs]

    @property
    def serial_out(self) -> int:
        return self.ffs[3].q

    def set_mode(self, mode: str) -> None:
        if mode in self.MODES:
            self.mode = mode

    def set_ext(self, now: Optional[float] = None, **kw) -> None:
        """Update external inputs (si, p, load); remembers the previous stable values for the
        setup-time check."""
        now = time.monotonic() if now is None else now
        new = {"si": self.ext["si"], "p": list(self.ext["p"]), "load": self.ext["load"]}
        for k, v in kw.items():
            new[k] = list(v) if k == "p" else int(v)
        if new != self.ext:
            if now - self._ext_change_t >= self.setup_s:                 # previous value became stable
                self._ext_prev = {"si": self.ext["si"], "p": list(self.ext["p"]), "load": self.ext["load"]}
            self._ext_change_t = now
            self.ext = new

    # -- combinational next-state ---------------------------------------------------
    def compute_d(self, ext: Optional[dict] = None, q: Optional[Sequence[int]] = None) -> List[int]:
        ext = self.ext if ext is None else ext
        q = self.q if q is None else q
        p = ext["p"]
        mode = self.mode
        if mode in ("SISO", "SIPO"):
            return [ext["si"], q[0], q[1], q[2]]
        if mode == "PISO":
            return list(p) if ext["load"] else [0, q[0], q[1], q[2]]
        return list(p)                                                    # PIPO

    def d_inputs(self) -> List[int]:
        return self.compute_d()

    # -- clocking -------------------------------------------------------------------
    def clock_edge(self, now: Optional[float] = None) -> dict:
        """Apply one rising clock edge. All flip-flops sample simultaneously."""
        now = time.monotonic() if now is None else now
        violation = bool(self.setup_s > 0 and (now - self._ext_change_t) < self.setup_s)
        ext = self._ext_prev if violation else self.ext   # data still settling -> old value captured
        d = self.compute_d(ext)
        before = self.q
        for ff, dv in zip(self.ffs, d):
            ff.clock(dv)
        self.edge_count += 1
        if violation:
            self.violations += 1
        self.last_violation = violation
        rec = {"edge": self.edge_count, "si": ext["si"], "d": d, "before": before, "q": self.q,
               "violation": violation, "mode": self.mode}
        self.history.append(rec)
        return rec

    def clock_level(self, level: int, now: Optional[float] = None) -> Optional[dict]:
        """Feed a clock *level*; fires on a 0 -> 1 transition only."""
        level = 1 if level else 0
        edge = level == 1 and self.clk_level == 0
        self.clk_level = level
        return self.clock_edge(now) if edge else None

    def clear(self) -> None:
        for ff in self.ffs:
            ff.async_set(1, 0)

    def preset(self) -> None:
        for ff in self.ffs:
            ff.async_set(0, 1)

    def reset_history(self) -> None:
        self.history.clear()
        self.edge_count = 0
        self.violations = 0


# ======================================================================================
#  HARDWARE INTERFACE - ASYNCHRONOUS SERIAL ENGINE
# ======================================================================================
def format_lcd(text: str) -> str:
    """Force an LCD line to exactly 16 printable-ASCII characters (':' is the field delimiter)."""
    clean = "".join(ch if 32 <= ord(ch) < 127 and ch != ":" else ("-" if ch == ":" else "?") for ch in str(text))
    return clean[:16].ljust(16)


def format_command(leds: Sequence[int], line1: str, line2: str) -> str:
    bits = [1 if (leds[i] if i < len(leds) else 0) else 0 for i in range(4)]
    return f"CMD:{bits[0]},{bits[1]},{bits[2]},{bits[3]}:{format_lcd(line1)}:{format_lcd(line2)}\n"


def parse_input_frame(line: str) -> Optional[Dict[str, int]]:
    """Parse one JSON line. Returns {'A':0/1,...} or None for corrupt / foreign frames."""
    line = line.strip()
    if not line.startswith("{") or not line.endswith("}"):
        return None
    try:
        obj = json.loads(line)
    except (ValueError, UnicodeDecodeError):
        return None
    if not isinstance(obj, dict) or obj.get("type") != "inputs":
        return None
    out: Dict[str, int] = {}
    for k in VARS:
        v = obj.get(k)
        if isinstance(v, bool):
            out[k] = int(v)
        elif isinstance(v, (int, float)) and v in (0, 1):
            out[k] = int(v)
        else:
            return None
    up = obj.get("u")                                  # optional board uptime (0.1 s units)
    if isinstance(up, int) and not isinstance(up, bool) and up >= 0:
        out["u"] = up
    return out


class SerialWorker(QThread):
    """Owns the serial port. Opens, reads and writes entirely inside run() so the GUI thread
    never blocks, and a sudden unplug only terminates this loop."""

    inputsReceived = pyqtSignal(dict)
    statusChanged = pyqtSignal(bool, str)      # connected, message
    logMessage = pyqtSignal(str, str)          # level, text
    frameRaw = pyqtSignal(str)                 # last raw line accepted (diagnostics)

    MIN_TX_INTERVAL = 0.05                     # 20 frames/s max (the Uno also drives a slow I2C LCD)
    KEEPALIVE = 2.5                            # re-send the last frame this often (board times out at 6 s)

    def __init__(self, port: str, baud: int = BAUD_RATE, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._stop_evt = threading.Event()
        self._wake = threading.Event()
        self._lock = threading.Lock()
        self._pending: Optional[str] = None
        self._last_sent: Optional[str] = None
        self._last_tx_t = 0.0
        self._stats = {"tx": 0, "rx": 0, "err": 0, "tx_bytes": 0, "rx_bytes": 0, "last_rx": 0.0}
        self._connected = False
        self._last_bad_emit = 0.0

    # -- public, thread-safe API ----------------------------------------------------
    def send_command(self, leds: Sequence[int], line1: str, line2: str, force: bool = False) -> bool:
        frame = format_command(leds, line1, line2)
        if not self._connected:
            return False
        with self._lock:
            if frame == self._last_sent and self._pending is None and not force:
                return True                        # board already shows this; keepalive covers it
            self._pending = frame                  # coalesce: only the newest frame matters
        self._wake.set()
        return True

    def stats(self) -> dict:
        with self._lock:
            return dict(self._stats)

    def stop(self, timeout_ms: int = 3000) -> None:
        self._stop_evt.set()
        self._wake.set()
        if self.isRunning() and not self.wait(timeout_ms):
            self.logMessage.emit("ERR", "Serial thread did not exit in time")

    # -- thread body ----------------------------------------------------------------
    def run(self) -> None:
        ser: Optional[serial.Serial] = None
        buf = bytearray()
        try:
            ser = serial.Serial(self.port, self.baud, bytesize=serial.EIGHTBITS,
                                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                timeout=0.1, write_timeout=0.5)
            try:
                ser.reset_input_buffer()
            except (serial.SerialException, OSError):
                pass
            self._connected = True
            self.statusChanged.emit(True, f"Connected {self.port} @ {self.baud} 8N1")
            while not self._stop_evt.is_set():
                self._service_tx(ser)
                waiting = ser.in_waiting
                if waiting:
                    chunk = ser.read(waiting)
                    if chunk:
                        with self._lock:
                            self._stats["rx_bytes"] += len(chunk)
                        buf.extend(chunk)
                        self._drain(buf)
                else:
                    self._wake.wait(0.004)
                    self._wake.clear()
        except (serial.SerialException, OSError) as exc:
            if not self._stop_evt.is_set():
                opened = self._connected
                self.logMessage.emit("ERR", f"Serial fault on {self.port}: {exc}")
                self.statusChanged.emit(False, f"Lost {self.port}: {exc}" if opened else f"Could not open {self.port}: {exc}")
                self._connected = False
                self._safe_close(ser)
                return
        except Exception as exc:                                      # never let a thread die silently
            self.logMessage.emit("ERR", f"Serial worker crashed: {exc!r}")
            self._connected = False
            self._safe_close(ser)
            self.statusChanged.emit(False, f"Worker error: {exc}")
            return
        self._connected = False
        self._safe_close(ser)
        self.statusChanged.emit(False, "Disconnected")

    @staticmethod
    def _safe_close(ser: Optional[serial.Serial]) -> None:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def _service_tx(self, ser: serial.Serial) -> None:
        now = time.monotonic()
        with self._lock:
            frame = self._pending
            if frame is None and self._last_sent and now - self._last_tx_t > self.KEEPALIVE:
                frame = self._last_sent
            if frame is None or now - self._last_tx_t < self.MIN_TX_INTERVAL:
                return
            self._pending = None
        data = frame.encode("ascii", "replace")
        ser.write(data)
        with self._lock:
            self._last_sent = frame
            self._last_tx_t = now
            self._stats["tx"] += 1
            self._stats["tx_bytes"] += len(data)

    def _drain(self, buf: bytearray) -> None:
        while True:
            nl = buf.find(b"\n")
            if nl < 0:
                if len(buf) > 512:                                    # runaway garbage without a newline
                    sample = bytes(buf[:40]).decode("ascii", errors="replace")
                    buf.clear()
                    with self._lock:
                        self._stats["err"] += 1
                    self.frameRaw.emit(f"<512 bytes with no newline - wrong baud rate?> {sample!r}")
                return
            raw = bytes(buf[:nl])
            del buf[:nl + 1]
            text = raw.decode("ascii", errors="ignore").strip()
            if not text:
                continue
            frame = parse_input_frame(text)
            with self._lock:
                if frame is None:
                    self._stats["err"] += 1
                    now = time.monotonic()
                    report = now - self._last_bad_emit > 1.0
                    if report:
                        self._last_bad_emit = now
                else:
                    self._stats["rx"] += 1
                    self._stats["last_rx"] = time.monotonic()
            if frame is not None:
                self.inputsReceived.emit(frame)
            elif report:
                self.frameRaw.emit(text[:90])



# ======================================================================================
#  SMALL REUSABLE WIDGETS
# ======================================================================================
class StatusLed(QWidget):
    """Round red/green/amber indicator with soft glow and optional blink."""

    def __init__(self, color: str = Theme.RED, size: int = 14, parent=None):
        super().__init__(parent)
        self._color = QColor(color)
        self._blink = 0.0
        self.setFixedSize(size, size)
        self._timer = QTimer(self)
        self._timer.setInterval(40)
        self._timer.timeout.connect(self._decay)

    def set_color(self, color: str) -> None:
        self._color = QColor(color)
        self.update()

    def flash(self) -> None:
        self._blink = 1.0
        if not self._timer.isActive():
            self._timer.start()

    def _decay(self) -> None:
        self._blink = max(0.0, self._blink - 0.12)
        if self._blink <= 0:
            self._timer.stop()
        self.update()

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = self.rect().adjusted(2, 2, -2, -2)
        c = self.rect().center()
        halo = QRadialGradient(QPointF(c), self.width() / 2)
        glow = QColor(self._color)
        glow.setAlpha(int(70 + 120 * self._blink))
        halo.setColorAt(0.5, glow)
        glow2 = QColor(self._color)
        glow2.setAlpha(0)
        halo.setColorAt(1.0, glow2)
        p.setBrush(halo)
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(self.rect())
        p.setBrush(self._color.lighter(110 + int(40 * self._blink)))
        p.setPen(QPen(self._color.darker(160), 1))
        p.drawEllipse(r.adjusted(1, 1, -1, -1))


class ToggleSwitch(QWidget):
    """Animated pill switch. `userToggled` fires only on user clicks; setValue() is silent."""

    userToggled = pyqtSignal(bool)

    def __init__(self, checked: bool = False, size: QSize = QSize(52, 26), parent=None):
        super().__init__(parent)
        self._on = bool(checked)
        self._pos = 1.0 if checked else 0.0
        self._interactive = True
        self.setFixedSize(size)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self._anim = QVariantAnimation(self)
        self._anim.setDuration(140)
        self._anim.setEasingCurve(QEasingCurve.Type.OutCubic)
        self._anim.valueChanged.connect(self._on_anim)

    def _on_anim(self, v) -> None:
        self._pos = float(v)
        self.update()

    def value(self) -> bool:
        return self._on

    def setValue(self, v: bool) -> None:
        v = bool(v)
        if v == self._on:
            return
        self._on = v
        self._anim.stop()
        self._anim.setStartValue(self._pos)
        self._anim.setEndValue(1.0 if v else 0.0)
        self._anim.start()

    def setInteractive(self, flag: bool) -> None:
        self._interactive = flag
        self.setCursor(Qt.CursorShape.PointingHandCursor if flag else Qt.CursorShape.ForbiddenCursor)
        self.update()

    def mousePressEvent(self, e) -> None:
        if e.button() == Qt.MouseButton.LeftButton and self._interactive:
            self.setValue(not self._on)
            self.userToggled.emit(self._on)

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = QRectF(self.rect()).adjusted(1, 1, -1, -1)
        off, on = QColor("#3b3b3f"), QColor(Theme.EMERALD)
        t = self._pos
        col = QColor(int(off.red() + (on.red() - off.red()) * t), int(off.green() + (on.green() - off.green()) * t),
                     int(off.blue() + (on.blue() - off.blue()) * t))
        if not self._interactive:
            col.setAlpha(140)
        p.setPen(QPen(col.lighter(130), 1))
        p.setBrush(col)
        p.drawRoundedRect(r, r.height() / 2, r.height() / 2)
        d = r.height() - 6
        x = r.left() + 3 + t * (r.width() - d - 6)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor("#f4f4f4") if self._interactive else QColor("#b9b9b9"))
        p.drawEllipse(QRectF(x, r.top() + 3, d, d))


class CheckBox(QCheckBox):
    """QCheckBox whose QSS-styled indicator also gets a visible tick when checked."""

    def paintEvent(self, e) -> None:
        super().paintEvent(e)
        if not self.isChecked():
            return
        from PyQt6.QtWidgets import QStyle, QStyleOptionButton
        opt = QStyleOptionButton()
        self.initStyleOption(opt)
        r = QRectF(self.style().subElementRect(QStyle.SubElement.SE_CheckBoxIndicator, opt, self))
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setPen(QPen(QColor("white"), 2, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
        p.drawPolyline(QPolygonF([QPointF(r.left() + r.width() * 0.26, r.center().y()),
                                  QPointF(r.left() + r.width() * 0.44, r.top() + r.height() * 0.70),
                                  QPointF(r.left() + r.width() * 0.76, r.top() + r.height() * 0.30)]))


class AnimatedCanvas(QWidget):
    """Base for QPainter canvases that need a ~30 fps clock (only runs while visible)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._t = 0.0
        self._last = time.monotonic()
        self._timer = QTimer(self)
        self._timer.setInterval(33)
        self._timer.timeout.connect(self._tick)
        self.setMinimumSize(420, 320)

    def showEvent(self, e) -> None:
        self._last = time.monotonic()
        self._timer.start()
        super().showEvent(e)

    def hideEvent(self, e) -> None:
        self._timer.stop()
        super().hideEvent(e)

    def _tick(self) -> None:
        now = time.monotonic()
        dt = min(0.1, now - self._last)
        self._last = now
        self._t += dt
        self.advance(dt)
        self.update()

    def advance(self, dt: float) -> None:       # override
        pass

    def fit_transform(self, p: QPainter, lw: float, lh: float, margin: float = 16, max_scale: float = 1.7) -> float:
        s = min((self.width() - 2 * margin) / lw, (self.height() - 2 * margin) / lh, max_scale)
        s = max(s, 0.2)
        p.translate((self.width() - lw * s) / 2, (self.height() - lh * s) / 2)
        p.scale(s, s)
        return s

    def paint_backdrop(self, p: QPainter) -> None:
        p.fillRect(self.rect(), QColor("#18181a"))
        p.setPen(QPen(QColor(255, 255, 255, 14), 1.5))
        step = 22
        for x in range(0, self.width(), step):
            for y in range(0, self.height(), step):
                p.drawPoint(x, y)


def wire_pen(val: int, width: float = 2.2) -> QPen:
    pen = QPen(QColor(Theme.LIVE) if val else QColor(Theme.DEAD), width)
    pen.setCapStyle(Qt.PenCapStyle.RoundCap)
    pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
    return pen


def draw_wire(p: QPainter, pts: Sequence[Tuple[float, float]], val: int, t: float, width: float = 2.2,
              flow: bool = True) -> None:
    path = QPainterPath(QPointF(*pts[0]))
    for x, y in pts[1:]:
        path.lineTo(x, y)
    p.setBrush(Qt.BrushStyle.NoBrush)
    if val:
        glow = QPen(qc(Theme.LIVE, 55), width + 5)
        glow.setCapStyle(Qt.PenCapStyle.RoundCap)
        glow.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        p.setPen(glow)
        p.drawPath(path)
    p.setPen(wire_pen(val, width))
    p.drawPath(path)
    if val and flow:
        dots = QPen(QColor(255, 255, 255, 190), width * 0.9)
        dots.setCapStyle(Qt.PenCapStyle.RoundCap)
        dots.setDashPattern([0.01, 9.0])
        dots.setDashOffset(-t * 6.0)
        p.setPen(dots)
        p.drawPath(path)


def draw_dot(p: QPainter, x: float, y: float, val: int, r: float = 3.6) -> None:
    p.setPen(Qt.PenStyle.NoPen)
    p.setBrush(QColor(Theme.LIVE) if val else QColor("#666666"))
    p.drawEllipse(QPointF(x, y), r, r)


# ======================================================================================
#  K-MAP WIDGET
# ======================================================================================
class KMapWidget(AnimatedCanvas):
    """4x4 Gray-code K-map. Left-click cycles 0 -> 1 -> X, right-click cycles backwards."""

    cellChanged = pyqtSignal(int, int)          # minterm, new state (0/1/2=X)

    STATE_TEXT = ("0", "1", "X")

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(520, 460)
        self.setMouseTracking(True)
        self.cells: List[int] = [0] * 16          # indexed by minterm
        self.result: Optional[MinimizationResult] = None
        self.show_all_pis = False
        self.active = 0                           # cell the live inputs point at
        self.live = True
        self.focus = -1                           # highlighted group (index into cover)
        self._hover = -1
        self._flash = [0.0] * 16
        self.pending: Optional[Tuple[int, float, float, str]] = None   # (cell, t0, duration, glyph)

    # -- model ---------------------------------------------------------------------
    def set_pending(self, pending: Optional[Tuple[int, float, float, str]]) -> None:
        self.pending = pending
        self.update()

    def set_cells(self, cells: Sequence[int]) -> None:
        self.cells = list(cells)
        self.update()

    def set_result(self, res: Optional[MinimizationResult]) -> None:
        self.result = res
        self.update()

    def set_active(self, idx: int) -> None:
        self.active = idx & 15
        self.update()

    def set_focus(self, i: int) -> None:
        self.focus = i
        self.update()

    def pulse_cell(self, m: int) -> None:
        self._flash[m] = 1.0

    def advance(self, dt: float) -> None:
        for i in range(16):
            if self._flash[i] > 0:
                self._flash[i] = max(0.0, self._flash[i] - dt * 2.5)

    # -- geometry -------------------------------------------------------------------
    def _grid(self) -> Tuple[float, float, float]:
        s = max(40.0, min((self.width() - 150) / 4, (self.height() - 150) / 4, 130.0))
        gx = (self.width() - 4 * s) / 2 + 24
        gy = (self.height() - 4 * s) / 2 + 24
        return gx, gy, s

    def _cell_at(self, pos) -> int:
        gx, gy, s = self._grid()
        c = int((pos.x() - gx) // s)
        r = int((pos.y() - gy) // s)
        if 0 <= r < 4 and 0 <= c < 4 and pos.x() >= gx and pos.y() >= gy:
            return cell_to_minterm(r, c)
        return -1

    def mouseMoveEvent(self, e) -> None:
        h = self._cell_at(e.position())
        if h != self._hover:
            self._hover = h
            self.update()

    def leaveEvent(self, _e) -> None:
        self._hover = -1
        self.update()

    def mousePressEvent(self, e) -> None:
        m = self._cell_at(e.position())
        if m < 0:
            return
        if e.button() == Qt.MouseButton.LeftButton:
            new = (self.cells[m] + 1) % 3
        elif e.button() == Qt.MouseButton.RightButton:
            new = (self.cells[m] - 1) % 3
        else:
            return
        self.cells[m] = new
        self._flash[m] = 1.0
        self.cellChanged.emit(m, new)
        self.update()

    # -- painting -------------------------------------------------------------------
    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.paint_backdrop(p)
        gx, gy, s = self._grid()
        grid = QRectF(gx, gy, 4 * s, 4 * s)

        # axis headers + diagonal corner
        p.setFont(mono(11, True))
        p.setPen(QColor(Theme.MUTED))
        for i in range(4):
            p.drawText(QRectF(gx + i * s, gy - 34, s, 28), Qt.AlignmentFlag.AlignCenter, GRAY_LABELS[i])
            p.drawText(QRectF(gx - 54, gy + i * s, 46, s), Qt.AlignmentFlag.AlignCenter, GRAY_LABELS[i])
        corner = QRectF(gx - 64, gy - 40, 62, 36)
        p.setPen(QPen(QColor(Theme.BORDER), 1.2))
        p.drawLine(QPointF(corner.left() + 2, corner.top() + 2), QPointF(corner.right() - 4, corner.bottom()))
        p.setFont(mono(9, True))
        p.setPen(QColor(Theme.CYAN_L))
        p.drawText(QRectF(corner.left() + 28, corner.top() - 2, 36, 16), Qt.AlignmentFlag.AlignCenter, "CD")
        p.drawText(QRectF(corner.left() - 2, corner.bottom() - 14, 30, 16), Qt.AlignmentFlag.AlignCenter, "AB")

        # variable bands (where each variable is 1)
        band_pen = QPen(qc(Theme.CYAN_L, 150), 2)
        p.setPen(band_pen)
        p.setFont(mono(9, True))
        # C=1 -> cols 2,3 ; D=1 -> cols 1,2 ; A=1 -> rows 2,3 ; B=1 -> rows 1,2
        p.drawLine(QPointF(gx + 2 * s + 6, gy - 42), QPointF(gx + 4 * s - 6, gy - 42))
        p.drawLine(QPointF(gx + 1 * s + 6, gy - 50), QPointF(gx + 3 * s - 6, gy - 50))
        p.drawLine(QPointF(gx - 62 - 8, gy + 2 * s + 6), QPointF(gx - 62 - 8, gy + 4 * s - 6))
        p.drawLine(QPointF(gx - 70 - 8, gy + 1 * s + 6), QPointF(gx - 70 - 8, gy + 3 * s - 6))
        p.setPen(QColor(Theme.CYAN_L))
        p.drawText(QPointF(gx + 4 * s - 2, gy - 38), "C")
        p.drawText(QPointF(gx + 3 * s - 2, gy - 46), "D")
        p.drawText(QPointF(gx - 70, gy + 4 * s + 14), "A")
        p.drawText(QPointF(gx - 78, gy + 3 * s + 14), "B")

        # cells
        for r in range(4):
            for c in range(4):
                m = cell_to_minterm(r, c)
                rect = QRectF(gx + c * s, gy + r * s, s, s).adjusted(2, 2, -2, -2)
                st = self.cells[m]
                if st == 1:
                    base = qc(Theme.EMERALD, 52)
                elif st == 2:
                    base = qc(Theme.AMBER, 44)
                else:
                    base = QColor(255, 255, 255, 9)
                p.setPen(QPen(QColor("#3a3a3e"), 1))
                p.setBrush(base)
                p.drawRoundedRect(rect, 8, 8)
                if m == self._hover:
                    p.setPen(QPen(qc(Theme.CYAN_L, 200), 1.6))
                    p.setBrush(qc(Theme.CYAN_L, 22))
                    p.drawRoundedRect(rect, 8, 8)
                if self._flash[m] > 0:
                    f = self._flash[m]
                    p.setPen(Qt.PenStyle.NoPen)
                    p.setBrush(qc(Theme.LIVE if st == 1 else Theme.AMBER if st == 2 else "#ffffff", int(90 * f)))
                    gr = rect.adjusted(-6 * (1 - f), -6 * (1 - f), 6 * (1 - f), 6 * (1 - f))
                    p.drawRoundedRect(gr, 10, 10)
                # value glyph
                col = {0: QColor("#6a6a70"), 1: QColor(Theme.LIVE), 2: QColor(Theme.AMBER)}[st]
                p.setPen(col)
                p.setFont(mono(max(14, int(s * 0.34)), True))
                p.drawText(rect, Qt.AlignmentFlag.AlignCenter, self.STATE_TEXT[st])
                p.setFont(mono(8))
                p.setPen(QColor("#7d7d85"))
                p.drawText(rect.adjusted(7, 4, -7, -4), Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft, f"m{m}")
                p.drawText(rect.adjusted(7, 4, -7, -4), Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignRight,
                           format(m, "04b"))

        # prime-implicant groups (toroidal wrap handled by clipped shifted copies)
        if self.result and self.result.constant is None:
            groups = self.result.primes if self.show_all_pis else self.result.cover
            p.save()
            p.setClipRect(grid.adjusted(-1, -1, 1, 1))
            for gi, imp in enumerate(groups):
                col = QColor(Theme.GROUP_COLORS[gi % len(Theme.GROUP_COLORS)])
                dim = self.focus >= 0 and gi != self.focus
                fill_a = 28 if dim else (95 if gi == self.focus else 58)
                stroke_a = 90 if dim else 235
                pad = 5 + 3 * (gi % 3)
                r0, rh, c0, cw = imp.grid_rect()
                for dr in (0, -4):
                    for dc in (0, -4):
                        rect = QRectF(gx + (c0 + dc) * s + pad, gy + (r0 + dr) * s + pad,
                                      cw * s - 2 * pad, rh * s - 2 * pad)
                        if not rect.intersects(grid):
                            continue
                        pen = QPen(qc(col, stroke_a), 3.2 if gi == self.focus else 2.4)
                        if self.show_all_pis and imp not in self.result.cover:
                            pen.setStyle(Qt.PenStyle.DashLine)
                        p.setPen(pen)
                        p.setBrush(qc(col, fill_a))
                        p.drawRoundedRect(rect, 14, 14)
            p.restore()

        # hardware reticle
        ar, ac = minterm_to_cell(self.active)
        arect = QRectF(gx + ac * s, gy + ar * s, s, s).adjusted(1, 1, -1, -1)
        pulse = 0.5 + 0.5 * math.sin(self._t * 5.0)
        grow = 2 + 4 * pulse
        halo = QRadialGradient(arect.center(), s * 0.85)
        halo.setColorAt(0.0, qc(Theme.CYAN_L, int(70 + 60 * pulse)))
        halo.setColorAt(1.0, qc(Theme.CYAN_L, 0))
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(halo)
        p.drawEllipse(arect.center(), s * 0.85, s * 0.85)
        ret = arect.adjusted(-grow, -grow, grow, grow)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(qc("#ffffff", int(150 + 100 * pulse)), 2.2))
        arm = max(10.0, s * 0.18)
        for (cx, cy, sx, sy) in ((ret.left(), ret.top(), 1, 1), (ret.right(), ret.top(), -1, 1),
                                 (ret.left(), ret.bottom(), 1, -1), (ret.right(), ret.bottom(), -1, -1)):
            path = QPainterPath(QPointF(cx + sx * arm, cy))
            path.lineTo(cx, cy)
            path.lineTo(cx, cy + sy * arm)
            p.drawPath(path)
        p.setPen(QPen(qc(Theme.CYAN_L, int(120 + 100 * pulse)), 1.6))
        p.drawRoundedRect(arect.adjusted(2, 2, -2, -2), 8, 8)

        # pending switch write: amber progress ring around the reticle
        if self.pending is not None and self.pending[0] == self.active:
            _cell, t0, dur, glyph = self.pending
            frac = 1.0 if dur <= 0 else min(1.0, (time.monotonic() - t0) / dur)
            ring = arect.adjusted(s * 0.2, s * 0.2, -s * 0.2, -s * 0.2)
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(qc(Theme.AMBER, 70), 5))
            p.drawEllipse(ring)
            pen = QPen(QColor(Theme.AMBER), 5)
            pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            p.setPen(pen)
            p.drawArc(ring, 90 * 16, -int(360 * 16 * frac))
            p.setFont(mono(9, True))
            p.drawText(arect.adjusted(0, 4, -8, 0), Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignRight,
                       f"\u2192{glyph}")

        # live readout
        p.setPen(QColor(Theme.MUTED))
        p.setFont(mono(9))
        p.drawText(QRectF(0, gy + 4 * s + 24, self.width(), 22), Qt.AlignmentFlag.AlignCenter,
                   f"live cell m{self.active} (ABCD={self.active:04b})   |   click: 0 > 1 > X   |   right-click: reverse")


# ======================================================================================
#  SCHEMATIC SYNTHESIS CANVAS
# ======================================================================================
class SchematicCanvas(AnimatedCanvas):
    """Builds AND/OR gate-level structure from the minimised SOP and paints live logic levels."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(560, 420)
        self.result: Optional[MinimizationResult] = None
        self.inputs = [0, 0, 0, 0]

    def set_result(self, res: Optional[MinimizationResult]) -> None:
        self.result = res
        self.update()

    def set_inputs(self, vals: Sequence[int]) -> None:
        self.inputs = [1 if v else 0 for v in vals]
        self.update()

    # -- gate symbols ---------------------------------------------------------------
    @staticmethod
    def and_path(x: float, y: float, w: float, h: float) -> QPainterPath:
        path = QPainterPath(QPointF(x, y))
        r = h / 2
        path.lineTo(x + w - r, y)
        path.arcTo(QRectF(x + w - 2 * r, y, 2 * r, h), 90, -180)
        path.lineTo(x, y + h)
        path.closeSubpath()
        return path

    @staticmethod
    def or_path(x: float, y: float, w: float, h: float) -> QPainterPath:
        path = QPainterPath(QPointF(x, y))
        path.cubicTo(x + w * 0.45, y, x + w * 0.85, y + h * 0.2, x + w, y + h / 2)
        path.cubicTo(x + w * 0.85, y + h * 0.8, x + w * 0.45, y + h, x, y + h)
        path.quadTo(x + w * 0.3, y + h / 2, x, y)
        return path

    def _gate(self, p: QPainter, path: QPainterPath, val: int, label: str, cx: float, cy: float) -> None:
        if val:
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(qc(Theme.LIVE, 36))
            p.drawPath(path.translated(0, 0))
        p.setPen(QPen(QColor(Theme.LIVE) if val else QColor("#7a7a80"), 2.4))
        p.setBrush(qc(Theme.LIVE, 30) if val else QColor("#202023"))
        p.drawPath(path)
        p.setPen(QColor("#d8d8dc") if val else QColor("#8d8d93"))
        p.setFont(mono(10, True))
        p.drawText(QRectF(cx - 30, cy - 10, 60, 20), Qt.AlignmentFlag.AlignCenter, label)

    def _inverter(self, p: QPainter, x: float, y: float, val_out: int) -> None:
        tri = QPolygonF([QPointF(x, y - 11), QPointF(x + 24, y), QPointF(x, y + 11)])
        p.setPen(QPen(QColor(Theme.LIVE) if val_out else QColor("#7a7a80"), 2.2))
        p.setBrush(qc(Theme.LIVE, 30) if val_out else QColor("#202023"))
        p.drawPolygon(tri)
        p.drawEllipse(QPointF(x + 28, y), 4, 4)

    # -- paint ----------------------------------------------------------------------
    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.paint_backdrop(p)
        res = self.result
        if res is None:
            return
        v = self.inputs
        t = self._t
        terms = [] if res.constant is not None else res.cover
        n_terms = len(terms)

        # --- layout (logical units) ---
        xt = [150 + i * 120 for i in range(4)]
        xc = [x + 70 for x in xt]
        yin = [66 + i * 36 for i in range(4)]
        y0 = 232
        x_and, w_and = 640, 84
        x_or, w_or = 842, 84
        x_out = 1010
        slots = []
        cur = y0
        for term in terms:
            n = term.literal_count()
            h = max(52.0, 18.0 * n + 16) if n >= 2 else 0.0
            slot = (h if n >= 2 else 20) + 30
            slots.append((cur, h, slot))
            cur += slot
        total_h = max(cur - y0, 60)
        lw, lh = x_out + 70, y0 + total_h + 60
        p.save()
        self.fit_transform(p, lw, lh)

        # title
        p.setPen(QColor(Theme.TEXT))
        p.setFont(mono(13, True))
        p.drawText(QPointF(10, 24), res.expression if len(res.expression) < 60 else res.expression[:57] + "...")
        p.setFont(mono(8))
        p.setPen(QColor(Theme.MUTED))
        p.drawText(QPointF(10, 40), "STAGE 1: input rails + inverters   STAGE 2: AND terms   STAGE 3: OR -> Y")

        used_true = {(i) for term in terms for (i, comp) in term.literals() if not comp}
        used_comp = {(i) for term in terms for (i, comp) in term.literals() if comp}
        bottom = y0 + total_h + 10

        # stage 1: inputs, inverters, rails
        for i in range(4):
            val = v[i]
            y = yin[i]
            p.setPen(QColor(Theme.CYAN_L))
            p.setFont(mono(14, True))
            p.drawText(QRectF(8, y - 12, 30, 24), Qt.AlignmentFlag.AlignCenter, VARS[i])
            p.setFont(mono(9))
            p.setPen(QColor(Theme.LIVE) if val else QColor("#777"))
            p.drawText(QRectF(8, y + 8, 30, 14), Qt.AlignmentFlag.AlignCenter, str(val))
            draw_wire(p, [(40, y), (xt[i], y)], val, t)
            draw_wire(p, [(xt[i], y), (xt[i] + 20, y)], val, t, flow=False)
            self._inverter(p, xt[i] + 20, y, 1 - val)
            draw_wire(p, [(xt[i] + 52, y), (xc[i], y)], 1 - val, t)
            # rails
            draw_wire(p, [(xt[i], y), (xt[i], bottom)], val, t, width=2.2 if i in used_true else 1.2,
                      flow=i in used_true)
            draw_wire(p, [(xc[i], y), (xc[i], bottom)], 1 - val, t, width=2.2 if i in used_comp else 1.2,
                      flow=i in used_comp)
            draw_dot(p, xt[i], y, val)
            p.setFont(mono(8))
            p.setPen(QColor("#8d8d93"))
            p.drawText(QPointF(xt[i] + 5, bottom + 14), VARS[i])
            p.drawText(QPointF(xc[i] + 5, bottom + 14), VARS[i] + "'")

        f_val = res.evaluate((v[0] << 3) | (v[1] << 2) | (v[2] << 1) | v[3])

        if res.constant is not None:
            cy = y0 + 20
            p.setPen(QPen(QColor(Theme.LIVE) if res.constant else QColor("#777"), 2))
            p.setBrush(QColor("#202023"))
            p.drawRoundedRect(QRectF(x_or - 20, cy - 16, 40, 32), 6, 6)
            p.setFont(mono(14, True))
            p.drawText(QRectF(x_or - 20, cy - 16, 40, 32), Qt.AlignmentFlag.AlignCenter, str(res.constant))
            draw_wire(p, [(x_or + 20, cy), (x_out, cy)], res.constant, t)
            self._draw_output(p, x_out, cy, res.constant)
            p.restore()
            return

        # stage 2: AND gates (or direct wires for 1-literal terms)
        term_out: List[Tuple[float, float, int]] = []
        for k, term in enumerate(terms):
            top, h, slot = slots[k]
            lits = term.literals()
            tv = term.evaluate(*v)
            if len(lits) >= 2:
                cy = top + h / 2
                for j, (vi, comp) in enumerate(lits):
                    py = top + h * (j + 1) / (len(lits) + 1)
                    rx = xc[vi] if comp else xt[vi]
                    val = (1 - v[vi]) if comp else v[vi]
                    draw_wire(p, [(rx, py), (x_and, py)], val, t)
                    draw_dot(p, rx, py, val)
                self._gate(p, self.and_path(x_and, top, w_and, h), tv, "AND", x_and + w_and * 0.42, cy)
                p.setPen(QColor("#a8a8b0"))
                p.setFont(mono(9))
                p.drawText(QRectF(x_and - 6, top - 16, w_and + 12, 14), Qt.AlignmentFlag.AlignCenter, term.to_string())
                term_out.append((x_and + w_and, cy, tv))
            else:
                cy = top + 10
                vi, comp = lits[0]
                rx = xc[vi] if comp else xt[vi]
                val = (1 - v[vi]) if comp else v[vi]
                draw_wire(p, [(rx, cy), (x_and + w_and, cy)], val, t)
                draw_dot(p, rx, cy, val)
                p.setPen(QColor("#a8a8b0"))
                p.setFont(mono(9))
                p.drawText(QRectF(x_and, cy - 18, w_and, 14), Qt.AlignmentFlag.AlignCenter, term.to_string())
                term_out.append((x_and + w_and, cy, tv))

        # stage 3: OR gate
        if n_terms == 1:
            ox, oy, ov = term_out[0]
            draw_wire(p, [(ox, oy), (x_out, oy)], ov, t)
            self._draw_output(p, x_out, oy, f_val)
        else:
            h_or = max(64.0, 24.0 * n_terms + 10)
            cy_or = y0 + total_h / 2
            top_or = cy_or - h_or / 2
            spacing = min(12.0, 70.0 / max(1, n_terms))
            for k, (ox, oy, ov) in enumerate(term_out):
                py = top_or + h_or * (k + 1) / (n_terms + 1)
                xm = x_and + w_and + 36 + k * spacing
                draw_wire(p, [(ox, oy), (xm, oy), (xm, py), (x_or + 8, py)], ov, t)
            self._gate(p, self.or_path(x_or, top_or, w_or, h_or), f_val, "OR", x_or + w_or * 0.45, cy_or)
            draw_wire(p, [(x_or + w_or, cy_or), (x_out, cy_or)], f_val, t)
            self._draw_output(p, x_out, cy_or, f_val)
        p.restore()

    def _draw_output(self, p: QPainter, x: float, y: float, val: int) -> None:
        pulse = 0.5 + 0.5 * math.sin(self._t * 6) if val else 0.0
        if val:
            halo = QRadialGradient(QPointF(x + 18, y), 34)
            halo.setColorAt(0, qc(Theme.LIVE, int(110 + 80 * pulse)))
            halo.setColorAt(1, qc(Theme.LIVE, 0))
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(halo)
            p.drawEllipse(QPointF(x + 18, y), 34, 34)
        p.setPen(QPen(QColor(Theme.LIVE) if val else QColor("#666"), 2.4))
        p.setBrush(QColor(Theme.LIVE) if val else QColor("#2a2a2d"))
        p.drawEllipse(QPointF(x + 18, y), 12, 12)
        p.setPen(QColor("#0b0b0c") if val else QColor("#aaa"))
        p.setFont(mono(12, True))
        p.drawText(QRectF(x + 6, y - 10, 24, 20), Qt.AlignmentFlag.AlignCenter, "Y")
        p.setPen(QColor(Theme.LIVE) if val else QColor("#777"))
        p.setFont(mono(10, True))
        p.drawText(QPointF(x + 36, y + 4), f"= {val}")


# ======================================================================================
#  REGISTER TIMING & STATE CANVAS
# ======================================================================================
class RegisterCanvas(AnimatedCanvas):
    """Four cascaded 7474 D-flip-flops plus a scrolling timing diagram."""

    VISIBLE_COLS = 10

    def __init__(self, reg: ShiftRegister, parent=None):
        super().__init__(parent)
        self.reg = reg
        self.setMinimumSize(620, 480)
        self._edge = 0.0
        self._changed = [False] * 4
        self._flash = [0.0] * 4

    def notify_edge(self, rec: dict) -> None:
        self._edge = 1.0
        self._changed = [a != b for a, b in zip(rec["before"], rec["q"])]
        for i, ch in enumerate(self._changed):
            self._flash[i] = 1.0 if ch else 0.35
        self.update()

    def advance(self, dt: float) -> None:
        self._edge = max(0.0, self._edge - dt * 1.1)
        for i in range(4):
            self._flash[i] = max(0.0, self._flash[i] - dt * 1.4)

    # -- helpers --------------------------------------------------------------------
    def _lamp(self, p: QPainter, x: float, y: float, val: int, label: str, dim: bool = False) -> None:
        p.setPen(QPen(QColor(Theme.LIVE) if val else QColor("#555"), 2))
        col = QColor(Theme.LIVE) if val else QColor("#2a2a2d")
        if dim:
            col.setAlpha(110)
        if val and not dim:
            halo = QRadialGradient(QPointF(x, y), 20)
            halo.setColorAt(0, qc(Theme.LIVE, 120))
            halo.setColorAt(1, qc(Theme.LIVE, 0))
            p.setBrush(halo)
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(QPointF(x, y), 20, 20)
            p.setPen(QPen(QColor(Theme.LIVE), 2))
        p.setBrush(col)
        p.drawEllipse(QPointF(x, y), 8, 8)
        p.setPen(QColor("#9a9aa2") if not dim else QColor("#5d5d63"))
        p.setFont(mono(9, True))
        p.drawText(QRectF(x - 24, y - 30, 48, 14), Qt.AlignmentFlag.AlignCenter, label)

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.paint_backdrop(p)
        reg = self.reg
        q = reg.q
        d = reg.d_inputs()
        mode = reg.mode
        t = self._t
        p.save()
        self.fit_transform(p, 1090, 710, margin=12, max_scale=1.5)

        # title
        p.setPen(QColor(Theme.TEXT))
        p.setFont(mono(13, True))
        p.drawText(QPointF(10, 22), f"4-BIT SHIFT REGISTER  [{mode}]   7474 D-FF x4")
        p.setFont(mono(9))
        p.setPen(QColor(Theme.MUTED))
        txt = f"edges: {reg.edge_count}    setup violations: {reg.violations}"
        p.drawText(QPointF(10, 40), txt)

        xs = [190 + i * 215 for i in range(4)]
        bw, bh, by = 104, 112, 190
        yD = by + 40
        LAMP_Y = by - 62
        clk_bus_y = by + bh + 52
        edge = self._edge

        # clock bus
        clk = reg.clk_level
        draw_wire(p, [(60, clk_bus_y), (xs[3] + bw / 2, clk_bus_y)], clk or edge > 0.55, t, width=2.6, flow=False)
        p.setFont(mono(10, True))
        p.setPen(QColor(Theme.AMBER))
        p.drawText(QRectF(8, clk_bus_y - 28, 56, 16), Qt.AlignmentFlag.AlignCenter, "CLK")
        p.setPen(QColor(Theme.LIVE) if clk else QColor("#777"))
        p.drawText(QRectF(8, clk_bus_y - 12, 56, 16), Qt.AlignmentFlag.AlignCenter, str(clk))

        for i, x in enumerate(xs):
            changed = self._changed[i]
            fl = self._flash[i]
            # flip-flop body
            body = QRectF(x, by, bw, bh)
            if fl > 0:
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(qc(Theme.CYAN_L if changed else "#ffffff", int(80 * fl)))
                p.drawRoundedRect(body.adjusted(-8 * fl, -8 * fl, 8 * fl, 8 * fl), 12, 12)
            qv = q[i]
            p.setPen(QPen(QColor(Theme.LIVE) if qv else QColor("#7a7a80"), 2.6))
            p.setBrush(qc(Theme.LIVE, 26) if qv else QColor("#202023"))
            p.drawRoundedRect(body, 8, 8)
            p.setPen(QColor("#b0b0b8"))
            p.setFont(mono(9, True))
            p.drawText(body.adjusted(8, 4, -8, -4), Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignHCenter,
                       f"FF{i} (7474)")
            p.setFont(mono(10, True))
            p.setPen(QColor(Theme.CYAN_L))
            p.drawText(QPointF(x + 8, yD + 5), "D")
            p.drawText(QPointF(x + bw - 18, yD + 5), "Q")
            p.setFont(mono(30, True))
            p.setPen(QColor(Theme.LIVE) if qv else QColor("#808088"))
            p.drawText(body.adjusted(0, 18, 0, 0), Qt.AlignmentFlag.AlignCenter, str(qv))
            # clock triangle
            tri = QPolygonF([QPointF(x + bw / 2 - 9, by + bh), QPointF(x + bw / 2, by + bh - 12),
                             QPointF(x + bw / 2 + 9, by + bh)])
            p.setPen(QPen(QColor(Theme.AMBER) if (clk or edge > 0.5) else QColor("#777"), 2))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawPolygon(tri)
            draw_wire(p, [(x + bw / 2, by + bh), (x + bw / 2, clk_bus_y)], clk or edge > 0.55, t, flow=False)
            draw_dot(p, x + bw / 2, clk_bus_y, clk or edge > 0.55)

            # D input network
            dv = d[i]
            if mode in ("SISO", "SIPO"):
                src = [(60, yD), (x, yD)] if i == 0 else [(xs[i - 1] + bw, yD), (x, yD)]
                draw_wire(p, src, dv, t)
                if i == 0:
                    p.setFont(mono(10, True))
                    p.setPen(QColor(Theme.CYAN_L))
                    p.drawText(QRectF(4, yD - 30, 60, 16), Qt.AlignmentFlag.AlignCenter, "SI")
                    p.setPen(QColor(Theme.LIVE) if dv else QColor("#777"))
                    p.drawText(QRectF(4, yD - 14, 60, 14), Qt.AlignmentFlag.AlignCenter, str(dv))
            elif mode == "PIPO":
                px = x - 40
                draw_wire(p, [(px, LAMP_Y), (px, yD), (x, yD)], dv, t)
                self._lamp(p, px, LAMP_Y, reg.ext["p"][i], f"P{i}")
            else:  # PISO: 2:1 mux in front of each D
                mx = x - 70
                mrect = QRectF(mx, yD - 34, 34, 68)
                load = reg.ext["load"]
                shift_src = 0 if i == 0 else q[i - 1]
                draw_wire(p, [(mx + 34, yD), (x, yD)], dv, t)
                p.setPen(QPen(QColor("#9a9aa2"), 1.8))
                p.setBrush(QColor("#232326"))
                p.drawRoundedRect(mrect, 5, 5)
                p.setFont(mono(8, True))
                p.setPen(QColor("#9a9aa2"))
                p.drawText(mrect, Qt.AlignmentFlag.AlignCenter, "MUX")
                draw_wire(p, [(mx + 8, LAMP_Y), (mx + 8, yD - 34)], reg.ext["p"][i], t)
                self._lamp(p, mx + 8, LAMP_Y, reg.ext["p"][i], f"P{i}")
                src_pts = ([(xs[i - 1] + bw + 16, yD), (xs[i - 1] + bw + 16, yD + 20), (mx, yD + 20)] if i > 0
                           else [(mx - 22, yD + 20), (mx, yD + 20)])
                draw_wire(p, src_pts, shift_src, t)
                p.setFont(mono(8))
                p.setPen(QColor("#8d8d93"))
                p.drawText(QPointF(mx + 40, yD - 24), "LD" if load else "SH")
                p.setPen(QPen(QColor(Theme.AMBER) if load else QColor("#555"), 1.6))
                p.drawLine(QPointF(mx + 34, yD - 20), QPointF(mx + 34 - 6, yD - 20))
            # Q output + parallel-out lamp
            serial_mode = mode in ("SISO", "PISO")
            qxo = x + bw
            if i == 3 and serial_mode:
                draw_wire(p, [(qxo, yD), (qxo + 70, yD)], qv, t)
            lamp_x = qxo + 16 if i < 3 else qxo + 30
            draw_wire(p, [(qxo, yD), (lamp_x, yD), (lamp_x, LAMP_Y)], qv, t, flow=False)
            draw_dot(p, lamp_x, yD, qv)
            self._lamp(p, lamp_x, LAMP_Y, qv, f"Q{i}", dim=serial_mode and i != 3)
            # edge-travelling data pulse along the chain
            if edge > 0 and mode in ("SISO", "SIPO") and i > 0 and self._changed[i]:
                prog = 1.0 - edge
                px = xs[i - 1] + bw + (x - (xs[i - 1] + bw)) * prog
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(qc("#ffffff", int(230 * edge)))
                p.drawEllipse(QPointF(px, yD), 6, 6)

        if mode in ("SISO", "PISO"):
            p.setFont(mono(10, True))
            p.setPen(QColor(Theme.CYAN_L))
            p.drawText(QPointF(xs[3] + bw + 76, yD + 4), "SO")
            p.setPen(QColor(Theme.LIVE) if q[3] else QColor("#777"))
            p.drawText(QPointF(xs[3] + bw + 76, yD + 20), f"= {q[3]}")

        # ---------------- timing diagram ----------------
        ty0 = 424
        p.setPen(QColor(Theme.MUTED))
        p.setFont(mono(9, True))
        p.drawText(QPointF(10, ty0 - 14), "TIMING DIAGRAM  (rising-edge sampling, newest on the right)")
        rows = ["CLK", "D0", "Q0", "Q1", "Q2", "Q3"]
        row_h = 46
        x0, x1 = 90.0, 1060.0
        hist = list(reg.history)[-self.VISIBLE_COLS:]
        ncols = self.VISIBLE_COLS + 1
        cw = (x1 - x0) / ncols
        # grid
        p.setPen(QPen(QColor(255, 255, 255, 14), 1))
        for c in range(ncols + 1):
            p.drawLine(QPointF(x0 + c * cw, ty0), QPointF(x0 + c * cw, ty0 + row_h * len(rows)))
        for r, name in enumerate(rows):
            yb = ty0 + r * row_h + row_h - 12
            yh = yb - 24
            p.setPen(QPen(QColor(255, 255, 255, 18), 1))
            p.drawLine(QPointF(x0, yb + 6), QPointF(x1, yb + 6))
            p.setPen(QColor(Theme.AMBER) if name == "CLK" else QColor(Theme.CYAN_L))
            p.setFont(mono(10, True))
            p.drawText(QRectF(10, yh, 70, 24), Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft, name)

            def level(vv):
                return yh if vv else yb

            cur_q = q
            path = QPainterPath()
            if name == "CLK":
                path.moveTo(x0, yb)
                for j in range(len(hist)):
                    xe = x0 + j * cw
                    path.lineTo(xe, yb)
                    path.lineTo(xe, yh)
                    path.lineTo(xe + cw * 0.5, yh)
                    path.lineTo(xe + cw * 0.5, yb)
                xe = x0 + len(hist) * cw
                path.lineTo(xe, yb)
                path.lineTo(x1, yb)
            else:
                if name == "D0":
                    prev = hist[0]["d"][0] if hist else d[0]
                    path.moveTo(x0, level(prev))
                    for j, rec in enumerate(hist):
                        xe = x0 + j * cw
                        nv = rec["d"][0]
                        xch = xe - cw * 0.3 if j > 0 else xe
                        path.lineTo(xch, level(prev))
                        path.lineTo(xch, level(nv))
                        prev = nv
                    xe = x0 + len(hist) * cw
                    path.lineTo(xe + 0.0, level(prev))
                    nvv = d[0]
                    xch = xe + cw * 0.3
                    path.lineTo(xch, level(prev))
                    path.lineTo(xch, level(nvv))
                    path.lineTo(x1, level(nvv))
                else:
                    qi = int(name[1])
                    prev = hist[0]["before"][qi] if hist else cur_q[qi]
                    path.moveTo(x0, level(prev))
                    for j, rec in enumerate(hist):
                        xe = x0 + j * cw
                        nv = rec["q"][qi]
                        path.lineTo(xe, level(prev))
                        path.lineTo(xe, level(nv))
                        prev = nv
                    path.lineTo(x1, level(prev))
            lw = 2.2
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(QColor("#4a8fd8") if name != "CLK" else QColor("#c98a2a"), lw, Qt.PenStyle.SolidLine,
                          Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.MiterJoin))
            p.drawPath(path)
            # highlight latest transition
            if hist and name.startswith("Q"):
                qi = int(name[1])
                last = hist[-1]
                if last["before"][qi] != last["q"][qi]:
                    xe = x0 + (len(hist) - 1) * cw
                    hp = QPen(qc(Theme.LIVE, int(120 + 135 * self._edge)), 3.4)
                    hp.setCapStyle(Qt.PenCapStyle.RoundCap)
                    p.setPen(hp)
                    p.drawLine(QPointF(xe, yb), QPointF(xe, yh))
                    p.setPen(Qt.PenStyle.NoPen)
                    p.setBrush(qc(Theme.LIVE, int(70 * self._edge)))
                    p.drawEllipse(QPointF(xe, (yb + yh) / 2), 12 * self._edge + 3, 12 * self._edge + 3)
        # newest-edge band + violation markers
        if hist:
            jx = x0 + (len(hist) - 1) * cw
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(qc(Theme.CYAN_L, int(36 * self._edge)))
            p.drawRect(QRectF(jx, ty0 - 4, cw, row_h * len(rows) + 4))
            for j, rec in enumerate(hist):
                if rec["violation"]:
                    p.setPen(QColor(Theme.RED))
                    p.setFont(mono(9, True))
                    p.drawText(QRectF(x0 + j * cw, ty0 + row_h * len(rows) + 2, cw, 14), Qt.AlignmentFlag.AlignCenter, "SETUP!")
        # 'now' marker
        nx = x0 + len(hist) * cw
        p.setPen(QPen(qc("#ffffff", 90), 1, Qt.PenStyle.DashLine))
        p.drawLine(QPointF(nx, ty0 - 4), QPointF(nx, ty0 + row_h * len(rows)))
        p.setPen(QColor("#8d8d93"))
        p.setFont(mono(8))
        p.drawText(QRectF(nx, ty0 + row_h * len(rows) + 2, cw * 2, 14), Qt.AlignmentFlag.AlignLeft, "now")
        p.restore()


# ======================================================================================
#  DEVICE (IC) VIEW CANVAS
# ======================================================================================
@dataclass
class Pin:
    name: str
    val: Optional[int]
    bubble: bool = False     # active-low
    hot: bool = False        # highlighted (the active output / selected input)


class DeviceCanvas(AnimatedCanvas):
    """Draws a DIP-style IC with live pin levels, used for adder/subtractor, 74138 and 74153."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(560, 420)
        self.title = ""
        self.subtitle = ""
        self.left: List[Pin] = []
        self.right: List[Pin] = []
        self.caption: List[str] = []

    def set_device(self, title: str, subtitle: str, left: List[Pin], right: List[Pin], caption: List[str]) -> None:
        self.title, self.subtitle, self.left, self.right, self.caption = title, subtitle, left, right, caption
        self.update()

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.paint_backdrop(p)
        n = max(len(self.left), len(self.right), 1)
        pitch = 36.0
        body_w, body_h = 300.0, n * pitch + 56 + 46
        lw, lh = body_w + 2 * 170, body_h + 40 + 22 * len(self.caption) + 20
        p.save()
        self.fit_transform(p, lw, lh, max_scale=1.5)
        bx, by = 170.0, 20.0
        body = QRectF(bx, by, body_w, body_h)
        grad = QLinearGradient(body.topLeft(), body.bottomRight())
        grad.setColorAt(0, QColor("#2c2c30"))
        grad.setColorAt(1, QColor("#1b1b1e"))
        p.setPen(QPen(QColor("#6a6a72"), 2.4))
        p.setBrush(grad)
        p.drawRoundedRect(body, 12, 12)
        p.setBrush(QColor("#101012"))
        p.setPen(QPen(QColor("#55555c"), 1.6))
        p.drawChord(QRectF(bx + body_w / 2 - 14, by - 14, 28, 28), 180 * 16, 180 * 16)
        p.setPen(QColor(Theme.TEXT))
        p.setFont(mono(18, True))
        p.drawText(QRectF(bx, by + 8, body_w, 36), Qt.AlignmentFlag.AlignCenter, self.title)
        p.setFont(mono(9))
        p.setPen(QColor(Theme.MUTED))
        p.drawText(QRectF(bx + 10, by + body_h - 40, body_w - 20, 34), Qt.AlignmentFlag.AlignCenter | Qt.TextFlag.TextWordWrap, self.subtitle)

        def draw_pins(pins: List[Pin], left_side: bool) -> None:
            for i, pin in enumerate(pins):
                y = by + 52 + i * pitch + pitch / 2 - 4
                v = pin.val if pin.val is not None else 0
                if left_side:
                    pts = [(bx - 90, y), (bx, y)]
                    tx = QRectF(bx - 178, y - 11, 74, 22)
                    al = Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
                    ix = bx + 10
                else:
                    pts = [(bx + body_w, y), (bx + body_w + 90, y)]
                    tx = QRectF(bx + body_w + 104, y - 11, 74, 22)
                    al = Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
                    ix = bx + body_w - 90
                draw_wire(p, pts, v, self._t, width=2.6)
                if pin.hot:
                    pulse = 0.5 + 0.5 * math.sin(self._t * 6)
                    p.setPen(Qt.PenStyle.NoPen)
                    p.setBrush(qc(Theme.LIVE if not pin.bubble else Theme.AMBER, int(60 + 60 * pulse)))
                    cx = pts[0][0] if left_side else pts[1][0]
                    p.drawEllipse(QPointF(cx, y), 11 + 3 * pulse, 11 + 3 * pulse)
                if pin.bubble:
                    bxp = bx - 2 if left_side else bx + body_w + 2
                    p.setPen(QPen(QColor("#9a9aa2"), 1.8))
                    p.setBrush(QColor("#1b1b1e"))
                    p.drawEllipse(QPointF(bxp + (-5 if not left_side else 5), y), 4.5, 4.5)
                p.setFont(mono(10, True))
                p.setPen(QColor(Theme.CYAN_L) if not pin.hot else QColor(Theme.AMBER))
                p.drawText(tx, al, pin.name)
                if pin.val is not None:
                    p.setPen(QColor(Theme.LIVE) if pin.val else QColor("#8b8b90"))
                    p.drawText(QRectF(ix - 14 if not left_side else ix, y - 11, 30, 22),
                               Qt.AlignmentFlag.AlignCenter, str(pin.val))
                draw_dot(p, pts[0][0] if left_side else pts[1][0], y, v, 4.5)

        draw_pins(self.left, True)
        draw_pins(self.right, False)
        p.setFont(mono(10))
        cy = by + body_h + 24
        for line in self.caption:
            p.setPen(QColor("#c8c8cf"))
            p.drawText(QRectF(0, cy, lw, 20), Qt.AlignmentFlag.AlignCenter, line)
            cy += 22
        p.restore()


# ======================================================================================
#  TRUTH TABLE
# ======================================================================================
class TruthTable(QTableWidget):
    """Compact monospace table; supports a highlighted live row and per-cell colouring."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.verticalHeader().setVisible(False)
        self.verticalHeader().setDefaultSectionSize(23)
        self.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.setShowGrid(True)
        self.interactive_col = -1
        self._live = -1

    def populate(self, headers: List[str], rows: List[List[str]], live_row: int = -1) -> None:
        if self.columnCount() != len(headers) or self.rowCount() != len(rows):
            self.clear()
            self.setColumnCount(len(headers))
            self.setRowCount(len(rows))
        self.setHorizontalHeaderLabels(headers)
        for r, row in enumerate(rows):
            for c, text in enumerate(row):
                it = self.item(r, c)
                if it is None:
                    it = QTableWidgetItem()
                    it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                    self.setItem(r, c, it)
                if it.text() != text:
                    it.setText(text)
                if text == "1":
                    it.setForeground(QColor(Theme.LIVE))
                elif text == "X":
                    it.setForeground(QColor(Theme.AMBER))
                elif text == "0":
                    it.setForeground(QColor("#7d7d85"))
                else:
                    it.setForeground(QColor(Theme.TEXT))
                is_live = r == live_row
                it.setBackground(qc(Theme.CYAN, 90) if is_live else (qc("#ffffff", 8) if c == self.interactive_col else QColor(0, 0, 0, 0)))
        self._live = live_row
        self.setFixedHeight(min(23 * len(rows) + 30, 23 * 17 + 30))


# ======================================================================================
#  LANDING PAGE  (light card on navy, dotted grid, isometric chip hero)
# ======================================================================================
LANDING_QSS = f"""
QWidget#landing {{ background: {Theme.NAVY}; }}
QFrame#lcard {{ background: {Theme.PAPER}; border-radius: 16px; }}
QWidget#nav {{ background: transparent; border-bottom: 1px solid {Theme.LINE}; }}
QLabel {{ background: transparent; font-family: '{Theme.SANS}'; color: {Theme.INK}; }}
QLabel#navlink {{ font-size: 8.5pt; font-weight: 600; letter-spacing: 0.4px; color: {Theme.INK}; padding: 4px 2px; }}
QLabel#navlink:hover {{ color: {Theme.BLUE}; }}
QLabel#brand {{ font-size: 11pt; font-weight: 700; color: {Theme.INK}; }}
QLabel#sub {{ font-size: 11pt; color: {Theme.INK_2}; }}
QLabel#lpill {{ background: {Theme.PAPER}; border: 1px solid {Theme.LINE}; border-radius: 13px; padding: 5px 12px;
               font-size: 9pt; font-weight: 600; color: {Theme.INK}; }}
QLabel#chip {{ background: {Theme.PAPER}; border: 1px solid {Theme.LINE}; border-radius: 11px; padding: 3px 10px;
              font-size: 8.5pt; font-weight: 600; color: {Theme.INK}; }}
QLabel#cap {{ font-size: 9pt; color: {Theme.INK_2}; }}
QFrame#lbox {{ background: {Theme.PAPER}; border: 1px solid {Theme.LINE}; border-radius: 10px; }}
QPushButton {{ font-family: '{Theme.SANS}'; font-size: 9.5pt; font-weight: 600; border-radius: 4px; padding: 9px 20px; }}
QPushButton#cta {{ background: {Theme.BLUE}; color: white; border: 1px solid {Theme.BLUE}; }}
QPushButton#cta:hover {{ background: #2E96FF; }}
QPushButton#cta:pressed {{ background: {Theme.BLUE_D}; }}
QPushButton#outline {{ background: {Theme.PAPER}; color: {Theme.INK}; border: 1px solid #6B7280; }}
QPushButton#outline:hover {{ border-color: {Theme.BLUE}; color: {Theme.BLUE}; }}
QPushButton#dark {{ background: #1F2328; color: white; border: 1px solid #1F2328; padding: 7px 16px; font-size: 8.5pt; }}
QPushButton#dark:hover {{ background: #000; }}
QPushButton#navcta {{ background: {Theme.BLUE}; color: white; border: 1px solid {Theme.BLUE}; padding: 7px 16px; font-size: 8.5pt; }}
QPushButton#navcta:hover {{ background: #2E96FF; }}
"""

DOCS_HTML = """
<style>
 body { font-family: 'Segoe UI'; color:#e6e6e6; } h2 { color:#3AA0FF; margin-top:18px; }
 code, pre { font-family: Consolas; color:#2ECC71; } pre { background:#171718; padding:8px; }
 td, th { padding:3px 10px; border-bottom:1px solid #3c3c3f; font-family: Consolas; }
</style>
<a name="start"></a><h2>Quick start</h2>
<ol>
<li>Click <b>Launch Console</b>. With no board attached, the A-D switches are software toggles (keys <code>A B C D</code>).</li>
<li>Use <b>Auto-sweep</b> (Input switches card) for a virtual board that counts 0-15 so every canvas animates.</li>
<li>Plug the trainer in: the port list refreshes automatically; <b>AUTO</b> mode connects to the first USB-serial adapter it finds.</li>
</ol>
<a name="modes"></a><h2>Modes</h2>
<table>
<tr><th>Mode</th><th>What it does</th></tr>
<tr><td>K-MAP / SOP</td><td>Click cells 0 &gt; 1 &gt; X. Quine-McCluskey + Petrick produce the minimal SOP, coloured groups (with toroidal wrap), a synthesised AND/OR schematic and a full step trace.</td></tr>
<tr><td>ADDER / SUB</td><td>Full adder or full subtractor on A, B, C(in). Push Sum/Carry straight into the K-map.</td></tr>
<tr><td>74138 DECODER</td><td>A=LSB..C=MSB selects; active only for G1=1, /G2A=0, /G2B=0. Active-low outputs Y0-Y7.</td></tr>
<tr><td>74153 MUX</td><td>S0=A, S1=B select data inputs of two 4:1 multiplexers with active-low strobes.</td></tr>
<tr><td>SHIFT REGISTER</td><td>SISO / SIPO / PISO / PIPO from four 7474 D-flip-flops. Clock by button, auto-clock, or a rising edge on switch D. Optional setup-time check.</td></tr>
</table>
<a name="protocol"></a><h2>Serial protocol (115200, 8-N-1)</h2>
<p><b>Board -&gt; PC</b>, one JSON object per line:</p>
<pre>{"type":"inputs","A":1,"B":0,"C":1,"D":0}</pre>
<p><b>PC -&gt; Board</b>, one frame per line (LCD fields are exactly 16 characters):</p>
<pre>CMD:&lt;LED0&gt;,&lt;LED1&gt;,&lt;LED2&gt;,&lt;LED3&gt;:&lt;LCD_LINE1&gt;:&lt;LCD_LINE2&gt;\\n
CMD:1,0,0,1:ABCD=0111 m7 F=1:F=A'B + CD        </pre>
<p>Corrupt or partial frames are discarded; outbound frames are rate-limited (40/s), coalesced and re-sent every 1.5 s as a keep-alive.</p>
<a name="hardware"></a><h2>LED / LCD mapping</h2>
<table>
<tr><th>Mode</th><th>LED0</th><th>LED1</th><th>LED2</th><th>LED3</th></tr>
<tr><td>K-Map</td><td>F</td><td>F'</td><td>cell = X</td><td>0</td></tr>
<tr><td>Adder/Sub</td><td>Sum/Diff</td><td>Cout/Bout</td><td>A</td><td>B</td></tr>
<tr><td>74138</td><td>enabled</td><td>sel bit0</td><td>sel bit1</td><td>sel bit2</td></tr>
<tr><td>74153</td><td>1Y</td><td>2Y</td><td>S0</td><td>S1</td></tr>
<tr><td>Shift reg</td><td>Q0</td><td>Q1</td><td>Q2</td><td>Q3</td></tr>
</table>
<a name="about"></a><h2>About</h2>
<p>All Boolean algebra (grouping, prime implicants, essential PIs, Petrick's method, K-map rectangle geometry,
schematic layout) is implemented natively - no SymPy, no pyeda.</p>
"""


class LogoMark(QWidget):
    def __init__(self, size: int = 24, dark_bg: bool = False, parent=None):
        super().__init__(parent)
        self.setFixedSize(size, size)

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = QRectF(self.rect()).adjusted(1, 1, -1, -1)
        g = QLinearGradient(r.topLeft(), r.bottomRight())
        g.setColorAt(0, QColor("#3AA0FF"))
        g.setColorAt(1, QColor(Theme.BLUE_D))
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(g)
        p.drawRoundedRect(r, 5, 5)
        p.setBrush(QColor("white"))
        u = r.width() / 5
        for (cx, cy) in ((1, 1), (3, 1), (2, 2), (1, 3), (3, 3)):
            p.drawRect(QRectF(r.left() + cx * u + 0.5, r.top() + cy * u + 0.5, u - 1, u - 1))


class HeroCanvas(AnimatedCanvas):
    """Isometric trainer-board illustration + animated status cards on a dotted grid."""

    A, B = math.cos(math.radians(30)), 0.5

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(420, 420)
        self._anim_t = 0.0

    # ---- iso helpers ----
    def iso(self, x: float, y: float, z: float = 0.0) -> QPointF:
        return QPointF(self.cx + (x - y) * self.A * self.u, self.cy + (x + y) * self.B * self.u - z * self.u)

    def box(self, p: QPainter, x, y, z, w, d, h, top, left, right, edge="#00000022") -> None:
        faces = [
            ([(x + w, y, z), (x + w, y + d, z), (x + w, y + d, z + h), (x + w, y, z + h)], right),
            ([(x, y + d, z), (x + w, y + d, z), (x + w, y + d, z + h), (x, y + d, z + h)], left),
            ([(x, y, z + h), (x + w, y, z + h), (x + w, y + d, z + h), (x, y + d, z + h)], top),
        ]
        p.setPen(QPen(QColor(edge), 1))
        for pts, col in faces:
            p.setBrush(QColor(col))
            p.drawPolygon(QPolygonF([self.iso(*q) for q in pts]))

    def plane(self, p: QPainter, x: float, y: float, z: float, fine: float = 10.0) -> None:
        """Set a transform that maps flat 'fine' units (1/fine of an iso unit) onto a horizontal plane."""
        o = self.iso(x, y, z)
        k = self.u / fine
        p.setTransform(QTransform(self.A * k, self.B * k, -self.A * k, self.B * k, o.x(), o.y()) * self._base)

    def advance(self, dt: float) -> None:
        self._anim_t += dt

    def paintEvent(self, _e) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        clip = QPainterPath()
        clip.addRoundedRect(QRectF(-40, -40, self.width() + 40, self.height() + 40), 16, 16)
        p.setClipPath(clip)
        p.fillRect(self.rect(), QColor("#FBFCFE"))
        # dotted grid
        p.setPen(QPen(QColor("#D6DDE8"), 1.6, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        for x in range(8, self.width(), 14):
            for y in range(8, self.height(), 14):
                p.drawPoint(x, y)
        p.setPen(QPen(QColor(Theme.LINE), 1))
        p.drawLine(0, 0, 0, self.height())

        t = self._anim_t
        lw, lh = 620.0, 640.0
        s = max(0.35, min(self.width() / lw, self.height() / lh, 1.5))
        p.translate((self.width() - lw * s) / 2, (self.height() - lh * s) / 2)
        p.scale(s, s)
        self._base = p.transform()
        self.u = 27.0
        self.cx, self.cy = 312.0, 372.0

        # soft ground shadow
        sh = QRadialGradient(QPointF(self.cx + 14, self.cy + 70), 270)
        sh.setColorAt(0, QColor(15, 30, 60, 46))
        sh.setColorAt(1, QColor(15, 30, 60, 0))
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(sh)
        p.drawEllipse(QPointF(self.cx + 14, self.cy + 70), 270, 110)

        # platform
        self.box(p, -5.6, -5.6, 0, 11.2, 11.2, 0.55, "#F7F9FC", "#252A31", "#12161B", "#0000002a")
        # circuit traces on the platform
        z = 0.56
        centre = (0.0, 0.0)
        blocks = [(-3.4, -3.4, "K-MAP"), (3.4, -3.4, "74138"), (-3.4, 3.4, "74153"), (3.4, 3.4, "7474")]
        paths = []
        for (bx, by, _n) in blocks:
            paths.append([centre, (bx, 0.0), (bx, by)])
        for path in paths:
            poly = QPolygonF([self.iso(x, y, z) for (x, y) in path])
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.setPen(QPen(QColor("#9CC8FF"), 2.2, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
            p.drawPolyline(poly)
        # data pulses
        for k, path in enumerate(paths):
            seg = [math.dist(path[i], path[i + 1]) for i in range(len(path) - 1)]
            total = sum(seg)
            for off in (0.0, 0.5):
                f = ((t * 0.35 + k * 0.23 + off) % 1.0) * total
                for i, sl in enumerate(seg):
                    if f <= sl:
                        a, b = path[i], path[i + 1]
                        x = a[0] + (b[0] - a[0]) * f / sl
                        y = a[1] + (b[1] - a[1]) * f / sl
                        pt = self.iso(x, y, z)
                        halo = QRadialGradient(pt, 12)
                        halo.setColorAt(0, QColor(10, 132, 255, 150))
                        halo.setColorAt(1, QColor(10, 132, 255, 0))
                        p.setPen(Qt.PenStyle.NoPen)
                        p.setBrush(halo)
                        p.drawEllipse(pt, 12, 12)
                        p.setBrush(QColor(Theme.BLUE))
                        p.drawEllipse(pt, 3.6, 3.6)
                        break
                    f -= sl

        # blocks + centre cube, painter-sorted
        items = [(bx, by, n) for (bx, by, n) in blocks] + [(0.0, 0.0, "CORE")]
        items.sort(key=lambda it: it[0] + it[1])
        for (bx, by, name) in items:
            if name == "CORE":
                bob = 0.18 * math.sin(t * 2.2) + 0.18
                self.box(p, -1.6, -1.6, 0.56 + bob * 0.0, 3.2, 3.2, 1.7 + bob, "#2D93FF", "#0A6FE0", "#0757B8", "#00000030")
                top_z = 0.56 + 1.7 + bob
                self.plane(p, 0, 0, top_z)
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(QColor("white"))
                for (cx, cy) in ((0, -9), (-9, 0), (0, 0), (9, 0), (0, 9)):
                    p.drawRect(QRectF(cx - 3.6, cy - 3.6, 7.2, 7.2))
                p.setTransform(self._base)
            else:
                self.box(p, bx - 1.5, by - 1.5, 0.56, 3.0, 3.0, 0.8, "#FFFFFF", "#E4E9F0", "#CBD3DE", "#0000002e")
                self.plane(p, bx, by, 0.56 + 0.8)
                active = int((t * 0.8) % 4) == [b[2] for b in blocks].index(name)
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(QColor(10, 132, 255, 40 if active else 12))
                p.drawRoundedRect(QRectF(-11, -11, 22, 22), 3, 3)
                p.setPen(QColor(Theme.BLUE) if active else QColor("#2B3440"))
                f = QFont(Theme.MONO)
                f.setPixelSize(5 if name != "K-MAP" else 4)
                f.setBold(True)
                p.setFont(f)
                p.drawText(QRectF(-13, -5, 26, 10), Qt.AlignmentFlag.AlignCenter, name)
                p.setTransform(self._base)

        # floating "3/3" style badges over three blocks
        for k, (bx, by, name) in enumerate(blocks[:3]):
            pos = self.iso(bx, by, 2.6 + 0.12 * math.sin(t * 1.8 + k))
            label = ["16/16 cells", "8/8 outputs", "4/4 flops"][k]
            self._badge(p, pos, label)

        # status cards (top-left), cycling spinner -> check
        cards = ["One click. Every circuit, minimised.", "Runs live on your trainer over serial.",
                 "Verified: K-Map, 74138, 74153 and 7474."]
        tags = ["Mode 01", "Mode 02", "Mode 03"]
        cycle = t % 9.0
        for i, txt in enumerate(cards):
            appear = min(1.0, max(0.0, (t - 0.25 * i) * 2.5))
            if appear <= 0:
                continue
            fade = 1.0 if cycle < 8.2 else max(0.0, (9.0 - cycle) / 0.8)
            done_at = 1.2 + 1.5 * i
            done = cycle > done_at
            y = 34 + i * 50 + (1 - appear) * 14
            w = QFontMetrics(sans(8, True)).horizontalAdvance(txt) + 38 + 84
            p.setOpacity(appear * fade)
            self._card(p, QRectF(34, y, w, 38), txt, tags[i], done, cycle - done_at)
            p.setOpacity(1.0)

        # live signal card (bottom right)
        idx = int(t / 1.1) % 16
        a, b, c, d = (idx >> 3) & 1, (idx >> 2) & 1, (idx >> 1) & 1, idx & 1
        fval = ((1 - a) & b) | (c & d)
        rect = QRectF(330, 548, 250, 62)
        p.setPen(QPen(QColor(Theme.LINE), 1))
        p.setBrush(QColor("white"))
        p.drawRoundedRect(rect, 10, 10)
        p.setPen(QColor("#8A94A3"))
        p.setFont(sans(7, True))
        p.drawText(QRectF(rect.left() + 14, rect.top() + 8, 200, 14), Qt.AlignmentFlag.AlignLeft, "LIVE SIGNAL")
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(Theme.EMERALD) if fval else QColor("#B8C0CC"))
        p.drawEllipse(QPointF(rect.left() + 20, rect.top() + 40), 5, 5)
        p.setPen(QColor(Theme.INK))
        p.setFont(mono(9, True))
        p.drawText(QRectF(rect.left() + 32, rect.top() + 28, 210, 24), Qt.AlignmentFlag.AlignVCenter,
                   f"ABCD={idx:04b}  Y=A'B+CD = {fval}")

    def _badge(self, p: QPainter, pos: QPointF, text: str) -> None:
        f = sans(7, True)
        p.setFont(f)
        w = p.fontMetrics().horizontalAdvance(text) + 36
        r = QRectF(pos.x() - w / 2, pos.y() - 12, w, 24)
        p.setPen(QPen(QColor("#D8E6F8"), 1))
        p.setBrush(QColor("white"))
        p.drawRoundedRect(r, 12, 12)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(Theme.BLUE))
        p.drawEllipse(QPointF(r.left() + 13, r.center().y()), 6, 6)
        p.setPen(QPen(QColor("white"), 1.6, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawPolyline(QPolygonF([QPointF(r.left() + 10, r.center().y()), QPointF(r.left() + 12.3, r.center().y() + 2.6),
                                  QPointF(r.left() + 16.5, r.center().y() - 2.6)]))
        p.setPen(QColor(Theme.INK))
        p.drawText(QRectF(r.left() + 22, r.top(), r.width() - 24, r.height()), Qt.AlignmentFlag.AlignVCenter, text)

    def _card(self, p: QPainter, r: QRectF, text: str, tag: str, done: bool, since: float) -> None:
        p.setPen(QPen(QColor(Theme.LINE), 1))
        p.setBrush(QColor("white"))
        p.drawRoundedRect(r, 9, 9)
        cx, cy = r.left() + 20, r.center().y()
        if done:
            pop = min(1.0, max(0.0, since * 5))
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QColor(Theme.EMERALD))
            rad = 8.5 * (0.6 + 0.4 * pop)
            p.drawEllipse(QPointF(cx, cy), rad, rad)
            p.setPen(QPen(QColor("white"), 1.9, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
            p.drawPolyline(QPolygonF([QPointF(cx - 3.6, cy), QPointF(cx - 1, cy + 3), QPointF(cx + 3.8, cy - 3)]))
        else:
            p.setPen(QPen(QColor("#D5DDE9"), 2.2))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(QPointF(cx, cy), 7.5, 7.5)
            p.setPen(QPen(QColor(Theme.BLUE), 2.2, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            p.drawArc(QRectF(cx - 7.5, cy - 7.5, 15, 15), int(-self._anim_t * 360 * 16 * 1.6) % (360 * 16), 100 * 16)
        p.setPen(QColor(Theme.INK))
        p.setFont(sans(8, True))
        p.drawText(QRectF(r.left() + 38, r.top(), r.width() - 100, r.height()), Qt.AlignmentFlag.AlignVCenter, text)
        p.setPen(QColor("#9AA4B2"))
        p.setFont(sans(7))
        p.drawText(QRectF(r.right() - 66, r.top(), 56, r.height()), Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight, tag)


class DocsDialog(QDialog):
    def __init__(self, anchor: str = "start", parent=None):
        super().__init__(parent)
        from PyQt6.QtWidgets import QTextBrowser
        self.setWindowTitle(f"{APP_NAME} - Documentation")
        self.resize(780, 620)
        self.setStyleSheet(CONSOLE_QSS + "QTextBrowser { background:#1E1E1E; border:1px solid #3c3c3f; border-radius:8px; padding:8px; }")
        lay = QVBoxLayout(self)
        view = QTextBrowser()
        view.setOpenExternalLinks(True)
        view.setHtml(DOCS_HTML)
        lay.addWidget(view)
        btn = QPushButton("Close")
        btn.setObjectName("primary")
        btn.clicked.connect(self.accept)
        row = QHBoxLayout()
        row.addStretch(1)
        row.addWidget(btn)
        lay.addLayout(row)
        QTimer.singleShot(0, lambda: view.scrollToAnchor(anchor))


class LandingPage(QWidget):
    launchConsole = pyqtSignal()
    launchDemo = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("landing")
        self.setStyleSheet(LANDING_QSS)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(36, 30, 36, 30)
        card = QFrame()
        card.setObjectName("lcard")
        outer.addWidget(card)
        cv = QVBoxLayout(card)
        cv.setContentsMargins(0, 0, 0, 0)
        cv.setSpacing(0)

        # --- nav ---
        nav = QWidget()
        nav.setObjectName("nav")
        nav.setFixedHeight(54)
        nl = QHBoxLayout(nav)
        nl.setContentsMargins(30, 0, 22, 0)
        nl.setSpacing(18)
        nl.addWidget(LogoMark(22))
        brand = QLabel(APP_NAME)
        brand.setObjectName("brand")
        nl.addWidget(brand)
        slash = QLabel("/")
        slash.setStyleSheet("color:#9AA4B2;")
        nl.addWidget(slash)
        for text, anchor in (("DOCS", "start"), ("MODES", "modes"), ("HARDWARE", "hardware"),
                             ("PROTOCOL", "protocol"), ("ABOUT", "about")):
            lbl = QLabel(text)
            lbl.setObjectName("navlink")
            lbl.setCursor(Qt.CursorShape.PointingHandCursor)
            lbl.mousePressEvent = lambda _e, a=anchor: self.show_docs(a)
            nl.addWidget(lbl)
        nl.addStretch(1)
        b_demo = QPushButton("Try demo")
        b_demo.setObjectName("navcta")
        b_demo.setCursor(Qt.CursorShape.PointingHandCursor)
        b_demo.clicked.connect(self.launchDemo.emit)
        b_con = QPushButton("Launch console")
        b_con.setObjectName("dark")
        b_con.setCursor(Qt.CursorShape.PointingHandCursor)
        b_con.clicked.connect(self.launchConsole.emit)
        nl.addWidget(b_demo)
        nl.addWidget(b_con)
        cv.addWidget(nav)

        # --- body ---
        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(64, 0, 24, 36)
        lv.setSpacing(0)
        lv.addStretch(2)
        self.pill = QLabel("<span style='color:#0A84FF'>●</span>&nbsp;&nbsp;Open hardware &nbsp;·&nbsp; PyQt6 &nbsp;·&nbsp; PySerial")
        self.pill.setTextFormat(Qt.TextFormat.RichText)
        self.pill.setObjectName("lpill")
        row = QHBoxLayout()
        row.addWidget(self.pill)
        row.addStretch(1)
        lv.addLayout(row)
        lv.addSpacing(18)
        self.headline = QLabel(
            "<p style='line-height:78%; margin:0; font-size:46px; font-weight:600; letter-spacing:-1.5px;'>"
            "Logic circuits<br>that minimise<br>themselves.</p>")
        self.headline.setTextFormat(Qt.TextFormat.RichText)
        self.headline.setWordWrap(True)
        lv.addWidget(self.headline)
        lv.addSpacing(18)
        self.sub = QLabel("Interactive K-Map, gate and memory trainer for your serial-connected\n"
                          "board. Minimise, simulate and watch it run live on hardware.")
        self.sub.setObjectName("sub")
        self.sub.setWordWrap(True)
        lv.addWidget(self.sub)
        lv.addSpacing(26)
        brow = QHBoxLayout()
        brow.setSpacing(12)
        self.cta = QPushButton("Launch Console")
        self.cta.setObjectName("cta")
        self.cta.setCursor(Qt.CursorShape.PointingHandCursor)
        self.cta.clicked.connect(self.launchConsole.emit)
        self.docs = QPushButton("View Docs  →")
        self.docs.setObjectName("outline")
        self.docs.setCursor(Qt.CursorShape.PointingHandCursor)
        self.docs.clicked.connect(lambda: self.show_docs("start"))
        brow.addWidget(self.cta)
        brow.addWidget(self.docs)
        brow.addStretch(1)
        lv.addLayout(brow)
        lv.addStretch(3)
        box = QFrame()
        box.setObjectName("lbox")
        bl = QVBoxLayout(box)
        bl.setContentsMargins(18, 16, 18, 16)
        bl.setSpacing(10)
        head = QLabel("✓  Every circuit simulated")
        head.setObjectName("chip")
        hr = QHBoxLayout()
        hr.addStretch(1)
        hr.addWidget(head)
        hr.addStretch(1)
        bl.addLayout(hr)
        chips = QHBoxLayout()
        chips.setSpacing(8)
        cap = QLabel("modes")
        cap.setObjectName("cap")
        chips.addWidget(cap)
        for t in ("K-Map", "Adder", "74138", "74153", "Shift Reg"):
            c = QLabel(t)
            c.setObjectName("chip")
            chips.addWidget(c)
        chips.addStretch(1)
        bl.addLayout(chips)
        lv.addWidget(box)
        self.hero = HeroCanvas()
        body.addWidget(left, 11)
        body.addWidget(self.hero, 10)
        cv.addLayout(body, 1)

        self._fx_widgets = [self.pill, self.headline, self.sub, self.cta, self.docs, box]
        self._effects: List[QGraphicsOpacityEffect] = []
        for w in self._fx_widgets:
            eff = QGraphicsOpacityEffect(w)
            eff.setOpacity(0.0)
            w.setGraphicsEffect(eff)
            self._effects.append(eff)
        self._anims: List[QPropertyAnimation] = []
        self._played = False

    def showEvent(self, e) -> None:
        super().showEvent(e)
        if self._played:
            return
        self._played = True
        for i, eff in enumerate(self._effects):
            anim = QPropertyAnimation(eff, b"opacity", self)
            anim.setDuration(520)
            anim.setStartValue(0.0)
            anim.setEndValue(1.0)
            anim.setEasingCurve(QEasingCurve.Type.OutCubic)
            self._anims.append(anim)
            QTimer.singleShot(90 * i, anim.start)

    def show_docs(self, anchor: str) -> None:
        DocsDialog(anchor, self.window()).exec()


# ======================================================================================
#  CONSOLE PAGE  (controller + all panels)
# ======================================================================================
def make_card(title: str = "") -> Tuple[QFrame, QVBoxLayout]:
    f = QFrame()
    f.setObjectName("card")
    lay = QVBoxLayout(f)
    lay.setContentsMargins(14, 12, 14, 14)
    lay.setSpacing(8)
    if title:
        h = QLabel(title.upper())
        h.setObjectName("h")
        lay.addWidget(h)
    return f, lay


def swatch_icon(color: str):
    from PyQt6.QtGui import QIcon, QPixmap
    pm = QPixmap(14, 14)
    pm.fill(Qt.GlobalColor.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.RenderHint.Antialiasing)
    p.setBrush(QColor(color))
    p.setPen(Qt.PenStyle.NoPen)
    p.drawRoundedRect(1, 1, 12, 12, 3, 3)
    p.end()
    return QIcon(pm)


def set_pill(label: QLabel, text: str, color: str) -> None:
    label.setText(text)
    label.setStyleSheet(f"QLabel {{ background: {qc(color, 28).name(QColor.NameFormat.HexArgb)}; "
                        f"border: 1px solid {color}; border-radius: 10px; padding: 2px 9px; color: {color}; "
                        f"font-size: 8.5pt; font-weight: 600; }}")


LIKELY_ADAPTERS = ("arduino", "ch340", "ch341", "cp210", "ftdi", "usb serial", "usb-serial", "silicon labs",
                   "wch", "esp32", "stm", "teensy", "pico")

MODE_DEFS = [("kmap", "K-MAP / SOP"), ("adder", "ADDER · SUB"), ("decoder", "74138 DECODER"),
             ("mux", "74153 MUX"), ("sreg", "SHIFT REGISTER")]
TAB_KMAP, TAB_SCHEM, TAB_REG, TAB_DEV, TAB_TRACE = range(5)
MODE_TAB = {"kmap": TAB_KMAP, "adder": TAB_DEV, "decoder": TAB_DEV, "mux": TAB_DEV, "sreg": TAB_REG}

PRESETS = [
    ("-- load preset --", None, None),
    ("A'B + CD  (default)", lambda a, b, c, d: ((1 - a) & b) | (c & d), None),
    ("Odd parity (A xor B xor C xor D)", lambda a, b, c, d: a ^ b ^ c ^ d, None),
    ("Majority of A,B,C", lambda a, b, c, d: int(a + b + c >= 2), None),
    ("Prime numbers 0-15", lambda a, b, c, d: int(((a << 3) | (b << 2) | (c << 1) | d) in (2, 3, 5, 7, 11, 13)), None),
    ("BCD digit >= 5 (10-15 = X)", lambda a, b, c, d: int(5 <= ((a << 3) | (b << 2) | (c << 1) | d) <= 9),
     lambda a, b, c, d: int(((a << 3) | (b << 2) | (c << 1) | d) >= 10)),
    ("7-seg 'a' segment (BCD, 10-15 = X)",
     lambda a, b, c, d: int(((a << 3) | (b << 2) | (c << 1) | d) in (0, 2, 3, 5, 6, 7, 8, 9)),
     lambda a, b, c, d: int(((a << 3) | (b << 2) | (c << 1) | d) >= 10)),
    ("Cyclic chart (Petrick demo)", lambda a, b, c, d: int(((a << 3) | (b << 2) | (c << 1) | d) in (0, 1, 2, 5, 6, 7)), None),
    ("Wrap-around corners", lambda a, b, c, d: int(((a << 3) | (b << 2) | (c << 1) | d) in (0, 2, 8, 10)), None),
    ("Checkerboard", lambda a, b, c, d: int(bin((a << 3) | (b << 2) | (c << 1) | d).count("1") % 2 == 0), None),
]


class ConsolePage(QWidget):
    goHome = pyqtSignal()

    # ---------------------------------------------------------------- construction
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("console")
        self.setStyleSheet(CONSOLE_QSS)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)

        self.qm = QuineMcCluskey()
        self.reg = ShiftRegister()
        self.result: Optional[MinimizationResult] = None
        self.worker: Optional[SerialWorker] = None
        self.conn_state = "idle"                 # idle | connecting | connected
        self.hw = [0, 0, 0, 0]
        self.manual = [0, 0, 0, 0]
        self.prev_inputs = [0, 0, 0, 0]
        self.mode = "kmap"
        self._user_disc = False
        self._port_entries: Optional[List[Tuple[str, str]]] = None
        self._sweep_idx = 0
        self._adder_sub = False
        self._rx_prev = (0, time.monotonic())
        self._rx_fps = 0.0
        self._last_cmd = ""
        self._warned_offline = False
        self._shut = False
        self._hw_seen = False                    # has the board sent a valid frame on this connection?
        self._connected_at = 0.0
        self._silence_warned = False
        self._last_recover = 0.0
        self._board_up: Optional[int] = None
        self._board_restarts = 0
        self._recovering = False                 # silent reconnect in progress: keep the UI as it was
        self._pending_write: Optional[int] = None
        self._undo: List[Tuple[int, int]] = []

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self._build_topbar())
        root.addWidget(self._build_modebar())
        split = QSplitter(Qt.Orientation.Horizontal)
        split.setChildrenCollapsible(False)
        split.addWidget(self._build_left())
        split.addWidget(self._build_right())
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setSizes([440, 980])
        split.setContentsMargins(10, 10, 10, 6)
        root.addWidget(split, 1)
        root.addWidget(self._build_log())
        root.addWidget(self._build_status())

        from PyQt6.QtGui import QKeySequence, QShortcut
        for i, k in enumerate("ABCD"):
            QShortcut(QKeySequence(k), self, activated=lambda i=i: self.key_toggle(i))
        QShortcut(QKeySequence(Qt.Key.Key_Space), self, activated=self.software_clock)
        QShortcut(QKeySequence("Ctrl+L"), self, activated=self.log_view.clear)

        # timers
        self.scan_timer = QTimer(self)
        self.scan_timer.setInterval(2500)
        self.scan_timer.timeout.connect(self.scan_ports)
        self.stats_timer = QTimer(self)
        self.stats_timer.setInterval(250)
        self.stats_timer.timeout.connect(self._stats_tick)
        self.sweep_timer = QTimer(self)
        self.sweep_timer.timeout.connect(self._sweep_tick)
        self.clk_timer = QTimer(self)
        self.clk_timer.timeout.connect(self._autoclock_tick)
        self.write_timer = QTimer(self)
        self.write_timer.setSingleShot(True)
        self.write_timer.timeout.connect(self._commit_write)
        self.scan_timer.start()
        self.stats_timer.start()

        # board polarity / LED source - remembered between runs
        self.settings = QSettings("GateLab", "GateLab")
        self.cb_inv_sw.setChecked(self.settings.value("invert_switches", False, type=bool))
        self.cb_inv_led.setChecked(self.settings.value("invert_leds", False, type=bool))
        self.cb_led_src.setCurrentIndex(max(0, self.cb_led_src.findData(self.settings.value("led_source", "mode"))))
        self.cb_inv_sw.toggled.connect(self._on_invert_switches)
        self.cb_inv_led.toggled.connect(lambda on: (self.settings.setValue("invert_leds", on), self.refresh_outputs()))
        self.cb_led_src.currentIndexChanged.connect(
            lambda _i: (self.settings.setValue("led_source", self.cb_led_src.currentData()), self.refresh_outputs()))

        # initial state: F = A'B + CD
        self.apply_preset_fn(PRESETS[1][1], None, announce=False)
        self.set_mode("kmap")
        self.update_input_widgets()
        self.sync_reg_ext()
        self.refresh_views()
        self.log("INFO", f"{APP_NAME} ready - baud {BAUD_RATE} 8N1. No board? Use the software switches or Auto-sweep.")
        QTimer.singleShot(200, self.scan_ports)

    # ---- top bars -------------------------------------------------------------------
    def _build_topbar(self) -> QWidget:
        top = QWidget()
        top.setObjectName("topbar")
        h = QHBoxLayout(top)
        h.setContentsMargins(14, 8, 14, 8)
        h.setSpacing(10)
        self.btn_home = QPushButton("←  Home")
        self.btn_home.setObjectName("ghost")
        self.btn_home.clicked.connect(self.goHome.emit)
        h.addWidget(self.btn_home)
        h.addSpacing(6)
        h.addWidget(LogoMark(24))
        name = QLabel(APP_NAME)
        name.setFont(sans(13, True))
        h.addWidget(name)
        tag = QLabel(APP_TAGLINE)
        tag.setObjectName("pill")
        h.addWidget(tag)
        h.addStretch(1)

        lab = QLabel("PORT")
        lab.setObjectName("h")
        h.addWidget(lab)
        self.cb_port = QComboBox()
        self.cb_port.setMinimumWidth(250)
        self.cb_port.setToolTip("Serial port (enabled in MANUAL mode)")
        h.addWidget(self.cb_port)
        self.btn_refresh = QPushButton("⟳")
        self.btn_refresh.setFixedWidth(36)
        self.btn_refresh.setToolTip("Rescan ports")
        self.btn_refresh.clicked.connect(self.scan_ports)
        h.addWidget(self.btn_refresh)
        self.sw_auto = ToggleSwitch(True, QSize(44, 22))
        self.sw_auto.userToggled.connect(self._on_auto_toggled)
        h.addWidget(self.sw_auto)
        self.lbl_auto = QLabel("AUTO")
        self.lbl_auto.setObjectName("h")
        self.lbl_auto.setMinimumWidth(48)
        h.addWidget(self.lbl_auto)
        baud = QLabel(f"{BAUD_RATE} · 8N1")
        baud.setObjectName("pill")
        h.addWidget(baud)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setObjectName("primary")
        self.btn_connect.setMinimumWidth(104)
        self.btn_connect.clicked.connect(self.toggle_connection)
        h.addWidget(self.btn_connect)
        self.led_hw = StatusLed(Theme.DEAD, 16)
        h.addWidget(self.led_hw)
        self.lbl_hw = QLabel("HW LINK  idle")
        self.lbl_hw.setObjectName("mono")
        self.lbl_hw.setMinimumWidth(170)
        h.addWidget(self.lbl_hw)
        self.cb_port.setEnabled(False)           # AUTO is the default; the first scan runs after the UI is built
        return top

    def _build_modebar(self) -> QWidget:
        bar = QWidget()
        bar.setObjectName("modebar")
        h = QHBoxLayout(bar)
        h.setContentsMargins(14, 6, 14, 6)
        h.setSpacing(6)
        self.mode_group = QButtonGroup(self)
        self.mode_group.setExclusive(True)
        self.mode_buttons: Dict[str, QPushButton] = {}
        for key, text in MODE_DEFS:
            b = QPushButton(text)
            b.setObjectName("mode")
            b.setCheckable(True)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            b.clicked.connect(lambda _c, k=key: self.set_mode(k))
            self.mode_group.addButton(b)
            self.mode_buttons[key] = b
            h.addWidget(b)
        h.addStretch(1)
        hint = QLabel("keys:  A B C D toggle switches   ·   Space = clock   ·   Ctrl+L clears log")
        hint.setObjectName("pill")
        h.addWidget(hint)
        return bar

    # ---- left panel -------------------------------------------------------------------
    def _build_left(self) -> QWidget:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(410)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        inner = QWidget()
        v = QVBoxLayout(inner)
        v.setContentsMargins(0, 0, 8, 0)
        v.setSpacing(10)

        # inputs
        card, lay = make_card("Input switches  A · B · C · D")
        row = QHBoxLayout()
        row.setSpacing(8)
        self.sw: List[ToggleSwitch] = []
        self.sw_val: List[QLabel] = []
        for i in range(4):
            col = QVBoxLayout()
            col.setSpacing(3)
            lab = QLabel(VARS[i])
            lab.setFont(mono(17, True))
            lab.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lab.setStyleSheet(f"color:{Theme.CYAN_L};")
            sw = ToggleSwitch(False, QSize(56, 28))
            sw.userToggled.connect(lambda v_, i=i: self.on_manual_toggle(i, v_))
            val = QLabel("0")
            val.setFont(mono(11, True))
            val.setAlignment(Qt.AlignmentFlag.AlignCenter)
            col.addWidget(lab)
            col.addWidget(sw, 0, Qt.AlignmentFlag.AlignCenter)
            col.addWidget(val)
            row.addLayout(col)
            self.sw.append(sw)
            self.sw_val.append(val)
        lay.addLayout(row)
        sr = QHBoxLayout()
        self.lbl_state = QLabel()
        self.lbl_state.setObjectName("mono")
        self.lbl_source = QLabel()
        sr.addWidget(self.lbl_state)
        sr.addStretch(1)
        sr.addWidget(self.lbl_source)
        lay.addLayout(sr)
        self.cb_override = CheckBox("Manual override (until a board switch is moved)")
        self.cb_override.toggled.connect(lambda _c: (self.update_input_widgets(), self.on_inputs_changed(force=True)))
        lay.addWidget(self.cb_override)
        sweep = QHBoxLayout()
        self.cb_sweep = CheckBox("Auto-sweep 0 → 15 (virtual board)")
        self.cb_sweep.toggled.connect(self._on_sweep_toggled)
        self.sp_sweep = QDoubleSpinBox()
        self.sp_sweep.setRange(0.2, 12.0)
        self.sp_sweep.setValue(1.5)
        self.sp_sweep.setSingleStep(0.5)
        self.sp_sweep.setSuffix(" Hz")
        self.sp_sweep.valueChanged.connect(lambda hz: self.sweep_timer.setInterval(int(1000 / hz)))
        sweep.addWidget(self.cb_sweep, 1)
        sweep.addWidget(self.sp_sweep)
        lay.addLayout(sweep)
        v.addWidget(card)

        # mode pages
        self.pages = QStackedWidget()
        self.pages.addWidget(self._build_kmap_page())
        self.pages.addWidget(self._build_adder_page())
        self.pages.addWidget(self._build_decoder_page())
        self.pages.addWidget(self._build_mux_page())
        self.pages.addWidget(self._build_sreg_page())
        v.addWidget(self.pages)

        # truth table
        card, lay = make_card("Truth table")
        self.lbl_table_hint = QLabel()
        self.lbl_table_hint.setObjectName("pill")
        lay.addWidget(self.lbl_table_hint)
        self.tbl = TruthTable()
        self.tbl.cellClicked.connect(self.on_table_click)
        lay.addWidget(self.tbl)
        v.addWidget(card)
        v.addStretch(1)
        scroll.setWidget(inner)
        return scroll

    def _build_kmap_page(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(10)
        card, lay = make_card("Switches write into the K-map")
        r = QHBoxLayout()
        r.addWidget(QLabel("Action"))
        self.cb_write = QComboBox()
        for text, key in (("Write 1 into the selected cell", "1"), ("Toggle the cell 0 / 1", "t"),
                          ("Write X (don't care)", "X"), ("Write 0", "0"), ("Only point - don't write", "-")):
            self.cb_write.addItem(text, key)
        self.cb_write.currentIndexChanged.connect(lambda _i: self._cancel_write())
        r.addWidget(self.cb_write, 1)
        lay.addLayout(r)
        r = QHBoxLayout()
        r.addWidget(QLabel("Write after switches are still for"))
        self.sp_settle = QSpinBox()
        self.sp_settle.setRange(0, 5000)
        self.sp_settle.setSingleStep(100)
        self.sp_settle.setValue(2000)
        self.sp_settle.setSuffix(" ms")
        self.sp_settle.setToolTip("Gives you time to set several switches: only the cell they point to once they "
                                  "stop moving is written. 0 = write instantly.")
        r.addWidget(self.sp_settle)
        lay.addLayout(r)
        r = QHBoxLayout()
        self.btn_undo = QPushButton("Undo last write")
        self.btn_undo.clicked.connect(self.undo_write)
        self.btn_undo.setEnabled(False)
        r.addWidget(self.btn_undo)
        r.addStretch(1)
        lay.addLayout(r)
        hint = QLabel("Set A B C D on the board (switch 1 = A ... 4 = D). The highlighted cell shows where they "
                      "point; the amber ring fills, then the value is written and the expression, groups, "
                      "schematic, truth table, LEDs and LCD all update.")
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color:{Theme.MUTED}; font-size:9pt;")
        lay.addWidget(hint)
        v.addWidget(card)
        card, lay = make_card("Minimised expression")
        self.lbl_expr = QLabel("F = 0")
        self.lbl_expr.setObjectName("expr")
        self.lbl_expr.setWordWrap(True)
        self.lbl_expr.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.lbl_sigma = QLabel()
        self.lbl_sigma.setObjectName("mono")
        self.lbl_sigma.setWordWrap(True)
        self.lbl_sigma.setStyleSheet(f"color:{Theme.MUTED}; font-size:9pt;")
        self.lbl_canon = QLabel()
        self.lbl_canon.setObjectName("mono")
        self.lbl_canon.setWordWrap(True)
        self.lbl_canon.setStyleSheet(f"color:{Theme.MUTED}; font-size:8.5pt;")
        lay.addWidget(self.lbl_expr)
        lay.addWidget(self.lbl_sigma)
        lay.addWidget(self.lbl_canon)
        v.addWidget(card)

        card, lay = make_card("Term sets  (0-15, ranges like 4-7 ok)")
        self.ed_m, self.ed_d, self.ed_M = QLineEdit(), QLineEdit(), QLineEdit()
        for ed, ph, lab in ((self.ed_m, "minterms  e.g. 1,3,5-7", "m"), (self.ed_d, "don't cares  e.g. 10-15", "d"),
                            (self.ed_M, "maxterms (POS input)  e.g. 0,2", "M")):
            r = QHBoxLayout()
            lb = QLabel(lab)
            lb.setFont(mono(12, True))
            lb.setFixedWidth(16)
            ed.setObjectName("mono")
            ed.setPlaceholderText(ph)
            ed.returnPressed.connect(self.apply_fields)
            r.addWidget(lb)
            r.addWidget(ed, 1)
            lay.addLayout(r)
        br = QHBoxLayout()
        b_apply = QPushButton("Apply")
        b_apply.setObjectName("primary")
        b_apply.clicked.connect(self.apply_fields)
        b_clear = QPushButton("Clear")
        b_clear.clicked.connect(lambda: self.apply_cells([0] * 16, "K-map cleared"))
        b_rand = QPushButton("Random")
        b_rand.clicked.connect(self.randomize)
        b_inv = QPushButton("Invert")
        b_inv.clicked.connect(self.invert_cells)
        for b in (b_apply, b_clear, b_rand, b_inv):
            br.addWidget(b)
        lay.addLayout(br)
        self.cb_preset = QComboBox()
        for name, _f, _d in PRESETS:
            self.cb_preset.addItem(name)
        self.cb_preset.currentIndexChanged.connect(self._on_preset)
        lay.addWidget(self.cb_preset)
        v.addWidget(card)

        card, lay = make_card("Prime implicants & cover")
        self.list_pi = QListWidget()
        self.list_pi.setFixedHeight(150)
        self.list_pi.currentRowChanged.connect(lambda r: self.kmap.set_focus(r))
        lay.addWidget(self.list_pi)
        self.cb_allpi = CheckBox("Show all prime implicants on the map")
        self.cb_allpi.toggled.connect(self._on_allpi)
        lay.addWidget(self.cb_allpi)
        v.addWidget(card)
        return w

    def _build_adder_page(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        card, lay = make_card("Full adder / subtractor")
        seg = QHBoxLayout()
        seg.setSpacing(0)
        self.btn_add = QPushButton("Full Adder")
        self.btn_sub = QPushButton("Full Subtractor")
        grp = QButtonGroup(self)
        for b in (self.btn_add, self.btn_sub):
            b.setObjectName("seg")
            b.setCheckable(True)
            grp.addButton(b)
            seg.addWidget(b)
        self.btn_add.setChecked(True)
        self.btn_add.clicked.connect(lambda: self._set_adder_sub(False))
        self.btn_sub.clicked.connect(lambda: self._set_adder_sub(True))
        lay.addLayout(seg)
        self.lbl_adder_eq = QLabel()
        self.lbl_adder_eq.setFont(mono(15, True))
        self.lbl_adder_eq.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_adder_eq.setStyleSheet(f"color:{Theme.EMERALD};")
        lay.addWidget(self.lbl_adder_eq)
        self.lbl_adder_out = QLabel()
        self.lbl_adder_out.setObjectName("mono")
        self.lbl_adder_out.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(self.lbl_adder_out)
        note = QLabel("Inputs:  A = A,  B = B,  Cin/Bin = C   (D unused)")
        note.setObjectName("pill")
        lay.addWidget(note)
        r = QHBoxLayout()
        self.btn_load_s = QPushButton("Sum / Diff → K-Map")
        self.btn_load_c = QPushButton("Carry / Borrow → K-Map")
        self.btn_load_s.clicked.connect(lambda: self._load_adder(0))
        self.btn_load_c.clicked.connect(lambda: self._load_adder(1))
        r.addWidget(self.btn_load_s)
        r.addWidget(self.btn_load_c)
        lay.addLayout(r)
        v.addWidget(card)
        return w

    def _mini_toggle(self, text: str, init: bool, cb: Callable[[bool], None]) -> Tuple[QWidget, ToggleSwitch]:
        w = QWidget()
        l = QVBoxLayout(w)
        l.setContentsMargins(0, 0, 0, 0)
        l.setSpacing(2)
        lab = QLabel(text)
        lab.setObjectName("mono")
        lab.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sw = ToggleSwitch(init, QSize(44, 22))
        sw.userToggled.connect(cb)
        l.addWidget(lab)
        l.addWidget(sw, 0, Qt.AlignmentFlag.AlignCenter)
        return w, sw

    def _build_decoder_page(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        card, lay = make_card("74138  3-to-8 decoder")
        er = QHBoxLayout()
        self.tg_dec: List[ToggleSwitch] = []
        for text, init in (("G1", True), ("/G2A", False), ("/G2B", False)):
            wd, sw = self._mini_toggle(text, init, lambda _v: self._dec_changed())
            self.tg_dec.append(sw)
            er.addWidget(wd)
        lay.addLayout(er)
        self.lbl_dec_en = QLabel()
        lay.addWidget(self.lbl_dec_en, 0, Qt.AlignmentFlag.AlignCenter)
        grid = QHBoxLayout()
        grid.setSpacing(4)
        self.dec_y: List[QLabel] = []
        for i in range(8):
            lb = QLabel(f"Y{i}")
            lb.setObjectName("pill")
            lb.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid.addWidget(lb)
            self.dec_y.append(lb)
        lay.addLayout(grid)
        note = QLabel("Selects: A = LSB, B, C = MSB.  Enabled when G1=1, /G2A=0, /G2B=0.")
        note.setObjectName("pill")
        note.setWordWrap(True)
        lay.addWidget(note)
        r = QHBoxLayout()
        self.cb_dec_y = QComboBox()
        self.cb_dec_y.addItems([f"Y{i}" for i in range(8)])
        b = QPushButton("Load selected Yn → K-Map")
        b.clicked.connect(self._load_decoder)
        r.addWidget(self.cb_dec_y)
        r.addWidget(b, 1)
        lay.addLayout(r)
        v.addWidget(card)
        return w

    def _build_mux_page(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        card, lay = make_card("74153  dual 4-to-1 multiplexer")
        self.mux_data = [[0] * 4, [0] * 4]
        self.mux_g = [0, 0]
        self.tg_mux: List[List[ToggleSwitch]] = [[], []]
        self.tg_mux_g: List[ToggleSwitch] = []
        for m in range(2):
            t = QLabel(f"MUX {m + 1}  (data inputs {m + 1}C0..{m + 1}C3, strobe /{m + 1}G)")
            t.setObjectName("h")
            lay.addWidget(t)
            r = QHBoxLayout()
            for k in range(4):
                wd, sw = self._mini_toggle(f"{m + 1}C{k}", False, lambda v_, m=m, k=k: self._mux_data(m, k, v_))
                self.tg_mux[m].append(sw)
                r.addWidget(wd)
            wd, sw = self._mini_toggle(f"/{m + 1}G", False, lambda v_, m=m: self._mux_strobe(m, v_))
            self.tg_mux_g.append(sw)
            r.addWidget(wd)
            lay.addLayout(r)
        self.lbl_mux = QLabel()
        self.lbl_mux.setFont(mono(13, True))
        self.lbl_mux.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_mux.setStyleSheet(f"color:{Theme.EMERALD};")
        lay.addWidget(self.lbl_mux)
        note = QLabel("Selects: S0 = A, S1 = B.  Strobe is active-low.")
        note.setObjectName("pill")
        lay.addWidget(note)
        r = QHBoxLayout()
        b1 = QPushButton("1Y → K-Map")
        b2 = QPushButton("2Y → K-Map")
        b1.clicked.connect(lambda: self._load_mux(0))
        b2.clicked.connect(lambda: self._load_mux(1))
        r.addWidget(b1)
        r.addWidget(b2)
        lay.addLayout(r)
        v.addWidget(card)
        return w

    def _build_sreg_page(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(10)
        card, lay = make_card("4-bit shift register  (7474 x4)")
        r = QHBoxLayout()
        r.addWidget(QLabel("Mode"))
        self.cb_sr_mode = QComboBox()
        for m, desc in (("SISO", "SISO - serial in / serial out"), ("SIPO", "SIPO - serial in / parallel out"),
                        ("PISO", "PISO - parallel in / serial out"), ("PIPO", "PIPO - parallel in / parallel out")):
            self.cb_sr_mode.addItem(desc, m)
        self.cb_sr_mode.currentIndexChanged.connect(self._sr_mode_changed)
        r.addWidget(self.cb_sr_mode, 1)
        lay.addLayout(r)
        self.cb_follow = CheckBox("Data follows switches  (SI = A,  P0..P3 = A..D)")
        self.cb_follow.setChecked(True)
        self.cb_follow.toggled.connect(lambda _c: (self.update_sreg_widgets(), self.sync_reg_ext(), self.refresh_views()))
        lay.addWidget(self.cb_follow)
        dr = QHBoxLayout()
        wd, self.tg_si = self._mini_toggle("SI", False, lambda _v: self._sreg_panel_changed())
        dr.addWidget(wd)
        self.tg_p: List[ToggleSwitch] = []
        for i in range(4):
            wd, sw = self._mini_toggle(f"P{i}", False, lambda _v: self._sreg_panel_changed())
            self.tg_p.append(sw)
            dr.addWidget(wd)
        wd, self.tg_load = self._mini_toggle("LD/SH", True, lambda _v: self._sreg_panel_changed())
        dr.addWidget(wd)
        lay.addLayout(dr)
        v.addWidget(card)

        card, lay = make_card("Clock")
        r = QHBoxLayout()
        r.addWidget(QLabel("Source"))
        self.cb_clk_src = QComboBox()
        self.cb_clk_src.addItem("Button / Space / Auto only")
        self.cb_clk_src.addItem("Board switch D (OFF \u2192 ON) + button")
        self.cb_clk_src.setCurrentIndex(1)
        self.cb_clk_src.currentIndexChanged.connect(lambda _i: (self.sync_reg_ext(), self.update_sreg_widgets()))
        r.addWidget(self.cb_clk_src, 1)
        lay.addLayout(r)
        r = QHBoxLayout()
        self.btn_clk = QPushButton("⏵  CLOCK  ↑")
        self.btn_clk.setObjectName("primary")
        self.btn_clk.setMinimumHeight(38)
        self.btn_clk.clicked.connect(self.software_clock)
        r.addWidget(self.btn_clk, 1)
        self.cb_autoclk = CheckBox("Auto")
        self.cb_autoclk.toggled.connect(self._on_autoclk)
        self.sp_clk = QDoubleSpinBox()
        self.sp_clk.setRange(0.2, 8.0)
        self.sp_clk.setValue(1.0)
        self.sp_clk.setSuffix(" Hz")
        self.sp_clk.valueChanged.connect(lambda hz: self.clk_timer.setInterval(int(1000 / hz)))
        r.addWidget(self.cb_autoclk)
        r.addWidget(self.sp_clk)
        lay.addLayout(r)
        r = QHBoxLayout()
        r.addWidget(QLabel("Setup window"))
        self.sp_setup = QSpinBox()
        self.sp_setup.setRange(0, 1000)
        self.sp_setup.setSuffix(" ms")
        self.sp_setup.setToolTip("If a data input changes this shortly before the edge, the flip-flops capture the "
                                 "OLD value and a SETUP violation is logged. 0 = ideal flip-flops.")
        self.sp_setup.valueChanged.connect(lambda ms: setattr(self.reg, "setup_s", ms / 1000.0))
        r.addWidget(self.sp_setup)
        r.addStretch(1)
        lay.addLayout(r)
        r = QHBoxLayout()
        b1, b2, b3 = QPushButton("Clear (/CLR)"), QPushButton("Preset (/PRE)"), QPushButton("Reset history")
        b1.clicked.connect(lambda: self._sreg_async("clear"))
        b2.clicked.connect(lambda: self._sreg_async("preset"))
        b3.clicked.connect(lambda: self._sreg_async("history"))
        for b in (b1, b2, b3):
            r.addWidget(b)
        lay.addLayout(r)
        hint = QLabel("With the board: switch A is the serial data (SI). Flip switch D OFF \u2192 ON to clock. "
                      "In PISO/PIPO, switches A-C are P0-P2 and P3 comes from the P3 toggle above. "
                      "The board LEDs show Q0-Q3.")
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color:{Theme.MUTED}; font-size:9pt;")
        lay.addWidget(hint)
        self.lbl_sreg = QLabel()
        self.lbl_sreg.setFont(mono(16, True))
        self.lbl_sreg.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_sreg.setStyleSheet(f"color:{Theme.EMERALD};")
        lay.addWidget(self.lbl_sreg)
        v.addWidget(card)
        return w

    # ---- right panel ------------------------------------------------------------------
    def _build_right(self) -> QWidget:
        self.tabs = QTabWidget()
        self.kmap = KMapWidget()
        self.kmap.cellChanged.connect(self.on_cell_changed)
        self.schem = SchematicCanvas()
        self.reg_canvas = RegisterCanvas(self.reg)
        self.dev = DeviceCanvas()
        self.trace_view = QPlainTextEdit()
        self.trace_view.setReadOnly(True)
        self.trace_view.setFont(mono(10))
        self.tabs.addTab(self.kmap, "K-MAP VISUALIZER")
        self.tabs.addTab(self.schem, "LOGIC SCHEMATIC")
        self.tabs.addTab(self.reg_canvas, "REGISTER TIMING")
        self.tabs.addTab(self.dev, "DEVICE VIEW")
        self.tabs.addTab(self.trace_view, "QM TRACE")
        return self.tabs

    # ---- log + status -----------------------------------------------------------------
    def _build_log(self) -> QWidget:
        wrap = QWidget()
        wrap.setStyleSheet(f"background:{Theme.CHARCOAL};")
        row = QHBoxLayout(wrap)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(0)

        box = QFrame()
        box.setObjectName("card")
        box.setStyleSheet("QFrame#card { border-radius:0; border-left:none; border-right:none; }")
        v = QVBoxLayout(box)
        v.setContentsMargins(12, 6, 12, 6)
        v.setSpacing(4)
        h = QHBoxLayout()
        t = QLabel("DIAGNOSTIC CONSOLE")
        t.setObjectName("h")
        h.addWidget(t)
        h.addStretch(1)
        self.btn_log_clear = QPushButton("Clear")
        self.btn_log_clear.setObjectName("ghost")
        self.btn_log_toggle = QPushButton("Hide")
        self.btn_log_toggle.setObjectName("ghost")
        for b in (self.btn_log_clear, self.btn_log_toggle):
            b.setStyleSheet("padding:2px 10px;")
            h.addWidget(b)
        v.addLayout(h)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(700)
        self.log_view.setFixedHeight(92)
        v.addWidget(self.log_view)
        self.btn_log_clear.clicked.connect(self.log_view.clear)
        self.btn_log_toggle.clicked.connect(self._toggle_log)
        row.addWidget(box, 1)

        prev = QFrame()
        prev.setObjectName("card")
        prev.setStyleSheet("QFrame#card { border-radius:0; border-right:none; }")
        pv = QVBoxLayout(prev)
        pv.setContentsMargins(14, 6, 14, 8)
        pv.setSpacing(4)
        ph = QLabel("BOARD PREVIEW  (what the trainer shows)")
        ph.setObjectName("h")
        pv.addWidget(ph)
        body = QHBoxLayout()
        body.setSpacing(14)
        self.led_out: List[StatusLed] = []
        for i in range(4):
            col = QVBoxLayout()
            col.setSpacing(2)
            led = StatusLed(Theme.DEAD, 20)
            lb = QLabel(f"L{i}")
            lb.setObjectName("mono")
            lb.setAlignment(Qt.AlignmentFlag.AlignCenter)
            col.addStretch(1)
            col.addWidget(led, 0, Qt.AlignmentFlag.AlignCenter)
            col.addWidget(lb)
            col.addStretch(1)
            body.addLayout(col)
            self.led_out.append(led)
        self.lcd = QLabel()
        self.lcd.setFont(mono(13, True))
        self.lcd.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lcd.setStyleSheet("QLabel { background:#9DBF6B; color:#1D2B10; border:3px solid #3b4a2a; "
                               "border-radius:6px; padding:6px 10px; }")
        body.addWidget(self.lcd)
        pv.addLayout(body, 1)
        self.lbl_led_map = QLabel()
        self.lbl_led_map.setObjectName("mono")
        self.lbl_led_map.setStyleSheet(f"color:{Theme.MUTED}; font-size:8.5pt;")
        pv.addWidget(self.lbl_led_map)
        opts = QHBoxLayout()
        self.cb_led_src = QComboBox()
        self.cb_led_src.addItem("LEDs: mode outputs", "mode")
        self.cb_led_src.addItem("LEDs: mirror switches A-D", "mirror")
        self.cb_inv_sw = CheckBox("Invert switches")
        self.cb_inv_led = CheckBox("Invert LEDs")
        self.cb_inv_sw.setToolTip("Tick if the app shows 1 while a board switch is OFF")
        self.cb_inv_led.setToolTip("Tick if a board LED is dark while the preview shows it lit")
        opts.addWidget(self.cb_led_src)
        opts.addWidget(self.cb_inv_sw)
        opts.addWidget(self.cb_inv_led)
        pv.addLayout(opts)
        prev.setFixedWidth(520)
        row.addWidget(prev)
        return wrap

    def _toggle_log(self) -> None:
        vis = not self.log_view.isVisible()
        self.log_view.setVisible(vis)
        self.btn_log_toggle.setText("Hide" if vis else "Show")

    def _build_status(self) -> QWidget:
        bar = QWidget()
        bar.setObjectName("topbar")
        bar.setStyleSheet(f"QWidget#topbar {{ background:{Theme.CHARCOAL}; border-top:1px solid {Theme.BORDER}; border-bottom:none; }}")
        h = QHBoxLayout(bar)
        h.setContentsMargins(14, 5, 14, 5)
        h.setSpacing(14)
        self.led_conn = StatusLed(Theme.RED, 16)
        h.addWidget(self.led_conn)
        self.lbl_conn = QLabel("Disconnected")
        self.lbl_conn.setMinimumWidth(240)
        h.addWidget(self.lbl_conn)
        self.lbl_tx = QLabel("TX 0 frames · 0 B")
        self.lbl_tx.setObjectName("mono")
        self.lbl_rx = QLabel("RX 0 ok · 0 bad · 0.0 fps")
        self.lbl_rx.setObjectName("mono")
        h.addWidget(self.lbl_tx)
        h.addWidget(self.lbl_rx)
        self.lbl_cmd = QLabel()
        self.lbl_cmd.setObjectName("mono")
        self.lbl_cmd.setStyleSheet(f"color:{Theme.CYAN_L};")
        self.lbl_cmd.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        h.addWidget(self.lbl_cmd, 1)
        return bar

    # ---------------------------------------------------------------- logging
    def log(self, level: str, msg: str) -> None:
        colors = {"INFO": Theme.MUTED, "OK": Theme.EMERALD, "WARN": Theme.AMBER, "ERR": Theme.RED,
                  "TX": Theme.CYAN_L, "RX": "#7fd1ff", "LOGIC": "#b9a3ff"}
        col = colors.get(level, Theme.TEXT)
        stamp = time.strftime("%H:%M:%S") + f".{int((time.time() % 1) * 1000):03d}"
        safe = msg.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        self.log_view.appendHtml(f"<span style='color:#6f7378'>{stamp}</span> "
                                 f"<span style='color:{col};font-weight:600'>{level:<5}</span> "
                                 f"<span style='color:#d4d4d4'>{safe}</span>")

    # ---------------------------------------------------------------- input model
    def _use_manual(self) -> bool:
        if self._recovering:
            return self.cb_override.isChecked() or self.cb_sweep.isChecked()
        return (self.conn_state != "connected" or not self._hw_seen or self.cb_override.isChecked()
                or self.cb_sweep.isChecked())

    def inputs(self) -> List[int]:
        return list(self.manual) if self._use_manual() else list(self.hw)

    def update_input_widgets(self) -> None:
        manual = self._use_manual()
        inp = self.inputs()
        for i in range(4):
            self.sw[i].setInteractive(True)          # clicking a hardware-driven switch takes manual control
            self.sw[i].setValue(bool(inp[i]))
            self.sw_val[i].setText(str(inp[i]))
            self.sw_val[i].setStyleSheet(f"color:{Theme.LIVE if inp[i] else '#777'};")
        idx = (inp[0] << 3) | (inp[1] << 2) | (inp[2] << 1) | inp[3]
        self.lbl_state.setText(f"ABCD={idx:04b}  m{idx}")
        if manual:
            if self.cb_sweep.isChecked():
                why = "sweep"
            elif self.cb_override.isChecked():
                why = "override"
            elif self.conn_state == "connected":
                why = "no frames yet"
            else:
                why = "no board"
            set_pill(self.lbl_source, f"SOFTWARE · {why}", Theme.AMBER)
        else:
            set_pill(self.lbl_source, "HARDWARE SWITCHES", Theme.EMERALD)

    def _take_manual_control(self) -> None:
        """A click on a switch while hardware (or the sweep) owns the inputs hands control to the software."""
        if self.cb_sweep.isChecked():
            self.cb_sweep.setChecked(False)
        if not self._use_manual():
            self.manual = list(self.hw)
            self.cb_override.setChecked(True)
            self.log("INFO", "manual override enabled (a software switch was clicked)")

    def _on_invert_switches(self, on: bool) -> None:
        self.settings.setValue("invert_switches", on)
        self.hw = [1 - v for v in self.hw]           # re-interpret the last frame immediately
        self.log("INFO", f"board switch polarity {'inverted' if on else 'normal'}")
        self.on_inputs_changed(force=True)

    def key_toggle(self, i: int) -> None:
        self._take_manual_control()
        self.manual[i] ^= 1
        self.on_inputs_changed(force=True, source="switch")

    def on_manual_toggle(self, i: int, v: bool) -> None:
        self._take_manual_control()
        self.manual[i] = int(v)
        self.on_inputs_changed(force=True, source="switch")

    def on_frame(self, frame: dict) -> None:
        new = [frame[k] for k in VARS]
        if self.cb_inv_sw.isChecked():
            new = [1 - v for v in new]
        self.led_hw.flash()
        up = frame.get("u")
        if up is not None:
            if self._board_up is not None and up < self._board_up:
                # the board restarted (a reset-line glitch on this clone): put LEDs/LCD back at once
                self._board_restarts += 1
                self.refresh_outputs(force=True)
            self._board_up = up
        if not self._hw_seen:
            self._hw_seen = True
            self.hw = new
            self.log("OK", "first valid frame from the board - hardware switches are now live")
            self.update_input_widgets()
            self.on_inputs_changed(force=True)
        elif new != self.hw:
            self.hw = new
            self.log("RX", f"board switches A={new[0]} B={new[1]} C={new[2]} D={new[3]}")
            if self.cb_override.isChecked() or self.cb_sweep.isChecked():
                # last touched wins: a physical switch takes control back from the software
                for cb in (self.cb_override, self.cb_sweep):
                    cb.blockSignals(True)
                    cb.setChecked(False)
                    cb.blockSignals(False)
                self.sweep_timer.stop()
                self.log("INFO", "board switch moved - hardware switches are back in control")
            self.on_inputs_changed(force=True, source="switch")

    def on_inputs_changed(self, force: bool = False, source: Optional[str] = None) -> None:
        """source == "switch" means a person flipped a switch (board or on-screen); only that writes
        into the K-map - sweeps, mode changes and the first frame after connecting only point."""
        inp = self.inputs()
        if inp == self.prev_inputs and not force:
            return
        prev = self.prev_inputs
        self.prev_inputs = list(inp)
        self.update_input_widgets()
        if source == "switch" and inp != prev:
            self._arm_write((inp[0] << 3) | (inp[1] << 2) | (inp[2] << 1) | inp[3])
        if self.mode == "sreg" and self.cb_clk_src.currentIndex() == 1 and inp[3] != prev[3]:
            rec = self.reg.clock_level(inp[3])
            if rec is not None:
                self._after_edge(rec, "hardware D↑")
        self.sync_reg_ext()
        self.refresh_views()

    # ---------------------------------------------------------------- switches -> K-map writes
    def _arm_write(self, idx: int) -> None:
        action = self.cb_write.currentData()
        if self.mode != "kmap" or action == "-":
            self._cancel_write()
            return
        self._pending_write = idx
        settle = self.sp_settle.value()
        if settle == 0:
            self._commit_write()
            return
        glyph = {"1": "1", "0": "0", "X": "X", "t": "0/1"}[action]
        self.kmap.set_pending((idx, time.monotonic(), settle / 1000.0, glyph))
        self.write_timer.start(settle)

    def _cancel_write(self) -> None:
        self.write_timer.stop()
        self._pending_write = None
        self.kmap.set_pending(None)

    def _commit_write(self) -> None:
        idx = self._pending_write
        self._cancel_write()
        if idx is None or self.mode != "kmap":
            return
        inp = self.inputs()
        if idx != ((inp[0] << 3) | (inp[1] << 2) | (inp[2] << 1) | inp[3]):
            return
        action = self.cb_write.currentData()
        old = self.kmap.cells[idx]
        new = {"1": 1, "0": 0, "X": 2}.get(action, 0 if old == 1 else 1)
        self.kmap.pulse_cell(idx)
        if new == old:
            self.log("LOGIC", f"switches -> m{idx} is already {'01X'[old]}")
            return
        self._undo.append((idx, old))
        self.btn_undo.setEnabled(True)
        self.kmap.cells[idx] = new
        self.log("LOGIC", f"switches wrote {'01X'[new]} into m{idx} (ABCD={idx:04b})")
        self.recompute()

    def undo_write(self) -> None:
        if not self._undo:
            return
        idx, old = self._undo.pop()
        self.btn_undo.setEnabled(bool(self._undo))
        self.kmap.cells[idx] = old
        self.kmap.pulse_cell(idx)
        self.log("LOGIC", f"undo: m{idx} back to {'01X'[old]}")
        self.recompute()

    # ---------------------------------------------------------------- mode
    def set_mode(self, key: str) -> None:
        self._cancel_write()
        self.mode = key
        keys = [k for k, _ in MODE_DEFS]
        self.mode_buttons[key].setChecked(True)
        self.pages.setCurrentIndex(keys.index(key))
        for i in range(self.pages.count()):
            pol = QSizePolicy.Policy.Preferred if i == self.pages.currentIndex() else QSizePolicy.Policy.Ignored
            self.pages.widget(i).setSizePolicy(pol, pol)
        self.pages.updateGeometry()
        self.tabs.setCurrentIndex(MODE_TAB[key])
        self.update_sreg_widgets()
        self.sync_reg_ext()
        self.refresh_views()
        self.log("INFO", f"mode -> {dict(MODE_DEFS)[key]}")

    # ---------------------------------------------------------------- K-map / QM
    def apply_cells(self, cells: Sequence[int], msg: Optional[str] = None) -> None:
        self.kmap.set_cells(cells)
        if msg:
            self.log("LOGIC", msg)
        self.recompute()

    def apply_preset_fn(self, fn, dc_fn, announce: bool = True, name: str = "") -> None:
        cells = []
        for m in range(16):
            a, b, c, d = (m >> 3) & 1, (m >> 2) & 1, (m >> 1) & 1, m & 1
            if dc_fn and dc_fn(a, b, c, d):
                cells.append(2)
            else:
                cells.append(1 if fn(a, b, c, d) else 0)
        self.apply_cells(cells, f"preset loaded: {name}" if announce else None)

    def _on_preset(self, i: int) -> None:
        if i <= 0:
            return
        name, fn, dc = PRESETS[i]
        self.apply_preset_fn(fn, dc, name=name)
        self.cb_preset.blockSignals(True)
        self.cb_preset.setCurrentIndex(0)
        self.cb_preset.blockSignals(False)
        self.tabs.setCurrentIndex(TAB_KMAP)

    def randomize(self) -> None:
        self.apply_cells([random.choice((0, 0, 1, 1, 1, 2)) for _ in range(16)], "random K-map generated")

    def invert_cells(self) -> None:
        self.apply_cells([{0: 1, 1: 0, 2: 2}[c] for c in self.kmap.cells], "K-map inverted (0 <-> 1)")

    def apply_fields(self) -> None:
        try:
            m = parse_index_list(self.ed_m.text())
            d = parse_index_list(self.ed_d.text())
            M = parse_index_list(self.ed_M.text())
        except ValueError as exc:
            self.log("ERR", f"bad term list: {exc}")
            return
        if set(m) & set(d) or set(m) & set(M) or set(d) & set(M):
            self.log("ERR", "term lists overlap - a cell cannot be in two sets")
            return
        ones = set(m)
        if M and not m:
            ones = set(range(16)) - set(M) - set(d)
        cells = [2 if i in d else (1 if i in ones else 0) for i in range(16)]
        self.apply_cells(cells, f"term sets applied: m={sorted(ones)} d={sorted(d)}")

    def on_cell_changed(self, m: int, st: int) -> None:
        self.recompute()

    def on_table_click(self, row: int, col: int) -> None:
        if self.mode == "kmap" and col == 5 and 0 <= row < 16:
            self.kmap.cells[row] = (self.kmap.cells[row] + 1) % 3
            self.kmap.pulse_cell(row)
            self.recompute()

    def _on_allpi(self, on: bool) -> None:
        self.kmap.show_all_pis = on
        self.kmap.set_focus(-1)
        self.kmap.update()

    def _sync_fields(self) -> None:
        cells = self.kmap.cells
        for ed, val in ((self.ed_m, 1), (self.ed_d, 2), (self.ed_M, 0)):
            if not ed.hasFocus():
                ed.setText(",".join(str(i) for i, c in enumerate(cells) if c == val))

    def recompute(self) -> None:
        cells = self.kmap.cells
        m = [i for i, c in enumerate(cells) if c == 1]
        d = [i for i, c in enumerate(cells) if c == 2]
        M = [i for i, c in enumerate(cells) if c == 0]
        try:
            res = self.qm.minimize(m, d, M)
        except ValueError as exc:
            self.log("ERR", f"minimiser rejected input: {exc}")
            return
        self.result = res
        self.kmap.set_result(res)
        self.schem.set_result(res)
        self.lbl_expr.setText(res.expression)
        self.lbl_sigma.setText(res.sigma)
        canon = res.canonical
        self.lbl_canon.setText(canon if len(canon) < 150 else canon[:147] + "...")
        self.list_pi.blockSignals(True)
        self.list_pi.clear()
        for gi, imp in enumerate(res.cover):
            tag = "EPI" if imp in res.essentials else "PI "
            it = QListWidgetItem(swatch_icon(Theme.GROUP_COLORS[gi % len(Theme.GROUP_COLORS)]),
                                 f"{imp.to_string():<7} {imp.pattern()}  {tag}  {len(imp.minterms)} cells")
            self.list_pi.addItem(it)
        self.list_pi.blockSignals(False)
        self.kmap.set_focus(-1)
        self.trace_view.setPlainText("\n".join(res.trace))
        self._sync_fields()
        extra = " (cyclic chart -> Petrick)" if res.cyclic_used else ""
        self.log("LOGIC", f"{res.expression}   [{len(res.primes)} PI, {len(res.essentials)} EPI]{extra}")
        self.refresh_views()

    # ---------------------------------------------------------------- combinational helpers
    def _set_adder_sub(self, sub: bool) -> None:
        self._adder_sub = sub
        self.refresh_views()

    def _load_function(self, fn: Callable[[int, int, int, int], int], name: str) -> None:
        cells = [1 if fn((m >> 3) & 1, (m >> 2) & 1, (m >> 1) & 1, m & 1) else 0 for m in range(16)]
        self.apply_cells(cells, f"{name} loaded into the K-map")
        self.tabs.setCurrentIndex(TAB_KMAP)

    def _load_adder(self, which: int) -> None:
        sub = self._adder_sub
        fn = FullSubtractor if sub else FullAdder
        names = (("Difference", "Borrow") if sub else ("Sum", "Carry"))
        self._load_function(lambda a, b, c, d: fn.evaluate(a, b, c)[which], names[which] + "(A,B,C)")

    def _dec_changed(self) -> None:
        self.refresh_views()

    def _dec_enables(self) -> Tuple[int, int, int]:
        return tuple(int(s.value()) for s in self.tg_dec)  # type: ignore[return-value]

    def _load_decoder(self) -> None:
        k = self.cb_dec_y.currentIndex()
        g1, g2a, g2b = self._dec_enables()
        self._load_function(lambda a, b, c, d: Decoder74138.evaluate(a, b, c, g1, g2a, g2b)[k], f"Y{k} (active-low)")

    def _mux_data(self, m: int, k: int, v: bool) -> None:
        self.mux_data[m][k] = int(v)
        self.refresh_views()

    def _mux_strobe(self, m: int, v: bool) -> None:
        self.mux_g[m] = int(v)
        self.refresh_views()

    def _load_mux(self, m: int) -> None:
        data, g = list(self.mux_data[m]), self.mux_g[m]
        self._load_function(lambda a, b, c, d: Mux74153.evaluate(a, b, data, g), f"{m + 1}Y(S0=A,S1=B)")

    # ---------------------------------------------------------------- shift register glue
    def _sr_mode_changed(self, _i: int) -> None:
        self.reg.set_mode(self.cb_sr_mode.currentData())
        self.update_sreg_widgets()
        self.sync_reg_ext()
        self.log("INFO", f"register mode -> {self.reg.mode}")
        self.refresh_views()

    def update_sreg_widgets(self) -> None:
        follow = self.cb_follow.isChecked()
        hw_clk = self.cb_clk_src.currentIndex() == 1
        mode = self.reg.mode
        serial_in = mode in ("SISO", "SIPO")
        self.tg_si.setInteractive(not follow and serial_in)
        for i, sw in enumerate(self.tg_p):
            sw.setInteractive((not follow or (hw_clk and i == 3)) and not serial_in)
        self.tg_load.setInteractive(mode == "PISO")
        self.btn_clk.setText("⏵  CLOCK  ↑" + ("   (also D↑)" if hw_clk else ""))

    def _sreg_panel_changed(self) -> None:
        self.sync_reg_ext()
        self.refresh_views()

    def sync_reg_ext(self) -> None:
        inp = self.inputs()
        follow = self.cb_follow.isChecked()
        hw_clk = self.cb_clk_src.currentIndex() == 1
        if follow:
            si = inp[0]
            p = [inp[0], inp[1], inp[2], inp[3]]
            if hw_clk:
                p[3] = int(self.tg_p[3].value())
            self.tg_si.setValue(bool(si))
            for i in range(4):
                self.tg_p[i].setValue(bool(p[i]))
        else:
            si = int(self.tg_si.value())
            p = [int(s.value()) for s in self.tg_p]
        self.reg.set_ext(si=si, p=p, load=int(self.tg_load.value()))

    def software_clock(self) -> None:
        if self.mode != "sreg":
            return
        rec = self.reg.clock_edge()
        if self.cb_clk_src.currentIndex() == 0:
            self.reg.clk_level = 1
            QTimer.singleShot(160, self._clk_low)
        self._after_edge(rec, "software")

    def _clk_low(self) -> None:
        if self.cb_clk_src.currentIndex() == 0:
            self.reg.clk_level = 0

    def _autoclock_tick(self) -> None:
        if self.mode == "sreg":
            self.software_clock()

    def _on_autoclk(self, on: bool) -> None:
        if on:
            self.clk_timer.setInterval(int(1000 / self.sp_clk.value()))
            self.clk_timer.start()
        else:
            self.clk_timer.stop()

    def _after_edge(self, rec: dict, src: str) -> None:
        self.reg_canvas.notify_edge(rec)
        self.log("LOGIC", f"CLK↑ #{rec['edge']} ({src}) {rec['mode']}  D={''.join(map(str, rec['d']))}  "
                          f"Q {''.join(map(str, rec['before']))} -> {''.join(map(str, rec['q']))}")
        if rec["violation"]:
            self.log("WARN", "SETUP violation: data changed inside the setup window - old value captured")
        self.sync_reg_ext()
        self.refresh_views()

    def _sreg_async(self, what: str) -> None:
        if what == "clear":
            self.reg.clear()
            self.log("LOGIC", "async /CLR asserted: Q = 0000")
        elif what == "preset":
            self.reg.preset()
            self.log("LOGIC", "async /PRE asserted: Q = 1111")
        else:
            self.reg.reset_history()
            self.log("INFO", "register history cleared")
        self.refresh_views()

    # ---------------------------------------------------------------- sweep
    def _on_sweep_toggled(self, on: bool) -> None:
        if on:
            self.sweep_timer.setInterval(int(1000 / self.sp_sweep.value()))
            self.sweep_timer.start()
            self.log("INFO", "virtual board: auto-sweep running")
        else:
            self.sweep_timer.stop()
        self.update_input_widgets()
        self.on_inputs_changed(force=True)

    def _sweep_tick(self) -> None:
        self._sweep_idx = (self._sweep_idx + 1) % 16
        i = self._sweep_idx
        self.manual = [(i >> 3) & 1, (i >> 2) & 1, (i >> 1) & 1, i & 1]
        self.on_inputs_changed()

    def start_demo(self) -> None:
        self.cb_sweep.setChecked(True)

    # ---------------------------------------------------------------- views & outputs
    def refresh_views(self) -> None:
        if self.result is None:
            return
        inp = self.inputs()
        a, b, c, d = inp
        idx = (a << 3) | (b << 2) | (c << 1) | d
        self.kmap.set_active(idx)
        self.schem.set_inputs(inp)
        mode = self.mode
        res = self.result

        if mode == "kmap":
            F = res.evaluate(idx)
            rows = [[str(m), str((m >> 3) & 1), str((m >> 2) & 1), str((m >> 1) & 1), str(m & 1),
                     "01X"[self.kmap.cells[m]]] for m in range(16)]
            self.tbl.interactive_col = 5
            self.tbl.populate(["m", "A", "B", "C", "D", "F"], rows, idx)
            self.lbl_table_hint.setText("click an F cell to cycle 0 / 1 / X")
            left = [Pin(VARS[i], inp[i]) for i in range(4)]
            self.dev.set_device("F(A,B,C,D)", res.expression, left, [Pin("F", F, hot=bool(F))],
                                [f"live m{idx}: cell = {'01X'[self.kmap.cells[idx]]}  ->  F = {F}"])
        elif mode == "adder":
            fn = FullSubtractor if self._adder_sub else FullAdder
            s, co = fn.evaluate(a, b, c)
            n1, n2 = ("Diff", "Bout") if self._adder_sub else ("Sum", "Cout")
            op = "-" if self._adder_sub else "+"
            rows = []
            for k in range(8):
                ka, kb, kc = (k >> 2) & 1, (k >> 1) & 1, k & 1
                ss, cc = fn.evaluate(ka, kb, kc)
                rows.append([str(ka), str(kb), str(kc), str(ss), str(cc)])
            self.tbl.interactive_col = -1
            self.tbl.populate(["A", "B", "C" + ("in" if not self._adder_sub else "(Bin)"), n1, n2], rows,
                              (a << 2) | (b << 1) | c)
            self.lbl_table_hint.setText("live row highlighted")
            self.lbl_adder_eq.setText(f"{a} {op} {b} {op} {c}")
            self.lbl_adder_out.setText(f"{n1} = {s}      {n2} = {co}" if self._adder_sub else
                                       f"{n1} = {s}      {n2} = {co}      (= {a + b + c} decimal)")
            title = "FULL SUBTRACTOR" if self._adder_sub else "FULL ADDER"
            sub = ("D = A xor B xor Bin\nBout = A'B + (A xnor B)Bin" if self._adder_sub else
                   "S = A xor B xor Cin\nCout = AB + Cin(A xor B)")
            self.dev.set_device(title, sub, [Pin("A", a), Pin("B", b), Pin("Cin" if not self._adder_sub else "Bin", c)],
                                [Pin(n1.upper(), s, hot=bool(s)), Pin(n2.upper(), co, hot=bool(co))],
                                [f"{a} {op} {b} {op} {c}  ->  {n1} {s}, {n2} {co}"])
        elif mode == "decoder":
            g1, g2a, g2b = self._dec_enables()
            en = Decoder74138.enabled(g1, g2a, g2b)
            outs = Decoder74138.evaluate(a, b, c, g1, g2a, g2b)
            sel = (c << 2) | (b << 1) | a
            rows = []
            for k in range(8):
                o = Decoder74138.evaluate(k & 1, (k >> 1) & 1, (k >> 2) & 1, g1, g2a, g2b)
                rows.append(["1" if en else "0", str((k >> 2) & 1), str((k >> 1) & 1), str(k & 1),
                             "".join(str(x) for x in reversed(o))])
            self.tbl.interactive_col = -1
            self.tbl.populate(["EN", "C", "B", "A", "Y7..Y0"], rows, sel)
            self.lbl_table_hint.setText("active-low outputs; live row highlighted")
            if en:
                set_pill(self.lbl_dec_en, f"ENABLED  -  Y{sel} LOW", Theme.EMERALD)
            else:
                set_pill(self.lbl_dec_en, "DISABLED  -  all outputs HIGH", Theme.AMBER)
            for k, lb in enumerate(self.dec_y):
                if outs[k] == 0:
                    set_pill(lb, f"Y{k}", Theme.EMERALD)
                else:
                    lb.setText(f"Y{k}")
                    lb.setStyleSheet("")
                    lb.setObjectName("pill")
                    lb.style().unpolish(lb)
                    lb.style().polish(lb)
            left = [Pin("A", a), Pin("B", b), Pin("C", c), Pin("G1", g1, hot=g1 == 1),
                    Pin("G2A", g2a, True, g2a == 0), Pin("G2B", g2b, True, g2b == 0)]
            right = [Pin(f"Y{k}", outs[k], True, outs[k] == 0) for k in range(8)]
            cap = (f"CBA = {c}{b}{a}  ->  Y{sel} goes LOW" if en else "DISABLED: needs G1=1, /G2A=0, /G2B=0")
            self.dev.set_device("74138", "3-to-8 line decoder / demultiplexer", left, right, [cap])
        elif mode == "mux":
            y = [Mux74153.evaluate(a, b, self.mux_data[m], self.mux_g[m]) for m in range(2)]
            sel = (b << 1) | a
            rows = []
            for k in range(4):
                rows.append([str((k >> 1) & 1), str(k & 1)] +
                            [str(Mux74153.evaluate(k & 1, (k >> 1) & 1, self.mux_data[m], self.mux_g[m])) for m in range(2)])
            self.tbl.interactive_col = -1
            self.tbl.populate(["S1", "S0", "1Y", "2Y"], rows, sel)
            self.lbl_table_hint.setText("uses the data inputs set above; live row highlighted")
            self.lbl_mux.setText(f"S1S0 = {b}{a}     1Y = {y[0]}     2Y = {y[1]}")
            left = [Pin("/1G", self.mux_g[0], True)]
            left += [Pin(f"1C{k}", self.mux_data[0][k], hot=(k == sel and not self.mux_g[0])) for k in range(4)]
            left += [Pin("/2G", self.mux_g[1], True)]
            left += [Pin(f"2C{k}", self.mux_data[1][k], hot=(k == sel and not self.mux_g[1])) for k in range(4)]
            left += [Pin("S0", a), Pin("S1", b)]
            right = [Pin("1Y", y[0], hot=bool(y[0])), Pin("2Y", y[1], hot=bool(y[1]))]
            self.dev.set_device("74153", "dual 4-line to 1-line data selector", left, right,
                                [f"S1S0 = {b}{a}  ->  1Y = 1C{sel}{' (strobed off)' if self.mux_g[0] else ''},  "
                                 f"2Y = 2C{sel}{' (strobed off)' if self.mux_g[1] else ''}"])
        else:  # sreg
            q = self.reg.q
            d_in = self.reg.d_inputs()
            rows = [["now", self._in_text(self.reg.ext, d_in)] + [str(x) for x in q]]
            for rec in list(self.reg.history)[::-1][:15]:
                rows.append([f"#{rec['edge']}", self._in_text({"si": rec["si"]}, rec["d"])] + [str(x) for x in rec["q"]])
            self.tbl.interactive_col = -1
            self.tbl.populate(["clk", "D in", "Q0", "Q1", "Q2", "Q3"], rows, 0)
            self.lbl_table_hint.setText("state after each rising edge, newest first")
            self.lbl_sreg.setText(f"Q0..Q3 = {''.join(map(str, q))}     SO = {q[3]}")
            e = self.reg.ext
            left = [Pin("SI", e["si"]), *[Pin(f"P{i}", e["p"][i]) for i in range(4)], Pin("LD", e["load"]),
                    Pin("CLK", self.reg.clk_level, hot=bool(self.reg.clk_level))]
            right = [Pin(f"Q{i}", q[i], hot=bool(q[i])) for i in range(4)]
            self.dev.set_device("7474 x4", f"{self.reg.mode} shift register", left, right,
                                [f"edges: {self.reg.edge_count}   setup violations: {self.reg.violations}"])
        self.refresh_outputs()

    def _in_text(self, ext: dict, d: Sequence[int]) -> str:
        if self.reg.mode in ("SISO", "SIPO"):
            return str(ext.get("si", d[0]))
        return "".join(map(str, d))

    def compose_outputs(self) -> Tuple[List[int], str, str]:
        inp = self.inputs()
        a, b, c, d = inp
        idx = (a << 3) | (b << 2) | (c << 1) | d
        mode = self.mode
        if mode == "kmap":
            F = self.result.evaluate(idx) if self.result else 0
            st = self.kmap.cells[idx]
            return ([F, 0, 0, 1 if st == 2 else 0], f"{idx:04b} m{idx:<2} F={F}", self.result.expression if self.result else "")
        if mode == "adder":
            fn = FullSubtractor if self._adder_sub else FullAdder
            s, co = fn.evaluate(a, b, c)
            n1, n2 = ("D", "B") if self._adder_sub else ("S", "C")
            return ([s, co, a, b], ("FULL SUB " if self._adder_sub else "FULL ADD ") + f"{a}{b}{c}", f"{n1}={s} {n2}out={co}")
        if mode == "decoder":
            g1, g2a, g2b = self._dec_enables()
            en = Decoder74138.enabled(g1, g2a, g2b)
            sel = (c << 2) | (b << 1) | a
            return ([int(en), a, b, c], f"74138 CBA={c}{b}{a}", f"Y{sel} LOW" if en else "DISABLED")
        if mode == "mux":
            y = [Mux74153.evaluate(a, b, self.mux_data[m], self.mux_g[m]) for m in range(2)]
            return ([y[0], y[1], a, b], f"74153 S1S0={b}{a}", f"1Y={y[0]} 2Y={y[1]}")
        q = self.reg.q
        return (q, f"{self.reg.mode} CLK#{self.reg.edge_count}", f"Q={''.join(map(str, q))} SO={q[3]}")

    LED_LEGEND = {"kmap": "L0=F   L1=-   L2=-   L3=live cell is X",
                  "adder": "L0=Sum/Diff   L1=Carry/Borrow   L2=A   L3=B",
                  "decoder": "L0=enabled   L1=A   L2=B   L3=C (select)",
                  "mux": "L0=1Y   L1=2Y   L2=S0   L3=S1",
                  "sreg": "L0=Q0   L1=Q1   L2=Q2   L3=Q3"}

    def refresh_outputs(self, force: bool = False) -> None:
        leds, l1, l2 = self.compose_outputs()
        if self.cb_led_src.currentData() == "mirror":
            leds = self.inputs()
            self.lbl_led_map.setText("L0=A   L1=B   L2=C   L3=D  (mirror)")
        else:
            self.lbl_led_map.setText(self.LED_LEGEND[self.mode])
        for led, v in zip(self.led_out, leds):
            led.set_color(Theme.EMERALD if v else Theme.DEAD)
        self.lcd.setText(format_lcd(l1) + "\n" + format_lcd(l2))
        wire_leds = [1 - v for v in leds] if self.cb_inv_led.isChecked() else list(leds)
        frame = format_command(wire_leds, l1, l2)
        sent = False
        if self.worker is not None and self.conn_state == "connected":
            sent = self.worker.send_command(wire_leds, l1, l2, force=force)
        if frame != self._last_cmd:
            self._last_cmd = frame
            if sent:
                self.log("TX", frame.strip())
        self.lbl_cmd.setText(("TX> " if sent else "[offline] ") + frame.strip().replace(" ", "·"))

    # ---------------------------------------------------------------- serial management
    def _on_auto_toggled(self, auto: bool) -> None:
        self.lbl_auto.setText("AUTO" if auto else "MANUAL")
        self.cb_port.setEnabled(not auto)
        if auto:
            self._user_disc = False
            self.scan_ports()

    @staticmethod
    def _is_likely(info) -> bool:
        blob = f"{info.description} {info.manufacturer or ''} {info.hwid}".lower()
        return any(k in blob for k in LIKELY_ADAPTERS)

    def scan_ports(self) -> None:
        try:
            infos = sorted(serial.tools.list_ports.comports(), key=lambda i: (len(i.device), i.device))
        except Exception as exc:
            self.log("ERR", f"port scan failed: {exc}")
            infos = []
        entries = [(i.device, f"{i.device}  -  {i.description}") for i in infos]
        if entries != self._port_entries:
            cur = self.cb_port.currentData()
            self.cb_port.blockSignals(True)
            self.cb_port.clear()
            for dev, text in entries:
                self.cb_port.addItem(text, dev)
            if not entries:
                self.cb_port.addItem("(no serial ports found)", None)
            idx = self.cb_port.findData(cur)
            self.cb_port.setCurrentIndex(idx if idx >= 0 else 0)
            self.cb_port.blockSignals(False)
            if self._port_entries or entries:
                self.log("INFO", f"ports: {', '.join(d for d, _ in entries) or 'none'}")
            self._port_entries = entries
            self._user_disc = False                          # port list changed -> allow a fresh auto-connect
        if self.sw_auto.value() and self.conn_state == "idle" and not self._user_disc and not self._shut:
            for info in infos:
                if self._is_likely(info):
                    i = self.cb_port.findData(info.device)
                    if i >= 0:
                        self.cb_port.setCurrentIndex(i)
                    self.log("INFO", f"auto-connect candidate: {info.device} ({info.description})")
                    self.connect_port(info.device)
                    break

    def toggle_connection(self) -> None:
        if self.conn_state == "idle":
            dev = self.cb_port.currentData()
            if not dev:
                self.log("WARN", "no serial port selected")
                return
            self._user_disc = False
            self.connect_port(dev)
        else:
            self._user_disc = True
            self.disconnect_port()

    def connect_port(self, device: str) -> None:
        if self.worker is not None:
            return
        self.conn_state = "connecting"
        self.btn_connect.setText("Connecting...")
        self.btn_connect.setEnabled(False)
        self.led_conn.set_color(Theme.AMBER)
        self.lbl_conn.setText(f"Opening {device}...")
        self.log("INFO", f"opening {device} @ {BAUD_RATE} 8N1 (timeout 0.1 s)")
        w = SerialWorker(device, BAUD_RATE, self)
        w.inputsReceived.connect(self.on_frame)
        w.statusChanged.connect(self.on_conn_status)
        w.logMessage.connect(self.log)
        w.frameRaw.connect(lambda line: self.log("WARN", f"unrecognised line from board: {line}"))
        w.finished.connect(lambda w=w: self._worker_done(w))
        self.worker = w
        w.start()

    def disconnect_port(self) -> None:
        self._recovering = False
        w = self.worker
        if w is None:
            return
        self.btn_connect.setEnabled(False)
        w.stop()

    def on_conn_status(self, connected: bool, msg: str) -> None:
        if self._recovering:
            if connected:
                self.conn_state = "connected"
                self._board_up = None
                self._connected_at = time.monotonic()
                self._last_cmd = ""
                self._recovering = False
                self.refresh_outputs(force=True)
                return
            if msg == "Disconnected":                 # the planned close of the old port
                self.conn_state = "idle"
                return
            self._recovering = False                  # reopen failed (e.g. unplugged): report it normally
        if connected:
            self.conn_state = "connected"
            self.led_conn.set_color(Theme.EMERALD)
            self.btn_connect.setText("Disconnect")
            self.btn_connect.setObjectName("danger")
            self.btn_connect.style().unpolish(self.btn_connect)
            self.btn_connect.style().polish(self.btn_connect)
            self.btn_connect.setEnabled(True)
            self.log("OK", msg)
            self._rx_prev, self._rx_fps = (0, time.monotonic()), 0.0
            self._hw_seen = False
            self._board_up = None
            self._connected_at = time.monotonic()
            self._silence_warned = False
            self._last_cmd = ""
            self.update_input_widgets()
            self.on_inputs_changed(force=True)
        else:
            self.conn_state = "idle"
            self.led_conn.set_color(Theme.RED)
            self.btn_connect.setText("Connect")
            self.btn_connect.setObjectName("primary")
            self.btn_connect.style().unpolish(self.btn_connect)
            self.btn_connect.style().polish(self.btn_connect)
            self.btn_connect.setEnabled(True)
            self.log("WARN", msg)
            self.lbl_hw.setText("HW LINK  idle")
            self.led_hw.set_color(Theme.DEAD)
            if msg.startswith("Could not"):
                self._user_disc = True                      # don't hammer a busy / denied port
            elif msg.startswith("Lost"):
                self._user_disc = not self.sw_auto.value()  # AUTO re-connects when the adapter returns
            self.update_input_widgets()
            self.on_inputs_changed(force=True)
        self.lbl_conn.setText(msg)

    def _worker_done(self, w: SerialWorker) -> None:
        if self.worker is w:
            self.worker = None
            if self.conn_state != "idle" and not self._recovering:
                self.conn_state = "idle"
                self.led_conn.set_color(Theme.RED)
                self.btn_connect.setText("Connect")
                self.btn_connect.setEnabled(True)
        w.deleteLater()

    def _stats_tick(self) -> None:
        now = time.monotonic()
        if self.worker is None:
            return
        st = self.worker.stats()
        rx_prev, t_prev = self._rx_prev
        if now - t_prev >= 0.5:
            self._rx_fps = max(0.0, (st["rx"] - rx_prev) / (now - t_prev))
            self._rx_prev = (st["rx"], now)
        self.lbl_tx.setText(f"TX {st['tx']} frames · {st['tx_bytes']} B")
        self.lbl_rx.setText(f"RX {st['rx']} ok · {st['err']} bad · {st['rx_bytes']} B · {self._rx_fps:.1f} fps")
        if self.conn_state == "connected":
            if st["last_rx"] == 0:
                silent = st["rx_bytes"] == 0
                self.led_hw.set_color(Theme.AMBER if now - self._connected_at < 5 else Theme.RED)
                self.lbl_hw.setText("HW LINK  waiting..." if now - self._connected_at < 5 else
                                    ("HW LINK  board silent" if silent else "HW LINK  bad frames"))
                if now - self._connected_at > 5 and not self._silence_warned:
                    self._silence_warned = True
                    if silent:
                        self.log("ERR", f"{self.worker.port} is open but the board has sent NOTHING in 5 s. "
                                        "Upload firmware/gatelab_trainer/gatelab_trainer.ino to the Arduino "
                                        "(press Disconnect first so the IDE can use the port).")
                    else:
                        self.log("ERR", "the board is sending data, but not GateLab input frames. Check the "
                                        "sketch's baud rate is 115200 and that it prints "
                                        '{"type":"inputs","A":0,"B":0,"C":0,"D":0} lines.')
            else:
                age = now - st["last_rx"]
                if age < 10.0:
                    self.led_hw.set_color(Theme.EMERALD)
                    self.lbl_hw.setText("HW LINK  live")
                    if age > 2.5 and now - self._last_recover > 5.0:
                        self._recover_link(age)
                else:
                    self.led_hw.set_color(Theme.AMBER)
                    self.lbl_hw.setText(f"HW LINK  stale {age:4.1f} s")

    def _recover_link(self, age: float) -> None:
        """The board stopped sending: reopen the port (this also resets the Uno) and carry on."""
        w = self.worker
        if w is None:
            return
        port = w.port
        self._last_recover = time.monotonic()
        self._recovering = True
        w.stop()
        QTimer.singleShot(300, lambda: self._quiet_connect(port))

    def _quiet_connect(self, port: str) -> None:
        if self._shut or self.worker is not None:
            return
        w = SerialWorker(port, BAUD_RATE, self)
        w.inputsReceived.connect(self.on_frame)
        w.statusChanged.connect(self.on_conn_status)
        w.logMessage.connect(lambda level, msg: self.log(level, msg) if not self._recovering else None)
        w.frameRaw.connect(lambda line: self.log("WARN", f"unrecognised line from board: {line}"))
        w.finished.connect(lambda w=w: self._worker_done(w))
        self.worker = w
        w.start()

    # ---------------------------------------------------------------- teardown
    def shutdown(self) -> None:
        if self._shut:
            return
        self._shut = True
        for t in (self.scan_timer, self.stats_timer, self.sweep_timer, self.clk_timer):
            t.stop()
        w = self.worker
        if w is not None:
            w.stop(3000)
            self.worker = None


# ======================================================================================
#  MAIN WINDOW
# ======================================================================================
class MainWindow(QMainWindow):
    def __init__(self, start_console: bool = False):
        super().__init__()
        self.setWindowTitle(f"{APP_NAME} - {APP_TAGLINE}")
        self.resize(1480, 920)
        self.setMinimumSize(1180, 720)
        self.stack = QStackedWidget()
        self.landing = LandingPage()
        self.console = ConsolePage()
        self.stack.addWidget(self.landing)
        self.stack.addWidget(self.console)
        self.setCentralWidget(self.stack)
        self.landing.launchConsole.connect(lambda: self.go(1))
        self.landing.launchDemo.connect(lambda: self.go(1, demo=True))
        self.console.goHome.connect(lambda: self.go(0))
        self._fade: Optional[QPropertyAnimation] = None
        if start_console:
            self.stack.setCurrentIndex(1)

    def go(self, page: int, demo: bool = False) -> None:
        if self.stack.currentIndex() == page and not demo:
            return
        out = QPropertyAnimation(self, b"windowOpacity", self)
        out.setDuration(140)
        out.setStartValue(1.0)
        out.setEndValue(0.0)

        def switch() -> None:
            self.stack.setCurrentIndex(page)
            if demo:
                self.console.start_demo()
            back = QPropertyAnimation(self, b"windowOpacity", self)
            back.setDuration(220)
            back.setStartValue(0.0)
            back.setEndValue(1.0)
            self._fade = back
            back.start()

        out.finished.connect(switch)
        self._fade = out
        out.start()

    def closeEvent(self, e) -> None:
        self.console.shutdown()
        e.accept()


def main() -> int:
    QApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    app = QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    app.setStyle("Fusion")
    win = MainWindow(start_console="--console" in sys.argv)
    app.aboutToQuit.connect(win.console.shutdown)
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
