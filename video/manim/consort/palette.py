"""Colours, kept in step with the paper.

Map colours follow Fig. 1 (fig1_overview_mpl.py); planner identity colours
follow make_figs_baseline.py:68-84, which is the fixed assignment used in every
figure of the paper.
"""
from manim import ManimColor

# --- Fig. 1 map palette -------------------------------------------------
PRIMARY = ManimColor("#0000ff")
SUPPORT = ManimColor("#800080")
START = ManimColor("#008000")
GOAL = ManimColor("#ffa500")
LANDMARK = ManimColor("#d2691e")
OBSTACLE = ManimColor("#595959")
HEX_FILL = ManimColor("#f0f8ff")      # aliceblue
HEX_EDGE = ManimColor("#5f9ea0")      # cadetblue
COMM = ManimColor("#00a000")
COMM_DEAD = ManimColor("#b0b0b0")     # checkpoint where the link was out of range
INK = ManimColor("#1a1a1a")
MUTED = ManimColor("#6b6b6b")

# --- meter --------------------------------------------------------------
OK = ManimColor("#1baf7a")
BAD = ManimColor("#d62d20")

# --- planner identity (paper-wide, make_figs_baseline.py) ---------------
METHODS = {
    "hexspline_cl": ("Ours", ManimColor("#2a78d6")),
    "formation":    ("Formation", ManimColor("#eb6834")),
    "sequential":   ("Sequential", ManimColor("#1baf7a")),
    "clgbt":        ("CL-GBT", ManimColor("#eda100")),
    "greedy":       ("Greedy", ManimColor("#e87ba4")),
}

FONT = "DejaVu Sans"
