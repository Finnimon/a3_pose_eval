#!/usr/bin/env python3

import sys

import matplotlib.pyplot as plt
import pandas as pd

# -------------------------------------------------
# Arguments
# -------------------------------------------------

if len(sys.argv) != 7:
    print(
        "Usage: python create_graphic.py file.csv actual_x actual_y width height title"
    )
    sys.exit(1)

filename = sys.argv[1]
actual_x = float(sys.argv[2])
actual_y = float(sys.argv[3])
plot_width = float(sys.argv[4])
plot_height = float(sys.argv[5])
title = sys.argv[6]

# -------------------------------------------------
# Load CSV
# -------------------------------------------------

df = pd.read_csv(filename)

df = df.rename(columns={"x[m]": "x", "y[m]": "y", "rot[°]": "rot"})

# -------------------------------------------------
# Plot
# -------------------------------------------------

fig, ax = plt.subplots(figsize=(8, 8))

# Measured positions
ax.plot(df["x"], df["y"], color="blue", label="Messungen")

# Actual position
ax.plot(
    [actual_x],
    [actual_y],
    marker="o",
    color="red",
    label="Position",
    zorder=5,
)

# Fixed plot size
ax.set_xlim(0, plot_width)
ax.set_ylim(0, plot_height)


ax.set_aspect("equal", adjustable="box")
ax.grid(True)

ax.set_xlabel("X [m]", fontsize=18)
ax.set_ylabel("Y [m]", fontsize=18)
ax.set_title(title, fontsize=22)

ax.tick_params(axis="both", labelsize=16)

ax.legend(fontsize=16)
plt.tight_layout()
plt.show()
