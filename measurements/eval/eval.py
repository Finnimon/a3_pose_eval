#!/usr/bin/env python3

import os
import sys

import numpy as np
import pandas as pd


def circular_mean_deg(values):
    """Circular mean for angles in degrees."""
    angles = np.deg2rad(values)

    mean = np.arctan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))

    return np.rad2deg(mean) % 360


def circular_std_deg(values):
    """Circular standard deviation for angles in degrees."""
    angles = np.deg2rad(values)

    R = np.sqrt(np.mean(np.sin(angles)) ** 2 + np.mean(np.cos(angles)) ** 2)

    return np.rad2deg(np.sqrt(-2 * np.log(R)))


def angular_error_deg(values, reference):
    """Smallest angular difference."""
    return np.abs(((values - reference + 180) % 360) - 180)


# -------------------------------------------------
# Load
# -------------------------------------------------

if len(sys.argv) < 2:
    print("Usage: python analyze.py file.csv")
    sys.exit(1)


filename = sys.argv[1]

df = pd.read_csv(filename)

df = df.rename(columns={"x[m]": "x", "y[m]": "y", "rot[°]": "rot"})


# -------------------------------------------------
# Time processing
# -------------------------------------------------

df["time"] = pd.to_datetime(df["time"], format="%H:%M:%S.%f")

df["dt"] = df["time"].diff().dt.total_seconds()

duration = (df["time"].iloc[-1] - df["time"].iloc[0]).total_seconds()


# -------------------------------------------------
# Position calculations
# -------------------------------------------------

mean_x = df["x"].mean()
mean_y = df["y"].mean()

df["radius"] = np.sqrt((df["x"] - mean_x) ** 2 + (df["y"] - mean_y) ** 2)


# movement between samples

df["dx"] = df["x"].diff()
df["dy"] = df["y"].diff()

df["step"] = np.sqrt(df["dx"] ** 2 + df["dy"] ** 2)


# velocity

df["velocity"] = df["step"] / df["dt"]

df.loc[~np.isfinite(df["velocity"]), "velocity"] = np.nan


# acceleration

df["acceleration"] = df["velocity"].diff() / df["dt"]

df.loc[~np.isfinite(df["acceleration"]), "acceleration"] = np.nan


# -------------------------------------------------
# Rotation
# -------------------------------------------------

rot_mean = circular_mean_deg(df["rot"])
rot_std = circular_std_deg(df["rot"])

rot_error = angular_error_deg(df["rot"], rot_mean)


# -------------------------------------------------
# Outliers
# -------------------------------------------------

radius_threshold = df["radius"].mean() + 3 * df["radius"].std()

outliers = df[df["radius"] > radius_threshold]


# -------------------------------------------------
# Markdown report
# -------------------------------------------------

report = []

report.append("# Indoor GPS Evaluation Report\n")

report.append("## Dataset")
report.append(f"""
- File: `{filename}`
- Samples: {len(df)}
- Duration: {duration:.3f} s
- Average update rate: {1 / df["dt"].mean():.2f} Hz
""")


report.append("## Position Statistics")

report.append(f"""
| Metric | Value |
|-|-:|
| Mean X | {mean_x:.4f} m |
| Mean Y | {mean_y:.4f} m |
| X std dev | {df.x.std():.4f} m |
| Y std dev | {df.y.std():.4f} m |
| X min/max | {df.x.min():.4f} / {df.x.max():.4f} m |
| Y min/max | {df.y.min():.4f} / {df.y.max():.4f} m |
""")


report.append("## Position Stability")

report.append(f"""
| Metric | Value |
|-|-:|
| RMS spread | {np.sqrt(np.mean(df.radius**2)):.4f} m |
| Mean distance from center | {df.radius.mean():.4f} m |
| Std distance | {df.radius.std():.4f} m |
| 50th percentile | {df.radius.quantile(0.50):.4f} m |
| 68th percentile | {df.radius.quantile(0.68):.4f} m |
| 95th percentile | {df.radius.quantile(0.95):.4f} m |
| 99th percentile | {df.radius.quantile(0.99):.4f} m |
| Maximum deviation | {df.radius.max():.4f} m |
""")


report.append("## Jitter / Motion")

report.append(f"""
| Metric | Value |
|-|-:|
| Mean step | {df.step.mean():.5f} m |
| Median step | {df.step.median():.5f} m |
| Max step | {df.step.max():.5f} m |
| Mean velocity | {df.velocity.mean():.4f} m/s |
| Max velocity | {df.velocity.max():.4f} m/s |
| Max acceleration | {df.acceleration.max():.4f} m/s² |
""")


report.append("## Timing")

report.append(f"""
| Metric | Value |
|-|-:|
| Mean interval | {df.dt.mean() * 1000:.3f} ms |
| Std interval | {df.dt.std() * 1000:.3f} ms |
| Minimum interval | {df.dt.min() * 1000:.3f} ms |
| Maximum interval | {df.dt.max() * 1000:.3f} ms |
""")


report.append("## Rotation")

report.append(f"""
| Metric | Value |
|-|-:|
| Circular mean | {rot_mean:.3f}° |
| Circular std dev | {rot_std:.3f}° |
| 95th percentile deviation | {np.percentile(rot_error, 95):.3f}° |
| Maximum deviation | {rot_error.max():.3f}° |
""")


report.append("## Outliers")

report.append(f"""
- Radius outliers (>3σ): {len(outliers)}
- Percentage: {len(outliers) / len(df) * 100:.2f}%
""")


report.append("## Bounding Box")

report.append(f"""
| Axis | Min | Max | Size |
|-|-:|-:|-:|
| X | {df.x.min():.4f} | {df.x.max():.4f} | {(df.x.max() - df.x.min()):.4f} m |
| Y | {df.y.min():.4f} | {df.y.max():.4f} | {(df.y.max() - df.y.min()):.4f} m |
""")


# Write file

basename = os.path.splitext(os.path.basename(filename))[0]

report_filename = f"report.{basename}.md"

with open(report_filename, "w", encoding="utf-8") as f:
    f.write("\n".join(report))


print("Generated " + "report_filename")
