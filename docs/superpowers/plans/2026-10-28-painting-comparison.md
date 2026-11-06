# Painting Comparison Implementation Plan

> Execute inline using superpowers:executing-plans; preserve current selected workspace and its uncommitted platform implementation.

**Goal:** Capture and compare three horizontal and three vertical commercial RB20 painting strokes.
**Architecture:** Existing platform simulator and verified workflow, optional observer camera, isolated experimental runner and offline analysis.
**Tech Stack:** Python, ROS2, Isaac Sim, NumPy, OpenCV, Matplotlib.
**Spec:** docs/superpowers/specs/2026-10-28-painting-comparison.md

## Constraints and review focus
Retain existing force/overlap. No safety bypass. Same world region and inactive-arm pose. Timestamp correspondence, exactly three strokes, measured contact classification, incomplete execution reporting, and camera visibility must be verified.

## Tasks
- [x] Test then implement pure three-stroke geometry and per-phase motion metrics in scripts/painting_comparison.py.
- [x] Add optional independent observation recording with frame timestamps and measured TCP pose; retain unmodified perception cameras.
- [x] Generate fresh assets, check camera and D405 readiness, run each direction through original workflow.
- [x] Export raw measured states, render path overlays, generate joint plots and inspect videos. Report any incomplete live evidence explicitly.

## Ledger
Preparation started; original working tree contains 151 modified/untracked entries, so preserve user-requested active folder and use experiment-specific assets. No branch reset, bulk staging, or automatic commit of existing changes.

Live evidence: D405 and simulator readiness confirmed; observation camera framing inspected. Existing focused suite 56 passed. First horizontal trial currently moving through original collision-checked D405 prescan. Camera setup attempts retained separately. Fresh review led to session consistency, force-age and frame-time resampling checks. No completed painting comparison yet.

Both live trials completed three PAINT strokes, original force validation passed, measured traversal lengths 420.20–420.32 mm, recorded frames dropped=0. Focused regression 123 passed. Final media rendering/packaging pending.

Final: both videos decoded at first/middle/last frame, 1280x720 / 15 fps; image previews and joint plots visually checked; comparison region difference <0.001 mm; results ZIP integrity checked. Owned simulation stacks terminated.
