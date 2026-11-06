# Three-stroke commercial RB20 comparison

Experiment output: `logs/painting-comparison-20261028/`.
The selected configuration is commercial platform / right RB20 / roller, with
an empty left wrist held in the same pose. `assets/bundle.json` records the
common initial posture and its measured provenance. No physical driver is used.

The current generated settings are 175 mm usable roller width, 30% overlap,
122.5 mm centerline pitch, 420 mm square nominal coverage, 20 N activation
request with the existing 20–30 N hold band, and 20 mm/s paint speed.
Each direction uses exactly three alternating contact strokes. The original
executor performs ramp-down, retract, noncontact travel and re-contact.

## Run one trial

Use a fresh observation directory and only one stack at a time. Preserve failed
attempts under distinct names; the recorder refuses to overwrite measured data.

```bash
PAINTING_OBSERVATION_DIR="$PWD/logs/painting-comparison-20261028/horizontal" \
ROS_DOMAIN_ID=107 bash scripts/run_platform_stack.sh \
  --assets "$PWD/logs/painting-comparison-20261028/assets" --rosbridge-port 9097
```

In another terminal:

```bash
source scripts/platform_ros_env.sh
ROS_DOMAIN_ID=107 python3 scripts/run_painting_comparison.py \
  --assets logs/painting-comparison-20261028/assets \
  --output logs/painting-comparison-20261028/horizontal \
  --direction horizontal --execute
```

For the vertical trial restart the stack from the same asset bundle and replace
both `horizontal` values with `vertical`. `--wait-scale` changes only the
observer's long workflow deadlines, never controller/safety freshness limits.
Stop the owned stack gracefully after the trial to flush camera metadata.

```bash
python3 scripts/render_painting_comparison.py logs/painting-comparison-20261028/horizontal
python3 scripts/render_painting_comparison.py logs/painting-comparison-20261028/vertical
python3 scripts/compare_painting_trials.py logs/painting-comparison-20261028
```

## Evidence and interpretation

`report.json` and `trace.json` retain original executor phases, measured force,
validity and session. `measured.jsonl` contains measured simulator joints and TCP
pose at simulation time; `frames.jsonl` associates each JPEG with renderer time.
The observer camera never publishes to the ZED/D405 perception topics.
The experimental work-area view uses ZED; D405 target and work-area refinement
remain mandatory. Raw ZED selection is converted to the same rectified full-view
selection before refinement so both projector and path generator share it.

Exports are `joint_states.csv`, `joint_states.png`, `joint_metrics.json` and
`annotated.mp4`. The video is resampled onto a 15 fps simulation-time grid;
missing gaps over 0.2 seconds are explicitly marked. Green is PAINT with fresh
18–32 N measured force, orange dashed is verified noncontact (absolute force
at most 2 N), purple is transition, gray is unknown. Trails represent the measured
roller cylinder center (rigid URDF offset from TCP) projected through the observer camera, and are visual overlays
without depth occlusion. The inset is a crop of the actual recorded image.

`comparison.png`/`comparison.json` require two successful three-stroke reports
and matching measured area corners within 5 mm. Joint movement is cumulative
absolute rotation, not Cartesian causal contribution. Noncontact comparison
includes only transfers between strokes; setup and final retreat are excluded.
Incomplete trials are visibly labeled and must not be used as a completed
horizontal/vertical result. The simulator's roller contact uses the existing
fixed-cylinder sliding approximation, not a newly validated rolling model.

Validation: focused regression suite 123 passed after using the normal local DDS
test transport (the simulation-specific DDS profile exceeds UDP port bounds for
the test that explicitly uses ROS domain 232).
