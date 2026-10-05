# Recorded H-beam depth fixture

`hbeam_depth.npz` is a cropped, subsampled fixture from the existing local
`work/beam-analysis/capture.npz` recording in the 2026-09-29 sketch workspace.
It contains 12 depth frames of the same static member and one RGB reference.

- Original crop: x=600:756, y=60:692.
- `rgb`: full-resolution RGB crop (632×156×3).
- `depths`: 12 depth crops sampled every four original pixels (12×158×39), metres.
- `K`: original camera intrinsics with the crop origin subtracted from cx/cy.
  Pixel coordinates for these depth samples must therefore be multiplied by four.
- Visible support: three steel faces and one foreground strip at the left.

The annotation is an image-space consistency check, not calibrated metric
plane ground truth. Reusing a single static RGB reference is intentional for
this extraction regression; timestamp pairing is tested separately.
