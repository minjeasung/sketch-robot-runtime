# Recorded H-beam depth fixtures

These recordings are regression evidence, not calibrated metric plane ground truth.
Depth samples use four-pixel spacing; multiply their array coordinates by four
before unprojection with the crop-adjusted `K`.

## hbeam_depth.npz

Cropped from the existing local `work/beam-analysis/capture.npz` recording in
the 2026-09-29 sketch workspace. Contains 12 static depth frames and one RGB reference.

- Original crop: x=600:756, y=60:692.
- `rgb`: full-resolution RGB crop (632×156×3).
- `depths`: sampled depth crops (12×158×39), metres.
- `K`: original intrinsics with the crop origin subtracted from cx/cy.

The former “three steel faces plus one foreground strip” annotation was an
image interpretation, not measured truth. The user subsequently identified
shadow-induced over-segmentation, so tests no longer require that four-plane
interpretation. This fixture tests RGB-shadow invariance of depth-established
models. Timestamp pairing is tested separately.

## shadow_beam_depth.npz

Cropped from the local 2026-10-05 `work/current-planes/capture.npz` capture,
frames 0, 5 and 11. RGB/depth were captured from the running ZED preview.

- Original crop: x=544:844, y=164:640.
- `rgb`: full-resolution RGB reference (476×300×3).
- `depths`: sampled crops (3×119×75), metres.
- `K`: crop-adjusted intrinsics.

User annotation: the former pink “plane 5” belongs to cyan “plane 1”; the shadow
must not establish an additional face. The recorded depth in this region is
systematically distorted. The regression checks RGB invariance, retention of the dominant measured face,
and raw-interior disagreement diagnostics. It does not require an arbitrary
plane count or claim that biased depth can identify the true physical faces.
In particular, a short false patch can satisfy the same measured-crease tests
as a genuine face. The fixture must not tune a hard rejection rule to that patch.
