# Spray EOAT Mesh Profile

The approved Spray contract requires a calibrated full EOAT mesh and a confirmed
nozzle endpoint. Paint keeps its existing behavior. An empty, missing, invalid,
or unconfirmed profile blocks Spray path generation and execution, including
camera-only preview generation. Preview does not qualify a path for execution.

## Configuration

In system settings, enter the JSON path in **Spray EOAT 프로파일** before starting.
Use a path on the robot/server PC; an absolute path is preferable. Relative paths
beginning with `~/` expand to the server account's home directory first. Other relative paths
are normalized against the supervisor workspace and the same absolute path is
forwarded to generation and execution, including the ZED preview launch.
The input follows the existing settings save/start flow and is disabled while
processes are running. Reloading the page restores the configured value.

`SKETCH_SPRAY_EOAT_PROFILE` sets the initial server configuration. The optional
`spray_eoat_profile` string in `POST /configuration` changes it; omitting it
(or sending null) preserves the current value, and `""` explicitly clears it.
`GET /configuration` and `GET /status` return the normalized value.
Paths are limited to 4096 characters before and after normalization, without
control characters. API changes last for the server session; set the environment
variable to retain the initial value across server restarts.

An empty or nonexistent profile does not prevent camera startup. The generator
still blocks Spray paths until a valid confirmed profile is available.

`spray_eoat_profile` is a read-only node string parameter, default `""`.
Pass one absolute JSON profile path to both `sketch_to_waypoints` and
`moveit_executor`. Restart the nodes to change this parameter, then regenerate
and validate the plan. Do not point the two nodes at independently maintained
copies. Both must read identical profile and mesh contents.

The argument is forwarded by `rb10_painting_system.launch.py`,
`rb10_real_perception_sketch.launch.py`, `rb10_perception_sketch.launch.py`,
`core.launch.py`, `sketch_control.launch.py`, `phase1_python.launch.py`, and
`phase2_unity.launch.py`. `zed_preview.launch.py` forwards it to its included
perception launch and generator, along with the Spray axis and path settings.
When launching generator and executor separately, supply the same path to both.
Add these arguments to the chosen launch alongside its existing required
camera, calibration, and interlock arguments:

```text
process_mode:=spray model_id:=rb20_1900es spray_tool_axis:=+z
spray_eoat_profile:=/absolute/path/to/verified_eoat.json spray_standoff_m:=0.5
```

The camera-only preview fixes `process_mode` to Spray, so omit that argument there.
The shared YAML leaves both profile paths empty. No example is auto-enabled.

## Profile Schema

The installed template is
[`spray_eoat_profile.UNCONFIRMED.json`](../src/sketch_control/config/spray_eoat_profile.UNCONFIRMED.json):

```json
{
  "schema_version": 1,
  "model_id": "rb20_1900es",
  "spray_tool_axis": "+z",
  "mesh_file": "REPLACE_WITH_VERIFIED_FULL_EOAT.stl",
  "mesh_scale_to_m": 0.001,
  "mesh_to_tcp": {
    "translation_m": [0, 0, 0],
    "quaternion_xyzw": [0, 0, 0, 1]
  },
  "endpoint_confirmed": false
}
```

This is an UNCONFIRMED template, not measured geometry. The asset does not exist;
zero translation, identity rotation, and the millimeter scale are placeholders.
No current actual mesh or calibrated mount is supplied or inferred.

- Only STL and OBJ are supported, using the pure Python `trimesh` library.
  ROS installs use the `python3-trimesh` runtime dependency; Python package
  installation declares `trimesh` in `setup.py`. The pure module also requires
  SciPy, declared as `python3-scipy` and `scipy` in the painting-control package.
  Mesh loading uses `trimesh.load_mesh` with `BytesIO` for compatibility with
  prior versions; local implementation testing used trimesh 5.1.0.
- OBJ must already be triangulated: supported records are `v`, `vt`, `vn`,
  triangular `f`, and group/material records. Face indices must be positive
  absolute indices; negative relative indices are rejected. N-gons, curves,
  and vertex colors are not supported.
- Binary STL must have exactly the size declared by its facet count. ASCII STL
  must contain complete solid and facet structures. Structural preflight and
  equality between the declared/parsed face count and the loaded face count
  reject corrupt or partially valid files; silently dropping faces or solids
  is not accepted.
- Resolve relative `mesh_file` paths against the profile's directory.
  Local absolute mesh paths are also supported; `package://` URIs are not.
- Profiles use strict JSON with only the schema keys documented above and the
  optional `endpoint_tcp_m` key. Unknown keys are rejected.
- `mesh_scale_to_m` must be positive and finite. Scale the source vertices
  into meters, then apply the rigid `mesh_to_tcp` rotation and translation.
  Translation is in meters; `quaternion_xyzw` must be finite and unit length.
  Do not guess mesh units or mounting transforms.
- The profile's `model_id` and signed TCP `spray_tool_axis` must match the
  selected robot and resolved runtime axis. The empty runtime axis selects the
  model default (RB10 `-y`, RB20 `+z`); the profile records the explicit axis.
- Optional `endpoint_tcp_m: [x, y, z]` supplies a physically verified nozzle
  outlet offset in TCP meters. All coordinates must be finite and the override
  must lie inside the transformed full mesh's axis-aligned bounding box
  (AABB), allowing a 2 mm tolerance.

Limits: mesh files must not exceed 50 MB or 250,000 faces; degenerate triangles
are rejected. Reach over 5 m is rejected as a units sanity check. These limits
do not establish that the selected robot can reach or safely execute a path.

Without an explicit endpoint, the candidate is the centroid of the unique
furthest mesh vertices along the configured signed TCP axis, after scaling and
mount transformation. Deduplicate vertices so STL triangle repetition does not
bias the centroid. A cap or guard may extend beyond the nozzle: compare the
candidate with the actual outlet through physical calibration and use the
explicit outlet override when appropriate. Setting `endpoint_confirmed: true`
is required for generation even with an explicit endpoint.

## Standoff and Collision Contract

The default `spray_standoff_m=0.5` is measured from the confirmed EOAT endpoint,
not the TCP origin. For outward surface normal `n` and TCP rotation `R`:

```text
p_tcp = p_surface + 0.5 * n - R * endpoint_tcp
R * spray_tool_axis = -n
```

For a nondefault standoff, replace 0.5 with the configured distance in meters.
The endpoint correction does not add the Paint roller radius or contact offset.

Collision checking must use the full transformed mesh, including mounts, guards,
and other protruding parts needed to represent the installed EOAT. An endpoint
point check is insufficient. Overriding the nozzle offset must not trim or
replace the collision mesh. The full mesh is additive: existing fixed URDF tool
geometry remains in obstacle checks and can conservatively reject custom paths.
Only the TCP/flange mounting pair and known fixed tool-assembly links are touch
links; other arm links and world obstacles remain checked.

Every generation rereads the profile and mesh files. Each loader call reads both
files before consulting the LRU cache, which is keyed by the full canonical JSON
and mesh bytes. Profile and mesh content hashes invalidate the existing plan
when their contents change, even if the path is unchanged.
Regenerate and revalidate after calibration, geometry, or process changes.

Verify the actual outlet, axis, mesh dimensions, mount transform, surface-normal
direction, measured standoff, and full collision geometry before use. Neither
this template nor software tests establish hardware qualification.
