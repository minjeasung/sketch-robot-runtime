# Commercial RB20 painting comparison

Approved conversation requirements: right commercial RB20, left arm bare and parked identically; same region; three contact strokes per direction; horizontal paint with noncontact vertical transitions and vice versa. Preserve existing overlap, force and speeds. Record actual simulation video, measured joint states, and direction-separated motion charts. Show full robot and roller/wall separation, with measured contact/noncontact paths overlaid offline.

Approach: extend existing measured platform workflow and add optional observation capture; retain original D405 measurement, planning, contact control and interlocks. Avoid replay animation or synthetic joint evidence. Use simulation timestamps for frames and measurements; retain force and execution phases. Invalid or incomplete runs remain labeled incomplete. Camera observation must not alter perception images. Default current values: roller 0.175m, overlap 0.30, spacing 0.1225m, square swept area 0.420m, 20N activation with existing hold band, 0.020m/s paint.

Artifacts: per-direction raw video/frame timestamps, trace/CSV and execution reports; annotated videos and comparison plots only for successfully recorded data. Contact classification uses measured force and phase, with transition/unknown distinct from verified noncontact. Preserve all pre-existing uncommitted files.
