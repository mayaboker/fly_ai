# Godot video frame normalization

Godot's shared-memory FPV frames are `640 × 360`, while the configured run
video is `960 × 540`. OpenCV's FFmpeg writer rejects differently sized input
frames. The red-target detector already converts Godot RGB into an annotated
BGR frame. For Godot-backed recording only, resize that BGR frame to the
output stream size before writing. Target detection continues to use the
original Godot frame, so TTC input is unchanged.

Validation: writing a native `640 × 360` frame to a `960 × 540` writer emits
FFmpeg's rejected-frame warning; writing the normalized frame produces one
decodable video packet without a warning.
