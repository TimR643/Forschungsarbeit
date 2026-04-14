#!/usr/bin/env python3
import pyrealsense2 as rs
import json

# Configure depth and color streams
pipeline = rs.pipeline()
config   = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)
# Get the intrinsics of the color stream
color_stream = profile.get_stream(rs.stream.color)
intr = color_stream.as_video_stream_profile().get_intrinsics()

data = {
    "fx": intr.fx,
    "fy": intr.fy,
    "cx": intr.ppx,
    "cy": intr.ppy,
    "distortion_model": intr.model.name,   # e.g. “BrownConrady”
    "coeffs": intr.coeffs.tolist()          # [k1,k2,p1,p2,k3]
}

# Print and save
print(json.dumps(data, indent=2))
with open("rs_intrinsics.json", "w") as f:
    json.dump(data, f, indent=2)

pipeline.stop()
