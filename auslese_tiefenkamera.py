import pyrealsense2 as rs
import numpy as np

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline.start(config)

try:
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()

    # gesamte Raw-Daten holen (uint16)
    depth_raw = np.asanyarray(depth_frame.get_data())

    print("Rohdaten (Z16):")
    print(depth_raw)

finally:
    pipeline.stop()
