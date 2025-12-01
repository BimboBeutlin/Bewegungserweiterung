import pyrealsense2 as rs
import numpy as np
import time

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline.start(config)

try:
    print("Warte auf stabile Frames...")
    for i in range(30):
        pipeline.wait_for_frames()
    
    print("\nStarte kontinuierliche Messung (Strg+C zum Beenden)...")
    
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        
        if depth_frame:
            depth_raw = np.asanyarray(depth_frame.get_data())
            h, w = depth_raw.shape
            
            # Bildmitte
            center_dist = depth_frame.get_distance(w // 2, h // 2)
            
            # Statistiken
            valid_depths = depth_raw[depth_raw > 0]
            
            print(f"Bildmitte: {center_dist:.3f}m | " +
                  f"Gültige Pixel: {len(valid_depths)} | " +
                  f"Min: {depth_raw[depth_raw>0].min() if len(valid_depths)>0 else 0}mm | " +
                  f"Max: {depth_raw.max()}mm | " +
                  f"Median: {np.median(valid_depths) if len(valid_depths)>0 else 0:.0f}mm")
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nBeende...")
finally:
    pipeline.stop()