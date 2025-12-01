import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth stream
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Create colorizer for depth visualization
colorizer = rs.colorizer()
colorizer.set_option(rs.option.color_scheme, 0)  # 0=Jet, 2=White to Black

try:
    print("Warte auf stabile Frames...")
    for i in range(30):
        pipeline.wait_for_frames()
    
    print("Starte Live-Feed (ESC oder 'q' zum Beenden)...")
    
    while True:
        # Wait for frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue
        
        # Convert to numpy arrays
        depth_raw = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Colorize depth frame
        depth_colormap = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        
        # Get dimensions
        h, w = depth_raw.shape
        
        # Measure distance at center
        center_dist = depth_frame.get_distance(w // 2, h // 2)
        
        # Draw crosshair at center
        cv2.line(depth_colormap, (w//2 - 20, h//2), (w//2 + 20, h//2), (0, 255, 0), 2)
        cv2.line(depth_colormap, (w//2, h//2 - 20), (w//2, h//2 + 20), (0, 255, 0), 2)
        cv2.circle(depth_colormap, (w//2, h//2), 5, (0, 255, 0), -1)
        
        # Add text with distance information
        valid_depths = depth_raw[depth_raw > 0]
        
        # Center distance
        text_center = f"Mitte: {center_dist:.3f}m"
        cv2.putText(depth_colormap, text_center, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Statistics
        if len(valid_depths) > 0:
            min_dist = depth_raw[depth_raw > 0].min() / 1000.0
            max_dist = depth_raw.max() / 1000.0
            median_dist = np.median(valid_depths) / 1000.0
            
            text_stats = f"Min: {min_dist:.3f}m | Max: {max_dist:.3f}m | Median: {median_dist:.3f}m"
            cv2.putText(depth_colormap, text_stats, (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            text_pixels = f"Gueltige Pixel: {len(valid_depths)} / {w*h}"
            cv2.putText(depth_colormap, text_pixels, (10, 85), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Stack images side by side
        images = np.hstack((color_image, depth_colormap))
        
        # Show images
        cv2.namedWindow('RealSense D435i - Color | Depth', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense D435i - Color | Depth', images)
        
        # Exit on ESC or 'q'
        key = cv2.waitKey(1)
        if key == 27 or key == ord('q'):
            break

except KeyboardInterrupt:
    print("\nBeende...")
finally:
    pipeline.stop()
    cv2.destroyAllWindows()