import pyrealsense2 as rs
import numpy as np

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline.start(config)

try:
    # Überspringe die ersten 30 Frames (Kamera-Aufwärmphase)
    print("Warte auf stabile Frames...")
    for i in range(30):
        pipeline.wait_for_frames()
    
    # Jetzt hole einen stabilen Frame
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    
    if not depth_frame:
        print("Kein Depth-Frame erhalten!")
    else:
        # Gesamte Raw-Daten holen (uint16, Werte in Millimetern)
        depth_raw = np.asanyarray(depth_frame.get_data())
        
        print("Rohdaten (Z16) Shape:", depth_raw.shape)
        print("Min:", depth_raw.min(), "mm")
        print("Max:", depth_raw.max(), "mm")
        print("Anzahl gültiger Pixel (>0):", np.count_nonzero(depth_raw))
        
        # Zeige einen Ausschnitt aus der Mitte
        h, w = depth_raw.shape
        center_region = depth_raw[h//2-5:h//2+5, w//2-5:w//2+5]
        print("\nAusschnitt Bildmitte (10x10 Pixel):")
        print(center_region)
        
        # Zeige Distanz in Metern für die Bildmitte
        distance = depth_frame.get_distance(w // 2, h // 2)
        print(f"\nDistanz in Bildmitte: {distance:.3f} m")
        
        # Finde Bereiche mit gültigen Daten
        valid_depths = depth_raw[depth_raw > 0]
        if len(valid_depths) > 0:
            print("\n=== Statistiken für gültige Pixel ===")
            print(f"Durchschnitt: {valid_depths.mean():.1f} mm ({valid_depths.mean()/1000:.3f} m)")
            print(f"Median: {np.median(valid_depths):.1f} mm ({np.median(valid_depths)/1000:.3f} m)")
            print(f"Std.abweichung: {valid_depths.std():.1f} mm")
            
            # Finde Position des nächsten Objekts
            min_idx = np.argmin(depth_raw[depth_raw > 0])
            y_coords, x_coords = np.where(depth_raw > 0)
            nearest_y = y_coords[min_idx]
            nearest_x = x_coords[min_idx]
            nearest_depth = depth_raw[nearest_y, nearest_x]
            print(f"\nNächstes Objekt bei Pixel ({nearest_x}, {nearest_y}): {nearest_depth} mm ({nearest_depth/1000:.3f} m)")
            
            # Zeige einen Ausschnitt um das nächste Objekt
            y_start = max(0, nearest_y - 5)
            y_end = min(h, nearest_y + 5)
            x_start = max(0, nearest_x - 5)
            x_end = min(w, nearest_x + 5)
            nearest_region = depth_raw[y_start:y_end, x_start:x_end]
            print(f"\nAusschnitt um nächstes Objekt ({x_start}:{x_end}, {y_start}:{y_end}):")
            print(nearest_region)
        else:
            print("\nWARNUNG: Keine gültigen Tiefenwerte! Überprüfe:")
            print("- Ist ein Objekt 0.3-3m vor der Kamera?")
            print("- Ist der IR-Emitter aktiv?")
            print("- Ist die Linse sauber?")

finally:
    pipeline.stop()
