"""
Einfacher Test für Tiefenkamera - Eingangserkennung
Zeigt Tiefenbild und erkennt niedrige Durchgänge in Echtzeit
"""

import pyrealsense2 as rs
import numpy as np
import cv2


def main():
    print("🎥 Starte RealSense Tiefenkamera-Test...")
    print("=" * 60)
    
    # Pipeline konfigurieren
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Nur Depth und Color Stream aktivieren
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # Pipeline starten
    print("📷 Starte Kamera...")
    profile = pipeline.start(config)
    
    # Depth Sensor Info
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"✅ Kamera bereit (Depth Scale: {depth_scale})")
    
    # Align-Filter (synchronisiert Depth und Color)
    align = rs.align(rs.stream.color)
    
    # Filter für bessere Depth-Qualität
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
    
    print("\n🔍 Erkennungsparameter:")
    print(f"   Min. Durchgangshöhe: 0.15m")
    print(f"   Max. Erkennungsreichweite: 3.0m")
    print(f"   Roboterhöhe (Normal): 0.30m")
    print("\n⌨️  Steuerung: 'q' = Beenden | 's' = Screenshot")
    print("=" * 60)
    
    try:
        while True:
            # Frames erfassen
            frames = pipeline.wait_for_frames()
            
            # Frames alignen
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue
            
            # Filter anwenden
            depth_frame = spatial.process(depth_frame)
            depth_frame = temporal.process(depth_frame)
            
            # Zu Numpy Arrays konvertieren
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # Depth in Meter umrechnen
            depth_meters = depth_image * depth_scale
            
            # === EINGANGSERKENNUNG ===
            
            height, width = depth_image.shape
            
            # 1. OBERER BEREICH: Suche nach Hindernissen/Decke
            # Analysiere obere 50% des Bildes
            ceiling_roi = depth_meters[int(height*0.1):int(height*0.6), :]
            
            # 2. UNTERER BEREICH: Boden als Referenz
            ground_roi = depth_meters[int(height*0.8):, :]
            ground_distances = ground_roi[ground_roi > 0]
            ground_distance = np.median(ground_distances) if len(ground_distances) > 0 else 0
            
            # 3. FINDE NAHE HINDERNISSE (< 3m)
            close_obstacles = (ceiling_roi > 0.1) & (ceiling_roi < 3.0)
            
            # Erstelle Maske
            obstacle_mask = close_obstacles.astype(np.uint8) * 255
            
            # Rauschunterdrückung
            kernel = np.ones((7, 7), np.uint8)
            obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel)
            obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)
            
            # Finde Konturen
            contours, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, 
                                          cv2.CHAIN_APPROX_SIMPLE)
            
            # Visualisierung vorbereiten
            output = color_image.copy()
            
            # Info-Text
            cv2.putText(output, "TIEFENKAMERA TEST", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            if ground_distance > 0:
                cv2.putText(output, f"Boden: {ground_distance:.2f}m", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # === ANALYSE KONTUREN ===
            entrances_found = 0
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Zu kleine Bereiche ignorieren
                if area < 2000:
                    continue
                
                # Bounding Box
                x, y, w, h = cv2.boundingRect(contour)
                y += int(height * 0.1)  # Anpassung an globale Koordinaten
                
                # Mittelpunkt
                center_x = x + w // 2
                center_y = y + h
                
                # Tiefe am Mittelpunkt
                if center_y < height and center_x < width:
                    obstacle_depth = depth_frame.get_distance(center_x, center_y)
                    
                    if obstacle_depth > 0:
                        # === HÖHENBERECHNUNG ===
                        # Vereinfachte Schätzung: Je weiter unten im Bild, desto niedriger
                        pixels_from_bottom = height - center_y
                        estimated_height = (pixels_from_bottom / height) * 1.0  # Bis 1m Höhe
                        
                        # Farbe basierend auf Höhe
                        if estimated_height >= 0.30:
                            color = (0, 255, 0)      # Grün = OK, passt
                            status = "OK"
                        elif estimated_height >= 0.15:
                            color = (0, 165, 255)    # Orange = Ducken
                            status = "DUCKEN"
                        elif estimated_height >= 0.10:
                            color = (0, 100, 255)    # Rot-Orange = Hinlegen
                            status = "HINLEGEN"
                        else:
                            color = (0, 0, 255)      # Rot = Zu niedrig
                            status = "BLOCKIERT"
                        
                        # Zeichne Erkennung
                        cv2.rectangle(output, (x, y), (x+w, y+h), color, 3)
                        
                        # Info-Text
                        info_text = f"H: ~{estimated_height:.2f}m | {status}"
                        cv2.putText(output, info_text, (x, y-30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        
                        dist_text = f"Distanz: {obstacle_depth:.2f}m"
                        cv2.putText(output, dist_text, (x, y-10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        
                        # Markierung Mittelpunkt
                        cv2.circle(output, (center_x, center_y), 8, color, -1)
                        
                        entrances_found += 1
            
            # Anzahl erkannter Durchgänge
            status_color = (0, 255, 0) if entrances_found > 0 else (128, 128, 128)
            cv2.putText(output, f"Durchgaenge: {entrances_found}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
            
            # === TIEFENBILD VISUALISIERUNG ===
            # Colormap für bessere Darstellung
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.08), 
                cv2.COLORMAP_JET)
            
            # Hindernisse im Tiefenbild markieren
            if len(contours) > 0:
                # Erstelle Overlay
                mask_colored = np.zeros_like(depth_colormap)
                for contour in contours:
                    if cv2.contourArea(contour) > 2000:
                        x, y, w, h = cv2.boundingRect(contour)
                        cv2.rectangle(mask_colored, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                depth_colormap = cv2.addWeighted(depth_colormap, 0.7, mask_colored, 0.3, 0)
            
            # === ANZEIGEN ===
            cv2.imshow('Farbbild mit Erkennung', output)
            cv2.imshow('Tiefenbild', depth_colormap)
            
            # === TASTATUR ===
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("\n👋 Beende...")
                break
            elif key == ord('s'):
                timestamp = int(cv2.getTickCount())
                cv2.imwrite(f'test_color_{timestamp}.png', output)
                cv2.imwrite(f'test_depth_{timestamp}.png', depth_colormap)
                print(f"📷 Screenshots gespeichert: test_*_{timestamp}.png")
    
    except KeyboardInterrupt:
        print("\n👋 Beende...")
    
    except Exception as e:
        print(f"\n❌ Fehler: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Aufräumen
        pipeline.stop()
        cv2.destroyAllWindows()
        print("✅ Kamera gestoppt")


if __name__ == "__main__":
    main()
