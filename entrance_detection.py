## License: Apache 2.0. See LICENSE file in root directory.
## Eingangserkennung mit RealSense Tiefenkamera

import pyrealsense2 as rs
import numpy as np
import cv2

class EntranceDetector:
    def __init__(self):
        # RealSense Pipeline konfigurieren
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Streams aktivieren
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Pipeline starten
        self.pipeline.start(self.config)
        
        # Align-Objekt um Depth und Color zu synchronisieren
        self.align = rs.align(rs.stream.color)
        
        # Parameter für Eingangserkennung
        self.min_entrance_width = 0.6  # Mindestbreite in Metern
        self.max_entrance_width = 2.5  # Maximalbreite in Metern
        self.max_entrance_depth = 5.0  # Maximale Erkennungstiefe in Metern
        self.depth_threshold = 2.5     # Schwellwert für "freien Raum"
        
    def detect_entrances(self, depth_frame, color_frame):
        """
        Erkennt Eingänge basierend auf Tiefeninformationen
        """
        # Depth-Daten als Numpy Array
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Depth in Meter konvertieren
        depth_meters = depth_image * depth_frame.get_units()
        
        # Maske für potenzielle Eingänge erstellen
        # Bereiche mit großer Tiefe oder ohne Messung
        entrance_mask = (depth_meters > self.depth_threshold) | (depth_meters == 0)
        entrance_mask = entrance_mask.astype(np.uint8) * 255
        
        # Morphologische Operationen zur Rauschunterdrückung
        kernel = np.ones((5,5), np.uint8)
        entrance_mask = cv2.morphologyEx(entrance_mask, cv2.MORPH_CLOSE, kernel)
        entrance_mask = cv2.morphologyEx(entrance_mask, cv2.MORPH_OPEN, kernel)
        
        # Kantenerkennung für Türrahmen
        edges = cv2.Canny(entrance_mask, 50, 150)
        
        # Linien erkennen (Hough Transform)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, 
                                minLineLength=100, maxLineGap=20)
        
        # Vertikale Linien filtern (potenzielle Türrahmen)
        vertical_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.abs(np.arctan2(y2-y1, x2-x1) * 180 / np.pi)
                # Vertikale Linien (80-100 Grad)
                if angle > 80 and angle < 100:
                    vertical_lines.append(line[0])
        
        # Konturen finden
        contours, _ = cv2.findContours(entrance_mask, cv2.RETR_EXTERNAL, 
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        entrances = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 1000:  # Zu kleine Bereiche ignorieren
                continue
                
            # Bounding Box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Aspect Ratio prüfen (Eingänge sind höher als breit)
            aspect_ratio = float(h) / w if w > 0 else 0
            
            if aspect_ratio > 1.2:  # Höhe > Breite
                # Breite in Meter berechnen (vereinfachte Schätzung)
                # Durchschnittliche Tiefe im Bereich
                roi_depth = depth_meters[y:y+h, x:x+w]
                avg_depth = np.mean(roi_depth[roi_depth > 0])
                
                if avg_depth > 0:
                    # Pixel zu Meter Konversion (vereinfacht)
                    # Basierend auf Kamera FOV (ca. 69° horizontal für RealSense)
                    fov_h = 69 * np.pi / 180
                    pixel_width_at_depth = (2 * avg_depth * np.tan(fov_h/2)) / 640
                    entrance_width = w * pixel_width_at_depth
                    
                    if self.min_entrance_width < entrance_width < self.max_entrance_width:
                        entrances.append({
                            'bbox': (x, y, w, h),
                            'width_m': entrance_width,
                            'depth_m': avg_depth,
                            'center': (x + w//2, y + h//2)
                        })
        
        return entrances, entrance_mask, vertical_lines
    
    def visualize(self, color_image, entrances, entrance_mask, vertical_lines):
        """
        Visualisiert erkannte Eingänge
        """
        output = color_image.copy()
        
        # Maske als Overlay
        mask_colored = cv2.applyColorMap(entrance_mask, cv2.COLORMAP_JET)
        output = cv2.addWeighted(output, 0.7, mask_colored, 0.3, 0)
        
        # Vertikale Linien (Türrahmen) zeichnen
        if vertical_lines:
            for x1, y1, x2, y2 in vertical_lines:
                cv2.line(output, (x1, y1), (x2, y2), (255, 0, 255), 2)
        
        # Erkannte Eingänge markieren
        for entrance in entrances:
            x, y, w, h = entrance['bbox']
            cv2.rectangle(output, (x, y), (x+w, y+h), (0, 255, 0), 3)
            
            # Informationen anzeigen
            center_x, center_y = entrance['center']
            info_text = f"Breite: {entrance['width_m']:.2f}m"
            depth_text = f"Distanz: {entrance['depth_m']:.2f}m"
            
            cv2.putText(output, info_text, (x, y-25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(output, depth_text, (x, y-5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.circle(output, (center_x, center_y), 5, (0, 0, 255), -1)
        
        return output
    
    def run(self):
        """
        Hauptschleife
        """
        try:
            print("Eingangserkennung gestartet...")
            print("Drücken Sie 'q' zum Beenden")
            
            while True:
                # Frames erfassen
                frames = self.pipeline.wait_for_frames()
                
                # Frames alignen
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    continue
                
                # Zu Numpy Arrays konvertieren
                color_image = np.asanyarray(color_frame.get_data())
                
                # Eingänge erkennen
                entrances, entrance_mask, vertical_lines = self.detect_entrances(
                    depth_frame, color_frame)
                
                # Visualisierung
                output = self.visualize(color_image, entrances, entrance_mask, 
                                       vertical_lines)
                
                # Status anzeigen
                status_text = f"Eingaenge gefunden: {len(entrances)}"
                cv2.putText(output, status_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Anzeigen
                cv2.imshow('Eingangserkennung', output)
                
                # Beenden mit 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()


def main():
    detector = EntranceDetector()
    detector.run()


if __name__ == "__main__":
    main()
