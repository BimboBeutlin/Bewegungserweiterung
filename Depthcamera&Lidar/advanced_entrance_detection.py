## Integration von RealSense Tiefenkamera und LiDAR für Eingangserkennung

import pyrealsense2 as rs
import numpy as np
import cv2
from dataclasses import dataclass
from typing import List, Tuple, Optional

@dataclass
class Entrance:
    """Repräsentiert einen erkannten Eingang"""
    position_2d: Tuple[int, int]  # Pixel-Position im Bild
    position_3d: Tuple[float, float, float]  # 3D-Position (x, y, z in Metern)
    width: float  # Breite in Metern
    depth: float  # Entfernung in Metern
    confidence: float  # Konfidenz 0-1
    is_passable: bool  # Ist der Eingang befahrbar?


class MultiSensorEntranceDetector:
    """
    Kombiniert Tiefenkamera und LiDAR für robuste Eingangserkennung
    """
    def __init__(self, use_lidar=False):
        # RealSense Setup
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Höhere Auflösung für bessere Erkennung
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Infrarot-Stream für bessere Kantenerkennung
        self.config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
        
        # Pipeline starten
        profile = self.pipeline.start(self.config)
        
        # Depth-Sensor für Intrinsics
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        # Intrinsics für 3D-Projektion
        depth_stream = profile.get_stream(rs.stream.depth)
        self.intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
        
        # Align-Objekt
        self.align = rs.align(rs.stream.color)
        
        # Filter für bessere Depth-Qualität
        self.spatial_filter = rs.spatial_filter()
        self.temporal_filter = rs.temporal_filter()
        self.hole_filling = rs.hole_filling_filter()
        
        # Parameter
        self.min_entrance_width = 0.7  # Meter
        self.max_entrance_width = 2.5
        self.min_entrance_height = 1.8  # Mindesthöhe für befahrbaren Eingang
        self.max_detection_range = 5.0
        
        # LiDAR Integration
        self.use_lidar = use_lidar
        self.lidar_data = None
        
    def apply_filters(self, depth_frame):
        """
        Wendet Filter auf Depth-Frame an für bessere Qualität
        """
        depth_frame = self.spatial_filter.process(depth_frame)
        depth_frame = self.temporal_filter.process(depth_frame)
        depth_frame = self.hole_filling.process(depth_frame)
        return depth_frame
    
    def pixel_to_3d(self, x, y, depth):
        """
        Konvertiert Pixel-Koordinaten zu 3D-Weltkoordinaten
        """
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, [x, y], depth)
        return point_3d
    
    def detect_depth_discontinuities(self, depth_image):
        """
        Erkennt Tiefensprünge (wichtig für Eingänge/Türen)
        """
        # Sobel-Filter für Gradientenberechnung
        sobelx = cv2.Sobel(depth_image, cv2.CV_64F, 1, 0, ksize=5)
        sobely = cv2.Sobel(depth_image, cv2.CV_64F, 0, 1, ksize=5)
        
        # Gradient-Magnitude
        gradient_magnitude = np.sqrt(sobelx**2 + sobely**2)
        
        # Normalisieren
        gradient_normalized = cv2.normalize(gradient_magnitude, None, 0, 255, 
                                           cv2.NORM_MINMAX, cv2.CV_8U)
        
        # Schwellwert für starke Tiefenänderungen
        _, discontinuities = cv2.threshold(gradient_normalized, 30, 255, 
                                          cv2.THRESH_BINARY)
        
        return discontinuities
    
    def detect_vertical_structures(self, ir_image):
        """
        Nutzt Infrarot-Bild zur Erkennung vertikaler Strukturen (Türrahmen)
        """
        # Kantenerkennung
        edges = cv2.Canny(ir_image, 50, 150)
        
        # Hough-Transform für Linien
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50,
                                minLineLength=80, maxLineGap=15)
        
        vertical_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Winkel berechnen
                angle = np.abs(np.arctan2(y2-y1, x2-x1) * 180 / np.pi)
                length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
                
                # Vertikale Linien (85-95 Grad) mit Mindestlänge
                if 85 < angle < 95 and length > 100:
                    vertical_lines.append({
                        'line': (x1, y1, x2, y2),
                        'length': length,
                        'x_pos': (x1 + x2) / 2
                    })
        
        return vertical_lines
    
    def find_entrance_candidates(self, depth_image, vertical_lines):
        """
        Findet Eingangs-Kandidaten basierend auf vertikalen Linien
        """
        if len(vertical_lines) < 2:
            return []
        
        # Sortiere Linien nach x-Position
        vertical_lines.sort(key=lambda l: l['x_pos'])
        
        candidates = []
        for i in range(len(vertical_lines) - 1):
            left_line = vertical_lines[i]
            right_line = vertical_lines[i + 1]
            
            # Abstand zwischen Linien
            x_distance = right_line['x_pos'] - left_line['x_pos']
            
            # Prüfe ob Abstand zu Eingangsbreite passt (grob in Pixeln)
            if 80 < x_distance < 400:  # ca. 0.7-2.5m bei typischen Abständen
                x1_left = int(left_line['x_pos'])
                x2_right = int(right_line['x_pos'])
                
                # Bereich zwischen den Linien analysieren
                y1 = int(min(left_line['line'][1], right_line['line'][1]))
                y2 = int(max(left_line['line'][3], right_line['line'][3]))
                
                if 0 <= x1_left < depth_image.shape[1] and \
                   0 <= x2_right < depth_image.shape[1] and \
                   0 <= y1 < depth_image.shape[0] and \
                   0 <= y2 < depth_image.shape[0]:
                    
                    roi = depth_image[y1:y2, x1_left:x2_right]
                    
                    if roi.size > 0:
                        avg_depth = np.mean(roi[roi > 0]) if np.any(roi > 0) else 0
                        
                        candidates.append({
                            'bbox': (x1_left, y1, x2_right - x1_left, y2 - y1),
                            'depth': avg_depth,
                            'left_line': left_line,
                            'right_line': right_line
                        })
        
        return candidates
    
    def calculate_entrance_dimensions(self, candidate, depth_frame):
        """
        Berechnet präzise Dimensionen eines Eingangs-Kandidaten
        """
        x, y, w, h = candidate['bbox']
        center_x = x + w // 2
        center_y = y + h // 2
        
        # Tiefe am Mittelpunkt
        depth = depth_frame.get_distance(center_x, center_y)
        
        if depth == 0 or depth > self.max_detection_range:
            return None
        
        # 3D-Positionen der Eckpunkte berechnen
        left_3d = self.pixel_to_3d(x, center_y, depth)
        right_3d = self.pixel_to_3d(x + w, center_y, depth)
        top_3d = self.pixel_to_3d(center_x, y, depth)
        bottom_3d = self.pixel_to_3d(center_x, y + h, depth)
        
        # Breite und Höhe in Metern
        width = np.linalg.norm(np.array(left_3d) - np.array(right_3d))
        height = np.linalg.norm(np.array(top_3d) - np.array(bottom_3d))
        
        # 3D-Position des Zentrums
        center_3d = self.pixel_to_3d(center_x, center_y, depth)
        
        return {
            'width': width,
            'height': height,
            'center_3d': center_3d,
            'depth': depth
        }
    
    def validate_entrance(self, dimensions):
        """
        Validiert ob ein Kandidat ein echter Eingang ist
        """
        if dimensions is None:
            return False, 0.0
        
        width = dimensions['width']
        height = dimensions['height']
        depth = dimensions['depth']
        
        confidence = 0.0
        
        # Breitenprüfung
        if self.min_entrance_width <= width <= self.max_entrance_width:
            confidence += 0.4
        
        # Höhenprüfung
        if height >= self.min_entrance_height:
            confidence += 0.3
        
        # Tiefenprüfung
        if 0.5 < depth < self.max_detection_range:
            confidence += 0.2
        
        # Aspect Ratio (Eingänge sind höher als breit)
        aspect_ratio = height / width if width > 0 else 0
        if 1.5 < aspect_ratio < 4:
            confidence += 0.1
        
        is_valid = confidence > 0.5
        return is_valid, confidence
    
    def integrate_lidar_data(self, lidar_scan):
        """
        Integriert LiDAR-Daten zur Verbesserung der Erkennung
        
        lidar_scan: Array von (angle, distance) Tupeln
        """
        self.lidar_data = lidar_scan
        # Hier könnte man LiDAR nutzen um:
        # - Freie Durchgänge auf Bodenhöhe zu bestätigen
        # - 2D-Grundriss für Navigationsentscheidungen zu erstellen
        # - Kamera-Ergebnisse zu validieren
    
    def process_frame(self):
        """
        Verarbeitet einen Frame und erkennt Eingänge
        """
        # Frames erfassen
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        ir_frame = aligned_frames.get_infrared_frame()
        
        if not depth_frame or not color_frame:
            return None, None, []
        
        # Filter anwenden
        depth_frame = self.apply_filters(depth_frame)
        
        # Zu Numpy konvertieren
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        ir_image = np.asanyarray(ir_frame.get_data())
        
        # Tiefensprünge erkennen
        discontinuities = self.detect_depth_discontinuities(depth_image)
        
        # Vertikale Strukturen erkennen
        vertical_lines = self.detect_vertical_structures(ir_image)
        
        # Eingangs-Kandidaten finden
        candidates = self.find_entrance_candidates(depth_image, vertical_lines)
        
        # Eingänge validieren
        entrances = []
        for candidate in candidates:
            dimensions = self.calculate_entrance_dimensions(candidate, depth_frame)
            is_valid, confidence = self.validate_entrance(dimensions)
            
            if is_valid and dimensions:
                x, y, w, h = candidate['bbox']
                entrance = Entrance(
                    position_2d=(x + w//2, y + h//2),
                    position_3d=tuple(dimensions['center_3d']),
                    width=dimensions['width'],
                    depth=dimensions['depth'],
                    confidence=confidence,
                    is_passable=(dimensions['height'] >= self.min_entrance_height)
                )
                entrances.append(entrance)
        
        return color_image, depth_image, entrances
    
    def visualize(self, color_image, entrances):
        """
        Visualisiert erkannte Eingänge
        """
        output = color_image.copy()
        
        for i, entrance in enumerate(entrances):
            x, y = entrance.position_2d
            
            # Farbe basierend auf Befahrbarkeit
            color = (0, 255, 0) if entrance.is_passable else (0, 165, 255)
            
            # Kreis am Mittelpunkt
            cv2.circle(output, (x, y), 10, color, -1)
            
            # Informationen
            info_lines = [
                f"Eingang {i+1}",
                f"Breite: {entrance.width:.2f}m",
                f"Distanz: {entrance.depth:.2f}m",
                f"Konfidenz: {entrance.confidence:.1%}",
                f"Befahrbar: {'Ja' if entrance.is_passable else 'Nein'}"
            ]
            
            for j, line in enumerate(info_lines):
                y_offset = y - 80 + j * 20
                cv2.putText(output, line, (x - 80, y_offset),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Gesamtstatus
        status = f"Eingaenge: {len(entrances)} | Befahrbar: {sum(e.is_passable for e in entrances)}"
        cv2.putText(output, status, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        return output
    
    def run(self):
        """
        Hauptschleife
        """
        print("Multi-Sensor Eingangserkennung gestartet")
        print("Drücken Sie 'q' zum Beenden, 's' zum Screenshot")
        
        try:
            while True:
                color_image, depth_image, entrances = self.process_frame()
                
                if color_image is not None:
                    output = self.visualize(color_image, entrances)
                    cv2.imshow('Eingangserkennung', output)
                    
                    # Depth-Visualisierung
                    depth_colormap = cv2.applyColorMap(
                        cv2.convertScaleAbs(depth_image, alpha=0.03),
                        cv2.COLORMAP_JET)
                    cv2.imshow('Tiefenbild', depth_colormap)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    cv2.imwrite('entrance_screenshot.png', output)
                    print("Screenshot gespeichert!")
                    
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()


def main():
    # Mit LiDAR: detector = MultiSensorEntranceDetector(use_lidar=True)
    detector = MultiSensorEntranceDetector(use_lidar=False)
    detector.run()


if __name__ == "__main__":
    main()
