"""
Adaptive Höhenanpassung für Unitree GO2
Erkennt niedrige Durchgänge und passt die Körperhöhe automatisch an
"""

import pyrealsense2 as rs
import numpy as np
import cv2
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Tuple
import time


class PostureMode(Enum):
    """Roboter-Haltungsmodi"""
    NORMAL = "normal"      # 30cm Höhe
    LOW = "low"           # 15cm Höhe (geduckt)
    PRONE = "prone"       # 8cm Höhe (liegend)
    BLOCKED = "blocked"   # Zu eng, Stopp


@dataclass
class PassageGeometry:
    """Geometrie eines erkannten Durchgangs"""
    height: float          # Höhe in Metern
    width: float           # Breite in Metern
    distance: float        # Entfernung zum Durchgang
    tilt_angle: float      # Neigung in Grad
    center_pos: Tuple[int, int]  # Position im Bild
    confidence: float      # Erkennungsqualität 0-1
    

@dataclass
class RobotState:
    """Aktueller Zustand des Roboters"""
    current_height: float = 0.30  # Aktuelle Körperhöhe
    current_mode: PostureMode = PostureMode.NORMAL
    is_adjusting: bool = False
    emergency_stop: bool = False


class AdaptiveHeightController:
    """
    Hauptsteuerung für adaptive Höhenanpassung
    
    INPUT (Sensing) → PROCESSING (Black Box) → DECISION → OUTPUT (Actuation)
    """
    
    def __init__(self, use_lidar=False):
        # RealSense Kamera Setup
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Streams konfigurieren
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
        
        # Pipeline starten
        profile = self.pipeline.start(self.config)
        
        # Kamera-Intrinsics für 3D-Berechnung
        depth_stream = profile.get_stream(rs.stream.depth)
        self.intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        
        # Align-Filter
        self.align = rs.align(rs.stream.color)
        
        # Tiefenfilter für bessere Qualität
        self.spatial_filter = rs.spatial_filter()
        self.temporal_filter = rs.temporal_filter()
        self.hole_filling = rs.hole_filling_filter()
        
        # Roboter-Parameter (Unitree GO2)
        self.robot_width = 0.28      # Meter
        self.robot_length = 0.65
        self.body_height_normal = 0.30
        self.body_height_low = 0.15
        self.body_height_prone = 0.08
        
        # Erkennungsparameter
        self.min_passage_width = 0.35   # Min. Breite
        self.min_passage_height = 0.10  # Min. Höhe
        self.max_detection_range = 3.0  # Max. Distanz
        self.safety_clearance = 0.08    # Sicherheitsabstand
        
        # Schwellwerte für Haltungswechsel
        self.threshold_normal = 0.25    # >= 25cm → Normal
        self.threshold_low = 0.15       # 15-25cm → Niedrig
        self.threshold_prone = 0.10     # 10-15cm → Liegend
        
        # Sicherheit
        self.approach_distance = 0.5    # Stopp vor Durchgang
        self.max_tilt_angle = 30        # Max. Neigung
        
        # State
        self.robot_state = RobotState()
        self.use_lidar = use_lidar
        
        print("🤖 Adaptive Height Controller initialisiert")
        print(f"   Roboter: Unitree GO2 (Breite: {self.robot_width}m)")
        print(f"   Modi: Normal({self.body_height_normal}m) | "
              f"Niedrig({self.body_height_low}m) | "
              f"Liegend({self.body_height_prone}m)")
    
    def apply_depth_filters(self, depth_frame):
        """Wendet Filter auf Tiefendaten an"""
        depth_frame = self.spatial_filter.process(depth_frame)
        depth_frame = self.temporal_filter.process(depth_frame)
        depth_frame = self.hole_filling.process(depth_frame)
        return depth_frame
    
    def detect_ground_plane(self, depth_image):
        """
        Erkennt Bodenhöhe als Referenz
        Nutzt untere Bildbereiche
        """
        height, width = depth_image.shape
        
        # Untere 20% des Bildes analysieren (Boden)
        ground_roi = depth_image[int(height*0.8):, :]
        ground_distances = ground_roi[ground_roi > 0]
        
        if len(ground_distances) > 100:
            ground_distance = np.median(ground_distances)
            return ground_distance * self.depth_scale
        return None
    
    def detect_ceiling_obstacle(self, depth_image, ir_image):
        """
        PHASE 1: INPUT - Sensing
        Erkennt niedrige Hindernisse (Höhlendecken, niedrige Durchgänge)
        """
        height, width = depth_image.shape
        
        # Obere Hälfte des Bildes analysieren (Decke/Hindernisse)
        ceiling_roi_y_start = int(height * 0.1)
        ceiling_roi_y_end = int(height * 0.6)
        
        # ROI für Decke
        ceiling_roi = depth_image[ceiling_roi_y_start:ceiling_roi_y_end, :]
        
        # Konvertiere zu Metern
        ceiling_distances = ceiling_roi * self.depth_scale
        
        # Finde nahe Hindernisse (< 3m)
        close_mask = (ceiling_distances > 0.1) & (ceiling_distances < self.max_detection_range)
        
        if not np.any(close_mask):
            return None
        
        # Morphologische Operationen für Rauschunterdrückung
        close_mask_uint8 = close_mask.astype(np.uint8) * 255
        kernel = np.ones((5, 5), np.uint8)
        close_mask_uint8 = cv2.morphologyEx(close_mask_uint8, cv2.MORPH_CLOSE, kernel)
        
        # Finde Konturen (potenzielle Hindernisse)
        contours, _ = cv2.findContours(close_mask_uint8, cv2.RETR_EXTERNAL, 
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Größte Kontur = Haupthindernis
        largest_contour = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(largest_contour) < 500:  # Zu klein
            return None
        
        # Bounding Box des Hindernisses
        x, y, w, h = cv2.boundingRect(largest_contour)
        y += ceiling_roi_y_start  # Anpassung an globale Koordinaten
        
        return (x, y, w, h), close_mask, largest_contour
    
    def calculate_passage_geometry(self, depth_frame, obstacle_bbox, ground_distance):
        """
        PHASE 2: PROCESSING - Black Box
        Berechnet Durchgangs-Dimensionen (Höhe, Breite, Distanz, Neigung)
        """
        x, y, w, h = obstacle_bbox
        depth_image = np.asanyarray(depth_frame.get_data()) * self.depth_scale
        
        # 1. HÖHE: Abstand Boden → Unterkante Hindernis
        obstacle_bottom_y = y + h
        
        # Tiefe an der Unterkante des Hindernisses
        obstacle_bottom_roi = depth_image[obstacle_bottom_y-5:obstacle_bottom_y+5, x:x+w]
        valid_depths = obstacle_bottom_roi[obstacle_bottom_roi > 0]
        
        if len(valid_depths) == 0:
            return None
        
        obstacle_distance = np.median(valid_depths)
        
        # 3D-Punkt der Unterkante
        center_x = x + w // 2
        bottom_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, [center_x, obstacle_bottom_y], obstacle_distance)
        
        # Bodenhöhe als Referenz (Y-Koordinate in RealSense ist nach unten)
        # Höhe = Differenz zwischen Boden und Hindernis-Unterkante
        if ground_distance:
            ground_3d = rs.rs2_deproject_pixel_to_point(
                self.intrinsics, [center_x, self.intrinsics.height - 10], ground_distance)
            passage_height = abs(bottom_3d[1] - ground_3d[1])
        else:
            # Fallback: Schätzung basierend auf Bildposition
            # Je weiter unten im Bild, desto näher am Boden
            pixel_height = self.intrinsics.height - obstacle_bottom_y
            passage_height = (pixel_height / self.intrinsics.height) * 0.5
        
        # 2. BREITE: Horizontale Ausdehnung
        left_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, [x, obstacle_bottom_y], obstacle_distance)
        right_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, [x+w, obstacle_bottom_y], obstacle_distance)
        
        passage_width = np.linalg.norm(np.array(left_3d) - np.array(right_3d))
        
        # 3. NEIGUNG: Prüfe ob Decke schräg
        obstacle_top_y = y
        top_roi = depth_image[obstacle_top_y:obstacle_top_y+5, x:x+w]
        valid_top = top_roi[top_roi > 0]
        
        tilt_angle = 0.0
        if len(valid_top) > 0:
            top_distance = np.median(valid_top)
            height_diff = abs(top_distance - obstacle_distance)
            tilt_angle = np.arctan(height_diff / max(passage_height, 0.1)) * 180 / np.pi
        
        # Konfidenz basierend auf Datenqualität
        confidence = min(1.0, len(valid_depths) / 100.0)
        
        return PassageGeometry(
            height=passage_height,
            width=passage_width,
            distance=obstacle_distance,
            tilt_angle=tilt_angle,
            center_pos=(center_x, obstacle_bottom_y),
            confidence=confidence
        )
    
    def decide_posture(self, geometry: PassageGeometry) -> Tuple[PostureMode, float]:
        """
        PHASE 3: DECISION - State Machine
        Entscheidet welche Körperhaltung nötig ist
        
        Returns: (PostureMode, target_height)
        """
        if geometry is None:
            return PostureMode.NORMAL, self.body_height_normal
        
        # Sicherheits-Checks
        if geometry.confidence < 0.4:
            print("⚠️  Niedrige Konfidenz, bleibe in NORMAL")
            return PostureMode.NORMAL, self.body_height_normal
        
        if geometry.width < self.min_passage_width:
            print(f"🚫 Durchgang zu schmal: {geometry.width:.2f}m < {self.min_passage_width}m")
            return PostureMode.BLOCKED, self.robot_state.current_height
        
        if geometry.tilt_angle > self.max_tilt_angle:
            print(f"⚠️  Neigung zu stark: {geometry.tilt_angle:.1f}° > {self.max_tilt_angle}°")
            return PostureMode.BLOCKED, self.robot_state.current_height
        
        # Ziel-Höhe berechnen: Durchgangshöhe - Sicherheitsabstand
        target_height = geometry.height - self.safety_clearance
        target_height = max(target_height, self.body_height_prone)  # Min. 8cm
        
        # State Machine: Entscheidung basierend auf Ziel-Höhe
        if target_height >= self.threshold_normal:
            mode = PostureMode.NORMAL
            actual_height = self.body_height_normal
            print(f"✅ NORMAL: Durchgang {geometry.height:.2f}m, keine Anpassung nötig")
        
        elif target_height >= self.threshold_low:
            mode = PostureMode.LOW
            actual_height = target_height
            print(f"🦆 NIEDRIG: Durchgang {geometry.height:.2f}m, ducken auf {actual_height:.2f}m")
        
        elif target_height >= self.threshold_prone:
            mode = PostureMode.PRONE
            actual_height = self.body_height_prone
            print(f"🐍 LIEGEND: Durchgang {geometry.height:.2f}m, hinlegen auf {actual_height:.2f}m")
        
        else:
            mode = PostureMode.BLOCKED
            actual_height = self.robot_state.current_height
            print(f"🚫 BLOCKIERT: Durchgang {geometry.height:.2f}m zu niedrig (< {self.threshold_prone}m)")
        
        return mode, actual_height
    
    def actuate_height_change(self, target_mode: PostureMode, target_height: float):
        """
        PHASE 4: OUTPUT - Actuation
        Steuert den Unitree GO2 über SDK (Stub-Implementierung)
        """
        if self.robot_state.emergency_stop:
            print("🛑 NOTAUS aktiv - keine Bewegung")
            return False
        
        if target_mode == self.robot_state.current_mode:
            return True  # Bereits im richtigen Modus
        
        print(f"\n🎯 Führe Haltungsänderung aus:")
        print(f"   Von: {self.robot_state.current_mode.value} ({self.robot_state.current_height:.2f}m)")
        print(f"   Nach: {target_mode.value} ({target_height:.2f}m)")
        
        self.robot_state.is_adjusting = True
        
        # TODO: Hier Unitree SDK Integration
        # unitree_sdk.SetBodyHeight(target_height)
        # unitree_sdk.SetPostureMode(target_mode)
        # time.sleep(2.0)  # Warte auf Bewegungsabschluss
        
        # Simulation der Bewegung
        time.sleep(0.5)
        
        self.robot_state.current_height = target_height
        self.robot_state.current_mode = target_mode
        self.robot_state.is_adjusting = False
        
        print(f"✅ Haltung geändert auf {target_mode.value}")
        return True
    
    def visualize(self, color_image, geometry: Optional[PassageGeometry], 
                  target_mode: PostureMode):
        """
        Visualisiert Erkennung und Entscheidung
        """
        output = color_image.copy()
        height, width = output.shape[:2]
        
        # Status-Info
        status_y = 30
        cv2.putText(output, f"Modus: {self.robot_state.current_mode.value.upper()}", 
                   (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.putText(output, f"Hoehe: {self.robot_state.current_height:.2f}m", 
                   (10, status_y+30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if geometry:
            # Durchgangs-Info
            info_x = width - 300
            cv2.rectangle(output, (info_x-10, 10), (width-10, 200), (0, 0, 0), -1)
            cv2.rectangle(output, (info_x-10, 10), (width-10, 200), (255, 255, 255), 2)
            
            info_lines = [
                "DURCHGANG ERKANNT",
                f"Hoehe: {geometry.height:.2f}m",
                f"Breite: {geometry.width:.2f}m",
                f"Distanz: {geometry.distance:.2f}m",
                f"Neigung: {geometry.tilt_angle:.1f}°",
                f"Konfidenz: {geometry.confidence:.0%}",
                "",
                f"Ziel: {target_mode.value.upper()}"
            ]
            
            # Farbe je nach Modus
            color_map = {
                PostureMode.NORMAL: (0, 255, 0),
                PostureMode.LOW: (0, 165, 255),
                PostureMode.PRONE: (0, 100, 255),
                PostureMode.BLOCKED: (0, 0, 255)
            }
            info_color = color_map.get(target_mode, (255, 255, 255))
            
            for i, line in enumerate(info_lines):
                y = 30 + i * 20
                cv2.putText(output, line, (info_x, y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, info_color, 1)
            
            # Markierung im Bild
            cx, cy = geometry.center_pos
            cv2.circle(output, (cx, cy), 10, info_color, -1)
            cv2.circle(output, (cx, cy), 15, info_color, 2)
        
        return output
    
    def run(self):
        """
        Hauptschleife
        """
        print("\n🚀 Starte Adaptive Höhenanpassung...")
        print("   'q' = Beenden | 's' = Screenshot | 'e' = Emergency Stop")
        print("=" * 60)
        
        try:
            while True:
                # PHASE 1: INPUT - Sensing
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                ir_frame = aligned_frames.get_infrared_frame()
                
                if not depth_frame or not color_frame:
                    continue
                
                # Filter anwenden
                depth_frame = self.apply_depth_filters(depth_frame)
                
                # Zu Numpy konvertieren
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                ir_image = np.asanyarray(ir_frame.get_data())
                
                # Bodenerkennung
                ground_dist = self.detect_ground_plane(depth_image)
                
                # Hindernis-/Deckenerkennung
                obstacle_result = self.detect_ceiling_obstacle(depth_image, ir_image)
                
                geometry = None
                target_mode = PostureMode.NORMAL
                
                if obstacle_result:
                    obstacle_bbox, close_mask, contour = obstacle_result
                    
                    # PHASE 2: PROCESSING - Geometrie berechnen
                    geometry = self.calculate_passage_geometry(
                        depth_frame, obstacle_bbox, ground_dist)
                    
                    if geometry:
                        # PHASE 3: DECISION - Haltung entscheiden
                        target_mode, target_height = self.decide_posture(geometry)
                        
                        # PHASE 4: OUTPUT - Nur bei Bedarf anpassen
                        if (target_mode != self.robot_state.current_mode and 
                            target_mode != PostureMode.BLOCKED and
                            geometry.distance < self.approach_distance):
                            self.actuate_height_change(target_mode, target_height)
                
                # Visualisierung
                output = self.visualize(color_image, geometry, target_mode)
                cv2.imshow('Adaptive Height Control', output)
                
                # Tiefenbild
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET)
                cv2.imshow('Depth View', depth_colormap)
                
                # Tastatur
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    cv2.imwrite('adaptive_height_screenshot.png', output)
                    print("📷 Screenshot gespeichert")
                elif key == ord('e'):
                    self.robot_state.emergency_stop = not self.robot_state.emergency_stop
                    status = "AKTIV" if self.robot_state.emergency_stop else "INAKTIV"
                    print(f"🛑 Emergency Stop: {status}")
        
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()
            print("\n✅ System beendet")


def main():
    controller = AdaptiveHeightController(use_lidar=False)
    controller.run()


if __name__ == "__main__":
    main()
