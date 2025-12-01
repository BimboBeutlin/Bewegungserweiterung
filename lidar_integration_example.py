"""
Beispiel für LiDAR-Integration mit RealSense Tiefenkamera
Kombiniert 2D-LiDAR-Scan mit 3D-Kameradaten für robuste Eingangserkennung
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple
import threading
import time

class LidarProcessor:
    """
    Verarbeitet LiDAR-Daten zur Unterstützung der Eingangserkennung
    """
    def __init__(self):
        self.latest_scan = None
        self.scan_lock = threading.Lock()
        
        # Parameter
        self.min_gap_width = 0.7  # Minimale Lückenbreite (Meter)
        self.max_gap_width = 2.5  # Maximale Lückenbreite
        self.max_range = 5.0      # Maximale Reichweite
        
    def process_scan(self, scan_data: List[Tuple[float, float]]):
        """
        Verarbeitet einen LiDAR-Scan
        
        scan_data: Liste von (angle_degrees, distance_meters) Tupeln
        """
        with self.scan_lock:
            self.latest_scan = self._analyze_scan(scan_data)
    
    def _analyze_scan(self, scan_data):
        """
        Analysiert LiDAR-Scan und findet Lücken (potenzielle Eingänge)
        """
        if not scan_data:
            return None
        
        # Nach Winkel sortieren
        scan_sorted = sorted(scan_data, key=lambda x: x[0])
        
        # Zu kartesischen Koordinaten konvertieren
        points = []
        for angle_deg, distance in scan_sorted:
            if 0 < distance < self.max_range:
                angle_rad = np.deg2rad(angle_deg)
                x = distance * np.cos(angle_rad)
                y = distance * np.sin(angle_rad)
                points.append((x, y, distance, angle_deg))
        
        if len(points) < 3:
            return None
        
        # Lücken finden
        gaps = self._find_gaps(points)
        
        return {
            'points': points,
            'gaps': gaps,
            'timestamp': time.time()
        }
    
    def _find_gaps(self, points):
        """
        Findet Lücken zwischen LiDAR-Punkten (potenzielle Durchgänge)
        """
        gaps = []
        
        for i in range(len(points) - 1):
            p1 = points[i]
            p2 = points[i + 1]
            
            # Abstand zwischen benachbarten Punkten
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            gap_width = np.sqrt(dx*dx + dy*dy)
            
            # Winkelunterschied prüfen
            angle_diff = abs(p2[3] - p1[3])
            
            # Ist dies eine signifikante Lücke?
            if (self.min_gap_width < gap_width < self.max_gap_width and 
                angle_diff < 30):  # Nicht mehr als 30° Unterschied
                
                # Mittelpunkt der Lücke
                center_x = (p1[0] + p2[0]) / 2
                center_y = (p1[1] + p2[1]) / 2
                center_dist = np.sqrt(center_x*center_x + center_y*center_y)
                center_angle = np.rad2deg(np.arctan2(center_y, center_x))
                
                gaps.append({
                    'width': gap_width,
                    'center': (center_x, center_y),
                    'distance': center_dist,
                    'angle': center_angle,
                    'point1': p1,
                    'point2': p2
                })
        
        return gaps
    
    def get_gaps_in_camera_fov(self, camera_fov_deg=69, camera_angle_deg=0):
        """
        Gibt Lücken zurück, die im Sichtfeld der Kamera liegen
        
        camera_fov_deg: Horizontales Sichtfeld der Kamera
        camera_angle_deg: Ausrichtung der Kamera relativ zum LiDAR
        """
        with self.scan_lock:
            if not self.latest_scan or not self.latest_scan['gaps']:
                return []
            
            fov_min = camera_angle_deg - camera_fov_deg / 2
            fov_max = camera_angle_deg + camera_fov_deg / 2
            
            gaps_in_fov = []
            for gap in self.latest_scan['gaps']:
                angle = gap['angle']
                # Normalisiere Winkel auf -180 bis 180
                angle = (angle + 180) % 360 - 180
                
                if fov_min <= angle <= fov_max:
                    gaps_in_fov.append(gap)
            
            return gaps_in_fov
    
    def visualize_scan(self, ax=None):
        """
        Visualisiert den LiDAR-Scan
        """
        with self.scan_lock:
            if not self.latest_scan:
                return
            
            if ax is None:
                fig, ax = plt.subplots(figsize=(10, 10))
            
            points = self.latest_scan['points']
            gaps = self.latest_scan['gaps']
            
            # LiDAR-Punkte plotten
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            ax.scatter(xs, ys, c='blue', s=5, label='LiDAR Punkte')
            
            # LiDAR-Position
            ax.scatter([0], [0], c='red', s=100, marker='x', label='LiDAR')
            
            # Lücken hervorheben
            for gap in gaps:
                cx, cy = gap['center']
                p1 = gap['point1']
                p2 = gap['point2']
                
                # Linie zwischen den Randpunkten
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 
                       'r-', linewidth=2, alpha=0.5)
                
                # Mittelpunkt markieren
                ax.scatter([cx], [cy], c='green', s=100, marker='o')
                
                # Text mit Breite
                ax.text(cx, cy + 0.3, f"{gap['width']:.2f}m",
                       ha='center', fontsize=10)
            
            ax.set_xlim(-self.max_range, self.max_range)
            ax.set_ylim(-self.max_range, self.max_range)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_title('LiDAR Scan - Erkannte Lücken')
            ax.legend()
            
            if ax is None:
                plt.show()


class FusedEntranceDetector:
    """
    Kombiniert Kamera und LiDAR für robuste Eingangserkennung
    """
    def __init__(self, camera_detector, lidar_processor):
        self.camera = camera_detector
        self.lidar = lidar_processor
        
    def fuse_detections(self, camera_entrances, lidar_gaps):
        """
        Fusioniert Kamera- und LiDAR-Detektionen
        
        Gibt für jeden Eingang einen Confidence-Score basierend auf:
        - Kamera-Detektion
        - LiDAR-Bestätigung (Lücke in ähnlicher Richtung/Distanz)
        """
        fused_results = []
        
        for entrance in camera_entrances:
            # Kamera-Daten
            cam_angle = np.rad2deg(np.arctan2(
                entrance.position_3d[1], entrance.position_3d[0]))
            cam_distance = entrance.depth
            
            # Suche passende LiDAR-Lücke
            best_match = None
            best_score = 0
            
            for gap in lidar_gaps:
                # Winkel- und Distanzunterschied
                angle_diff = abs(cam_angle - gap['angle'])
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff
                
                dist_diff = abs(cam_distance - gap['distance'])
                
                # Score berechnen (je kleiner der Unterschied, desto besser)
                angle_score = max(0, 1 - angle_diff / 30)  # 30° Toleranz
                dist_score = max(0, 1 - dist_diff / 0.5)    # 0.5m Toleranz
                
                score = (angle_score + dist_score) / 2
                
                if score > best_score:
                    best_score = score
                    best_match = gap
            
            # Kombinierte Konfidenz
            combined_confidence = entrance.confidence * 0.6 + best_score * 0.4
            
            fused_results.append({
                'entrance': entrance,
                'lidar_match': best_match,
                'confidence': combined_confidence,
                'confirmed_by_lidar': best_score > 0.5
            })
        
        return fused_results


# ============================================================================
# BEISPIEL: Simulierter LiDAR-Scan
# ============================================================================

def simulate_lidar_scan_with_entrance():
    """
    Simuliert einen LiDAR-Scan mit einem Eingang
    """
    scan = []
    
    # Simuliere Wände und einen Eingang
    for angle in range(0, 360, 1):
        angle_rad = np.deg2rad(angle)
        
        # Wände an den Seiten
        if -45 <= angle <= 45:  # Vorne
            # Eingang bei 0° (1.5m breit)
            if -10 <= angle <= 10:
                distance = 10.0  # Kein Hindernis (großer Wert)
            else:
                distance = 3.0   # Wand
        elif 45 < angle < 135:  # Rechts
            distance = 2.5
        elif -135 <= angle < -45:  # Links
            distance = 2.5
        else:  # Hinten
            distance = 1.5
        
        # Rauschen hinzufügen
        distance += np.random.normal(0, 0.05)
        
        scan.append((angle, distance))
    
    return scan


def example_lidar_processing():
    """
    Beispiel: LiDAR-Datenverarbeitung
    """
    print("=== LiDAR-Verarbeitung Beispiel ===\n")
    
    # LiDAR-Prozessor erstellen
    lidar = LidarProcessor()
    
    # Simulierten Scan erstellen
    print("Generiere simulierten LiDAR-Scan...")
    scan = simulate_lidar_scan_with_entrance()
    
    # Scan verarbeiten
    print("Verarbeite Scan...")
    lidar.process_scan(scan)
    
    # Lücken im Kamera-Sichtfeld finden
    gaps_in_fov = lidar.get_gaps_in_camera_fov()
    
    print(f"\nErkannte Lücken im Kamera-Sichtfeld: {len(gaps_in_fov)}")
    for i, gap in enumerate(gaps_in_fov):
        print(f"\nLücke {i+1}:")
        print(f"  Breite: {gap['width']:.2f}m")
        print(f"  Distanz: {gap['distance']:.2f}m")
        print(f"  Winkel: {gap['angle']:.1f}°")
        print(f"  Position: ({gap['center'][0]:.2f}, {gap['center'][1]:.2f})")
    
    # Visualisierung
    print("\nErstelle Visualisierung...")
    plt.figure(figsize=(10, 10))
    ax = plt.gca()
    lidar.visualize_scan(ax)
    
    # Kamera-Sichtfeld einzeichnen
    fov_angle = 69 / 2  # RealSense FOV
    fov_range = 5.0
    fov_x1 = fov_range * np.cos(np.deg2rad(fov_angle))
    fov_y1 = fov_range * np.sin(np.deg2rad(fov_angle))
    fov_x2 = fov_range * np.cos(np.deg2rad(-fov_angle))
    fov_y2 = fov_range * np.sin(np.deg2rad(-fov_angle))
    
    ax.plot([0, fov_x1], [0, fov_y1], 'g--', linewidth=2, alpha=0.5, 
            label='Kamera FOV')
    ax.plot([0, fov_x2], [0, fov_y2], 'g--', linewidth=2, alpha=0.5)
    ax.legend()
    
    plt.savefig('lidar_scan_visualization.png', dpi=150, bbox_inches='tight')
    print("Visualisierung gespeichert: lidar_scan_visualization.png")
    plt.show()


# ============================================================================
# Integration mit echtem LiDAR (RPLidar Beispiel)
# ============================================================================

def connect_rplidar(port='/dev/ttyUSB0'):
    """
    Verbindet mit einem RPLidar
    
    Installation: pip install rplidar-roboticia
    """
    try:
        from rplidar import RPLidar
        
        print(f"Verbinde mit RPLidar auf {port}...")
        lidar = RPLidar(port)
        
        # Info ausgeben
        info = lidar.get_info()
        print(f"LiDAR Info: {info}")
        
        health = lidar.get_health()
        print(f"LiDAR Health: {health}")
        
        return lidar
        
    except ImportError:
        print("RPLidar-Bibliothek nicht installiert.")
        print("Installation: pip install rplidar-roboticia")
        return None
    except Exception as e:
        print(f"Fehler beim Verbinden: {e}")
        return None


def run_with_real_lidar(port='/dev/ttyUSB0'):
    """
    Beispiel mit echtem RPLidar
    """
    lidar_device = connect_rplidar(port)
    if not lidar_device:
        return
    
    processor = LidarProcessor()
    
    try:
        print("Starte LiDAR-Scan...")
        for scan in lidar_device.iter_scans():
            # Scan-Daten konvertieren
            scan_data = [(quality, angle, distance/1000.0)  # mm zu m
                        for quality, angle, distance in scan]
            
            # Verarbeiten
            processor.process_scan([(angle, dist) for _, angle, dist in scan_data])
            
            # Lücken ausgeben
            gaps = processor.get_gaps_in_camera_fov()
            if gaps:
                print(f"Erkannte Lücken: {len(gaps)}")
                for gap in gaps:
                    print(f"  {gap['width']:.2f}m bei {gap['angle']:.1f}°")
            
    except KeyboardInterrupt:
        print("\nBeende...")
    finally:
        lidar_device.stop()
        lidar_device.disconnect()


if __name__ == "__main__":
    print("LiDAR Integration Beispiel\n")
    print("1. Simulierter Scan")
    print("2. Echter RPLidar (benötigt Hardware)")
    
    choice = input("\nWählen Sie eine Option (1/2): ").strip()
    
    if choice == "1":
        example_lidar_processing()
    elif choice == "2":
        port = input("LiDAR Port (Standard: /dev/ttyUSB0): ").strip() or "/dev/ttyUSB0"
        run_with_real_lidar(port)
    else:
        print("Starte Simulation...")
        example_lidar_processing()
