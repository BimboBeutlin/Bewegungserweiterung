# Projekt: Adaptive Höhenanpassung zur Höhlenerkundung
## MPEC - Motion Path Extension and Control

> **Projektbeschreibung**: Entwicklung einer autonomen Höhenanpassungssoftware für den Unitree GO2 Roboterhund zur Erkundung enger Höhleneingänge und niedriger Durchgänge

---

## 📋 Projektübersicht

### Zielsetzung
Entwicklung einer intelligenten Steuerungssoftware für den Unitree GO2, die:
- **Höhleneingänge/enge Durchgänge automatisch erkennt** (Höhe, Breite, Neigung)
- **Optimale Körperhaltung berechnet** (Normal → Niedrig → Liegend)
- **Sicheres Ducken/Senken ermöglicht** (mit Sicherheitsabstand)
- **Multi-Sensor-Fusion nutzt** (Stereo-Depth-Kamera + LiDAR)
- **Autonome Höhlenanpassung** in Echtzeit durchführt

### Anwendungsfälle
- **Höhlenerkundung**: Autonome Navigation durch enge Höhlengänge
- **Search & Rescue**: Durchqueren von Trümmern und engen Räumen
- **Inspection**: Inspektion unter Fahrzeugen, in Rohren, niedrigen Gebäuden
- **Research**: Geologische und speleologische Forschung

---

## 🎯 Projektziele

### Hauptziele
1. **Zuverlässige Eingangsgeometrie-Erkennung** mit >90% Genauigkeit
2. **Echtzeit-Verarbeitung** (<100ms pro Frame)
3. **Robuste Multi-Sensor-Fusion** (Stereo-Depth-Kamera + LiDAR)
4. **Präzise Höhenberechnung** mit cm-Genauigkeit
5. **Sichere autonome Körperanpassung** des Unitree GO2

### Technische Anforderungen (MVP)
- **Erkennungsreichweite**: 0,5m - 3,0m
- **Minimale Eingangsbreite**: 0,4m (Roboterbreite + Sicherheit)
- **Erkennbare Eingangshöhe**: 0,2m - 1,0m
- **Verarbeitungsrate**: mindestens 10 FPS
- **Sicherheitsabstand**: 5-10cm über der Durchgangshöhe
- **Reaktionszeit**: <500ms von Erkennung bis Anpassung

### Roboter-Spezifikationen (Unitree GO2)
- **Normale Körperhöhe**: ~30cm
- **Minimale Körperhöhe**: ~10-15cm (geduckter Stand)
- **Liegende Position**: ~8cm
- **Breite**: ~28cm
- **Länge**: ~65cm
- **Max. Geschwindigkeit**: 3,5 m/s

---

## 🔧 Hardware-Setup

### Sensoren

#### 1. Intel RealSense Tiefenkamera (D400-Serie)
- **Modell**: D435/D455 empfohlen
- **Auflösung**: 640x480 @ 30fps (Depth + Color)
- **Reichweite**: 0,3m - 10m
- **FOV**: 87° × 58° (diagonal ~95°)
- **Technologie**: Stereo-Vision
- **Zusätzlich**: Infrarot-Projektor für Low-Light

**Spezifikationen:**
```
Depth Stream:  640x480, z16 Format, 30 FPS
Color Stream:  640x480, BGR8 Format, 30 FPS
Infrared:      640x480, Y8 Format, 30 FPS
Depth Range:   0.3m - 3m (optimal), bis 10m möglich
Accuracy:      <2% bei 2m Entfernung
```

#### 2. LiDAR-Sensor (Optional)
- **Empfohlen**: RPLidar A1/A2 oder SLAMTEC
- **Typ**: 2D 360° Scanner
- **Reichweite**: 6m - 12m
- **Scan-Rate**: 5-10 Hz
- **Auflösung**: 360 Messpunkte pro Scan

**Spezifikationen:**
```
Scan Rate:     5.5 Hz (A1), 10 Hz (A2)
Sample Rate:   8000 samples/sec
Range:         12m (A1), 16m (A2)
Accuracy:      ±0.5cm
Angular:       0.9° - 1° Auflösung
```

### Systemanforderungen
- **OS**: Ubuntu 20.04/22.04 oder macOS 10.14+
- **CPU**: Intel i5 oder besser (i7 empfohlen)
- **RAM**: mindestens 4GB (8GB empfohlen)
- **USB**: USB 3.0 Port für RealSense
- **Python**: 3.7+

---

## 🏗️ Software-Architektur

### Systemkomponenten (UML-basiert)

```
┌─────────────────────────────────────────────────────────────────┐
│                    HAUPTSTEUERUNG (Main Loop)                   │
│                  (AdaptiveHeightController)                     │
└────────────┬────────────────────────────────────┬───────────────┘
             │                                    │
    ┌────────▼──────────┐              ┌─────────▼──────────┐
    │  SENSING MODUL    │              │   UNITREE SDK      │
    │                   │              │   Interface        │
    │ • Stereo-Depth    │              │                    │
    │ • LiDAR 2D        │              │ • Body Height      │
    │ • IMU/Pose        │              │ • Posture Mode     │
    └────────┬──────────┘              │ • Emergency Stop   │
             │                         └─────────▲──────────┘
    ┌────────▼──────────┐                        │
    │  PROCESSING       │                        │
    │  (Black Box)      │                        │
    │                   │                        │
    │ • Geometrie-      │                        │
    │   Extraktion      │                        │
    │ • Höhen-Berechnung│                        │
    │ • Safety-Check    │                        │
    └────────┬──────────┘                        │
             │                                    │
    ┌────────▼──────────┐                        │
    │  DECISION         │                        │
    │  (State Machine)  │                        │
    │                   │                        │
    │ • Passt so        │─────► WEITER          │
    │ • Zu hoch         │─────► DUCKEN ─────────┤
    │ • Viel zu hoch    │─────► HINLEGEN ───────┤
    │ • Blockiert       │─────► STOPP ──────────┤
    └───────────────────┘                        │
                                                 │
    ┌────────────────────────────────────────────▼───────────┐
    │  ACTUATION (Motor Control)                             │
    │  • Normal (h=30cm) • Niedrig (h=15cm) • Liegend (h=8cm)│
    └────────────────────────────────────────────────────────┘
```

### Zustandsautomat (State Machine)

```
     ┌──────────┐
     │  NORMAL  │ (h = 30cm)
     └────┬─────┘
          │
          │ Eingang erkannt (h < 25cm)
          ▼
     ┌──────────┐
     │ ANALYSE  │ (Messe Höhe + Breite)
     └────┬─────┘
          │
          ├──► h >= 25cm ────────► NORMAL
          │
          ├──► 15cm ≤ h < 25cm ──► NIEDRIG (Ducken)
          │
          ├──► 10cm ≤ h < 15cm ──► LIEGEND (Hinlegen)
          │
          └──► h < 10cm ─────────► BLOCKIERT (Stopp)
```

### Verarbeitungspipeline (Logik-Ablauf)

#### Phase 1: INPUT - Sensing (Messung)
```python
1. Stereo-Depth Camera Capture
   ├─ Depth Stream (z16) - Tiefeninformation
   ├─ Color Stream (BGR8) - Visuelle Referenz
   └─ Infrared Stream (Y8) - Low-Light Erkennung

2. LiDAR 2D Scan Capture
   └─ 360° Distance Measurements (Bodenhöhe)

3. Robot State
   ├─ Current Body Height (IMU)
   ├─ Current Posture Mode
   └─ Position & Orientation
```

#### Phase 2: PROCESSING - Black Box (Berechnung)
```python
4. Eingangs-Geometrie Extraktion
   ├─ Höhenmessung (Unterkante Hindernis → Boden)
   ├─ Breitenmessung (Links → Rechts)
   ├─ Neigungserkennung (Schräge Decken)
   └─ Distanzbestimmung (Roboter → Eingang)

5. Sicherheitsberechnung
   ├─ Ziel_Höhe = Eingangs_Höhe - Sicherheitsabstand
   ├─ Sicherheitsabstand = 5-10cm (konfigurierbar)
   └─ Minimalhöhe = max(Ziel_Höhe, 8cm)

6. Machbarkeits-Check
   ├─ Breite >= Roboter_Breite + 5cm ?
   ├─ Höhe >= 8cm (Minimale liegende Position) ?
   └─ Neigung < 30° (Stabilitätslimit) ?
```

#### Phase 3: DECISION - Entscheidung (State Machine)
```python
7. Haltungs-Entscheidung
   if Ziel_Höhe >= 25cm:
       → NORMAL (Keine Anpassung, h=30cm)
   elif 15cm <= Ziel_Höhe < 25cm:
       → NIEDRIG (Ducken, h=15-25cm)
   elif 10cm <= Ziel_Höhe < 15cm:
       → LIEGEND (Hinlegen, h=8-10cm)
   else:
       → BLOCKIERT (Zu eng, Stopp)

8. Safety Override
   ├─ Emergency Stop aktiv? → STOPP
   ├─ Sensorfehler? → SICHERER ZUSTAND
   └─ Instabilität (IMU)? → ABBRUCH
```

#### Phase 4: OUTPUT - Actuation (Ansteuerung)
```python
9. Unitree SDK Kommandos
   ├─ SetBodyHeight(target_height)
   ├─ SetPostureMode(mode)  # STAND/CROUCH/PRONE
   └─ SetMovementSpeed(reduced_speed)

10. Bewegungsablauf
    ├─ Langsames Annähern (0.2 m/s)
    ├─ Stopp vor Eingang (0.5m Abstand)
    ├─ Höhenänderung (2-3 Sekunden)
    ├─ Durchquerung (langsam, 0.1 m/s)
    └─ Zurück zu Normal (nach Durchgang)
```

#### Phase 5: MONITORING - Überwachung
```python
11. Kontinuierliches Tracking
    ├─ Kollisionserkennung (Depth-Daten)
    ├─ Stabilitäts-Check (IMU)
    └─ Erfolgs-Validierung (Ist durchgekommen?)

12. Logging & Telemetrie
    ├─ Eingangs-Dimensionen
    ├─ Gewählter Modus
    ├─ Durchquerungszeit
    └─ Fehler/Warnungen
```

---

## 🧮 Algorithmen und Methoden

### 1. Tiefensprung-Detektion
**Zweck**: Identifikation von Kanten/Übergängen

```python
# Sobel-Gradientenberechnung
Gx = Sobel(depth_image, direction='x')
Gy = Sobel(depth_image, direction='y')
Magnitude = sqrt(Gx² + Gy²)

# Schwellwert für signifikante Sprünge
threshold = 30  # anpassbar
discontinuities = Magnitude > threshold
```

**Parameter:**
- Kernel-Größe: 5x5
- Threshold: 30 (normalisiert 0-255)
- Morphologische Operationen: Close → Open

### 2. Vertikale Strukturerkennung
**Zweck**: Türrahmen-Identifikation

```python
# Hough-Transform für Linien
lines = HoughLinesP(
    edges,
    rho=1,              # 1 Pixel Auflösung
    theta=π/180,        # 1° Winkelauflösung
    threshold=50,       # Mindest-Votes
    minLineLength=100,  # Mindestlänge in Pixel
    maxLineGap=20       # Max. Lückengröße
)

# Vertikalitäts-Filter
for line in lines:
    angle = atan2(y2-y1, x2-x1) * 180/π
    if 85° < angle < 95°:  # ±5° Toleranz
        vertical_lines.append(line)
```

**Kriterien:**
- Winkel: 85° - 95° (vertikal ±5°)
- Mindestlänge: 100 Pixel (~30cm bei 2m Distanz)
- Max. Lücke: 20 Pixel

### 3. 3D-Projektion
**Zweck**: Pixel → Weltkoordinaten

```python
# RealSense Intrinsics
fx, fy = intrinsics.fx, intrinsics.fy  # Focal Length
cx, cy = intrinsics.ppx, intrinsics.ppy  # Principal Point

# Deprojektion
X = (x - cx) * depth / fx
Y = (y - cy) * depth / fy
Z = depth

point_3d = [X, Y, Z]  # in Metern
```

**Koordinatensystem:**
- X: Rechts (horizontal)
- Y: Unten (vertikal)
- Z: Vorwärts (Tiefe)

### 4. Dimensionsmessung
**Zweck**: Breite/Höhe in Metern

```python
# 3D-Distanz zwischen Punkten
left_3d = deproject(x_left, y_center, depth)
right_3d = deproject(x_right, y_center, depth)

width = norm(left_3d - right_3d)

top_3d = deproject(x_center, y_top, depth)
bottom_3d = deproject(x_center, y_bottom, depth)

height = norm(top_3d - bottom_3d)
```

### 5. LiDAR-Gap-Detektion
**Zweck**: Durchgänge im 2D-Scan finden

```python
# Nachbarpunkt-Analyse
for i in range(len(points) - 1):
    p1 = points[i]
    p2 = points[i+1]
    
    # Kartesische Distanz
    gap_width = sqrt((p2.x - p1.x)² + (p2.y - p1.y)²)
    
    # Winkelsprung prüfen
    angle_diff = abs(p2.angle - p1.angle)
    
    if min_width < gap_width < max_width and angle_diff < 30°:
        gaps.append(Gap(p1, p2, gap_width))
```

**Parameter:**
- Min. Breite: 0,7m
- Max. Breite: 2,5m
- Max. Winkelsprung: 30°

### 6. Sensor-Fusion
**Zweck**: Kamera + LiDAR kombinieren

```python
# Matching-Score
angle_diff = abs(camera_angle - lidar_angle)
distance_diff = abs(camera_dist - lidar_dist)

angle_score = max(0, 1 - angle_diff/30°)
dist_score = max(0, 1 - distance_diff/0.5m)

match_score = (angle_score + dist_score) / 2

# Kombinierte Konfidenz
final_confidence = camera_conf * 0.6 + match_score * 0.4
```

**Gewichtung:**
- Kamera: 60% (primäre Detektion)
- LiDAR: 40% (Validierung)

### 7. Konfidenz-Berechnung
**Zweck**: Qualitätsbewertung der Detektion

```python
confidence = 0.0

# Breitenkriteria (40%)
if min_width <= width <= max_width:
    confidence += 0.4

# Höhenkriteria (30%)
if height >= min_height:
    confidence += 0.3

# Distanzkriteria (20%)
if valid_range_min < distance < valid_range_max:
    confidence += 0.2

# Aspect Ratio (10%)
ratio = height / width
if 1.5 < ratio < 4.0:
    confidence += 0.1

# Akzeptanzschwelle
is_valid = confidence > 0.5
```

---

## 📊 Parameter-Referenz

### Roboter-Parameter (Unitree GO2)

| Parameter | Wert | Einheit | Beschreibung |
|-----------|------|---------|--------------|
| `robot_width` | 0.28 | m | Breite des Roboters |
| `robot_length` | 0.65 | m | Länge des Roboters |
| `body_height_normal` | 0.30 | m | Normale Standhöhe |
| `body_height_low` | 0.15 | m | Geduckte Position |
| `body_height_prone` | 0.08 | m | Liegende Position |
| `body_height_min` | 0.08 | m | Absolute Minimalhöhe |

### Erkennungsparameter

| Parameter | Wert | Einheit | Beschreibung |
|-----------|------|---------|--------------|
| `min_entrance_width` | 0.35 | m | Minimale Durchgangsbreite |
| `min_entrance_height` | 0.10 | m | Minimale erkennbare Höhe |
| `max_entrance_height` | 1.00 | m | Maximale relevante Höhe |
| `max_detection_range` | 3.0 | m | Maximale Erkennungsdistanz |
| `safety_clearance` | 0.05-0.10 | m | Sicherheitsabstand oben |
| `confidence_threshold` | 0.6 | - | Min. Konfidenz für Aktion |

### Schwellwerte für Haltungswechsel

| Zustand | Höhenbereich | Aktion |
|---------|--------------|--------|
| **NORMAL** | h ≥ 0.25m | Keine Anpassung, weiterfahren |
| **NIEDRIG** | 0.15m ≤ h < 0.25m | Ducken, Körper senken |
| **LIEGEND** | 0.10m ≤ h < 0.15m | Hinlegen, maximale Senkung |
| **BLOCKIERT** | h < 0.10m | Stopp, zu eng |

### Sicherheitsparameter

| Parameter | Wert | Einheit | Beschreibung |
|-----------|------|---------|--------------|
| `approach_distance` | 0.5 | m | Stopp-Abstand vor Eingang |
| `approach_speed` | 0.2 | m/s | Annäherungsgeschwindigkeit |
| `traverse_speed` | 0.1 | m/s | Durchquerungsgeschwindigkeit |
| `height_adjust_time` | 2.0-3.0 | s | Zeit für Höhenänderung |
| `max_tilt_angle` | 30 | ° | Max. Neigung/Schräge |
| `emergency_stop_time` | 0.5 | s | Reaktionszeit Notaus |

### Bildverarbeitungsparameter

| Parameter | Wert | Beschreibung |
|-----------|------|--------------|
| `canny_low` | 50 | Unterer Canny-Schwellwert |
| `canny_high` | 150 | Oberer Canny-Schwellwert |
| `hough_threshold` | 50 | Min. Votes für Linie |
| `hough_minLineLength` | 100 | Min. Linienlänge (Pixel) |
| `hough_maxLineGap` | 20 | Max. Lücke in Linie (Pixel) |
| `morphology_kernel` | 5×5 | Kernel für Morphologie |
| `gradient_threshold` | 30 | Schwelle Tiefensprung |

### Filter-Parameter (RealSense)

| Filter | Parameter | Wert |
|--------|-----------|------|
| Spatial | Magnitude | 2 |
| Spatial | Alpha | 0.5 |
| Spatial | Delta | 20 |
| Temporal | Alpha | 0.4 |
| Temporal | Delta | 20 |
| Hole Filling | Mode | Farest from around |

### LiDAR-Parameter

| Parameter | Wert | Einheit | Beschreibung |
|-----------|------|---------|--------------|
| `min_gap_width` | 0.7 | m | Min. Lückenbreite |
| `max_gap_width` | 2.5 | m | Max. Lückenbreite |
| `max_lidar_range` | 6.0 | m | Max. Messreichweite |
| `angle_tolerance` | 30 | ° | Max. Winkelsprung |
| `camera_fov` | 69 | ° | Kamera-Sichtfeld (horizontal) |

### Fusion-Parameter

| Parameter | Wert | Beschreibung |
|-----------|------|--------------|
| `camera_weight` | 0.6 | Gewichtung Kamera-Detektion |
| `lidar_weight` | 0.4 | Gewichtung LiDAR-Validierung |
| `angle_tolerance` | 30° | Max. Winkelabweichung |
| `distance_tolerance` | 0.5m | Max. Distanzabweichung |
| `match_threshold` | 0.5 | Min. Score für Match |

---

## 📁 Projektstruktur

```
Bewegungserweiterung/
├── README.md                          # Hauptdokumentation
├── PROJECT_SPEC.md                    # Diese Datei
├── Setup.py                           # Setup-Skript
├── setup.txt                          # Installationsanleitung
│
├── entrance_detection.py              # Basis-Eingangserkennung
├── advanced_entrance_detection.py    # Erweiterte Erkennung + 3D
├── lidar_integration_example.py      # LiDAR-Integration
│
├── opencv_viewer_example.py          # RealSense Viewer Beispiel
├── python-tutorial-1-depth.py        # Depth-Tutorial
├── python-rs400-advanced-mode-example.py
├── rs_launch.py                      # RealSense Launcher
│
├── tests/                            # Unit Tests (geplant)
│   ├── test_entrance_detection.py
│   ├── test_lidar_processing.py
│   └── test_sensor_fusion.py
│
├── data/                             # Testdaten
│   ├── recordings/                   # RealSense Recordings
│   └── lidar_scans/                  # LiDAR Scan-Daten
│
├── docs/                             # Zusätzliche Dokumentation
│   ├── api_reference.md
│   ├── calibration_guide.md
│   └── troubleshooting.md
│
└── results/                          # Ergebnisse/Logs
    ├── visualizations/
    ├── performance_logs/
    └── detection_results/
```

---

## 🔌 API-Referenz

### EntranceDetector (Basis)

```python
detector = EntranceDetector()

# Hauptmethode
entrances, mask, lines = detector.detect_entrances(depth_frame, color_frame)

# Parameter anpassen
detector.min_entrance_width = 0.8
detector.max_entrance_width = 2.0
detector.depth_threshold = 3.0

# Visualisierung
output = detector.visualize(color_image, entrances, mask, lines)

# Hauptschleife
detector.run()
```

### MultiSensorEntranceDetector (Erweitert)

```python
detector = MultiSensorEntranceDetector(use_lidar=False)

# Frame verarbeiten
color_img, depth_img, entrances = detector.process_frame()

# Entrance-Objekt
entrance = entrances[0]
entrance.position_2d      # (x, y) in Pixeln
entrance.position_3d      # (X, Y, Z) in Metern
entrance.width            # Breite in Metern
entrance.depth            # Entfernung in Metern
entrance.confidence       # 0.0 - 1.0
entrance.is_passable      # Boolean

# LiDAR-Integration
detector.integrate_lidar_data(scan_data)

# Visualisierung
output = detector.visualize(color_image, entrances)
```

### LidarProcessor

```python
lidar = LidarProcessor()

# Scan verarbeiten
lidar.process_scan(scan_data)  # [(angle, distance), ...]

# Lücken im Kamera-FOV
gaps = lidar.get_gaps_in_camera_fov(camera_fov_deg=69)

# Gap-Objekt
gap = gaps[0]
gap['width']        # Breite in Metern
gap['center']       # (x, y) Position
gap['distance']     # Entfernung
gap['angle']        # Winkel in Grad

# Visualisierung
lidar.visualize_scan()
```

### FusedEntranceDetector

```python
camera_detector = MultiSensorEntranceDetector()
lidar_processor = LidarProcessor()
fused = FusedEntranceDetector(camera_detector, lidar_processor)

# Fusionierte Ergebnisse
results = fused.fuse_detections(camera_entrances, lidar_gaps)

# Result-Objekt
result = results[0]
result['entrance']           # Entrance-Objekt
result['lidar_match']        # Matched Gap
result['confidence']         # Fusionierte Konfidenz
result['confirmed_by_lidar'] # Boolean
```

---

## 🧪 Testing & Validierung

### Test-Szenarien

1. **Einzelner Eingang**
   - Frontale Ansicht
   - Verschiedene Distanzen (0.5m - 5m)
   - Verschiedene Breiten (0.7m - 2.5m)

2. **Multiple Eingänge**
   - Mehrere Türen im Sichtfeld
   - Überlappende Detektionen
   - Priorisierung nach Konfidenz

3. **Herausforderungen**
   - Schlechte Beleuchtung
   - Reflektierende Oberflächen
   - Transparente Türen (Glas)
   - Offene vs. geschlossene Türen
   - Teilweise verdeckte Eingänge

4. **Kantenfälle**
   - Sehr breite Öffnungen (>2.5m)
   - Sehr schmale Durchgänge (<0.7m)
   - Niedrige Decken
   - Stufen/Rampen im Eingang

### Performance-Metriken

| Metrik | Zielwert | Aktuell | Status |
|--------|----------|---------|--------|
| Erkennungsrate | >90% | TBD | 🔄 Testing |
| Falsch-Positiv-Rate | <5% | TBD | 🔄 Testing |
| Verarbeitungszeit | <100ms | TBD | 🔄 Testing |
| FPS | >10 | TBD | 🔄 Testing |
| Positionsgenauigkeit | <10cm | TBD | 🔄 Testing |
| Breitengenauigkeit | <5cm | TBD | 🔄 Testing |

### Benchmarking

```python
import time

# Performance-Test
times = []
for i in range(100):
    start = time.time()
    color_img, depth_img, entrances = detector.process_frame()
    elapsed = time.time() - start
    times.append(elapsed)

print(f"Durchschnitt: {np.mean(times)*1000:.2f}ms")
print(f"FPS: {1/np.mean(times):.1f}")
print(f"Min: {np.min(times)*1000:.2f}ms")
print(f"Max: {np.max(times)*1000:.2f}ms")
```

---

## 🚀 Deployment

### Produktiv-Einstellungen

```python
# Optimierte Parameter für Echtzeit
detector = MultiSensorEntranceDetector(use_lidar=True)

# Reduzierte Auflösung für Performance
config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30)

# Aggressive Filterung
detector.spatial_filter.set_option(rs.option.filter_magnitude, 3)
detector.temporal_filter.set_option(rs.option.filter_smooth_alpha, 0.6)

# Engere Validierung
detector.confidence_threshold = 0.7
detector.max_detection_range = 3.0
```

### ROS-Integration (geplant)

```python
# ROS Node für Eingangserkennung
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

class EntranceDetectionNode:
    def __init__(self):
        rospy.init_node('entrance_detection')
        self.pub = rospy.Publisher('/entrances', PoseStamped, queue_size=10)
        self.detector = MultiSensorEntranceDetector()
    
    def publish_entrance(self, entrance):
        pose = PoseStamped()
        pose.pose.position.x = entrance.position_3d[0]
        pose.pose.position.y = entrance.position_3d[1]
        pose.pose.position.z = entrance.position_3d[2]
        self.pub.publish(pose)
```

---

## 📈 Roadmap

### Phase 1: Grundfunktionalität ✅
- [x] RealSense Integration
- [x] Basis-Eingangserkennung
- [x] 3D-Projektion
- [x] Visualisierung

### Phase 2: Erweiterte Features 🔄
- [x] Tiefenfilterung
- [x] Vertikale Strukturerkennung
- [x] Konfidenz-Scoring
- [x] LiDAR-Integration (Basis)
- [ ] Performance-Optimierung

### Phase 3: Validierung & Testing 📋
- [ ] Test-Suite erstellen
- [ ] Benchmark-Szenarien
- [ ] Performance-Messung
- [ ] Kalibrierungs-Tools

### Phase 4: Produktion 🎯
- [ ] ROS-Integration
- [ ] SLAM-Integration
- [ ] Pfadplanung
- [ ] Echtzeitnavigation
- [ ] Dokumentation vervollständigen

---

## 👥 Team & Kontakt

**Projekt**: Bewegungserweiterung (MPEC)  
**Repository**: github.com/eliasbuergin/Bewegungserweiterung  
**Entwickler**: Elias Bürgin  
**Institution**: [Ihre Institution]  

---

## 📚 Referenzen & Ressourcen

### Dokumentation
- [Intel RealSense SDK](https://dev.intelrealsense.com/)
- [OpenCV Documentation](https://docs.opencv.org/)
- [NumPy Documentation](https://numpy.org/doc/)

### Paper & Forschung
- "Depth-Based Object Detection and Tracking" (2019)
- "Multi-Sensor Fusion for Indoor Navigation" (2020)
- "Real-Time Entrance Detection for Autonomous Robots" (2021)

### Tools & Libraries
- pyrealsense2: RealSense Python Wrapper
- OpenCV: Computer Vision
- NumPy: Numerische Berechnungen
- Matplotlib: Visualisierung
- RPLidar: LiDAR Interface

---

## 📝 Changelog

### Version 0.3.0 (2025-12-01)
- LiDAR-Integration hinzugefügt
- Sensor-Fusion implementiert
- Projektdokumentation erstellt

### Version 0.2.0 (2025-11-XX)
- 3D-Projektion implementiert
- Erweiterte Filterung
- Konfidenz-Scoring

### Version 0.1.0 (2025-11-XX)
- Initiale Version
- Basis-Eingangserkennung
- RealSense Integration

---

**Letzte Aktualisierung**: 1. Dezember 2025  
**Status**: In Entwicklung  
**Version**: 0.3.0
