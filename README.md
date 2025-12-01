# Bewegungserweiterung - Eingangserkennung mit RealSense & LiDAR

Dieses Projekt nutzt eine Intel RealSense Tiefenkamera (und optional einen LiDAR) zur automatischen Erkennung von Eingängen/Türen für autonome Navigation.

## 🎯 Funktionen

- **Tiefenbasierte Eingangserkennung**: Erkennt Türöffnungen durch Analyse von Tiefensprüngen
- **Vertikale Strukturerkennung**: Identifiziert Türrahmen mittels Kantenerkennung
- **3D-Positionsbestimmung**: Berechnet exakte 3D-Koordinaten der Eingänge
- **Dimensionsmessung**: Misst Breite und Höhe der Eingänge
- **Befahrbarkeitsprüfung**: Validiert ob Eingänge passierbar sind
- **Multi-Sensor-Fusion**: Kombiniert Tiefenkamera und LiDAR-Daten

## 📁 Dateien

- `entrance_detection.py` - Einfache Eingangserkennung (Einstieg)
- `advanced_entrance_detection.py` - Erweiterte Erkennung mit 3D-Projektion und Filterung
- `opencv_viewer_example.py` - Basis-Beispiel für RealSense
- `python-tutorial-1-depth.py` - Einfaches Tiefendaten-Tutorial

## 🚀 Installation

### 1. RealSense SDK installieren (siehe setup.txt)
```bash
# Auf Ubuntu/Linux
pip3 install pyrealsense2
```

### 2. Python-Abhängigkeiten
```bash
pip install opencv-python numpy pyrealsense2
```

## � Quick Start

### Installation
```bash
# Python-Abhängigkeiten
pip install opencv-python numpy pyrealsense2

# Optional: LiDAR-Support
pip install rplidar-roboticia matplotlib
```

### Hauptsystem starten
```bash
python adaptive_height_control.py
```

**Steuerung:**
- `q` - Beenden
- `s` - Screenshot
- `e` - Emergency Stop (Toggle)

## � MVP-Anforderungen (Meilensteine)

### ✅ Phase 1: Setup (Abgeschlossen)
- [x] Hardware-Integration (RealSense)
- [x] Basis-Tiefenerkennung
- [x] Visualisierung

### 🔄 Phase 2: Schnittstellen (MS2 - Aktuell)
- [x] LiDAR-Daten auslesen
- [x] Stereo-Depth-Verarbeitung
- [ ] **Unitree SDK Integration** 🎯
  - [ ] `SetBodyHeight()` implementieren
  - [ ] `SetPostureMode()` implementieren
  - [ ] Bewegungs-Feedback auslesen

### 📋 Phase 3: Logik (MS3 - Next)
- [x] Durchgangs-Geometrie-Extraktion
- [x] Höhenberechnung mit Sicherheit
- [x] State Machine (4 Modi)
- [ ] **SDK-Ansteuerung verbinden**
- [ ] Bewegungsablauf implementieren
- [ ] Safety-Override testen

### 🎯 Phase 4: Integration & Test
- [ ] End-to-End Tests
- [ ] Performance-Optimierung
- [ ] Feldtests in echter Umgebung
- [ ] Dokumentation finalisieren

## 🧮 Technische Details

### Haltungs-Modi

| Modus | Höhe | Verwendung |
|-------|------|------------|
| **NORMAL** | 30cm | Freie Fahrt, keine Hindernisse |
| **NIEDRIG** | 15cm | Durchgänge 15-25cm Höhe |
| **LIEGEND** | 8cm | Durchgänge 10-15cm Höhe |
| **BLOCKIERT** | - | < 10cm, zu eng, Stopp |

### Sicherheitsparameter
- **Sicherheitsabstand**: 5-10cm über Durchgangshöhe
- **Annäherung**: Stopp bei 0,5m vor Durchgang
- **Geschwindigkeit**: 0,1 m/s beim Durchqueren
- **Max. Neigung**: 30° (schräge Decken)

### Sensor-Reichweiten
- **Tiefenkamera**: 0,3m - 3,0m (optimal: 0,5m - 2,0m)
- **LiDAR**: 0,1m - 12m
- **Min. Durchgangsbreite**: 35cm (Roboter + Sicherheit)
- **Min. Durchgangshöhe**: 10cm (absolute Grenze)

## 🔧 Code-Struktur

### Hauptklasse: `AdaptiveHeightController`

```python
# Initialisierung
controller = AdaptiveHeightController(use_lidar=False)

# Hauptschleife
controller.run()

# Interner Ablauf pro Frame:
# 1. detect_ceiling_obstacle()     # INPUT
# 2. calculate_passage_geometry()  # PROCESSING  
# 3. decide_posture()              # DECISION
# 4. actuate_height_change()       # OUTPUT
```

### Datenstrukturen

```python
@dataclass
class PassageGeometry:
    height: float          # Durchgangshöhe in Metern
    width: float           # Breite in Metern
    distance: float        # Entfernung zum Durchgang
    tilt_angle: float      # Neigung in Grad
    confidence: float      # Erkennungsqualität 0-1

@dataclass  
class RobotState:
    current_height: float       # Aktuelle Körperhöhe
    current_mode: PostureMode   # NORMAL/LOW/PRONE/BLOCKED
    is_adjusting: bool          # Gerade am Anpassen?
    emergency_stop: bool        # Notaus aktiv?
```

## 🎓 Algorithmen im Detail

### 1. Deckenerkennung
```python
# Obere Bildhälfte analysieren
ceiling_roi = depth_image[10%:60%, :]

# Nahe Hindernisse finden (< 3m)
close_obstacles = ceiling_distances < 3.0

# Kontur-Extraktion → Bounding Box
```

### 2. Höhenberechnung
```python
# 3D-Projektion der Unterkante
bottom_3d = rs2_deproject_pixel_to_point([x, y], depth)

# Höhe = Abstand Boden → Hindernis
passage_height = abs(bottom_3d[1] - ground_3d[1])

# Mit Sicherheit
target_height = passage_height - safety_clearance
```

### 3. State Machine
```python
if target_height >= 0.25:
    return NORMAL, 0.30
elif target_height >= 0.15:
    return LOW, target_height  # Ducken
elif target_height >= 0.10:
    return PRONE, 0.08         # Hinlegen
else:
    return BLOCKED, current    # Zu eng
```

## 🔌 Unitree SDK Integration (TODO)

### Benötigte Funktionen
```python
# In adaptive_height_control.py, Methode: actuate_height_change()

# TODO: Ersetze Stub durch echte SDK-Calls:

from unitree_sdk import RobotInterface  # Beispiel

robot = RobotInterface()

# Höhe setzen
robot.SetBodyHeight(target_height)  # in Metern

# Haltung wechseln
robot.SetPostureMode(mode)  # "stand", "crouch", "prone"

# Status abfragen
current_height = robot.GetBodyHeight()
is_stable = robot.IsStable()

# Notaus
robot.EmergencyStop()
```

### Integration Steps
1. Unitree SDK installieren/importieren
2. `actuate_height_change()` anpassen
3. Bewegungs-Feedback implementieren
4. Safety-Checks mit IMU verbinden
5. Testen mit echtem Roboter

## 🧪 Testing

### Simulation
```bash
# Teste ohne Hardware (mit Recording)
python adaptive_height_control.py
# Halte Objekte vor Kamera in verschiedenen Höhen
```

### Mit Roboter
1. Roboter in sicherer Umgebung platzieren
2. Niedrige Hindernisse (20cm, 15cm, 10cm) vorbereiten
3. System starten
4. Langsam Hindernisse annähern
5. Beobachte Höhenanpassung

### Testfälle
- [ ] Normaler Durchgang (>25cm) → Keine Anpassung
- [ ] Niedriger Durchgang (20cm) → Ducken
- [ ] Sehr niedriger Durchgang (12cm) → Hinlegen
- [ ] Zu enger Durchgang (8cm) → Blockiert
- [ ] Schräge Decke (15° Neigung) → Anpassung
- [ ] Emergency Stop → Sofortiger Halt

## 📚 Weitere Ressourcen

## 📚 Weitere Ressourcen

- **PROJECT_SPEC.md** - Vollständige technische Spezifikation
- [Intel RealSense Documentation](https://dev.intelrealsense.com/)
- [Unitree Robotics](https://www.unitree.com/)
- [OpenCV Tutorials](https://docs.opencv.org/)

## 👥 Projekt-Info

**Repository**: github.com/eliasbuergin/Bewegungserweiterung  
**Entwickler**: Elias Bürgin  
**Projekt**: MPEC - Motion Path Extension and Control  
**Zweck**: Autonome Höhlenerkundung mit Unitree GO2

---

## 📝 Nächste Schritte für Entwicklung

### Priorität 1: SDK-Integration (MS2)
```bash
# TODO:
1. Unitree SDK installieren
2. actuate_height_change() mit echten Calls ersetzen
3. Feedback-Loop implementieren (GetBodyHeight)
4. Safety-Checks mit IMU verbinden
```

### Priorität 2: Bewegungsablauf (MS3)
```bash
# TODO:
1. Annäherungs-Sequenz (langsam bis 0.5m)
2. Höhenanpassung (2-3 Sekunden warten)
3. Durchquerung (0.1 m/s, Kollisionserkennung)
4. Zurück zu Normal (nach Durchgang)
```

### Priorität 3: Robustheit
```bash
# TODO:
1. Mehrfach-Messungen für Stabilität
2. Kalman-Filter für Höhenschätzung
3. Fehlerbehandlung (Sensorfehler, Timeouts)
4. Logging & Telemetrie
```

**Status**: MVP in Entwicklung | Version 0.3.0 | Stand: 1. Dezember 2025
