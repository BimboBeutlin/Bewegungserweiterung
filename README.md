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

## 💻 Verwendung

### Einfache Eingangserkennung
```bash
python entrance_detection.py
```

### Erweiterte Erkennung mit 3D-Berechnung
```bash
python advanced_entrance_detection.py
```

**Steuerung:**
- `q` - Beenden
- `s` - Screenshot speichern

## 🔧 Wie funktioniert die Eingangserkennung?

### 1. Tiefenanalyse
Die Kamera misst die Entfernung zu Objekten. Eingänge/Türen haben charakteristische Eigenschaften:
- **Große Tiefenwerte** oder **fehlende Messwerte** (freier Raum)
- **Tiefensprünge** an den Rändern (Türrahmen)

### 2. Kantenerkennung
Vertikale Kanten werden erkannt:
- Infrarot-Bild für besseren Kontrast
- Canny-Kantendetektor
- Hough-Transform für Linienerkennung
- Filterung nach vertikalen Linien (Türrahmen)

### 3. Kandidatenvalidierung
Jeder Kandidat wird geprüft:
- ✓ Breite: 0,7 - 2,5 Meter
- ✓ Höhe: mindestens 1,8 Meter (befahrbar)
- ✓ Aspect Ratio: höher als breit
- ✓ Entfernung: < 5 Meter

### 4. 3D-Projektion
Pixel-Koordinaten werden in 3D-Weltkoordinaten umgerechnet:
```python
point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)
```

## 🤖 LiDAR-Integration

Der LiDAR kann zusätzliche Informationen liefern:

### Vorteile der Kombination:
- **LiDAR**: 360° 2D-Scan auf Bodenhöhe, präzise Distanzen
- **Tiefenkamera**: 3D-Information, Höhenerkennung, visuelle Details

### Verwendung:
```python
detector = MultiSensorEntranceDetector(use_lidar=True)

# LiDAR-Daten integrieren
lidar_scan = [...]  # Array von (angle, distance)
detector.integrate_lidar_data(lidar_scan)
```

### Beispiel LiDAR-Interface:
```python
# Für RPLidar
from rplidar import RPLidar

lidar = RPLidar('/dev/ttyUSB0')
for scan in lidar.iter_scans():
    detector.integrate_lidar_data(scan)
```

## 📊 Parameter anpassen

In `advanced_entrance_detection.py`:
```python
self.min_entrance_width = 0.7      # Minimale Breite (Meter)
self.max_entrance_width = 2.5      # Maximale Breite
self.min_entrance_height = 1.8     # Mindesthöhe für Befahrbarkeit
self.max_detection_range = 5.0     # Maximale Erkennungsdistanz
```

## 🎓 Algorithmus im Detail

1. **Frame-Erfassung**: Depth + Color + IR Streams synchronized
2. **Filterung**: Spatial, Temporal, Hole-Filling Filter
3. **Tiefensprung-Detektion**: Sobel-Filter für Gradienten
4. **Strukturerkennung**: Vertikale Linien im IR-Bild
5. **Kandidatenfindung**: Linienpaare analysieren
6. **Dimensionsberechnung**: 3D-Projektion
7. **Validierung**: Konfidenz-Score berechnen
8. **Ausgabe**: Visualisierung + 3D-Koordinaten

## 🐛 Troubleshooting

### Kamera nicht gefunden
```bash
# USB-Verbindung prüfen
rs-enumerate-devices

# Rechte setzen (Linux)
sudo usermod -a -G plugdev $USER
```

### Schlechte Erkennung
- Beleuchtung verbessern
- Kamera-Parameter anpassen (Exposure)
- Filter-Schwellwerte tunen
- Mindestabstand einhalten (> 0,5m)

### Performance-Probleme
- Auflösung reduzieren (320x240)
- FPS reduzieren (15 statt 30)
- Nur Depth-Stream nutzen

## 📚 Ressourcen

- [Intel RealSense Documentation](https://dev.intelrealsense.com/)
- [OpenCV Tutorials](https://docs.opencv.org/)
- [RealSense Python Examples](https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python/examples)

## 🎯 Nächste Schritte

1. **Echte LiDAR-Integration** implementieren
2. **SLAM** für Kartierung hinzufügen
3. **Navigation** basierend auf erkannten Eingängen
4. **Machine Learning** für robustere Erkennung
5. **ROS-Integration** für Roboter-Plattformen
