# test_TestKamera.py
import pytest
import pyrealsense2 as rs
import numpy as np

def test_realsense_pipeline_creation():
    """Test ob eine RealSense Pipeline erstellt werden kann"""
    pipeline = rs.pipeline()
    assert pipeline is not None

def test_realsense_context():
    """Test ob RealSense-Geräte erkannt werden"""
    ctx = rs.context()
    devices = ctx.query_devices()
    assert len(devices) > 0, "Keine RealSense-Kamera gefunden"

def test_d435i_device_info():
    """Test ob D435i erkannt wird und Geräteinformationen abrufbar sind"""
    ctx = rs.context()
    devices = ctx.query_devices()
    
    device = devices[0]
    name = device.get_info(rs.camera_info.name)
    serial = device.get_info(rs.camera_info.serial_number)
    
    assert name is not None
    assert serial is not None
    print(f"Gefundenes Gerät: {name}, Serial: {serial}")

def test_stream_configuration():
    """Test ob Streams konfiguriert werden können"""
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    assert config is not None

def test_capture_frames():
    """Test ob Frames von der D435i erfasst werden können"""
    pipeline = rs.pipeline()
    config = rs.config()
    
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        pipeline.start(config)
        
        # Erste Frames überspringen (Kamera-Aufwärmphase)
        for _ in range(30):
            pipeline.wait_for_frames()
        
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        assert depth_frame is not None
        assert color_frame is not None
        
        # Prüfe Frame-Dimensionen
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        assert depth_image.shape == (480, 640)
        assert color_image.shape == (480, 640, 3)
        
    finally:
        pipeline.stop()

def test_imu_streams():
    """Test ob IMU-Daten (Gyro/Accel) von D435i verfügbar sind"""
    pipeline = rs.pipeline()
    config = rs.config()
    
    config.enable_stream(rs.stream.accel)
    config.enable_stream(rs.stream.gyro)
    
    try:
        pipeline.start(config)
        
        frames = pipeline.wait_for_frames()
        accel_frame = frames.first_or_default(rs.stream.accel)
        gyro_frame = frames.first_or_default(rs.stream.gyro)
        
        # IMU-Streams sollten vorhanden sein (D435i hat IMU)
        assert accel_frame is not None
        assert gyro_frame is not None
        
    finally:
        pipeline.stop()

def test_depth_measurement():
    """Test ob Tiefenmessungen plausible Werte liefern"""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    try:
        pipeline.start(config)
        
        for _ in range(30):
            pipeline.wait_for_frames()
        
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        
        # Messe Distanz in der Bildmitte
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        distance = depth_frame.get_distance(width // 2, height // 2)
        
        # Distanz sollte zwischen 0.1m und 10m liegen (typischer Bereich)
        assert 0.1 <= distance <= 10.0, f"Unplausible Distanz: {distance}m"
        
    finally:
        pipeline.stop()