import cv2
import numpy as np
import time
from picamera2 import Picamera2

class CameraModule:
    def __init__(self, resolution=(640, 480), framerate=30):
        # Initialize the camera
        self.picam2 = Picamera2()
        self.config = self.picam2.create_video_configuration(
            main={"size": resolution, "format": "RGB888"},
            controls={"FrameRate": framerate}
        )
        self.picam2.configure(self.config)
        self.picam2.start()
        
        # Allow camera to warm up
        time.sleep(2)
        
        # Video recording setup
        self.recording = False
        self.video_writer = None
        self.output_file = None
    
    def start_recording(self, output_file="lane_detection.mp4", fps=20.0):
        """Start recording video to a file"""
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use 'XVID' if MP4 doesn't work
        h, w = self.picam2.camera_properties["ScalerCropMaximum"][3:1:-1]
        self.video_writer = cv2.VideoWriter(output_file, fourcc, fps, (w, h))
        self.recording = True
        self.output_file = output_file
        print(f"Recording started: {output_file}")
    
    def stop_recording(self):
        """Stop recording and save the video file"""
        if self.recording and self.video_writer is not None:
            self.recording = False
            self.video_writer.release()
            print(f"Recording saved: {self.output_file}")
    
    def capture_frame(self):
        """Capture a frame from the camera"""
        return self.picam2.capture_array()
    
    def record_frame(self, frame):
        """Add frame to the video recording if active"""
        if self.recording and self.video_writer is not None:
            self.video_writer.write(frame)
    
    def release(self):
        """Release all resources"""
        self.stop_recording()
        self.picam2.stop()
        print("Camera released")