#!/usr/bin/env python3
import cv2
import numpy as np
import time
import sys
from car_control import Car  # Import the Car class from your existing file

# Check if running on Raspberry Pi
try:
    from picamera2 import Picamera2
    ON_RASPBERRY_PI = True
except ImportError:
    ON_RASPBERRY_PI = False
    print("Not running on Raspberry Pi or PiCamera2 not installed.")
    print("Will attempt to use webcam instead.")

class LaneDetector:
    def __init__(self):
        # Initialize camera
        if ON_RASPBERRY_PI:
            self.camera = Picamera2()
            config = self.camera.create_preview_configuration(
                main={"size": (640, 480)},
                lores={"size": (320, 240), "format": "YUV420"}
            )
            self.camera.configure(config)
            self.camera.start()
            time.sleep(2)  # Give camera time to warm up
        else:
            # Use webcam as fallback for testing
            self.camera = cv2.VideoCapture(0)
            if not self.camera.isOpened():
                raise Exception("Could not open camera/video stream")
        
        # Parameters for lane detection
        self.yellow_lower = np.array([20, 100, 100], dtype=np.uint8)  # HSV range for yellow
        self.yellow_upper = np.array([40, 255, 255], dtype=np.uint8)
        
        # Lane following parameters
        self.frame_center_x = 320  # Assuming 640x480 resolution
        self.target_x = self.frame_center_x  # Where we want the lane center to be
        self.error_threshold = 60  # Acceptable error in pixels
        self.lane_history = []  # To store recent lane positions for smoothing
        self.history_size = 5
        
        # Control parameters
        self.max_speed = 40  # Maximum speed (0-100)
        self.turn_speed = 30  # Speed during turns
        self.pid = {
            'Kp': 0.5,  # Proportional gain
            'Ki': 0.05,  # Integral gain
            'Kd': 0.1,   # Derivative gain
            'integral': 0,
            'last_error': 0
        }

    def capture_frame(self):
        """Capture a frame from the camera"""
        if ON_RASPBERRY_PI:
            return self.camera.capture_array()
        else:
            ret, frame = self.camera.read()
            if not ret:
                raise Exception("Failed to capture frame")
            return frame

    def preprocess_frame(self, frame):
        """Preprocess the frame for lane detection"""
        # Convert to HSV for better color filtering
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for yellow color
        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.erode(yellow_mask, kernel, iterations=1)
        yellow_mask = cv2.dilate(yellow_mask, kernel, iterations=1)
        
        # Focus on bottom half of the image (where lanes are more relevant)
        height, width = yellow_mask.shape
        roi_mask = yellow_mask[height//2:height, :]
        
        return yellow_mask, roi_mask, height, width

    def detect_lanes(self, mask, original_frame):
        """Detect lanes in the processed image"""
        height, width = mask.shape
        
        # Find contours in the masked image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by size to remove noise
        filtered_contours = [c for c in contours if cv2.contourArea(c) > 100]
        
        lane_centers = []
        # Process each contour
        for contour in filtered_contours:
            # Calculate the center of the contour
            M = cv2.moments(contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"]) + height//2  # Adjust for ROI
                lane_centers.append(cx)
                
                # Draw contour and center for visualization
                if original_frame is not None:
                    cv2.drawContours(original_frame, [contour], -1, (0, 255, 0), 2)
                    cv2.circle(original_frame, (cx, cy), 5, (255, 0, 0), -1)
        
        return lane_centers, original_frame

    def compute_steering(self, lane_centers):
        """Calculate steering direction based on lane positions"""
        if not lane_centers:
            return 0, "No lanes detected"
        
        # Calculate the center point between lanes
        lane_center = sum(lane_centers) / len(lane_centers)
        
        # Update history for smoothing
        self.lane_history.append(lane_center)
        if len(self.lane_history) > self.history_size:
            self.lane_history.pop(0)
        
        # Smooth the lane center position
        smooth_center = sum(self.lane_history) / len(self.lane_history)
        
        # Calculate error (distance from center of frame)
        error = self.target_x - smooth_center
        
        # PID control
        self.pid['integral'] += error
        # Limit integral to prevent windup
        self.pid['integral'] = max(min(self.pid['integral'], 100), -100)
        
        derivative = error - self.pid['last_error']
        self.pid['last_error'] = error
        
        # Calculate steering value using PID formula
        steering = (self.pid['Kp'] * error +
                   self.pid['Ki'] * self.pid['integral'] +
                   self.pid['Kd'] * derivative)
        
        status = f"Lane: {int(smooth_center)}, Error: {int(error)}, Steering: {int(steering)}"
        
        return steering, status

    def control_car(self, car, steering):
        """Control the car based on steering value"""
        # Normalize steering to -1 (full left) to 1 (full right)
        normalized_steering = max(min(steering / 100, 1), -1)
        
        # Determine speed and direction based on steering
        if abs(normalized_steering) < 0.2:
            # Going mostly straight - full speed ahead
            car.left_motor.set_target_speed(self.max_speed)
            car.right_motor.set_target_speed(self.max_speed)
        elif normalized_steering > 0:
            # Turn right - slow down right wheel
            right_speed = self.max_speed * (1 - abs(normalized_steering))
            car.left_motor.set_target_speed(self.max_speed)
            car.right_motor.set_target_speed(max(0, right_speed))
        else:
            # Turn left - slow down left wheel
            left_speed = self.max_speed * (1 - abs(normalized_steering))
            car.left_motor.set_target_speed(max(0, left_speed))
            car.right_motor.set_target_speed(self.max_speed)
        
    def cleanup(self):
        """Release resources"""
        if ON_RASPBERRY_PI:
            self.camera.stop()
        else:
            self.camera.release()
        cv2.destroyAllWindows()

def main():
    # Initialize car and lane detector
    car = Car()
    detector = LaneDetector()
    
    # Start car control thread
    car.start_control_thread()
    
    # Enable display if not on Raspberry Pi or if specifically requested
    display_enabled = not ON_RASPBERRY_PI or "--display" in sys.argv
    
    try:
        while True:
            # Capture and process frame
            frame = detector.capture_frame()
            yellow_mask, roi_mask, height, width = detector.preprocess_frame(frame)
            
            # Detect lanes
            display_frame = frame.copy() if display_enabled else None
            lane_centers, annotated_frame = detector.detect_lanes(roi_mask, display_frame)
            
            # Calculate steering
            steering, status = detector.compute_steering(lane_centers)
            
            # Control the car
            detector.control_car(car, steering)
            
            # Display status and visualization if enabled
            print(status, end='\r')
            
            if display_enabled:
                # Draw center line and target
                cv2.line(annotated_frame, 
                         (detector.frame_center_x, 0), 
                         (detector.frame_center_x, height), 
                         (255, 0, 0), 2)
                
                # Display the frame
                cv2.imshow('Lane Detection', annotated_frame)
                cv2.imshow('Yellow Mask', yellow_mask)
                
                # Check for exit key
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            # Control the loop speed
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"\nError: {str(e)}")
    finally:
        # Cleanup
        car.stop()
        car.cleanup()
        detector.cleanup()
        print("\nProgram terminated")

if __name__ == "__main__":
    main()