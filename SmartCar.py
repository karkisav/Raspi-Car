#!/usr/bin/env python3
import time
import sys
import threading
import cv2
import numpy as np
from picamera2 import Picamera2
import math

# Import your existing car control module
# Assuming the file is named car_control.py
# If it has a different name, adjust accordingly
from car_control import Car

class LaneDetection:
    def __init__(self):
        # Lane detection parameters
        self.lower_white = np.array([0, 0, 200])
        self.upper_white = np.array([255, 30, 255])
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])
        
    def detect_lanes(self, frame):
        """Detect lanes in the given frame"""
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create masks for white and yellow lanes
        white_mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
        yellow_mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # Combine masks
        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
        
        # Apply mask to the original frame
        result = cv2.bitwise_and(frame, frame, mask=combined_mask)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(combined_mask, (5, 5), 0)
        
        # Apply Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)
        
        # Define region of interest (bottom half of the frame)
        height, width = edges.shape
        mask = np.zeros_like(edges)
        roi_vertices = np.array([[(0, height), (0, height/2), 
                                 (width, height/2), (width, height)]], dtype=np.int32)
        cv2.fillPoly(mask, roi_vertices, 255)
        roi_edges = cv2.bitwise_and(edges, mask)
        
        # Detect lines using Hough transform
        lines = cv2.HoughLinesP(roi_edges, 1, np.pi/180, 50, 
                               minLineLength=40, maxLineGap=100)
        
        # Separate left and right lane lines
        left_lines = []
        right_lines = []
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x1 == x2:
                    continue  # Skip vertical lines
                    
                slope = (y2 - y1) / (x2 - x1)
                
                # Filter lines based on slope
                if slope < 0:  # Left lane
                    left_lines.append(line[0])
                else:  # Right lane
                    right_lines.append(line[0])
        
        return left_lines, right_lines, result

    def calculate_lane_position(self, frame, left_lines, right_lines):
        """Calculate position of the car relative to the center of the lane"""
        height, width = frame.shape[:2]
        
        # If no lane lines detected, return None
        if len(left_lines) == 0 and len(right_lines) == 0:
            return None, None, None
        
        # Calculate the average position of the left and right lanes
        left_x = 0
        right_x = width
        
        if len(left_lines) > 0:
            left_x_sum = sum([x1 + x2 for x1, y1, x2, y2 in left_lines]) / 2
            left_x = left_x_sum / len(left_lines)
        
        if len(right_lines) > 0:
            right_x_sum = sum([x1 + x2 for x1, y1, x2, y2 in right_lines]) / 2
            right_x = right_x_sum / len(right_lines)
        
        # Calculate the center of the lane
        lane_center = (left_x + right_x) / 2
        
        # Calculate the car's position
        car_position = width / 2
        
        # Calculate the offset from the center (positive: right of center, negative: left of center)
        offset = car_position - lane_center
        
        # Normalize offset to a value between -1 and 1
        normalized_offset = offset / (width / 2)
        
        return lane_center, car_position, normalized_offset

class ObjectDetection:
    def __init__(self):
        # Load pre-trained object detection model for traffic lights and vehicles
        # This is a simplified version - you might want to use a proper model (YOLO, SSD, etc.)
        self.car_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_car.xml')
        
        # For traffic lights, a color-based approach (simplified)
        self.lower_red = np.array([0, 70, 50])
        self.upper_red = np.array([10, 255, 255])
        self.lower_green = np.array([40, 70, 50])
        self.upper_green = np.array([80, 255, 255])
        
    def detect_cars(self, frame):
        """Detect cars in the given frame"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cars = self.car_cascade.detectMultiScale(gray, 1.1, 2)
        
        car_detected = len(cars) > 0
        car_boxes = []
        
        for (x, y, w, h) in cars:
            car_boxes.append((x, y, x+w, y+h))
            
        return car_detected, car_boxes
    
    def detect_traffic_light(self, frame):
        """Detect traffic lights and their colors"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Detect red and green colors
        red_mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
        green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
        # Count non-zero pixels in each mask
        red_count = cv2.countNonZero(red_mask)
        green_count = cv2.countNonZero(green_mask)
        
        # Simple threshold-based detection
        red_light = red_count > 500
        green_light = green_count > 500
        
        return red_light, green_light

class AutonomousCar:
    def __init__(self):
        # Initialize the car controller
        self.car = Car()
        
        # Initialize the camera
        self.camera = Picamera2()
        self.camera.configure(self.camera.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
        self.camera.start()
        time.sleep(2)  # Allow camera to warm up
        
        # Initialize lane and object detectors
        self.lane_detector = LaneDetection()
        self.object_detector = ObjectDetection()
        
        # Control parameters
        self.running = False
        self.debug_mode = False
        self.current_lane = "right"  # Start in the right lane
        self.default_speed = 30
        self.lane_change_in_progress = False
        
        print("Autonomous car system initialized")
    
    def start(self, debug_mode=False):
        """Start the autonomous driving system"""
        self.debug_mode = debug_mode
        self.running = True
        
        # Start the car's speed control thread
        self.car.start_control_thread()
        
        # Start the vision processing thread
        self.vision_thread = threading.Thread(target=self._vision_loop)
        self.vision_thread.daemon = True
        self.vision_thread.start()
        
        print("Autonomous driving system started")
    
    def stop(self):
        """Stop the autonomous driving system"""
        self.running = False
        self.car.stop()
        time.sleep(0.5)
        print("Autonomous driving system stopped")
    
    def _vision_loop(self):
        """Main loop for processing camera frames and making driving decisions"""
        while self.running:
            # Capture frame
            frame = self.camera.capture_array()
            
            # Process frame for lane detection
            left_lines, right_lines, lane_result = self.lane_detector.detect_lanes(frame)
            lane_center, car_position, offset = self.lane_detector.calculate_lane_position(frame, left_lines, right_lines)
            
            # Process frame for object detection
            car_detected, car_boxes = self.object_detector.detect_cars(frame)
            red_light, green_light = self.object_detector.detect_traffic_light(frame)
            
            # Make driving decisions
            self._make_driving_decisions(offset, car_detected, car_boxes, red_light, green_light)
            
            # Display debug information if enabled
            if self.debug_mode:
                # Draw lane lines
                debug_frame = frame.copy()
                if left_lines is not None:
                    for x1, y1, x2, y2 in left_lines:
                        cv2.line(debug_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                
                if right_lines is not None:
                    for x1, y1, x2, y2 in right_lines:
                        cv2.line(debug_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                
                # Draw car position and lane center
                if lane_center is not None:
                    cv2.circle(debug_frame, (int(lane_center), 240), 5, (0, 255, 0), -1)
                    cv2.circle(debug_frame, (int(car_position), 240), 5, (255, 0, 255), -1)
                
                # Draw detected cars
                for x1, y1, x2, y2 in car_boxes:
                    cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                
                # Display status text
                cv2.putText(debug_frame, f"Lane: {self.current_lane}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                if red_light:
                    cv2.putText(debug_frame, "Red Light!", (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                if green_light:
                    cv2.putText(debug_frame, "Green Light!", (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                if car_detected:
                    cv2.putText(debug_frame, "Car Detected!", (10, 90), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                # Show the debug frame
                cv2.imshow("Autonomous Driving", debug_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.stop()
            
            time.sleep(0.1)  # Process at 10 Hz
    
    def _make_driving_decisions(self, offset, car_detected, car_boxes, red_light, green_light):
        """Make driving decisions based on detected information"""
        # Stop at red lights
        if red_light:
            self.car.stop()
            return
        
        # Handle car ahead - consider lane change
        if car_detected and not self.lane_change_in_progress:
            # Calculate distance to the car ahead (simplified)
            car_ahead = False
            for x1, y1, x2, y2 in car_boxes:
                # Check if car is in our lane
                box_center_x = (x1 + x2) / 2
                frame_center_x = 320  # Assuming 640x480 resolution
                
                # Check if the car is ahead of us in our lane
                if (self.current_lane == "right" and box_center_x > frame_center_x) or \
                   (self.current_lane == "left" and box_center_x < frame_center_x):
                    # Car is in our lane, check if it's close enough to warrant a lane change
                    if y2 > 240:  # Car is in the bottom half of the frame (closer to us)
                        car_ahead = True
            
            if car_ahead:
                # Initiate lane change
                target_lane = "left" if self.current_lane == "right" else "right"
                print(f"Car detected ahead, changing to {target_lane} lane")
                self._change_lane(target_lane)
                return
        
        # Regular lane following logic
        if offset is not None:
            # Convert offset to steering angle (-30 to 30 degrees)
            steering_angle = -offset * 30
            
            # Adjust speed based on curve sharpness
            curve_factor = min(1.0, 1.0 - abs(offset) * 0.5)
            adjusted_speed = self.default_speed * curve_factor
            
            # Apply steering
            if steering_angle < -5:
                # Turn left
                turn_factor = min(1.0, abs(steering_angle) / 30)
                left_speed = adjusted_speed * (1 - turn_factor)
                right_speed = adjusted_speed
                self.car.left_motor.set_target_speed(left_speed)
                self.car.right_motor.set_target_speed(right_speed)
            elif steering_angle > 5:
                # Turn right
                turn_factor = min(1.0, abs(steering_angle) / 30)
                left_speed = adjusted_speed
                right_speed = adjusted_speed * (1 - turn_factor)
                self.car.left_motor.set_target_speed(left_speed)
                self.car.right_motor.set_target_speed(right_speed)
            else:
                # Go straight
                self.car.forward(adjusted_speed)
        else:
            # No lane detected, slow down
            self.car.forward(self.default_speed * 0.5)
    
    def _change_lane(self, target_lane):
        """Perform a lane change maneuver"""
        self.lane_change_in_progress = True
        
        # First phase: move slightly forward while turning
        if target_lane == "left":
            self.car.left_motor.set_target_speed(self.default_speed * 0.3)
            self.car.right_motor.set_target_speed(self.default_speed)
        else:  # right lane
            self.car.left_motor.set_target_speed(self.default_speed)
            self.car.right_motor.set_target_speed(self.default_speed * 0.3)
        
        # Hold the turn for some time
        time.sleep(1.0)
        
        # Second phase: straighten out
        self.car.forward(self.default_speed)
        time.sleep(0.5)
        
        # Update current lane
        self.current_lane = target_lane
        self.lane_change_in_progress = False
        
        print(f"Lane change to {target_lane} completed")
    
    def cleanup(self):
        """Clean up resources"""
        self.stop()
        self.car.cleanup()
        if self.debug_mode:
            cv2.destroyAllWindows()
        self.camera.close()
        print("Autonomous car resources released")


if __name__ == "__main__":
    print("Raspberry Pi Autonomous Car System")
    
    # Parse command line arguments
    debug_mode = "--debug" in sys.argv
    
    try:
        # Create and start the autonomous car
        auto_car = AutonomousCar()
        auto_car.start(debug_mode=debug_mode)
        
        print("Autonomous driving started. Press Ctrl+C to stop.")
        
        # Keep the main thread alive
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"\nError: {str(e)}")
    finally:
        if 'auto_car' in locals():
            auto_car.cleanup()
        print("\nProgram terminated")