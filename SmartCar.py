import cv2
import numpy as np
import threading
import time
from car_control import Car  # Importing our existing Car class

class SmartCar:
    def __init__(self):
        # Initialize the car controller
        self.car = Car()
        self.car.start_control_thread()
        
        # Initialize camera
        self.camera = cv2.VideoCapture(0)
        self.camera.set(3, 320)  # Width
        self.camera.set(4, 240)  # Height
        
        # Control flags
        self.is_running = False
        self.current_lane = 1  # 1 for left lane, 2 for right lane
        self.current_speed = 40  # Default speed
        
    def start(self):
        """Start the vision and control loops"""
        self.is_running = True
        self.vision_thread = threading.Thread(target=self.vision_loop)
        self.vision_thread.daemon = True
        self.vision_thread.start()
        print("Smart car started")
        
    def stop(self):
        """Stop the car and release resources"""
        self.is_running = False
        self.car.stop()
        time.sleep(0.5)
        self.car.cleanup()
        self.camera.release()
        cv2.destroyAllWindows()
        print("Smart car stopped")
        
    def vision_loop(self):
        """Main loop for processing camera frames and controlling the car"""
        while self.is_running:
            ret, frame = self.camera.read()
            if not ret:
                print("Failed to capture image")
                continue
                
            # Process image for different detections
            yellow_edges = self.detect_yellow_edges(frame)
            traffic_signals = self.detect_traffic_signals(frame)
            car_detected = self.detect_car(frame)
            
            # Make driving decisions
            self.make_driving_decisions(yellow_edges, traffic_signals, car_detected)
            
            # Optionally display the processed image
            cv2.imshow('Car Vision', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop()
                break
    
    def detect_yellow_edges(self, frame):
        """Detect yellow edges (track boundaries)"""
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define yellow color range in HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        
        # Create mask for yellow color
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.erode(yellow_mask, kernel, iterations=1)
        yellow_mask = cv2.dilate(yellow_mask, kernel, iterations=1)
        
        # Find contours
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw contours on the original image for visualization
        cv2.drawContours(frame, contours, -1, (0, 255, 255), 2)
        
        return contours
    
    def detect_traffic_signals(self, frame):
        """Detect traffic signals (red, yellow, green)"""
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define color ranges for traffic signals
        lower_red = np.array([0, 70, 50])
        upper_red = np.array([10, 255, 255])
        
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        
        # Create masks for each color
        red_mask = cv2.inRange(hsv, lower_red, upper_red)
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Check if significant areas of each color are detected
        red_area = cv2.countNonZero(red_mask)
        yellow_area = cv2.countNonZero(yellow_mask)
        green_area = cv2.countNonZero(green_mask)
        
        signal = None
        if red_area > 500:  # Adjust threshold as needed
            signal = "red"
            cv2.putText(frame, "RED SIGNAL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        elif yellow_area > 500:
            signal = "yellow"
            cv2.putText(frame, "YELLOW SIGNAL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        elif green_area > 500:
            signal = "green"
            cv2.putText(frame, "GREEN SIGNAL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
        # Additional check for green with arrow direction
        if signal == "green":
            # This would require more sophisticated shape detection for arrows
            # For simplicity, we're just checking where the green area is located
            h, w = green_mask.shape
            left_area = cv2.countNonZero(green_mask[:, :w//2])
            right_area = cv2.countNonZero(green_mask[:, w//2:])
            
            if left_area > right_area * 1.5:
                signal = "green_left"
                cv2.putText(frame, "LEFT ARROW", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            elif right_area > left_area * 1.5:
                signal = "green_right"
                cv2.putText(frame, "RIGHT ARROW", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        return signal
    
    def detect_car(self, frame):
        """Detect other cars on the track"""
        # For a simple version, we can look for a specific color or shape
        # This could be improved with actual object detection models
        
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Assuming cars might be blue for visibility
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
        
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Check if there's a significant blue area (potential car)
        blue_area = cv2.countNonZero(blue_mask)
        
        car_detected = blue_area > 500  # Adjust threshold as needed
        
        if car_detected:
            cv2.putText(frame, "CAR DETECTED", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
            
        return car_detected
    
    def make_driving_decisions(self, yellow_edges, traffic_signal, car_detected):
        """Make driving decisions based on detected features"""
        # Process traffic signals
        if traffic_signal == "red":
            # Stop the car
            self.car.stop()
            print("Traffic signal: RED - Stopping")
            return
        
        elif traffic_signal == "yellow":
            # Slow down
            self.current_speed = 20
            print("Traffic signal: YELLOW - Slowing down")
        
        elif traffic_signal == "green":
            # Normal speed
            self.current_speed = 40
            print("Traffic signal: GREEN - Normal speed")
        
        elif traffic_signal == "green_left":
            # Turn left
            self.car.left(speed=30, duration=0.5)
            print("Traffic signal: GREEN LEFT - Turning left")
            return
        
        elif traffic_signal == "green_right":
            # Turn right
            self.car.right(speed=30, duration=0.5)
            print("Traffic signal: GREEN RIGHT - Turning right")
            return
        
        # Check for other cars
        if car_detected:
            # Change lane
            if self.current_lane == 1:
                self.current_lane = 2
                self.car.right(speed=30, duration=0.5)
                print("Car detected - Changing to right lane")
            else:
                self.current_lane = 1
                self.car.left(speed=30, duration=0.5)
                print("Car detected - Changing to left lane")
            return
        
        # Default: follow the track
        self.car.forward(speed=self.current_speed)

def main():
    # Create and start the smart car
    smart_car = SmartCar()
    
    try:
        smart_car.start()
        
        # Keep the program running
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        smart_car.stop()
        print("Program terminated")

if __name__ == "__main__":
    main()