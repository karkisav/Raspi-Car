import cv2
import numpy as np
import threading
import time

class LaneDetector:
    def __init__(self, camera_resolution=(320, 240), fps=30):
        self.camera = cv2.VideoCapture(0)  # Use Pi camera
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, camera_resolution[0])
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_resolution[1])
        self.camera.set(cv2.CAP_PROP_FPS, fps)
        
        # Detection parameters
        self.yellow_lower = np.array([20, 100, 100], dtype=np.uint8)
        self.yellow_upper = np.array([40, 255, 255], dtype=np.uint8)
        
        # Lane state
        self.frame = None
        self.processed_frame = None
        self.lane_deviation = 0  # Negative: need to turn right, Positive: need to turn left
        self.is_running = False
        self.thread = None
        
        # Debug/display options
        self.show_camera_feed = False
    
    def start(self):
        """Start the lane detection thread"""
        self.is_running = True
        self.thread = threading.Thread(target=self._detection_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def stop(self):
        """Stop the lane detection thread"""
        self.is_running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        self.camera.release()
        cv2.destroyAllWindows()
    
    def _detection_loop(self):
        """Main lane detection loop running in a separate thread"""
        while self.is_running:
            ret, self.frame = self.camera.read()
            if not ret:
                continue
            
            # Process the frame to detect lanes
            self._process_frame()
            
            # Display camera feed if enabled
            if self.show_camera_feed and self.processed_frame is not None:
                cv2.imshow("Lane Detection", self.processed_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            time.sleep(0.01)  # Small delay to prevent CPU hogging
    
    def _process_frame(self):
        """Process the current frame to detect lane markings"""
        if self.frame is None:
            return
        
        # Create a copy for processing
        frame = self.frame.copy()
        height, width = frame.shape[:2]
        
        # Define region of interest (lower portion of the frame)
        roi_height = int(height * 0.6)
        roi = frame[height - roi_height:height, 0:width]
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Create mask for yellow color
        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.erode(yellow_mask, kernel, iterations=1)
        yellow_mask = cv2.dilate(yellow_mask, kernel, iterations=1)
        
        # Find contours
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw contours on original image for visualization
        roi_with_contours = roi.copy()
        cv2.drawContours(roi_with_contours, contours, -1, (0, 255, 0), 2)
        
        # Find lane markers
        left_boundary = []
        right_boundary = []
        center_line = []
        
        # Process contours to identify lane markers
        for contour in contours:
            if cv2.contourArea(contour) < 100:  # Filter out small contours
                continue
                
            # Get the centroid of the contour
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
                
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Classify based on x-position
            if cx < width * 0.3:
                left_boundary.append((cx, cy))
                cv2.circle(roi_with_contours, (cx, cy), 5, (255, 0, 0), -1)  # Blue for left
            elif cx > width * 0.7:
                right_boundary.append((cx, cy))
                cv2.circle(roi_with_contours, (cx, cy), 5, (0, 0, 255), -1)  # Red for right
            else:
                center_line.append((cx, cy))
                cv2.circle(roi_with_contours, (cx, cy), 5, (0, 255, 255), -1)  # Yellow for center
        
        # Calculate lane deviation
        self._calculate_deviation(left_boundary, right_boundary, center_line, width)
        
        # Add deviation text to display
        cv2.putText(roi_with_contours, f"Deviation: {self.lane_deviation:.2f}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Update the processed frame
        self.processed_frame = frame.copy()
        self.processed_frame[height - roi_height:height, 0:width] = roi_with_contours
    
    def _calculate_deviation(self, left_boundary, right_boundary, center_line, width):
        """Calculate lane deviation based on detected boundaries"""
        # Default is center
        frame_center = width / 2
        
        # Try to find the ideal center position
        if left_boundary and right_boundary:
            # We have both boundaries, center between them
            left_avg_x = sum(x for x, _ in left_boundary) / len(left_boundary)
            right_avg_x = sum(x for x, _ in right_boundary) / len(right_boundary)
            ideal_center = (left_avg_x + right_avg_x) / 2
        elif center_line:
            # Use center line markers
            center_avg_x = sum(x for x, _ in center_line) / len(center_line)
            ideal_center = center_avg_x
        elif left_boundary:
            # Only left boundary, estimate center
            left_avg_x = sum(x for x, _ in left_boundary) / len(left_boundary)
            ideal_center = left_avg_x + (width * 0.4)  # Assume lane width is ~40% of frame
        elif right_boundary:
            # Only right boundary, estimate center
            right_avg_x = sum(x for x, _ in right_boundary) / len(right_boundary)
            ideal_center = right_avg_x - (width * 0.4)  # Assume lane width is ~40% of frame
        else:
            # No markers detected, use frame center
            ideal_center = frame_center
        
        # Calculate deviation as a value between -1 and 1
        # Negative means car needs to turn right, positive means turn left
        self.lane_deviation = (ideal_center - frame_center) / (width / 2)
        # Limit to range [-1, 1]
        self.lane_deviation = max(-1.0, min(1.0, self.lane_deviation))
    
    def get_deviation(self):
        """Return the current lane deviation value"""
        return self.lane_deviation

    def toggle_display(self):
        """Toggle the camera feed display"""
        self.show_camera_feed = not self.show_camera_feed
        if not self.show_camera_feed:
            cv2.destroyAllWindows()

# For testing
if __name__ == "__main__":
    detector = LaneDetector()
    detector.show_camera_feed = True
    detector.start()
    
    try:
        while True:
            deviation = detector.get_deviation()
            print(f"Lane deviation: {deviation:.2f}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        detector.stop()