import cv2
import numpy as np
import threading
import time
import os
from datetime import datetime

class LaneDetector:
    def __init__(self, camera_resolution=(320, 240), fps=30):
        self.camera = cv2.VideoCapture(0)  # Use Pi camera
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, camera_resolution[0])
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_resolution[1])
        self.camera.set(cv2.CAP_PROP_FPS, fps)
        
        # Detection parameters - adjust these based on your yellow tape color
        self.yellow_lower = np.array([15, 80, 80], dtype=np.uint8)  # Broader yellow range
        self.yellow_upper = np.array([45, 255, 255], dtype=np.uint8)
        
        # Lane state
        self.frame = None
        self.processed_frame = None
        self.lane_deviation = 0  # Negative: need to turn right, Positive: need to turn left
        self.is_running = False
        self.thread = None
        
        # Debug/display options
        self.show_camera_feed = False
        self.save_images = True
        self.image_save_interval = 1.0  # Save an image every second
        self.last_save_time = 0
        
        # Create output directory for saved images
        self.output_dir = "/tmp/lane_detection"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Smoothing for deviation values
        self.deviation_history = []
        self.history_size = 3  # Number of frames to average
    
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
        roi_height = int(height * 0.7)  # Increased ROI to see more of the road
        roi = frame[height - roi_height:height, 0:width]
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Create mask for yellow color
        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.erode(yellow_mask, kernel, iterations=1)
        yellow_mask = cv2.dilate(yellow_mask, kernel, iterations=2)  # More dilation to connect broken lines
        
        # Find contours
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Create visualizations for debugging
        roi_with_contours = roi.copy()
        cv2.drawContours(roi_with_contours, contours, -1, (0, 255, 0), 2)
        
        # Find lane markers
        left_boundary = []
        right_boundary = []
        center_line = []
        
        # Process contours to identify lane markers
        for contour in contours:
            if cv2.contourArea(contour) < 50:  # Reduced size threshold to detect smaller markers
                continue
                
            # Get the centroid of the contour
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
                
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Classify based on x-position - adjusted for better lane classification
            if cx < width * 0.4:  # Increased left boundary region
                left_boundary.append((cx, cy))
                cv2.circle(roi_with_contours, (cx, cy), 5, (255, 0, 0), -1)  # Blue for left
            elif cx > width * 0.6:  # Decreased right boundary region
                right_boundary.append((cx, cy))
                cv2.circle(roi_with_contours, (cx, cy), 5, (0, 0, 255), -1)  # Red for right
            else:
                center_line.append((cx, cy))
                cv2.circle(roi_with_contours, (cx, cy), 5, (0, 255, 255), -1)  # Yellow for center
        
        # Create a mask visualization (white = detected yellow)
        mask_colored = cv2.cvtColor(yellow_mask, cv2.COLOR_GRAY2BGR)
        
        # Calculate lane deviation
        self._calculate_deviation(left_boundary, right_boundary, center_line, width)
        
        # Add deviation text and markers to display
        cv2.putText(roi_with_contours, f"Deviation: {self.lane_deviation:.2f}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                   
        # Draw center line of the frame for reference
        cv2.line(roi_with_contours, (width//2, 0), (width//2, roi_height), (0, 255, 0), 1)
        
        # Update the processed frame
        self.processed_frame = frame.copy()
        self.processed_frame[height - roi_height:height, 0:width] = roi_with_contours
        
        # Save debug images periodically
        current_time = time.time()
        if self.save_images and (current_time - self.last_save_time >= self.image_save_interval):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            
            # Save the original frame
            cv2.imwrite(f"{self.output_dir}/original_{timestamp}.jpg", roi)
            
            # Save the mask
            cv2.imwrite(f"{self.output_dir}/mask_{timestamp}.jpg", yellow_mask)
            
            # Save the processed frame with contours
            cv2.imwrite(f"{self.output_dir}/processed_{timestamp}.jpg", roi_with_contours)
            
            # Log the detection results
            with open(f"{self.output_dir}/detection_log.txt", "a") as f:
                f.write(f"{timestamp}: Deviation={self.lane_deviation:.2f}, " +
                        f"Left={len(left_boundary)}, Right={len(right_boundary)}, " +
                        f"Center={len(center_line)}\n")
            
            self.last_save_time = current_time
    
    def _calculate_deviation(self, left_boundary, right_boundary, center_line, width):
        """Calculate lane deviation based on detected boundaries"""
        # Default is center
        frame_center = width / 2
        
        # Try to find the ideal center position with weighted priorities
        if left_boundary and right_boundary:
            # We have both boundaries, center between them (highest priority)
            left_avg_x = sum(x for x, _ in left_boundary) / len(left_boundary)
            right_avg_x = sum(x for x, _ in right_boundary) / len(right_boundary)
            ideal_center = (left_avg_x + right_avg_x) / 2
            
            # Log boundary positions for debugging
            with open(f"{self.output_dir}/boundaries.txt", "a") as f:
                f.write(f"Time={time.time()}: Left={left_avg_x}, Right={right_avg_x}, " +
                        f"Center={ideal_center}, Frame_Center={frame_center}\n")
                        
        elif center_line:
            # Use center line markers (second priority)
            center_avg_x = sum(x for x, _ in center_line) / len(center_line)
            ideal_center = center_avg_x
        elif left_boundary:
            # Only left boundary, estimate center (third priority)
            left_avg_x = sum(x for x, _ in left_boundary) / len(left_boundary)
            ideal_center = left_avg_x + (width * 0.35)  # Assume lane width is ~35% of frame
        elif right_boundary:
            # Only right boundary, estimate center (third priority)
            right_avg_x = sum(x for x, _ in right_boundary) / len(right_boundary)
            ideal_center = right_avg_x - (width * 0.35)  # Assume lane width is ~35% of frame
        else:
            # No markers detected, use frame center and log this case
            ideal_center = frame_center
            with open(f"{self.output_dir}/no_markers.txt", "a") as f:
                f.write(f"Time={time.time()}: No markers detected\n")
        
        # Calculate deviation as a value between -1 and 1
        # Negative means car needs to turn right, positive means turn left
        current_deviation = (ideal_center - frame_center) / (width / 2)
        
        # Apply smoothing with history
        self.deviation_history.append(current_deviation)
        if len(self.deviation_history) > self.history_size:
            self.deviation_history.pop(0)
        
        # Average the recent deviations for smoothing
        smoothed_deviation = sum(self.deviation_history) / len(self.deviation_history)
        
        # Apply a non-linear response curve to make small deviations more sensitive
        # This makes the car respond more aggressively to small deviations
        if abs(smoothed_deviation) < 0.3:
            # Amplify small deviations
            smoothed_deviation *= 1.5
        
        # Limit to range [-1, 1]
        self.lane_deviation = max(-1.0, min(1.0, smoothed_deviation))
    
    def get_deviation(self):
        """Return the current lane deviation value"""
        return self.lane_deviation

    def toggle_display(self):
        """Toggle image saving since we're using SSH and can't display windows"""
        self.save_images = not self.save_images
        message = "Image saving enabled" if self.save_images else "Image saving disabled"
        with open(f"{self.output_dir}/status.txt", "a") as f:
            f.write(f"{datetime.now()}: {message}\n")
        return message

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