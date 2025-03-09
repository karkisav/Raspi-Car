import cv2
import numpy as np

class LaneDetector:
    def __init__(self):
        # Yellow color range in HSV
        # These values may need adjustment based on your specific yellow lanes and lighting conditions
        self.yellow_lower = np.array([20, 100, 100], dtype=np.uint8)
        self.yellow_upper = np.array([40, 255, 255], dtype=np.uint8)
        
        # Region of Interest (ROI) parameters - bottom half of the frame by default
        self.roi_height = 0.5  # Use bottom 50% of the frame

    def detect_lanes(self, frame):
        """
        Detect yellow lane markings in the given frame
        Returns: processed frame with annotations, lane position info, and binary mask
        """
        height, width = frame.shape[:2]
        roi_y = int(height * (1 - self.roi_height))
        
        # Create a working copy of the frame
        result_frame = frame.copy()
        
        # Apply ROI (bottom portion of the frame)
        roi = frame[roi_y:height, 0:width]
        
        # Convert ROI to HSV color space for better color filtering
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Create a mask for yellow color
        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw ROI boundary on the result frame
        cv2.line(result_frame, (0, roi_y), (width, roi_y), (0, 255, 0), 2)
        
        # Initialize lane position variables
        lane_center_x = width // 2  # Default to center
        lane_detected = False
        
        # If contours found, process them
        if contours:
            # Find the largest contour (assumed to be the lane)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Only proceed if the contour is large enough (to filter noise)
            if cv2.contourArea(largest_contour) > 200:
                lane_detected = True
                
                # Get the moments to find centroid
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"]) + roi_y  # Adjust for ROI
                    lane_center_x = cx
                    
                    # Draw centroid
                    cv2.circle(result_frame, (cx, cy), 10, (0, 0, 255), -1)
                
                # Draw contour on the result frame
                cv2.drawContours(result_frame, [largest_contour], -1, (255, 0, 0), 2, offset=(0, roi_y))
        
        # Create full-size binary mask for visualization and debugging
        full_mask = np.zeros((height, width), dtype=np.uint8)
        full_mask[roi_y:height, 0:width] = yellow_mask
        
        # Draw a line from bottom center to the detected lane center
        bottom_center = (width // 2, height)
        lane_point = (lane_center_x, roi_y + (height - roi_y) // 2)
        cv2.line(result_frame, bottom_center, lane_point, (255, 255, 0), 2)
        
        # Calculate deviation from center
        deviation = lane_center_x - (width // 2)
        
        # Add text information
        cv2.putText(result_frame, f"Lane Detected: {lane_detected}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(result_frame, f"Deviation: {deviation}px", (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return result_frame, {
            'detected': lane_detected,
            'center_x': lane_center_x,
            'deviation': deviation
        }, full_mask
    
    def adjust_yellow_threshold(self, hue_low, hue_high, sat_low, sat_high, val_low, val_high):
        """Adjust yellow threshold parameters for calibration"""
        self.yellow_lower = np.array([hue_low, sat_low, val_low], dtype=np.uint8)
        self.yellow_upper = np.array([hue_high, sat_high, val_high], dtype=np.uint8)
    
    def set_roi_height(self, roi_height_percentage):
        """Set the ROI height percentage (0.0 to 1.0)"""
        self.roi_height = max(0.1, min(1.0, roi_height_percentage))