from car_control import Car
import time

class AutonomousController:
    def __init__(self, car):
        """
        Initialize the controller with a Car object
        """
        self.car = car
        
        # Configure PID-like control parameters
        self.speed = 20      # Base forward speed
        self.turn_speed = 15  # Base turning speed
        
        # Deviation thresholds for control decisions
        self.center_threshold = 30    # Pixels of deviation considered "centered"
        self.turn_threshold = 100     # Pixels of deviation for full turn speed
        
        # Control state
        self.is_active = False
        self.last_control_time = time.time()
        self.control_interval = 0.05  # 50ms control interval
        
        # Safety timeout - stop if no lane detected for this long
        self.safety_timeout = 1.0     # seconds
        self.last_lane_detection_time = time.time()
        
        # For smooth control
        self.last_deviation = 0
        self.smoothing_factor = 0.3  # 0.0 to 1.0, higher = more smoothing
    
    def start(self):
        """Start autonomous mode"""
        self.is_active = True
        self.car.start_control_thread()
        print("Autonomous mode activated")
    
    def stop(self):
        """Stop autonomous mode"""
        self.is_active = False
        self.car.stop()
        print("Autonomous mode deactivated")
    
    def cleanup(self):
        """Clean up resources"""
        self.stop()
        self.car.cleanup()
    
    def process_lane_info(self, lane_info):
        """
        Process lane detection information and control the car
        
        Args:
            lane_info: Dictionary with lane detection results
                {
                    'detected': bool,
                    'center_x': int,
                    'deviation': int
                }
        """
        if not self.is_active:
            return
        
        # Check if we have a lane detection
        if lane_info['detected']:
            self.last_lane_detection_time = time.time()
            
            # Apply smoothing to deviation
            smoothed_deviation = (self.smoothing_factor * self.last_deviation + 
                                 (1 - self.smoothing_factor) * lane_info['deviation'])
            self.last_deviation = smoothed_deviation
            
            # Current time for control intervals
            current_time = time.time()
            
            # Only adjust controls at the control interval
            if current_time - self.last_control_time >= self.control_interval:
                self.last_control_time = current_time
                
                # Determine if we need to turn and by how much
                if abs(smoothed_deviation) < self.center_threshold:
                    # Go straight
                    self.car.forward(speed=self.speed)
                    print("STRAIGHT")
                else:
                    # Calculate turn intensity (0.0 to 1.0)
                    turn_intensity = min(1.0, abs(smoothed_deviation) / self.turn_threshold)
                    
                    # Calculate actual turn speed based on intensity
                    actual_turn_speed = int(self.turn_speed * turn_intensity)
                    
                    # Ensure minimum turn speed
                    actual_turn_speed = max(actual_turn_speed, 10)
                    
                    # Turn left or right based on deviation
                    if smoothed_deviation > 0:  # Deviation to the right, turn left
                        self.car.forward_left(speed=self.speed, turn_speed=actual_turn_speed)
                        print(f"LEFT (intensity: {turn_intensity:.2f}, speed: {actual_turn_speed})")
                    else:  # Deviation to the left, turn right
                        self.car.forward_right(speed=self.speed, turn_speed=actual_turn_speed)
                        print(f"RIGHT (intensity: {turn_intensity:.2f}, speed: {actual_turn_speed})")
        else:
            # Check safety timeout - stop if no lane detected for a while
            if time.time() - self.last_lane_detection_time > self.safety_timeout:
                self.car.stop()
                print("STOP - Lane lost")
    
    def set_speeds(self, forward_speed, turning_speed):
        """Set the base speeds for forward motion and turning"""
        self.speed = max(10, min(100, forward_speed))
        self.turn_speed = max(10, min(65, turning_speed))
        print(f"Speeds updated - Forward: {self.speed}, Turn: {self.turn_speed}")
    
    def set_thresholds(self, center_threshold, turn_threshold):
        """Set thresholds for control decisions"""
        self.center_threshold = max(5, center_threshold)
        self.turn_threshold = max(self.center_threshold + 10, turn_threshold)
        print(f"Thresholds updated - Center: {self.center_threshold}, Turn: {self.turn_threshold}")
        
    def update_smoothing(self, smoothing_factor):
        """Update smoothing factor for control"""
        self.smoothing_factor = max(0.0, min(0.9, smoothing_factor))
        print(f"Smoothing factor updated: {self.smoothing_factor}")