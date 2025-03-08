import RPi.GPIO as GPIO
import time

# GPIO pin setup
# Motor A (left side)
ENB = 12  # PWM pin for controlling speed
IN3 = 23  # Direction control
IN4 = 22  # Direction control

# Motor B (right side)
ENA = 13  # PWM pin for controlling speed
IN1 = 18  # Direction control
IN2 = 17  # Direction control

class PiCar:
    def __init__(self, default_speed=70, left_bias=1.0, right_bias=1.0):
        # Initialize with a reasonable default speed (70% duty cycle)
        self.speed = default_speed
        self.running = False
        
        # Calibration factors for motors
        self.left_bias = left_bias
        self.right_bias = right_bias
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup motor pins
        GPIO.setup(ENA, GPIO.OUT)
        GPIO.setup(IN1, GPIO.OUT)
        GPIO.setup(IN2, GPIO.OUT)
        GPIO.setup(ENB, GPIO.OUT)
        GPIO.setup(IN3, GPIO.OUT)
        GPIO.setup(IN4, GPIO.OUT)
        
        # Create PWM objects for speed control (100Hz frequency)
        self.pwm_left = GPIO.PWM(ENA, 100)
        self.pwm_right = GPIO.PWM(ENB, 100)
        
        # Start PWM with 0% duty cycle (stopped)
        self.pwm_left.start(0)
        self.pwm_right.start(0)
    
    def set_motor_directions(self, left_forward, right_forward):
        """Set the direction for both motors"""
        # Left motor
        GPIO.output(IN1, GPIO.HIGH if left_forward else GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW if left_forward else GPIO.HIGH)
        
        # Right motor
        GPIO.output(IN3, GPIO.HIGH if right_forward else GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW if right_forward else GPIO.HIGH)
    
    def set_motor_speeds(self, left_speed, right_speed):
        """Set speed for both motors with calibration applied"""
        # Apply calibration bias
        calibrated_left = left_speed * self.left_bias
        calibrated_right = right_speed * self.right_bias
        
        # Ensure speeds are within valid range (0-100)
        calibrated_left = max(0, min(100, calibrated_left))
        calibrated_right = max(0, min(100, calibrated_right))
        
        # Apply speeds
        self.pwm_left.ChangeDutyCycle(calibrated_left)
        self.pwm_right.ChangeDutyCycle(calibrated_right)
        
        # If any motor is running, set running state to True
        self.running = (left_speed > 0 or right_speed > 0)
    
    def forward(self, speed=None):
        """Move the car forward"""
        if speed is not None:
            self.speed = max(0, min(100, speed))
        
        # Set both motors in forward direction
        self.set_motor_directions(True, True)
        
        # Apply the same speed to both motors
        self.set_motor_speeds(self.speed, self.speed)
        
        return self  # Enable method chaining
    
    def backward(self, speed=None):
        """Move the car backward"""
        if speed is not None:
            self.speed = max(0, min(100, speed))
        
        # Set both motors in reverse direction
        self.set_motor_directions(False, False)
        
        # Apply the same speed to both motors
        self.set_motor_speeds(self.speed, self.speed)
        
        return self  # Enable method chaining
    
    def turn_left(self, turn_factor=0.8):
        """
        Turn the car left with enhanced turning
        turn_factor: 0.0 to 1.0 controls how sharp the turn is (higher = sharper)
        """
        # Get current direction flags
        current_direction_left = GPIO.input(IN1)
        current_direction_right = GPIO.input(IN3)
        
        # Calculate motor speeds for more aggressive turning
        left_speed = self.speed * (1 - turn_factor)  # Reduce left motor significantly
        right_speed = min(100, self.speed * 1.2)     # Boost right motor slightly
        
        # Maintain current direction while changing speeds
        self.set_motor_speeds(left_speed, right_speed)
        
        return self  # Enable method chaining
    
    def turn_right(self, turn_factor=0.8):
        """
        Turn the car right with enhanced turning
        turn_factor: 0.0 to 1.0 controls how sharp the turn is (higher = sharper)
        """
        # Get current direction flags
        current_direction_left = GPIO.input(IN1)
        current_direction_right = GPIO.input(IN3)
        
        # Calculate motor speeds for more aggressive turning
        left_speed = min(100, self.speed * 1.2)      # Boost left motor slightly
        right_speed = self.speed * (1 - turn_factor)  # Reduce right motor significantly
        
        # Maintain current direction while changing speeds
        self.set_motor_speeds(left_speed, right_speed)
        
        return self  # Enable method chaining
    
    def spin_left(self, spin_speed=None):
        """Spin the car to the left (counterclockwise) by reversing the left motor"""
        spin_speed = spin_speed if spin_speed is not None else self.speed
        
        # Set left motor backward, right motor forward
        self.set_motor_directions(False, True)
        
        # Apply the same speed to both motors
        self.set_motor_speeds(spin_speed, spin_speed)
        
        return self  # Enable method chaining
    
    def spin_right(self, spin_speed=None):
        """Spin the car to the right (clockwise) by reversing the right motor"""
        spin_speed = spin_speed if spin_speed is not None else self.speed
        
        # Set left motor forward, right motor backward
        self.set_motor_directions(True, False)
        
        # Apply the same speed to both motors
        self.set_motor_speeds(spin_speed, spin_speed)
        
        return self  # Enable method chaining
    
    def change_lane_left(self, turn_factor=0.8, duration=0.5):
        """Perform a lane change to the left"""
        # Store current direction
        left_forward = GPIO.input(IN1) == GPIO.HIGH
        right_forward = GPIO.input(IN3) == GPIO.HIGH
        
        # Apply left turn
        original_speed = self.speed
        self.turn_left(turn_factor)
        time.sleep(duration)
        
        # Restore original direction and speed
        self.set_motor_directions(left_forward, right_forward)
        self.set_motor_speeds(original_speed, original_speed)
        
        return self  # Enable method chaining
    
    def change_lane_right(self, turn_factor=0.8, duration=0.5):
        """Perform a lane change to the right"""
        # Store current direction
        left_forward = GPIO.input(IN1) == GPIO.HIGH
        right_forward = GPIO.input(IN3) == GPIO.HIGH
        
        # Apply right turn
        original_speed = self.speed
        self.turn_right(turn_factor)
        time.sleep(duration)
        
        # Restore original direction and speed
        self.set_motor_directions(left_forward, right_forward)
        self.set_motor_speeds(original_speed, original_speed)
        
        return self  # Enable method chaining
    
    def stop(self):
        """Stop both motors"""
        self.set_motor_speeds(0, 0)
        return self  # Enable method chaining
    
    def cleanup(self):
        """Release all resources"""
        self.stop()
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()
        print("GPIO pins cleaned up")
    
    def __del__(self):
        """Destructor to ensure cleanup when object is deleted"""
        try:
            if GPIO.getmode() is not None:  # Check if GPIO is still initialized
                self.stop()
                self.pwm_left.stop()
                self.pwm_right.stop()
                GPIO.cleanup()
        except:
            pass  # Ignore errors during cleanup in destructor


# Example usage
if __name__ == "__main__":
    # You can adjust these bias values if one motor is stronger than the other
    # Example: car = PiCar(default_speed=70, left_bias=1.1, right_bias=0.9)
    car = PiCar(default_speed=70)
    
    try:
        print("Testing PiCar functionality...")
        
        # print("Moving forward...")
        # car.forward()
        # time.sleep(2)
        
        print("Testing left turn...")
        car.forward().turn_left(0.8)  # Higher turn_factor for sharper turn
        time.sleep(2)
        
        # print("Testing right turn...")
        # car.forward().turn_right(0.8)  # Higher turn_factor for sharper turn
        # time.sleep(2)
        
        # print("Moving backward...")
        # car.backward()
        # time.sleep(2)
        
        # print("Testing left turn while moving backward...")
        # car.backward().turn_left(0.8)
        # time.sleep(2)
        
        # print("Testing right turn while moving backward...")
        # car.backward().turn_right(0.8)
        # time.sleep(2)
        
        print("Testing spin left...")
        car.spin_left()
        time.sleep(2)
        
        # print("Testing spin right...")
        # car.spin_right()
        # time.sleep(2)
        
        # print("Testing left lane change...")
        # car.forward().change_lane_left(0.9, 1.0)  # Sharper turn, longer duration
        # time.sleep(2)
        
        # print("Testing right lane change...")
        # car.forward().change_lane_right(0.9, 1.0)  # Sharper turn, longer duration
        # time.sleep(2)
        
        print("All tests completed!")
        
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
        
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        print("Cleaning up...")
        car.cleanup()
        print("Program ended")