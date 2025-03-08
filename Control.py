import RPi.GPIO as GPIO
import time

# GPIO pin setup
# Motor A (left side)
ENA = 12  # PWM pin for controlling speed
IN1 = 23  # Direction control
IN2 = 22  # Direction control

# Motor B (right side)
ENB = 13  # PWM pin for controlling speed
IN3 = 18  # Direction control
IN4 = 17  # Direction control

class PiCar:
    def __init__(self, default_speed=70):
        # Initialize with a reasonable default speed (70% duty cycle)
        self.speed = default_speed
        self.running = False
        
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
        """Set speed for both motors separately"""
        # Ensure speeds are within valid range (0-100)
        left_speed = max(0, min(100, left_speed))
        right_speed = max(0, min(100, right_speed))
        
        # Apply speeds
        self.pwm_left.ChangeDutyCycle(left_speed)
        self.pwm_right.ChangeDutyCycle(right_speed)
        
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
    
    def turn_left(self, turn_factor=0.5):
        """
        Turn the car left by reducing the left motor speed
        turn_factor: 0.0 to 1.0, where 1.0 means the left wheel stops completely
        """
        # Calculate the left motor speed reduction
        left_speed = self.speed * (1 - turn_factor)
        
        # Apply the speeds (direction pins remain unchanged)
        self.set_motor_speeds(left_speed, self.speed)
        
        return self  # Enable method chaining
    
    def turn_right(self, turn_factor=0.5):
        """
        Turn the car right by reducing the right motor speed
        turn_factor: 0.0 to 1.0, where 1.0 means the right wheel stops completely
        """
        # Calculate the right motor speed reduction
        right_speed = self.speed * (1 - turn_factor)
        
        # Apply the speeds (direction pins remain unchanged)
        self.set_motor_speeds(self.speed, right_speed)
        
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
    
    def change_lane_left(self, turn_factor=0.7, duration=0.5):
        """Perform a lane change to the left"""
        # Remember the current direction
        left_forward = GPIO.input(IN1) == GPIO.HIGH
        right_forward = GPIO.input(IN3) == GPIO.HIGH
        
        # Quick left turn
        self.turn_left(turn_factor)
        time.sleep(duration)
        
        # Return to original direction and speed
        self.set_motor_directions(left_forward, right_forward)
        self.set_motor_speeds(self.speed, self.speed)
        
        return self  # Enable method chaining
    
    def change_lane_right(self, turn_factor=0.7, duration=0.5):
        """Perform a lane change to the right"""
        # Remember the current direction
        left_forward = GPIO.input(IN1) == GPIO.HIGH
        right_forward = GPIO.input(IN3) == GPIO.HIGH
        
        # Quick right turn
        self.turn_right(turn_factor)
        time.sleep(duration)
        
        # Return to original direction and speed
        self.set_motor_directions(left_forward, right_forward)
        self.set_motor_speeds(self.speed, self.speed)
        
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
    car = PiCar(default_speed=10)  # Create car with 70% speed
    
    try:
        print("Testing PiCar functionality...")
        
        print("Moving forward...")
        car.forward()
        time.sleep(2)
        
        print("Moving backward...")
        car.backward()
        time.sleep(2)
        
        # print("Turning left...")
        # car.forward().turn_left(0.6)  # Demonstrating method chaining
        # time.sleep(2)
        
        # print("Turning right...")
        # car.forward().turn_right(0.6)
        # time.sleep(2)
        
        print("Spinning left...")
        car.spin_left()
        time.sleep(2)
        
        print("Spinning right...")
        car.spin_right()
        time.sleep(2)
        
        # print("Changing to left lane...")
        # car.forward().change_lane_left()
        # time.sleep(1)
        
        # print("Changing to right lane...")
        # car.forward().change_lane_right()
        # time.sleep(1)
        
        print("All tests completed!")
        
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
        
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        print("Cleaning up...")
        car.cleanup()
        print("Program ended")