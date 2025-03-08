#!/usr/bin/env python3
import time
import sys
import threading

# Flag to indicate if we're running on a Raspberry Pi
try:
    import RPi.GPIO as GPIO
    ON_RASPBERRY_PI = True
except ImportError:
    ON_RASPBERRY_PI = False
    # Create mock GPIO for testing on non-Raspberry Pi systems
    class MockGPIO:
        BCM = 'BCM'
        OUT = 'OUT'
        LOW = 'LOW'
        HIGH = 'HIGH'
        
        def __init__(self):
            self.pins = {}
            print("MOCK GPIO: Initialized")
            
        def setmode(self, mode):
            print(f"MOCK GPIO: Set mode to {mode}")
            
        def setwarnings(self, flag):
            print(f"MOCK GPIO: Set warnings to {flag}")
            
        def setup(self, pin, mode):
            self.pins[pin] = {'mode': mode, 'value': self.LOW}
            print(f"MOCK GPIO: Setup pin {pin} as {mode}")
            
        def output(self, pin, value):
            if pin in self.pins:
                self.pins[pin]['value'] = value
                print(f"MOCK GPIO: Set pin {pin} to {value}")
            else:
                print(f"MOCK GPIO: Error - pin {pin} not set up")
                
        def cleanup(self):
            print("MOCK GPIO: Cleanup")
            
        class PWM:
            def __init__(self, pin, freq):
                self.pin = pin
                self.freq = freq
                self.duty_cycle = 0
                print(f"MOCK GPIO: PWM initialized on pin {pin} at {freq}Hz")
                
            def start(self, duty_cycle):
                self.duty_cycle = duty_cycle
                print(f"MOCK GPIO: PWM started on pin {self.pin} with duty cycle {duty_cycle}%")
                
            def ChangeDutyCycle(self, duty_cycle):
                self.duty_cycle = duty_cycle
                print(f"MOCK GPIO: PWM changed to {duty_cycle}% on pin {self.pin}")
                
            def stop(self):
                print(f"MOCK GPIO: PWM stopped on pin {self.pin}")
    
    GPIO = MockGPIO()
    print("Running with Mock GPIO for testing (not on a Raspberry Pi)")

class Motor:
    def __init__(self, forward_pin, backward_pin, pwm_freq=100):
        self.forward_pin = forward_pin
        self.backward_pin = backward_pin
        self.pwm_freq = pwm_freq
        self.current_speed = 0
        self.target_speed = 0
        
        # Setup the GPIO pins
        GPIO.setup(forward_pin, GPIO.OUT)
        GPIO.setup(backward_pin, GPIO.OUT)
        
        # Create PWM objects
        self.pwm_forward = GPIO.PWM(forward_pin, pwm_freq)
        self.pwm_backward = GPIO.PWM(backward_pin, pwm_freq)
        
        # Start PWM with 0% duty cycle
        self.pwm_forward.start(0)
        self.pwm_backward.start(0)
    
    def set_speed(self, speed):
        """
        Set the motor speed directly (-100 to 100)
        Positive values: forward, Negative values: backward
        """
        self.current_speed = max(min(speed, 100), -100)
        
        if self.current_speed > 0:
            self.pwm_forward.ChangeDutyCycle(self.current_speed)
            self.pwm_backward.ChangeDutyCycle(0)
        else:
            self.pwm_forward.ChangeDutyCycle(0)
            self.pwm_backward.ChangeDutyCycle(abs(self.current_speed))
    
    def set_target_speed(self, speed):
        """Set the target speed for gradual acceleration/deceleration"""
        self.target_speed = max(min(speed, 100), -100)
    
    def stop(self):
        """Stop the motor"""
        self.set_speed(0)
        self.target_speed = 0
    
    def cleanup(self):
        """Cleanup resources"""
        self.stop()
        self.pwm_forward.stop()
        self.pwm_backward.stop()


class Car:
    def __init__(self, left_forward_pin=17, left_backward_pin=18, 
                 right_forward_pin=22, right_backward_pin=23):
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Create motors
        self.left_motor = Motor(left_forward_pin, left_backward_pin)
        self.right_motor = Motor(right_forward_pin, right_backward_pin)
        
        # Control parameters
        self.acceleration_rate = 2
        self.deceleration_rate = 3
        self.running = False
        self.default_speed = 50
        self.default_duration = 2  # seconds
        
        print("Car initialized and ready")
    
    def start_control_thread(self):
        """Start the speed control thread for smooth acceleration"""
        self.running = True
        self.control_thread = threading.Thread(target=self._speed_control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        print("Speed control thread started")
    
    def _speed_control_loop(self):
        """Thread function for handling gradual speed changes"""
        last_update_time = time.time()
        
        while self.running:
            now = time.time()
            dt = now - last_update_time
            last_update_time = now
            
            # Calculate adjustment based on time
            rate_factor = dt * 50
            accel = self.acceleration_rate * rate_factor
            decel = self.deceleration_rate * rate_factor
            
            # Left motor adjustment
            if self.left_motor.current_speed < self.left_motor.target_speed:
                self.left_motor.set_speed(min(self.left_motor.current_speed + accel, 
                                             self.left_motor.target_speed))
            elif self.left_motor.current_speed > self.left_motor.target_speed:
                self.left_motor.set_speed(max(self.left_motor.current_speed - decel, 
                                             self.left_motor.target_speed))
            
            # Right motor adjustment
            if self.right_motor.current_speed < self.right_motor.target_speed:
                self.right_motor.set_speed(min(self.right_motor.current_speed + accel, 
                                              self.right_motor.target_speed))
            elif self.right_motor.current_speed > self.right_motor.target_speed:
                self.right_motor.set_speed(max(self.right_motor.current_speed - decel, 
                                              self.right_motor.target_speed))
            
            time.sleep(0.01)
    
    def forward(self, speed=None, duration=None):
        """Move the car forward"""
        speed = speed if speed is not None else self.default_speed
        self.left_motor.set_target_speed(speed)
        self.right_motor.set_target_speed(speed)
        print(f"Moving forward at speed {speed}")
        
        if duration:
            time.sleep(duration)
            self.stop()
    
    def backward(self, speed=None, duration=None):
        """Move the car backward"""
        speed = speed if speed is not None else self.default_speed
        self.left_motor.set_target_speed(-speed)
        self.right_motor.set_target_speed(-speed)
        print(f"Moving backward at speed {speed}")
        
        if duration:
            time.sleep(duration)
            self.stop()
    
    def left(self, speed=None, duration=None):
        """Turn the car left"""
        speed = speed if speed is not None else self.default_speed
        # To turn left, right motor goes forward, left motor stays or goes backward
        if self.left_motor.target_speed >= 0 and self.right_motor.target_speed >= 0:
            # Currently moving forward or stopped
            self.left_motor.set_target_speed(0)
            self.right_motor.set_target_speed(speed)
        else:
            # Currently moving backward
            self.left_motor.set_target_speed(-speed)
            self.right_motor.set_target_speed(0)
        
        print("Turning left")
        
        if duration:
            time.sleep(duration)
            self.stop()
    
    def right(self, speed=None, duration=None):
        """Turn the car right"""
        speed = speed if speed is not None else self.default_speed
        # To turn right, left motor goes forward, right motor stays or goes backward
        if self.left_motor.target_speed >= 0 and self.right_motor.target_speed >= 0:
            # Currently moving forward or stopped
            self.left_motor.set_target_speed(speed)
            self.right_motor.set_target_speed(0)
        else:
            # Currently moving backward
            self.left_motor.set_target_speed(0)
            self.right_motor.set_target_speed(-speed)
        
        print("Turning right")
        
        if duration:
            time.sleep(duration)
            self.stop()
    
    def stop(self):
        """Stop the car"""
        self.left_motor.set_target_speed(0)
        self.right_motor.set_target_speed(0)
        print("Stopping")
    
    def execute_commands(self, commands):
        """Execute a list of commands with optional durations"""
        print(f"Executing {len(commands)} command(s)")
        for cmd in commands:
            command = cmd.get('command', '').lower()
            speed = cmd.get('speed', self.default_speed)
            duration = cmd.get('duration', self.default_duration)
            
            if command == 'forward':
                self.forward(speed)
            elif command == 'backward':
                self.backward(speed)
            elif command == 'left':
                self.left(speed)
            elif command == 'right':
                self.right(speed)
            elif command == 'stop':
                self.stop()
            else:
                print(f"Unknown command: {command}")
                continue
            
            # Wait for specified duration
            time.sleep(duration)
        
        # Stop after all commands are executed
        self.stop()
        print("All commands executed")
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        time.sleep(0.1)  # Give the control thread time to stop
        self.left_motor.cleanup()
        self.right_motor.cleanup()
        GPIO.cleanup()
        print("Car resources released")


def run_test_sequence():
    """Run a simple test sequence"""
    car = Car()
    
    try:
        # Start speed control thread
        car.start_control_thread()
        
        # Example of manual commands
        print("\nRunning manual command sequence:")
        # car.forward(speed=100, duration=2)
        # car.left(duration=0.5)
        # car.forward(duration=1)
        # car.right(duration=1)
        # car.backward(speed=30, duration=2)
        # car.stop()
        time.sleep(1)
        
        # Example of command list
        print("\nRunning command list:")
        commands = [
            {'command': 'forward', 'speed': 100, 'duration': 1},
            {'command': 'left', 'duration': 1},
            # {'command': 'forward', 'speed': 70, 'duration': 1.5},
            {'command': 'right', 'duration': 1},
            # {'command': 'backward', 'speed': 40, 'duration': 2},
            {'command': 'stop', 'duration': 1}
        ]
        car.execute_commands(commands)
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\nError during test: {str(e)}")
    finally:
        car.cleanup()
        print("\nTest completed")


if __name__ == "__main__":
    print("Raspberry Pi Car Control - Simplified Class Version")
    
    if len(sys.argv) > 1 and sys.argv[1] == "--demo":
        print("Running demo sequence...")
        run_test_sequence()
    else:
        print("Creating car instance for you to control...")
        print("Example usage:")
        print("  car = Car()")
        print("  car.start_control_thread()")
        print("  car.forward(speed=50, duration=2)")
        print("  car.left(duration=1)")
        print("  car.stop()")
        print("  car.cleanup()")
        
        # You can add your own command sequence here or
        # use this code as a module and import it elsewhere