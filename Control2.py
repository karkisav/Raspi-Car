#!/usr/bin/env python3
"""
Raspberry Pi Car Control Script
Controls a car using PWM signals to motor controllers.
Use W/S for forward/backward, A/D for left/right turns.
"""
import curses
import time
import threading
import sys
import os
import traceback

# Logging setup
LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "car_control_debug.log")
def log(message):
    """Write a timestamped message to the log file."""
    with open(LOG_FILE, "a") as f:
        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {message}\n")

# Determine if running on a Raspberry Pi and set up GPIO accordingly
try:
    import RPi.GPIO as GPIO
    ON_RASPBERRY_PI = True
    log("Running on Raspberry Pi with real GPIO")
except ImportError:
    ON_RASPBERRY_PI = False
    log("Not running on Raspberry Pi, using mock GPIO")
    
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
                print(f"MOCK GPIO: PWM duty cycle changed to {duty_cycle}%")
                
            def stop(self):
                print(f"MOCK GPIO: PWM stopped on pin {self.pin}")
    
    GPIO = MockGPIO()
    print("Running with Mock GPIO for testing (not on a Raspberry Pi)")

# Configuration
class Config:
    # GPIO pin setup
    MOTOR_LEFT_FORWARD = 17
    MOTOR_LEFT_BACKWARD = 18
    MOTOR_RIGHT_FORWARD = 22
    MOTOR_RIGHT_BACKWARD = 23
    PWM_FREQ = 100
    
    # Control parameters
    MAX_SPEED = 100
    ACCELERATION_RATE = 2
    DECELERATION_RATE = 3
    CONTROL_LOOP_DELAY = 0.01

class CarController:
    def __init__(self):
        log("Initializing CarController")
        
        # Motor control variables
        self.target_left_speed = 0
        self.target_right_speed = 0
        self.current_left_speed = 0
        self.current_right_speed = 0
        self.running = True
        
        # Initialize GPIO and PWM
        try:
            self._setup_gpio()
        except Exception as e:
            log(f"GPIO setup failed: {str(e)}")
            log(traceback.format_exc())
            raise
            
    def _setup_gpio(self):
        """Set up GPIO pins and PWM for motor control."""
        log("Setting up GPIO")
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Initialize motor pins
        motor_pins = [
            Config.MOTOR_LEFT_FORWARD, 
            Config.MOTOR_LEFT_BACKWARD, 
            Config.MOTOR_RIGHT_FORWARD, 
            Config.MOTOR_RIGHT_BACKWARD
        ]
        
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        log("Setting up PWM")
        # Set up PWM for each motor
        self.pwm_left_forward = GPIO.PWM(Config.MOTOR_LEFT_FORWARD, Config.PWM_FREQ)
        self.pwm_left_backward = GPIO.PWM(Config.MOTOR_LEFT_BACKWARD, Config.PWM_FREQ)
        self.pwm_right_forward = GPIO.PWM(Config.MOTOR_RIGHT_FORWARD, Config.PWM_FREQ)
        self.pwm_right_backward = GPIO.PWM(Config.MOTOR_RIGHT_BACKWARD, Config.PWM_FREQ)

        # Start PWM with 0% duty cycle
        self.pwm_left_forward.start(0)
        self.pwm_left_backward.start(0)
        self.pwm_right_forward.start(0)
        self.pwm_right_backward.start(0)
        
        log("GPIO and PWM setup complete")
        
    def start(self):
        """Start the speed control thread."""
        self.speed_thread = threading.Thread(target=self._speed_control_loop)
        self.speed_thread.daemon = True
        self.speed_thread.start()
        log("Speed control thread started")
        
    def stop(self):
        """Stop motors and clean up resources."""
        log("Stopping car controller")
        self.running = False
        time.sleep(0.2)  # Give thread time to stop
        
        # Stop motors
        self.target_left_speed = 0
        self.target_right_speed = 0
        self.current_left_speed = 0
        self.current_right_speed = 0
        self._set_motors()
        
        # Clean up
        self._cleanup()
        
    def _cleanup(self):
        """Stop PWM and clean up GPIO."""
        log("Cleaning up GPIO")
        try:
            self.pwm_left_forward.stop()
            self.pwm_left_backward.stop()
            self.pwm_right_forward.stop()
            self.pwm_right_backward.stop()
            GPIO.cleanup()
            log("GPIO cleanup completed")
        except Exception as e:
            log(f"Exception during cleanup: {str(e)}")
            
    def _clamp(self, value, min_val, max_val):
        """Limit a value between min and max."""
        return max(min(value, max_val), min_val)
        
    def _set_motors(self):
        """Apply current speed values to motor PWM."""
        try:
            # Left motor
            if self.current_left_speed > 0:
                self.pwm_left_forward.ChangeDutyCycle(self.current_left_speed)
                self.pwm_left_backward.ChangeDutyCycle(0)
            else:
                self.pwm_left_forward.ChangeDutyCycle(0)
                self.pwm_left_backward.ChangeDutyCycle(abs(self.current_left_speed))
            
            # Right motor
            if self.current_right_speed > 0:
                self.pwm_right_forward.ChangeDutyCycle(self.current_right_speed)
                self.pwm_right_backward.ChangeDutyCycle(0)
            else:
                self.pwm_right_forward.ChangeDutyCycle(0)
                self.pwm_right_backward.ChangeDutyCycle(abs(self.current_right_speed))
        except Exception as e:
            log(f"Exception in set_motors: {str(e)}")
            
    def _speed_control_loop(self):
        """Thread function for speed control with smooth acceleration/deceleration."""
        log("Speed control thread started")
        last_update_time = time.time()
        
        try:
            while self.running:
                now = time.time()
                dt = now - last_update_time
                last_update_time = now
                
                # Calculate rate of change
                rate_factor = dt * 50
                accel = Config.ACCELERATION_RATE * rate_factor
                decel = Config.DECELERATION_RATE * rate_factor
                
                # Smoothly adjust motor speeds toward target values
                # Left motor
                if self.current_left_speed < self.target_left_speed:
                    self.current_left_speed = min(self.current_left_speed + accel, self.target_left_speed)
                elif self.current_left_speed > self.target_left_speed:
                    self.current_left_speed = max(self.current_left_speed - decel, self.target_left_speed)
                
                # Right motor
                if self.current_right_speed < self.target_right_speed:
                    self.current_right_speed = min(self.current_right_speed + accel, self.target_right_speed)
                elif self.current_right_speed > self.target_right_speed:
                    self.current_right_speed = max(self.current_right_speed - decel, self.target_right_speed)
                
                # Apply motor speeds
                self._set_motors()
                
                # Short delay
                time.sleep(Config.CONTROL_LOOP_DELAY)
        except Exception as e:
            log(f"Exception in speed control thread: {str(e)}")
            log(traceback.format_exc())
            self.running = False
        
        log("Speed control thread ended")
        
    def move_forward(self, increment=None):
        """Increase speed forward."""
        if increment is None:
            increment = Config.ACCELERATION_RATE
            
        self.target_left_speed = min(self.target_left_speed + increment, Config.MAX_SPEED)
        self.target_right_speed = min(self.target_right_speed + increment, Config.MAX_SPEED)
        log(f"Forward: L={self.target_left_speed}, R={self.target_right_speed}")
        
    def move_backward(self, increment=None):
        """Increase speed backward."""
        if increment is None:
            increment = Config.ACCELERATION_RATE
            
        self.target_left_speed = max(self.target_left_speed - increment, -Config.MAX_SPEED)
        self.target_right_speed = max(self.target_right_speed - increment, -Config.MAX_SPEED)
        log(f"Backward: L={self.target_left_speed}, R={self.target_right_speed}")
        
    def turn_left(self):
        """Turn left by adjusting motor speeds appropriately."""
        # Determine if we're moving forward or backward
        if self.target_left_speed >= 0 and self.target_right_speed >= 0:
            # Moving forward or stopped - increase right motor speed
            self.target_right_speed = min(self.target_right_speed + Config.ACCELERATION_RATE, Config.MAX_SPEED)
            # Optionally reduce left motor speed for sharper turns
            self.target_left_speed = max(self.target_left_speed - Config.ACCELERATION_RATE/2, 0)
        else:
            # Moving backward - decrease right motor speed (less negative)
            self.target_right_speed = min(self.target_right_speed + Config.ACCELERATION_RATE, 0)
            # Optionally make left more negative for sharper turns
            self.target_left_speed = max(self.target_left_speed - Config.ACCELERATION_RATE/2, -Config.MAX_SPEED)
            
        log(f"Left: L={self.target_left_speed}, R={self.target_right_speed}")
        
    def turn_right(self):
        """Turn right by adjusting motor speeds appropriately."""
        # Determine if we're moving forward or backward
        if self.target_left_speed >= 0 and self.target_right_speed >= 0:
            # Moving forward or stopped - increase left motor speed
            self.target_left_speed = min(self.target_left_speed + Config.ACCELERATION_RATE, Config.MAX_SPEED)
            # Optionally reduce right motor speed for sharper turns
            self.target_right_speed = max(self.target_right_speed - Config.ACCELERATION_RATE/2, 0)
        else:
            # Moving backward - decrease left motor speed (less negative)
            self.target_left_speed = min(self.target_left_speed + Config.ACCELERATION_RATE, 0)
            # Optionally make right more negative for sharper turns
            self.target_right_speed = max(self.target_right_speed - Config.ACCELERATION_RATE/2, -Config.MAX_SPEED)
            
        log(f"Right: L={self.target_left_speed}, R={self.target_right_speed}")
        
    def slow_down(self):
        """Gradually slow down both motors."""
        decel = Config.DECELERATION_RATE
        
        if abs(self.target_left_speed) < decel:
            self.target_left_speed = 0
        else:
            self.target_left_speed -= decel if self.target_left_speed > 0 else -decel
            
        if abs(self.target_right_speed) < decel:
            self.target_right_speed = 0
        else:
            self.target_right_speed -= decel if self.target_right_speed > 0 else -decel
            
        log(f"Slowing down: L={self.target_left_speed}, R={self.target_right_speed}")
        
    def run_test(self):
        """Run a simple sequence to test motors."""
        log("Starting simple test mode")
        print("Simple test mode - no keyboard control")
        print("Motors will cycle through: forward, stop, backward, stop")
        
        # Start speed control
        self.start()
        
        try:
            # Forward
            log("Test: Moving forward")
            print("Moving forward...")
            self.target_left_speed = 50
            self.target_right_speed = 50
            time.sleep(3)
            
            # Stop
            log("Test: Stopping")
            print("Stopping...")
            self.target_left_speed = 0
            self.target_right_speed = 0
            time.sleep(2)
            
            # Backward
            log("Test: Moving backward")
            print("Moving backward...")
            self.target_left_speed = -50
            self.target_right_speed = -50
            time.sleep(3)
            
            # Stop
            log("Test: Final stop")
            print("Stopping...")
            self.target_left_speed = 0
            self.target_right_speed = 0
            time.sleep(2)
            
            # Left turn test
            log("Test: Turning left")
            print("Turning left...")
            self.target_left_speed = 30
            self.target_right_speed = 60
            time.sleep(3)
            
            # Right turn test
            log("Test: Turning right")
            print("Turning right...")
            self.target_left_speed = 60
            self.target_right_speed = 30
            time.sleep(3)
            
            # Final stop
            log("Test: Final stop")
            print("Stopping...")
            self.target_left_speed = 0
            self.target_right_speed = 0
            time.sleep(1)
            
            print("Test complete")
            log("Simple test completed successfully")
        except Exception as e:
            log(f"Exception during simple test: {str(e)}")
            log(traceback.format_exc())
            print(f"Test failed with error: {str(e)}")
        finally:
            self.stop()
            
    def run_curses_interface(self):
        """Run with interactive keyboard control using curses."""
        log("Starting curses interface")
        curses.wrapper(self._curses_main)
    
    def _curses_main(self, stdscr):
        """Main curses UI function."""
        try:
            # Set up curses
            curses.curs_set(0)  # Hide cursor
            stdscr.nodelay(True)  # Non-blocking input
            stdscr.timeout(100)   # Input polling interval in ms
            log("Curses initialized")
            
            # Start speed control
            self.start()
            
            # Get window dimensions
            height, width = stdscr.getmaxyx()
            status_win = curses.newwin(5, width, height-5 if height > 5 else 0, 0)
            log(f"Window size: {width}x{height}")
            
            while self.running:
                # Display status
                status_win.clear()
                status_win.addstr(0, 0, "Car Control - Press Q to quit")
                status_win.addstr(1, 0, f"Left Motor: {self.current_left_speed:.1f}% (Target: {self.target_left_speed}%)")
                status_win.addstr(2, 0, f"Right Motor: {self.current_right_speed:.1f}% (Target: {self.target_right_speed}%)")
                status_win.addstr(3, 0, "Controls: W (forward), S (backward), A (left), D (right)")
                status_win.refresh()
                
                # Get key input
                key = stdscr.getch()
                
                # Default to gradual stop when no keys pressed
                self.slow_down()
                
                # Process key presses
                if key == ord('q') or key == ord('Q'):
                    log("Quit command received")
                    self.running = False
                    break
                elif key == ord('w') or key == ord('W'):  # Forward
                    self.move_forward()
                elif key == ord('s') or key == ord('S'):  # Backward
                    self.move_backward()
                elif key == ord('a') or key == ord('A'):  # Left
                    self.turn_left()
                elif key == ord('d') or key == ord('D'):  # Right
                    self.turn_right()
                
                # Ensure values stay within bounds
                self.target_left_speed = self._clamp(self.target_left_speed, -Config.MAX_SPEED, Config.MAX_SPEED)
                self.target_right_speed = self._clamp(self.target_right_speed, -Config.MAX_SPEED, Config.MAX_SPEED)
                
                time.sleep(0.01)
        except Exception as e:
            log(f"Exception in curses_main: {str(e)}")
            log(traceback.format_exc())
        finally:
            self.stop()
            log("curses_main completed")

def main():
    """Main program entry point."""
    log("Main program started")
    print("Raspberry Pi Car Control")
    print(f"Debug log: {LOG_FILE}")
    
    try:
        car = CarController()
        
        # Check command-line arguments
        if len(sys.argv) > 1 and sys.argv[1] == "--test":
            print("Running in test mode (no keyboard control)")
            car.run_test()
        else:
            print("Starting with keyboard control (curses)")
            print("If the program crashes, check the debug log")
            car.run_curses_interface()
    except Exception as e:
        log(f"Exception in main: {str(e)}")
        log(traceback.format_exc())
        print(f"Error: {str(e)}")
        print(f"See log file for details: {LOG_FILE}")
    finally:
        log("Program terminated")
        print("Program terminated")

if __name__ == "__main__":
    main()