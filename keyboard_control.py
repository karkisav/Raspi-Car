#!/usr/bin/env python3
import time
import sys
import threading
import curses  # For keyboard input

# Import the Car class from your existing file
from car_control import Car  # Assuming your original file is saved as car_control.py

def keyboard_control():
    """Control the car using keyboard inputs"""
    # Initialize the car
    car = Car()
    car.start_control_thread()
    
    # Initialize curses for keyboard input
    stdscr = curses.initscr()
    curses.noecho()        # Don't echo keypresses
    curses.cbreak()        # React to keys instantly
    stdscr.keypad(True)    # Enable special keys
    stdscr.nodelay(True)   # Non-blocking input mode
    
    try:
        # Display instructions
        stdscr.addstr(0, 0, "Raspberry Pi Car Keyboard Control")
        stdscr.addstr(1, 0, "-----------------------------")
        stdscr.addstr(2, 0, "Arrow Up    : Forward")
        stdscr.addstr(3, 0, "Arrow Down  : Backward")
        stdscr.addstr(4, 0, "Arrow Left  : Turn Left")
        stdscr.addstr(5, 0, "Arrow Right : Turn Right")
        stdscr.addstr(6, 0, "Spacebar    : Stop")
        stdscr.addstr(7, 0, "'+' / '-'   : Increase/Decrease Speed")
        stdscr.addstr(8, 0, "Q           : Quit")
        stdscr.addstr(10, 0, "Current speed: 50")
        stdscr.refresh()
        
        # Set initial speed
        speed = 50
        
        # Main loop for keyboard control
        while True:
            # Get keyboard input
            key = stdscr.getch()
            
            if key == curses.KEY_UP:
                car.forward(speed=speed)
                stdscr.addstr(12, 0, "Action: Moving Forward    ")
            elif key == curses.KEY_DOWN:
                car.backward(speed=speed)
                stdscr.addstr(12, 0, "Action: Moving Backward   ")
            elif key == curses.KEY_LEFT:
                car.left(speed=speed)
                stdscr.addstr(12, 0, "Action: Turning Left      ")
            elif key == curses.KEY_RIGHT:
                car.right(speed=speed)
                stdscr.addstr(12, 0, "Action: Turning Right     ")
            elif key == ord(' '):  # Spacebar
                car.stop()
                stdscr.addstr(12, 0, "Action: Stopping          ")
            elif key == ord('+') or key == ord('='):
                speed = min(speed + 10, 100)
                stdscr.addstr(10, 0, f"Current speed: {speed}  ")
                stdscr.addstr(12, 0, "Action: Speed Increased   ")
            elif key == ord('-'):
                speed = max(speed - 10, 10)
                stdscr.addstr(10, 0, f"Current speed: {speed}  ")
                stdscr.addstr(12, 0, "Action: Speed Decreased   ")
            elif key == ord('q') or key == ord('Q'):
                break
            
            stdscr.refresh()
            time.sleep(0.05)  # Small delay to prevent CPU hogging
            
    except Exception as e:
        stdscr.addstr(15, 0, f"Error: {str(e)}")
        stdscr.refresh()
        time.sleep(2)
    finally:
        # Clean up
        car.cleanup()
        
        # Reset terminal settings
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()
        print("Keyboard control terminated")

if __name__ == "__main__":
    print("Starting keyboard control for Raspberry Pi car...")
    keyboard_control()