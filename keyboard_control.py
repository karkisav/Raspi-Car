import curses
import time
import sys
from car_control import Car  # Importing from your existing file

def main(stdscr):
    # Set up curses
    curses.curs_set(0)  # Hide cursor
    stdscr.clear()
    stdscr.refresh()
    stdscr.nodelay(True)  # Non-blocking input
    stdscr.timeout(50)  # Refresh every 50ms (more responsive)
    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
    
    # Enable keypad for arrow keys
    stdscr.keypad(True)
    
    # Create car instance
    car = Car()
    car.start_control_thread()
    
    # Default settings with separate speeds
    straight_speed = 20  # Speed for forward/backward
    turn_speed = 15      # Speed for turning
    duration = 0.05
    
    # Control state
    current_direction = "Stopped"
    last_key_time = time.time()
    
    # Instructions
    instructions = [
        "Raspberry Pi RC Car Keyboard Control",
        "-----------------------------------",
        "Arrow Keys: Drive car",
        "      Space: Stop",
        "          +: Increase speeds (turn +5, straight +10)",
        "          -: Decrease speeds (turn -5, straight -10)",
        "          q: Quit",
        "",
        "Straight speed: {}  |  Turn speed: {}",
        "Current direction: {}"
    ]
    
    # Display initial screen
    for i, line in enumerate(instructions[:-2]):
        stdscr.addstr(i, 0, line, curses.color_pair(1))
    stdscr.addstr(len(instructions)-2, 0, instructions[-2].format(straight_speed, turn_speed), curses.color_pair(1))
    stdscr.addstr(len(instructions)-1, 0, instructions[-1].format(current_direction), curses.color_pair(1))
    stdscr.refresh()
    
    try:
        running = True
        while running:
            # Get key input
            key = stdscr.getch()
            
            # Clear status line for updates
            stdscr.addstr(len(instructions)-1, 0, " " * 50)
            
            # Process key input
            current_time = time.time()
            if key != -1 and (current_time - last_key_time >= duration or key == ord(' ') or key == ord('q')):
                if key == curses.KEY_UP:
                    car.forward(speed=straight_speed)
                    current_direction = "Forward"
                    last_key_time = current_time
                elif key == curses.KEY_DOWN:
                    car.backward(speed=straight_speed)
                    current_direction = "Backward"
                    last_key_time = current_time
                elif key == curses.KEY_RIGHT:
                    car.left(speed=turn_speed)
                    current_direction = "Left"
                    last_key_time = current_time
                elif key == curses.KEY_LEFT:
                    car.right(speed=turn_speed)
                    current_direction = "Right"
                    last_key_time = current_time
                elif key == ord(' '):
                    car.stop()
                    current_direction = "Stopped"
                    last_key_time = current_time
                elif key == ord('+') or key == ord('='):
                    # Increase both speeds with different increments
                    straight_speed = min(straight_speed + 10, 100)
                    turn_speed = min(turn_speed + 5, 65)
                elif key == ord('-') or key == ord('_'):
                    # Decrease both speeds with different increments
                    straight_speed = max(straight_speed - 10, 10)
                    turn_speed = max(turn_speed - 5, 10)
                elif key == ord('q'):
                    running = False
                
                # Update speed display
                stdscr.addstr(len(instructions)-2, 0, " " * 50)  # Clear line
                stdscr.addstr(len(instructions)-2, 0, instructions[-2].format(straight_speed, turn_speed), curses.color_pair(1))
                # Update direction display
                stdscr.addstr(len(instructions)-1, 0, instructions[-1].format(current_direction), curses.color_pair(1))
            
            stdscr.refresh()
            time.sleep(0.01)  # Small delay to prevent CPU hogging
            
    except Exception as e:
        # Exit cleanly on error
        stdscr.clear()
        stdscr.addstr(0, 0, f"Error: {str(e)}")
        stdscr.refresh()
        time.sleep(2)
    finally:
        # Clean up
        car.stop()
        car.cleanup()

if __name__ == "__main__":
    print("Starting keyboard control for Raspberry Pi RC Car...")
    print("The control interface will appear in the terminal.")
    print("Press Ctrl+C to exit if the interface doesn't work.")
    time.sleep(2)  # Short delay to read the message
    
    # Start curses application
    curses.wrapper(main)
    
    print("Keyboard control terminated.")