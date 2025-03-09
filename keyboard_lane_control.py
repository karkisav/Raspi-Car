import curses
import time
import sys
from car_control import Car  # Importing from your existing file
from lane_detection import LaneDetector  # Import the lane detection module

def main(stdscr):
    # Set up curses
    curses.curs_set(0)  # Hide cursor
    stdscr.clear()
    stdscr.refresh()
    stdscr.nodelay(True)  # Non-blocking input
    stdscr.timeout(50)  # Refresh every 50ms (more responsive)
    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
    
    # Enable keypad for arrow keys
    stdscr.keypad(True)
    
    # Create car instance
    car = Car()
    car.start_control_thread()
    
    # Create lane detector
    lane_detector = LaneDetector()
    
    # Default settings with separate speeds
    straight_speed = 20  # Speed for forward/backward
    turn_speed = 15      # Speed for turning
    duration = 0.05
    
    # Control state
    current_direction = "Stopped"
    last_key_time = time.time()
    auto_mode = False
    
    # Auto mode settings
    auto_straight_speed = 15  # Lower speed for auto mode
    auto_turn_sensitivity = 0.7  # How sensitive the turning is to deviation
    
    # Instructions
    instructions = [
        "Raspberry Pi RC Car Control with Lane Following",
        "-------------------------------------------",
        "Arrow Keys: Manual drive",
        "      Space: Stop",
        "          +: Increase speeds (turn +5, straight +10)",
        "          -: Decrease speeds (turn -5, straight -10)",
        "          a: Toggle autonomous mode",
        "          d: Toggle camera display",
        "          q: Quit",
        "",
        "Straight speed: {}  |  Turn speed: {}",
        "Current direction: {}",
        "Mode: {}"
    ]
    
    # Display initial screen
    for i, line in enumerate(instructions[:-3]):
        stdscr.addstr(i, 0, line, curses.color_pair(1))
    stdscr.addstr(len(instructions)-3, 0, instructions[-3].format(straight_speed, turn_speed), curses.color_pair(1))
    stdscr.addstr(len(instructions)-2, 0, instructions[-2].format(current_direction), curses.color_pair(1))
    stdscr.addstr(len(instructions)-1, 0, instructions[-1].format("Manual"), curses.color_pair(1))
    stdscr.refresh()
    
    try:
        running = True
        while running:
            # Get key input
            key = stdscr.getch()
            
            # Clear status lines for updates
            stdscr.addstr(len(instructions)-2, 0, " " * 50)
            stdscr.addstr(len(instructions)-1, 0, " " * 50)
            
            # Process key input
            current_time = time.time()
            manual_input = False
            
            if key != -1:
                if key == ord('a'):
                    # Toggle autonomous mode
                    auto_mode = not auto_mode
                    if auto_mode:
                        # Start lane detection when entering auto mode
                        lane_detector.start()
                        car.forward(speed=auto_straight_speed)
                    else:
                        # Stop lane detection when exiting auto mode
                        lane_detector.stop()
                        car.stop()
                    current_direction = "Auto" if auto_mode else "Stopped"
                    last_key_time = current_time
                elif key == ord('d'):
                    # Toggle camera display
                    lane_detector.toggle_display()
                elif key == ord('q'):
                    running = False
                elif key == ord('+') or key == ord('='):
                    # Increase both speeds with different increments
                    straight_speed = min(straight_speed + 10, 100)
                    turn_speed = min(turn_speed + 5, 65)
                    auto_straight_speed = min(auto_straight_speed + 5, 50)
                elif key == ord('-') or key == ord('_'):
                    # Decrease both speeds with different increments
                    straight_speed = max(straight_speed - 10, 10)
                    turn_speed = max(turn_speed - 5, 10)
                    auto_straight_speed = max(auto_straight_speed - 5, 10)
                elif auto_mode and (key == curses.KEY_UP or key == curses.KEY_DOWN or 
                                    key == curses.KEY_LEFT or key == curses.KEY_RIGHT or 
                                    key == ord(' ')):
                    # Exit auto mode if manual driving keys are pressed
                    auto_mode = False
                    lane_detector.stop()
                    manual_input = True
                
                # Handle manual driving only when not in auto mode or just exited auto mode
                if not auto_mode and (manual_input or key == curses.KEY_UP or key == curses.KEY_DOWN or 
                                      key == curses.KEY_LEFT or key == curses.KEY_RIGHT or key == ord(' ')):
                    if current_time - last_key_time >= duration or key == ord(' '):
                        if key == curses.KEY_UP:
                            car.forward(speed=straight_speed)
                            current_direction = "Forward"
                        elif key == curses.KEY_DOWN:
                            car.backward(speed=straight_speed)
                            current_direction = "Backward"
                        elif key == curses.KEY_RIGHT:
                            car.left(speed=turn_speed)
                            current_direction = "Left"
                        elif key == curses.KEY_LEFT:
                            car.right(speed=turn_speed)
                            current_direction = "Right"
                        elif key == ord(' '):
                            car.stop()
                            current_direction = "Stopped"
                        last_key_time = current_time
            
            # Handle autonomous driving mode
            if auto_mode:
                deviation = lane_detector.get_deviation()
                
                # Apply steering based on lane deviation
                if abs(deviation) < 0.1:
                    # Going straight
                    car.forward(speed=auto_straight_speed)
                    current_direction = "Auto-Straight"
                elif deviation < 0:
                    # Need to turn right (negative deviation)
                    turn_amount = min(abs(deviation) * auto_turn_sensitivity, 1.0)
                    right_speed = int(turn_speed * turn_amount)
                    car.right(speed=right_speed)
                    current_direction = f"Auto-Right ({right_speed})"
                else:
                    # Need to turn left (positive deviation)
                    turn_amount = min(abs(deviation) * auto_turn_sensitivity, 1.0)
                    left_speed = int(turn_speed * turn_amount)
                    car.left(speed=left_speed)
                    current_direction = f"Auto-Left ({left_speed})"
                    
                # Show deviation in UI
                stdscr.addstr(len(instructions), 0, f"Lane deviation: {deviation:.2f}", 
                             curses.color_pair(1 if abs(deviation) < 0.3 else 2))
            
            # Update speed display
            stdscr.addstr(len(instructions)-3, 0, " " * 50)  # Clear line
            stdscr.addstr(len(instructions)-3, 0, instructions[-3].format(
                f"{straight_speed}{'/' + str(auto_straight_speed) if auto_mode else ''}", 
                f"{turn_speed}"
            ), curses.color_pair(1))
            
            # Update direction display
            stdscr.addstr(len(instructions)-2, 0, instructions[-2].format(current_direction), curses.color_pair(1))
            
            # Update mode display
            mode_str = "Autonomous" if auto_mode else "Manual"
            stdscr.addstr(len(instructions)-1, 0, instructions[-1].format(mode_str), 
                         curses.color_pair(2 if auto_mode else 1))
            
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
        if auto_mode:
            lane_detector.stop()

if __name__ == "__main__":
    print("Starting keyboard control with lane following for Raspberry Pi RC Car...")
    print("The control interface will appear in the terminal.")
    print("Press Ctrl+C to exit if the interface doesn't work.")
    time.sleep(2)  # Short delay to read the message
    
    # Start curses application
    curses.wrapper(main)
    
    print("Keyboard control terminated.")