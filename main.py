#!/usr/bin/env python3
"""
Lane Following AI for Raspberry Pi Car
Detects yellow lane markings and controls the car to follow the lane
"""

import cv2
import numpy as np
import time
import sys
import argparse
import curses
from threading import Thread, Event

# Import our modules
from camera_module import CameraModule
from lane_detector import LaneDetector
from car_control import Car
from autonomous_controller import AutonomousController

# Function to display frame with OpenCV (in a separate thread)
def display_thread(frame_queue, stop_event):
    cv2.namedWindow('Lane Detection', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Lane Detection', 800, 600)
    
    while not stop_event.is_set():
        if not frame_queue.empty():
            frame_data = frame_queue.get()
            
            if frame_data is None:
                break
                
            main_frame, mask_frame = frame_data
            
            # Combine main frame and mask for display
            h, w = main_frame.shape[:2]
            mask_resized = cv2.resize(mask_frame, (w//3, h//3))
            mask_color = cv2.cvtColor(mask_resized, cv2.COLOR_GRAY2BGR)
            main_frame[0:h//3, 0:w//3] = mask_color
            
            cv2.imshow('Lane Detection', main_frame)
            
            # Check for key presses
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):  # ESC or 'q' to quit
                stop_event.set()
                break
    
    cv2.destroyAllWindows()

def main(stdscr=None):
    # Clear screen and hide cursor if running in curses mode
    if stdscr is not None:
        curses.curs_set(0)
        stdscr.clear()
        stdscr.nodelay(True)
        curses.start_color()
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
        stdscr.keypad(True)
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Lane following AI for Raspberry Pi Car')
    parser.add_argument('--no-display', action='store_true', help='Run without GUI display')
    parser.add_argument('--record', action='store_true', help='Record video of the lane detection')
    parser.add_argument('--output', default='lane_following.mp4', help='Output video file')
    
    # If running in curses mode, we can't get command line args directly
    if stdscr is None:
        args = parser.parse_args()
    else:
        # Default args when running in curses mode
        args = parser.parse_args([])
        args.no_display = False
        args.record = True
        args.output = 'lane_following.mp4'
    
    # Initialize components
    print("Initializing camera...")
    camera = CameraModule(resolution=(640, 480), framerate=30)
    
    print("Initializing lane detector...")
    lane_detector = LaneDetector()
    
    print("Initializing car control...")
    car = Car()
    
    print("Initializing autonomous controller...")
    controller = AutonomousController(car)
    
    # Set up video recording if requested
    if args.record:
        camera.start_recording(output_file=args.output)
    
    # Set up display if not disabled
    if not args.no_display:
        from queue import Queue
        frame_queue = Queue(maxsize=2)  # Limit queue size to reduce latency
        stop_event = Event()
        display_thread_instance = Thread(target=display_thread, args=(frame_queue, stop_event))
        display_thread_instance.daemon = True
        display_thread_instance.start()
    
    # Print instructions
    if stdscr is not None:
        instructions = [
            "Lane Following AI for Raspberry Pi Car",
            "-----------------------------------",
            "     a: Toggle autonomous mode",
            "     +: Increase speed",
            "     -: Decrease speed",
            "     r: Toggle recording",
            "     q: Quit",
            "",
            "Speed: {}  |  Turn Speed: {}",
            "Status: {}  |  Recording: {}"
        ]
        
        for i, line in enumerate(instructions[:-2]):
            stdscr.addstr(i, 0, line, curses.color_pair(1))
    else:
        print("===== Lane Following AI =====")
        print("Press 'a' to toggle autonomous mode")
        print("Press '+' to increase speed")
        print("Press '-' to decrease speed")
        print("Press 'r' to toggle recording")
        print("Press 'q' or Esc to quit")
    
    # Main loop
    try:
        autonomous_active = False
        running = True
        speed = 20
        turn_speed = 15
        
        while running:
            # Capture frame
            frame = camera.capture_frame()
            
            if frame is None:
                print("Failed to capture frame")
                break
            
            # Detect lanes
            result_frame, lane_info, mask = lane_detector.detect_lanes(frame)
            
            # Process lane info if autonomous mode is active
            if autonomous_active:
                controller.process_lane_info(lane_info)
            
            # Add status information to the frame
            status_text = "Active" if autonomous_active else "Manual"
            cv2.putText(result_frame, f"Mode: {status_text}", (10, 90), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Record frame if recording is enabled
            camera.record_frame(result_frame)
            
            # Update display if enabled
            if not args.no_display:
                if not frame_queue.full():
                    frame_queue.put((result_frame, mask))
            
            # Handle keyboard input
            if stdscr is not None:
                # Curses mode
                key = stdscr.getch()
                if key != -1:
                    if key == ord('a'):
                        autonomous_active = not autonomous_active
                        if autonomous_active:
                            controller.start()
                        else:
                            controller.stop()
                    elif key == ord('+') or key == ord('='):
                        speed = min(speed + 5, 50)
                        turn_speed = min(turn_speed + 3, 30)
                        controller.set_speeds(speed, turn_speed)
                    elif key == ord('-') or key == ord('_'):
                        speed = max(speed - 5, 15)
                        turn_speed = max(turn_speed - 3, 10)
                        controller.set_speeds(speed, turn_speed)
                    elif key == ord('r'):
                        if camera.recording:
                            camera.stop_recording()
                        else:
                            camera.start_recording(args.output)
                    elif key == ord('q'):
                        running = False
                
                # Update status display
                record_status = "Yes" if camera.recording else "No"
                stdscr.addstr(8, 0, instructions[-2].format(speed, turn_speed), curses.color_pair(1))
                stdscr.addstr(9, 0, instructions[-1].format(status_text, record_status), curses.color_pair(1))
                stdscr.refresh()
            else:
                # Non-curses mode, get input from opencv window
                if not args.no_display and stop_event.is_set():
                    running = False
            
            # Small delay to prevent CPU hogging
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        # Clean up
        print("Cleaning up...")
        if autonomous_active:
            controller.stop()
        
        controller.cleanup()
        
        if camera.recording:
            camera.stop_recording()
        camera.release()
        
        if not args.no_display:
            stop_event.set()
            if display_thread_instance.is_alive():
                display_thread_instance.join(timeout=1.0)
        
        print("Lane following terminated")

if __name__ == "__main__":
    # Check if we should use curses interface
    if '--no-curses' in sys.argv:
        sys.argv.remove('--no-curses')
        main()
    else:
        print("Starting lane following with curses interface...")
        print("Use --no-curses to run without the curses interface")
        time.sleep(1)
        curses.wrapper(main)