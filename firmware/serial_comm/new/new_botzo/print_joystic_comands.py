import pygame
import sys
import threading
import time
import os

# Initialize the controller dictionary
controller_state = {
    # Button states
    "X": 0,
    "CIRCLE": 0,
    "SQUARE": 0,
    "TRIANGLE": 0,
    "L1": 0,
    "R1": 0,
    "L2": 0,
    "R2": 0,
    "SELECT": 0,
    "START": 0,
    "PS": 0,
    "L3": 0,  # Left stick button
    "R3": 0,  # Right stick button
    "D_UP": 0,
    "D_DOWN": 0,
    "D_LEFT": 0,
    "D_RIGHT": 0,
    
    # Joystick values
    "LEFT_STICK_X": 0.0,  # Range: -1.0 to 1.0
    "LEFT_STICK_Y": 0.0,  # Range: -1.0 to 1.0
    "RIGHT_STICK_X": 0.0, # Range: -1.0 to 1.0
    "RIGHT_STICK_Y": 0.0, # Range: -1.0 to 1.0
    "L2_TRIGGER": 0.0,    # Range: -1.0 to 1.0
    "R2_TRIGGER": 0.0     # Range: -1.0 to 1.0
}

# Previous button states for edge detection
previous_button_states = {key: 0 for key in controller_state.keys() if not key.endswith("_X") and not key.endswith("_Y") and not key.endswith("TRIGGER")}

# Mapping dictionaries
PS3_AXIS_MAP = {
    "LEFT_STICK_X": 0,   # Left stick horizontal axis
    "LEFT_STICK_Y": 1,   # Left stick vertical axis
    "RIGHT_STICK_X": 3,  # Changed from 2 to 3 - Right stick horizontal axis
    "RIGHT_STICK_Y": 4,  # Changed from 3 to 4 - Right stick vertical axis
    "L2_TRIGGER": 2,     # Changed from 4 to 2 - Left trigger
    "R2_TRIGGER": 5      # Right trigger
}

PS3_BUTTON_MAP = {
    "X": 0,        # X button
    "CIRCLE": 1,   # Circle button
    "SQUARE": 2,   # Square button
    "TRIANGLE": 3, # Triangle button
    "L1": 4,       # L1 button
    "R1": 5,       # R1 button
    "L2": 6,       # L2 button (digital)
    "R2": 7,       # R2 button (digital)
    "SELECT": 8,   # Select button
    "START": 9,    # Start button
    "PS": 10,      # PS button
    "L3": 11,      # Left stick button
    "R3": 12,      # Right stick button
    "D_UP": 13,    # D-pad up
    "D_DOWN": 14,  # D-pad down
    "D_LEFT": 15,  # D-pad left
    "D_RIGHT": 16  # D-pad right
}

# Joystick deadzone (values below this are considered 0)
JOYSTICK_DEADZONE = 0.15

# Thread flag
running = True

# Ensure pygame can run headlessly (no X/display) by using the SDL dummy drivers
if "SDL_VIDEODRIVER" not in os.environ and "DISPLAY" not in os.environ:
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

def controller_reader_thread():
    """Thread function to continuously read controller input and update the dictionary"""
    global running, controller_state, previous_button_states
    
    pygame.init()
    # Initialize display and joystick subsystems explicitly to avoid
    # "video system not initialized" errors on headless systems
    try:
        pygame.display.init()
        # Minimal hidden window ensures event queue works even with dummy driver
        if not pygame.display.get_init():
            raise RuntimeError("pygame display failed to initialize")
        try:
            pygame.display.set_mode((1, 1))
        except Exception:
            # With dummy driver set_mode may still raise; ignore if so
            pass
    except Exception as e:
        print(f"Warning: initializing headless video driver failed: {e}")

    try:
        pygame.joystick.init()
    except Exception as e:
        print(f"Warning: joystick init failed: {e}")
    
    # Check if a joystick is connected
    if pygame.joystick.get_count() == 0:
        print("No joystick detected! Please connect a PS3 controller.")
        pygame.joystick.quit()
        pygame.event.quit()
        running = False
        return
    
    # Initialize the joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Detected joystick: {joystick.get_name()}")
    
    button_count = joystick.get_numbuttons()
    
    try:
        while running:
            # Save the previous button states for edge detection
            for button_name in previous_button_states:
                previous_button_states[button_name] = controller_state.get(button_name, 0)
                
            # Process pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    break
                
                if event.type == pygame.JOYBUTTONDOWN:
                    button_index = event.button
                    for button_name, index in PS3_BUTTON_MAP.items():
                        if index == button_index and index < button_count:
                            controller_state[button_name] = 1
                            break
                
                if event.type == pygame.JOYBUTTONUP:
                    button_index = event.button
                    for button_name, index in PS3_BUTTON_MAP.items():
                        if index == button_index and index < button_count:
                            controller_state[button_name] = 0
                            break
            
            # Update axis values each frame (continuous)
            for axis_name, axis_index in PS3_AXIS_MAP.items():
                if axis_index < joystick.get_numaxes():
                    value = joystick.get_axis(axis_index)
                    # Apply deadzone
                    if abs(value) < JOYSTICK_DEADZONE:
                        value = 0.0
                    controller_state[axis_name] = value
            
            # Small delay to prevent high CPU usage
            time.sleep(0.01)
    
    except Exception as e:
        print(f"Error in controller reader thread: {e}")
    finally:
        pygame.quit()
        running = False

def start_controller_reader():
    """Start the controller reader thread"""
    global running
    running = True
    thread = threading.Thread(target=controller_reader_thread, daemon=True)
    thread.start()
    return thread

def stop_controller_reader():
    """Stop the controller reader thread"""
    global running
    running = False

def get_button_state(button_name):
    """Get the current state of a button"""
    return controller_state.get(button_name, 0)

def button_just_pressed(button_name):
    """Returns true if the button was just pressed (rising edge)"""
    current = controller_state.get(button_name, 0)
    previous = previous_button_states.get(button_name, 0)
    return current == 1 and previous == 0

def button_just_released(button_name):
    """Returns true if the button was just released (falling edge)"""
    current = controller_state.get(button_name, 0)
    previous = previous_button_states.get(button_name, 0)
    return current == 0 and previous == 1

def get_axis_value(axis_name):
    """Get the current value of an axis"""
    return controller_state.get(axis_name, 0.0)

def get_controller_state():
    """Get the entire controller state dictionary"""
    return controller_state.copy()

# Auto-start the controller reader when imported
controller_thread = None

if __name__ == "__main__":
    # If run directly, start the controller reader and print state updates
    controller_thread = start_controller_reader()
    
    try:
        while running:
            # Print current controller state
            print("\033c", end="")  # Clear console
            print("Controller State:")
            for key, value in controller_state.items():
                if key.startswith("LEFT_") or key.startswith("RIGHT_") or key.endswith("TRIGGER"):
                    # Format axis values with 2 decimal places
                    print(f"{key}: {value:.2f}")
                else:
                    print(f"{key}: {value}")
            
            # Print any just pressed buttons
            just_pressed = [btn for btn in previous_button_states.keys() if button_just_pressed(btn)]
            if just_pressed:
                print("\nJust pressed:", ", ".join(just_pressed))
            
            # Print any just released buttons
            just_released = [btn for btn in previous_button_states.keys() if button_just_released(btn)]
            if just_released:
                print("\nJust released:", ", ".join(just_released))
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\nStopping controller reader...")
    finally:
        stop_controller_reader()
        if controller_thread:
            controller_thread.join(timeout=1.0)
else:
    # Start the controller reader when imported
    controller_thread = start_controller_reader() 