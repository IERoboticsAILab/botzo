import pygame
import sys
import threading
import time

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

# Mapping dictionaries
PS3_AXIS_MAP = {
    "LEFT_STICK_X": 0,
    "LEFT_STICK_Y": 1,
    "RIGHT_STICK_X": 2,
    "RIGHT_STICK_Y": 3,
    "L2_TRIGGER": 4,
    "R2_TRIGGER": 5
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

# Thread flag
running = True

def controller_reader_thread():
    """Thread function to continuously read controller input and update the dictionary"""
    global running, controller_state
    
    pygame.init()
    
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
                    controller_state[axis_name] = joystick.get_axis(axis_index)
            
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