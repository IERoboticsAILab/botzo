import pygame
import sys
import socket  # Add socket import for UDP communication
import subprocess  # For running IK_Sender_Jump.py

def main():
    
    pygame.init()
    
    # Mapping dictionary:
    PS3_AXIS_MAP = {
        "LEFT_STICK_X": 0,
        "LEFT_STICK_Y": 1,
        "RIGHT_STICK_X": 2,
        "RIGHT_STICK_Y": 3,
        "L2_TRIGGER": 4,
        "R2_TRIGGER": 5
    }
    
    # PS3 Button mapping for reference
    # These might need adjustment based on your controller
    PS3_BUTTON_MAP = {
        "X": 0,        # X button
        "CIRCLE": 1,   # Circle button
        "SQUARE": 2,   # Square button
        "TRIANGLE": 3, # Triangle button
    }

    # Set up UDP socket for sending commands to IK_Sender.py
    ik_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    IK_SENDER_IP = '127.0.0.1'
    IK_SENDER_PORT = 12346
    
    WIDTH, HEIGHT = 900, 660
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("PS3 Controller Demo")

    # Initialize
    if pygame.joystick.get_count() == 0:
        print("No joystick detected! Please connect a PS3 controller.")
        pygame.joystick.quit()
        pygame.event.quit()
        sys.exit()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Detected joystick: {joystick.get_name()}")
    
    button_count = joystick.get_numbuttons()
    button_states = [0] * button_count
    
    # Flag to prevent multiple executions of IK_Sender_Jump.py
    jump_process = None
    
    # Function to convert an axis value in [-1..1] to a 0-100 speed.
    def axis_to_speed(value):
        return int(abs(value) * 100)
    
    # Helper function 
    def draw_joystick_graph(surface, x_val, y_val, center_x, center_y, radius=60):
        """
        Draw a circle representing the joystick range, and a dot for the current axis position.
        x_val, y_val in [-1..1]. center_x, center_y is the center of the "graph".
        """
        pygame.draw.circle(surface, (230, 230, 230), (center_x, center_y), radius, 3)
        dot_x = center_x + int(x_val * (radius - 10))
        dot_y = center_y + int(y_val * (radius - 10))
        pygame.draw.circle(surface, (255, 0, 0), (dot_x, dot_y), 8)
    
    clock = pygame.time.Clock()
    running = True
    
    title_font = pygame.font.SysFont("arial", 36, bold=True)
    panel_title_font = pygame.font.SysFont("arial", 24, bold=True)
    label_font = pygame.font.SysFont("arial", 20)
    
    # Track the previous button states to detect changes
    prev_button_states = [0] * button_count
    
    try:
        while running:
            clock.tick(60)
            
            # Read axis values each frame (continuous)
            left_x  = joystick.get_axis(PS3_AXIS_MAP["LEFT_STICK_X"])
            left_y  = joystick.get_axis(PS3_AXIS_MAP["LEFT_STICK_Y"])
            right_x = joystick.get_axis(PS3_AXIS_MAP["RIGHT_STICK_X"])
            right_y = joystick.get_axis(PS3_AXIS_MAP["RIGHT_STICK_Y"])
            
            l2_axis = joystick.get_axis(PS3_AXIS_MAP["L2_TRIGGER"])
            r2_axis = joystick.get_axis(PS3_AXIS_MAP["R2_TRIGGER"])
            # Threshold approach to treat triggers as binary
            l2_pressed = 0 if l2_axis < 0.5 else 1
            r2_pressed = 0 if r2_axis < 0.5 else 1
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                
                if event.type == pygame.JOYBUTTONDOWN:
                    button_index = event.button
                    button_states[button_index] = 1
                    
                if event.type == pygame.JOYBUTTONUP:
                    button_index = event.button
                    button_states[button_index] = 0
            
            # Process button presses for different movements
            
            # X button - Jump
            if button_states[PS3_BUTTON_MAP["X"]] == 1 and prev_button_states[PS3_BUTTON_MAP["X"]] == 0:
                print("X button pressed - Jump")
                
                # Terminate any existing jump process
                if jump_process is not None and jump_process.poll() is None:
                    jump_process.terminate()
                    jump_process = None
                
                # Start new jump process
                jump_process = subprocess.Popen(["python3", "IK_Sender_Jump.py"])
            
            # Circle button - Forward
            if button_states[PS3_BUTTON_MAP["CIRCLE"]] == 1 and prev_button_states[PS3_BUTTON_MAP["CIRCLE"]] == 0:
                print("Circle button pressed - Forward movement")
                ik_socket.sendto(b"FORWARD", (IK_SENDER_IP, IK_SENDER_PORT))
            
            # Square button - Backward
            if button_states[PS3_BUTTON_MAP["SQUARE"]] == 1 and prev_button_states[PS3_BUTTON_MAP["SQUARE"]] == 0:
                print("Square button pressed - Backward movement")
                ik_socket.sendto(b"BACKWARD", (IK_SENDER_IP, IK_SENDER_PORT))
            

            # Triangle button - Stop the robot after returning to home position
            if button_states[PS3_BUTTON_MAP["TRIANGLE"]] == 1 and prev_button_states[PS3_BUTTON_MAP["TRIANGLE"]] == 0:
                print("Triangle button pressed - Stop robot after going to home position")
                ik_socket.sendto(b"HOME_AND_STOP", (IK_SENDER_IP, IK_SENDER_PORT))
            
            # Update previous button states
            prev_button_states = button_states.copy()
            
            screen.fill((45, 48, 62))
            
            title_text = title_font.render("PS3 Controller Tester", True, (255, 255, 255))
            screen.blit(
                title_text, 
                (WIDTH // 2 - title_text.get_width() // 2, 20)
            )
            
            # ---Joystick Panels---
            
            panel_width = (WIDTH // 2) - 60
            panel_height = 250
            
            left_panel_rect = pygame.Rect(40, 140, panel_width, panel_height)
            right_panel_rect = pygame.Rect((WIDTH // 2) + 20, 140, panel_width, panel_height)
            
            left_panel_surf = pygame.Surface((panel_width, panel_height), pygame.SRCALPHA)
            left_panel_surf.fill((70, 70, 90, 180))  
            
            right_panel_surf = pygame.Surface((panel_width, panel_height), pygame.SRCALPHA)
            right_panel_surf.fill((70, 70, 90, 180))
            
            left_panel_title = panel_title_font.render("Left Joystick", True, (255, 255, 255))
            right_panel_title = panel_title_font.render("Right Joystick", True, (255, 255, 255))
            
            left_panel_surf.blit(
                left_panel_title, 
                (left_panel_surf.get_width() // 2 - left_panel_title.get_width() // 2, 10)
            )
            
            right_panel_surf.blit(
                right_panel_title, 
                (right_panel_surf.get_width() // 2 - right_panel_title.get_width() // 2, 10)
            )
            
            j_center_x = panel_width // 2
            j_center_y = (panel_height // 2) + 20
            
            # Left joystick
            draw_joystick_graph(left_panel_surf, left_x, left_y, j_center_x, j_center_y, radius=60)
            
            # Right joystick
            draw_joystick_graph(right_panel_surf, right_x, right_y, j_center_x, j_center_y, radius=60)
            
            #"speed values" for each joystick
            left_speed_x  = axis_to_speed(left_x)
            left_speed_y  = axis_to_speed(left_y)
            right_speed_x = axis_to_speed(right_x)
            right_speed_y = axis_to_speed(right_y)
            
            speed_info_left = f"Lx Speed: {left_speed_x}    Ly Speed: {left_speed_y}"
            speed_info_right = f"Rx Speed: {right_speed_x}    Ry Speed: {right_speed_y}"
            
            text_left = label_font.render(speed_info_left, True, (255, 255, 255))
            text_right = label_font.render(speed_info_right, True, (255, 255, 255))
            
            left_panel_surf.blit(
                text_left,
                (
                    j_center_x - text_left.get_width() // 2,
                    j_center_y + 70
                )
            )
            right_panel_surf.blit(
                text_right,
                (
                    j_center_x - text_right.get_width() // 2,
                    j_center_y + 70
                )
            )
            
            screen.blit(left_panel_surf, (left_panel_rect.x, left_panel_rect.y))
            screen.blit(right_panel_surf, (right_panel_rect.x, right_panel_rect.y))
            
            # --- Bottom Panels ---
            
            bottom_panel_height = 200
            bottom_panel_rect = pygame.Rect(40, 400, WIDTH - 80, bottom_panel_height)
            
            bottom_panel_surf = pygame.Surface((bottom_panel_rect.width, bottom_panel_rect.height), pygame.SRCALPHA)
            bottom_panel_surf.fill((70, 70, 90, 180))
            
            btn_title = panel_title_font.render("Button States (Binary)", True, (255, 255, 255))
            bottom_panel_surf.blit(btn_title, (20, 10))
            
            # Add button command mapping information
            button_commands = label_font.render("X: Jump | Circle: Forward | Square: Backward | Triangle: Rotate Left", True, (255, 255, 255))
            bottom_panel_surf.blit(button_commands, (20, 200 - 30))
            
            col_x_start = 20
            col_y_start = 50
            line_spacing = 25
            
            max_per_column = 8
            
            col_count = 0
            row_count = 0
            
            for i, state in enumerate(button_states):
                text = label_font.render(f"Button {i}: {state}", True, (255, 255, 255))
                
                x_pos = col_x_start + col_count * 180
                y_pos = col_y_start + row_count * line_spacing
                
                bottom_panel_surf.blit(text, (x_pos, y_pos))
                
                row_count += 1
                if row_count >= max_per_column:
                    row_count = 0
                    col_count += 1
            
            triggers_text_L2 = label_font.render(f"L2: {l2_pressed}", True, (255, 255, 255))
            triggers_text_R2 = label_font.render(f"R2: {r2_pressed}", True, (255, 255, 255))
            
            x_pos_l2 = col_x_start + col_count * 180
            y_pos_l2 = col_y_start + row_count * line_spacing
            bottom_panel_surf.blit(triggers_text_L2, (x_pos_l2, y_pos_l2))
            
            row_count += 1
            y_pos_r2 = col_y_start + row_count * line_spacing
            bottom_panel_surf.blit(triggers_text_R2, (x_pos_l2, y_pos_r2))
            
            screen.blit(bottom_panel_surf, (bottom_panel_rect.x, bottom_panel_rect.y))
            
            pygame.display.flip()
    
    finally:
        # Clean up: terminate any running jump processes
        if jump_process is not None and jump_process.poll() is None:
            print("Terminating running jump process...")
            jump_process.terminate()
        
        # Close the socket
        ik_socket.close()
        
        # Quit pygame
        pygame.quit()
        
if __name__ == "__main__":
    main()
