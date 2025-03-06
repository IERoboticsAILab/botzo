import pygame
import sys
import socket  # Add socket import for UDP communication
import subprocess  # For running IK_Sender_Jump.py
import time  # For waiting


def main():

    pygame.init()

    # Mapping dictionary:
    PS3_AXIS_MAP = {
        "LEFT_STICK_X": 0,
        "LEFT_STICK_Y": 1,
        "RIGHT_STICK_X": 2,
        "RIGHT_STICK_Y": 3,
        "L2_TRIGGER": 4,
        "R2_TRIGGER": 5,
    }

    # PS3 Button mapping for reference
    # These might need adjustment based on your controller
    PS3_BUTTON_MAP = {
        "X": 0,  # X button
        "CIRCLE": 1,  # Circle button
        "SQUARE": 2,  # Square button
        "TRIANGLE": 3,  # Triangle button
        "L1": 4,  # Left shoulder button
        "R1": 5,  # Right shoulder button
    }

    # Set up UDP socket for sending commands to IK_Sender.py
    ik_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    IK_SENDER_IP = "127.0.0.1"
    IK_SENDER_PORT = 12346
    
    # Set up socket to receive ready notification from IK_Sender_Jorge.py
    ready_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ready_socket.bind(("127.0.0.1", 12347))
    ready_socket.settimeout(0.1)  # Non-blocking with short timeout
    
    # Wait for IK_Sender_Jorge.py to be ready
    print("Waiting for IK_Sender_Jorge.py to be ready...")
    ik_sender_ready = False
    wait_start_time = time.time()
    wait_timeout = 10  # Wait up to 10 seconds
    
    while not ik_sender_ready and (time.time() - wait_start_time) < wait_timeout:
        try:
            data, addr = ready_socket.recvfrom(1024)
            if data.decode('utf-8') == "READY":
                ik_sender_ready = True
                print("IK_Sender_Jorge.py is ready!")
        except socket.timeout:
            # Just continue the loop
            pass
        time.sleep(0.1)
    
    if not ik_sender_ready:
        print("Warning: Timed out waiting for IK_Sender_Jorge.py to be ready.")
        print("Commands may not work until IK_Sender_Jorge.py is running.")
        print("Make sure to start IK_Sender_Jorge.py in another terminal.")
    
    # Connection status for display in UI
    connection_status = "Connected to IK_Sender" if ik_sender_ready else "NOT CONNECTED - Start IK_Sender_Jorge.py"
    connection_color = (0, 255, 0) if ik_sender_ready else (255, 0, 0)  # Green if connected, red if not
    
    # For periodic connection check
    last_connection_check = time.time()
    connection_check_interval = 5  # Check every 5 seconds
    
    WIDTH, HEIGHT = 900, 660
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("PS3 Controller Demo")

    # Initialize
    if pygame.joystick.get_count() == 0:
        print("No joystick detected! Using simulated joystick for testing.")
        # Instead of exiting, we'll create a simulated environment for testing
        class SimulatedJoystick:
            def __init__(self):
                self.name = "Simulated PS3 Controller"
                self.num_buttons = 12
                self.num_axes = 6
                
            def get_name(self):
                return self.name
                
            def get_numbuttons(self):
                return self.num_buttons
                
            def get_numaxes(self):
                return self.num_axes
                
            def get_axis(self, axis):
                return 0.0  # Neutral position
                
            def get_button(self, button):
                return 0  # Not pressed
        
        joystick = SimulatedJoystick()
        print("Created simulated joystick for testing")
    else:
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
            left_x = joystick.get_axis(PS3_AXIS_MAP["LEFT_STICK_X"])
            left_y = joystick.get_axis(PS3_AXIS_MAP["LEFT_STICK_Y"])
            right_x = joystick.get_axis(PS3_AXIS_MAP["RIGHT_STICK_X"])
            right_y = joystick.get_axis(PS3_AXIS_MAP["RIGHT_STICK_Y"])

            l2_axis = joystick.get_axis(PS3_AXIS_MAP["L2_TRIGGER"])
            r2_axis = joystick.get_axis(PS3_AXIS_MAP["R2_TRIGGER"])
            # Threshold approach to treat triggers as binary
            l2_pressed = 0 if l2_axis < 0.5 else 1
            r2_pressed = 0 if r2_axis < 0.5 else 1

            # Handle rotation using right stick X-axis
            # Use a deadzone to prevent small movements from triggering rotation
            deadzone = 0.2
            if abs(right_x) > deadzone:
                rotation_speed = int(abs(right_x) * 100)  # Scale to 0-100
                if right_x < 0:  # Left rotation
                    print(f"Right stick left - Rotate Left (speed: {rotation_speed})")
                    rotation_cmd = f"ROTATE_LEFT:{rotation_speed}"
                    try:
                        ik_socket.sendto(rotation_cmd.encode(), (IK_SENDER_IP, IK_SENDER_PORT))
                    except Exception as e:
                        print(f"Error sending ROTATE_LEFT command: {e}")
                else:  # Right rotation
                    print(f"Right stick right - Rotate Right (speed: {rotation_speed})")
                    rotation_cmd = f"ROTATE_RIGHT:{rotation_speed}"
                    try:
                        ik_socket.sendto(rotation_cmd.encode(), (IK_SENDER_IP, IK_SENDER_PORT))
                    except Exception as e:
                        print(f"Error sending ROTATE_RIGHT command: {e}")

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                if event.type == pygame.JOYBUTTONDOWN:
                    button_index = event.button
                    button_states[button_index] = 1

                if event.type == pygame.JOYBUTTONUP:
                    button_index = event.button
                    button_states[button_index] = 0
                    
                # Add keyboard controls for testing
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_x:  # X key for X button (Jump)
                        print("X key pressed - Simulating X button (Jump)")
                        button_states[PS3_BUTTON_MAP["X"]] = 1
                        try:
                            ik_socket.sendto(b"JUMP", (IK_SENDER_IP, IK_SENDER_PORT))
                        except Exception as e:
                            print(f"Error sending JUMP command: {e}")
                    
                    elif event.key == pygame.K_c:  # C key for Circle button (Forward)
                        print("C key pressed - Simulating Circle button (Forward)")
                        button_states[PS3_BUTTON_MAP["CIRCLE"]] = 1
                        try:
                            ik_socket.sendto(b"FORWARD", (IK_SENDER_IP, IK_SENDER_PORT))
                        except Exception as e:
                            print(f"Error sending FORWARD command: {e}")
                    
                    elif event.key == pygame.K_s:  # S key for Square button (Backward)
                        print("S key pressed - Simulating Square button (Backward)")
                        button_states[PS3_BUTTON_MAP["SQUARE"]] = 1
                        try:
                            ik_socket.sendto(b"BACKWARD", (IK_SENDER_IP, IK_SENDER_PORT))
                        except Exception as e:
                            print(f"Error sending BACKWARD command: {e}")
                    
                    elif event.key == pygame.K_t:  # T key for Triangle button (Home and Stop)
                        print("T key pressed - Simulating Triangle button (Home and Stop)")
                        button_states[PS3_BUTTON_MAP["TRIANGLE"]] = 1
                        try:
                            ik_socket.sendto(b"HOME_AND_STOP", (IK_SENDER_IP, IK_SENDER_PORT))
                        except Exception as e:
                            print(f"Error sending HOME_AND_STOP command: {e}")
                
                if event.type == pygame.KEYUP:
                    if event.key == pygame.K_x:
                        button_states[PS3_BUTTON_MAP["X"]] = 0
                    elif event.key == pygame.K_c:
                        button_states[PS3_BUTTON_MAP["CIRCLE"]] = 0
                    elif event.key == pygame.K_s:
                        button_states[PS3_BUTTON_MAP["SQUARE"]] = 0
                    elif event.key == pygame.K_t:
                        button_states[PS3_BUTTON_MAP["TRIANGLE"]] = 0

            # Process button presses for different movements

            # X button - Jump
            if (
                button_states[PS3_BUTTON_MAP["X"]] == 1
                and prev_button_states[PS3_BUTTON_MAP["X"]] == 0
            ):
                print("X button pressed - Jump")
                
                # Send JUMP command directly to IK_Sender_Jorge.py
                try:
                    ik_socket.sendto(b"JUMP", (IK_SENDER_IP, IK_SENDER_PORT))
                except Exception as e:
                    print(f"Error sending JUMP command: {e}")
                
                # Note: Keeping the code below commented out for reference
                # # Terminate any existing jump process
                # if jump_process is not None and jump_process.poll() is None:
                #     jump_process.terminate()
                #     jump_process = None
                # 
                # # Start new jump process
                # jump_process = subprocess.Popen(["python3", "IK_Sender_Jump.py"])

            # Circle button - Forward
            if (
                button_states[PS3_BUTTON_MAP["CIRCLE"]] == 1
                and prev_button_states[PS3_BUTTON_MAP["CIRCLE"]] == 0
            ):
                print("Circle button pressed - Forward movement")
                try:
                    ik_socket.sendto(b"FORWARD", (IK_SENDER_IP, IK_SENDER_PORT))
                except Exception as e:
                    print(f"Error sending FORWARD command: {e}")

            # Square button - Backward
            if (
                button_states[PS3_BUTTON_MAP["SQUARE"]] == 1
                and prev_button_states[PS3_BUTTON_MAP["SQUARE"]] == 0
            ):
                print("Square button pressed - Backward movement")
                try:
                    ik_socket.sendto(b"BACKWARD", (IK_SENDER_IP, IK_SENDER_PORT))
                except Exception as e:
                    print(f"Error sending BACKWARD command: {e}")

            # Triangle button - Stop the robot after returning to home position
            if (
                button_states[PS3_BUTTON_MAP["TRIANGLE"]] == 1
                and prev_button_states[PS3_BUTTON_MAP["TRIANGLE"]] == 0
            ):
                print(
                    "Triangle button pressed - Stop robot after going to home position"
                )
                try:
                    ik_socket.sendto(b"HOME_AND_STOP", (IK_SENDER_IP, IK_SENDER_PORT))
                except Exception as e:
                    print(f"Error sending HOME_AND_STOP command: {e}")

            # L1 button - Rotate Left
            if (
                button_states[PS3_BUTTON_MAP["L1"]] == 1
                and prev_button_states[PS3_BUTTON_MAP["L1"]] == 0
            ):
                print("L1 button pressed - Rotate Left")
                rotation_cmd = "ROTATE_LEFT:50"  # Default speed of 50
                try:
                    ik_socket.sendto(rotation_cmd.encode(), (IK_SENDER_IP, IK_SENDER_PORT))
                except Exception as e:
                    print(f"Error sending ROTATE_LEFT command: {e}")

            # R1 button - Rotate Right
            if (
                button_states[PS3_BUTTON_MAP["R1"]] == 1
                and prev_button_states[PS3_BUTTON_MAP["R1"]] == 0
            ):
                print("R1 button pressed - Rotate Right")
                rotation_cmd = "ROTATE_RIGHT:50"  # Default speed of 50
                try:
                    ik_socket.sendto(rotation_cmd.encode(), (IK_SENDER_IP, IK_SENDER_PORT))
                except Exception as e:
                    print(f"Error sending ROTATE_RIGHT command: {e}")

            # Update previous button states
            prev_button_states = button_states.copy()

            screen.fill((45, 48, 62))

            title_text = title_font.render(
                "PS3 Controller Tester", True, (255, 255, 255)
            )
            screen.blit(title_text, (WIDTH // 2 - title_text.get_width() // 2, 20))
            
            # Periodic connection check
            current_time = time.time()
            if current_time - last_connection_check > connection_check_interval:
                last_connection_check = current_time
                
                # Try to send a ping to check if IK_Sender_Jorge.py is still running
                if ik_sender_ready:
                    try:
                        # Send a ping command that won't cause any movement
                        ik_socket.sendto(b"PING", (IK_SENDER_IP, IK_SENDER_PORT))
                        print("Sent ping to IK_Sender_Jorge.py")
                    except Exception as e:
                        print(f"Error sending ping: {e}")
                        ik_sender_ready = False
                        connection_status = "CONNECTION LOST - Restart IK_Sender_Jorge.py"
                        connection_color = (255, 0, 0)
            
            # Display connection status
            status_text = label_font.render(
                connection_status, True, connection_color
            )
            screen.blit(status_text, (WIDTH // 2 - status_text.get_width() // 2, 60))

            # ---Joystick Panels---

            panel_width = (WIDTH // 2) - 60
            panel_height = 250

            left_panel_rect = pygame.Rect(40, 140, panel_width, panel_height)
            right_panel_rect = pygame.Rect(
                (WIDTH // 2) + 20, 140, panel_width, panel_height
            )

            left_panel_surf = pygame.Surface(
                (panel_width, panel_height), pygame.SRCALPHA
            )
            left_panel_surf.fill((70, 70, 90, 180))

            right_panel_surf = pygame.Surface(
                (panel_width, panel_height), pygame.SRCALPHA
            )
            right_panel_surf.fill((70, 70, 90, 180))

            left_panel_title = panel_title_font.render(
                "Left Joystick", True, (255, 255, 255)
            )
            right_panel_title = panel_title_font.render(
                "Right Joystick", True, (255, 255, 255)
            )

            left_panel_surf.blit(
                left_panel_title,
                (
                    left_panel_surf.get_width() // 2
                    - left_panel_title.get_width() // 2,
                    10,
                ),
            )

            right_panel_surf.blit(
                right_panel_title,
                (
                    right_panel_surf.get_width() // 2
                    - right_panel_title.get_width() // 2,
                    10,
                ),
            )

            j_center_x = panel_width // 2
            j_center_y = (panel_height // 2) + 20

            # Left joystick
            draw_joystick_graph(
                left_panel_surf, left_x, left_y, j_center_x, j_center_y, radius=60
            )

            # Right joystick
            draw_joystick_graph(
                right_panel_surf, right_x, right_y, j_center_x, j_center_y, radius=60
            )

            # "speed values" for each joystick
            left_speed_x = axis_to_speed(left_x)
            left_speed_y = axis_to_speed(left_y)
            right_speed_x = axis_to_speed(right_x)
            right_speed_y = axis_to_speed(right_y)

            speed_info_left = f"Lx Speed: {left_speed_x}    Ly Speed: {left_speed_y}"
            speed_info_right = f"Rx Speed: {right_speed_x}    Ry Speed: {right_speed_y}"

            text_left = label_font.render(speed_info_left, True, (255, 255, 255))
            text_right = label_font.render(speed_info_right, True, (255, 255, 255))

            left_panel_surf.blit(
                text_left, (j_center_x - text_left.get_width() // 2, j_center_y + 70)
            )
            right_panel_surf.blit(
                text_right, (j_center_x - text_right.get_width() // 2, j_center_y + 70)
            )

            screen.blit(left_panel_surf, (left_panel_rect.x, left_panel_rect.y))
            screen.blit(right_panel_surf, (right_panel_rect.x, right_panel_rect.y))

            # --- Bottom Panels ---

            bottom_panel_height = 200
            bottom_panel_rect = pygame.Rect(40, 400, WIDTH - 80, bottom_panel_height)

            bottom_panel_surf = pygame.Surface(
                (bottom_panel_rect.width, bottom_panel_rect.height), pygame.SRCALPHA
            )
            bottom_panel_surf.fill((70, 70, 90, 180))
            
            # Add rotation indicator
            rotation_text = "Rotation: "
            if abs(right_x) > deadzone:
                rotation_speed = int(abs(right_x) * 100)
                if right_x < 0:
                    rotation_text += f"LEFT (Speed: {rotation_speed})"
                else:
                    rotation_text += f"RIGHT (Speed: {rotation_speed})"
            else:
                rotation_text += "NONE"
                
            rotation_indicator = label_font.render(rotation_text, True, (255, 255, 255))
            bottom_panel_surf.blit(
                rotation_indicator, 
                (bottom_panel_rect.width // 2 - rotation_indicator.get_width() // 2, 20)
            )

            btn_title = panel_title_font.render(
                "Button States (Binary)", True, (255, 255, 255)
            )
            bottom_panel_surf.blit(btn_title, (20, 10))

            # Add button command mapping information
            button_commands = label_font.render(
                "X: Jump | Circle: Forward | Square: Backward | Triangle: Rotate Left",
                True,
                (255, 255, 255),
            )
            bottom_panel_surf.blit(button_commands, (20, 200 - 30))
            
            # Add keyboard controls information
            keyboard_commands = label_font.render(
                "Keyboard: X=Jump | C=Forward | S=Backward | T=Home and Stop",
                True,
                (255, 255, 0),  # Yellow text to stand out
            )
            bottom_panel_surf.blit(keyboard_commands, (20, 200))

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

            triggers_text_L2 = label_font.render(
                f"L2: {l2_pressed}", True, (255, 255, 255)
            )
            triggers_text_R2 = label_font.render(
                f"R2: {r2_pressed}", True, (255, 255, 255)
            )

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

        # Close the sockets
        ik_socket.close()
        ready_socket.close()

        # Quit pygame
        pygame.quit()


if __name__ == "__main__":
    main()
