import numpy as np
import time
import threading

# Define standard positions and movement sequences
HOME_POSITION = [[0,2,16]]  # [x, y, z]
#SLEEP_POSITION = [[0,2,13],[0,2,10],[0,2,8]] # Lower height for sleep mod
SLEEP_POSITION = [[0,2,13]]

# Movement Patterns
# FORWARD movement coordinates
FORWARD_TARGETS_FR_BL = [
    [2,2,16],[1.5,2,15],[1,2,14],[0,2,13],[-1,2,14],[-1.5,2,15],[-2,2,16],[-1,2,16],[0,2,16],
    [1,2,16]
]

FORWARD_TARGETS_FL_BR = [
    [-2,2,16],[-1,2,16],[0,2,16],
    [1,2,16], [2,2,16],[1.5,2,15],[1,2,14],[0,2,13],[-1,2,14],[-1.5,2,15]
]

# BACKWARD movement coordinates
BACKWARD_TARGETS_FR_BL = [
    [2,2,16],[1,2,16],[0,2,16],
    [-1,2,16],[-2,2,16],[-1.5,2,15],[-1,2,14],
    [0,2,13],[1,2,14],[1.5,2,15]
]

BACKWARD_TARGETS_FL_BR = [
    [-2,2,16],[-1.5,2,15],[-1,2,14],
    [0,2,13],[1,2,14],[1.5,2,15],
    [2,2,16],[1,2,16],[0,2,16],
    [-1,2,16]
]

# SPIN LEFT movement coordinates
SPIN_LEFT_TARGETS_FR_BL = [
    [0,4,16],[0,3.5,15],[0,3,14],[0,2,13],
    [0,1,14],[0,0.5,15],[0,0,16],
    [0,1,16],[0,2,16],[0,3,16]
]

SPIN_LEFT_TARGETS_FL_BR = [
    [0,0,16],
    [0,1,16],[0,2,16],[0,3,16],
    [0,4,16],[0,3.5,15],[0,3,14],[0,2,13],
    [0,1,14],[0,0.5,15]
]

# SPIN RIGHT movement coordinates (reverse of spin left)
SPIN_RIGHT_TARGETS_FR_BL = [
    [0,4,16],
    [0,3,16], [0,2,16], [0,1,16],
    [0,0,16], [0,0.5,15], [0,1,14], [0,2,13],
    [0,3,14], [0,3.5,15]
]

SPIN_RIGHT_TARGETS_FL_BR = [
    [0,0,16], [0,0.5,15], [0,1,14], [0,2,13],
    [0,3,14], [0,3.5,15], [0,4,16],
    [0,3,16], [0,2,16], [0,1,16]
]

# JUMP movement sequence
JUMP_TARGETS = [
    # Crouch down
    [[0,3,12], [0,3,12], [0,3,12], [0,3,12]],
    # Jump up
    [[0,3,18], [0,3,18], [0,3,18], [0,3,18]],
    # Land and recover
    [[0,3,14], [0,3,14], [0,3,14], [0,3,14]],
    [[0,3,15], [0,3,15], [0,3,15], [0,3,15]]
]

class SkillExecutor:
    """
    Class to manage execution of robot skills
    """
    def __init__(self, move_legs_func=None):
        self.movement_in_progress = False
        self.current_skill = None
        self.stop_requested = False
        self.move_legs_func = move_legs_func
        self.movement_thread = None
    
    def set_move_legs_function(self, func):
        """Set the function that will be used to move the legs"""
        self.move_legs_func = func
    
    def go_home(self):
        """Move robot to home position"""
        if self.move_legs_func:
            # All legs to home position
            self.move_legs_func(HOME_POSITION * 4)
            return True
        return False
    
    def go_sleep(self):
        """Move robot to sleep position"""
        if not self.move_legs_func:
            return False
        # All legs to sleep position
        #for position in SLEEP_POSITION:
         #   self.move_legs_func(position * 4)
          #  time.sleep(0.2) 
        self.move_legs_func(SLEEP_POSITION * 4)
        return True
    
    def jump(self):
        """Execute a jump sequence"""
        if not self.move_legs_func:
            return False
            
        # Execute each step of the jump sequence
        for position in JUMP_TARGETS:
            self.move_legs_func(position)
            time.sleep(0.2)  # Pause between jump phases
        
        return True
    
    def start_continuous_movement(self, movement_type):
        """
        Start a continuous movement of the specified type
        movement_type: 'forward', 'backward', 'spin_left', 'spin_right'
        """
        if self.movement_in_progress:
            self.stop_continuous_movement()
        
        self.current_skill = movement_type
        self.stop_requested = False
        self.movement_in_progress = True
        
        # Start movement in a separate thread
        self.movement_thread = threading.Thread(
            target=self._execute_continuous_movement, 
            args=(movement_type,), 
            daemon=True
        )
        self.movement_thread.start()
        return True
    
    def stop_continuous_movement(self):
        """Stop any ongoing continuous movement"""
        self.stop_requested = True
        self.current_skill = None
        
        # Wait for movement thread to end
        if self.movement_thread and self.movement_thread.is_alive():
            self.movement_thread.join(timeout=1.0)
        
        self.movement_in_progress = False
        return True
    
    def _execute_continuous_movement(self, movement_type):
        """Internal method to execute continuous movement sequences"""
        if not self.move_legs_func:
            self.movement_in_progress = False
            return
        
        # Select the appropriate targets based on movement type
        if movement_type == 'forward':
            targets_FR_BL = FORWARD_TARGETS_FR_BL
            targets_FL_BR = FORWARD_TARGETS_FL_BR
        elif movement_type == 'backward':
            targets_FR_BL = BACKWARD_TARGETS_FR_BL
            targets_FL_BR = BACKWARD_TARGETS_FL_BR
        elif movement_type == 'spin_left':
            targets_FR_BL = SPIN_LEFT_TARGETS_FR_BL
            targets_FL_BR = SPIN_LEFT_TARGETS_FL_BR
        elif movement_type == 'spin_right':
            targets_FR_BL = SPIN_RIGHT_TARGETS_FR_BL
            targets_FL_BR = SPIN_RIGHT_TARGETS_FL_BR
        else:
            self.movement_in_progress = False
            return
        
        # Execute continuous movement until stopped
        index = 0
        sequence_length = min(len(targets_FR_BL), len(targets_FL_BR))
        
        while not self.stop_requested:
            if index >= sequence_length:
                index = 0  # Loop the sequence
            
            # Get current targets for each leg
            FR_target = targets_FR_BL[index]
            FL_target = targets_FL_BR[index]
            BL_target = FR_target  # Use same target for diagonal legs
            BR_target = FL_target
            
            # Move legs to target positions
            self.move_legs_func([FR_target, FL_target, BR_target, BL_target])
            
            # Increment index for next position
            index += 1
            
            # Pause between steps
            time.sleep(0.05)
        
        # When stopped, return to home position
        self.go_home()
        self.movement_in_progress = False

# Create a default skill executor instance
skill_executor = SkillExecutor() 