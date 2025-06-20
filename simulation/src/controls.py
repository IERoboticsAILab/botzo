# controls.py
import pybullet as p

def is_reset_key_pressed():
    """
    Check if the "R" key is pressed.
    Returns:
        bool: True if "R" is pressed, False otherwise.
    """
    keys = p.getKeyboardEvents()
    # ASCII code for "R" is 114 in lowercase and 82 in uppercase
    return keys.get(ord("R")) == p.KEY_IS_DOWN or keys.get(ord("r")) == p.KEY_IS_DOWN

def is_left_key_pressed():
    """
    Check if the left arrow key or "A" is pressed.
    Returns:
        bool: True if left arrow key or "A" is pressed, False otherwise.
    """
    keys = p.getKeyboardEvents()
    return (keys.get(p.B3G_LEFT_ARROW) == p.KEY_IS_DOWN) or (keys.get(ord("A")) == p.KEY_IS_DOWN or keys.get(ord("a")) == p.KEY_IS_DOWN)

def is_right_key_pressed():
    """
    Check if the right arrow key or "D" is pressed.
    Returns:
        bool: True if right arrow key or "D" is pressed, False otherwise.
    """
    keys = p.getKeyboardEvents()
    return (keys.get(p.B3G_RIGHT_ARROW) == p.KEY_IS_DOWN) or (keys.get(ord("D")) == p.KEY_IS_DOWN or keys.get(ord("d")) == p.KEY_IS_DOWN)