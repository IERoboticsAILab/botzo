#!/usr/bin/env python3

'''
Left lever for forward backward (up-down)
Left lever for moving left and right (left-rught)
right lever for rotationg left or right while standing in the same place (left-right)
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
 
 
# Tune these to taste
LINEAR_SCALE  = 1.0   # max m/s sent on linear.x / linear.y
ANGULAR_SCALE = 1.0   # max rad/s sent on angular.z
DEADZONE      = 0.05  # ignore stick noise below this threshold
 
 
def apply_deadzone(value: float, threshold: float) -> float:
    return value if abs(value) >= threshold else 0.0
 
 
class JoyToCmdVel(Node):
    def __init__(self):
        super().__init__('joy_to_cmd_vel')
 
        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
 
        self.get_logger().info('joy_to_cmd_vel ready — L-stick: translate | R-stick LR: rotate')
 
    def joy_callback(self, msg: Joy):
        axes = msg.axes
 
        # Safety: guard against controllers with fewer axes
        lx = axes[0] if len(axes) > 0 else 0.0   # L lever left(+) / right(-)
        ly = axes[1] if len(axes) > 1 else 0.0   # L lever fwd(+)  / back(-)
        rx = axes[3] if len(axes) > 3 else 0.0   # R lever left(+) / right(-)
 
        # Deadzone
        lx = apply_deadzone(lx, DEADZONE)
        ly = apply_deadzone(ly, DEADZONE)
        rx = apply_deadzone(rx, DEADZONE)
 
        twist = Twist()
        twist.linear.x  =  ly * LINEAR_SCALE   # forward / backward
        twist.linear.y  =  lx * LINEAR_SCALE   # strafe left / right
        twist.angular.z =  rx * ANGULAR_SCALE  # rotate left / right in place
 
        self.pub.publish(twist)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
 