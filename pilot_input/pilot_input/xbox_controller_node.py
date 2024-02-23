#!/usr/bin/env python3
import string

from numpy import float32
import rclpy
from rclpy.node import Node
import evdev
from evdev import InputDevice, categorize, ecodes
from std_msgs.msg import Float32MultiArray
import os


controller = evdev.InputDevice('/dev/input/event5')
inputpath = '/dev/input/'

CENTER_TOLERANCE = 350
STICK_MAX = 65536
TRIG_MAX = 1023




center = {
    'ls_x': STICK_MAX/2,
    'ls_y': STICK_MAX/2,
    'rs_x': STICK_MAX/2,
    'rs_y': STICK_MAX/2
}

last = {
    'ls_x': STICK_MAX/2,
    'ls_y': STICK_MAX/2,
    'rs_x': STICK_MAX/2,
    'rs_y': STICK_MAX/2
}

#for evdev.InputDevice in os.listdir(inputpath):
#    if 
def map_value(value, from_low, from_high, to_low, to_high):
    return (value - from_low) * ((to_high - to_low) / (from_high - from_low)) + to_low



class XboxControllerNode(Node):

    def __init__(self):
        super().__init__("xbcontroller_node")
        self.publisher_ = self.create_publisher(Float32MultiArray,'user_input',10)
        self.get_logger().info("hi")
        #print(controller)
        z_mapped = 0.0
        rz_mapped = 0.0
        x_mapped = 0.0
        y_mapped = 0.0
        rx_mapped = 0.0
        ry_mapped = 0.0
        throttle = 0.0
        command_array = Float32MultiArray()
        command_array.data = [throttle,x_mapped,y_mapped,rx_mapped,ry_mapped]#throttle,x,y,rx,ry



        for event in controller.read_loop():
            if event.type == evdev.ecodes.EV_ABS:
                
                #I KNOW THIS CODE IS GARBAG I WILL FIX LATER
                
                if event.code == evdev.ecodes.ABS_X:
                    x_mapped = map_value(event.value, -STICK_MAX/2, STICK_MAX/2,0,255)
                    #print(f"X-axis: {x_mapped}")
                    command_array.data = [throttle,x_mapped,y_mapped,rx_mapped,ry_mapped]
                    self.publisher_.publish(command_array)

                elif event.code == evdev.ecodes.ABS_Y:
                    y_mapped = map_value(event.value, -STICK_MAX/2, STICK_MAX/2,0,255)
                    #print(f"Y-axis: {y_mapped}")
                    command_array.data = [throttle,x_mapped,y_mapped,rx_mapped,ry_mapped]
                    self.publisher_.publish(command_array)

                elif event.code == evdev.ecodes.ABS_Z:
                    z_mapped = map_value(event.value, 0, TRIG_MAX,0,128)
                    
                    throttle = rz_mapped-z_mapped
                    command_array.data = [throttle,x_mapped,y_mapped,rx_mapped,ry_mapped]
                    self.publisher_.publish(command_array)
                
                elif event.code == evdev.ecodes.ABS_RZ:
                    rz_mapped = map_value(event.value, 0, TRIG_MAX,128,255)
                    #print(f"RZ-axis: {rz_mapped}")
                    #print(f"Throttle: {rz_mapped-z_mapped}")
                    throttle = rz_mapped-z_mapped
                    command_array.data = [throttle,x_mapped,y_mapped,rx_mapped,ry_mapped]
                    self.publisher_.publish(command_array)
                
                
               
                
                



def main(args=None):
    rclpy.init(args=args)

    node = XboxControllerNode()

    rclpy.spin(node)




    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()