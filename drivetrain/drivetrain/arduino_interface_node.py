#!/usr/bin/env python3
from serial import Serial
import time
import struct

from numpy import float32
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from evdev import InputDevice, categorize, ecodes
from std_msgs.msg import Float32MultiArray

ser = Serial('/dev/ttyUSB1', 9600)
def send_motor_data(identifier, speed):
        data_to_send = f"{identifier} {speed}".encode()
        ser.write(data_to_send)

def calculate_checksum(data):
    return int(sum(data)) & 0xFF

data_to_send = [0,0]
float_holder_array = [0.0,0.0,0.0,0.0,0.0]
start_of_message = 's'

def write_characters(input_string):
    for char in input_string:
        ser.write(char.encode())
        print((char.encode()).decode())

def write_to_serial(throttle, twist):
        dataString = b''
        ser.write(b's')
        write_characters(str(throttle))
        ser.write(b'd')
        print(' ')

        write_characters(str(twist))
        print(' ')
        ser.write(b'z')





class ArduinoInterfaceNode(Node):
    

    def __init__(self):
        super().__init__("arduino_interface_node")


        self.subscription_throttle = self.create_subscription(
            Float32,'throttle',self.callback_throttle, 10
        )

        self.subscription_twist = self.create_subscription(
            Float32,'twist',self.callback_twist, 10
        )
        
        time.sleep(2)
    
    
    


    def callback_twist(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
        float_holder_array[0] = msg.data
        data_to_send = [0,0,0,0,0]
        data_to_send[0] = int(float_holder_array[0])
        data_to_send[1] = int(float_holder_array[1])
        data_to_send[2] = int(float_holder_array[2])
        data_to_send[3] = int(float_holder_array[3])
        data_to_send[4] = int(float_holder_array[4])
       
        
        #checksum = calculate_checksum(data_to_send)
        
        write_to_serial(data_to_send[0],data_to_send[1])

    def callback_throttle(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        
        float_holder_array[1] = msg.data
        data_to_send = [0,0,0,0,0]
        data_to_send[0] = int(float_holder_array[0])
        data_to_send[1] = int(float_holder_array[1])
        data_to_send[2] = int(float_holder_array[2])
        data_to_send[3] = int(float_holder_array[3])
        data_to_send[4] = int(float_holder_array[4])
       
        
        write_to_serial(data_to_send[0],data_to_send[1])

    

    
        

        

        
        
             
        
        
        
        

             
        
        




def main(args=None):
    rclpy.init(args=args)

    interface = ArduinoInterfaceNode()

    rclpy.spin(interface)
    ser.close





    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()