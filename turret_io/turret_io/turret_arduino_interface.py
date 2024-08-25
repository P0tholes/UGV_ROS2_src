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
from std_msgs.msg._int32 import Int32

ser = Serial('/dev/ttyUSB0', 9600)
def send_turret_data(identifier, value):
        data_to_send = f"{identifier} {value}".encode()
        ser.write(data_to_send)

def calculate_checksum(data):
    return int(sum(data)) & 0xFF
#turret state including laser
#yaw setpoint
#pitch setpoint
data_to_send = [0,0,0]
int_holder_array = [0,0,0]
start_of_message = 's'

def write_characters(input_string):
    for char in input_string:
        ser.write(char.encode())
        print((char.encode()).decode())

def write_to_serial(state, yaw, pitch):
        dataString = b''
        ser.write(b's')
        write_characters(str(state))
        print("state: " + str(state))
        ser.write(b'd')
        print(' ')
        write_characters(str(yaw))
        print("yaw: " + str(yaw))
        ser.write(b'e')
        print(' ')

        write_characters(str(pitch))
        print("pitch: " + str(pitch))
        print(' ')
        ser.write(b'z')





class ArduinoInterfaceNode(Node):
    

    def __init__(self):
        super().__init__("turret_arduino_interface")
        self.subscription_turret_state = self.create_subscription(
            Int32,'turretState',self.callback_turret_state, 10
        )
        


        self.subscription_yaw_setpoint = self.create_subscription(
            Int32,'turretYawSetpoint',self.callback_turret_yaw, 10
        )

        self.subscription_pitch_setpoint = self.create_subscription(
            Int32,'turretPitchSetpoint',self.callback_turret_pitch, 10
        )
        
        time.sleep(2)
    
    def callback_turret_state(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
        int_holder_array[0] = msg.data
        data_to_send = [0,0,0]
        data_to_send[0] = int_holder_array[0]
        data_to_send[1] = int_holder_array[1]
        data_to_send[2] = int_holder_array[2]
        print("STATE CHANGE")
        
        #data_to_send[4] = int(float_holder_array[4])
       
        
        #checksum = calculate_checksum(data_to_send)
        
        write_to_serial(data_to_send[0],data_to_send[1],data_to_send[2])
    


    def callback_turret_pitch(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
        int_holder_array[1] = msg.data
        data_to_send = [0,0,0]
        data_to_send[0] = int_holder_array[0]
        data_to_send[1] = int_holder_array[1]
        data_to_send[2] = int_holder_array[2]
        
        #data_to_send[4] = int(float_holder_array[4])
       
        
        #checksum = calculate_checksum(data_to_send)
        
        write_to_serial(data_to_send[0],data_to_send[1],data_to_send[2])

    def callback_turret_yaw(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        
        int_holder_array[2] = msg.data
        data_to_send = [0,0,0,]
        data_to_send[0] = int_holder_array[0]
        data_to_send[1] = int_holder_array[1]
        data_to_send[2] = int_holder_array[2]
        
        #data_to_send[4] = int(float_holder_array[4])
       
        
        write_to_serial(data_to_send[0],data_to_send[1],data_to_send[2])

    

    
        

        

        
        
             
        
        
        
        

             
        
        




def main(args=None):
    rclpy.init(args=args)

    interface = ArduinoInterfaceNode()

    rclpy.spin(interface)
    ser.close





    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()