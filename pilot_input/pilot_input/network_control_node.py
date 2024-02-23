#!/usr/bin/env python3
import dataclasses
import string

from numpy import float32
import rclpy
from rclpy.node import Node

from flask import Flask, request, jsonify
from array import array
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import os


rclpy.init()
networkControlNode = rclpy.create_node('network_control_node')
throttlePublisher = networkControlNode.create_publisher(Float32,'throttle',10)
twistPublisher = networkControlNode.create_publisher(Float32,'twist',10)

throttle = 0
turn = 0
cam_mode = 0   


def publish_params_to_topic(pub, data):
    pub.publish(data)




app = Flask(__name__)

@app.route('/control_robot', methods=['POST'])
def command_robot():
    data = request.get_json()
    if 'command' in data and len(data['command']) == 3:
        try:
            # Extract three integers from the 'command' key
            global throttle
            global turn
            global cam_mode
            throttle, turn, cam_mode = map(int, data['command'])
            dataChunk = Float32MultiArray
            throttleData = Float32()
            twistData = Float32()
            # Use x, y, z to control the robot
            print(throttle)
            
            print(turn)
            
            print(cam_mode)
            
            #dataChunk.data = [throttle + 127.0,turn + 127.0,0.0,0.0,0.0]
            throttleData.data = throttle + 127.0
            twistData.data = turn + 127.0
            publish_params_to_topic(throttlePublisher, throttleData)
            publish_params_to_topic(twistPublisher, twistData)
            
            # ...

            return {'status': 'success'}
        except ValueError:
            return {'status': 'error', 'message': 'Invalid integer values'}
    else:
        return {'status': 'error', 'message': 'Invalid payload format'}

@app.route('/control_robot', methods=['GET'])
def get_integers():
    # Your implementation here
    global throttle
    global turn
    global cam_mode
    return jsonify([throttle, turn, cam_mode])  # Replace with actual data










        
                
                
               
                
                



def main(args=None):
    #rclpy.init(args=args)

    

    rclpy.spin(networkControlNode)




    networkControlNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)
    main()