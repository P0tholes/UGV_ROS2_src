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
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import os


rclpy.init()
networkControlNode = rclpy.create_node('network_control_node')
throttlePublisher = networkControlNode.create_publisher(Float32,'throttle',10)
twistPublisher = networkControlNode.create_publisher(Float32,'twist',10)
turretStatePublisher = networkControlNode.create_publisher(Int32, "turretState", 10)
#laserStatePublisher = networkControlNode.create_publisher(Bool, "laserState", 10)
yawSetpointPublisher = networkControlNode.create_publisher(Int32, "turretYawSetpoint", 10)
pitchSetpointPublisher = networkControlNode.create_publisher(Int32, "turretPitchSetpoint", 10)
lockModePublisher = networkControlNode.create_publisher(Int32, "lockModeSetpoint", 10)
cycleLockPublisher = networkControlNode.create_publisher(Bool, "cycleLock", 10)
yawSlewRatePublisher = networkControlNode.create_publisher(Int32, "turretYawSlewRate", 10)
pitchSlewRatePublisher = networkControlNode.create_publisher(Int32, "turretPitchSlewRate", 10)
throttle = 0
turn = 0
cam_mode = 0   
turretState = False
laserState = False
yawSetpoint = 0
pitchSetpoint = 0
lockMode = 0
cycleLock = False
slewRateYaw = 0
slewRatePitch = 0
previousYawSetpoint = 0
previousPitchSetpoint = 0
previousYawSetpointForSlew = 0
previousPitchSetpointForSlew = 0

yawSetpointForSlew = 0.0
pitchSetpointForSlew = 0.0

def publish_params_to_topic(pub, data):
    pub.publish(data)

def turret_state(turretState, laserState):
    if(turretState == 0 and laserState ==0):
        return 0
    elif (turretState == 0 and laserState == 1):
        return 1
    elif (turretState == 1 and laserState == 0):
        return 2
    elif (turretState == 1 and laserState == 1):
        return 3
    else: return 0

def normaliseAsDegrees(value):
    return int((value / 255)*360)

def avoid_value_down(input, value):
    if (input == value):
        return input - 1
    else:
        return input

def clamp(value, min, max):
    if (value > max):
        return max
    elif (value < min):
        return min
    else:
        return value

def degreesTo8bit(input):
    return int(input*255/360)

    
gearing_multiplier = 1.5625
    




app = Flask(__name__)

@app.route('/control_robot', methods=['POST'])
def command_robot():
    #print("I do b tryin")
    data = request.get_json()
    if 'command' in data and len(data['command']) == 11:
        try:
            #print("commands received!")
            # Extract several integers from the 'command' key
            global throttle
            global turn
            global cam_mode
            global turretState
            global laserState
            global yawSetpoint
            global pitchSetpoint
            global lockMode
            global cycleLock
            global slewRateYaw
            global slewRatePitch
            global yawSetpointForSlew
            global pitchSetpointForSlew

            global previousYawSetpoint
            global previousPitchSetpoint
            global previousYawSetpointForSlew
            global previousPitchSetpointForSlew


            throttle,turn,cam_mode,turretState,laserState,yawSetpoint,pitchSetpoint,lockMode,cycleLock,slewRateYaw,slewRatePitch = map(int, data['command'])
            dataChunk = Float32MultiArray

            throttleData = Float32()
            twistData = Float32()
            turretStateData = Int32()
            laserStateData = Bool()
            yawSetpointData = Int32()
            pitchSetpointData = Int32()
            lockModeData = Int32()
            cycleLockData = Bool()
            yawSlewRateData = Int32()
            pitchSlewRateData = Int32()

            # Use x, y, z to control the robot
            #print(slewRateYaw)
            
            print("yaw setpoint 8 bit",avoid_value_down(degreesTo8bit(yawSetpoint),255))
            print("pitch setpoint",pitchSetpoint)
            
            #print("trt st: " + str(turretState))
            #print("lsr st: " + str(laserState))

            yawSetpointForSlew = clamp(((slewRateYaw-127)/1000.0) + previousYawSetpointForSlew, 0.0, 255.0)
            pitchSetpointForSlew = clamp(((slewRatePitch-127)/1000.0) + previousPitchSetpointForSlew, 0.0, 80.0)
            #dataChunk.data = [throttle + 127.0,turn + 127.0,0.0,0.0,0.0]
            throttleData.data = throttle + 127.0
            twistData.data = turn + 127.0
            turretStateData.data = turret_state(turretState, laserState)
            #print("Combined State: " + str(turret_state(turretState, laserState)))
            #laserStateData.data = laserState
            '''
            if (yawSetpoint == previousYawSetpoint):
                yawSetpointData.data = normaliseAsDegrees(int(yawSetpointForSlew))
                previousYawSetpoint = yawSetpoint
                previousYawSetpointForSlew = yawSetpointForSlew
                print("yaw slew rate: " + str(slewRateYaw))
                print("yaw setpoint for slew: " + str(yawSetpointForSlew))
                print("delta yaw: " + str((slewRateYaw-127)/1000.0))
            else:
                yawSetpointData.data = normaliseAsDegrees(clamp(yawSetpoint,0,255))

            if (pitchSetpoint == previousPitchSetpoint):
                pitchSetpointData.data = normaliseAsDegrees(int(pitchSetpointForSlew))
                previousPitchSetpoint = pitchSetpoint
                previousPitchSetpointForSlew = pitchSetpointForSlew
                print("pitch slew rate: " + str(slewRatePitch))
                print("pitch setpoint for slew: " + str(pitchSetpointForSlew))
                print("delta pitch: " + str((slewRatePitch-127)/1000.0))
            else:
                pitchSetpointData.data = normaliseAsDegrees(clamp(pitchSetpoint,0,30))
            '''
            #yawSetpointData.data = normaliseAsDegrees(clamp(yawSetpoint,0,255))
            #pitchSetpointData.data = normaliseAsDegrees(clamp(pitchSetpoint,0,90))
            #print("YAW SP:" + str(yawSetpoint))
            #print("PITCH SP:" + str(pitchSetpoint))
            yawSetpointData.data = avoid_value_down(degreesTo8bit(yawSetpoint),255)
            pitchSetpointData.data = pitchSetpoint
            lockModeData.data = lockMode
            cycleLockData.data = bool(cycleLock)
            yawSlewRateData.data = slewRateYaw
            pitchSlewRateData.data = slewRatePitch




            publish_params_to_topic(throttlePublisher, throttleData)
            publish_params_to_topic(twistPublisher, twistData)
            publish_params_to_topic(turretStatePublisher, turretStateData)
            #publish_params_to_topic(laserStatePublisher, laserStateData)
            publish_params_to_topic(yawSetpointPublisher, yawSetpointData)
            publish_params_to_topic(pitchSetpointPublisher, pitchSetpointData)
            publish_params_to_topic(lockModePublisher, lockModeData)
            publish_params_to_topic(cycleLockPublisher, cycleLockData)
            publish_params_to_topic(yawSlewRatePublisher, yawSlewRateData)
            publish_params_to_topic(pitchSlewRatePublisher,pitchSlewRateData)
            
            # ...

            return {'status': 'success'}
        except ValueError:
            print("value error tho")
            return {'status': 'error', 'message': 'Invalid integer values'}
    else:
        print("payload format error tho")
        return {'status': 'error', 'message': 'Invalid payload format'}

@app.route('/control_robot', methods=['GET'])
def get_integers():
    # Your implementation here
    global throttle
    global turn
    global cam_mode
    global turretState
    return jsonify([throttle, turn, cam_mode])  # Replace with actual data










        
                
                
               
                
                



def main(args=None):
    #rclpy.init(args=args)

    

    rclpy.spin(networkControlNode)




    networkControlNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)
    main()