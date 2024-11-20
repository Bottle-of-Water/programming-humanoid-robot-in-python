'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from kinematics.inverse_kinematics import InverseKinematicsAgent
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler

from software_installation.spark_agent import SparkAgent
from joint_control.angle_interpolation import AngleInterpolationAgent
from joint_control.pid import PIDAgent
from joint_control.recognize_posture import PostureRecognitionAgent


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE

    def __init__(self, host="localhost", port=9000):
        self.host = host
        self.port = port
        self.server = SimpleXMLRPCServer((self.host, self.port))
        self.server.register_instance(self)

    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        angle = self.perception.joint[joint_name] #SparkAgent
        return angle 
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle #PidAgent 
        return "success"

    def get_posture(self):
        '''return current posture of robot''' 
        # YOUR CODE HERE
        return (self.posture) # PostureRecognitionAgent

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE    
        self.keyframes = keyframes  #AngleInterpolationAgent 

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return (self.transforms[name]) #kinematics 

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        #related to reverse kinematics, which I didn't manage to implement. 
        # YOUR CODE HERE

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

