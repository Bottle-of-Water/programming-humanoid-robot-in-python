'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity, sin, cos, pi

from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain, I used the joints that were defined in spark_agent.py
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll',]
                       }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle, joint_length):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        s = sin(joint_angle)
        c = cos(joint_angle)
        
        if joint_name in ['LShoulderRoll', 'RShoulderRoll', 'LElbowRoll', 'RElbowRoll', 
                          'LHipRoll', 'RHipRoll', 'LAnkleRoll', 'RAnkleRoll']:
            T = [
                [1, 0, 0, joint_length],
                [0, c, -s, 0],
                [0, s, c, 0],
                [0, 0, 0, 1]
            ]
        elif joint_name in ['LShoulderPitch', 'RShoulderPitch', 'HeadPitch', 
                            'LHipPitch', 'RHipPitch', 'LKneePitch', 'RKneePitch', 
                            'LAnklePitch', 'RAnklePitch']:
            T = [
                [c, 0, s, joint_length],
                [0, 1, 0, 0],
                [-s, 0, c, 0],
                [0, 0, 0, 1]
            ]
        elif joint_name in ['HeadYaw', 'LElbowYaw', 'RElbowYaw']:
            T = [
                [c, -s, 0, joint_length],
                [s, c, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ]

        return T

    def addTorsoOffsets (self, T, chain_joints):
        if chain_joints == 'Head':
            T = T + [
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0.1265],
                    [0, 0, 0, 0]
                    ]
        elif chain_joints == 'LArm':
            T = T + [
                    [0, 0, 0, 0],
                    [0, 0, 0, (0.098+0.015)], #+0.015 for the elbow offst
                    [0, 0, 0, 0.10],
                    [0, 0, 0, 0]
                    ]
        elif chain_joints == 'RArm':
            T = T + [
                    [0, 0, 0, 0],
                    [0, 0, 0, -(0.098+0.015)],
                    [0, 0, 0, 0.10],
                    [0, 0, 0, 0]
                    ]
        elif chain_joints == 'LLeg':
            T = T + [
                    [0, 0, 0, 0],
                    [0, 0, 0, 0.05],
                    [0, 0, 0, -0.085],
                    [0, 0, 0, 0]
                    ]
        elif chain_joints == 'RLeg':
            T = T + [
                    [0, 0, 0, 0],
                    [0, 0, 0, -0.05],
                    [0, 0, 0, -0.085],
                    [0, 0, 0, 0]
                    ]
        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        joint_lengths = {'HeadYaw': 0, 'HeadPitch': 0, 
                        'LShoulderPitch': 0, 'LShoulderRoll': 0.105, 'LElbowYaw':  0, 'LElbowRoll': 0.05595,
                        'RShoulderPitch': 0, 'RShoulderRoll': 0.105, 'RElbowYaw': 0, 'RElbowRoll': 0.05595,
                        'LHipYawPitch': 0, 'LHipRoll': 0, 'LHipPitch': 0.1, 'LKneePitch': 0.1029, 'LAnklePitch': 0, 'LAnkleRoll': 0,
                        'RHipYawPitch': 0, 'RHipRoll': 0, 'RHipPitch': 0.1, 'RKneePitch': 0.1029, 'RAnklePitch': 0, 'RAnkleRoll': 0
                       }
        #To keep track of which chain we are at
        chain_keys = list(self.chains.keys())
        i = 0

        for chain_joints in self.chains.values():
            T = identity(4)
            T = self.addTorsoOffsets(T, chain_keys[i])
            i += 1

            for joint in chain_joints:

                angle = joints[joint] 
                joint_length = joint_lengths[joint] 
                
                Tl = self.local_trans(joint, angle, joint_length) #local transformation
                
                T = T * Tl #multiply all the the transformations to get the end effector
                if joint == 'RElbowRoll':
                    print(T)
                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
