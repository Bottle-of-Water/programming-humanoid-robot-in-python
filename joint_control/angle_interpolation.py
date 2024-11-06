'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from keyframes import leftBackToStand
from keyframes import leftBellyToStand
from keyframes import rightBackToStand
from keyframes import rightBellyToStand
from keyframes import wipe_forehead


# Initialize lists to store data points
time_values = []
angle_values = []

start_time = None
current_time = None  #define new values for the time because perception.time begins as soon as simspark as opened

class AngleInterpolationAgent(PIDAgent):

    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        #target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes, i just did this in the keyframes folder
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        names, times, keys = keyframes
        #Define our current time
        global start_time 
        global current_time

        if (start_time == None) or (current_time > 15): #time resets at 15 to replay the animation
            start_time = perception.time 
            current_time = 0
        current_time = perception.time - start_time #to make current_time start at 0
        #print(current_time)#TEST
        
        for i, label in enumerate(names):#label = joint_name 
            joint_time = times[i]
            joint_key = keys[i]
            
              
            # Find the segment in which current_time falls
            for j in range(len(joint_time) - 1):
                t0, t1 = joint_time[j], joint_time[j + 1]
                
                if current_time <= joint_time[0]: # to smooth the motion going to our first key, wasnt sure if this was necessary but thought id add it anyway
                    
                    if(i == 0):
                        print(current_time) #TEST

                    p0 = (0.0, 0.0) #assume our starting angle is 0 and time is 0
                    p1 = (0.25, 0.0) #offset from (0,0)
                    p2 = (joint_time[0] - 0.25, joint_key[0][0]) #offset from first key
                    p3 = (joint_time[0], joint_key[0][0]) #p3 is our first key

                    # Calculate interpolation factor u in [0, 1] for Bezier curve
                    u = current_time / joint_time[0]
                    interpolated_angle = self.bezier_interpolate(p0, p1, p2, p3, u)
 
                    #if(i==0): #and (len(time_values) < 102):   FOR TESTING
                    #    time_values.append(current_time)
                    #    angle_values.append(interpolated_angle)

                    # Set the target joint angle
                    target_joints[label] = interpolated_angle
                    break
                    
                if t0 <= current_time <= t1:
                    angle0, _, handle1 = joint_key[j]
                    angle1, handle2, _ = joint_key[j+ 1]  #we are describing the curve between key[j] so we want to use the handle offset coming out of key[j] and the one going in to key[j+1]
                    if(i == 0):
                        print(current_time) #TEST

                    # Control points for cubic Bézier
                    p0 = (t0, angle0)
                    p3 = (t1, angle1)

                    # Calculate handle control points
                    h1_type, h1_time_offset, h1_angle_offset = handle1
                    h2_type, h2_time_offset, h2_angle_offset = handle2

                    p1 = (t0 + h1_time_offset, angle0 + h1_angle_offset)  #we always add the offsets because they are both positive and negative if you look into the keyframes folder
                    p2 = (t1 + h2_time_offset, angle1 + h2_angle_offset)

                    # Calculate interpolation factor u in [0, 1] for Bézier curve
                    u = (current_time - t0) / (t1 - t0)
                    interpolated_angle = self.bezier_interpolate(p0, p1, p2, p3, u)
                    
                    #TESTING
                    #if(i == 0):
                    #    print("at time t: " + str(current_time) +  " u: " + str(u) + " angle is: " + str(interpolated_angle))
                    #if(i==0): #and (len(time_values) < 102):
                    #    time_values.append(current_time)
                    #    angle_values.append(interpolated_angle)
                    #    #print(angle_values)
                    #if(4.58 <= current_time <= 4.63) and (i==0):
                    #    print("should plot now")
                    #    print(time_values)
                    #    print(angle_values)
                        #self.plotResults(time_values, angle_values)
                    # Set the target joint angle

                    target_joints[label] = interpolated_angle
                    break
        return target_joints


    def bezier_interpolate(self, p0, p1, p2, p3, i):
        # Calculate the cubic Bézier point at parameter i
        # Formula: B(i) = (1 - i)^3 * p0 + 3 * (1 - i)^2 * i * p1 + 3 * (1 - i) * i^2 * p2 + i^3 * p3
        return (
            (1 - i)**3 * p0[1] + 3 * (1 - i)**2 * i * p1[1] +
            3 * (1 - i) * i**2 * p2[1] + i**3 * p3[1]  #we are returning the first index because we are interested in the the angle not the time
        )

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()