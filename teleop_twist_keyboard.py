#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

# we control /equilibrium_pose, which is a pose
# not a velocity
# from geometry_msgs.msg import Twist
# from geometry_msgs.msg import TwistStamped

from geometry_msgs.msg import Pose,PoseStamped,Quaternion


import tf2_ros, tf.transformations as trans

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

PoseMsg = Pose

BASEFRAME = "panda_link0"
EEFRAME   = "panda_EE"

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   X +/- : i/k
   Y +/- : j/l
   Z +/- : t/b

   Rx +/- : c/v
   Ry +/- : n/m
   Rz +/- : u/o

q/z : increase/decrease linear offset by 10%
w/x : increase/decrease angular offset by 10%

CTRL-C to quit (does not work)
"""

moveBindings = {
        # move along world x axis
        'i':(1,0,0,0,0,0),
        'k':(-1,0,0,0,0,0),
        # move along world y axis
        'j':(0,1,0,0,0,0),
        'l':(0,-1,0,0,0,0),
        # move along world z axis
        't':(0,0,1,0,0,0),
        'b':(0,0,-1,0,0,0),
        # rotate along EE x axis
        'c':(0,0,0,1,0,0),
        'v':(0,0,0,-1,0,0),
        # rotate along EE y axis
        'n':(0,0,0,0,1,0),
        'm':(0,0,0,0,-1,0),
        # rotate along EE z axis
        'u':(0,0,0,0,0,1),
        'o':(0,0,0,0,0,-1),
    }

offsetBindings={
        'q':(1.1,1),
        'z':(.9,1),
        'w':(1,1.1),
        'x':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()

        # get the topic
        self.topic = rospy.get_param("out_topic", 
                     default="/cartesian_impedance_example_controller/equilibrium_pose")

        self.publisher = rospy.Publisher(self.topic, 
                                         PoseMsg, 
                                         queue_size = 1, # discard all but the last msg
                                         )
        
        # get the starting pose
        # http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        
        msg = tfBuffer.lookup_transform(BASEFRAME, EEFRAME, 
                                        rospy.Time(0), 
                                        # TODO 
                                        # timeout less than 10 causes ExtrapolationError
                                        timeout=rospy.Duration(10)) 

        pose = msg.transform
        print(pose)
        # pose.translation ...
        # pose.rotation ...

        self.x = pose.translation.x 
        self.y = pose.translation.y
        self.z = pose.translation.z
        self.Q = [pose.rotation.x, 
                  pose.rotation.y, 
                  pose.rotation.z, 
                  pose.rotation.w]
        
        


        self.linear_offset = 0.0
        self.angular_offset = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, rx, ry, rz, lin_offset, ang_offset):
        self.condition.acquire()
        self.linear_offset = lin_offset
        self.angular_offset = ang_offset

        # define increments given key inputs
        self.x += x*lin_offset
        self.y += y*lin_offset
        self.z += z*lin_offset
        
        self.Q = trans.quaternion_multiply(self.Q, 
                                           trans.quaternion_from_euler(rx*ang_offset, ry*ang_offset, rz*ang_offset))

        #

        # Notify publish thread that we have a new message.

        

        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        # no need to send any command
        # self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        pose_msg = PoseMsg()

        if stamped:
            pose = pose_msg.pose
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = pose_frame
        else:
            pose = pose_msg
        while not self.done:
            if stamped:
                pose_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

           
            pose.position.x = self.x 
            pose.position.y = self.y 
            pose.position.z = self.z            
            pose.orientation.w = self.Q[3]
            pose.orientation.x = self.Q[0]
            pose.orientation.y = self.Q[1]
            pose.orientation.z = self.Q[2]
            # 
            self.condition.release()

            # Publish.
            self.publisher.publish(pose_msg)


def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(lin_offset, ang_offset):
    return "currently:\tlin_offset %s\tang_offset %s " % (lin_offset,ang_offset)

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('teleop_twist_keyboard')

    lin_offset = rospy.get_param("~lin_offset", 0.1)
    ang_offset = rospy.get_param("~ang_offset", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    stamped = rospy.get_param("~stamped", True)
    pose_frame = rospy.get_param("~frame_id", '')
    if stamped:
        PoseMsg = PoseStamped

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    rx = 0
    ry = 0
    rz = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, rx ,ry, rz, lin_offset, ang_offset)

        print(msg)
        print(vels(lin_offset,ang_offset))
        while(1):
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                rx = moveBindings[key][3]
                ry = moveBindings[key][4]
                rz = moveBindings[key][5]
            elif key in offsetBindings.keys():
                lin_offset = pub_thread.linear_offset*offsetBindings[key][0]
                ang_offset = pub_thread.angular_offset*offsetBindings[key][1]
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and rx == 0 and ry == 0 and rz == 0:
                    continue
                x = 0
                y = 0
                z = 0
                rx = 0
                ry = 0
                rz = 0
            if (key == '\x03'):
                pub_thread.stop()
                break
            

            pub_thread.update(x, y, z, rx ,ry, rz, lin_offset, ang_offset)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)
