#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2020-2021 Johns Hopkins University (JHU), Worcester Polytechnic Institute (WPI) All Rights Reserved.


#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.


#     \author    <amunawar@jhu.edu>
#     \author    Adnan Munawar
#     \version   1.0
# */
# //==============================================================================

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from PyKDL import Frame, Rotation, Vector
import rospy
import math
import numpy as np
from enum import Enum
import time
from std_msgs.msg import Empty
from tf_conversions import posemath

class ArmType(Enum):
    PSM1=1
    PSM2=2
    ECM=3

def frame_to_pose_stamped_msg(frame):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = frame.p[0]
    msg.pose.position.y = frame.p[1]
    msg.pose.position.z = frame.p[2]

    msg.pose.orientation.x = frame.M.GetQuaternion()[0]
    msg.pose.orientation.y = frame.M.GetQuaternion()[1]
    msg.pose.orientation.z = frame.M.GetQuaternion()[2]
    msg.pose.orientation.w = frame.M.GetQuaternion()[3]

    return msg


def list_to_sensor_msg_position(jp_list):
    msg = JointState()
    msg.position = jp_list
    return msg


class ARMInterface:
    def __init__(self, arm_type):
        if arm_type == ArmType.PSM1:
            arm_name = '/CRTK/psm1'
        elif arm_type == ArmType.PSM2:
            arm_name = '/CRTK/psm2'
        elif arm_type == ArmType.ECM:
            arm_name = '/CRTK/ecm'
        else:
            raise ("Error! Invalid Arm Type")

        self._cp_sub = rospy.Subscriber(arm_name + "/measured_cp", PoseStamped, self.cp_cb, queue_size=1)
        self._T_b_w_sub = rospy.Subscriber(arm_name + "/T_b_w", PoseStamped, self.T_b_w_cb, queue_size=1)
	self._T_w_b_sub = rospy.Subscriber(arm_name + "/T_w_b", PoseStamped, self.T_w_b_cb, queue_size=1)
        self._jp_sub = rospy.Subscriber(arm_name + "/measured_cp", JointState, self.jp_cb, queue_size=1)
        self.cp_pub = rospy.Publisher(arm_name + "/servo_cp", PoseStamped, queue_size=1)
        self.jp_pub = rospy.Publisher(arm_name + "/servo_jp", JointState, queue_size=1)
        self.jaw_jp_pub = rospy.Publisher(arm_name + '/jaw/' + 'servo_jp', JointState, queue_size=1)
	self.grasp_pub = rospy.Publisher(arm_name + "/run_grasp_logic", PoseStamped, queue_size=1)

        self.measured_cp_msg = None
	self.T_w_b_msg = None
        self.T_b_w_msg = None
        self.measured_jp_msg = None

    def cp_cb(self, msg):
        self.measured_cp_msg = msg

    def T_b_w_cb(self, msg):
        self.T_b_w_msg = msg

    def T_w_b_cb(self, msg):
        self.T_w_b_msg = msg

    def jp_cb(self, msg):
        self.measured_jp_msg = msg

    def measured_cp(self):
        return self.measured_cp_msg

    def get_T_b_w(self):
        return self.T_b_w_msg

    def get_T_w_b(self):
        return self.T_w_b_msg

    def measured_jp(self):
        return self.measured_jp_msg

    def servo_cp(self, pose):
        if type(pose) == Frame:
            msg = frame_to_pose_stamped_msg(pose)
        else:
            msg = pose
        self.cp_pub.publish(msg)

    def grasp_needle(self, pose):
	if type(pose) == Frame:
            msg = frame_to_pose_stamped_msg(pose)
        else:
            msg = pose
        self.grasp_pub.publish(msg)

    def servo_jp(self, jp):
        if type(jp) == list:
            msg = list_to_sensor_msg_position(jp)
        else:
            msg = jp
        self.jp_pub.publish(msg)

    def set_jaw_angle(self, val):
        msg = list_to_sensor_msg_position([val])
        self.jaw_jp_pub.publish(msg)

class SceneObjectType(Enum):
    Needle=1
    Entry1=2
    Entry2=3
    Entry3=4
    Entry4=5
    Exit1=6
    Exit2=7
    Exit3=8
    Exit4=9

class SceneInterface:
    def __init__(self):
        self._scene_object_poses = dict()
        self._scene_object_poses[SceneObjectType.Needle] = None
        self._scene_object_poses[SceneObjectType.Entry1] = None
        self._scene_object_poses[SceneObjectType.Entry2] = None
        self._scene_object_poses[SceneObjectType.Entry3] = None
        self._scene_object_poses[SceneObjectType.Entry4] = None
        self._scene_object_poses[SceneObjectType.Exit1] = None
        self._scene_object_poses[SceneObjectType.Exit2] = None
        self._scene_object_poses[SceneObjectType.Exit3] = None
        self._scene_object_poses[SceneObjectType.Exit4] = None
        self._subs = []

        namespace = '/CRTK/'
        suffix = '/measured_cp'
        for k, i in self._scene_object_poses.items():
            self._subs.append(rospy.Subscriber(namespace + k.name + suffix, PoseStamped,
                                               self.state_cb, callback_args=k, queue_size=1))

        self._task_3_ready = False
        self._task_3_setup_init_pub = rospy.Publisher('/CRTK/scene/task_3_setup/init', Empty, queue_size=1)

        self._task_3_setup_ready_sub = rospy.Subscriber('/CRTK/scene/task_3_setup/ready',
                                                        Empty, self.task_3_setup_ready_cb, queue_size=1)

    def state_cb(self, msg, key):
        self._scene_object_poses[key] = msg

    def measured_cp(self, object_type):
        return self._scene_object_poses[object_type]

    def task_3_setup_ready_cb(self, msg):
        self._task_3_ready = True

    def task_3_setup_init(self):
        self._task_3_ready = False
        self._task_3_setup_init_pub.publish(Empty())
        while not self._task_3_ready:
            time.sleep(0.1)


rospy.init_node("sur_chal_crtk_test")
rate = rospy.Rate(50)
time.sleep(0.5)

# Get a handle to PSM2
psm2 = ARMInterface(ArmType.PSM2)
scene = SceneInterface()
time.sleep(3.0)


#################### Move PSM2 to Needle and Grab it ############################
# Get the transformation matrix from world to base
trans_psm2 = psm2.get_T_w_b()
trans_frame_psm2 = posemath.fromMsg(trans_psm2.pose)
	
# Transformation matrix to change the jaw rotation
jaw_rot = Frame(Rotation.RPY(-np.pi/2., 0., np.pi/2),
                   Vector(0, 0, 0))

# Get the position of the needle and change it to a frame
needle_pos = scene.measured_cp(SceneObjectType.Needle)
needle_pos.pose.position.y -= 0.008
needle_pos.pose.position.x -= 0.2
needle_pos.pose.position.z += 0.08
needle_frame = posemath.fromMsg(needle_pos.pose)

# Times together the transformation matrix, needle position, and jaw rotation
# to get the needle's position in the base frame
needle_trans = trans_frame_psm2*(needle_frame*jaw_rot)

# publish the target position to the robot
psm2.servo_cp(needle_trans)

# First, open the jaw
psm2.set_jaw_angle(180)
time.sleep(5.0)

# Grab the needle by closing the jaw
psm2.set_jaw_angle(0)
time.sleep(5.0)
################################# END ##########################################


############################### Move PSM2 to Entry2 ############################
	
# Transformation matrix to change the jaw rotation
jaw_rot_2 = Frame(Rotation.RPY(np.pi/1.6, np.pi/1.1, -np.pi/0.5),
                   Vector(0, 0, 0))

# Get the position of entry 2 and change it to a frame
entry_pos = scene.measured_cp(SceneObjectType.Entry2)
entry_pos.pose.position.y -= 0.04
entry_pos.pose.position.x -= 0.15
entry_pos.pose.position.z += 0.17
entry_frame = posemath.fromMsg(entry_pos.pose)

# Times together the transformation matrix, entry position, and jaw rotation
# to get the entry's position in the base frame
entry_trans = trans_frame_psm2*(entry_frame*jaw_rot_2)

# publish the target position to the robot
psm2.servo_cp(entry_trans)

time.sleep(3.0)
################################# END ##########################################

###################### Move Needle through the Tunnel ############################

# Transformation matrix to change the jaw rotation
jaw_rot_2 = Frame(Rotation.RPY(np.pi/1.6, np.pi/1.3, -np.pi/0.5),
                   Vector(0, 0, 0))

# Get the position of the other side of entry 2 and change it to a frame
entry_pos.pose.position.x += 0.18
exit_frame = posemath.fromMsg(entry_pos.pose)

# Times together the transformation matrix, exit position, and jaw rotation
# to get the exit's position in the base frame
exit_trans = trans_frame_psm2*(exit_frame*jaw_rot_2)

# publish the target position to the robot
psm2.servo_cp(exit_trans)

time.sleep(3.0)
################################# END ##########################################

############### Grab and Pull the Needle through Tunnel with Arm1 ################

# Get a handle to PSM1
psm1 = ARMInterface(ArmType.PSM1)
time.sleep(5.0)

# Get the transformation matrix from world to base for PSM1
trans_psm1 = psm1.get_T_w_b()
trans_frame_psm1 = posemath.fromMsg(trans_psm1.pose)
	
# Transformation matrix to change the jaw rotation
jaw_rot_3 = Frame(Rotation.RPY(0., 0., 0.),
                   Vector(0, 0, 0))

# Get the position of the needle and change it to a frame
needle_pos_2 = scene.measured_cp(SceneObjectType.Needle)
time.sleep(3.0)
needle_pos_2.pose.position.y += 0.10
needle_pos_2.pose.position.x += 0.06
needle_pos_2.pose.position.z += 0.01
needle_frame_2 = posemath.fromMsg(needle_pos_2.pose)

# Times together the transformation matrix, needle position, and jaw rotation
# to get the needle's position in the base frame
needle_trans_2 = trans_frame_psm1*(needle_frame_2*jaw_rot_3)

# publish the target position to psm1
psm1.servo_cp(needle_trans_2)
time.sleep(2.0)

# Grab the needle by closing the jaw
psm1.set_jaw_angle(0)
time.sleep(3.0)

# Let psm2 release the needle
psm2.set_jaw_angle(90)
time.sleep(2.0)

# Move psm2 away
entry_pos.pose.position.x -= 0.2
entry_pos.pose.position.y += 0.3
exit_frame = posemath.fromMsg(entry_pos.pose)
exit_trans = trans_frame_psm2*(exit_frame*jaw_rot_2)
psm2.servo_cp(exit_trans)
time.sleep(2.0)

# Rotate the needle a little bit to be ready to be pulled out
jaw_rot_4 = Frame(Rotation.RPY(-np.pi/3.4, -np.pi/4., -np.pi/3.45),
                   Vector(0, 0, 0))
pulling_frame = posemath.fromMsg(needle_pos_2.pose)
pulling = trans_frame_psm1*(pulling_frame*jaw_rot_4)
psm1.servo_cp(pulling)
time.sleep(5.0)

# Pull through the tunnel
needle_pos_2.pose.position.x += 0.2
needle_pos_2.pose.position.y -= 0.03
needle_pos_2.pose.position.z -= 0.005
pulling_frame = posemath.fromMsg(needle_pos_2.pose)
pulling = trans_frame_psm1*(pulling_frame*jaw_rot_4)
psm1.servo_cp(pulling)

time.sleep(3.0)
################################# END ##########################################

#################### Pass the Needle from PSM1 to PSM2 ##########################

# PSM1 moves to the center and holds the needle upright
jaw_rot_5 = Frame(Rotation.RPY(0., 0., 0.),
                   Vector(0, 0, 0))
needle_pos_2.pose.position.x -= 0.15
passing_frame = posemath.fromMsg(needle_pos_2.pose)
passing = trans_frame_psm1*(passing_frame*jaw_rot_5)
psm1.servo_cp(passing)
time.sleep(2.0)

# PSM2 moves close to the new needle position
jaw_rot_6 = Frame(Rotation.RPY(-np.pi/2., 0., np.pi/2.),
                   Vector(0, 0, 0))
needle_pos_3 = scene.measured_cp(SceneObjectType.Needle)
time.sleep(5.0)
needle_pos_3.pose.position.y += 0.09
needle_pos_3.pose.position.x -= 0.23
needle_pos_3.pose.position.z += 0.04
needle_frame_3 = posemath.fromMsg(needle_pos_3.pose)
passing = trans_frame_psm2*(needle_frame_3*jaw_rot_6)
psm2.set_jaw_angle(180)
psm2.servo_cp(passing)
time.sleep(5.0)

# PSM2 grabs the needle by closing the jaw
psm2.set_jaw_angle(0)
time.sleep(2.0)

# Let psm2 release the needle
psm1.set_jaw_angle(90)

# Move psm1 away
needle_pos_2.pose.position.x += 0.3
passing_frame = posemath.fromMsg(needle_pos_2.pose)
passing = trans_frame_psm1*passing_frame
psm1.servo_cp(passing)
time.sleep(2.0)
################################# END ##########################################

############################### Move PSM2 to Entry3 ############################
	
# Transformation matrix to change the jaw rotation
jaw_rot_7 = Frame(Rotation.RPY(np.pi/1.6, np.pi/1.1, -np.pi/0.5),
                   Vector(0, 0, 0))

# Get the position of entry 3 and change it to a frame
entry_pos_2 = scene.measured_cp(SceneObjectType.Entry3)
entry_pos_2.pose.position.y -= 0.05
entry_pos_2.pose.position.x -= 0.15
entry_pos_2.pose.position.z += 0.17
entry_frame_2 = posemath.fromMsg(entry_pos_2.pose)

# Times together the transformation matrix, entry position, and jaw rotation
# to get the entry's position in the base frame
entry_trans_2 = trans_frame_psm2*(entry_frame_2*jaw_rot_7)

# publish the target position to the robot
psm2.servo_cp(entry_trans_2)

time.sleep(3.0)
################################# END ##########################################


###################### Move Needle through the Tunnel (2) ############################

# Transformation matrix to change the jaw rotation
jaw_rot_8 = Frame(Rotation.RPY(np.pi/1.6, np.pi/1.3, -np.pi/0.5),
                   Vector(0, 0, 0))

# Get the position of the other side of entry 3 and change it to a frame
entry_pos_2.pose.position.x += 0.18
exit_frame_2 = posemath.fromMsg(entry_pos_2.pose)

# Times together the transformation matrix, exit position, and jaw rotation
# to get the exit's position in the base frame
exit_trans_2 = trans_frame_psm2*(exit_frame_2*jaw_rot_8)

# publish the target position to the robot
psm2.servo_cp(exit_trans_2)

time.sleep(3.0)
################################# END ##########################################

############## Grab and Pull the Needle through Tunnel with Arm1 (2) ###############

# Get a handle to PSM1
psm1 = ARMInterface(ArmType.PSM1)
time.sleep(5.0)

# Get the transformation matrix from world to base
trans_psm1 = psm1.get_T_w_b()
trans_frame_psm1 = posemath.fromMsg(trans_psm1.pose)

# Get the position of the needle and change it to a frame
needle_pos_4 = scene.measured_cp(SceneObjectType.Needle)
time.sleep(3.0)
needle_pos_4.pose.position.y += 0.08
needle_pos_4.pose.position.x += 0.03
needle_pos_4.pose.position.z += 0.00
needle_frame_4 = posemath.fromMsg(needle_pos_4.pose)

# Times together the transformation matrix, needle position, and jaw rotation
# to get the needle's position in the base frame
needle_trans_4 = trans_frame_psm1*(needle_frame_4*jaw_rot_3)

# publish the target position to psm1
psm1.servo_cp(needle_trans_4)
time.sleep(2.0)

# Grab the needle by closing the jaw
psm1.set_jaw_angle(0)
time.sleep(3.0)

# Let psm2 release the needle
psm2.set_jaw_angle(90)
time.sleep(2.0)

# Move psm2 away
entry_pos_2.pose.position.x -= 0.2
entry_pos_2.pose.position.y += 0.3
exit_frame_2 = posemath.fromMsg(entry_pos_2.pose)
exit_trans_2 = trans_frame_psm2*(exit_frame_2*jaw_rot_2)
psm2.servo_cp(exit_trans_2)
time.sleep(2.0)

jaw_rot_9 = Frame(Rotation.RPY(-np.pi/3.2, -np.pi/6., -np.pi/3.),
                   Vector(0, 0, 0))

pulling_frame_2 = posemath.fromMsg(needle_pos_4.pose)
pulling_2 = trans_frame_psm1*(pulling_frame_2*jaw_rot_4)
psm1.servo_cp(pulling_2)
time.sleep(5.0)

# Pull through the tunnel
needle_pos_4.pose.position.x += 0.2
needle_pos_4.pose.position.y -= 0.05
needle_pos_4.pose.position.z -= 0.005
pulling_frame_2 = posemath.fromMsg(needle_pos_4.pose)
pulling_2 = trans_frame_psm1*(pulling_frame_2*jaw_rot_9)
psm1.servo_cp(pulling_2)

time.sleep(3.0)

rate.sleep()
################################# END ##########################################

