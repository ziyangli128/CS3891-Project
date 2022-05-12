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
from ambf_client import Client
from surgical_robotics_challenge.psm_arm import PSM, jpRecorder
from surgical_robotics_challenge.ecm_arm import ECM
import time
import rospy
from PyKDL import Frame, Rotation, Vector
from argparse import ArgumentParser
from input_devices.geomagic_device import GeomagicDevice
from itertools import cycle
from surgical_robotics_challenge.utils.jnt_control_gui import JointGUI
from surgical_robotics_challenge.utils.utilities import get_boolean_from_opt
import sys


class ControllerInterface:
    def __init__(self, leader, psm_arms, camera):
        self.counter = 0
        self.leader = leader
        self.psm_arms = cycle(psm_arms)
        self.active_psm = next(self.psm_arms)
        self.gui = JointGUI('ECM JP', 4, ["ecm j0", "ecm j1", "ecm j2", "ecm j3"])

        self.cmd_xyz = self.active_psm.T_t_b_home.p
        self.cmd_rpy = None
        self.T_IK = None
        self._camera = camera

        self._T_c_b = None
        self._update_T_c_b = True

    def switch_psm(self):
        self._update_T_c_b = True
        if sys.version_info[0] >= 3:
            self.active_psm = next(self.psm_arms)
        else:
            self.active_psm = self.active_psm.next()
        print('Switching Control of Next PSM Arm: ', self.active_psm.name)

    def update_T_c_b(self):
        if self._update_T_c_b or self._camera.has_pose_changed():
            self._T_c_b = self.active_psm.get_T_w_b() * self._camera.get_T_c_w()
            self._update_T_c_b = False

    def update_camera_pose(self):
        self.gui.App.update()
        self._camera.servo_jp(self.gui.jnt_cmds)

    def update_arm_pose(self):
        self.update_T_c_b()
        twist = self.leader.measured_cv()
        self.cmd_xyz = self.active_psm.T_t_b_home.p
        if not self.leader.clutch_button_pressed:
            delta_t = self._T_c_b.M * twist.vel * 0.00002
            self.cmd_xyz = self.cmd_xyz + delta_t
            self.active_psm.T_t_b_home.p = self.cmd_xyz

        self.cmd_rpy = self._T_c_b.M * self.leader.measured_cp().M * Rotation.RPY(3.14, 0, 3.14 / 2)
        self.T_IK = Frame(self.cmd_rpy, self.cmd_xyz)
        self.active_psm.servo_cp(self.T_IK)
        self.active_psm.set_jaw_angle(self.leader.get_jaw_angle())

    def update_visual_markers(self):
        # Move the Target Position Based on the GUI
        if self.active_psm.target_IK is not None:
            T_t_w = self.active_psm.get_T_b_w() * self.T_IK
            self.active_psm.target_IK.set_pos(T_t_w.p[0], T_t_w.p[1], T_t_w.p[2])
            self.active_psm.target_IK.set_rpy(T_t_w.M.GetRPY()[0], T_t_w.M.GetRPY()[1], T_t_w.M.GetRPY()[2])
        # if self.arm.target_FK is not None:
        #     ik_solution = self.arm.get_ik_solution()
        #     ik_solution = np.append(ik_solution, 0)
        #     T_7_0 = convert_mat_to_frame(compute_FK(ik_solution))
        #     T_7_w = self.arm.get_T_b_w() * T_7_0
        #     P_7_0 = T_7_w.p
        #     RPY_7_0 = T_7_w.M.GetRPY()
        #     self.arm.target_FK.set_pos(P_7_0[0], P_7_0[1], P_7_0[2])
        #     self.arm.target_FK.set_rpy(RPY_7_0[0], RPY_7_0[1], RPY_7_0[2])

    def run(self):
        if self.leader.switch_psm:
            self.switch_psm()
            self.leader.switch_psm = False
        self.update_camera_pose()
        self.update_arm_pose()
        self.update_visual_markers()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('--one', action='store', dest='run_psm_one', help='Control PSM1', default=True)
    parser.add_argument('--two', action='store', dest='run_psm_two', help='Control PSM2', default=True)
    parser.add_argument('--three', action='store', dest='run_psm_three', help='Control PSM3', default=True)
    parser.add_argument('--save', action='store', dest='jp_record', help='save using jp_recorder', default=False)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    parsed_args.run_psm_one = get_boolean_from_opt(parsed_args.run_psm_one)
    parsed_args.run_psm_two = get_boolean_from_opt(parsed_args.run_psm_two)
    parsed_args.run_psm_three = get_boolean_from_opt(parsed_args.run_psm_three)

    c = Client()
    c.connect()

    cam = ECM(c, 'CameraFrame')
    time.sleep(0.5)

    controllers = []
    psm_arms = []

    if parsed_args.run_psm_one is True:
        # Initial Target Offset for PSM1
        # init_xyz = [0.1, -0.85, -0.15]
        arm_name = 'psm1'
        print('LOADING CONTROLLER FOR ', arm_name)
        psm = PSM(c, arm_name, add_joint_errors=False, save_jp=parsed_args.jp_record)
        if psm.is_present():
            T_psmtip_c = Frame(Rotation.RPY(3.14, 0.0, -1.57079), Vector(-0.2, 0.0, -1.0))
            T_psmtip_b = psm.get_T_w_b() * cam.get_T_c_w() * T_psmtip_c
            psm.set_home_pose(T_psmtip_b)
            psm_arms.append(psm)

    if parsed_args.run_psm_two is True:
        # Initial Target Offset for PSM1
        # init_xyz = [0.1, -0.85, -0.15]
        arm_name = 'psm2'
        print('LOADING CONTROLLER FOR ', arm_name)
        psm = PSM(c, arm_name, add_joint_errors=False, save_jp=parsed_args.jp_record)
        if psm.is_present():
            T_psmtip_c = Frame(Rotation.RPY(3.14, 0.0, -1.57079), Vector(0.2, 0.0, -1.0))
            T_psmtip_b = psm.get_T_w_b() * cam.get_T_c_w() * T_psmtip_c
            psm.set_home_pose(T_psmtip_b)
            psm_arms.append(psm)

    if parsed_args.run_psm_three is True:
        # Initial Target Offset for PSM1
        # init_xyz = [0.1, -0.85, -0.15]
        arm_name = 'psm3'
        print('LOADING CONTROLLER FOR ', arm_name)
        psm = PSM(c, arm_name, add_joint_errors=False, save_jp=parsed_args.jp_record)
        if psm.is_present():
            psm_arms.append(psm)

    rate = rospy.Rate(200)

    if len(psm_arms) == 0:
        print('No Valid PSM Arms Specified')
        print('Exiting')

    else:
        leader = GeomagicDevice('/Geomagic/')
        theta_base = -0.9
        theta_tip = -theta_base
        leader.set_base_frame(Frame(Rotation.RPY(theta_base, 0, 0), Vector(0, 0, 0)))
        leader.set_tip_frame(Frame(Rotation.RPY(theta_base + theta_tip, 0, 0), Vector(0, 0, 0)))
        controller = ControllerInterface(leader, psm_arms, cam)
        controllers.append(controller)
        while not rospy.is_shutdown():
            try:
                for cont in controllers:
                    cont.run()
            except KeyboardInterrupt:
                    jpRecorder.flush()
            rate.sleep()

