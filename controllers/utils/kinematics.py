# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Port of the inverse kinematics code from this paper:
N. Kofinas, “Forward and inverse kinematics for the NAO humanoid robot,” Diploma Thesis,
Technical University of Crete, Greece, 2012,
available at: https://www.cs.umd.edu/~nkofinas/Projects/KofinasThesis.pdf
C++ code available at: https://github.com/kouretes/NAOKinematics
/!\ This code works in millimeters, not meters like Webots
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

# Constants from paper's C++ code
ShoulderOffsetY = 98.0
ElbowOffsetY = 15.0
UpperArmLength = 105.0
ShoulderOffsetZ = 100.0
LowerArmLength = 57.75
HandOffsetX = 55.95
HandOffsetZ = 12.31
HipOffsetZ = 85.0
HipOffsetY = 50.0
ThighLength = 100.0
TibiaLength = 102.9
FootHeight = 45.11
NeckOffsetZ = 126.5
CameraBottomX = 48.8
CameraBottomZ = 23.81
CameraTopX = 53.9
CameraTopZ = 67.9

# Head Limits
HeadYawHigh = 2.0857
HeadYawLow = -2.0857
HeadPitchHigh = 0.5149
HeadPitchLow = -0.6720

# Left Hand limits
LShoulderPitchHigh = 2.0857
LShoulderPitchLow = -2.0857
LShoulderRollHigh = 1.3265
LShoulderRollLow = -0.3142
LElbowYawHigh = 2.0875
LElbowYawLow = -2.0875
LElbowRollHigh = -0.0349
LElbowRollLow = -1.5446
LWristYawHigh = 1.8238
LWristYawLow = -1.8238

# Right Hand limits
RShoulderPitchHigh = 2.0857
RShoulderPitchLow = -2.0857
RShoulderRollHigh = 0.3142
RShoulderRollLow = -1.3265
RElbowYawHigh = 2.0875
RElbowYawLow = -2.0875
RElbowRollHigh = 1.5446
RElbowRollLow = 0.0349
RWristYawHigh = 1.8238
RWristYawLow = -1.8238

# Left Leg limits
# thetas = [LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll]
LHipYawPitchHigh = 0.7408
LHipYawPitchLow = -1.1453
LHipRollHigh = 0.7904
LHipRollLow = -0.3794
LHipPitchHigh = 0.4840
LHipPitchLow = -1.7739
LKneePitchHigh = 2.1125
LKneePitchLow = -0.0923
LAnklePitchHigh = 0.9227
LAnklePitchLow = -1.1895
LAnkleRollHigh = 0.7690
LAnkleRollLow = -0.3978

# Left Right limits
RHipYawPitchHigh = 0.7408
RHipYawPitchLow = -1.1453
RHipRollHigh = 0.4147
RHipRollLow = -0.7383
RHipPitchHigh = 0.4856
RHipPitchLow = -1.7723
RKneePitchHigh = 2.1201
RKneePitchLow = -0.1030
RAnklePitchHigh = 0.9320
RAnklePitchLow = -1.1864
RAnkleRollHigh = 0.3886
RAnkleRollLow = -1.1864

# The thetas are stored in the order they are computed in the paper:
# theta6, theta4, theta5, theta2, theta3, theta1
# Here we store the default standing joints
left_leg_previous_joints = [0, 1.047, -0.524, 0, -0.524, 0]
right_leg_previous_joints = [0, 1.047, -0.524, 0, -0.524, 0]


def DH(a, alpha, d, theta):
    """Return the Denavit-Hartenberg matrix for the given parameters"""
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) *
         np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) *
         np.sin(alpha), np.cos(alpha), d * np.cos(alpha)],
        [0, 0, 0, 1]
    ])


def orientation_to_transform(orientation):
    """Return the affine transform matrix for the given orientation"""
    T = np.eye(4)
    T[:3, :3] = R.from_euler('ZYX', orientation).as_matrix()
    return T


def orientation_from_transform(T):
    """Return the orientation from the given affine transform matrix"""
    roll = np.arctan2(T[2, 1], T[2, 2])
    pitch = np.arctan2(-T[2, 0], np.sqrt(T[2, 1]**2 + T[2, 2]**2))
    yaw = np.arctan2(T[1, 0], T[0, 0])
    return np.array([roll, pitch, yaw])


def transform_from_position_and_orientation(position, orientation):
    """Return the affine transform matrix for the desired position and orientation"""
    T = orientation_to_transform(orientation)
    T[0:3, 3] = position
    return T


def get_A_base_0(is_left):
    A_base_0 = np.eye(4)
    A_base_0[1, 3] = HipOffsetY if is_left else -HipOffsetY
    A_base_0[2, 3] = -HipOffsetZ
    return A_base_0


def get_T_0_1(theta_1, is_left):
    T_0_1 = DH(0, -np.pi/4 * 3 if is_left else 1, 0, theta_1 - np.pi/2)
    return T_0_1


def get_T_1_2(theta_2, is_left):
    T_1_2 = DH(0, -np.pi/2, 0, theta_2 + np.pi/4 if is_left else -np.pi/4)
    return T_1_2


def get_T_2_3(theta_3):
    T_2_3 = DH(-ThighLength, 0, 0, theta_3)
    return T_2_3


def get_T_3_4(theta_4):
    T_3_4 = DH(-ThighLength, 0, 0, theta_4)
    return T_3_4


def get_T_4_5(theta_5):
    T_4_5 = DH(-TibiaLength, 0, 0, theta_5)
    return T_4_5


def get_T_5_6(theta_6):
    T_5_6 = DH(0, -np.pi/2, 0, theta_6)
    return T_5_6


def get_Rot_zy():
    Rot_zy = orientation_to_transform([np.pi, -np.pi/2, 0])
    return Rot_zy


def get_A_6_end():
    A_6_end = np.eye(4)
    A_6_end[2, 3] = -FootHeight
    return A_6_end


def forward_left_leg(thetas):
    """Return the position and orientation of the left foot for the given joint angles (forwards kinematics)"""
    A_base_0 = get_A_base_0(True)
    T_0_1 = get_T_0_1(thetas[0], True)
    T_1_2 = get_T_1_2(thetas[1], True)
    T_2_3 = get_T_2_3(thetas[2])
    T_3_4 = get_T_3_4(thetas[3])
    T_4_5 = get_T_4_5(thetas[4])
    T_5_6 = get_T_5_6(thetas[5])
    Rot_zy = get_Rot_zy()
    A_6_end = get_A_6_end()
    T_base_end = A_base_0 @ T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5 @ T_5_6 @ Rot_zy @ A_6_end
    return np.concatenate((T_base_end[0:3, 3], orientation_from_transform(T_base_end)))


def forward_right_leg(thetas):
    """Return the position and orientation of the right foot for the given joint angles (forwards kinematics)"""
    A_base_0 = get_A_base_0(False)
    T_0_1 = get_T_0_1(thetas[0], False)
    T_1_2 = get_T_1_2(thetas[1], False)
    T_2_3 = get_T_2_3(thetas[2])
    T_3_4 = get_T_3_4(thetas[3])
    T_4_5 = get_T_4_5(thetas[4])
    T_5_6 = get_T_5_6(thetas[5])
    Rot_zy = get_Rot_zy()
    A_6_end = get_A_6_end()
    T_base_end = A_base_0 @ T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5 @ T_5_6 @ Rot_zy @ A_6_end
    return np.concatenate((T_base_end[0:3, 3], orientation_from_transform(T_base_end)))


class Tree:
    """A tree data structure for storing all the possible inverse kinematics solutions."""

    def __init__(self, angle):
        self.angle = angle
        self.children = []

    def add_child_node(self, angle):
        self.children.append(Tree(angle))

    def add_child(self, child):
        self.children.append(child)


def get_angle_combinations(node):
    """Return all the possible combinations of joint angles for the given node and its children"""
    if not node.children:
        return [[node.angle]]
    combinations = []
    for child in node.children:
        child_combinations = get_angle_combinations(child)
        for combination in child_combinations:
            combinations.append([node.angle] + combination)
    return combinations


def inverse_leg(x, y, z, roll, pitch, yaw, is_left):
    """Return the joint angles for the desired position and orientation of the foot (inverse kinematics)"""
    global left_leg_previous_joints
    global right_leg_previous_joints

    # This does all the maths from the paper step-by-step (look at the paper if you are interested)
    T = transform_from_position_and_orientation([x, y, z], [yaw, pitch, roll])
    A_base_0 = get_A_base_0(is_left)
    A_6_end = get_A_6_end()
    T_hat = np.linalg.inv(A_base_0) @ T @ np.linalg.inv(A_6_end)
    # This angle offset depends on which leg we are doing the inverse kinematics
    plus_or_minus_pi_over_4 = np.pi/4 if is_left else -np.pi/4
    T_tilde = orientation_to_transform([0, 0, plus_or_minus_pi_over_4]) @ T_hat
    T_prime = np.linalg.inv(T_tilde)
    px_prime, py_prime, pz_prime = T_prime[0:3, 3]
    theta_6 = np.arctan(py_prime/pz_prime)
    solution_tree = Tree(theta_6)
    d = np.linalg.norm([px_prime, py_prime, pz_prime])
    theta_4_double_prime = np.pi - np.arccos((ThighLength**2 + TibiaLength**2 - d**2) /
                                             (2*ThighLength*TibiaLength))
    for theta_4_test in [theta_4_double_prime, -theta_4_double_prime]:
        if LKneePitchLow < theta_4_test < LKneePitchHigh:
            solution_tree.add_child_node(theta_4_test)
    T_tilde_prime = T_tilde @ np.linalg.inv(get_T_5_6(theta_6) @ get_Rot_zy())
    T_double_prime = np.linalg.inv(T_tilde_prime)
    for theta_4_node in solution_tree.children:
        theta_4_test = theta_4_node.angle
        numerator = T_double_prime[1, 3] * (TibiaLength + ThighLength * np.cos(theta_4_test)) + \
            ThighLength * T_double_prime[0, 3] * np.sin(theta_4_test)
        denominator = ThighLength**2 * np.sin(theta_4_test)**2 + \
            (TibiaLength + ThighLength * np.cos(theta_4_test))**2
        theta_5_prime = np.arcsin(-numerator / denominator)
        for theta_5_test in [theta_5_prime, (np.pi if theta_5_prime >= 0 else -np.pi) - theta_5_prime]:
            if LAnklePitchLow < theta_5_test < LAnklePitchHigh:
                theta_4_node.add_child_node(theta_5_test)
    for theta_4_node in solution_tree.children:
        for theta_5_node in theta_4_node.children:
            theta_4_test = theta_4_node.angle
            theta_5_test = theta_5_node.angle
            T_3_4 = get_T_3_4(theta_4_test)
            T_4_5 = get_T_4_5(theta_5_test)
            T_triple_prime = T_tilde_prime @ np.linalg.inv(T_3_4 @ T_4_5)
            theta_2_prime = np.arccos(T_triple_prime[1, 2])
            for theta_2_test in [theta_2_prime - plus_or_minus_pi_over_4, - theta_2_prime - plus_or_minus_pi_over_4]:
                if LHipRollLow < theta_2_test < LHipRollHigh:
                    theta_2_node = Tree(theta_2_test)
                else:
                    continue
                theta_3_prime = np.arcsin(
                    T_triple_prime[1, 1] / np.sin(theta_2_test + plus_or_minus_pi_over_4))
                theta_3 = []
                for theta_3_test in [theta_3_prime, (np.pi if theta_3_prime >= 0 else -np.pi) - theta_3_prime]:
                    if LHipPitchLow < theta_3_test < LHipPitchHigh:
                        theta_3.append(theta_3_test)
                if len(theta_3) == 0:
                    continue
                theta_1_prime = np.arccos(
                    T_triple_prime[0, 2] / np.sin(theta_2_test + plus_or_minus_pi_over_4))
                theta_1 = []
                for theta_1_test in [theta_1_prime + np.pi/2, - theta_1_prime + np.pi/2]:
                    if LHipYawPitchLow < theta_1_test < LHipYawPitchHigh:
                        theta_1.append(theta_1_test)
                for theta_3_angle in theta_3:
                    theta_3_node = Tree(theta_3_angle)
                    for theta_1_angle in theta_1:
                        theta_3_node.add_child_node(theta_1_angle)
                    theta_2_node.add_child(theta_3_node)
                # We only add the theta_2_node if it results in a valid theta_3 and theta_1
                theta_5_node.add_child(theta_2_node)
    combinations = get_angle_combinations(solution_tree)
    if len(combinations) != 1:
        print("Number of combination different than one:", combinations)
        # compute the distance between the different combinations and the previous joints and return the closest one
        shortest_distance = np.Inf
        best_index = -1
        for i, combination in enumerate(combinations):
            distance = np.linalg.norm(np.array(
                left_leg_previous_joints if is_left else right_leg_previous_joints) - np.array(combination))
            if distance < shortest_distance:
                shortest_distance = distance
                best_index = i
        best_solution = combinations[best_index]
    elif len(combinations[0]) != 6:
        print("Incomputable desired end point position", combinations)
        best_solution = left_leg_previous_joints if is_left else right_leg_previous_joints
    else:
        best_solution = combinations[0]
    if is_left:
        left_leg_previous_joints = best_solution
    else:
        right_leg_previous_joints = best_solution
    theta_6, theta_4, theta_5, theta_2, theta_3, theta_1 = best_solution
    return theta_1, theta_2, theta_3, theta_4, theta_5, theta_6
