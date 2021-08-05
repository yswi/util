#! /usr/bin/env python
import rospy
import numpy as np
from mmint_camera_utils.camera_calibration import CameraApriltagCalibration

# from arm_robots.med import Med # TODO : change it to Panda
# from victor_hardware_interface_msgs.msg import ControlMode # franka_interface.ArmInterface
from panda_robot import PandaArm
from tqdm import tqdm

def kuka_calibration():
    rospy.init_node('panda_camera_calibration')
    sim = rospy.get_param('~sim', default=True)

    # Initialize the robot abstraction
    p = PandaArm()
    # med.connect()
    # med.set_control_mode(ControlMode.JOINT_POSITION, vel=0.1)

    # Initialize the calibration
    calibrator = CameraApriltagCalibration(tag_id=0, calibration_frame_name='calibration_frame', parent_frame_name='med_base')


    ## TODO : Change this calibration configurations ##
    calibration_configurations_base= [
        [0.552, 0.614, 0.971, -1.57, 2.298, -1.432, -2.538],
        [0.149, 0.796, 0.516, -1.567, 1.798, -0.562, -2.243],
        [0.039, 1.21, 0.234, -0.847, 1.71, -0.371, -1.913],
        [-0.26, 0.512, 0.408, -1.399, 2.381, -0.663, -2.535],
        [-0.389, 0.587, -0.002, -1.57, 2.394, -1.04, -1.535],
        [-0.389, 0.587, -0.002, -1.57, 2.394, -1.04, -1.535],
        [-0.655, 0.823, 0.059, -1.568, 2.278, -0.833, -1.563]
    ] # poses to take measurments
    wrist_joint_values = [-2.5, -1.5, 0]
    calibration_configurations = []
    for cal_conf_base in calibration_configurations_base:
        for wrist_joint_v in wrist_joint_values:
            cc_i = cal_conf_base[:-1] + [wrist_joint_v]
            calibration_configurations.append(cc_i)
    # calibration_configurations = calibration_configurations_base
    for i, calibration_conf in enumerate(tqdm(calibration_configurations, desc="Calibrating Cameras")):
        # # med.plan_to_pose(med.arm_group, med., target_position=target_position)
        # # print('calibration pose {}/{}'.format(i, len(calibration_configurations-1)))
        # med.plan_to_joint_config(med.arm_group, calibration_conf)
        p.move_to_joint_position(calibration_conf)
        calibrator.take_mesurement()

    # move the robot back to default configuration
    med.plan_to_joint_config(med.arm_group, [0, 0, 0, 0, 0, 0, 0])

    # compute the camera position and broadcast it
    calibrator.broadcast_tfs()
    rospy.spin()

if __name__ == '__main__':
    kuka_calibration()
