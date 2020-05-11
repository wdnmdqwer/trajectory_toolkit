#!/usr/bin/env python

import os, sys, inspect
import matplotlib.pyplot as plt
import numpy as np
import rospkg

from trajectory_toolkit.TimedData import TimedData
from trajectory_toolkit.Plotter import Plotter
from trajectory_toolkit import Quaternion
from trajectory_toolkit import Utils
from trajectory_toolkit import RosDataAcquisition
from trajectory_toolkit import CsvDataAcquisition
CsvDataAcquisition

# data_path = "/home/zsk/trajectory_toolkit/data"
gt_data_path = "/home/zsk/Downloads/bebop2data/gt/Take 2020-04-28 12.44.38 AM.csv"

td = TimedData(1)
td.clearLabeling()
td.addLabelingIncremental("pos", 3)
td.addLabelingIncremental("att", 4)
td.reInit()
CsvDataAcquisition.csvLoadTransform(gt_data_path, 1, [6, 7, 8], [5, 2, 3, 4], td, 'pos', 'att')
# print(td.d[1, :])

# isNode = False;
#
# """ Initialize Plotters
# plotter1: figureID=1, three subplots
# """
# plotter1 = Plotter(1, [3,1])
#
#
# """ Initialize TimedData
#         td1: TimedData with 18 columns: 0:t, 1-3:pos, 4-7:att, 8-10:vel,  11-13:velInBodyFrame, 14-16:ror, 17:rorNorm
#         td2: TimedData with 18 columns: 0:t, 1-4:att, 5-7:ror, 8:rorNorm, 9-11:pos, 12-14:vel, 15-17:velInBodyFrame
# """
# td1 = TimedData(18)
# td2 = TimedData(18)
# posIDs1 = [1,2,3]
# posIDs2 = [9,10,11]
# attIDs1 = [4,5,6,7]
# attIDs2 = [1,2,3,4]
# velIDs1 = [8,9,10]
# velIDs2 = [12,13,14]
# velBIDs1 = [11,12,13]
# velBIDs2 = [15,16,17]
# rorIDs1 = [14,15,16]
# rorIDs2 = [5,6,7]
# rorNID1 = 17
# rorNID2 = 8
#
# """
#         First we will fill the two TimedData structures with the same StampedTransforms,
#         loaded from topic 'rovio/transform' in example.bag.
#         The indices denote the start column of the position(td1=1, td2=9) and attitude(td1=4, td2=1)
# """
# RosDataAcquisition.rosBagLoadTransformStamped(os.path.join(data_path, 'example.bag'), '/rovio/transform',td1,posIDs1,attIDs1)
# RosDataAcquisition.rosBagLoadTransformStamped(os.path.join(data_path, 'example.bag'), '/rovio/transform',td2,posIDs2,attIDs2)
#
# # Add initial x to plot
plotter1.addDataToSubplot(td1, posIDs1[0], 1, 'r', 'td1In x');
plotter1.addDataToSubplot(td2, posIDs2[0], 1, 'b', 'td2In x');


# Add transformed x to plot
plotter1.addDataToSubplot(td1, posIDs1[0], 2, 'r', 'td1In x');
plotter1.addDataToSubplot(td2, posIDs2[0], 2, 'b', 'td2Trans x');

"""
        Now we are ready to calculate the other TimedData properties.
"""
# Calculate the velocity in the world frame provide pos(td1=[1,2,3], td2=[9,10,11]) and vel(td1=[8,9,10], td2=[12,13,14]) column IDs.
td1.computeVectorNDerivative(posIDs1, velIDs1)
td2.computeVectorNDerivative(posIDs2, velIDs2)
# Calculate the velocity in the body frame provide pos(td1=[1,2,3], td2=[9,10,11]) and velInBodyFrame(td1=[11,12,13], td2=[15,16,17]) column IDs.
# Additionally the rotation Quaternion qBI rps qCJ has to be provided.
td1.computeVelocitiesInBodyFrameFromPostionInWorldFrame(posIDs1, velBIDs1, attIDs1)
td2.computeVelocitiesInBodyFrameFromPostionInWorldFrame(posIDs2, velBIDs2, attIDs2)
# Calculate the Rotational Rate provide att(td1=4, td2=1) and rot(td1=14, td2=5) start column IDs.
td1.computeRotationalRateFromAttitude(attIDs1,rorIDs1)
td2.computeRotationalRateFromAttitude(attIDs2,rorIDs2)
# Calculate the Norm of the Rotational Rate provide ror(td1=[14,15,16], td2=[5,6,7]) and rorNorm(td1=17,td2=8) column IDs.
td1.computeNormOfColumns(rorIDs1,rorNID1)
td2.computeNormOfColumns(rorIDs2,rorNID2)

"""
        We can estimate the time offset using the norm of the rotational rate.
        The estimated time offset is then applied to td2.
"""
to = td2.getTimeOffset(rorNID2,td1,rorNID1)
td2.applyTimeOffset(-to)

"""
        The calibration of the Body Transform needs the velocity and the rotational rate start IDs.
"""
B_r_BC_est, qCB_est = td1.calibrateBodyTransform(velBIDs1,rorIDs1,td2, velBIDs2,rorIDs2)
vCB_est = Quaternion.q_log(qCB_est)
vCB_err = vCB-vCB_est
B_r_BC_err = B_r_BC - B_r_BC_est
print('Calibrate Body Transform:')
print('Rotation Vector ln(qCB_est):\tvx:' + str(vCB_est[0]) + '\tvy:' + str(vCB_est[1]) + '\tvz:' + str(vCB_est[2]))
print('Translation Vector B_r_BC_est:\trx:' + str(B_r_BC_est[0]) + '\try:' + str(B_r_BC_est[1]) + '\trz:' + str(B_r_BC_est[2]))
print('Rotation Error ln(qCB_err):\tvx:' + str(vCB_err[0]) + '\tvy:' + str(vCB_err[1]) + '\tvz:' + str(vCB_err[2]))
print('Translation Error B_r_BC_err:\trx:' + str(B_r_BC_err[0]) + '\try:' + str(B_r_BC_err[1]) + '\trz:' + str(B_r_BC_err[2]))

"""
        The calibration of the Intertial Transform needs the velocity and the rotational rate start IDs and the estimated body transform.
"""
J_r_JI_est, qIJ_est = td1.calibrateInertialTransform(posIDs1, attIDs1, td2, posIDs2, attIDs2, B_r_BC_est, qCB_est, [0,1,2])
vIJ_est = Quaternion.q_log(qIJ_est);
vIJ_err = vIJ-vIJ_est;
J_r_JI_err = J_r_JI - J_r_JI_est;
print('Calibrate Inertial Transform:')
print('Rotation Vector ln(qIJ_est):\tvx:' + str(vIJ_est[0]) + '\tvy:' + str(vIJ_est[1]) + '\tvz:' + str(vIJ_est[2]))
print('Translation Vector J_r_JI_est:\trx:' + str(J_r_JI_est[0]) + '\try:' + str(J_r_JI_est[1]) + '\trz:' + str(J_r_JI_est[2]))
print('Rotation Error ln(qIJ_err):\tvx:' + str(vIJ_err[0]) + '\tvy:' + str(vIJ_err[1]) + '\tvz:' + str(vIJ_err[2]))
print('Translation Error J_r_JI_err:\trx:' + str(J_r_JI_err[0]) + '\try:' + str(J_r_JI_err[1]) + '\trz:' + str(J_r_JI_err[2]))


# Add calibrated x to plot
td1.applyBodyTransform(posIDs1, attIDs1, B_r_BC_est, qCB_est)
td1.applyInertialTransform(posIDs1, attIDs1,J_r_JI_est,qIJ_est)

plotter1.addDataToSubplot(td1, posIDs1[0], 3, 'r', 'td1Cal x');
plotter1.addDataToSubplot(td2, posIDs2[0], 3, 'b', 'td2Trans x');

