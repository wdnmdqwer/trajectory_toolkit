#!/usr/bin/python
import os, sys, inspect
import numpy as np

from trajectory_toolkit.TimedData import TimedData
from trajectory_toolkit.Plotter import Plotter
from trajectory_toolkit import Quaternion
from trajectory_toolkit import Utils
from trajectory_toolkit import RosDataAcquisition
from trajectory_toolkit import VIEvaluator

plotRon = False
plotAtt = False
plotPos = True
plotVel = True
plotRor = False
plotYpr = True
plotExt = True

td_rovio = TimedData()
td_vicon = TimedData()

rovioEvaluator = VIEvaluator.VIEvaluator()
rovioEvaluator.bag = '/home/zsk/of_velocity/results/2020_04_28_12_44_23_direct/2020_04_28_12_44_23_est.csv'
# rovioEvaluator.odomTopic = '/rovio/odometry'
# rovioEvaluator.pclTopic = '/rovio/pcl'
# rovioEvaluator.extrinsicsTopic = '/rovio/extrinsics0'
rovioEvaluator.gtFile = '/home/zsk/Downloads/bebop2data/gt/Take 2020-04-28 12.44.38 AM.csv'
# rovioEvaluator.gtTopic = '/bluebird/vrpn_client/estimated_transform'
rovioEvaluator.startcut = 0
rovioEvaluator.endcut = 0
rovioEvaluator.doCov = False
rovioEvaluator.doNFeatures = 0
rovioEvaluator.doExtrinsics = False
rovioEvaluator.doBiases = False
rovioEvaluator.alignMode = 1
rovioEvaluator.plotLeutiDistances = []

gtTimeCol = 1
gtPosCol = [6, 7, 8]
gtAttCol = [5, 2, 3, 4]

estTimeCol = 0
estPosCol = [1, 2, 3]
estAttCol = [4, 5, 6, 7]
estVelCol = [9, 10, 11]
estRorCol = [23, 24, 25]

rovioEvaluator.initTimedData(td_rovio)
rovioEvaluator.initTimedDataGT(td_vicon)
rovioEvaluator.acquireData(estTimeCol, estPosCol, estAttCol, estVelCol, estRorCol, delimiter=' ')
rovioEvaluator.acquireDataGT(gtTimeCol, gtPosCol, gtAttCol)
attIds = td_vicon.getColIDs('att')
# td_vicon.d[:, attIds[1:]] *= -1
# print(td_rovio.d.shape)
rovioEvaluator.getAllDerivatives()
rovioEvaluator.alignTime()
rovioEvaluator.alignBodyFrame()
rovioEvaluator.alignInertialFrame()
rovioEvaluator.getYpr()
rovioEvaluator.evaluateSigmaBounds()

if plotPos: # Position plotting
    plotterPos = Plotter(-1, [3,1],'Position',['','','time[s]'],['x[m]','y[m]','z[m]'],10000)
    if rovioEvaluator.doCov:
        plotterPos.addDataToSubplotMultiple(td_rovio, 'posSm', [1,2,3], ['r--','r--','r--'], ['','',''])
        plotterPos.addDataToSubplotMultiple(td_rovio, 'posSp', [1,2,3], ['r--','r--','r--'], ['','',''])
    plotterPos.addDataToSubplotMultiple(td_rovio, 'pos', [1,2,3], ['r','r','r'], ['','',''])
    plotterPos.addDataToSubplotMultiple(td_vicon, 'pos', [1,2,3], ['b','b','b'], ['','',''])

if plotVel: # Velocity plotting
    plotterVel = Plotter(-1, [3,1],'Robocentric Velocity',['','','time[s]'],['$v_x$[m/s]','$v_y$[m/s]','$v_z$[m/s]'],10000)
    plotterVel.addDataToSubplotMultiple(td_rovio, 'vel', [1,2,3], ['r','r','r'], ['','',''])
    plotterVel.addDataToSubplotMultiple(td_vicon, 'vel', [1,2,3], ['b','b','b'], ['','',''])

if plotAtt: # Attitude plotting
    plotterAtt = Plotter(-1, [4,1],'Attitude Quaternion',['','','','time[s]'],['w[1]','x[1]','y[1]','z[1]'],10000)
    plotterAtt.addDataToSubplotMultiple(td_rovio, 'att', [1,2,3,4], ['r','r','r','r'], ['','','',''])
    plotterAtt.addDataToSubplotMultiple(td_vicon, 'att', [1,2,3,4], ['b','b','b','b'], ['','','',''])

if plotYpr: # Yaw-pitch-roll plotting
    plotterYpr = Plotter(-1, [3,1],'Yaw-Pitch-Roll Decomposition',['','','time[s]'],['roll[rad]','pitch[rad]','yaw[rad]'],10000)
    if rovioEvaluator.doCov:
        plotterYpr.addDataToSubplotMultiple(td_rovio, 'yprSm', [1,2,3], ['r--','r--','r--'], ['','',''])
        plotterYpr.addDataToSubplotMultiple(td_rovio, 'yprSp', [1,2,3], ['r--','r--','r--'], ['','',''])
    plotterYpr.addDataToSubplotMultiple(td_rovio, 'ypr', [1,2,3], ['r','r','r'], ['','',''])
    plotterYpr.addDataToSubplotMultiple(td_vicon, 'ypr', [1,2,3], ['b','b','b'], ['','',''])

# if plotRor: # Rotational rate plotting
#     plotterRor = Plotter(-1, [3,1],'Rotational Rate',['','','time[s]'],['$\omega_x$[rad/s]','$\omega_y$[rad/s]','$\omega_z$[rad/s]'],10000)
#     plotterRor.addDataToSubplotMultiple(td_rovio, 'ror', [1,2,3], ['r','r','r'], ['','',''])
#     plotterRor.addDataToSubplotMultiple(td_vicon, 'ror', [1,2,3], ['b','b','b'], ['','',''])
#
# if plotRon: # Plotting rotational rate norm
#     plotterRon = Plotter(-1, [1,1],'Norm of Rotational Rate',['time [s]'],['Rotational Rate Norm [rad/s]'],10000)
#     plotterRon.addDataToSubplot(td_rovio, 'ron', 1, 'r', 'rovio rotational rate norm')
#     plotterRon.addDataToSubplot(td_vicon, 'ron', 1, 'b', 'vicon rotational rate norm')
#
# if plotExt and rovioEvaluator.doExtrinsics: # Extrinsics Plotting
#     plotterExt = Plotter(-1, [3,1],'Extrinsics Translational Part',['','','time[s]'],['x[m]','y[m]','z[m]'],10000)
#     if rovioEvaluator.doCov:
#         plotterExt.addDataToSubplotMultiple(td_rovio, 'extPosSm', [1,2,3], ['r--','r--','r--'], ['','',''])
#         plotterExt.addDataToSubplotMultiple(td_rovio, 'extPosSp', [1,2,3], ['r--','r--','r--'], ['','',''])
#     plotterExt.addDataToSubplotMultiple(td_rovio, 'extPos', [1,2,3], ['r','r','r'], ['','',''])
#
rovioEvaluator.doLeutiEvaluation()
# rovioEvaluator.doFeatureDepthEvaluation()

raw_input("Press Enter to continue...")
