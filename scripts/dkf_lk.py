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

datasource = 'bebop'
if datasource == 'mynt':
    video_path_prefix = "/home/zsk/Downloads/mynt/zsk/"
    gt_path_prefix = "/home/zsk/Downloads/mynt/zsk/gt/"
elif datasource == 'bebop':
    video_path_prefix = "/home/zsk/Downloads/bebop2data/"
    gt_path_prefix = "/home/zsk/Downloads/bebop2data/gt/"
est_result_path_prefix = "/home/zsk/of_velocity/results"

datas = [
    # ("2019_07_31_16_39_00", "Take 2019-07-31 04.44.00 PM.csv"),
    # ("2019_07_31_16_10_00", "Take 2019-07-31 04.12.03 PM.csv"), # rovio start after 300 frames
    # ("2019_07_31_16_46_00", "Take 2019-07-31 04.48.25 PM.csv"),
    # ("2019_07_31_15_23_00", "Take 2019-07-31 03.25.33 PM.csv"),
    # ("2019_07_31_15_54_00", "Take 2019-07-31 03.56.12 PM.csv"),
    # ("2019_08_04_20_50_00", "Take 2019-08-04 08.50.26 PM.csv"), # vins start after 20s
    # ("2019_08_04_20_43_00", "Take 2019-08-04 08.44.19 PM.csv"),


    # bebop data
    # ("2020_04_28_12_44_23", "Take 2020-04-28 12.44.38 AM.csv"), # 0.51, 1.27
    # ("2020_04_16_20_16_01", "Take 2020-04-16 08.16.03 PM.csv"), # 0.65, 1.37
    # ("2020_04_28_13_37_49", "Take 2020-04-28 01.38.04 PM.csv"), # 0.77, 1.58
    # ("2020_04_28_13_54_45", "Take 2020-04-28 01.55.00 PM.csv"), # 0.82, 1.81
    # ("2020_04_10_13_01_11", "Take 2020-04-10 01.01.10 PM.csv"),
    # ("2020_04_10_15_10_06", "Take 2020-04-10 03.10.04 PM.csv"),
    # ("2020_03_14_20_00_11", "Take 2020-03-14 08.00.23 PM.csv"),
    # ("2020_04_10_15_19_49", "Take 2020-04-10 03.19.49 PM.csv"),
    ("2020_10_23_13_18_39", "Take 2020-10-23 01.18.37 PM.csv"),
    ("2020_10_23_19_13_51", "Take 2020-10-23 07.13.54 PM.csv"),
]
evaluate = "vins"
# evaluate = "rovio"
# evaluate = "dkf"
draw_fig = False
# draw_fig = False
save_paths = ["direct", "nokf", "lk"]

for seq_name, gt_name in datas:
    for i in range(3):
        rovioEvaluator = VIEvaluator.VIEvaluator()
        if evaluate == "vins":
            rovioEvaluator.bag = "%s/%s/vins_result_no_loop.csv"%(video_path_prefix, seq_name)
            # rovioEvaluator.bag = "%s/%s/vins_result_loop.csv"%(video_path_prefix, seq_name)
        elif evaluate == "rovio":
            # os.system('roslaunch rovio mynt_rosbag_node.launch seq_name:="%s"'% seq_name)
            # os.system('roslaunch rovio bebop2_rosbag_node.launch seq_name:="%s"'% seq_name)
            rovioEvaluator.bag = "%s/%s/rovio_default.bag"%(video_path_prefix, seq_name)
            rovioEvaluator.odomTopic = '/rovio/odometry'
        else:
            # rovioEvaluator.bag = "%s/%s_direct/%s_est.txt"%(est_result_path_prefix, seq_name, seq_name)
            rovioEvaluator.bag = "%s/%s/%s/%s_est.txt"%(est_result_path_prefix, seq_name, save_paths[i], seq_name)
            # rovioEvaluator.bag = "%s/%s/direct/%s_est.csv"%(est_result_path_prefix, seq_name, seq_name)
# rovioEvaluator.pclTopic = '/rovio/pcl'
# rovioEvaluator.extrinsicsTopic = '/rovio/extrinsics0'
        rovioEvaluator.gtFile = gt_path_prefix+gt_name
# rovioEvaluator.gtTopic = '/bluebird/vrpn_client/estimated_transform'
        rovioEvaluator.startcut = 0
        rovioEvaluator.endcut = 0
        rovioEvaluator.doCov = False
        rovioEvaluator.doNFeatures = 0
        rovioEvaluator.doExtrinsics = False
        rovioEvaluator.doBiases = False
        rovioEvaluator.alignMode = 1
        rovioEvaluator.plotLeutiDistances = [0.0]

        gtTimeCol = 1
        gtPosCol = [6, 7, 8]
        gtAttCol = [5, 2, 3, 4]

        if evaluate == "vins":
            estTimeCol = 0
            estPosCol = [1, 2, 3]
            estAttCol = [4, 5, 6, 7]
            estVelCol = None
            estRorCol = None
            delimiter = ','
            timescale = 1e-9
            rovioEvaluator.derMode = 0
            start = 150
        elif evaluate in ["dkf", "rovio"]:
            estTimeCol = 0
            estPosCol = [1, 2, 3]
            estAttCol = [4, 5, 6, 7]
            estVelCol = [9, 10, 11]
            estRorCol = [22, 23, 24]
            delimiter = ' '
            timescale = 1.
            if datasource == 'bebop':
                start = 450
            else:
                start = 150

        rovioEvaluator.initTimedData(td_rovio)
        rovioEvaluator.initTimedDataGT(td_vicon)
        rovioEvaluator.acquireData(estTimeCol, estPosCol, estAttCol, estVelCol, estRorCol, delimiter=delimiter, timescale = timescale, start = start)
        rovioEvaluator.acquireDataGT(gtTimeCol, gtPosCol, gtAttCol)
        td_vicon.averageFilter(smoothwin = 10)
        attIds = td_vicon.getColIDs('att')
        qnorm = np.linalg.norm(td_vicon.d[:, attIds], axis = 1, keepdims = 1)
        td_vicon.d[:, attIds] /= np.repeat(qnorm, 4, axis = 1)


        # attIds = td_rovio.getColIDs('att')
        # td_rovio.d[:, attIds[1:]] *= -1
# print(td_rovio.d.shape)
        rovioEvaluator.getAllDerivatives()
        rovioEvaluator.alignTime()
        rovioEvaluator.alignBodyFrame()
        rovioEvaluator.alignInertialFrame()
        rovioEvaluator.getYpr()
        rovioEvaluator.evaluateSigmaBounds()
        td_rovio.computeRMS('pos', 'att', 'vel', td_vicon, 'pos', 'att', 'vel', 0.0)

        if plotPos and draw_fig: # Position plotting
            plotterPos = Plotter(-1, [3,1],'Position',['','','time[s]'],['x[m]','y[m]','z[m]'],10000)
            if rovioEvaluator.doCov:
                plotterPos.addDataToSubplotMultiple(td_rovio, 'posSm', [1,2,3], ['r--','r--','r--'], ['','',''])
                plotterPos.addDataToSubplotMultiple(td_rovio, 'posSp', [1,2,3], ['r--','r--','r--'], ['','',''])
            plotterPos.addDataToSubplotMultiple(td_rovio, 'pos', [1,2,3], ['r','r','r'], ['','',''])
            plotterPos.addDataToSubplotMultiple(td_vicon, 'pos', [1,2,3], ['b','b','b'], ['','',''])

        if plotVel and draw_fig: # Velocity plotting
            plotterVel = Plotter(-1, [3,1],'Robocentric Velocity',['','','time[s]'],['$v_x$[m/s]','$v_y$[m/s]','$v_z$[m/s]'],10000)
            plotterVel.addDataToSubplotMultiple(td_rovio, 'vel', [1,2,3], ['r','r','r'], ['','',''])
            plotterVel.addDataToSubplotMultiple(td_vicon, 'vel', [1,2,3], ['b','b','b'], ['','',''])

        if plotAtt and draw_fig: # Attitude plotting
            plotterAtt = Plotter(-1, [4,1],'Attitude Quaternion',['','','','time[s]'],['w[1]','x[1]','y[1]','z[1]'],10000)
            plotterAtt.addDataToSubplotMultiple(td_rovio, 'att', [1,2,3,4], ['r','r','r','r'], ['','','',''])
            plotterAtt.addDataToSubplotMultiple(td_vicon, 'att', [1,2,3,4], ['b','b','b','b'], ['','','',''])

        if plotYpr and draw_fig: # Yaw-pitch-roll plotting
            plotterYpr = Plotter(-1, [3,1],'Yaw-Pitch-Roll Decomposition',['','','time[s]'],['roll[rad]','pitch[rad]','yaw[rad]'],10000)
            if rovioEvaluator.doCov:
                plotterYpr.addDataToSubplotMultiple(td_rovio, 'yprSm', [1,2,3], ['r--','r--','r--'], ['','',''])
                plotterYpr.addDataToSubplotMultiple(td_rovio, 'yprSp', [1,2,3], ['r--','r--','r--'], ['','',''])
            yprIdsEst = td_rovio.getColIDs('ypr')
            plotterYpr.addDataToSubplotMultiple(td_rovio, 'ypr', [1,2,3], ['r','r','r'], ['','',''])
            plotterYpr.addDataToSubplotMultiple(td_vicon, 'ypr', [1,2,3], ['b','b','b'], ['','',''])

        if draw_fig:
            raw_input("Press Enter to continue...")
