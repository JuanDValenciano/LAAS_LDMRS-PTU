#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# ------------------------------------------

# LAAS-CNRS: Robotic and Interaction Systems
# SICK LDMRS, Platine Light
"""
Created on Thu Sep 28 16:01:59 2017

@author: haroldfmurcia
"""
# ------------------------------------------
import  rospy, os, sys, time, math
import  scan3D

import pickle

path_data = os.getcwd() + "/data"
path_bin  = "/home/haroldfmurcia/ROS/bin"

def initScan():
    PTU   = scan3D.ptu_light(device='/dev/ttyUSB0')
    LiDAR = scan3D.sickLDMRS(ip='192.168.0.1',port='12002', scanFindx=1)
    pointCloud = scan3D.LDMRS_CLOUD()
    pointCloud.description = raw_input("Short description of Data: ")
    pointCloud.ip = LiDAR.ip
    pointCloud.port = LiDAR.port
    pointCloud.scanFreq = LiDAR.scanFindx
    return PTU, LiDAR, pointCloud

def TiltScan(PTU, LiDAR, cloud, TiltMin, TiltMax, stepRes):
    #Phisical TiltMax= maxAng #2354-2
    #Phisical TiltMin= minAng #-3569+10
    radsPerStep = PTU.TiltRes
    degPerStep = radsPerStep*180.0/math.pi
    MAX_T = PTU.TiltMax
    MIN_T = PTU.TiltMin
    if (TiltMax > MAX_T*degPerStep):
        print "\n Error: \n"
        print "\n Maximum indicated angle exceeds maximum allowed value \n"
        return
    if (TiltMin < MIN_T*degPerStep):
        print "\n Error: \n"
        print "Minimum indicated angle exceeds minimum allowed value \n"
        return
    print ("STARTING SCAN -----------------------------------------------------------")
    print "\n Go2 Initial Position: minimum Tilt: " + str(TiltMin) + "\n"
    PTU.Go2Position(1,TiltMin,0)
    PTU.Go2Position(0,0,0)
    PTU.PlatineLight_getPos(1,0)  # to obtain Tilt in Degs
    lastTime=time.time()
    stepMin = int( TiltMin/degPerStep )
    stepMax = int( TiltMax/degPerStep )
    delta   = int( stepRes/degPerStep )   # [degs] / [degs/step]
    for k in range (stepMin,stepMax, delta):
        PTU.Go2Position(1,k,2)
        error = abs(k*degPerStep - PTU.Tilt)
        # Tilt_speed = (Tilt[k]-Tilt[k-1])/dT :
        T_speed = PTU.Tilt - PTU.Tilt_1
        # Tilt_acc = (Tilt_speed[k]-Tilt_speed[k-1])/dT :
        T_acc = PTU.Tilt_2 - PTU.Tilt_1
        while ((error > 1e-6) or ((T_speed !=0) and (T_acc !=0)) ):
            error = abs(k*degPerStep - PTU.Tilt)
            T_speed = PTU.Tilt - PTU.Tilt_1
            T_acc   = PTU.Tilt_2 - PTU.Tilt_1
        lastScan=LiDAR.LDMRS_getData(PTU.Tilt,PTU.Pan)
        cloud.add_ldmrsScan(lastScan)
        dT=time.time()-lastTime
        lastTime=time.time()
        print ("\t" + "Tilt: " + str(PTU.Tilt) + "\t" "deltaTime: " + str(dT))
    print("----------------------------------------------------------- END OF TILT SCAN")
    # Go2Zero
    PTU.Go2Position(1,0,2)
    PTU.Go2Position(0,0,2)
    return cloud

def saveCloud(cloud, fName):
    fileName= fName + "_" + time.strftime("%d-%m-%y")+'-'+time.strftime("%I-%M-%S")
    completeName = os.path.join(path_data, fileName + '.pkl')
    print "\n -- SAVING DATA --"
    print "\t File: " + fileName
    print "\t Path: " + path_data
    output = open(completeName, 'w')
    #pickle.dump(cloud, output)
    # Pickle the list using the highest protocol available.
    pickle.dump(cloud, output, -1)
    output.close()
# ------------------------------------------------------------------------------
# MAIN
# ------------------------------------------------------------------------------

if __name__ == "__main__":
    # Init Basic 3d Scan Data
    fileName = raw_input("Name of the pointCloud-File: ")
    [PTU, LiDAR, pointCloud]=initScan()
    try:
        os.system(path_bin + '/platine_light-ros&')
        os.system(path_bin + '/LiDARldmrs-ros&')
        time.sleep(1)
    except:
        print("Warning: Nodes")
    try:
        rospy.init_node('scan3D_client_py')
        time.sleep(1)
        PTU.PlatineLight_open_client()
        PTU.PlatineLight_set_baudRate()
        LiDAR.LDMRS_open_client()
        # scanning parameters in deg
        scan_Tilt_min = -0.1
        scan_Tilt_max = 0.1
        scan_Tilt_step= 1/50.0
        pointCloud= TiltScan(PTU, LiDAR, pointCloud, scan_Tilt_min, scan_Tilt_max, scan_Tilt_step)
        PTU.PlatineLight_close()
        PTU.PlatineLight_kill()
        LiDAR.LDMRS_close_client()
        saveCloud(pointCloud, fileName)
        LiDAR.LDMRS_kill()
        sys.exit()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
