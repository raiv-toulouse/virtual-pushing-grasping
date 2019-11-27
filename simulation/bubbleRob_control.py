import vrep  # access all the VREP elements
import sys
import time

vrep.simxFinish(-1) #just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start aconnection
if clientID!=-1:
    print ("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")
err_code,l_motor_handle = vrep.simxGetObjectHandle(clientID,"bubbleRob_leftMotor", vrep.simx_opmode_blocking)
err_code,r_motor_handle = vrep.simxGetObjectHandle(clientID,"bubbleRob_rightMotor", vrep.simx_opmode_blocking)

err_code = vrep.simxSetJointTargetVelocity(clientID,l_motor_handle,1.0,vrep.simx_opmode_streaming)
err_code = vrep.simxSetJointTargetVelocity(clientID,r_motor_handle,1.0,vrep.simx_opmode_streaming)
time.sleep(5)  # Nécessaire sinon la communication est coupée avant même que Vrep ne reçoive les commandes
# voir : http://www.forum.coppeliarobotics.com/viewtopic.php?t=5545
#vrep.simxGetPingTime(clientID);  # Autre solution