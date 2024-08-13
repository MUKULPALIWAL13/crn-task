'''
*****************************************************************************************
*
* All the functions in this file are used to control the robot in the CoppeliaSim
* simulation via APIs
*
*****************************************************************************************
'''

import sys
import traceback
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import cv2
import numpy as np
import cv2.aruco as aruco
import math

############################## GLOBAL VARIABLES ######################################



############################ USER DEFINED FUNCTIONS ##################################



################################ MAIN FUNCTION #######################################

def simulator(sim):
    """
    Purpose:
    ---
    This function should implement the control logic for the given problem statement
    You are required to actuate the rotary joints of the robot in this function, such that
    it does the required tasks.

    Input Arguments:
    ---
    `sim`    :   [ object ]
        ZeroMQ RemoteAPI object

    Returns:
    ---
    None

    Example call:
    ---
    simulator(sim)
    """
    
    #### YOUR CODE HERE ####
    global robot 
    global v1
    global detectionCount
    global l1
    global l2
    
    
    
    while True:
        # Get the handles for the robot and its joints
        robot = sim.getObject('/crn_bot')
        l_motor = sim.getObject('/crn_bot/joint_l')
        r_motor = sim.getObject('/crn_bot/joint_r')
        v1 = sim.getObject('/vision_sensor')
        # print (v1)  
        
        #if velocity of left and right wheel is different and moreover in oppsite direction than it will rotate 
       
        sim.setJointTargetVelocity(l_motor,-3)
        sim.setJointTargetVelocity(r_motor,6)
        # sim.handleVisionSensor(int(v1))
       
        image, resolution = sim.getVisionSensorImg(int(v1))
        # print(resolution)
        # print(image)
        uint8Numbers = sim.unpackUInt8Table(image)
        # print(np.array(uint8Numbers).shape)
        # print(uint8Numbers)
        




#  img buffer to a NumPy array
        image_2 = np.array(uint8Numbers, dtype=np.uint8)
        # print(image)
        # print(image.shape)
        image_2 = image_2.reshape((1024, 1024, 3))
        # converting RGB TO BGR
        image_2 = cv2.cvtColor(image_2, cv2.COLOR_RGB2BGR)
         
      #  USING ARUCO MARKERS DICTIONARY 4X4_1000
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
        parameters = aruco.DetectorParameters()
        #  CHECK FOR IMAGE LOAD 
        image = image_2
        if image is None:
          print(f"Error: Unable to load imag")
        else:
          print(f"Image loaded successfully")
          gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
          #  DETECTION OF MARKER AND ITS CORNER POINTS 
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        # MARKING MARKER BOUNDARIES
        if ids is not None:
          print("Detected marker IDs: ", ids)
          print("Marker corners: ", corners)
          aruco.drawDetectedMarkers(image, corners, ids)
        else:
          print("No markers detected.")
        output_path = 'detected_markers.jpg'
        cv2.imwrite(output_path, image)
        print(f"Saved detected markers image as {output_path}")
        # CONVERTING TO CORNER COOORDINATES TO GEOMETRIC COORDINATES
        if ids is not None:
          for i in range(len(ids)):
           c = corners[i][0]  
           print(f"Marker ID {ids[i]} corners:")
          for j in range(4):
            print(f"Corner {j+1}: {c[j]}")
        if ids is not None:
         for i in range(len(ids)):
          c = corners[i][0]  
          center_x = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
          center_y = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)
          print(f"Marker ID {ids[i]} center: ({center_x}, {center_y})")
        if ids is not None:
         for i in range(len(ids)):
          c = corners[i][0]
        # Calculate the angle (orientation) using the top-left and top-right corners
          dx = c[1][0] - c[0][0]
          dy = c[1][1] - c[0][1]
          # IMPORT MATH 
          angle = math.degrees(math.atan2(dy, dx))
        print(f"Marker ID {ids[i]} orientation: {angle} degrees")
        angle_rad = angle*math.pi/180
        angle_rad=round(angle_rad,2)
        # FINAL IMAGE DISPLAY RESOLUTION SETTINGS CHANGED FOR VIEWING PURPOSE :)
        image_2 = cv2.resize(image_2, (512,512)) 
        image_2=cv2.putText(image_2,f'x={center_x},y={center_y},theta={angle_rad}',(25,25),fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.75, color=(0,0,0,), thickness=1)
        cv2.imshow("Captured Image Press Q to Quit", image_2)
          
      # KEY BREAK Q
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("quitting")
            break
        
    
    return None
    
######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THE MAIN CODE BELOW #########

if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    try:

        ## Start the simulation using ZeroMQ RemoteAPI
        try:
            return_code = sim.startSimulation()
            if sim.getSimulationState() != sim.simulation_stopped:
                print('\nSimulation started correctly in CoppeliaSim.')
            else:
                print('\nSimulation could not be started correctly in CoppeliaSim.')
                sys.exit()

        except Exception:
            print('\n[ERROR] Simulation could not be started !!')
            traceback.print_exc(file=sys.stdout)
            sys.exit()

        ## Runs the robot navigation logic written by participants
        try:
            simulator(sim)
            time.sleep(5)

        except Exception:
            print('\n[ERROR] Your simulator function threw an exception, kindly debug your code!')
            print('Stop the CoppeliaSim simulation manually if required.\n')
            traceback.print_exc(file=sys.stdout)
            print()
            sys.exit()

        
        ## Stop the simulation using ZeroMQ RemoteAPI
        try:
            return_code = sim.stopSimulation()
            time.sleep(0.5)
            if sim.getSimulationState() == sim.simulation_stopped:
                print('\nSimulation stopped correctly in CoppeliaSim.')
            else:
                print('\nSimulation could not be stopped correctly in CoppeliaSim.')
                sys.exit()

        except Exception:
            print('\n[ERROR] Simulation could not be stopped !!')
            traceback.print_exc(file=sys.stdout)
            sys.exit()

    except KeyboardInterrupt:
        ## Stop the simulation using ZeroMQ RemoteAPI
        return_code = sim.stopSimulation()
        time.sleep(0.5)
        if sim.getSimulationState() == sim.simulation_stopped:
            print('\nSimulation interrupted by user in CoppeliaSim.')
        else:
            print('\nSimulation could not be interrupted. Stop the simulation manually.')
            sys.exit()
