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
    stop = False
    next = []
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    code_run_count=0
    red_points = []
    red_points_new=[]
    f=[]
    count = 0
    class PIDController:
        def __init__(self, Kp, Ki, Kd, setpoint=0):
            self.Kp = Kp
            self.Ki = Ki
            self.Kd = Kd
            self.setpoint = setpoint
            self.previous_error = 0
            self.integral = 0

        def update(self, current_value, dt):
            error = self.setpoint - current_value
            self.integral += error * dt
            derivative = (error - self.previous_error) / dt
            output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
            self.previous_error = error
            return output

    def calculate_heading(bot_position, target_position):
        # """Calculate the heading from the bot's current position to the target."""
            delta_x = target_position[0] - bot_position[0]
            delta_y = target_position[1] - bot_position[1]
            z= math.atan2(delta_y, delta_x)
            print(z)
            return z 
    
        

    def distance_to_target(bot_position, target_position):
        """Calculate the distance from the bot's current position to the target."""
        delta_x = target_position[0] - bot_position[0]
        delta_y = target_position[1] - bot_position[1]
        k=math.sqrt(delta_x**2 + delta_y**2)
        print(k)
        return k 
    while True:
        
        code_run_count=code_run_count+1
        # print(code_run_count)
        # Get the handles for the robot and its joints
        robot = sim.getObject('/crn_bot')
        l_motor = sim.getObject('/crn_bot/joint_l')
        r_motor = sim.getObject('/crn_bot/joint_r')
        v1 = sim.getObject('/vision_sensor')
        # print (v1)  
        
        #if velocity of left and right wheel is different and moreover in oppsite direction than it will rotate 
       
        # sim.setJointTargetVelocity(l_motor,-3)
        # sim.setJointTargetVelocity(r_motor,6)7
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
        #   aruco.drawDetectedMarkers(image, corners, ids)
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
        angle_rad_copy=round(angle_rad,2)
        # FINAL IMAGE DISPLAY RESOLUTION SETTINGS CHANGED FOR VIEWING PURPOSE :)
        # image_2 = cv2.resize(image_2, (512,512)) 
        image_2=cv2.putText(image_2,f'x={center_x},y={center_y},theta={angle_rad_copy}',(25,25),fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.75, color=(0,0,0,), thickness=1)
        # cv2.imshow("Captured Image Press Q to Quit", image_2)
        
        if code_run_count<=1:
            image_c =image_2.copy()
            
            hsv_image = cv2.cvtColor(image_c, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, lower_red, upper_red)
            # cv2.imshow("Captured Image Press Q to Quit", mask)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for i, contour in enumerate(contours):
                
                x, y, w, h = cv2.boundingRect(contour)
                print(f"Detected red point {i+1} at: x={x}, y={y}")
                M = cv2.moments(contour)
                # print(M)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    red_points.append([cX,cY,0])
                    red_points_new.append([cX,cY])
                
                # red_points_new.reverse()
                    # print(f"Centroid of red color at: x={cX}, y={cY}")
            for p in red_points:
                if p[2] == 0:
                    print(f"Number of uncrossed points {len(red_points)}")     
        else:
            pass         
        
        
          # Initialize PID controllers for heading and speed control
        heading_pid = PIDController(Kp=0.1, Ki=0.0, Kd=0.05)
        speed_pid = PIDController(Kp=0.1, Ki=0.0, Kd=0.01)

        # # List of target coordinates
        
        target_points = red_points_new # Replace with your coordinates    
    

        
        a = [3, 2, 1, 0, 4, 5, 7 , 6]
        pi = math.pi
        angles = [-pi/2, 0, -pi/2, pi, pi/2,pi,-pi/2]
        
        # # Main loop to move the bot to each pointttttt
        
        target = target_points[a[count]]
        # Step 1: Get the bot's current position and heading
        current_position = [center_x, center_y]  # Replace with actual function
        current_heading = angle_rad    # Replace with actual function

#         # Step 2: Calculate the desired heading and distance to the target
        desired_heading =calculate_heading(current_position,target_points[a[count]])
        print("desired heading and current ",desired_heading, angle_rad)

        distance = distance_to_target([center_x,center_y],target)  
        print("distance is = ", distance)
        

#         # Step 3: Update the heading PID controller
        dt = (0.1)  # Time step between updates
        
        heading_error = desired_heading - current_heading
        heading_control_signal = heading_pid.update(heading_error, dt)

#         # Step 4: Update the speed PID controller
        speed_control_signal = speed_pid.update(distance, dt)

        # Step 5: Apply control signals to the motors
#         # You might need to convert the heading control signal to left and right wheel velocities


        left_motor_velocity = speed_control_signal - heading_control_signal
        right_motor_velocity = speed_control_signal + heading_control_signal
        # print(left_motor_velocity, right_motor_velocity)
        if distance > 100:
            sim.setJointTargetVelocity(l_motor, -1)
            sim.setJointTargetVelocity(r_motor, 1)
        elif distance > 75:
            sim.setJointTargetVelocity(l_motor, -1)
            sim.setJointTargetVelocity(r_motor, 1)
               
        elif distance> 25 and distance < 50:
            sim.setJointTargetVelocity(l_motor, -0.5)
            sim.setJointTargetVelocity(r_motor, 0.5)
        elif distance > 15 and distance < 25:
            sim.setJointTargetVelocity(l_motor, -0.25)
            sim.setJointTargetVelocity(r_motor, 0.25)
        elif distance > 10 and distance < 15:
            sim.setJointTargetVelocity(l_motor, -0.125)
            sim.setJointTargetVelocity(r_motor, 0.125)  
            
            
        # Step 6: Check if the bot has reached the  target (e.g., within a small threshold)
        if  distance < 15*((count+1)*.99) or stop==True:
            next = (calculate_heading(current_position,target_points[a[count+1]]))
            stop = True
            # Move to the next target point
            # print(f"Reached target: {target}")
            sim.setJointTargetVelocity(l_motor, 0)
            sim.setJointTargetVelocity(r_motor, 0)
            
            # print(abs(current_heading-desired_heading))
            if  abs(current_heading-angles[count])<pi*0.0125/6:
                sim.setJointTargetVelocity(l_motor, +0.025/6)
                sim.setJointTargetVelocity(r_motor, +0.025/6)
            if  abs(current_heading-angles[count])<pi*0.0125/4:
                sim.setJointTargetVelocity(l_motor, +0.025/4)
                sim.setJointTargetVelocity(r_motor, +0.025/4)
            elif  abs(current_heading-angles[count])<pi*0.0125/2:
                sim.setJointTargetVelocity(l_motor, +0.025/2)
                sim.setJointTargetVelocity(r_motor, +0.025/2)
            
            elif  abs(current_heading-angles[count])<pi*0.0125:
                sim.setJointTargetVelocity(l_motor, +0.025)
                sim.setJointTargetVelocity(r_motor, +0.025)
                
            elif abs(current_heading-angles[count])<pi*0.0250:
                sim.setJointTargetVelocity(l_motor, +0.05)
                sim.setJointTargetVelocity(r_motor, +.05)
            elif not abs(current_heading-angles[count])<pi*0.05:
                print("next is: ", next)
                print(abs(current_heading)-angles[count])
                sim.setJointTargetVelocity(l_motor, +.1)
                sim.setJointTargetVelocity(r_motor, +.1)
            
            
                
                
            else:
                stop = False
                print("terriiiii")
                # time.sleep(10)
                count+=1
                sim.setJointTargetVelocity(l_motor, +0)
                sim.setJointTargetVelocity(r_motor, +0)
                
                    
        
        
            # while current_heading < desired_heading:
            #     sim.setJointTargetVelocity(l_motor, -1)
            #     sim.setJointTargetVelocity(r_motor, -1)
                
                
            
            # count+=1
            
            
            
        elif not stop:
            sim.setJointTargetVelocity(l_motor, -1)
            sim.setJointTargetVelocity(r_motor,1)


        # Optionally stop the motors when the last target is reached
        
     
      # KEY BREAK Q
        cv2.imshow("Captured Image Press Q to Quit", mask)
        for i, point in enumerate(red_points):
            cv2.putText(mask,f'{i}',(point[0],point[1]),fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.75, color=(255,255,255,), thickness=1)
        
        cv2.putText(mask,f'THE BOT IS HERE',(center_x,center_y),fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255,255,255,), thickness=1)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("quitting")
            # sim.stopSimulation()
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
