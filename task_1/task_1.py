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
    
    while True:
        # Get the handles for the robot and its joints
        robot = sim.getObject('/crn_bot')
        l_motor = sim.getObject('/crn_bot/joint_l')
        r_motor = sim.getObject('/crn_bot/joint_r')
        
            


        # Set the velocities to make the robot move in a circular path
        # Left motor should move faster than the right motor
        sim.setJointTargetVelocity(l_motor, -20)
        sim.setJointTargetVelocity(r_motor, 30)

        

    
    
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