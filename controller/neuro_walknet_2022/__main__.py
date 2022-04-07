'''
Main file for hexapod simulations using the hector simulator.

The file initialises the environment and loads the robot description from xml.
Then it continues with the controller.
'''
import comminter
import sys, time
import inspect, os

from Hector.RobotF import Robot
import geomparse

##  Getting the command line arguments.
def _args():
    import argparse
    parser = argparse.ArgumentParser(description="WALKNET start skript ")
    parser.add_argument("-t", "--simulationDuration", action="store", default=float('inf'), type=float,
        dest="simulationDuration", help="Duration of the simulation in seconds (in simulation time).")
    parser.add_argument("-l", "--log", action="store_true", default=False,
        dest="log", help="Log output.")
    return parser.parse_args()

##  Initialisation of the environment.
#   Building up the communication client and starting the main loop.
def init_environment(args):
    controllerFrequency=100 # Define the frequency with which the controller should run.

    # Create the communication interface
    communication_interface=comminter.CommunicationInterface()
    protocolXmlDirectory="../hector/BioFlexBusProtocolXmls/"
    communication_interface.ParseProtocolXmls([protocolXmlDirectory+protocolXml for protocolXml in ["BIOFLEX_1_PROT.xml", "BIOFLEX_ROTATORY_1_PROT.xml", "BIOFLEX_ROTATORY_CONTROL_1_PROT.xml", "BIOFLEX_ROTATORY_ERROR_PROT.xml", "SIMSERV_1_PROT.xml", "IMU_PROT.xml", "PRESSURE_SENSOR_PROT.xml"]]) # Parse the xml files that contain the command-protocol definitions

    # Create a communication client for the timing of the simulation
    simServ=communication_interface.CreateBfbClient(14, ["SIMSERV_1_PROT"])

    # Setup the robot and its environment in the simulation
    filename = inspect.getframeinfo(inspect.currentframe()).filename
    path = os.path.dirname(os.path.abspath(filename))

    # Send the geometry of the universe and the robot to the simulation.
    simServ.geometryXml=open(path+'/../../GeometryXmls/UniverseWithPlane.xml','r').read()
    simServ.geometryXml=open(path+'/../../GeometryXmls/Hector.xml','r').read()
    geometryData = geomparse.parseHectorXml(open(path+'/../../GeometryXmls/StartingPosture.xml','r').read())

    # Create a communication client for the inertial measurement unit
    imu=communication_interface.CreateBfbClient(120, ["IMU_PROT"])

    # Module main executor
    from ProcessOrganisation.ProcessModule.ProcessModuleQueuedExecution import ProcessModuleQueuedExecution
    mainProcessModuleExecution = ProcessModuleQueuedExecution(debug_time=False)

    # Build up the individual Modules

    # Build up the Robot - connects to the simulator
    robot = Robot("Robot_object", geometryData, communication_interface)
    mainProcessModuleExecution.add_module(robot)

    # Drive robot into starting posture
    from controller.neuro_walknet_2022.LiftLeg import LiftLeg
  #  initLift = LiftLeg("initlift", robot.front_left_leg, False )
  #  initLift = LiftLeg("initlift", robot.middle_left_leg, False )
  #  initLift = LiftLeg("initlift", robot.hind_left_leg, True, 1.1 )
  #  initLift = LiftLeg("initlift", robot.front_right_leg )
  #  initLift = LiftLeg("initlift", robot.middle_right_leg )
  #  initLift = LiftLeg("initlift", robot.hind_right_leg, False, 1.1 )
  #  mainProcessModuleExecution.add_control_module_to_queue(initLift, 3.0)

############################
## MAIN PART OF THE program
## CALL THE CONTROLLER
############################

    # Build up the individual Modules
    # 1. The Controller
    #from controller.holk.SimpleMovement import SimpleMovement
    #from controller.holk.SimpleMovement import SimpleMovement
    from controller.neuro_walknet_2022.NeuroWalknet import NeuroWalknet
    controller_obj = NeuroWalknet("neuro_walknet", robot)
    mainProcessModuleExecution.add_control_module_to_queue(controller_obj, float('Inf'))

    ####################
    # Turn visualizations on and off.

    # Footfall pattern
    from Visualizations.PEPVisualizationModule import PEPVisualizationModule
    pepVisualization = PEPVisualizationModule("pepVisualization", controller_obj)
    mainProcessModuleExecution.add_module(pepVisualization)
##
# Stimulus over time visualization #####
    from Visualizations.StimulusOverTime import StimulusVisualizationModule
    stimulusVisualization = StimulusVisualizationModule("stimulusVisualization", controller_obj, 1) # leg nr
    mainProcessModuleExecution.add_module(stimulusVisualization)

    # Footpositions in robot CS
#    from Visualizations.FootPointVisualizationModule import FootPointVisualizationModule
 #   fpVisualization = FootPointVisualizationModule("fpVisualization", controller_obj, robot)
  #  mainProcessModuleExecution.add_module(fpVisualization)

    # Record start of swing movements
#    from Visualizations.PEPWriterModule import PEPWriterModule
 #   pepWriter = PEPWriterModule("pepWriter", controller_obj)
  #  mainProcessModuleExecution.add_module(pepWriter)

    # CPG Activation figure Retractor Borgm, Sax
#    from Visualizations.CPGVisualizationModule import CPGVisualizationModule
 #   cpgVisualization = CPGVisualizationModule("cpgVisualization", controller_obj)
  #  mainProcessModuleExecution.add_module(cpgVisualization)

    # Depressor Activation figure Kneb  and CRoachfast
#    from Visualizations.DepressorVisualizationModule import DepressorVisualizationModule
 #   depressorVisualization = DepressorVisualizationModule("depressorVisualization", controller_obj)
  #  mainProcessModuleExecution.add_module(depressorVisualization)

   # Leg Position over time
#    from Visualizations.PositionVisualizationModule import PositionVisualizationModule
 #   positionVisualization = PositionVisualizationModule("positionVisualization", robot)
  #  mainProcessModuleExecution.add_module(positionVisualization)

  # torque over time ( or joint positions)
    #from Visualizations.NeuronActivationVisualizationModule import NeuronActivationVisualizationModule # torque
    #neuronActVisualization = NeuronActivationVisualizationModule("neuronActivationVisualization", controller_obj, 5) # leg nr
    #mainProcessModuleExecution.add_module(neuronActVisualization)


##
# Motor outputs for a single leg visualization #####
    from Visualizations.MotorOutputsSingleLegVisualization import MotorOutputsSingleLegVisualization #
    motorOutputVisualization = MotorOutputsSingleLegVisualization("motorOutputVisualization", controller_obj, 1)  # leg nr
    mainProcessModuleExecution.add_module(motorOutputVisualization)

#    from Visualizations.JointPositionVisualizationModule import JointPositionVisualizationModule
 #   jointPosVisualization = JointPositionVisualizationModule("jointPosVisualization", controller_obj, 5)  # leg nr
  #  mainProcessModuleExecution.add_module(jointPosVisualization)

############################
## CONNECTION TO Simulator
############################

    # Load the module for the timing of the simulator
    from ProcessOrganisation.SimulatorModule.SimulatorTimerModule import SimulatorTimerModule as SimulatorTimerModule
    simulatorTimer = SimulatorTimerModule("Simulator_Timer", robot, controllerFrequency)
    mainProcessModuleExecution.add_module(simulatorTimer)

    try:

        simulationTime = 0 # In order to let the simulation run in realtime, the simulation time is tracked and compared to the real time in order to let the simulation sleep if it's too fast.
        realTimeOffset = time.time() # The current time at the beginning of the simulation

        mainProcessModuleExecution.init_all_modules()

        # THE MAIN LOOP
        while (simulationTime <= args.simulationDuration):
            start = time.time()

            mainProcessModuleExecution.execute_complete_step(simulationTime)

            simulationTime+=1/controllerFrequency # Update the variable holding the current simulation time.
            communication_interface.NotifyOfNextIteration() # Tell the communication interface that a new controller iteration has begun.

            if args.log == True:
                print(timeStamp,";",(time.time()-start))

    except KeyboardInterrupt as err:
        print("\n terminated by user", err)

## Main method
if __name__ == "__main__":
    if sys.version_info < (3, 2):
        print("This programm request Python 3.2 or higher")
        quit()
    args = _args()
    init_environment(args)
