
#Communication
from Hector.RobotF import Robot
import numpy

from controller.neuro_walknet_2022.NeuroLegMovement import NeuroLegMovement
from controller.neuro_walknet_2022.NeuroCoordinationRules import NeuroCoordinationRules

from ProcessOrganisation.ProcessModule.ProcessingModule import ProcessingModule

class NeuroWalknet (ProcessingModule):
    '''
    NeuroWalknet is a neuronal based controller for insect walking.
    It consists of a
        - decentralized structure - each leg has its individual controller
            Realized in NeuroLegMovement.py
        - coordination rules between the individual controllers
            Realized in NeuroCoordinationRules.py
            and applied after the calculations of neuron activity on the leg level.

    The controller is used for the control of the hexapod robot hector.
    ...

    Attributes
    ----------
    name : str
        Name of the controller
    robot : RobotF
        Controlled robot object

    Methods
    -------
    processing_step(timeStamp)
        Called automatically from the simulation loop as a step of the controller.
    '''

    def __init__(self, name, robot=None):
        """
        Parameters
        ----------
        name : str
            Name of the controller.
        robot : RobotF, optional
            reference to the robot object from which current sensor values
            can be read and commands are send to.
        """
        self.name = name
        ProcessingModule.__init__(self, name)
        # Instances
        if robot is None:
            self.robot = Robot()
        else:
            self.robot = robot
        self.count = 0

        # Six leg controllers are initialized and each gets a reference
        # for the specific leg.
        self.controller_objs = [None, None, None, None, None, None]
        controller_obj_FL = NeuroLegMovement("controller_FL", robot.front_left_leg)
        self.controller_objs[0] = controller_obj_FL
        controller_obj_FR = NeuroLegMovement("controller_FR", robot.front_right_leg)
        self.controller_objs[1] = controller_obj_FR
        controller_obj_ML = NeuroLegMovement("controller_ML", robot.middle_left_leg)
        self.controller_objs[2] = controller_obj_ML
        controller_obj_MR = NeuroLegMovement("controller_MR", robot.middle_right_leg)
        self.controller_objs[3] = controller_obj_MR
        controller_obj_HL = NeuroLegMovement("controller_HL", robot.hind_left_leg)
        self.controller_objs[4] = controller_obj_HL
        controller_obj_HR = NeuroLegMovement("controller_HR", robot.hind_right_leg)
        self.controller_objs[5] = controller_obj_HR

        # Coordination rules are loaded
        self.coordination_rules = NeuroCoordinationRules(self.controller_objs)

        # Settings for joint parameters
        # Original value for all joints was 855 (and 0.4 for damping)
        ov = 855 #200 #855
        spf = 850 # 800
        spm = 450 #  400 450 500 600
        sph = 850 #800 700
        #robot.front_left_leg.beta.dampingConstant = 0.4
        robot.front_left_leg.beta.springConstant = spf #100
        robot.middle_left_leg.beta.springConstant = spm #100
        robot.hind_left_leg.beta.springConstant = sph #100
        robot.front_right_leg.beta.springConstant = spf #100
        robot.middle_right_leg.beta.springConstant = spm #100
        robot.hind_right_leg.beta.springConstant = sph #100

    #\param timeStamp       current simulator time
    def processing_step(self, timeStamp):
        """
        CONTROL STEP
        Is called from the simulation loop: updates sensory input values
        and advances the neuronal network afterwards.
        Last, controller outputs are send to the joint motors.

        Parameters
        ----------
        timestamp : float
            Current timestep.
        """
        # Update sensory values from robot object.
        for controller in self.controller_objs:
            if controller:
                controller.update_joint_positions()
        # Compute the neural networks
        # (is done with a 10 times higher frequency:
        #  simulator runs at 100 Hz, but neurons operate on millisecond scale)
        for i in range(0,10):
            self.count = self.count +1
            # Update Leg networks.
            for controller in self.controller_objs:
                if controller:
                    controller.update_leg_controller(timeStamp)
            # Update coordination influences.
            self.coordination_rules.update_coordination_rules(timeStamp)

            # Show current state of the leg controller
          #  self.controller_objs[0].output_current_state()  # FL
          #  self.controller_objs[2].output_current_state()  # ML
          #  self.controller_objs[4].output_current_state()  # HL
            self.controller_objs[1].output_current_state()  # FR
          #  self.controller_objs[3].output_current_state()  # MR
          #  self.controller_objs[5].output_current_state()  # HR

        # Send motor commands to robot object (joint velocities).
        for controller in self.controller_objs:
            if controller:
                controller.send_control_velocities()

    ## Init the module - called before it is iterated over all the modules
    def init_module(self):
        print('Init neuro walknet')
