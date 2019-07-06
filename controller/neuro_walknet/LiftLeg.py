#Communication
from Hector.RobotF import Robot

# ProcessingModules are automatically called with a control frequency.
from ProcessOrganisation.ProcessModule.ProcessingModule import ProcessingModule

class LiftLeg (ProcessingModule):
    """
    Simple Movement controller for lifting a single leg

    ...

    Attributes
    ----------
    name : str
        Name of the controller    
    leg : LegF
        Controlled leg object
    direction : Bool
        direction of movement (true = positive velocity)
    target : float
        target value for the beta joint

    Methods
    -------
    processing_step(timeStamp)
        Called automatically from the simulation loop as a step of the controller.
    """

    def __init__(self, name, leg, direction=True, target=1.2):
        """
        Parameters
        ----------
        name : str
            Name of the controller.
        leg : LegF
            reference to the leg object from which current sensor values
            can be read and commands are send to.
        direction : Bool
            direction of movement (true = positive velocity)
        target : float
            target value for the beta joint
        """
        self.name = name
        self.leg = leg
        self.direction = direction
        self.target=target
        ProcessingModule.__init__(self, name)

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
        anglesRead = self.leg.beta.inputPosition
        if (self.direction):
            if (anglesRead < self.target) :
                self.leg.beta.desiredValue_ISC = 0.1
            else:
                self.leg.beta.desiredValue_ISC = 0.
        else:
            if (anglesRead > -self.target) :
                self.leg.beta.desiredValue_ISC = -0.1
            else:
                self.leg.beta.desiredValue_ISC = 0. 

    ## Init the module - called before it is iterated over all the modules
    def init_module(self):
        print('Init lift leg')
