from .LegF import Leg
from .BodyF import Body
from . import RobotSettings as RSTATIC
import numpy
from ProcessOrganisation.ProcessModule.ProcessingModule import ProcessingModule
from tools.FreezableF import Freezable as Freezable
from Hector.DriveSafetyCheck import DriveSafetyCheck
import time
##
#	Robot object - encapsulating access to variables of robot 
#	(simulator or real). 
#	The robot is segmented into 
#		- three body segments
#		- six legs
#	Robot is a ProcessingModule - therefore it is automatically updated.
#	This is done in the pre-processing step.
##
class Robot(ProcessingModule, Freezable):

	##	Initialisation of the robot object.
	#	The legs are constructed and switched to the right kind of leg,
	#	and the body segments are initialized.
	#
	#	Missing are the joints between the body segments.
	#
	#	@param name name of the module
	#	@param geometryData robot xml file
	#	@param communication_interface reference to the active communication
	def __init__(self, name, geometryData, communication_interface):
		ProcessingModule.__init__(self, name)
		self.communication_interface = communication_interface
		
		leg_names=RSTATIC.leg_names
		legs=[]
		self.relative_leg_onsets=[]
		for leg_name in leg_names:
			beta_direction = True # For all drives, for which a positive rotation of beta will lift the leg
			if leg_name in ['front_left_leg', 'middle_left_leg','hind_right_leg']:
				beta_direction = False # For all drives, for which a positive rotation of beta will lower the leg
			temp_leg=Leg(leg_name, geometryData[leg_name]['bfb_client_ids'], geometryData[leg_name]['segment_lengths'], geometryData[leg_name]['segment_masses'], geometryData[leg_name]['segment_centers_of_mass'], geometryData[leg_name]['phi'], geometryData[leg_name]['psi'], geometryData[leg_name]['chi'], beta_direction, communication_interface)
			setattr(self,leg_name,temp_leg)
			legs.append(temp_leg)
			self.relative_leg_onsets.append(geometryData[leg_name]['relative_onset'])
		self.legs=tuple(legs)
		
		# Body segments
		self.front_body = Body("front_body")
		self.middle_body = Body("middle_body")
		self.hind_body = Body("hind_body")
		self.driveSafetyCheck = None
		
		
		self.center_of_mass_of_body_segments=numpy.array([0.22, 0, 0.05])
		self.mass_of_body_segments=6.4
		
		self.frozen=True

	##
	#	Initialization of the robot module.
	#	Create the safety drive check if this has not been registered already.
	def init_module(self):
		if (self.driveSafetyCheck == None):
			print("Created Standard Drive Safety Check") 
			self.driveSafetyCheck = DriveSafetyCheck(self)
		for leg_nr, leg in enumerate(self.legs):
			leg.leg_enabled=RSTATIC.legs_enabled[leg_nr]
		
	##	Before the control step
	#	\param timeStamp	current simulator time
	# 	Right now we have to pull already here once the data we want to use
	# 	(there is some timing problem with the current communication)
	# 	In this case the sensor update is called (non-blocking) and
	# 	the server is asked to provide new sensor data.
	def pre_processing_step(self, timeStamp):
		self.driveSafetyCheck.safetyCheck(timeStamp)
		for leg in self.legs:
			if leg.leg_enabled:
				leg.updateJointSensorInformation()
				
	def restartClients(self):
		clients=[]
		for leg_num, leg in enumerate(self.legs):
			if RSTATIC.legs_enabled[leg_num]:
				for joint in leg.joints:
					clients.append(joint)
						
		for client in clients:
			client.SetValue('resetState',0)
			client.UpdateValue('resetState')
				
		for client in clients:
			for _ in range(10):
				client.SetValue('resetState',0)
				client.UpdateValue('resetState')
				if client.resetState==0:
					break
			else:
				raise Exception('The client with ID ', client.GetBioFlexBusId, ' could not be restartet since the resetState could not be set to 0.')
				
		for client in clients:
			client.SetValue('resetState',3)
			
		if clients:
			time.sleep(3)
		
		for client in clients:
			client.UpdateValue('resetState')		
				
		for client in clients:
			for _ in range(10):
				if client.resetState==1:
					break
				client.SetValue('resetState',3)
				time.sleep(3)
				client.UpdateValue('resetState')
			else:
				raise Exception('The client with ID ', client.GetBioFlexBusId, ' could not be restartet since the resetState could not be set to 1.')
			client.ClearErrorState()
			client.errorState=0
	
	def transformFromLegCoordinatesToRobotCoordinates(self, leg_num, point):
		return point+self.relative_leg_onsets[leg_num]

	def transformFromRobotCoordinatesToLegCoordinates(self, leg_num, point):
		return point-self.relative_leg_onsets[leg_num]
	
	def getInputFootPositions(self):
		return [self.getInputFootPositionOfLeg(leg_num) for leg_num in range(6)]

	def getInputFootPositionOfLeg(self, leg_num):
		return self.transformFromLegCoordinatesToRobotCoordinates(leg_num, self.legs[leg_num].input_foot_position)
	
	def getOutputFootPositions(self):
		return [self.getOutputFootPositionOfLeg(leg_num) for leg_num in range(6)]
	
	def getOutputFootPositionOfLeg(self, leg_num):
		return self.transformFromLegCoordinatesToRobotCoordinates(leg_num, self.legs[leg_num].output_foot_position)
	
	def getCenterOfMassOfLeg(self,leg_num):
		return self.transformFromLegCoordinatesToRobotCoordinates(leg_num, self.legs[leg_num].center_of_mass)
	
	@property
	def center_of_mass(self):
		return self.getCenterOfMass()
	
	def getCenterOfMass(self):
		sum_of_centers_of_mass=numpy.zeros(3)
		for leg_num in range(6):
			sum_of_centers_of_mass+=self.getCenterOfMassOfLeg(leg_num)*self.legs[leg_num].mass
		sum_of_centers_of_mass+=self.center_of_mass_of_body_segments*self.mass_of_body_segments
		center_of_mass=sum_of_centers_of_mass/(numpy.sum([leg.mass for leg in self.legs])+self.mass_of_body_segments)
		return center_of_mass