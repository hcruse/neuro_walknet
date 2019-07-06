import numpy
import copy
from math import sin, cos, atan2, pow, pi, acos
from . import RobotSettings as RSTATIC
from tools.FreezableF import Freezable as Freezable

##
#	Leg object - encapsulating access to variables of a single leg
#	(simulator or real). 
#	The leg consists in particular out of the three joints, but the class also provides
#	methods for kinematics.
##
class Leg(Freezable):

	##	Initialisation of the leg: joint drives, read out length from xml, 
	#	setting up the phiPsi transformation ...
	#
	#	@param name name of the Leg
	#	@param bfb_client_ids communication IDs of the joints
	#	@param segment_lengths lengths of the leg segments
	def __init__(self, name, bfb_client_ids, segment_lengths, segment_masses, segment_centers_of_mass, phi, psi, chi, beta_direction, communication_interface):
		self.communication_interface = communication_interface
		self.name = name
		# Defines if the leg is switched on
		self._leg_enabled = False
		
		self.alpha = communication_interface.CreateBfbClient(bfb_client_ids[0], ["BIOFLEX_1_PROT", "BIOFLEX_ROTATORY_1_PROT", "BIOFLEX_ROTATORY_CONTROL_1_PROT", "BIOFLEX_ROTATORY_ERROR_PROT"])
		self.beta = communication_interface.CreateBfbClient(bfb_client_ids[1], ["BIOFLEX_1_PROT", "BIOFLEX_ROTATORY_1_PROT", "BIOFLEX_ROTATORY_CONTROL_1_PROT", "BIOFLEX_ROTATORY_ERROR_PROT"])
		self.gamma = communication_interface.CreateBfbClient(bfb_client_ids[2], ["BIOFLEX_1_PROT", "BIOFLEX_ROTATORY_1_PROT", "BIOFLEX_ROTATORY_CONTROL_1_PROT", "BIOFLEX_ROTATORY_ERROR_PROT"])

		self.joints=(self.alpha, self.beta, self.gamma)

			

		# The sensor measuring the pressure at the tarsus
#		self.tarsus_pressure_sensor = communication_interface.CreateBfbClient(bfb_client_ids[3], ["PRESSURE_SENSOR_PROT"])
		
		self.segment_lengths = segment_lengths
		self.segment_masses= segment_masses
		self.mass=sum(self.segment_masses)
		self.segment_centers_of_mass=segment_centers_of_mass
		self.phi=phi
		self.psi=psi
		self.chi=chi

		if 'right' in name:
			self.left_leg=False
		else: 
			self.left_leg=True

		self.beta_direction = beta_direction
		self._phi_psi_trans=self.__phi_psi_transform()
		self._phi_psi_trans_inv = numpy.linalg.inv(self._phi_psi_trans)
		self._input_foot_position = None
		self._output_foot_position = None
		
		self._center_of_mass=None
		
		# Variables for detection of ground contact
		self.contact = 0
		self.contTime = 0
		self.contactOld = 0
		self.lift_counter = 0

		self.input_velocities=[0,0,0]
		
		# This is the end of the working range
		self.min_x=-0.3
		
		self.frozen=True

	def __del__(self):
		pass
	
	@property
	def leg_enabled(self):
		return self._leg_enabled
	
	@leg_enabled.setter
	def leg_enabled(self, value):
		self._leg_enabled=(True if value else False)
		for joint in self.joints:
			joint.driveActivation=(1 if self._leg_enabled else 0)

#=========== Communication Commands ====================
	##	Updating sensor values for the joint positions and
	#	from this calculate current foot position (fw kinematics) once before 
	#	the control step (pre).
	def updateJointSensorInformation(self):
		for joint in self.joints:
			joint.UpdateValueIfTooOld('inputPosition')
			joint.UpdateValueIfTooOld('outputPosition')
		self._input_foot_position = None
		self._output_foot_position=None
		self._center_of_mass=None
#		self.tarsus_pressure_sensor.UpdateValue('highestPressure')

	##	Computes the tarsus velocity based on the angle of the joints and their velocities
	# 	As input parameters, the method expects numpy arrays.
	def computeTarsusVelocity(self, joint_angles=None, joint_velocities=None):
		if joint_angles is None:
			joint_angles=numpy.array([self.alpha.inputPosition,self.beta.inputPosition,self.gamma.inputPosition])
		if joint_velocities is None:
			joint_velocities=self.input_velocities
			
		old_position=self.computeForwardKinematics(joint_angles)
		new_position=self.computeForwardKinematics(joint_angles+joint_velocities/RSTATIC.controller_frequency)
		return (new_position-old_position)*RSTATIC.controller_frequency
		
		
	##	Sending new motor commands for all three leg joints.
	def setAngleVelocity(self, velocities):
		self.input_velocities=velocities
		for joint, velocity in zip(self.joints, velocities):
			joint.desiredValue_ISC=float(velocity)

	@property
	def input_foot_position(self):
		if self._input_foot_position==None:
			self.updateInputFootPosition()
		return self._input_foot_position
		
	@input_foot_position.setter
	def input_foot_position(self, value):
		if value==None:
			self._input_foot_position=None

	@property
	def output_foot_position(self):
		if self._output_foot_position==None:
			self.updateOutputFootPosition()
		return self._output_foot_position
		
	@output_foot_position.setter
	def output_foot_position(self, value):
		if value==None:
			self._output_foot_position=None

	@property
	def center_of_mass(self):
		if self._center_of_mass==None:
			self.updateCenterOfMass()
		return self._center_of_mass
	
	@center_of_mass.setter
	def center_of_mass(self, value):
		if value==None:
			self._center_of_mass=None

	##	Calculate current position of foot/tarsus.
	#	Applies forward kinematics (this is done initially one time during the
	#	pre control step after new sensory data is pulled)
	def updateInputFootPosition(self):
		self._input_foot_position = self.computeForwardKinematics()

	def updateOutputFootPosition(self):
		self._output_foot_position = self.computeForwardKinematics(angles=self.getOutputPosition())

	def updateCenterOfMass(self):
		self._center_of_mass = self.computeCenterOfMass(angles=self.getOutputPosition())


	##	Calculation of forward kinematics for a leg:
	#	Given a set of angles a position in 3D space is calculated.
	#	When no angles are provided the joints of the current leg provide
	#	the angles directly. 
	#	PhiPsi Transformation is applied (meaning the position is with respect
	#	to the leg coordinate system)
	#	@param angles input angles from which the current position is calculated
	#		If none are provided the current joint angles of this leg are used.
	def computeForwardKinematics(self, angles=None):
		if angles is None:
			alpha 	= self.alpha.inputPosition
			beta 	= self.beta.inputPosition
			gamma 	= self.gamma.inputPosition
		else:
			alpha 	= angles[0]
			beta 	= angles[1]
			gamma 	= angles[2]
			
		temp_tarsus_position=self._alphaForwardKinematics(alpha, self._betaForwardKinematics(beta, self._gammaForwardKinematics(gamma, numpy.array([0,0,0,1]))))
		# inverse kinematics alpha angle check
		alpha_check=-atan2(temp_tarsus_position[1], (temp_tarsus_position[0]))
		if abs(alpha_check-alpha)>=0.01:
			raise Exception('The provided angles for '+self.name+'('+str(alpha)+', '+str(beta)+', '+str(gamma)+') are not valid for the forward/inverse kinematics.')
		return self._phi_psi_trans.dot(temp_tarsus_position)[0:3]

	def computeCenterOfMass(self, angles=None):
		if angles is None:
			alpha 	= self.alpha.inputPosition
			beta 	= self.beta.inputPosition
			gamma 	= self.gamma.inputPosition
		else:
			alpha 	= angles[0]
			beta 	= angles[1]
			gamma 	= angles[2]
		#angles=[alpha, beta, gamma]
		#overall_weight=0
		#for segment_num, for_kin in zip(angles, [self._alphaForwardKinematics, self._betaForwardKinematics, self._gammaForwardKinematics]):
		#	pass

		center_of_mass_of_coxa=self._alphaForwardKinematics(alpha, numpy.append(self.segment_centers_of_mass[0],1))
		center_of_mass_of_femur=self._alphaForwardKinematics(alpha, self._betaForwardKinematics(beta, numpy.append(self.segment_centers_of_mass[1],1)))
		center_of_mass_of_tibia=self._alphaForwardKinematics(alpha, self._betaForwardKinematics(beta, self._gammaForwardKinematics(gamma, numpy.append(self.segment_centers_of_mass[2],1))))
		center_of_mass=(center_of_mass_of_coxa*self.segment_masses[0] + center_of_mass_of_femur*self.segment_masses[1] + center_of_mass_of_tibia*self.segment_masses[2])/sum(self.segment_masses)
		return (self._phi_psi_trans.dot(center_of_mass))[0:3]

	def _alphaForwardKinematics(self, alpha, point=numpy.array([0, 0, 0, 1])):
		#point=numpy.append(point,1)
		alpha*=-1 # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
		cos_alpha=cos(alpha)
		sin_alpha=sin(alpha)
		alpha_trans = numpy.array([(cos_alpha, 0, sin_alpha, (self.segment_lengths[0] * cos_alpha)), 
							(sin_alpha, 0, -cos_alpha, self.segment_lengths[0] * sin_alpha), 
							(0, 1, 0, 0,), 
							(0, 0, 0, 1)])
		#print('alpha_trans: ', alpha_trans)
		return alpha_trans.dot(point)#[0:3]
	
	def _betaForwardKinematics(self, beta, point=numpy.array([0, 0, 0, 1])):
		#point=numpy.append(point,1)
		if self.beta_direction is False:
			beta *= -1 # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
		cos_beta=cos(beta)
		sin_beta=sin(beta)
		beta_trans = numpy.array([(cos_beta, -sin_beta, 0, self.segment_lengths[1] * cos_beta), 
								(sin_beta, cos_beta, 0,self.segment_lengths[1] * sin_beta), 
								(0, 0, 1, 0,), 
								(0, 0, 0, 1)])
		return beta_trans.dot(point)#[0:3]
	
	def _gammaForwardKinematics(self, gamma, point=numpy.array([0, 0, 0, 1])):
		#point=numpy.append(point,1)
		if self.beta_direction is True:
			gamma *= -1 # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
		gamma=gamma-pi/2
		cos_gamma=cos(gamma)
		sin_gamma=sin(gamma)
		gamma_trans = numpy.array([(cos_gamma, -sin_gamma, 0, self.segment_lengths[2] * cos_gamma),
							(sin_gamma, cos_gamma, 0, self.segment_lengths[2] * sin_gamma),
							(0, 0, 1, 0), 
							(0, 0, 0, 1)])
		return gamma_trans.dot(point)#[0:3]

	##	Calculation of inverse kinematics for a leg:
	#	Given a position in 3D space a joint configuration is calculated.
	#	When no position is provided the current position of the current leg is used
	#	to calculate the complementing angles.
	#	PhiPsi Inverse Transformation is applied (meaning the position is with respect
	#	to the leg coordinate system aligned with the body coordinate system)
	#	@param p point in leg coordinate system 
	def computeInverseKinematics(self, p=None):
		if p==None:
			p=self.input_foot_position
		if len(p)==3:
			p=numpy.append(p,[1])
		p_temp=copy.copy(p)
		p = self._phi_psi_trans_inv.dot(p)
		alpha_angle = -atan2(p[1],(p[0]))
		beta_pos=self._alphaForwardKinematics(alpha_angle)
		lct = numpy.linalg.norm(p[0:3] - beta_pos[0:3])
		try:
			gamma_angle = (acos((pow(self.segment_lengths[2], 2) + pow(self.segment_lengths[1], 2) - pow(lct, 2)) / (2 * self.segment_lengths[1] * self.segment_lengths[2]))) - pi / 2
			h1 = (acos((pow(self.segment_lengths[1], 2) + pow(lct, 2) - pow(self.segment_lengths[2], 2)) / (2 * self.segment_lengths[1] * lct)))
			h2 = (acos((pow(lct, 2) + pow(self.segment_lengths[0], 2) - pow(numpy.linalg.norm(p[0:3]), 2)) /
					(2 * self.segment_lengths[0] * lct)))
		except ValueError:
			raise ValueError('The provided position ('+str(p_temp[0])+', '+str(p_temp[1])+', '+str(p_temp[2])+') is not valid for the given geometry for leg '+ self.name+'.')
		if p[2] < 0:
			beta_angle = (h1 + h2) - pi
		else:
			beta_angle = (pi - h2) + h1
			
		if self.beta_direction is True:
			gamma_angle *= -1
		else:
			beta_angle *= -1
		return numpy.array([alpha_angle, beta_angle, gamma_angle])
	
	def computeJacobian(self, angles=None):
		if angles is None:
			alpha 	= self.alpha.inputPosition
			beta 	= self.beta.inputPosition
			gamma 	= self.gamma.inputPosition
		else:
			alpha 	= angles[0]
			beta 	= angles[1]
			gamma 	= angles[2]
		
		if self.beta_direction is True:
			gF=-1 # The direction of alpha is reversed as compared to the denavit-hartenberg notation.
			bF=1
		else:
			gF=1
			bF=-1 # The direction of alpha is reversed as compared to the denavit-hartenberg notation.

		cac  = cos(alpha-self.chi);
		sac  = sin(alpha-self.chi);
		cb   = cos(bF*beta);
		sb   = sin(bF*beta);
		sphi = sin(self.phi);
		cphi = cos(self.phi);
		spsi = sin(self.psi);
		cpsi = cos(self.psi);
		cbg  = cos(bF*beta + gF*gamma);
		sbg  = sin(bF*beta + gF*gamma);
		
		lc=self.segment_lengths[0]
		lf=self.segment_lengths[1]
		lt=self.segment_lengths[2]
		
		J=numpy.array( [(-(lc+lf*cb+lt*sbg)*(cphi*cpsi*sac-cac*sphi),		bF*(lt*cbg-lf*sb)*(cac*cphi*cpsi+sac*sphi)-bF*(lf*cb+lt*sbg)*cphi*spsi,  gF*lt*(cbg*(cac*cphi*cpsi+sac*sphi)-sbg*cphi*spsi)),
						(-(lc+lf*cb+lt*sbg)*(cac*cphi+cpsi*sac*sphi), 		bF*(lf*sb-lt*cbg)*(cphi*sac-cac*cpsi*sphi)-bF*(lf*cb+lt*sbg)*sphi*spsi, -gF*lt*(cbg*(cphi*sac-cac*cpsi*sphi)+sbg*sphi*spsi)),
						(				 -(lc+lf*cb+lt*sbg)*sac*spsi, 						  bF*(lf*cb*cpsi+lt*sbg*cpsi+cac*(-lf*sb+lt*cbg)*spsi), 				      gF*lt*(sbg*cpsi+cac*cbg*spsi))]);
		return J
	
	def computeTorquesFromForce(self, force, angles=None):
		return (self.computeJacobian(angles).transpose()).dot(force)

	def computeForceFromTorques(self, torques, angles=None):
		return numpy.linalg.inv(self.computeJacobian(angles).transpose()).dot(torques)		
	
	def estimateForceFromTorsion(self, torsions=None, angles=None):
		if torsions==None:
			torsions=[joint.torsion for joint in self.joints]
		torques=[855*(torsion**3)/abs(torsion) for torsion in torsions]
		return numpy.linalg.inv(numpy.transpose(self.computeJacobian(angles))).dot(torques)		
	
	##	Pre calculate phiPsi transformation once.
	def __phi_psi_transform(self):
		phi_trans = numpy.array([	(cos(self.phi), 			0, 	sin(self.phi), 0), 
									(sin(self.phi), 			0, -cos(self.phi), 0), 
									(			 0,				1, 				0, 0), 
									(			 0,				0, 				0, 1)])
		psi_trans = numpy.array([	(cos(self.psi), 			0, -sin(self.psi), 0), 
									(sin(self.psi), 			0,  cos(self.psi), 0), 
									(			 0,			   -1, 				0, 0), 
									(			 0, 			0, 				0, 1)])
		chi_trans = numpy.array([	(cos(self.chi),-sin(self.chi), 				0, 0), 
									(sin(self.chi), cos(self.chi), 				0, 0), 
									(			 0, 			0, 				1, 0), 
									(			 0, 			0, 				0, 1)])
		return phi_trans.dot(psi_trans).dot(chi_trans)
	
	def getBetaTorsion(self):
		print("Torsion GC method not implemented")
		return 1

	##	Get current angles of the drives (target angle) for the leg.
	def getInputAngles(self):
		return [joint.inputPosition for joint in self.joints]
	
	def getInputPosition(self):
		return self.getInputAngles()
	
	def getOutputAngles(self):
		return [joint.outputPosition for joint in self.joints]

	def getOutputPosition(self):
		return self.getOutputAngles()
	
	def getDistanceToWorkspaceBorder(self, direction):
		return self.input_foot_position[0]-self.min_x
