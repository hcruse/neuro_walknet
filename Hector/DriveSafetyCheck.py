##
#	Check if all drives are ready and if not
#	shut the drive down and disable the corresponding leg.
##
class DriveSafetyCheck:

	##
	# Init check drive.
	def __init__(self, robot):
		self.robot = robot
		self.fatalErrorDetected=False
		self.error_names=(	'communicationTimeoutError',
							'rotorPositionMeasurementError',
							'torsionMeasurementError',
							'outputPositionMeasurementError',
							'torsionError',
							'outputPositionError')
		self.erroneous_clients={}
	##
	#	Check for a single leg if for one drive a fatal error has been detected.	
	def fatalErrorCheck(self, leg):
		numOfErrors=0
		for joint, joint_nr in zip(leg.joints, range(len(leg.joints))):
			if joint.errorState:
				numOfErrors+=1
				bus_id=joint.GetBioFlexBusId()
				try:
					error_list=self.erroneous_clients[bus_id]
				except Exception:
					error_list=['fatalError']
					print('A fatal error was detected in joint "' + ('alpha', 'beta', 'gamma')[joint_nr] + '" in leg "'+ leg.name+ '".')
					
				for error_name in self.error_names:
					if error_name not in error_list:
						if joint.GetValue(error_name)[0]:
							print('Detected error "'+error_name+' in joint "' + ('alpha', 'beta', 'gamma')[joint_nr] + '" in leg "'+ leg.name+ '".')
							error_list.append(error_name)
				self.erroneous_clients[bus_id]=error_list
		return numOfErrors
					
		
	##	Check all drives.
	#	Is called during the precontrol step from the Robot object.
	#	\param timeStamp	current simulator time
	#	Check all the single drives and if an error has been found disable the leg.
	def safetyCheck(self, timeStamp):
		numOfErrors=0
		for leg in self.robot.legs:
			if leg.leg_enabled:
				if self.fatalErrorCheck(leg)>0:
					numOfErrors+=1
		if numOfErrors>0:
			for leg in self.robot.legs:
				leg.leg_enabled = False
				print('deactivating leg: ', leg.name)
			if self.fatalErrorDetected==False:
				self.fatalErrorDetected=True
				print('Fatal Error detected! The robot will now be deactivated!')
