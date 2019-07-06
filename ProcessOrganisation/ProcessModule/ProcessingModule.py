'''
Created on 3.11.2012

@author: mschilling
'''

##
#	A ProcessingModule is defining the interface for modules.
#
#	It basically provides a temporal skeleton for execution order of registered modules.
#	The main process is calling all modules - first the pre, then the main processing and
#	at last the post processing step. The order of the calls in these steps is not 
#	defined in the ProcessingModuleExecution.
##
class ProcessingModule:
	def __init__(self, name):
		self.name = name

	##	Initialisation of the modules.
	#	Before the modules are called (but after they are created)
	#	the initialisation module function is called for each module.
	#	This is useful when there are circular dependencies between modules and
	#	e.g. a module depends on some init values that are created in the
	#	object __init__ call of another mdule.
	def init_module(self):
		pass
	
	##	The pre-processing time step of the module.
	#	Has to be overwritten when a module shall execute certain function before
	#	the main processing, e.g. update of sensor values.
	#	@param time current simulation time
	def pre_processing_step(self, time, args=None):
		pass
		
	##	The processing time step of the module.
	#	Has to be overwritten when a module shall do some main processing,
	#	e.g. the controller should act during this time window.
	#	@param time current simulation time
	def processing_step(self, time, args=None):
		pass
	
	##	The post-processing time step of the module.
	#	Has to be overwritten when a module shall execute certain function after
	#	the main processing, e.g. visualisation of data, initiate simulation to advance.
	#	@param time current simulation time
	def post_processing_step(self, time, args=None):
		pass