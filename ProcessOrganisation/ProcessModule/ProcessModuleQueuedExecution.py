'''
Created on 10.3.2014

@author: mschilling
'''
from .ProcessModuleExecution import ProcessModuleExecution
from .ControllerModule import ControllerModule as ControllerModule
from decimal import Decimal as Decimal
from ..SimulatorModule.SimulatorTimerModule import SimulatorTimerModule

##
#	The execution program for the Process Module execution.
#	In this particular case different controllers can be queued and 
#	executed subsequentially.
#
#	A ProcessingModule is defining the interface for modules.
#
#	It basically provides a temporal skeleton for execution order of registered modules.
#	The main process is calling all modules - first the pre, then the main processing and
#	at last the post processing step. The order of the calls in these steps is not 
#	defined in the ProcessingModuleExecution.
##
class ProcessModuleQueuedExecution(ProcessModuleExecution):
	
	##	Initialisation
	# 	@param debug_time Providing information on the timing of the individual modules.
	def __init__(self, debug_time = False):
		ProcessModuleExecution.__init__(self, debug_time)
		self.control_module_queue = []
		self.current_control_module_end_time = Decimal('0')
		self.current_control_module = None
		self.time_precision=SimulatorTimerModule.time_precision

	##	Inits all registered modules before the execution is started.
	def init_all_modules(self):
		ProcessModuleExecution.init_all_modules(self)
		if len(self.control_module_queue) > 0:
			self.__switch_to_next_queued_control_module(Decimal('0'))
		

	##	Add a module to the Execution - the different time
	#	steps of this module will be called alongside the other modules.
	#	@param newMod the module which shall be added
	def add_control_module_to_queue(self, newMod, duration):
		duration_decimal=Decimal('0')
		if duration==float('inf'):
			duration_decimal=Decimal('Inf')
		else:
			duration_decimal=Decimal(duration).quantize(self.time_precision)
			
		self.control_module_queue.append( (newMod, duration_decimal) )

	def __switch_to_next_queued_control_module(self, timestamp_decimal):
		if len(self.control_module_queue) > 0:
			if self.current_control_module != None:
				self.remove_module(self.current_control_module.name)
			self.current_control_module, duration = self.control_module_queue.pop(0)
			self.add_module( self.current_control_module )
			
			print("Switched to next control module: ", self.current_control_module.name)
			self.current_control_module.init_module()
			
			self.current_control_module_end_time = timestamp_decimal + duration
			
	def forwardMessageToCurrentController(self, message):
		self.current_control_module.handleMessage(message)
			
	def execute_complete_step(self, timestamp):
		timestamp_decimal=Decimal(timestamp).quantize(self.time_precision)
		
		controller_has_completed=False
		if isinstance(self.current_control_module, ControllerModule):
			controller_has_completed=self.current_control_module.hasCompleted()
			
		if (timestamp_decimal > self.current_control_module_end_time or controller_has_completed):
			self.__switch_to_next_queued_control_module(timestamp_decimal)
		ProcessModuleExecution.execute_complete_step(self, timestamp)
