'''
Created on 14.11.2012

@author: mschilling
'''
import time

##
#	A ProcessModuleExecution 
#
#   Calls the different ProcessModules.
##
class ProcessModuleExecution:
	
	##	Initialisation
	# 	@param debug_time Providing information on the timing of the individual modules.
	def __init__(self, debug_time = False):
		self.registered_module_names = []
		self.registered_modules = []
		self.time_debug_output = debug_time


	##	Inits all registered modules before the execution is started.
	def init_all_modules(self):
		for mod in self.registered_modules:
			mod.init_module()


	##	Add a module to the Execution - the different time
	#	steps of this module will be called alongside the other modules.
	#	@param newMod the module which shall be added
	def add_module(self, newMod):
		self.registered_module_names.append(newMod.name)
		self.registered_modules.append(newMod)

	##	Remove a module from the main execution.
	#	@param name of the module which shall be removed
	def remove_module(self, name):
		index=self.registered_module_names.index(name)
		del self.registered_modules[index]
		del self.registered_module_names[index]
		
	##	Execute a complete time step.
	#	Iterate over all modules, first over all for the pre-processing step;
	#	followed by calls to all modules for the processing step and
	#	finally the post-processing step for all modules is called.
	#
	#	If timing should be output each module is timed during the execution.
	#
	#	@param timeStamp current simulation time
	def execute_complete_step(self, timeStamp):
		if __debug__ and self.time_debug_output:
			time_pre_step = [0.] * len(self.registered_modules)
			time_processing_step = [0.] * len(self.registered_modules)
			time_post_step = [0.] * len(self.registered_modules)
			for mod_nr, mod in enumerate(self.registered_modules):
				time_pre_step[mod_nr] = time.time()
				mod.pre_processing_step(timeStamp)
				time_pre_step[mod_nr] = time.time() - time_pre_step[mod_nr]
			for mod_nr, mod in enumerate(self.registered_modules):
				time_processing_step[mod_nr] = time.time()
				mod.processing_step(timeStamp)
				time_processing_step[mod_nr] = time.time() - time_processing_step[mod_nr]
			for mod_nr, mod in enumerate(self.registered_modules):
				time_post_step[mod_nr] = time.time()
				mod.post_processing_step(timeStamp)
				time_post_step[mod_nr] = time.time() - time_post_step[mod_nr]
			# Print out times of the individual modules
			for mod_nr, mod_name in enumerate(self.registered_module_names):
				print('t=%(t)4.2f - %(name)20s pre=%(pre)4.3f   process=%(process)4.3f   post=%(post)4.3f' %\
					{'t': timeStamp,'name': mod_name[:20], 'pre': time_pre_step[mod_nr], 'process': time_processing_step[mod_nr], 'post': time_post_step[mod_nr]})
		else:
			for mod in self.registered_modules:
				mod.pre_processing_step(timeStamp)
			for mod in self.registered_modules:
				mod.processing_step(timeStamp)
			for mod in self.registered_modules:
				mod.post_processing_step(timeStamp)
