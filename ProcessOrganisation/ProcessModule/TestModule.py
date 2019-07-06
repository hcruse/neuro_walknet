'''
Created on 3.11.2012

@author: mschilling
'''
from .ProcessingModule import ProcessingModule

##
#	A very simple example module which only prints out whatever is
#	passed during the pre_processing_step.
##
class TestModule(ProcessingModule):
	##
	#	Just prints out passed arguments.
	def pre_processing_step(self, args=None):
		print(args)