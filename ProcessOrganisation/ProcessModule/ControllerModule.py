from .ProcessingModule import ProcessingModule as ProcessingModule
class ControllerModule(ProcessingModule):
	def __init__(self, name, robot):
		ProcessingModule.__init__(self, name)
		self.__completed=False
		self.robot = robot
	
	def handleMessage(self, message):
		pass
		
	def _notifyOfCompletion(self):
			self.__completed=True
	
	def hasCompleted(self):
		return self.__completed
