import matplotlib.pylab as py
import matplotlib.pyplot as plt
import numpy as np

import Hector.RobotSettings as RSTATIC

from ProcessOrganisation.ProcessModule.ProcessingModule import ProcessingModule
from .PositionVisualizationLegFigure import PositionVisualizationLegFigure

##
#	Sub-Plots of leg position (front to back) over time.
#	Shown is for all active legs the current position (along the body axis)
#	in six individual figures.
#	The PEP and AEP are also shown.
#
#	The visualization is a ProcessingModule which pulls in the post processing step
#	updated values from the wrobot structure.
##
class PositionVisualizationModule(ProcessingModule):
	
	##
	#	Init figure and subplots.
	def __init__ (self, name, robot):
		ProcessingModule.__init__(self, name)
		#self.wRobot = wRobot
		self.robot = robot
		plt.ion()
		self.fig_position = plt.figure(figsize=(12, 12))
		self.position_visualization = [0,0,0,0,0,0]
		self.time_window = 2500
		
		plot_nr = 321
		for leg in self.robot.legs:
#			if motiv_leg.wleg.leg.leg_enabled:
			self.position_visualization[plot_nr-321] = PositionVisualizationLegFigure(leg, self.fig_position.add_subplot(plot_nr), self.time_window)
			plot_nr += 1
		plt.tight_layout(h_pad=6, w_pad=4)
		plt.ioff()
	
	##
	#	Update figure.
	def post_processing_step(self, args=None):
		#plt.savefig("/Users/mschilling/Desktop/Pos/pos_during_sim_" + str(self.position_visualization[0].it) + ".pdf")
		#if (self.position_visualization[0].it==self.time_window):
		#	plt.savefig("/Users/mschilling/Desktop/pos_during_sim.pdf")
		#	print("SAVED")
		#	input()
		for i in range(0,6):
			if self.position_visualization[i] != 0 :
				self.position_visualization[i].update_visualization()
		self.fig_position.canvas.draw()
		
