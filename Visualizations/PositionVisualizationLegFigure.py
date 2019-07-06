import matplotlib.pylab as py
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
##
#   Figure showing current leg position over time (front to back movement)
#   and AEP and PEP.
##
class PositionVisualizationLegFigure:
    
    ##
    #   Init figure as a subplot.
    def __init__ (self, leg, ax_fig, time_window):
        self.it = 1
        self.leg = leg
        self.ax_fig = ax_fig
        
        self.time_window = 2500#time_window
        
        # Variable remembering last state swing
        self.swing_state = False
        
#       self.ax_fig.plot([0,self.time_window],[self.motivationNetLeg.pep[0],self.motivationNetLeg.pep[0]], linestyle=':', linewidth=1.0, color='blue')
#       self.ax_fig.plot([0,self.time_window],[self.motivationNetLeg.aep[0],self.motivationNetLeg.aep[0]], linestyle=':', linewidth=1.0, color='blue')

# AEP and PEP turned off currently
#       self.pep_shift_line, = self.ax_fig.plot([],[], color='blue')
#       self.aep_shift_line, = self.ax_fig.plot([],[], color='blue')
        self.position_line, = self.ax_fig.plot([],[], color='green')
        self.ax_fig.set_xlim(0,self.time_window) 
#       self.ax_fig.set_ylim(self.motivationNetLeg.pep[0]-0.1,self.motivationNetLeg.aep[0]+0.05)
        self.ax_fig.set_ylim(-0.25, 0.2)
#       self.ax_fig.xticks(np.arange(min(x), max(x)+1, 1.0))
        plt.locator_params(nbins=8)
        self.phase_box = None
        self.old_phase = 0
        self.draw_phase_box = False
    
    ##
    #   Update the subplot pulling data from the wrobot structure.
    def update_visualization(self):
    
#       if (self.it == 600):
#           self.ax_fig.plot((600, 600), (self.motivationNetLeg.pep[0]-0.2, self.motivationNetLeg.aep[0]+0.1), linewidth=1.0, color='red')
                
#       if (self.it % self.time_window == 0):
#           self.aep_shift_line.set_xdata([])
#           self.aep_shift_line.set_ydata([])
#           self.pep_shift_line.set_xdata([])
#           self.pep_shift_line.set_ydata([])
#           self.it = 0
#           self.position_line.set_xdata([])
#           self.position_line.set_ydata([])
#       self.aep_shift_line.set_xdata(np.append(self.aep_shift_line.get_xdata(), [self.it]))
#       self.aep_shift_line.set_ydata(np.append(self.aep_shift_line.get_ydata(), [self.motivationNetLeg.aep_shifted[0]]))
#       
#       self.pep_shift_line.set_xdata(np.append(self.pep_shift_line.get_xdata(), [self.it]))
#       self.pep_shift_line.set_ydata(np.append(self.pep_shift_line.get_ydata(), [self.motivationNetLeg.pep_shifted[0]]))

        self.it += 1
        position = self.leg.input_foot_position
        
        self.position_line.set_xdata(np.append(self.position_line.get_xdata(), [self.it]))
        self.position_line.set_ydata(np.append(self.position_line.get_ydata(), [position[0]]))
        

