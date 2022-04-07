import matplotlib.pylab as py
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
##
##
class JointPositionLegFigure:
    
    ##
    #   Init figure as a subplot.
    def __init__ (self, neuro_leg, joint, unit_n, ax_fig, time_window):
        self.it = 1
        self.neuro_leg = neuro_leg
        self.unit_n = unit_n
        self.ax_fig = ax_fig
        self.lastswing = -1
        self.joint = joint
        
        self.time_window = time_window

        self.neuron_lines = []
        if (self.neuro_leg):
            # CREATE FOR EACH LINE IN THE PLOT ONE ENTRY HERE
            self.neuron_lines.append(self.ax_fig.plot([],[], color='green')[0])
            self.neuron_lines.append(self.ax_fig.plot([],[], color='red')[0])
        self.ax_fig.set_xlim(0,self.time_window)
        plt.xticks(np.arange(0, self.time_window+1, self.time_window/10.), 0.02*np.arange(0, self.time_window+1, self.time_window/10.))
        self.ax_fig.set_ylim(-5, 55.)
        
        self.ax_fig2 = self.ax_fig.twinx()
        self.joint_line, = self.ax_fig2.plot([],[], color='blue')
        self.ax_fig2.set_ylabel('joint angle', color='blue')
        self.ax_fig2.set_xlim(0,self.time_window)
        if (unit_n[0] < 20):
            self.ax_fig2.set_ylim(-1., .5)
        elif (unit_n[0] < 60):
            self.ax_fig2.set_ylim(0., 3.)
        else:
            self.ax_fig2.set_ylim(-0.75, .75)
        for tl in self.ax_fig2.get_yticklabels():
            tl.set_color('b')
        
        
#       self.ax_fig.xticks(np.arange(min(x), max(x)+1, 1.0))
        plt.locator_params(nbins=8)
        self.new_value = [0 for i in range(0, len(self.neuron_lines))]
    
    ##
    #   Update the subplot pulling data from the wrobot structure.
    def update_visualization(self):
        self.it += 1

        if (self.neuro_leg):
            # PROVIDE FOR EACH LINE A NEW UPDATED VALUE
            if (self.neuro_leg.v[122] > self.neuro_leg.v[123]) and self.lastswing<0 :
                self.lastswing = self.it
            elif (self.lastswing>=0 and (self.neuro_leg.v[122] < self.neuro_leg.v[123]) ):  
                    # Create a Rectangle patch
                    rect = Rectangle((self.lastswing,-5),(self.it-self.lastswing),60,linewidth=0,edgecolor='r',facecolor='gray', alpha=0.25)    
                     # Add the patch to the Axes
                    self.ax_fig.add_patch(rect)
                    #self.ax_fig.plot([self.lastswing,self.it], [0,0], linestyle='-', linewidth=20.0, color='black', alpha=0.25, marker='', solid_capstyle="butt")
                    self.lastswing=-1
                    
            for i in range(0,len(self.neuron_lines)):
                self.new_value[i] = [self.neuro_leg.v[self.unit_n[i]]]
                plot_line = self.neuron_lines[i]
                plot_line.set_xdata(np.append(plot_line.get_xdata(), [self.it]))
                plot_line.set_ydata(np.append(plot_line.get_ydata(), [self.new_value[i]]))
            
            
            # Update leg position plot
            joint_angle = -self.joint.inputPosition
            self.joint_line.set_xdata(np.append(self.joint_line.get_xdata(), [self.it]))
            self.joint_line.set_ydata(np.append(self.joint_line.get_ydata(), [joint_angle]))
