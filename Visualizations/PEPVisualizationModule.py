import matplotlib.pylab as py
import matplotlib.pyplot as plt
import numpy as np

from ProcessOrganisation.ProcessModule.ProcessingModule import ProcessingModule

##
#   Sub-Plot showing current leg position and current PEP position 
#   for every leg over time.
#
#   The visualization is a ProcessingModule which pulls in the post processing step
#   updated values from the bodyModel structure.
##
class PEPVisualizationModule(ProcessingModule):
    
    def __init__ (self, name, neuro_wn):
        ProcessingModule.__init__(self, name)
        self.controller_objs = neuro_wn.controller_objs
        plt.ion()
        """ Initialising the drawing window.
            Must be called before the visualisation can be updated
            by calling draw_manipulator"""
        self.leg_order = [0,3,1,4,2,5]
        self.footfall = plt.figure(figsize=(24, 3))
        self.footfall.canvas.set_window_title('Footfall Pattern')
        # Number of shown iteration steps
        self.max_time = 8000
        plt.ioff()
        
    ##
    #   Init figure and subplots.
    def init_module(self):
        self.draw_counter = 0
        self.lastswing = [-1,-1,-1,-1,-1,-1]
        self.ax_footfall = self.footfall.add_subplot(111)
        self.ax_footfall.set_yticks([6,5,4,3,2,1])
        self.ax_footfall.set_yticklabels(['FL', 'ML','HL','FR', 'MR','HR'], size=18)
        self.ax_footfall.set_xticklabels(['0','10', '20','30','40', '50','60','70','80'],size=18)
        #py.rcParams['figure.figsize'] = 4, 4
        self.ax_footfall.plot([0,self.max_time], [3.5,3.5], linestyle=':', linewidth=1.0, color='gray', alpha=0.7, marker='')
        self.ax_footfall.set_xlim(0,self.max_time) 
        self.ax_footfall.set_ylim(0.5,6.5)
        #self.update_footfall_pattern(self.bodyModel.gc[-1], 0)
        self.post_processing_step()
        #plt.axes().get_yaxis().set_visible(False)
        #plt.axes().get_xaxis().set_visible(False)
        self.footfall.canvas.draw()
        #plt.savefig("/Users/mschilling/Desktop/podogram.pdf")
    
    ##
    #   Update visualization using data from the body model.    
    def post_processing_step(self, args=None):
        self.draw_counter+=1
        #if self.draw_counter%100 == 0:
        #   print(self.draw_counter)
        if (self.draw_counter > 0):
            for j in range(0,6) :
                if (self.controller_objs[j]):
                        if (self.controller_objs[j].v[122] > self.controller_objs[j].v[123]) and self.lastswing[j]<0 :
                            self.lastswing[j] = self.draw_counter
                        elif (self.lastswing[j]>=0 and (self.controller_objs[j].v[122] < self.controller_objs[j].v[123]) ):      
                            self.ax_footfall.plot([self.lastswing[j],self.draw_counter], [6-self.leg_order[j],6-self.leg_order[j]], linestyle='-', linewidth=20.0, color='black', marker='', solid_capstyle="butt")
                            self.lastswing[j]=-1
            self.footfall.canvas.draw()
        if (self.draw_counter == self.max_time):
            import time
            timestr = time.strftime("%Y%m%d-%H%M%S")
            self.footfall.savefig("footfall/footfall_post1_" + timestr + ".eps")
            print("Wrote file")
            self.ax_footfall.clear()
            self.init_module()
