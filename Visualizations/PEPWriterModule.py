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
class PEPWriterModule(ProcessingModule):
    
    def __init__ (self, name, neuro_wn):
        ProcessingModule.__init__(self, name)
        self.controller_objs = neuro_wn.controller_objs
        
        self.draw_counter = 0 
        
        self.data_points =  10000
        self.data_array = np.empty([self.data_points,23])#[None]*self.data_points
        
        self.leg_order = [0,3,1,4,2,5]

        
    ##
    #   Init figure and subplots.
    def init_module(self):
        self.file_name = "neuro_wn_swingstarts"
        self.draw_counter = 0
        self.lastswing = [-1,-1,-1,-1,-1,-1]
        with open( self.file_name,'a') as f_handle:
            f_handle.write('# Leg (0 - FL, 1 - FR, 2 - ML, 3 - MR, 4 - HL, 5 - HR; starts swing at iteration \n') 
        self.post_processing_step()

    
    ##
    #   Update visualization using data from the body model.    
    def post_processing_step(self, args=None):
        self.draw_counter+=1  
        
        if (self.draw_counter > 0):
            for j in range(0,6) :
                if (self.controller_objs[j]):
                        if (self.controller_objs[j].v[122] > self.controller_objs[j].v[123]) and self.lastswing[j]<0 :
                            self.lastswing[j] = self.draw_counter
                            with open( self.file_name,'ba') as f_handle:
                                np.savetxt( f_handle, [[j, self.draw_counter]], fmt='%i %i')# %f %f %d %d %d %d %d %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f')
                        elif (self.lastswing[j]>=0 and (self.controller_objs[j].v[122] < self.controller_objs[j].v[123]) ):      
                            #self.ax_footfall.plot([self.lastswing[j],self.draw_counter], [6-self.leg_order[j],6-self.leg_order[j]], linestyle='-', linewidth=20.0, color='black', marker='', solid_capstyle="butt")
                            self.lastswing[j]=-1
