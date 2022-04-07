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
class StimulusVisualizationModule(ProcessingModule):
    
    def __init__ (self, name, neuro_wn, leg_nr=1):
        ProcessingModule.__init__(self, name)
        self.counter = 0
        self.start_count = 0. #550. #700. #700. # v30_FR 900. #v30_HR 400. #vel30 MR_1350 # 1100 M # 1350 F,H   # 100 = 1 s
        self.end_count = 3600. #1250. #1220. #1800. # v30_FR 2000. #v30_HR 1500. #vel30 MR_2150. #1900 M # 2150 F,H, vel = 40, loadbeta * 0.5
        self.time_window = self.end_count - self.start_count
        
        self.neuro_walknet = neuro_wn
        plt.ion()
        
        self.fig_stimulus = plt.figure(figsize=(6, 6))
        self.fig_stimulus.canvas.set_window_title('Stimulus activation - ' + str(leg_nr))
        self.sub_a = self.fig_stimulus.add_subplot(211)
        self.sub_b = self.fig_stimulus.add_subplot(212)
        
        self.neuro_leg = self.neuro_walknet.controller_objs[leg_nr]

        self.plot_a = self.sub_a.plot([],[], color='green')[0]
        self.plot_b = self.sub_b.plot([],[], color='red')[0]
        self.sub_a.set_xlim(0,0.01*self.time_window)
        self.sub_b.set_xlim(0,0.01*self.time_window)

        self.sub_a.set_ylim(-5, 55.)
        self.sub_b.set_ylim(-5, 55.)
        
        plt.tight_layout(h_pad=6, w_pad=4) # 4
        plt.ioff()
        
    ##
    #   Init figure and subplots.
    def init_module(self):
        self.post_processing_step()
        self.fig_stimulus.canvas.draw()
    
    ##
    #   Update visualization using data from the body model.    
    def post_processing_step(self, args=None):
        if (self.counter > self.start_count):
            if (self.neuro_leg):
                self.new_value = [self.neuro_leg.v[170]] #  ChOvel     
                self.plot_a.set_xdata(np.append(self.plot_a.get_xdata(), [0.01*(self.counter-self.start_count)]))
                self.plot_a.set_ydata(np.append(self.plot_a.get_ydata(), [self.new_value]))

                self.new_value = [self.neuro_leg.ChOpos] # v[46]]
                self.plot_b.set_xdata(np.append(self.plot_b.get_xdata(), [0.01*(self.counter-self.start_count)]))
                self.plot_b.set_ydata(np.append(self.plot_b.get_ydata(), [self.new_value]))

        self.counter+=1
                
        self.fig_stimulus.canvas.draw()
        if (self.counter == self.end_count):
            import time
            timestr = time.strftime("%Y%m%d-%H%M%S")
            self.fig_stimulus.savefig("exp_intraleg/stimulus_output_" + timestr + ".eps")
            print("Wrote file")
