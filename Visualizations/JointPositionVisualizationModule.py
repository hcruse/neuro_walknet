import matplotlib.pylab as py
import matplotlib.pyplot as plt
import numpy as np

import Hector.RobotSettings as RSTATIC

from ProcessOrganisation.ProcessModule.ProcessingModule import ProcessingModule
from .JointPositionLegFigure import JointPositionLegFigure

##
#   Sub-Plots of Activations for neurons in the six legs.
#
#   The visualization is a ProcessingModule which pulls in the post processing step
#   updated values from the wrobot structure.
##
class JointPositionVisualizationModule(ProcessingModule):
    
    ##
    #   Init figure and subplots.
    def __init__ (self, name, neuro_wn, leg_nr=0):  #
        ProcessingModule.__init__(self, name)
        
        self.counter = 0
        #            vel = 30:  # _FR 700.- 1800 # MR 400.- 1500 #_HR 900. - 2000
        self.start_count = 900. #700. # v30_FR 900. #v30_HR 400. #vel30 MR_1350 # 1100 M # 1350 F,H   # 100 = 1 s
        self.end_count = 2000. #1800. # v30_FR 2000. #v30_HR 1500. #vel30 MR_2150. #1900 M # 2150 F,H, vel = 40, loadbeta * 0.5
        self.time_window = self.end_count - self.start_count
        
        self.neuro_walknet = neuro_wn
        plt.ion()
        self.fig_activation = plt.figure(figsize=(6, 8)) 
        self.neuronAct_visualization = [0,0,0]#,0,0,0]
        
        neuro_leg = self.neuro_walknet.controller_objs[leg_nr]
        print(neuro_leg.leg.alpha)
        #plot_nr = 321
  #      plot_nr = 311
#        for neuro_leg in self.neuro_walknet.controller_objs:
    #        if (plot_nr < 314):
      #          self.neuronAct_visualization[plot_nr-311] = NeuronActivationLegFigure(neuro_leg, self.fig_activation.add_subplot(plot_nr), self.time_window)
        #    plot_nr += 1
        self.neuronAct_visualization[0] = JointPositionLegFigure(neuro_leg, neuro_leg.leg.alpha, [1,21], self.fig_activation.add_subplot(311), self.time_window) # for FL,0: 29,9,for ML,2: 29,9, for HL,4: 9,29 for FR,1: 9,29,for MR,3: 9,29, for HR,5: 9,29   
        self.neuronAct_visualization[1] = JointPositionLegFigure(neuro_leg, neuro_leg.leg.beta, [41,61], self.fig_activation.add_subplot(312), self.time_window)
        self.neuronAct_visualization[2] = JointPositionLegFigure(neuro_leg, neuro_leg.leg.gamma, [81,101], self.fig_activation.add_subplot(313), self.time_window)

        plt.tight_layout(h_pad=6, w_pad=4) # 4
        plt.ioff()
    
    ##
    #   Update figure.
    def post_processing_step(self, args=None):
        #if (self.neuronAct_visualization[0].it==self.time_window):
        #   plt.savefig("/Users/mschilling/Desktop/pos_during_sim.pdf")
        #   print("SAVED")
        #   input()
        if (self.counter > self.start_count):
            for i in range(0,3):
                if self.neuronAct_visualization[i] != 0 :
                    self.neuronAct_visualization[i].update_visualization()
        self.fig_activation.canvas.draw()
        self.counter +=1
        if (self.counter == self.end_count):
            import time
            timestr = time.strftime("%Y%m%d-%H%M%S")
            self.fig_activation.savefig("footfall/motor_output_" + timestr + ".eps")
            print("Wrote file")
            #self.fig_activation.clear()
            #self.
        #plt.pause(0.00001)
        
