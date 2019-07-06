import matplotlib.pylab as py
import matplotlib.pyplot as plt
import numpy as np

import Hector.RobotSettings as RSTATIC

from ProcessOrganisation.ProcessModule.ProcessingModule import ProcessingModule

##
#   Showing the foot points in the body centred cordinate system during stance movements.
#
#   The visualization is a ProcessingModule which pulls in the post processing step
#   updated values from the wrobot structure.
##
class FootPointVisualizationModule(ProcessingModule):
    
    ##
    #   Init figure and subplots.
    def __init__ (self, name, neuro_wn, robot):
        ProcessingModule.__init__(self, name)
        self.controller_objs = neuro_wn.controller_objs
        self.robot = robot
        plt.ion()
        """ Initialising the drawing window.
            Must be called before the visualisation can be updated
            by calling draw_manipulator"""
        self.footpoint = plt.figure(figsize=(6, 6))
        plt.axis('off')
        self.footpoint.canvas.set_window_title('Footpoint positions')
        
        self.width = 0.07
        self.coxae = [[-0.07,0.38],[0.07,0.38],
                    [-0.07,0.02],[0.07,0.02],
                    [-0.07,-0.2],[0.07,-0.2]]
        
        self.max_time = 4000 #100 iterations = 1 second
        plt.ioff()
        
    ##
    #   Init figure and subplots.
    def init_module(self):
        self.draw_counter = -1000
        self.lastswing = [-1,-1,-1,-1,-1,-1]
        self.ax_footpoint = self.footpoint.add_subplot(111)
        self.ax_footpoint.set_xlim(-1.,1.) 
        self.ax_footpoint.set_ylim(-1., 1.)
        #self.position_line, = self.ax_footpoint.plot([],[], color='blue')
        
        #self.ax_footpoint.plot([0.,self.width,0.,self.width, 0.,self.width,0.],
         #   [-0.36,-0.2,0.,0.02,0.35,0.38,0.57],
          #  linestyle='-', linewidth=2.0, color='gray', marker='o')
        #self.ax_footpoint.plot([0.,-self.width,0.,-self.width, 0.,-self.width,0.],
         #   [-0.36,-0.2,0.,0.02,0.35,0.38,0.57],
          #  linestyle='-', linewidth=2.0, color='gray', marker='o')
        self.ax_footpoint.plot([self.width,self.width, self.width],
            [-0.2,0.02,0.38],
            linestyle='-', linewidth=0.0, color='gray', marker='o')
        self.ax_footpoint.plot([-self.width,-self.width,-self.width],
            [-0.2,0.02,0.38],
            linestyle='-', linewidth=0.0, color='gray', marker='o')

        self.footposition_scatter, = self.ax_footpoint.plot([],[], ls='', marker='.', markersize=2)
        self.foot_position_x = []
        self.foot_position_y = []
        #self.ax_footfall.set_yticks([6,5,4,3,2,1])
        #self.ax_footfall.set_yticklabels(['FL', 'ML','HL','FR', 'MR','HR'])
        #py.rcParams['figure.figsize'] = 4, 4
#        self.ax_footfall.plot([0,self.max_time], [3.5,3.5], linestyle=':', linewidth=1.0, color='gray', alpha=0.7, marker='')
 #       self.ax_footfall.set_xlim(0,self.max_time) 
  #      self.ax_footfall.set_ylim(0.5,6.5)
        self.post_processing_step()
        self.footpoint.canvas.draw()
        plt.pause(0.00001)
        #plt.savefig("/Users/mschilling/Desktop/footpoints.pdf")
    
    ##
    #   Update visualization using data from the body model.    
    def post_processing_step(self, args=None):
        self.draw_counter+=1
        #if self.draw_counter%100 == 0:
        #   print(self.draw_counter)
        if (self.draw_counter > 0):
            for j in range(0,6) :
                if (self.controller_objs[j]):
                        if (self.controller_objs[j].v[122] < self.controller_objs[j].v[123]):    
                            #position = self.leg.input_foot_position
                            position = self.robot.legs[j].input_foot_position
                            self.foot_position_x.append( -position[1]+self.coxae[j][0] )
                            self.foot_position_y.append( position[0]+self.coxae[j][1] )
#                            self.ax_footpoint.scatter([-position[1]+self.coxae[j][0]],
 #                               [position[0]+self.coxae[j][1]], s=1., color='blue')

            if (self.draw_counter % 100 == 0):  # 1000
                self.footposition_scatter.set_xdata( self.foot_position_x )
                self.footposition_scatter.set_ydata( self.foot_position_y )  
                self.footpoint.canvas.draw()
                plt.pause(0.00001)
        if (self.draw_counter == self.max_time):
            import time
            timestr = time.strftime("%Y%m%d-%H%M%S")
            self.footpoint.savefig("footfall/foot_pos_post1_" + timestr + ".eps")
            print("Wrote file")
            self.ax_footpoint.clear()
            self.init_module()
