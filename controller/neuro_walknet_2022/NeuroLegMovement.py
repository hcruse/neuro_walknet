# -*- coding: utf-8 -*-
from Hector.RobotF import Robot  # _test
import numpy, math, random

import controller.neuro_walknet_2022.NeuroWNSettings as WNParams

from ProcessOrganisation.ProcessModule.ProcessingModule import ProcessingModule

class NeuroLegMovement:
    """"
    Leg controller: neural network for control of a single leg and its' joints.

    Describes structure of the neural network connections (see Fig. in repository and
    original article) and performs computation. Importantly, computation of neurons
    is called with a frequency of 1 kHz.

    Attributes
    ----------
    name :
        Name of the leg
    leg:
        interface to simulated robot leg - can read sensors and write motor commands

    Methods
    -------
    There are three basic methods - these are called from the outer process loop (runs at 100 Hz):
    update_joint_positions(self):
        First - new sensor values are pulled from the simulator.
    update_leg_controller(timeStamp):
        Second - neural net activation is propagated and processed.
        As the neurons are assumed to work faster than the given technical framework
        connection, we are updating during each control step the neural network 10 times
        each control step. During these updates there are no new sensor values,
        but this allows the network to settle.
        When there is a faster protocol allowing access to the robot, this could be
        synchronized.
    send_control_velocities(self):
        Third - joint velocities are the output of the neural network.
        These are used to drive the motors of the simulated robot.
    """
    def __init__(self, name, leg):
        self.name = name
        self.leg = leg

        # Set global velocity of walking
        self.vel = WNParams.velocity

        # To allow for testing noisy input on the motor level
        # additive noise value is given. If noisefct = 0, there is no noise given
        self.noisefct = 0.

        # Select current walking mode - turn on specific neurons
        if (WNParams.w_mode == WNParams.walking_modes["forward"]):
            self.forward, self.backward = 1., 0.
            self.Run = 0.
        elif (WNParams.w_mode == WNParams.walking_modes["backward"]):
            self.forward, self.backward = 0., 1.
            self.Run = 0.
        elif (WNParams.w_mode == WNParams.walking_modes["running"]):
            self.forward, self.backward = 1., 0.
            self.Run = 1.

        self.alpha, self.beta, self.gamma = None, None, None
        self.alpha_e, self.beta_e, self.gamma_e = None, None, None
        self.alpha_vel, self.beta_vel, self.gamma_vel = None, None, None

        self.n = 328 #######intraleg see Settings
        self.pilo2 = 0.

        self.CS = 0.    ###### intraleg
        self.min = 0. #15. for ramp only
        if WNParams.HellHess == 1: self.min = 15.   # zu intraleg 5Aug L87
        self.ChOpos, self.ChOvel = self.min,0.  ###### intraleg
        self.period, self.stim_duration = 0.,0.  ## intraleg

        # Setup of the neural net weight matrix
        self.WE = numpy.zeros( (self.n, self.n) ) # weights excitatory
        self.WI = numpy.zeros( (self.n, self.n) ) # weights inhibitory

        self.g = numpy.zeros(self.n) # synaptic input

        self.Sumgin = numpy.zeros(self.n) # synaptic input inhibitory
        self.Sumgex = numpy.zeros(self.n) # synaptic input excitatory
        self.zeroShift = - 60. # for mV values neurons

        self.v = numpy.empty(self.n) # voltage of neuron n
        self.v.fill(-60. - self.zeroShift)
        self.vn = numpy.empty(self.n)
        self.vn.fill(-60. - self.zeroShift) # new voltage of neuron n

        self.Sumg = numpy.zeros(self.n) #sum of synaptic input of neuron n

        self.Iapp = numpy.zeros(self.n) # external input to neuron n
        self.Iself = numpy.zeros(self.n) # current of neuron n

        self.LeakSwing = numpy.zeros(self.n) # HPF time constant of neuron n
        self.Rest = 0.
        self.outHPF = numpy.zeros(self.n) # internal variable for high pass filter
        self.outHPF2 = numpy.zeros(self.n)# internal variable for high pass filter
        self.auxHPF = numpy.zeros(self.n) # internal variable for high pass filter
        self.auxHPFold = numpy.zeros(self.n) # internal variable for high pass filter

        self.Cmem = numpy.empty(self.n) # membrane constant
        self.Cmem.fill(4.5) # tau = 4.5 ms
        self.alphaHvelout = 0. # velocity output alpha
        self.betaHvelout = 0.  # velocity output beta
        self.gammaHvelout = 0.# velocity output gamma
        self.betaswing = 0. # set point for beta negative feedback controller
        self.count = 0  # time counter (unit ms)

        # For disturbance tests only, swing prolongation of individual legs
        self.disturbduration = 0. #0. # parameter for disturbance signal
        self.disturbstart = 0. # parameter for disturbance signal, begin
        self.countdisturbtime = 0 # parameter for disturbance signal

        self.alphaMRe = 0.  # alpha joint position after elastic element
        self.betaMRe = 0.   # beta joint position after elastic element
        self.gammaMRe = 0.  # gamma joint position after elastic element
        self.alphaMRh = 0.  # alpha joint position before elastic element
        self.betaMRh = 0.   # beta joint position before elastic element
        self.gammaMRh = 0.  # gamma joint position before elastic element
        self.loadbeta = 0. #  load approximation: difference between beta joint position after and before elastic element
        self.fovel = 0. # factor velocity output
        self.ampl = 0. # step amplitude
        self.alpha_e = 0. # alpha joint position, elastic
        self.beta_e = 0.  # beta joint position, elastic
        self.gamma_e = 0. # gamma joint position, elastic
        self.shiftHl = 0.  # shift in ring net
        self.vellocal = 0.  # parameter for leg specific velocity
        self.AEPloadCurve = 0. # alpha position beyond which load can stop swing
        self.ampl = 33.  # normal step amplitude

        # CPG, membrane constants Cmem
        self.Cmem[122],self.Cmem[123] = 5.,5.   # Swing - Stance
        self.Cmem[124], self.Cmem[125] = 3.5,3.5   # forward - backward
        self.Cmem[37] = 5.
        self.Cmem[77] = 5.
        self.Cmem[117] = 5.
        self.Cmem[16] = 5.
        self.Cmem[56] = 5.
        self.Cmem[96] = 5.
        self.Cmem[155] = 10.
        self.Cmem[154] = 80.
        self.Cmem[160] = 5.
        self.Cmem[313] = 80.
        self.Cmem[317] = 10.
        self.Cmem[319] = 100.

#        ### vel 11.10.19   L 142
        self.alpha_old = 0.            ###
        self.beta_old = 0.             ###
        self.gamma_old = 0.            ###
        self.alphae_old = 0.           ###
        self.betae_old = 0.            ###
        self.gammae_old = 0.           ###
        self.alpha_vel = 0.            ###
        self.beta_vel =  0.            ###
        self.gamma_vel = 0.            ###

        self.alphae_vel = 0.           ###
        self.betae_vel =  0.           ###
        self.gammae_vel = 0.           ###  end vel 11.10.19
        self.CS = 0.
        self.start = 0.      #####intraleg  L163
        self.countStim = 0.  ######intraleg L164

        # Define strength of excitatory weights
        we = 1.
        # Define strength of inhibitory weights
        wi = 1.

        ##################################################
        # Setting the weight matrix of the neural network:
        # see article and figure in repository (includes neuron numbers)
        #
        #### Weights controlling stance - swing and forward - backward
        self.WE[121][122],self.WE[122][121],self.WE[121][123],self.WE[123][121] = we,we,we,we # WTA net swing - stance
        self.WE[126][122],self.WE[127][123] = we,we # WTA net swing - stance
        self.WI[123][126],self.WI[122][127] = 10.,10. # WTA net swing - stance
        self.WE[121][124],self.WE[124][121],self.WE[121][125],self.WE[125][121] = we,we,we,we # WTA net forward-backward
        self.WE[133][124],self.WE[132][125] = we,we # WTA net forward-backward
        self.WI[124][132],self.WI[125][133] = 10.,10.  # # WTA net forward-backward
        self.WE[134][123] = 20.   # inhibit swing
        self.WE[135][122] = we  # inhibit stance

        #### Coordination rules
        self.WE[120][121] = we   # bias to coordin rules and short step
        self.WI[137][126] = wi # to coordination rules
        self.WE[171][141],self.WE[122][171],self.WE[142][171] = we,we,we  # for coordination rules 1 - 3 input
        self.WE[123][170],self.WE[170][140] = we,we # input to stance

        if WNParams.intraleg[self.leg.name] == 1.:        # L 193     #######intraleg   Y2
             self.WE[171][141],self.WE[170][140] = 0.,0.              ####### switch off rule 1-3 input
        if WNParams.pilo[self.leg.name] == 1.:        # L 195     #######pilo
             self.WE[171][141],self.WE[170][140] = 0.,0.              ####### switch off rule 1-3 input
        #### legs deaff, therefore no input from other legs, for Hellekes, see Methods

        self.WI[126][170] = 0.    #########intraleg  Stimulus 170 inhibits 126 Inhibitor of ST
        if WNParams.intraleg[self.leg.name] == 1.: self.WI[126][170] = 10.   #########intraleg L199
        self.WI[123][142] = wi  # input to stance
        self.WE[155][122] = we  # rule 1a
        self.WI[143][126] = 10.   # rule 1b,, 2i
        self.WE[143][143] = 0.9999 # rule 1b 2i
        self.WE[143][121] = 0.0001  # rule 1b,2i
        self.WI[144][120] = 0.004  # rule 1b, 2i
        self.WI[144][162] = 1. # vel to 1b, 2i
        self.WE[144][143] = 1. # ruli 2i, 1b
        self.WE[153][144] = 40.  # rule 1b
        self.WE[146][143] = 40.  # rule 1b
        self.WI[146][153] = 5. #  rule 1b
        self.WE[154][155], self.WE[158][155], self.WE[158][154] = 1.,1.,1. # rule 1a-1b
        self.WE[145][144], self.WE[160][145] = 90.,90.  # rule 2i
        self.WI[149][147] = 25.  # rule 3i
        self.WI[147][8],self.WE[149][148] = wi,2.  # rule 3i
        self.WI[164][8] = wi # rule 3i
        self.WE[148][164] = 10.  # rule 3i
        self.WI[164][126],self.WI[147][126] = 5.,5.   # rule 3i
        self.WI[156][8],self.WE[166][156] = wi, 90. # rule 2c
        self.WE[166][159] = 90.  # rule 2c Backward
        self.WE[167][166], self.WE[157][167] = we,10.  # rule 2c
        self.WI[167][120] = 0.18   # rule 2c
        self.WI[150][8],self.WI[165][8],self.WI[150][126],self.WI[165][126] = wi,wi,5.,5.  # rule 3c
        self.WI[152][168],self.WI[151][165] = 80., 50. # rule 3c
        self.WE[151][150],self.WE[152][151] = 5.,5.  # rule 3c

        self.WE[169][180] = 1. # input to box local(theta)
        self.WI[137][169] = 1.

        # Rule 3i
        self.WE[164][180] = 2.4/4.3 # 0.56 # rule 3i, input: velocity
        self.WI[164][137] = 0.25  # rule 3i, input: 50 - velocity
        self.WE[147][180] = 0.4/4.3 # # rule 3i, input: velocity
        self.WI[137][169] = 1. # rule 1, 2i, 3i, input: velocity
        # Rule 2i
        self.WE[162][161] = 0.75 * 0.04*3./(5.*0.85) # 0.021 # rule 1, 2i input: velocity
        # Rule 2c
        self.WI[156][8],self.WE[159][8] = 1.,1. # input: alpha position
        self.WI[156][132],self.WI[159][133] = 10.,10.  # inhibition during backward, resp. forward
        # rule 5  retractor-protractor
        # in:
        self.WE[179][173] = 1. # input from rule 5i
        self.WE[179][174] = 1. #  input from rule 5c
        self.WE[178][175] = 1. # input, from rule 5ch
        self.WE[22][179] = 0.3 # to retractor
        self.WE[177][179],self.WI[2][177] = 1.,2. # to protractor
        self.WE[176][178],self.WI[22][176] = 1.,0.3 # to retractor
        self.WE[2][178] = 1.   # rule 5ch, to protractor
        self.WI[179][126] = 30. # rule 5c, inhibitory input during Swing
        self.WI[178][134] = 30. # inhibition of input during Stance, antiphase influence only during swing
        if WNParams.Deaffonly[self.leg.name] == 1.:self.WE[179][174] = 0. # rule 5c input off
        if WNParams.pilo[self.leg.name] == 1.: # leg treated with pilocarpine
             self.WI[179][126] = 0. # rule 5c, inhibitory input during Swing, off
             self.WI[178][134] = 0. # inhibition of input during Stance, off
        # out:
        self.WE[172][22] = 1 # rule 5, output
        self.Iapp[172] = -20.  # threshold for output
        # Rule 5  Levator - Depressor # All legs deafferented
        # in:
        self.WE[309][303] = 1. # input from rule 5i
        self.WE[309][304] = 1. #  input from rule 5c
        self.WE[308][305] = 1. # input, from rule 5ch
        self.WE[62][309] = 0.3 # to depressor
        self.WE[307][309],self.WI[42][307] = 1.,2. # to levator
        self.WE[306][308],self.WI[62][306] = 1.,0.3  # to depressor
        self.WE[42][308] = 1.   # rule 5ch, to levator
        self.WI[309][126] = 30. # rule 5c, inhibitory input during Swing
        self.WI[308][134] = 30. # inhibition of input during Stance, antiphase influence only during swing
        if WNParams.Deaffonly[self.leg.name] == 1.:self.WE[309][304] = 0. # rule 5c input off
        if WNParams.pilo[self.leg.name] == 1.: # leg treated with pilocarpine
             self.WI[309][126] = 0. # rule 5c, inhibitory input during Swing, off
             self.WI[308][134] = 0. # inhibition of input during Stance, off
        # out:
        self.WE[302][62] = 1.  # rule 5, output
        self.Iapp[302] = -20. # threshold for output

        if WNParams.intraleg[self.leg.name] == 1.: # Aug 9, required for rule5 off: Akay, Hess, also for Curve if treated (Hellekes, see Methods), but not for free curve walking L280 / 295     Y8
             self.WE[62][309], self.WE[307][309], self.WE[42][308], self.WE[306][308] = 0.,0.,0.,0.
             self.WE[2][178],self.WE[176][178],self.WE[177][179],self.WE[22][179] = 0.,0.,0.,0.

        # short steps at PEP
        self.WE[46][66] = 1.
        self.WI[46][120] = 0.9
        self.WI[138][46] = 80.

        # During state "Run", coordination rules 1 - 3 and 5 are not active, due to sensory delay
        # instead, rule 5 P (Pearson) is activated, sensory input is not required
        # Rule 5 P weights inhibit levation of neighboring legs during levation of sender leg
        # Fig 2:  bright-yellow units, pink lines
         # weights for Rule 5 P (Pearson)
        self.WE[301][42] = 1.  # output from levator premotor neuron
        self.WE[312][310] = 1.  # input from ipsilateral legs
        self.WE[312][311] = 1.  # input from contralateral leg
        self.WE[318][312] = 1.
        self.WI[42][318] = 4. # inhibits levator premotor neuron
        self.WE[63][312] = .2 # activates inihibitory unit
        self.WE[319][312] = 5.
        self.WE[320][319] = 4.
        self.WI[42][320] = 4.  # inhibits levator premotor neuron
        self.WE[322][121] = 1.
        self.WI[312][322],self.WI[322][321] = 10.,5.  # disinhibiton of rule 5P through Run[321]

        # sensor input off, coactivation on activate CPG
        self.WI[172][321],self.WI[302][321] = 10.,10. # rule 5, 5D off
        self.WI[140][321],self.WI[141][321] = 10.,10. #  rule 1-3 in: off
        self.WI[4][321],self.WI[24][321] = 10.,10.  # sensory input alpha branch: off
        self.WI[84][321],self.WI[104][321] = 10.,10. # sensory input gamma branch: off
        self.WI[44][321],self.WI[64][321] = 10.,10.  # # sensory input beta branch: off
        self.WE[2][321],self.WE[22][321] = 6./50.,4./50.  # coactivation of premotor units alpha
        self.WE[82][321],self.WE[102][321] = 2./50.,1./50. # coactivation of premotor units gamma
        self.WE[42][321],self.WE[62][321] = 3./50.,2./50.  # coactivation of premotor units beta

        # SRP,  intraleg coupling: from beta to alpha and to gamma
        #NN, but not necessary muss aber noch abgeschaltet werden    ???
        self.WE[313][325],self.WE[314][313],self.WE[313][313] = .05,1., 0.9
        self.WE[22][314],self.WI[314][120] = 100.,.015  # output to retractor
        self.WE[315][325],self.WE[316][314],self.WI[315][316] = 1.,1.,20.
        self.WE[317][324],self.WE[102][324] = 20.,20 # output to extensor
        self.WE[82][325],self.WE[2][317],self.WE[2][315] = 20., 3.,20. # output to flexor, protractor
        self.WE[324][42],self.WE[325][62] = 1.,1. # input from levator, depressor
        self.WI[324][326],self.WI[325][326] = 10.,10.
        self.WI[326][321],self.WE[326][121] = 10., 1. # disinhibition through Run[321]

        # Intact walking legs and deafferented legs, Standing legs in walking insect
        if WNParams.standL[self.leg.name] == 1.: self.WE[172][22] = 2. # leg standing on force transducer

        # bias and starting values
        self.Erest = -60. - self.zeroShift  # resting potential -60 mV in neurons, but here shifted to 0 mV
        self.Vmax = 50.  + self.Erest - self.zeroShift  # = 50. mV upper potential
        self.v[120] = self.Vmax # bias
        self.v[121] = self.Vmax # leg: on
        self.v[122] = self.Erest # ground contact off, swing: on   start stance
        self.v[123] = self.Vmax # groound contact on, stance: off    start stance
        self.v[126] = self.Erest # inhib groud contact_on: on
        self.v[127] = self.Vmax # inhib ground contact_off: off
        # Set the direction for the leg controller
        if self.forward == 1.:
            self.v[124] = self.Vmax # local_fw: on
            self.v[125] = self.Erest # local_bw: off
            self.v[132] = self.Erest # inhib fw: off
            self.v[133] = self.Vmax # inhib bw: on
        if self.backward == 1.:
            self.v[124] = self.Erest # local_fw: off
            self.v[125] = self.Vmax # local_bw: on
            self.v[132] = self.Vmax # inhib fw: on
            self.v[133] = self.Erest # inhib bw: off

        self.v[134] = self.Erest #
        self.v[135] = self.Vmax #
        self.v[137] = self.Erest  # global velocity, maximum
        self.v[138] = self.Erest #  HPF swing
        self.v[139] = self.Erest #  HPF swing, threshold

        ################
        # General structure of the individual joint controller,
        # repeated for every joint
        for joint_nr in range(0,3):
            #  protractor  1 - 19, levator 41 - 59,  flexor 81 - 99,
            self.WE[40*joint_nr+1][40*joint_nr+2],self.WE[40*joint_nr+2][40*joint_nr+4],self.WE[40*joint_nr+3][40*joint_nr+2] = we,we,2.
            self.WE[40*joint_nr+4][40*joint_nr+5], self.WE[40*joint_nr+5][40*joint_nr+7] = we, we
            self.WE[40*joint_nr+7][40*joint_nr+10]= we
            self.WE[40*joint_nr+4][40*joint_nr+11] = we
            self.WI[40*joint_nr+22][40*joint_nr+3], self.WI[40*joint_nr+7][40*joint_nr+8] = 5.,wi

            #  retractor  21 - 39, depressor 61 - 79, extensor 101 - 119
            self.WE[40*joint_nr+21][40*joint_nr+22],self.WE[40*joint_nr+22][40*joint_nr+24],self.WE[40*joint_nr+23][40*joint_nr+22] = we,we,2.
            self.WE[40*joint_nr+24][40*joint_nr+25], self.WE[40*joint_nr+25][40*joint_nr+27] = we, we
            self.WE[40*joint_nr+27][40*joint_nr+30] = we
            self.WE[40*joint_nr+24][40*joint_nr+31] = we
            self.WI[40*joint_nr+2][40*joint_nr+23],self.WI[40*joint_nr+27][40*joint_nr+28] = 5.,wi

            # Initial Values
            self.v[40*joint_nr+8] = self.Vmax/2.   #  sensor input Pro, Lev, Flex
            self.v[40*joint_nr+28] = self.Vmax/2.  #  sensor input Ret, Dep, Ext

            # For running mode only:
            # we added specific output strengths for each motor neuron and each leg
            if self.Run == 1.:
                if self.leg.name == "front_left_leg":
                    self.WE[1][2],self.WE[41][42],self.WE[81][82] =     0.23*0.85,2.3*2,0.3   #Pro,Lev,Flex
                    self.WE[21][22],self.WE[61][62],self.WE[101][102] = 1.2*.57*0.85,1.33*5.,0.26  #Ret,Dep,Ext
                elif self.leg.name == "front_right_leg":
                    self.WE[1][2],self.WE[41][42],self.WE[81][82] =     0.35*1.4,2.8*4,0.31   #Pro,Lev,Flex
                    self.WE[21][22],self.WE[61][62],self.WE[101][102] = 1.8*.55*1.4,1.4*10,0.6  #Ret,Dep,Ext
                elif self.leg.name == "middle_left_leg":
                    self.WE[1][2],self.WE[41][42],self.WE[81][82] =     0.27*1.35,2.5*4,0.22   #Pro,Lev,Flex
                    self.WE[21][22],self.WE[61][62],self.WE[101][102] = 0.8*.6*1.35*1.5,1.33*10,.42  #Ret,Dep,Ext
                elif self.leg.name == "middle_right_leg":
                    self.WE[1][2],self.WE[41][42],self.WE[81][82] =     0.22*1.3*1.2,2.5*4,0.21    #Pro,Lev,Flex
                    self.WE[21][22],self.WE[61][62],self.WE[101][102] = 1.*.55*1.3,1.33*10,0.45  #Ret,Dep,Ext.
                elif self.leg.name == "hind_left_leg":
                    self.WE[1][2],self.WE[41][42],self.WE[81][82] =     0.2*1.15,3.5*1.5,0.17    #Pro,Lev,Flex
                    self.WE[21][22],self.WE[61][62],self.WE[101][102] = 1.1*.6*1.15,1.4*5,0.35  #Ret,Dep,Ext
                elif self.leg.name == "hind_right_leg":
                    self.WE[1][2],self.WE[41][42],self.WE[81][82] =     0.21*1.05,3.5*1.8,0.18 #Pro,Lev,Flex
                    self.WE[21][22],self.WE[61][62],self.WE[101][102] = 1.1*.6*1.05,1.33*5,0.38  #Ret,Dep,Ext

        # control swing - stance branches, input from WTA net Swing - Stance
        self.WE[10][13],self.WE[10][14] =  we,we # input set points, swing, protractor
        self.WI[11][135] = wi # inhibit stance, protractor
        self.WI[31][135] = wi # inhibit stance, retractor
        self.WE[30][33],self.WE[30][34] =  we,we # input set points, swing, retractor
        self.WE[90][93],self.WE[90][94] =  we,we # input set points, swing, flexor
        self.WI[91][135] = wi   # inhibit stance, flexor
        self.WI[111][135] = wi  # inhibit stance, extensor
        self.WE[110][113],self.WE[110][114] =  we,we # input set points, swing, extensor

        #  swing branch protractor, retractor, flexor, extensor,
        self.WE[6][7],self.WE[4][6] = 100.,0.2
        self.WE[86][87],self.WE[84][86] = 100.,0.2
        self.WE[26][27],self.WE[24][26] = 100.,0.2
        self.WE[106][107],self.WE[104][106] = 100.,0.2
        self.WI[7][134],self.WI[87][134] = 10.,10. # inhibit stance, protractor, flexor
        self.WI[27][134],self.WI[107][134] = 10.,10. # inhibit stance, retractor, extensor

        #   Connections from Ring net to set points
        self.WE[11][15],self.WE[11][36] = we,we
        self.WE[31][35],self.WE[31][16] = we,we
        self.WE[15][191],self.WE[16][191] = we,we
        self.WE[35][192],self.WE[36][192] = we,we

        # control of set points by forward, backward for alpha joint and gamma joint
        self.WI[13][132],self.WI[14][133],self.WI[33][132],self.WI[34][133] = 10.,10.,10.,10.
        self.WI[93][132],self.WI[94][133],self.WI[113][132],self.WI[114][133] = 10.,10.,10.,10.

        # control of ring input by forward, backward for alpha joint and gamma joint
        self.WI[15][132],self.WI[36][133],self.WI[35][132],self.WI[16][133] = 10.,10.,10.,10.
        self.WI[95][132],self.WI[116][133],self.WI[115][132],self.WI[96][133] = 10.,10.,10.,10.
        self.WE[95][193],self.WE[96][193] =  we,we
        self.WE[115][194],self.WE[116][194] = we,we
        self.WE[91][95],self.WE[91][116] =  we,we
        self.WE[111][115],self.WE[111][96] = we,we

        ################
        # Setting up the Ring net
        for i in range(13):
            self.WE[190][8] = 4./50.
            self.WE[221+i][201+i] = 1.
            if i > 0: self.WI[221+i-1][201+i] = 2.
            self.WI[241+i][221+i] = 1.
            self.WI[261+i][241+i] = 20.
            self.WI[281+i][241+i] = 20.
            self.WE[241+i][120] = 1./50.

        # Definition of ring net weights (see Fig 10). i: number of unit in Ring net, one unit represents 15 degrees
        for i in range(13):
            self.shiftHl = 0
            if WNParams.shiftHl[self.leg.name] == 1: self.shiftHl = -1 # zero position of hind leg ís shifted rearwards
            self.WE[261+i][260] =  math.fabs(math.sin((i+self.shiftHl)*3.14/12.))*0.2       # sin
            self.WE[281+i][260] =  math.fabs(math.cos((i+self.shiftHl)*3.14/12.))*0.2       # cos
            if WNParams.shiftHl[self.leg.name] == 1: self.WE[273][260] = 0.
            self.WE[192][261+i] = 1. # alpha Retractor
            self.WE[191][261+i] = 0. # alpha Protractor
        for i in range(7): #  gamma Extensor
            self.WE[194][281+i] = 1.
        for i in range(7,12):  # gamma Flexor
            self.WE[193][281+i] = 1.

        # Motor weights for lat. inhibition for alpha joint, beta joint and gamma joint
        # are overwritten here to influence CPG frequency
        self.factorfr = WNParams.CPGfrequ[self.leg.name]   # alpha joint
        self.WE[3][2],self.WE[23][22] = 2.*self.factorfr, 2.*self.factorfr  # for test CPG different frequencies per leg
        self.WE[43][42],self.WE[63][62] = 2.,2. # beta joint
        self.WE[83][82],self.WE[103][102] = 0.5,0.5  # gamma joint

        #################
        # Control of beta joint: Height net
        self.Iapp[54] = 50.
        self.WE[53][54],self.WE[78][54] = we,we
        self.WE[50][58],self.WE[70][78] = we,we
        self.WE[79][58],self.WE[58][55],self.WE[58][59] = we,we,we
        self.WI[59][53],self.WI[78][79],self.WI[59][122],self.WI[55][123] = wi,wi,wi,wi

        # HPF Swing lift
        self.betaswing = WNParams.SwingSetpoint[self.leg.name] # set point for beta negative feedback controller
        # setpoint for beta_negFB controller during Swing; 10mV = 30+12 degrees
        self.Cmem[138] = 5.  # decay HPF Swing
        self.WE[138][122], self.WE[139][138] = we,we  # HPF for Swing levator
        self.WE[45][138] = WNParams.SwingWeight[self.leg.name]*0.4 # swing lift amplitude
        self.WI[65][139] = 10.  # swing lift to depressor
        self.WE[104][138] =  3.  # swing lift to extensor
        self.LeakSwing[138] = WNParams.SwingTau[self.leg.name] # Tau for HPF

        # General settings for the leg
        if self.leg.left_leg:
            self.orientation_factor = 1
        else:
            self.orientation_factor = -1
        if 'hind' in self.leg.name:
            self.hind_leg_fact = -1
        else:
            self.hind_leg_fact = 1
        self.PI = 3.14
        print("Initialized leg controller - ", self.leg.name)

    def applyBandPassFilter(self, neuron, leak_1, leak_2):
        """
        Application of high pass (band pass filter as a model for neurons) filter
        """
        self.auxHPF[neuron] = self.auxHPF[neuron] +  self.outHPF[neuron]
        if self.auxHPFold[neuron] < self.auxHPF[neuron]:  # increasing
            self.outHPF[neuron] = self.vn[neuron] - self.auxHPF[neuron]* leak_1
        else:
            self.outHPF[neuron] = self.vn[neuron] - self.auxHPF[neuron]* leak_2
        self.auxHPFold[neuron] = self.auxHPF[neuron]
        self.outHPF2[neuron] = self.outHPF[neuron]
        if self.outHPF2[neuron] < 0.: self.outHPF2[neuron] = 0.
        return self.outHPF2[neuron]


    def update_joint_positions(self):
        """
        Update current sensor information

        Called before processing of the neural network once.
        Pulls sensor values from the simulator and buffers these into variables:
            joint positions for alpha, beta, and gamma
        The rotational joints are not directly coupling two segments -
        there is an elastic element (a spring) between the two connected parts.
        As a consequence, the rotational movement of the motor is acting on the spring
        which creates a torque that is then moving the outgoing segment.
        This helps when dealing with strong impacts and can stabilize behavior.
        But as a consequence: there are two different positons for each joint
            - the exact position of the motor (where this is driven to)
            - and the resulting joint position which is a result of the loaded spring

        The difference of these two angles represents how much the spring is
        loaded and gives us the torque acting on that joint.
        """
        # Update sensor values that are processed by the neural net:
        # pull new values from robot.
        # In case of alpha joint (specific alignment): apply offset
        self.alphaMRh = self.leg.alpha.inputPosition + WNParams.alpha_offset[self.leg.name]
        self.betaMRh = self.leg.beta.inputPosition
        self.gammaMRh = self.leg.gamma.inputPosition
        # Feedback from robot, joint angle after elastic element,
        self.alphaMRe = self.leg.alpha.outputPosition + WNParams.alpha_offset[self.leg.name]
        self.betaMRe = self.leg.beta.outputPosition
        self.gammaMRe = self.leg.gamma.outputPosition

     #   self.alpha_e, self.beta_e, self.gamma_e = self.alphaMRe,self.betaMRh,self.gammaMRh  # elastic alpha
        self.alpha_e, self.beta_e, self.gamma_e = self.alphaMRe,self.betaMRe,self.gammaMRe # corr 23.10.19  L523
        self.alpha , self.beta, self.gamma = self.alphaMRh,self.betaMRh,self.gammaMRh # hard


    def send_control_velocities(self):
        """
        Set control signals for motors

        At the end of each control step (called from the outside loop) the neural net
        has as outputs control signals for movement of the joint motors. These are applied
        as joint velocities (i.e. a positional difference over a time step).
        """
        self.leg.alpha.desiredValue_ISC = float(self.alphaHvelout) # vel-output to robot
        self.leg.beta.desiredValue_ISC = float(self.betaHvelout)
        self.leg.gamma.desiredValue_ISC = float(self.gammaHvelout)
        self.alphaHvelout = 0
        self.betaHvelout = 0
        self.gammaHvelout = 0

    def update_leg_controller(self, timeStamp):
        """
        Update leg controller - computation of the neural network

        Activation inside the neural net is propagated and processed.
        Using the weight matrices, current activations to propagate activations inside
        the network and apply output functions.

        As the neurons are assumed to work faster than the given technical framework
        connection, we are updating during each control step the neural network 10 times
        each control step. During these updates there are no new sensor values,
        but this allows the network to settle.

        Parameters
        ----------
        timestamp : float
            Current timestep.
        """
        self.count = self.count +1
        E = self.Erest  # = 0.
        f = 180./self.PI

        # for curve, vel = 45. #as proof of concept (Supplement)
        self.v[321] = 0. # (slow) Walk
        if self.Run == 1:
             self.v[321] = 50. # Run

        # Experimental Settings:
        # Testing starting or interruption of walks, tested, but not shown in articles
        #   if self.count < 1000: self.vel = 0. # start walk
        #   if self.count > 5000: self.vel = 0. # stop walk
        # SplitBelt (simple curve walking = left and right side walk at different speeds)
        #   tested, but not shown
        # if WNParams.SplitBelt[self.leg.name] == 1.: self.vel = self.vel * .5 # left legs slower

        self.Iapp[180] = self.vel
        self.Iapp[137] =  50.

        # rule 3i
        self.Iapp[164]  = WNParams.pep[self.leg.name] # 3i threshold depends on velocity, 3i on
        self.Iapp[147] =  WNParams.pep[self.leg.name] - 3. # 3i threshold depends on velocity, 3i off

        # rule 2i  #NN
        self.Iapp[161] = (50. - self.v[180])*(50./30.)*0.85 #

        # rule 2c
        self.Iapp[156] =  0.87*self.ampl + WNParams.pep[self.leg.name] # defines position threshold, forward
        self.Iapp[159] = -(0.4 *self.ampl + WNParams.pep[self.leg.name]) # # defines position threshold, backward

        # rule 3c
        self.Iapp[150] = 0.57 * 33. + WNParams.pep[self.leg.name]  # defines position threshold, 3c on
        self.Iapp[165] = 0.57 * 33. + WNParams.pep[self.leg.name] - 3.  # defines position threshold,3c off
        self.Iapp[168] = self.v[180] - 12. # threshold for inhib of 3c, active for vel < 12 mV

        # Intact walking legs and deafferented legs, Standing legs in walking insect
        if WNParams.frictW[self.leg.name] == 1.: # leg walking on treadmill
              self.WE[172][22] = 1.
              if self.v[8] > 25. and self.v[123] > self.v[122]: #  # front position range during Stance
                  self.WE[172][22] = 15. # 40. #high load to overcome threshold of 20. strong load due to friction
        if WNParams.standL[self.leg.name] == 1.: self.WE[172][22] = 2.

        if self.Run > 0.5:  # activation of depressor premotor neuron to reach sensible starting configuration
            # for all legs, during about the first 500 ms
            if self.leg.name == "front_left_leg":
                 if self.count < 200: self.v[62] = 10.
            if self.leg.name == "middle_right_leg":
                 if self.count < 210: self.v[62] = 10.
            if self.leg.name == "hind_left_leg":
                 if self.count < 220: self.v[62] = 10.
            if self.leg.name == "front_right_leg":
                 if self.count < 400: self.v[62] = 10.
            if self.leg.name == "middle_left_leg":
                 if self.count < 420: self.v[62] = 10.
            if self.leg.name == "hind_right_leg":
                 if self.count < 410: self.v[62] = 10.

        # sensor values, position, degrees to mV
        self.Iapp[8] = (self.orientation_factor * self.alpha + WNParams.offset[self.leg.name])*50. # alpha actual value in mV, hard version: reference input
        #  self.Iapp[8] = (self.orientation_factor * self.alpha_e + WNParams.offset[self.leg.name])*50. + WNParams.offset_el[self.leg.name]  # elastic version, actual output
        if WNParams.intraleg[self.leg.name] == 1.:
            self.Iapp[8] = 35. #20.  #   L 649 #######intraleg ,fix alpha position
            if self.v[125] > self.v[124]: self.Iapp[8] = 35. # BW
        self.Iapp[48] = (-self.orientation_factor * self.hind_leg_fact * self.beta - self.PI/6.)*50./(self.PI/3.) # includes (+30°) psi shift
        self.Iapp[88] = (-self.orientation_factor * self.hind_leg_fact * self.gamma*0.5 + 0.5)*50. #

        # Hellekes and Hess experimental condition
        if WNParams.HellHess == 1:
            self.Iapp[88] = self.ChOpos   #

        # Definition of leg position for "Standing legs in walking insect"
        if WNParams.standL[self.leg.name]  == 1:
            self.Iapp[8] = WNParams.standLalpha[self.leg.name]
            self.Iapp[48] = WNParams.standLbeta[self.leg.name]
            self.Iapp[88] = WNParams.standLgamma[self.leg.name]

        # lower and upper limits for position values
        if self.Iapp[8] < 0.: self.Iapp[8] = 0.
        if self.Iapp[8] > 50.: self.Iapp[8] = 50.
        if self.Iapp[48] < 0.: self.Iapp[48] = 0.
        if self.Iapp[48] > 50.: self.Iapp[48] = 50.
        if self.Iapp[88] < 0.: self.Iapp[88] = 0.
        if self.Iapp[88] > 50.: self.Iapp[88] = 50.
        self.Iapp[28] = 50. - self.Iapp[8] # alpha actual value
        self.Iapp[68] = 50. - self.Iapp[48] # beta actual value
        self.Iapp[108] = 50. - self.Iapp[88] # gammaExt actual value

        ################
        # Alpha
        if self.forward == 1.:
            self.Iapp[13] = 50. # reference input for alpha (protractor), swing, forward
            # Adjustment for curve walking
            if WNParams.curve_walking:
                # Adjust protractor for inner front leg of the example
                if self.leg.name == "front_right_leg":
                    self.Iapp[13] = 20.
                # Adjust protractor for outer front leg of the example
                if self.leg.name == "front_left_leg":
                    self.Iapp[13] = 50.
            self.Iapp[33] = 50.-self.Iapp[13] # reference input for alpha (retractor), swing, forward
        if self.backward == 1.:
            self.Iapp[14] = 0. # reference input for alpha (protractor), swing, backward
            self.Iapp[34] = 50.-self.Iapp[14] #  reference input for alpha (retractor), swing, backward

        ################
        # Gamma
        if self.forward == 1.:
            self.Iapp[93] = WNParams.gammamorphAEP[self.leg.name] # reference input for gamma (flexor), swing, forward
            # Adjustment for curve walking
            if WNParams.curve_walking:
                # Adjust flexor for inner front leg of the example
                if self.leg.name == "front_right_leg":
                    self.Iapp[93] = 20.
                # Adjust flexor for outer front leg of the example
                if self.leg.name == "front_left_leg":
                    self.Iapp[93] = 45.
            self.Iapp[113] = 50.- self.Iapp[93] # reference input for gamma (extensor), swing, forward
        if self.backward == 1.:
            self.Iapp[94] = WNParams.gammamorphPEPBW[self.leg.name] # reference input for gamma (flexor), swing, backward
            self.Iapp[114] = 50.- self.Iapp[94] # reference input for gamma (extensor), swing, backward

        ################
        # Beta
        # Height control beta, swing set point
        if self.forward == 1.:
            self.Iapp[55] = 31. - self.v[8] * 0.5 # swing set point for forward walking
        if self.backward == 1.:
            self.Iapp[55] = 31. - self.v[28] * 0.5 # swing set point for backward walking

        # Height control beta, stance set point
        self.Iapp[59] = self.Iapp[88] # set point for beta stance, input from gamma sensor
        self.WE[53][54] = 2.5/(32. - 0.6*self.Iapp[88])- 0.1 # presynaptic inhibition

        # curve walking, local leg velocity, depends on leg# and on theta # 764
        self.vellocal = 1.

        # During curve walking: adapt local velocities of controllers
        # (differentiating the individual legs = from higher control level)
        # Currently, this is set for one specific example (only detailed experiment)
        # This could be replaced in the future through a simple NN that
        # determines these values (it puts the different leg contributions to
        # curve walking into relation - this requires a form of global perspective
        # on the legs and might form a kind of internal model)
        # TODO: OK?
        if WNParams.curve_walking:
            # Inner legs:
            if self.leg.name == "front_right_leg": self.vellocal = .75  # FR
            elif self.leg.name == "middle_right_leg": self.vellocal = .35  # MR
            elif self.leg.name == "hind_right_leg": self.vellocal = 0.1  # HR
            # Outer legs
            elif self.leg.name == "front_left_leg": self.vellocal = 1.   # FL
            elif self.leg.name == "middle_left_leg": self.vellocal = 1.   # ML
            elif self.leg.name == "hind_left_leg": self.vellocal = 1.   # HL

        # local(theta)
        # Local velocities of the legs modulate one part of the control network
        # (currently -see above- there is only one given curvature used
        # but when curves are negotiated freely this would require a form of modulation
        # of the underlying control circuit)
        self.WE[260][169] = self.vellocal

        # Curve walking - calculation of individual leg. direction for stance movements
        # Selected example for theta and leg#, in box local(theta) and box spatial coding
        # approximated to Duerr, Ebeling 2005
        self.theta = 0.
        # Adjustment for curve walking
        if WNParams.curve_walking:
            # Adjust theta for inner front leg of the example
            if self.leg.name == "front_right_leg":
                self.theta = 5.+1.
            # Adjust theta for outer front leg of the example
            if self.leg.name == "front_left_leg":
                self.theta = -5.*0.4

        self.v[190] = 4.5 + self.v[8]*self.WE[190][8] + self.theta # delta ~ alpha* + theta, spatial code
        for i in range(12):
            self.v[201+i] = 0.
            if i < self.v[190]: self.v[201+i] = 1.

        # short steps at PEP
        self.WI[66][260] = self.vellocal
        self.Iapp[66] = 50.

        countstart =  0 #
        if WNParams.pilo[self.leg.name] == 1.:
                  countstart = 10  # CPG, pilo on

        #  CPGs on if Stance  >  Swing and after 10 ms. For "All legs deafferented" and "Intact walking legs and deafferented legs"
        if (self.v[123]) > (self.v[122]) and (self.count >= countstart):
                if WNParams.pilo[self.leg.name]  == 1: #
                   #  self.Iapp[2] = 25. #40.  # pilocarpine  protractor
                     self.pilo2 =     3. #40. #40. #neu # klass
                     self.Iapp[22] =  6. #30. #25. #neu # klass  # pilocarpine  retractor
                     self.Iapp[44] = 13. #8. #15. #40. #neu # klass  # pilocarpine levator
                     self.Iapp[64] =  3. #6. #30. #25. #neu # klass  # pilocarpine depressor
                     self.Iapp[82] =  5.#  20. #30. #50. #neu # klass  # pilocarpine flexor
                     self.Iapp[102] = 5. #30. #50. #neu # klass # pilocarpine extensor

        #### piecewise linear synapses
        self.g =  ( self.v - self.Erest)             # sum of excitatory synaptic input
        self.g = numpy.maximum( self.g, 0.)
        self.g = numpy.minimum( self.g, 50.)
        self.Sumgex = self.WE.dot(self.g)
        self.Sumgex = numpy.minimum( self.Sumgex, 80. )

        self.g =  ( self.v - self.Erest)             # sum of inhibitory synaptic input
        self.g = numpy.maximum( self.g, 0.)
        self.g = numpy.minimum( self.g, 50.)
        self.Sumgin = self.WI.dot(self.g) * (-1.)
        self.Sumgin = numpy.maximum( self.Sumgin, -80. )
        self.Sumg = self.Sumgex + self.Sumgin        # total sum

        ################
        ################
        ## Application of a simplified Hodgkin Huxley Differential equation
        ## Update the activations of the neurons
        ################
        for i in range(self.n):              # HH-Diff Equ.
            self.Iself[i] = 1.*(self.Erest - self.v[i])
            self.vn[i] = self.v[i] + (self.Iself[i] + self.Sumg[i] + self.Iapp[i])/(self.Cmem[i])  # exc, mV, ms

            if self.vn[i] < 0.: self.vn[i] = 0.
            if self.vn[152] > 25. : self.vn[152] = 25. #  clip 3c output
            if self.vn[157] > 30. : self.vn[157] = 30. #  clip 2coutput

            # The following units have to pass a NL HPF, input vn[i], output vn[i], thereby forming a bandpassfilter
            if i in [3, 23, 43, 63, 83, 103, 138]:  # 3-103 CPG, 138 swingTau
                if i == 138: leak_1 = self.LeakSwing[138]
                if i in [3, 23, 43, 63, 83, 103]:
                    leak_1 = 0.0001
                    if i == 43: leak_1 = 0.0003 # asymmetric levator
                    # specific leak values to produce appropriate periods for Run
                    if self.Run == 1:
                        if i == 43:  leak_1 = 0.0006   # lev
                        if i == 63:  leak_1 = 0.00011  # depr
                        if i == 3:   leak_1 = 0.0003   # protr SW-like
                        if i == 23:  leak_1 = 0.00014  # retr  ST-like
                        if i == 83:  leak_1 = 0.00009  # flex
                        if i == 103: leak_1 = 0.00005  # ext

                    if i == 138: leak_2 = 0.1 # for Swing[138]
                    if i == 3 or i == 23 or i == 43 or i == 63 or i == 83 or i == 103:
                        leak_2 = 0.1 #
                self.vn[i] =  self.applyBandPassFilter(i, leak_1, leak_2)
            if i == 145:  # for coordin rule 2i
                self.vn[i] =  self.applyBandPassFilter(i, 0.001, 0.1)
            if i == 160:  # for coordin rule 2i
                self.vn[i] = self.vn[i] =  self.applyBandPassFilter(i, 0.001, 0.1)
            if i == 166:  # for coordin rule 2c
                self.vn[i] = self.vn[i] =  self.applyBandPassFilter(i, 0.005, 0.1)
            if i == 149:  # for coordin rule 3i
                self.vn[i] = self.vn[i] =  self.applyBandPassFilter(i, 0.0001, 0.1)
         # end of nl HPF #

        for i in range(self.n):    # self.v[i] is required as input for the next iteration
            self.v[i] = self.vn[i]
        # end of Differential Equation

        ################
        ################
        # Apply specific experimental situations:
        ################
        ################
        # Running
        if self.Run == 1:  #
            #   version with hard mechanical limits, CPG is running, but output interrupted
                    if self.v[8] > 45.: self.v[1] = 0. # mechanical stop AEP, no influence to CPG
                    if self.v[8] < 10.: self.v[21] = 0. # mechanical stop PEP    # alpha
                    if self.v[48] < 20.: self.v[61] = 0. # limit Depr           # beta
                    if self.v[88] > 36.: self.v[81] = 0. # limit Flexor        # gamma


        ################
        # Pilocarpine application
        if WNParams.pilo[self.leg.name]  == 1 and not WNParams.intraleg[self.leg.name] == 1: # new version: the latter is now possible (intraleg input)   leg deafferented  L859   #######intraleg  Y3
            self.v[4], self.v[24] = 0., 0.  # = 0.   # inhibits sensory influences via ring net
            self.v[45], self.v[65] = 0., 0.
            self.v[84], self.v[104] = 0., 0.
            if self.count == 12:
                self.v[4] += 5.
                self.v[64] += 5.
                self.v[84] += 5.


#        if WNParams.pilointraleg[self.leg.name]  == 1: # leg afferented  L 870    #######intraleg  pilo
 #           self.v[328] = 0.
  #          if self.Iapp[170] > 0: self.v[328] = 40. # for Graphik pilo on TODO
        # lower and upper limits for self.v[i] units
        for i in range(self.n):
                if self.v[i] < 0.: self.v[i] = 0.
                if self.v[i] > 50.:  self.v[i] = 50.

        ################
        # Disturbance
        #
        # for testing various disturbances, prolongation of swing duration
        # disturbance of a specific leg during normal walking, swing
        self.disturb = 0.
        self.disturbduration = 4500.  # MR, for vel 30 Mirror pattern
        if  WNParams.disturb[self.leg.name] == 1.:            # choose leg in Settings
            if self.disturbduration > 0.:
                if self.disturbstart == 0 and self.count > 5000: #23000:   # example: right hind leg, vel 20
                    if self.forward == 1:
                        if self.v[122] > self.v[123] and self.v[8] > 25.:  # SW > ST
                            # first swing after self.count and alpha > 25 (i.e. swing beginn + ca 1000 ms)
                            self.countdisturbtime += 1
                            self.v[1] = 0.  # protractor stop
                            self.v[61] = 0. # depressor stop
                            self.disturb = 10.
                            # or: self.v[1] += HPF; self.v[41] += HPF # for levator reflex
                            if self.countdisturbtime > self.disturbduration: self.disturbstart = 1 # end of disturbance
                    if self.backward == 1:
                        if self.v[122] > self.v[123] and self.v[8] < 25.:  # SW > ST  # 1059
                            # first swing after self.count and alpha > 25 (i.e. swing beginn + ca 1000 ms)
                            self.countdisturbtime += 1
                            self.v[21] = 0.  # retractor stop
                            self.v[41] = 0. # levator stop
                            self.disturb = 10.
                            # or: self.v[1] += HPF; self.v[41] += HPF # for levator reflex
                            if self.countdisturbtime > self.disturbduration: self.disturbstart = 1 # end of disturbance
        # End of applying disturbance

        ###########################
        ###########################
        # Novel intraleg studies
        ###########################
        ###########################
        # Experiment:
        ###################
        # setting only FR
        # Stimulus: CampSens / ChordOrgan
        # leg fixed, therefore no movement from this leg
        if WNParams.intraleg[self.leg.name] == 1.:  # L 920
            self.v[1],self.v[21],self.v[41],self.v[61],self.v[81],self.v[101] = 0.,0.,0.,0.,0.,0. # output FR = 0

        # motor output
        self.fovel = WNParams.fovelstance[self.leg.name] * 1.7  #force velocity gain
        if self.v[122] > self.v[123]:   # SW > ST
            self.fovel = WNParams.fovelswingFW[self.leg.name] *0.72
            if self.backward == 1.:  self.fovel = WNParams.fovelswingBW[self.leg.name] *0.72

        ################
        # for "Standing legs in walking insect": legs walking on treadmill
        if (WNParams.frictW[self.leg.name]) == 1.:   # strong activation of premotor unit, due to friction, should not provid velocities above a given threshold
             if self.v[22] > 0.2*self.vel: self.v[21] = 0.2*self.vel

        ################
        # Application of noise in alpha
        # alpha
        outPro = self.v[1] - self.Erest + (random.random() - 0.5) * self.noisefct # noisefactor, currently set to zero
        if outPro < 0.: outPro = 0.   # MN velocity output
        outRet = self.v[21] - self.Erest + (random.random() - 0.5) * self.noisefct
        if outRet < 0.: outRet = 0.   # MN velocity output

        # beta
        outLev = self.v[41] - self.Erest
        if outLev < 0.: outLev = 0.   # MN velocity output
        outDep = self.v[61] - self.Erest
        if outDep < 0.: outDep = 0.   # MN velocity output

        # gamma
        outFle = self.v[81] - self.Erest
        if outFle < 0.: outFle = 0.   # MN velocity output
        outExt = self.v[101] - self.Erest
        if outExt < 0.: outExt = 0.   # MN velocity output

        # summation of velocity output over 10 iterations, to cope with Hectors time resolution
        self.alphaHvelout += self.orientation_factor * (outPro - outRet) * self.fovel   # alpha joint
        self.betaHvelout += -self.orientation_factor * self.hind_leg_fact * (outLev - outDep) * self.fovel*WNParams.SwingBetaFactor[self.leg.name] # beta joint
        self.gammaHvelout += -self.orientation_factor * self.hind_leg_fact * (outFle - outExt) * self.fovel # gamma joint,

        # Novel pilocarpine experiment : legs are deafferented, therefore motor output off
        if WNParams.pilo[self.leg.name]  == 1:
             self.alphaHvelout = 0. # CPG
             self.betaHvelout = 0. # CPG
             self.gammaHvelout = 0. # CPG

 ###       if WNParams.pilo[self.leg.name]  == 1:   # Borg  raus
    ###        self.v[123] = 50. # ST on AEP earlier than with pos
       ###     self.Iapp[2] = -5. # inhibits prtotractor

        # for "Standing legs in walking insect": motor output (velocity) = 0
        if WNParams.standL[self.leg.name]  == 1: # leg position fixed
             self.alphaHvelout = 0. # alpha joint
             self.betaHvelout = 0. # beta joint
             self.gammaHvelout = 0. # gamma joint
           # for "Standing legs in walking insect" retractor output shown only if above threshold,
           #  thr = 7. For Graphics to show strong output only.
           #  SaxA: 5.8 - 19.2, SaxB: 5.8 - 7.28, SaxC: 6.0 - 8.8, 19.5
             if self.v[22] < 7: self.v[21] = 0.


        # End of stance, End of Swing:  sens. feedback, PEP: here alpha position instead of load
        self.Iapp[123] = 0.   # leg loaded, activates v[123], Stance
        self.Iapp[170] = 0.   # leg loaded, activates v[123], Stance
        self.Iapp[171] = 0.   # leg unloaded, activates v[122], Swing


        if self.v[124] > self.v[125]:  # forward on
            #### ((1)) AEP position dependent SW->ST, strong limitation of swing
            if self.Iapp[8] >  WNParams.aep[self.leg.name]:  # AEP: Fl 40., Ml 43. Hl 38:
                 self.v[123] = 50. # switch to Stance

            #### ((2)) AEP, load dependent SW -> ST, strong limit of swing, even before position limit above (1)
            # load feedback for AEP and PEP
            self.Iapp[2] = 0.
            thr1 =  WNParams.loadthr[self.leg.name]   # threshold: Fl -1 Ml 0. Hl -2
            self.loadbeta=(self.betaMRe-self.betaMRh)*100.*WNParams.loadsign[self.leg.name]
            if (self.v[122] > self.v[123]) and (self.loadbeta < thr1): # if Swing on, but load on
                if self.v[61] > 0.1:    # depressor ground contact,
                    self.AEPloadCurve = WNParams.aepload[self.leg.name] # alpha position beyond which load can stop swing, for straight forward: Fl: 38., Ml 32., Hl 32.
                    # Adjustment for curve walking
                    if WNParams.curve_walking:
                        # Adjust load threshold for inner front leg of the example
                        if self.leg.name == "front_right_leg":
                            self.AEPloadCurve = 20.
                        # Adjust load threshold for outer hind leg of the example
                        if self.leg.name == "hind_right_leg":   # corr 6.Aug
                            self.AEPloadCurve = 20.
                    if self.v[8] > self.AEPloadCurve:
                        self.v[123] = 50.  # Stance on, AEP earlier than normal AEP
                        self.Iapp[2] = -5.  # inhibits protractor

            #### ((3))  PEP position dependent
            if self.v[123] > self.v[122]:  # Stance on,  limits Stance, but may be dominated by rule 1ab
                if (self.Iapp[8] < WNParams.pep[self.leg.name]): # if leg position < threshold: Fl 6., Ml 10., Hl 5.
                    self.Iapp[171] = 50. #    # 171 activates Swing, inhibits stance

                # Adjustment for curve walking: inner front leg position threshold:
                if WNParams.curve_walking and self.leg.name == "front_right_leg":
                    if (self.Iapp[8] < WNParams.pep[self.leg.name]) or (self.v[88] > 40.):
                        self.Iapp[171] = 50.   # 171 activates Swing, inhibits stance

            #### ((4)) PEP position + load dependent, ST -> SW ,decrease of load during rearward position, weakens Stance motivation
            if self.v[123] > self.v[122]:  # Stance on
                self.Iapp[170] = 10. # load signal
                if WNParams.intraleg[self.leg.name] == 1.:  self.Iapp[170] = 0.    #######intraleg, no load input during stance, L1017

                if self.Iapp[8] < WNParams.pep[self.leg.name] + 10.:  # if leg position < 10 + threshold: Fl 6., Ml 10., Hl 5.
                    self.Iapp[170] = self.Iapp[8] * 0.5 # load signal depends on leg (alpha) position

            #### ((5)) PEP:  if Swing on, but load still on: protraction off, avoids slipping at beginning of swing
            if (self.v[122] > self.v[123]) and (self.loadbeta < thr1): #Swing on, but load on
                     self.Iapp[2] = 0.
                     if self.v[41] > 0.1:  # levator on
                         self.Iapp[2] = -50.      # inhibits protractor, only lift leg at PEP

            if WNParams.pilo[self.leg.name] == 1.:  # L 1035  ####piloneu
                self.Iapp[2] = self.pilo2
            # pilocarpine: legs are deafferented, therefore: motor output off L1030
            if WNParams.pilo[self.leg.name]  == 1:
                self.alphaHvelout = 0. # CPG
                self.betaHvelout = 0. # CPG
                self.gammaHvelout = 0. # CPG

            #### to Fig 2 - 5: period 6000, stance: 4000, sw 2000

            #### to Fig 6:  period 10800, stance: 6800, sw 4000
            #### to Fig 6:  period 3600, stance: 1800, sw 1800
            ####
            #### to Fig  7: period 7000, stance: 4000, sw 3000
            #### to Figs 7: period 3600, stance: 1800, sw 1800
            ###
            #### gamma  period 3800, ext 1800,  flex 2000  sehr schnell  L1079

            if WNParams.intraleg[self.leg.name] == 1.:  # for vel = 30: period        FORWARD
                 self.start = 2. #
                 if self.count < self.start:
                     self.countStim = 0.
                     self.CS, self.ChOvel = 0.,0.
                     self.Iapp[170] = 0.
                     self.Iapp[171] = 50. # rechanged from 170 to 171
                 if self.count >= self.start:    # start Stimulus CS, ChO
                     self.countStim = self.count - self.start

                 self.period = 3600. #10800. # 7000. # 3600 # 20000 #  L1091

                 if self.period == 6000:
                     self.stim_duration = 4000.
                     self.mod1 = self.countStim%6000  ## period    Figs 2 - 5
                 if self.period == 20000:                  # test
                     self.stim_duration = 11000.           # test
                     self.mod1 = self.countStim%20000      # test
                 if self.period == 10800:              ##           Fig 6
                     self.stim_duration = 6800.        ## 4000  ##  Fig 6
                     self.mod1 = self.countStim%10800  ## period    Fig 6
                 if self.period == 7000:               ##           Fig 7
                     self.stim_duration = 4000.        ## 3000  ##  Fig 7
                     self.mod1 = self.countStim%7000   ## period    Fig 7
                 if self.period == 3600:               ##           Figs 6,7
                     self.stim_duration = 1800.        ## 1800      Figs 6,7
                     self.mod1 = self.countStim%3600   ## period    Figs 6,7
                 if self.mod1 > 0 and self.mod1 < self.stim_duration:  # stimulus on
                         if WNParams.Akay == 1:   # CS
                             self.CS = 50.
                             self.Iapp[170] = self.CS  # load on    ######  Iapp
                             self.Iapp[171] = 0. #  SW
                         if WNParams.HellHess == 1:  # ChO
                           #  self.ChOpos = self.mod1 *20./4000. + self.min # startpos 15 - 35 mV
                             self.ChOpos = 20. * (self.mod1/self.stim_duration) + self.min #
                             self.Iapp[88] = self.ChOpos
                             self.ChOvel = 50. # elong = flexion
                             self.Iapp[170] = self.ChOvel
                             self.Iapp[171] = 0.  #
                   #          if self.mod1 > 2400:       # AR2
                    #              self.ChOpos = (self.mod1 - 3.*(self.mod1)) *20./4000.+self.min+38. # AR2
                     #             self.Iapp[170] = 0.
                      #            self.Iapp[171] = 50.
                             ##output Hellekes 2, 22, 82, 102  output Hess:  42, 62
                             # graphicoutput f. Hellekes, Hess for illustration only ChOpos, ChOvel
                             # amplitude used here larger than normal step

                 if self.mod1 >= self.stim_duration:  #  # stimulus off
                     if WNParams.Akay == 1:
                          self.CS = 0.
                          self.Iapp[170] = self.CS # ST
                          self.Iapp[171] = 50.  # load off, + load on to 126    test unterschied mit/ohne   L1062
                     if WNParams.HellHess == 1:
                          self.Iapp[170] = 0. # ST
                          self.Iapp[171] = 50.
                          self.ChOvel = 0.
                        #  self.ChOpos = (self.mod1 - 3.*self.mod1) *20./4000.+ 60. + self.min
                          self.ChOpos = 20. - 20. *((self.mod1 - self.stim_duration)/(self.period - self.stim_duration)) + self.min #

        # Velocities is calculated after first iteration - otherwise 0
        if (self.alpha_vel != None):                                       ###
            self.alpha_vel = self.alpha - self.alpha_old                   ###
            self.beta_vel = self.beta - self.beta_old                      ###
            self.gamma_vel = self.gamma - self.gamma_old                   ###

            self.alphae_vel = self.alpha_e - self.alphae_old               ###
            self.betae_vel = self.beta_e - self.betae_old                  ###
            self.gammae_vel = self.gamma_e - self.gammae_old               ###
        else:                                                              ###
            self.alpha_vel, self.beta_vel, self.gamma_vel = 0., 0., 0.     ###
            self.alphae_vel, self.betae_vel, self.gammae_vel = 0., 0., 0.  ###

        if (self.alpha != None):                                           ### for velocity
            self.alpha_old , self.beta_old, self.gamma_old = self.alpha, self.beta, self.gamma  ### for velocity
            self.alphae_old , self.betae_old, self.gammae_old = self.alpha_e, self.beta_e, self.gamma_e  ### for velocity


        ###############
        # Recording of load (torque)
        # for details see Fig S2
        self.v[9] = (self.alphaMRe-self.alphaMRh)*200.*WNParams.loadsign[self.leg.name] # self.loadalpha
        self.v[29] = (-self.alphaMRe+self.alphaMRh)*200.*WNParams.loadsign[self.leg.name] # self.loadalpha
        self.v[49] = (self.betaMRe-self.betaMRh)*100.*WNParams.loadsign[self.leg.name] # self.loadbeta
        self.v[69] = (-self.betaMRe+self.betaMRh)*100.*WNParams.loadsign[self.leg.name] # self.loadbeta
        self.v[89] = (self.gammaMRe-self.gammaMRh)*200.*WNParams.loadsign[self.leg.name] # self.loadgamma
        self.v[109] = (-self.gammaMRe+self.gammaMRh)*200.*WNParams.loadsign[self.leg.name] # self.loadgamma
        if self.v[9] < 0.: self.v[9] = 0.
        if self.v[29] < 0.: self.v[29] = 0.
        if self.v[49] < 0.: self.v[49] = 0.
        if self.v[69] < 0.: self.v[69] = 0.
        if self.v[89] < 0.: self.v[89] = 0.
        if self.v[109] < 0.: self.v[109] = 0.

        # end of Swing, end of stance, backward walking
        if self.v[124] < self.v[125]:     # backward on
            if self.Iapp[8] > WNParams.aep[self.leg.name]:  # if leg (aplpha) position > threshold: Fl 40., Ml 43., Hl 38.
                  self.Iapp[171] = 50.   # swing on
            if self.Iapp[8] < WNParams.pep[self.leg.name]:  # if leg (alpha) position < threshold: Fl 6., Ml 10., Hl 5.
                  self.Iapp[123] = 50.   # stance on

            #### to Fig 2 - 5 period 6000, stance: 4000

            if WNParams.intraleg[self.leg.name] == 1.:  # for vel = 30: period = 8.0 s    BACKWARD
                 self.start = 2. #
                 if self.count < self.start:
                     self.countStim = 0.
                     self.CS, self.ChOvel = 0.,0.
                     self.Iapp[170] = 0.
                     self.Iapp[171] = 50. #
                 if self.count >= self.start:    # start Stimulus CS, ChO
                     self.countStim = self.count - self.start
                 self.mod1 = self.countStim%6000 #    # period  Fig 2,4,5,6
                 #self.mod1 = self.countStim%10800 #  Figs 3, 7
                 if self.mod1 > 0 and self.mod1 < 4000:  # stimulus on  Fig 2,4,5,6
                 #if self.mod1 > 0 and self.mod1 < 5800:  Figs 3, 7
                     if WNParams.Akay == 1:   # CS
                         self.CS = 50.
                         self.Iapp[170] = self.CS  # load on    ######  Iapp
                         self.Iapp[171] = 0. #  SW
                     if WNParams.HellHess == 1:  # ChO
                             self.ChOpos = self.mod1 *20./4000. + self.min # startpos 15 - 35 mV
                             self.Iapp[88] = self.ChOpos
                             # output 42 62 HessBü LevDepr beta, not dependent on walking, also Stand
                             self.ChOvel = 50. # elong = flexion
                             self.Iapp[170] = self.ChOvel
                             self.Iapp[171] = 0.  #
                        #     if self.mod1 > 2400:       # AR2
                         #         self.ChOpos = (self.mod1 - 3.*(self.mod1)) *20./4000.+self.min+38. # AR2
                          #        self.Iapp[170] = 0.
                           #       self.Iapp[171] = 50.
                             ##output Hellekes 2, 22, 82, 102  output Hess:  42, 62
                             # graphicoutput f. Hellekes, Hess for illustration only ChOpos, ChOvel
                             # amplitude used here larger than normal step

                 if self.mod1 >= 4000:  #  # stimulus off Fig 2,4,5,6
                 #if self.mod1 >= 5800:  # Figs 3, 7
                     if WNParams.Akay == 1:
                         self.CS = 0.
                         self.Iapp[170] = self.CS # ST
                         self.Iapp[171] = 50.  # load off, + load on to 126    test unterschied mit/ohne   L1062
                     if WNParams.HellHess == 1:
                         self.Iapp[170] = 0. # ST
                         self.Iapp[171] = 50.
                         self.ChOvel = 0.
                         self.ChOpos = (self.mod1 - 3.*self.mod1) *20./4000. + 60. + self.min

        ###############
        self.v[140], self.v[141] = 0.,0.

        if (self.count%1000) == 0:
             if (self.name == "controller_HR") :   # 150
                 print("    10     SW     ST    171  170    2   22    42   62    82  102  ChOpos ") #for walk

    ## Print leg controller variables
    def output_current_state(self):
        mod = self.count%200 # 200
        if mod == 0:
            print(self.name[11:], "%4i %7.1f %5.1f %5.1f %4.1f %5.1f %4.1f %5.1f %4.1f %5.1f %4.1f %5.1f" %(self.count,self.v[122],self.v[123],
                                                                                                                   self.v[171],self.v[170],self.v[2],self.v[22],self.v[42],self.v[62],self.v[82],self.v[102],self.ChOpos)) #
