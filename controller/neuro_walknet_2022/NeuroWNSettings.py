'''
Settings for the neuroWalknet.

In some cases these are encoded values for the neural network
- for these activations are in the range of 0 to 50 and values are scaled in this range.
'''

###########################
# General Walking Parameters
###########################
walking_modes = {"forward" : 1, "backward" : 2, "running" : 3}
w_mode = walking_modes["forward"]
velocity = 30. # 45., curve

###########################
# Coordination Rule Strengths
###########################
coord_rules = []
# Rule 1 ipsilateral: R1a and R1b
# Rule 2 - ipsilateral: R2i, contralateral front-middle-hind: R2cf, R2cm, R2ch
# Rule 3 - ipsilateral: R3i, contralateral front-middle-hind: R3cf, R3cm, R3ch
# Rule 5 - ipsilateral: R5i, contralateral - contralateral hind legs: R5c, R5ch
if (w_mode == walking_modes["forward"]):
    coord_rules = {
        "R1a" : 1., "R1b" : 1.,
        "R2i" : 3., "R2cf" : 1., "R2cm" : 1., "R2ch" : 3.,
        "R3i" : 1., "R3cf" : 1., "R3cm" : 0., "R3ch" : 3.,
        "R5i" : 0.2, "R5c" : 0.1, "R5ch" : 0.1,
        "R5iD" : 0.2, "R5cD" : 0.1, "R5chD" : 0.1,
        "R5Pi" : 0., "R5Pc" : 0.}
elif (w_mode == walking_modes["running"]):
    coord_rules = {
        "R1a" : 0., "R1b" : 0.,
        "R2i" : 0., "R2cf" : 0., "R2cm" : 0., "R2ch" : 0.,
        "R3i" : 0., "R3cf" : 0., "R3cm" : 0., "R3ch" : 0.,
        "R5i" : 0., "R5c" : 0., "R5ch" : 0.,
        "R5iD" : 0., "R5cD" : 0., "R5chD" : 0.,
        "R5Pi" : 0.4, "R5Pc" : 0.3}
elif (w_mode == walking_modes["backward"]):
    coord_rules = {
        "R1a" : 1., "R1b" : 1.,
        "R2i" : 3., "R2cf" : 1., "R2cm" : 1., "R2ch" : 3.,
        "R3i" : 0., "R3cf" : 0., "R3cm" : 0., "R3ch" : 0.,
        "R5i" : 0., "R5c" : 0., "R5ch" : 0.,
        "R5iD" : 0., "R5cD" : 0., "R5chD" : 0.,
        "R5Pi" : 0., "R5Pc" : 0.}

###########################
# Joint configurations
###########################
# The dictionary defines an offset in alpha (as an angle in rad).
alpha_offset = {
    "front_left_leg": -0.5,
    "front_right_leg": 0.5,
    "middle_left_leg": 0.,
    "middle_right_leg": 0.,
    "hind_left_leg": 0.,
    "hind_right_leg": 0.
}

###########################
# Definition of Extreme positions of the legs
###########################
# The dictionary defines the Anterior Extreme Position (AEP) as encoded in the neurons.
aep = {
    "front_left_leg":  40.,
    "front_right_leg": 40.,
    "middle_left_leg":  43.,
    "middle_right_leg": 43.,
    "hind_left_leg":  38.,
    "hind_right_leg": 38.
}

# The dictionary defines the Posterior Extreme Position (PEP) as encoded in the neurons.
pep = {
    "front_left_leg": 6.,
    "front_right_leg": 6.,
    "middle_left_leg": 10.,
    "middle_right_leg": 10.,
    "hind_left_leg": 5.,
    "hind_right_leg": 5.
}

# Sets gamma_set_points for AEP
gammamorphAEP = {
    "front_left_leg": 30.,
    "front_right_leg": 30.,
    "middle_left_leg": 40.,
    "middle_right_leg": 40.,
    "hind_left_leg": 45.,
    "hind_right_leg": 45.
}
# Sets gamma_set_points for PEP in forward walking
gammamorphPEPFW = {
    "front_left_leg":  40.,
    "front_right_leg": 40.,
    "middle_left_leg":  40.,
    "middle_right_leg": 40.,
    "hind_left_leg":  45.,
    "hind_right_leg": 45.
}
# Sets gamma_set_points for the PEP in backward walking
gammamorphPEPBW = {
    "front_left_leg":  40.,
    "front_right_leg": 40.,
    "middle_left_leg":  40.,
    "middle_right_leg": 40.,
    "hind_left_leg":  30.,
    "hind_right_leg": 30.
}

# Parameter that determines alpha angle set point in degrees (rad)
# in body coordinates
offset = {
    "front_left_leg": 0.5,
    "front_right_leg": 0.5,
    "middle_left_leg": 0.5,
    "middle_right_leg": 0.5,
    "hind_left_leg": 0.75,
    "hind_right_leg": 0.75
}

###########################
# Parameters for swing movements
###########################
# SwingSetpoint: set point for beta feedback controller during swing
SwingSetpoint = {
    "front_left_leg": 10.,
    "front_right_leg": 10.,
    "middle_left_leg": 10.,
    "middle_right_leg": 10.,
    "hind_left_leg": 10.,
    "hind_right_leg" : 10.
}

# SwingTau: High-pass filter time constant to control duration of swing movement
SwingTau = {
    "front_left_leg":   0.0018*0.5,
    "front_right_leg":  0.0018*0.5,
    "middle_left_leg":  0.0015*0.45,
    "middle_right_leg": 0.0015*0.45,
    "hind_left_leg":    0.0015*0.44,
    "hind_right_leg" :  0.0015*0.44
}

# Swingweight:   factor that controls amplitude of swing movement via beta joint
SwingWeight = {
    "front_left_leg":  12.,
    "front_right_leg": 12.,
    "middle_left_leg":  8.,
    "middle_right_leg": 8.,
    "hind_left_leg":    10.,
    "hind_right_leg" :  10.,
}

# SwingBetaFactor: factor for correction of beta velocity during swing
SwingBetaFactor = {
    "front_left_leg":  1.8,
    "front_right_leg": 1.8,
    "middle_left_leg":  1.4,
    "middle_right_leg": 1.4,
    "hind_left_leg":    3.,
    "hind_right_leg" :  3.,
}

# fovelswingFW:  factor for velocity output during swing, forward walk
fovelswingFW = {
     "front_left_leg":  0.00176   *0.85*0.9*0.98,
    "front_right_leg":  0.00176   *0.85*0.9*0.98,
    "middle_left_leg":  0.00176,
    "middle_right_leg": 0.00176,
    "hind_left_leg":    0.00176   *0.7,
    "hind_right_leg" :  0.00176   *0.7
}

# fovelswingBW:  factor for velocity output during swing, backward walk
fovelswingBW = {
     "front_left_leg":  0.00176   *0.85*0.9*0.98 *1.15,
    "front_right_leg":  0.00176   *0.85*0.9*0.98 *1.15,
    "middle_left_leg":  0.00176                  *0.75,
    "middle_right_leg": 0.00176                  *0.75,
    "hind_left_leg":    0.00176   *0.7           *1.23,
    "hind_right_leg" :  0.00176   *0.7           *1.23
}

###########################
# Parameters for Stance movements
###########################

# fovelstance:  factor for velocity output during stance
fovelstance = {
    "front_left_leg":   0.00144   *0.93,
    "front_right_leg":  0.00144   *0.93,
    "middle_left_leg":  0.00144   *0.9,
    "middle_right_leg": 0.00144   *0.9,
    "hind_left_leg":    0.00144   *0.82 * 1.,
    "hind_right_leg" :  0.00144   *0.82 * 1.
}

# for Ring net   Sollte nach vorne zu Parameters for Stance movement
# see Sections Methods, B) Leg controller
# shiftHl: shift of zero position by 15 degrees (for hind legs),
#  which means shift by 1 neuron.
shiftHl = {
    "front_left_leg":  0,
    "front_right_leg": 0,
    "middle_left_leg": 0,
    "middle_right_leg":0,
    "hind_left_leg":   1,
    "hind_right_leg" : 1.
}

###########################
# Parameters controlling end of swing via load
###########################
# describes sign of motor movement to determine force
# (difference between velocity setpoint and current velocity)
loadsign = {
    "front_left_leg":  1.,
    "front_right_leg": -1.,
    "middle_left_leg":  1.,
    "middle_right_leg": -1.,
    "hind_left_leg":    -1.,
    "hind_right_leg" :  1.
}

# loadthr: threshold for load to determine end of swing
loadthr = {
    "front_left_leg":  -1.,
    "front_right_leg": -1.,
    "middle_left_leg":  0.,
    "middle_right_leg": 0.,
    "hind_left_leg":    -2.,
    "hind_right_leg" :  -2.
}

# determines threshold beyond which load controls end of swing
aepload = {
    "front_left_leg":  38.,
    "front_right_leg": 38.,
    "middle_left_leg": 32.,
    "middle_right_leg":32.,
    "hind_left_leg":   32.,
    "hind_right_leg" : 32.
}

###########################
###########################
# Specific experimental situations
# 2020 Paper, PLoS Computational Biology
###########################
###########################

###########################
# Experiments shown in section Results (2020 Paper):
# All legs deafferented (Kn - biological experiment by Knebel) and
# intact walking legs and deafferented legs (BG - Borgmann)
# pilo:  factor that determines a leg to be treated with pilocarpine
#   (plus deafferented) or not
# In comments, the values are shown for different experimental situations.
#   free walking leg: 0, pilo + deafferentedCPG: 1
pilo = {
    "front_left_leg":  0., # any_Kn: 1. #Bg: 0.  #free walking leg: 0, CPG: 1
    "front_right_leg": 0., #         1.      1.
    "middle_left_leg": 0., #         1.      1.
    "middle_right_leg":0., #         1.      1.
    "hind_left_leg":   0., #         1.      1.
    "hind_right_leg" : 0.  #         1.      1.
}

# Experiments shown in section Results (2020 Paper): All legs deafferented.
# Deaffonly: a factor that determines a leg that is only deafferented
# but not treated with pilocarpine
# weights for Deaffonly (1: deaff but no pilo, above Deaff+Pilo), shuts input to v174/304 (5c)
# In comments, the values are shown for different experimental situations.
#   free walking leg: 0, CPG: 1
Deaffonly = {
    "front_left_leg":  0., # KnPro: 0. #KnMes: 1. KnMet: 1. Kneall: 0.  Bo07: 0. #free walking leg: 0, CPG: 1
    "front_right_leg": 0., #        0.         1.        1.         0.        1.
    "middle_left_leg": 0., #        1.         0.        1.         0.        1.
    "middle_right_leg":0., #        1.         0.        1.         0.        1.
    "hind_left_leg":   0., #        1.         1.        0.         0.        1.
    "hind_right_leg" : 0.  #        1.         1.        0.         0.        1.
}

###########################
# Experiments shown in sections Results (2020 paper): Intact walking leg and deafferented legs, (Bg)
# and in Results: Standing legs in walking insects (Sax - Saxler)
#
# frictW: a factor that determines friction of a leg walking on a treadmill
# In comments, the values are shown for different experimental situations.
# Bg, Sax,  default: 0. Only three cases of Sax are shown in the paper
#       free leg: 0, FrictLeg(=wheelwalk): 1.
frictW = {
    "front_left_leg":  0., #  Bg: 1. #Saxa: 1. Saxb: 1.  Saxc:  0.
    "front_right_leg": 0., #      0.        1.       0.         0.
    "middle_left_leg": 0., #      0.        1.       1.         1.
    "middle_right_leg":0., #      0.        1.       0.         1.
    "hind_left_leg":   0., #      0.        0.       1.         1.
    "hind_right_leg" : 0.  #      0.        0.       0.         1.
}

# standL: a factor that determines a leg standing on a fixed force transducer
# In comments, the values are shown for different experimental situations.
#weights for standLeg, Sax  default: 0.
#       free walking leg: 0, standL: 1.
standL = {
    "front_left_leg":  0.,        # Saxa:  0. Saxb:  0. Saxc:  1.
    "front_right_leg": 0.,        #        0.        1.        1.
    "middle_left_leg": 0.,        #        0.        0.        0.
    "middle_right_leg":0.,        #        0.        1.        0.
    "hind_left_leg":   0.,        #        1.        0.        0.
    "hind_right_leg" : 0.         #        1.        1.        0.
}

# standLalpha: alpha position of a leg standing on a fixed force transducer
standLalpha = {
    "front_left_leg":  31.,
    "front_right_leg": 31.,
    "middle_left_leg": 31.,
    "middle_right_leg":31.,
    "hind_left_leg":   31.,
    "hind_right_leg" : 31.
}

# standLbeta: beta position of a leg standing on a fixed force transducer
standLbeta = {
    "front_left_leg":  26.,
    "front_right_leg": 26.,
    "middle_left_leg": 29.,
    "middle_right_leg":29.,
    "hind_left_leg":   29.,
    "hind_right_leg" : 29.
}

# standLgamma: gamma position of a leg standing on a fixed force transducer
standLgamma = {
    "front_left_leg":  31.,
    "front_right_leg": 31.,
    "middle_left_leg": 34.,
    "middle_right_leg":34.,
    "hind_left_leg":   33.,
    "hind_right_leg" : 33.
}

###########################
# Testing variation of parameters, here frequency of CPGs
###########################
# CPGfrequ:  parameter that allows changing of CPG frequencies for testing
CPGfrequ = {
    "front_left_leg":  1.,
    "front_right_leg": 1.,
    "middle_left_leg": 1.,  # .8
    "middle_right_leg":1.,  # .9
    "hind_left_leg":   1.,
    "hind_right_leg" : 1.
}

###########################
# Simple version of "curve" walking
# Not addressed in paper! - different from detailed curve walking there
#
# SplitBelt:  factor for different velocities for right legs and left legs.
# Parameter = 1 allows for velocity factors different from 1.
# Currently not applied in controller.
SplitBelt = {
    "front_left_leg":  1., #
    "front_right_leg": 0., #
    "middle_left_leg": 1., #
    "middle_right_leg":0., #
    "hind_left_leg":   1., #
    "hind_right_leg" : 0., #
}


# Tested (only mentioned in the paper)
# Testing stability of walking pattern by disturbances.
# disturb: number of leg that may receive a disturbance
# (prolongation of swing duration)
# In comments, the values are shown for different experimental situations.
disturb = {
    "front_left_leg":   0., # select disturbed leg   0.
    "front_right_leg":  0., #   e.g front right leg  1.
    "middle_left_leg":  0., #                        0.
    "middle_right_leg": 0., #                        0.
    "hind_left_leg":    0., #                        0.
    "hind_right_leg" :  0.  #                        0.
}

###########################
###########################
# Novel Specific experimental situations
# Intraleg studies, pilo
###########################
###########################

# Turning on and off experimental situations from Akay et al. and Hellekes
Akay, HellHess = 0.,0.  #L86   ###### intraleg, no curve see L 280, rule5 off

###########################
# Curve Walking - see Section Supplement, C) Negotiation of Curves
curve_walking = False


#Test AkayFig1E
CSrev1 = {
    "front_left_leg":   0., # select disturbed leg   0.
    "front_right_leg":  0., #   e.g front right leg  1.
    "middle_left_leg":  0., #                        0.
    "middle_right_leg": 0., #                        0.
    "hind_left_leg":    0., #                        0.
    "hind_right_leg" :  0.  #                        0.
}

# Selection of Stimulus for Akay CS, Helekes ChO.
intraleg = {
    "front_left_leg":   0., # select stimulated leg   0.
    "front_right_leg":  1., #   e.g front right leg  1. # for Akay, HellHess, not for free curve walk
    "middle_left_leg":  0., #                        0.
    "middle_right_leg": 0., #                        0.
    "hind_left_leg":    0., #                        0.
    "hind_right_leg" :  0.  #                        0.
}
