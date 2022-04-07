# -*- coding: utf-8 -*-

import numpy
import controller.neuro_walknet_2022.NeuroWNSettings as WNParams

class NeuroCoordinationRules:
    """
    Coordination of the leg controllers.
    Acts on PEP and on load signals.

    ...

    Attributes
    ----------
    contr :
        neuroWalknet controller object

    Methods
    -------
    processing_step(timeStamp)
        Called automatically from the simulation loop as a step of the controller.
    """

    def __init__(self, contr):
        self.controller_objs = contr
        print("Initialized coordination rule processor")

    def update_coordination_rules(self, timeStamp):
        """
        Update coordination rules

        Is called from the main control loop at the end.
        After each leg controller is executed, the coordination rules are applied
        on top of these: the coordination rules connect different legs and act between
        these (on a local level)

        Parameters
        ----------
        timestamp : float
            Current timestep.
        """
        # Update the current activation values of neurons in the different legs
        # = simply overwrite these externally
        # weights for interleg coordination, e.g. rule 1: vML 122 (Swing) to vFL 123(Stance)
        # Leg_nr: FL = 0, FR = 1, ML = 2, MR = 3, HL = 4, HR = 5
        # Coordination from Middle Left Leg - to Front Left Leg
        if (self.controller_objs[2] and self.controller_objs[0]):    # FL 0, ML 2
            self.controller_objs[0].v[140] += self.controller_objs[2].v[158]*WNParams.coord_rules["R1a"] # rule 1a
            self.controller_objs[0].v[140] += self.controller_objs[2].v[146]*WNParams.coord_rules["R1b"] # rule 1b
            self.controller_objs[0].v[141] += self.controller_objs[2].v[160]*WNParams.coord_rules["R2i"] # rule 2i
            self.controller_objs[2].v[141] += self.controller_objs[0].v[149]*WNParams.coord_rules["R3i"] # rule 3i

            self.controller_objs[0].v[173] += self.controller_objs[2].v[172]*WNParams.coord_rules["R5i"] # rule 5i
            self.controller_objs[2].v[173] += self.controller_objs[0].v[172]*WNParams.coord_rules["R5i"] # rule 5i
            self.controller_objs[0].v[303] += self.controller_objs[2].v[302]*WNParams.coord_rules["R5iD"] # rule 5i   Dep
            self.controller_objs[2].v[303] += self.controller_objs[0].v[302]*WNParams.coord_rules["R5iD"] # rule 5i   Dep
            self.controller_objs[0].v[310] += self.controller_objs[2].v[301]*WNParams.coord_rules["R5Pi"] # rule 5Pi   Lev
            self.controller_objs[2].v[310] += self.controller_objs[0].v[301]*WNParams.coord_rules["R5Pi"] # rule 5Pi   Lev

        # Coordination from Hind Left Leg - to Middle Left Leg
        if (self.controller_objs[4] and self.controller_objs[2]):    # ML 2, HL 4
            self.controller_objs[2].v[140] += self.controller_objs[4].v[158]*WNParams.coord_rules["R1a"] # rule 1a
            self.controller_objs[2].v[140] += self.controller_objs[4].v[146]*WNParams.coord_rules["R1b"] # rule 1b
            self.controller_objs[2].v[141] += self.controller_objs[4].v[160]*WNParams.coord_rules["R2i"] # rule 2i
            self.controller_objs[4].v[141] += self.controller_objs[2].v[149]*WNParams.coord_rules["R3i"] # rule 3i
            self.controller_objs[2].v[173] += self.controller_objs[4].v[172]*WNParams.coord_rules["R5i"] # rule 5i
            self.controller_objs[4].v[173] += self.controller_objs[2].v[172]*WNParams.coord_rules["R5i"] # rule 5i
            self.controller_objs[2].v[303] += self.controller_objs[4].v[302]*WNParams.coord_rules["R5iD"] # rule 5i  Dep
            self.controller_objs[4].v[303] += self.controller_objs[2].v[302]*WNParams.coord_rules["R5iD"] # rule 5i  Dep
            self.controller_objs[2].v[310] += self.controller_objs[4].v[301]*WNParams.coord_rules["R5Pi"] # rule 5Pi  Lev
            self.controller_objs[4].v[310] += self.controller_objs[2].v[301]*WNParams.coord_rules["R5Pi"] # rule 5Pi  Lev

        # Coordination from Middle Right Leg - to Front Right Leg
        if (self.controller_objs[3] and self.controller_objs[1]):    # FR 1, MR 3
            self.controller_objs[1].v[140] += self.controller_objs[3].v[158]*WNParams.coord_rules["R1a"] # rule 1a
            self.controller_objs[1].v[140] += self.controller_objs[3].v[146]*WNParams.coord_rules["R1b"] # rule 1b
            self.controller_objs[1].v[141] += self.controller_objs[3].v[160]*WNParams.coord_rules["R2i"] # rule 2i
            self.controller_objs[3].v[141] += self.controller_objs[1].v[149]*WNParams.coord_rules["R3i"] # rule 3i
            self.controller_objs[1].v[173] += self.controller_objs[3].v[172]*WNParams.coord_rules["R5i"] # rule 5i
            self.controller_objs[3].v[173] += self.controller_objs[1].v[172]*WNParams.coord_rules["R5i"] # rule 5i
            self.controller_objs[1].v[303] += self.controller_objs[3].v[302]*WNParams.coord_rules["R5iD"] # rule 5i   Dep
            self.controller_objs[3].v[303] += self.controller_objs[1].v[302]*WNParams.coord_rules["R5iD"] # rule 5i   Dep
            self.controller_objs[1].v[310] += self.controller_objs[3].v[301]*WNParams.coord_rules["R5Pi"] # rule 5Pi   Lev
            self.controller_objs[3].v[310] += self.controller_objs[1].v[301]*WNParams.coord_rules["R5Pi"] # rule 5Pi   Lev

        # Coordination from Hind Right Leg - to Middle Right Leg
        if (self.controller_objs[5] and self.controller_objs[3]):    # MR 3 , HR 5
            self.controller_objs[3].v[140] += self.controller_objs[5].v[158]*WNParams.coord_rules["R1a"] # rule 1a
            self.controller_objs[3].v[140] += self.controller_objs[5].v[146]*WNParams.coord_rules["R1b"] # rule 1b
            self.controller_objs[3].v[141] += self.controller_objs[5].v[160]*WNParams.coord_rules["R2i"] # rule 2i
            self.controller_objs[5].v[141] += self.controller_objs[3].v[149]*WNParams.coord_rules["R3i"] # rule 3i
            self.controller_objs[3].v[173] += self.controller_objs[5].v[172]*WNParams.coord_rules["R5i"] # rule 5i
            self.controller_objs[5].v[173] += self.controller_objs[3].v[172]*WNParams.coord_rules["R5i"] # rule 5i
            self.controller_objs[3].v[303] += self.controller_objs[5].v[302]*WNParams.coord_rules["R5iD"] # rule 5i   Dep
            self.controller_objs[5].v[303] += self.controller_objs[3].v[302]*WNParams.coord_rules["R5iD"] # rule 5i   Dep
            self.controller_objs[3].v[310] += self.controller_objs[5].v[301]*WNParams.coord_rules["R5Pi"] # rule 5Pi   Lev
            self.controller_objs[5].v[310] += self.controller_objs[3].v[301]*WNParams.coord_rules["R5Pi"] # rule 5Pi   Lev

        # Contralateral Coordination between Front Right Leg - and Front Left Leg
        if (self.controller_objs[1] and self.controller_objs[0]):    # FL 0 , FR 1
            self.controller_objs[0].v[141] += self.controller_objs[1].v[157]*WNParams.coord_rules["R2cf"] # rule 2c
            self.controller_objs[0].v[141] += self.controller_objs[1].v[152]*WNParams.coord_rules["R3cf"] # rule 3c
            self.controller_objs[1].v[141] += self.controller_objs[0].v[157]*WNParams.coord_rules["R2cf"] # rule 2c
            self.controller_objs[1].v[141] += self.controller_objs[0].v[152]*WNParams.coord_rules["R3cf"] # rule 3c

            self.controller_objs[0].v[174] += self.controller_objs[1].v[172]*WNParams.coord_rules["R5c"]# rule 5c
            self.controller_objs[1].v[174] += self.controller_objs[0].v[172]*WNParams.coord_rules["R5c"]# rule 5c
            self.controller_objs[0].v[304] += self.controller_objs[1].v[302]*WNParams.coord_rules["R5cD"]# rule 5c   Dep
            self.controller_objs[1].v[304] += self.controller_objs[0].v[302]*WNParams.coord_rules["R5cD"]# rule 5c   Dep
            self.controller_objs[0].v[311] += self.controller_objs[1].v[301]*WNParams.coord_rules["R5Pc"]# rule 5Pc   Lev
            self.controller_objs[1].v[311] += self.controller_objs[0].v[301]*WNParams.coord_rules["R5Pc"]# rule 5Pc   Lev

        # Contralateral Coordination between Middle Left Leg - and Middle Right Leg
        if (self.controller_objs[2] and self.controller_objs[3]):    # ML 2 , MR 3
            self.controller_objs[2].v[141] += self.controller_objs[3].v[157]*WNParams.coord_rules["R2cm"] # rule 2c
            self.controller_objs[3].v[141] += self.controller_objs[2].v[157]*WNParams.coord_rules["R2cm"] # rule 2c
            self.controller_objs[2].v[174] += self.controller_objs[3].v[172]*WNParams.coord_rules["R5c"] # rule 5c
            self.controller_objs[3].v[174] += self.controller_objs[2].v[172]*WNParams.coord_rules["R5c"] # rule 5c
            self.controller_objs[2].v[304] += self.controller_objs[3].v[302]*WNParams.coord_rules["R5cD"] # rule 5c   Dep
            self.controller_objs[3].v[304] += self.controller_objs[2].v[302]*WNParams.coord_rules["R5cD"] # rule 5c   Dep
            self.controller_objs[2].v[311] += self.controller_objs[3].v[301]*WNParams.coord_rules["R5Pc"] # rule 5Pc   Lev
            self.controller_objs[3].v[311] += self.controller_objs[2].v[301]*WNParams.coord_rules["R5Pc"] # rule 5Pc   Lev

        # Contralateral Coordination between Hind Right Leg - and Hind Left Leg
        if (self.controller_objs[4] and self.controller_objs[5]):    # HL 4 , HR 5
            self.controller_objs[4].v[141] += self.controller_objs[5].v[157]*WNParams.coord_rules["R2ch"] # rule 2c
            self.controller_objs[4].v[141] += self.controller_objs[5].v[152]*WNParams.coord_rules["R3ch"] # rule 3c
            self.controller_objs[5].v[141] += self.controller_objs[4].v[157]*WNParams.coord_rules["R2ch"] # rule 2c
            self.controller_objs[5].v[141] += self.controller_objs[4].v[152]*WNParams.coord_rules["R3ch"] # rule 3c

            self.controller_objs[4].v[175] += self.controller_objs[5].v[172]*WNParams.coord_rules["R5ch"] # rule 5ch
            self.controller_objs[5].v[175] += self.controller_objs[4].v[172]*WNParams.coord_rules["R5ch"] # rule 5ch
            self.controller_objs[4].v[305] += self.controller_objs[5].v[302]*WNParams.coord_rules["R5chD"] # rule 5ch   Dep
            self.controller_objs[5].v[305] += self.controller_objs[4].v[302]*WNParams.coord_rules["R5chD"] # rule 5ch   Dep
            self.controller_objs[4].v[311] += self.controller_objs[5].v[301]*WNParams.coord_rules["R5Pc"] # rule 5Pc   Lev
            self.controller_objs[5].v[311] += self.controller_objs[4].v[301]*WNParams.coord_rules["R5Pc"] # rule 5Pc   Lev

        # Threshold activation of influenced neurons.
        if self.controller_objs[0].v[173] > 50.: self.controller_objs[0].v[173] = 50.
        if self.controller_objs[1].v[173] > 50.: self.controller_objs[1].v[173] = 50.
        if self.controller_objs[2].v[173] > 50.: self.controller_objs[2].v[173] = 50.
        if self.controller_objs[3].v[173] > 50.: self.controller_objs[3].v[173] = 50.
        if self.controller_objs[4].v[173] > 50.: self.controller_objs[4].v[173] = 50.
        if self.controller_objs[5].v[173] > 50.: self.controller_objs[5].v[173] = 50.
        if self.controller_objs[0].v[175] > 50.: self.controller_objs[0].v[175] = 50.
        if self.controller_objs[1].v[175] > 50.: self.controller_objs[1].v[175] = 50.
        if self.controller_objs[2].v[175] > 50.: self.controller_objs[2].v[175] = 50.
        if self.controller_objs[3].v[175] > 50.: self.controller_objs[3].v[175] = 50.
        if self.controller_objs[4].v[175] > 50.: self.controller_objs[4].v[175] = 50.
        if self.controller_objs[5].v[175] > 50.: self.controller_objs[5].v[175] = 50.

        #print("Updated coordination rules" )
