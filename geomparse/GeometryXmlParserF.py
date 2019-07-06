#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import numpy
## Warning: This is currently only a mockup function!
# It does not what it promises (parsing an xml-file). Instead, it just returns a dictionary with some values regarding the lengths of the leg segments etc.
# @return dictionary
def parseHectorXml(xml):
	dic={}
	leg_names=['front_left_leg', 'front_right_leg', 'middle_left_leg', 'middle_right_leg', 'hind_left_leg', 'hind_right_leg']
	phis=[angleInDegree/180*math.pi for angleInDegree in (80, -80, 90, -90, 115, -115) ]
	psis=[-30/180*math.pi]*6
	chis=[angleInDegree/180*math.pi for angleInDegree in (11.5, -11.5, 0, 0, -28.3, 28.3) ]
	leg_onsets=[numpy.array([0.198+0.020+0.330+0.030,0.070,0]),numpy.array([0.198+0.020+0.330+0.030,-0.070,0]), numpy.array([0.198+0.020,0.070,0]),numpy.array([0.198+0.020,-0.070,0]), numpy.array([0.,0.070,0.]), numpy.array([0.,-0.070,0.])]
	beta_directions=[-1,1,-1,1,1,-1]
	for leg_name, legId in zip(leg_names, range(6)):
		dic[leg_name]={};
		dic[leg_name]['bfb_client_ids']=[i+16*legId for i in (17,18,19,20) ]
		dic[leg_name]['segment_lengths']=[0.03,0.260,0.300]
		dic[leg_name]['segment_masses']=[0.4,0.05,0.45]
		dic[leg_name]['segment_centers_of_mass']=[numpy.array([0.0, 0., 0.]),
													numpy.array([ 0.13-dic[leg_name]['segment_lengths'][1],  0.,  beta_directions[legId]*0.04117]),
													numpy.array([-dic[leg_name]['segment_lengths'][2]+0.005,  -0.03, 0])]
		dic[leg_name]['phi']=phis[legId]
		dic[leg_name]['psi']=psis[legId]
		dic[leg_name]['chi']=chis[legId]
		dic[leg_name]['relative_onset']=leg_onsets[legId]
	return dic
