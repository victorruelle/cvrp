# -*- coding: utf-8 -*-
"""
Created on Mon Jun 25 15:58:34 2018

@author: GVKD1542
"""

''' GENERAL ''' 
from globals import allow_single_node_roads

def set_bounds(instance, i, j):
	#creates the appropriate bounds for the solution values 
	if(i<=j):
		return(-1,-1)
	if(j==0):	#bridge connecting to the depot
		if allow_single_node_roads:
			return (0,2) 
		else:
			return (0,1)
	return (0,1)   #other bridge


def lower_tri_filter(instance,i,j):
	# unused?) pyomo filter for selecting only valid bridges (i,j)
	return j<i


def rule_deg(instance,i):
	#returns the rule for constructing degree constraints
	return sum( ( instance.x[i,j] if i>j else instance.x[j,i] ) for j in instance.nodes if i!=j  ) == (2 if i>0 else 2*instance.number_of_vehicles)

	
def to_list_locations(instance):
	d = []
	for i in instance.nodes:
		d.append((instance.locations[i,0],instance.locations[i,1]))
	return d


def list_to_string(ls):
	#returns more readible string of a list
	expr = ""
	for el in ls:
		expr += str(el)+"/"
	return expr[:-1]