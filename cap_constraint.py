# -*- coding: utf-8 -*-
"""
Created on Tue Jun 26 13:39:47 2018

@author: GVKD1542
"""


from numpy.random import choice
import pyomo.environ as pyo
from numpy import ceil
from numpy import Infinity
from globals import *
from time import time
from itertools import combinations as comb
debug = False

###################
### ADDING CUTS ###
###################

successive_connectected_comp_fails = 0

def add_c_cap(instance):
	start = len(instance.c_cap)
	
	#1) connected components heuristic
	success = False
	# for i in range(len(connected_threshold.vals)):
		# success_temp,components_single_demand_constrainted = add_c_cap_connectedcomps(instance)
		# connected_threshold.update() #after each call to connectedcomps we update the associated threshold with a given strategy to optimise the next iteration's success
		# success = success or success_temp
	success,components_single_demand_constrainted = add_c_cap_connectedcomps(instance)
	connected_threshold.update()
	
	global successive_connectected_comp_fails
	if not(success):
		successive_connectected_comp_fails += 1
	else:
		successive_connectected_comp_fails = 0
	
	if successive_connectected_comp_fails > len(connected_threshold.vals):
		success = False
	else:
		success = True
	
	#2 other heuristics if first one failed
	#if not(success):
		#...

		
	return success, len(instance.c_cap)-start #second variable gives the number of cuts found during add_c_cap



##################
### HEURISTICS ###
##################


def add_c_cap_connectedcomps(instance):
	if debug:
		print()
		print("starting connectedcomps heuristic")
		
		print()
	'''heurisitque des connected components
	on calcule les routes connectées (hormis le dépot)
	et on calcule les contraintes de capacité pour les 
	sous graphes ainsi générés ainsi que pour leur complémentaire
	'''
	count = 0 #number of cuts
	comp = find_components(instance)
	demand_of_non_connected_to_depot = 0
	passage_of_non_connected_to_depot = 0
	components_single_demand_constrainted = []
	for c in comp:
		demand = 0
		passage = 0 #attention on a enlevé infini, si aucun passage il faut couper aussi non?
		for node in c:
			demand += instance.demands[node]
			for i in instance.nodes:
				if not(i in c):
					passage += instance.x[max(node,i),min(node,i)]
		lb = ceil(demand/instance.capacity)*2
		if pyo.value(passage) < lb:
			instance.c_cap.add(passage>=lb)
			count+=1
			if lb == 2 :
				components_single_demand_constrainted.append(c) #will be used for next heuristic
		if instance.x[c[-1],0].value<eps: # this component is not connected to the depot
			demand_of_non_connected_to_depot += demand
			passage_of_non_connected_to_depot += passage
			
		c_complementary = [i for i in instance.nodes if not(i in c+[0])] #0 ne doit pas être dedans!!
		demand = 0
		passage = 0
		for node in c_complementary:
			demand+= instance.demands[node]
			for i in instance.nodes:
				if not(i in c_complementary):
					passage += instance.x[max(node,i),min(node,i)]
		lb = ceil(demand/instance.capacity)*2
		if pyo.value(passage) < lb:
			instance.c_cap.add(passage>=lb)
			count+=1
			if lb == 2 :
				components_single_demand_constrainted.append(c_complementary) #will be used for next heuristic
	lb = ceil(demand_of_non_connected_to_depot/instance.capacity)*2
	if pyo.value(passage_of_non_connected_to_depot) < lb :
		instance.c_cap.add(passage_of_non_connected_to_depot>= lb)
	del(passage,demand,lb,passage_of_non_connected_to_depot,demand_of_non_connected_to_depot,comp)
	return count>0, components_single_demand_constrainted

''' H2+H3+H4 :
H2 : adapted max-flow
1) we shrink the graph
		rule : try sets S of cardinal 2,3 ; those whose lower bound value is 1 and that already have
		a generated cap constraint ; those for which x(delta(S)) = 2 imposed during branching
2) max-flow problem with lower bound capacity inequalities 
	to find cuts (subtilty to find more than one cut
3) repeat 2) 3 times with previous cuts in memory

H3 : greedy construnction
1) shrinking
2) 

H4 : ??
'''


###############
### SUPPORT ###
###############


'''	 CONNETED COMPONENTS CUTS SUPPORT ''' 
  
class Components:
	def __init__(self,n):
		self.r = [i for i in range(1,n)]
		self.c = []
	
	def free(self,i):
		return i in self.r
	
	def take(self,i,k):
		self.r.remove(i)
		self.c[k].append(i)
		
		
def find_components(instance):
	comps = Components(instance.n.value)
	for i in instance.nodes:
		if comps.free(i) :
			comps.c.append([])
			k = len(comps.c)-1
			explore_component(instance,i,comps,k)
	return comps.c
			

def explore_component(instance,i,comps,k):
	comps.take(i,k)
	for j in comps.r:
		if connected(instance,i,j):
			explore_component(instance,j,comps,k)
		

def connected(instance,i,j):
	a,b = max(i,j),min(i,j)
	return instance.x[a,b].value>connected_threshold.value
	
''' GRAPH SHRINKING '''

def shrink_graph(instance,components_single_demand_constrainted):
	#takes an instance and returns a new instance with shrunk support graph
	#we chose to disable branches between nodes of a new super-node by fixing the value of the related x variable
	#other option is :  to set the value of the costs of the edge binding them to +inf 
	shrunk_instance = instance.clone()
	start = time()
	while time()-start < max_time_for_shrinking :
		#we are going to start with sets of 2. Later implementations will look at sets of 3, 
		#those which need one truck and have had a capacity constraint generated 
		#those for which x(δ(S)) = 2 has been imposed during branching (in the later version of branching
		
		for c in components_single_demand_constrainted :
			c = c.sort(reverse=True) #we don't care about the order and sorting them will allow for easier indexing later on
			if verify_safe_shrinking(shrunk_instance,c):
				shrink_set(shrunk_instance,c)
				
		i = choice(shrunk_instance.nodes)
		j = choice(list(shrunk_instance.nodes)[:i])
		if shrunk_instance.x[i,j].value>=1:
			shrink_set(shrunk_instance,[(i,j)])
	return shrunk_instance
	
def outgoing_passage(instance,set):
	total = 2 * len(set)
	for i,j in comb(set,2):
		total -= instance.x[set[i],set[j]].value
	return total

def verify_safe_shrinking(instance,set):
	if len(set) > max_size_for_shrinking : #we don't want to have to check all subsets of sets that are too large
		return False
	if outgoing_passage(instance,set) > 2 :
		return False
	for i in range(2,len(set)):
		sub_sets = com(set,i)
		for sub in sub_sets :
			if outgoing_passage(instance,sub) < 2:
				return False
	return True

def shrink_set(instance,set):
	demand = outgoing_passage(instance,set)
	super_node = set[0]
	instance.demands[super_node] = demand
	for node in instance.nodes:
		if node not in set:
			instance.costs[max(node,super_node),min(node,super_node)] = sum( instance.costs[max(node,i),min(node,i)] for i in set )
	for node in set[1:]:
		for node2 in instance.nodes:
			instance.x[max(node,node2),min(node,node2)].value = 0
			instance.x[max(node,node2),min(node,node2)].fixed = True
		