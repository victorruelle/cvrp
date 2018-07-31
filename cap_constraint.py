# -*- coding: utf-8 -*-
"""
Created on Tue Jun 26 13:39:47 2018

@author: GVKD1542
"""

from statistics import stats
from numpy.random import choice
from random import random
import pyomo.environ as pyo
from numpy import ceil
from numpy import Infinity
from globals import eps,precision,connected_threshold,vals_for_projection,max_size_for_shrinking,max_time_for_shrinking,random_components,normal_components,max_flow_cuts,random_cuts,given_demand_components
from time import time
from itertools import combinations as comb
import networkx as nx
from random import shuffle


###################
### ADDING CUTS ###
###################

def add_c_cap(instance):
	start = len(instance.c_cap)
	success = False
	#connected comps candidates
	if random_components:
		candidate_cuts = find_components_random(instance.x,instance.n.value)
		candidate_cuts += complementaries(instance,candidate_cuts) #we also test for the complementary of each connected comp found
		success_temp,components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance,candidate_cuts,position=" random components")
		success = success or success_temp
	if normal_components:
		candidate_cuts = find_components(instance.x,instance.n.value)
		candidate_cuts += complementaries(instance,candidate_cuts) #we also test for the complementary of each connected comp found
		success_temp,components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance,candidate_cuts,position=" normal components")
		success = success or success_temp
	if max_flow_cuts:
		candidate_cuts = find_maxflow_cuts(instance)
		success_temp,components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance,candidate_cuts,ub="fractional",position=" max flow cuts") 
		success = success or success_temp
	if random_cuts:
		candidate_cuts = find_random_cuts(instance.x,instance.n.value)
		success_temp,components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance,candidate_cuts,position=" random cuts") 
		success = success or success_temp
	if given_demand_components:
		count, success_temp = 0, False
		while not(success_temp) and count<10:
			count += 1
			candidate_cuts = find_components_correct_implementation_given_demand(instance.x,instance.n.value,instance.demands)
			success_temp,components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance,candidate_cuts,position=" given demand components correct")
			success = success or success_temp
		count, success_temp = 0, False
		while not(success_temp) and count<10:
			count += 1
			candidate_cuts = find_components_given_demand(instance.x,instance.n.value,instance.demands)
			success_temp,components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance,candidate_cuts,position=" given demand components")
			success = success or success_temp
	del(candidate_cuts)
	connected_threshold.update()
	return success, len(instance.c_cap)-start #second variable gives the number of cuts found during add_c_cap
	
	

###################################
### HEURISTICS FOR FINDING CUTS ###
###################################

def add_capacity_cuts_from_cuts(instance,cuts,ub="rounded",position=None):
	count = 0 #number of added cuts
	demand_of_non_connected_to_depot = 0
	passage_of_non_connected_to_depot = 0
	components_single_demand_constrainted = []
	for c in cuts:
		demand = 0
		passage = 0 #attention on a enlevé infini, si aucun passage il faut couper aussi non?
		for node in c:
			demand += instance.demands[node]
			for i in instance.nodes:
				if not(i in c):
					passage += instance.x[max(node,i),min(node,i)]
		lb = (ceil(demand/instance.capacity)*2 if ub=="rounded" else demand*2/instance.capacity)
		if pyo.value(passage) < lb-precision:
			if position != None:
				stats.update([instance.objective_value,instance.depth,len(instance.c_cap)],"slack"+position,extra_val = pyo.value(passage) - lb+precision )
			instance.c_cap.add(passage>=lb-precision)
			count+=1
			if lb == 2 :
				components_single_demand_constrainted.append(c) #will be used for next heuristic
		if instance.x[c[-1],0].value<eps: # this component is not connected to the depot
			demand_of_non_connected_to_depot += demand
			passage_of_non_connected_to_depot += passage
	lb = (ceil(demand_of_non_connected_to_depot/instance.capacity)*2 if ub=="rounded" else demand_of_non_connected_to_depot*2/instance.capacity)
	if pyo.value(passage_of_non_connected_to_depot) < lb - precision :
		instance.c_cap.add(passage_of_non_connected_to_depot>= lb-precision)
		count += 1
	#del(passage,demand,lb,passage_of_non_connected_to_depot,demand_of_non_connected_to_depot)
	if position != None:
		stats.update([instance.objective_value,instance.depth,len(instance.c_cap)],"number of valid cuts"+position,extra_val = count )
	return count>0, components_single_demand_constrainted

	

''' H2+H3+H4 :
H2 : adapted max-flow
1) we shrink the graph
		rule : try sets S of cardinal 2,3 ; those whose lower bound value is 1 and that already have
		a generated cap constraint ; those for which x(delta(S)) = 2 imposed during branching
2) max-flow problem with lower bound capacity inequalities 
	to find cuts (subtilty to find more than one cut
3) repeat 2) 3 times with previous cuts in memory
'''

def find_maxflow_cuts(instance):
	g = [ (i,j,{'capacity':1,'weigth':instance.x[max(i,j),min(i,j)]}) for i in range(1,instance.n.value) for j in range(1,instance.n.value) if i!=j]
	for i in range(1,instance.n.value):
		if instance.x[i,0].value >= instance.demands[i]*2/instance.capacity :
			g.append( (i,0,{'capacity':2,'weigth':instance.x[i,0].value-instance.demands[i]*2/instance.capacity}) )
		else:
			g.append( (instance.n.value+1,i,{'capacity':2,'weigth':-instance.x[i,0].value+instance.demands[i]*2/instance.capacity}) )
	val,partition = nx.minimum_cut(nx.DiGraph(g),0,instance.n.value+1)
	a = list(partition[0])
	if val<sum(max(0,-instance.x[i,0].value+instance.demands[i]*2/instance.capacity) for i in range(1,instance.n.value)):
		#print("max_flow cuts found")
		a.remove(0)
		return [a]
	return []


def find_components(x,n,thresh=None):
	comps = Components(n)
	first_set = [ i for i in range(1,n) if x[i,0].value>=1-eps ]
	for i in first_set:
		if comps.free(i) :
			comps.c.append([])
			k = len(comps.c)-1
			explore_component(x,i,comps,k,thresh=thresh)
	for i in comps.r:
		if comps.free(i) :
			comps.c.append([])
			k = len(comps.c)-1
			explore_component(x,i,comps,k,thresh=thresh)
	cuts = []
	for c in comps.c:
		if len(c)>1:
			cuts.append(c)
	return cuts

def find_integer_components(x,n):
	comps = Components(n)
	first_set = [ i for i in range(1,n) if x[i,0].value>=1-eps ]
	for i in first_set:
		if comps.free(i) :
			comps.c.append([])
			k = len(comps.c)-1
			explore_integer_components(x,i,comps,k)
	for i in comps.r:
		if comps.free(i) :
			comps.c.append([])
			k = len(comps.c)-1
			explore_integer_components(x,i,comps,k)
	cuts = []
	for c in comps.c:
		if len(c)>1:
			cuts.append(c)
	return cuts

def find_components_random(x,n,thresh=0.7):
	comps = Components(n)
	first_set = [ i for i in range(1,n) if x[i,0].value>=1-eps ]
	for i in first_set:
		if comps.free(i) :
			comps.c.append([])
			k = len(comps.c)-1
			explore_component_random(x,i,comps,k,thresh=thresh)
	for i in comps.r:
		if comps.free(i) :
			comps.c.append([])
			k = len(comps.c)-1
			explore_component_random(x,i,comps,k,thresh=thresh)
	cuts = []
	for c in comps.c:
		if len(c)>1:
			cuts.append(c)
	return cuts

def find_components_given_demand(x,n,demands):
	comps = Components(n)
	first_set = [ i for i in range(1,n) if x[i,0].value>=1-eps ]
	for i in first_set:
		if comps.free(i) :
			comps.c.append([])
			k = len(comps.c)-1
			explore_component_given_demand(x,i,comps,k,demands)
	for i in comps.r:
		if comps.free(i) :
			comps.c.append([])
			k = len(comps.c)-1
			explore_component_given_demand(x,i,comps,k,demands)
	cuts = []
	for c in comps.c:
		if len(c)>1:
			cuts.append(c)
	return cuts

def find_components_correct_implementation_given_demand(x,n,demands):
	comps = Components(n)
	first_set = [ i for i in range(1,n) if x[i,0].value>=1-eps ]
	for i in first_set:
		if comps.free(i) :
			comps.c.append([])
			k = len(comps.c)-1
			explore_component_correct_implementation_given_demand(x,i,comps,k,demands)
	for i in comps.r:
		if comps.free(i) :
			comps.c.append([])
			k = len(comps.c)-1
			explore_component_correct_implementation_given_demand(x,i,comps,k,demands)
	cuts = []
	for c in comps.c:
		if len(c)>1:
			cuts.append(c)
	return cuts


def find_random_cuts(x,n):
	amount = 5
	cuts = [[]*amount]
	for i in range(len(cuts)):
		for j in range(max(1,int(random()*n))):
			k = max(1,int(random()*n))
			if not(k in cuts[i]):
				cuts[i].append(k)
	return cuts

###############
### SUPPORT ###
###############

def feasible_paths(instance):
    #checks to capacity constraints on solution that has been found as integer
	# returns true if solution is feasible, else return false and adds corresponding constraints
	roads = find_integer_components(instance.x,instance.n.value)
	success,components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance,roads)
	return not(success)

def complementaries(instance,cuts):
	return [ complementary(instance,c) for c in cuts if complementary(instance,c)!=[] ]

def complementary(instance,cut):
	return 	[i for i in range(1,instance.n.value) if not(i in cut)]

def closest(val,list=vals_for_projection):
	l = sorted(list,key = lambda x : abs(x-val))
	return l[0]


'''	 CONNETED COMPONENTS CUTS SUPPORT ''' 
  
class Components:
	def __init__(self,n):
		self.r = [i for i in range(1,n)]
		shuffle(self.r)
		self.c = []
	
	def free(self,i):
		return i in self.r
	
	def take(self,i,k):
		self.r.remove(i)
		self.c[k].append(i)
		

def explore_component_random(x,i,comps,k,thresh=0.7):
	comps.take(i,k)
	if connected(x,i,0) and random()>thresh:
		return
	for j in comps.r:
		if connected(x,i,j):
			explore_component_random(x,j,comps,k)

def explore_component_given_demand(x,i,comps,k,demands):
	comps.take(i,k)
	if sum(demands[i] for i in comps.c[k])>=8 and connected(x,i,0):
		return
	for j in comps.r:
		if connected(x,i,j):
			explore_component_given_demand(x,j,comps,k,demands)

def explore_component(x,i,comps,k,thresh=None):
	comps.take(i,k)
	for j in comps.r:
		if connected(x,i,j,thresh=thresh):
			explore_component(x,j,comps,k)

def explore_component_correct_implementation(x,i,comps,k):
#actually does what I had thought it did : i want a true road at the end, one with no junctions!
	comps.take(i,k)
	for j in comps.r:
		if connected(x,i,j):
			explore_component_correct_implementation(x,j,comps,k)
			break

def explore_component_correct_implementation_given_demand(x,i,comps,k,demands):
#actually does what I had thought it did : i want a true road at the end, one with no junctions!
	comps.take(i,k)
	if sum(demands[i] for i in comps.c[k])>=8 and connected(x,i,0):
		return
	for j in comps.r:
		if connected(x,i,j):
			explore_component_correct_implementation(x,j,comps,k)
			break


def explore_integer_components(x,i,comps,k):
	comps.take(i,k)
	for j in comps.r:
		if connected_specific(x,i,j):
			explore_integer_components(x,j,comps,k)

def connected(x,i,j,thresh=None):
	a,b = max(i,j),min(i,j)
	return x[a,b].value>(connected_threshold.value if thresh==None else thresh)

def connected_specific(x,i,j):
	a,b = max(i,j),min(i,j)
	return x[a,b].value>1-eps
	
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
		sub_sets = comb(set,i)
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
		