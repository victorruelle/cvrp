# -*- coding: utf-8 -*-
"""
Created on Tue Jun 26 13:39:47 2018

@author: GVKD1542
"""
#from random import choices
from logging_cvrp import log
from globals import eps,precision
from numpy import ceil


# specific use of previous method to find only roads to are integer 
def find_integer_components(x,n):
	comps = Components(n)
	first_set = [ i for i in range(1,n) if x[i,0].value>=1-eps ]
	for i in first_set:
		if comps.free(i) :
			comps.c.append([])
			k = len(comps.c)-1
			explore_component(x,i,comps,k)
	for i in comps.r.copy():
		if comps.free(i) :
			comps.c.append([])
			k = len(comps.c)-1
			explore_component(x,i,comps,k)
	cuts = []
	for c in comps.c:
		if len(c)>1:
			cuts.append(c)
	return cuts
	

def paths_are_feasible(instance):
    #checks to capacity constraints on solution that has been found as integer
	# returns true if solution is feasible, else return false and adds corresponding constraints
	roads = find_integer_components(instance.x,instance.n.value)
	return roads_are_feasible(instance,roads)

def roads_are_feasible(instance,roads):
	for road in roads:
		demand = sum( instance.demands[i] for i in road)
		passage = sum( sum( instance.x[max(node_in,node_out),min(node_in,node_out)].value for node_out in instance.nodes if not(node_out in road) ) for node_in in road )
		lb = ceil(demand/instance.capacity.value)*2
		if passage<lb-precision:
			return False
	return True


# writing shortcut return wether i and j are connected in the given solution 
# uses a given thresh or the thresh defined in global.py if none is given
connected =  lambda x,i,j,thresh=None: x[max(i,j),min(i,j)].value>eps

def explore_component(x,i,comps,k):
	# method to find the connected components by expanding a given set of nodes written in the class "comps"
	comps.take(i,k)
	for j in comps.r:
		if connected(x,i,j):
			explore_component(x,j,comps,k)
			break

class Components:
	# class describing a set of nodes 
	def __init__(self,n):
		self.r = [i for i in range(1,n)]
		self.c = []
	
	def free(self,i):
		return i in self.r
	
	def take(self,i,k):
		self.r.remove(i)
		self.c[k].append(i)













""" DEPRECATED HAND WRITTEN HEURISTICS
some may still be interesting for further search but in all but these methods are, all in all, outdated in compraison to the c++ library and thus commented out for 
better readability 

from random import random
import pyomo.environ as pyo
import networkx as nx
from random import shuffle
from globals import connected_threshold,random_components,normal_components,max_flow_cuts,random_cuts,given_demand_components, random_shrink_cuts

###################
### ADDING CUTS ###
###################

def add_capacity_cuts_from_cuts(instance,cuts,ub="rounded"):
    # adds the capcity cuts defined by given cuts = [ [set of nodes 1], [set of nodes 2] ] to instance
	# a set of node must : not contain 0, not have any duplicates
	# returns the success of this method and sets of nodes that have been constrainted with a singular demand (can be used for branching)
	count = 0 #number of added cuts
	demand_of_non_connected_to_depot = 0
	passage_of_non_connected_to_depot = 0
	components_single_demand_constrainted = []
	for cut in cuts:
		demand = sum( instance.demands[i] for i in cut)
		passage = sum( sum( instance.x[max(node_in,node_out),min(node_in,node_out)] for node_out in instance.nodes if not(node_out in cut) ) for node_in in cut )
		lb = (ceil(demand/instance.capacity.value)*2 if ub=="rounded" else demand*2/instance.capacity.value)
		if pyo.value(passage) < lb-precision:
			instance.c_cap.add(passage>=lb-precision)
			count+=1
			if lb == 2 :
				components_single_demand_constrainted.append(cut) #will be used for next heuristic
		if instance.x[cut[-1],0].value<eps: # this component is not connected to the depot
			demand_of_non_connected_to_depot += demand
			passage_of_non_connected_to_depot += passage
	lb = (ceil(demand_of_non_connected_to_depot/instance.capacity.value)*2 if ub=="rounded" else demand_of_non_connected_to_depot*2/instance.capacity.value)
	if pyo.value(passage_of_non_connected_to_depot) < lb - precision :
		instance.c_cap.add(passage_of_non_connected_to_depot>= lb-precision)
		count += 1
	#del(passage,demand,lb,passage_of_non_connected_to_depot,demand_of_non_connected_to_depot)
	return count>0, components_single_demand_constrainted

def add_c_cap(instance):
    # global method to add all possible capacity inequalities
	# returns the success of this method and the number of cuts that were added (redundant)
	start = len(instance.c_cap)
	success = False
	#connected comps candidates
	if random_components:
		candidate_cuts = find_components(instance.x,instance.n.value,random_stop=True)
		candidate_cuts += complementaries(instance,candidate_cuts) #we also test for the complementary of each connected comp found
		success_temp,components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance,candidate_cuts)
		success = success or success_temp
	if normal_components:
		candidate_cuts = find_components(instance.x,instance.n.value)
		candidate_cuts += complementaries(instance,candidate_cuts) #we also test for the complementary of each connected comp found
		success_temp,components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance,candidate_cuts)
		success = success or success_temp
	if max_flow_cuts:
		candidate_cuts = find_maxflow_cuts(instance)
		success_temp,components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance,candidate_cuts,ub="fractional") 
		success = success or success_temp
	if random_cuts:
		candidate_cuts = find_random_cuts(instance.x,instance.n.value)
		success_temp,components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance,candidate_cuts) 
		success = success or success_temp
	if given_demand_components:
		count, success_temp = 0, False
		while not(success_temp) and count<10:
			count += 1
			candidate_cuts = find_components_given_demand(instance.x,instance.n.value,instance.demands,single_neighboor=False)
			success_temp,components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance,candidate_cuts)
			success = success or success_temp
		count, success_temp = 0, False
		while not(success_temp) and count<10:
			count += 1
			candidate_cuts = find_components_given_demand(instance.x,instance.n.value,instance.demands,single_neighboor=True)
			success_temp,components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance,candidate_cuts)
			success = success or success_temp
	if random_shrink_cuts:
		candidate_cuts = find_random_shrink_components(instance)
		success_temp,components_single_demand_constrainted = add_capacity_cuts_from_cuts(instance,candidate_cuts)
		success = success or success_temp
	del(candidate_cuts)
	connected_threshold.update()
	end = len(instance.c_cap)
	return success, end-start #second variable gives the number of cuts found during add_c_cap
	
	

###################################
### HEURISTICS FOR FINDING CUTS ###
###################################

def find_components(x, n, thresh=None, random_stop = False, single_neighboor=True):
    # finds components (roads) by starting from one node and expanding to the nodes that are connected to it
	# thresh defines the threshold value for two nodes to be connected
	# random_stop, if True, will randomly divide the roads if. This is shown to be benifitial
	# single_neighboor, if True, will force only every node to expand to maximum 1 of its neighboors. Else, if thresh is for instance at 0.3,the cuts found may be very large
	comps = Components(n)
	first_set = [ i for i in range(1,n) if x[i,0].value>=1-eps ]
	for i in first_set:
		if comps.free(i) :
			comps.c.append([])
			k = len(comps.c)-1
			explore_component(x,i,comps,k,thresh=thresh, random_stop = random_stop,single_neighboor=single_neighboor)
	for i in comps.r.copy():
		if comps.free(i) :
			comps.c.append([])
			k = len(comps.c)-1
			explore_component(x,i,comps,k,thresh=thresh, random_stop = random_stop,single_neighboor=single_neighboor)
	cuts = []
	for c in comps.c:
		if len(c)>1:
			cuts.append(c)
	return cuts

# specific use of previous method to find only roads to are integer 
find_integer_components = lambda x,n :  find_components(x,n,thresh=1-eps, single_neighboor = True)
	

''' HEURISTICS FOR FINDING VALID SETS OF NODES '''

def find_maxflow_cuts(instance):
    # applies the max-flow problem in order to find the minimum cut
	g = [ (i,j,{'capacity':1,'weigth':instance.x[max(i,j),min(i,j)]}) for i in range(1,instance.n.value) for j in range(1,instance.n.value) if i!=j]
	for i in range(1,instance.n.value):
		if instance.x[i,0].value >= instance.demands[i]*2/instance.capacity.value :
			g.append( (i,0,{'capacity':2,'weigth':instance.x[i,0].value-instance.demands[i]*2/instance.capacity.value}) )
		else:
			g.append( (instance.n.value+1,i,{'capacity':2,'weigth':-instance.x[i,0].value+instance.demands[i]*2/instance.capacity.value}) )
	val,partition = nx.minimum_cut(nx.DiGraph(g),0,instance.n.value+1)
	a = list(partition[0])
	if val<sum(max(0,-instance.x[i,0].value+instance.demands[i]*2/instance.capacity.value) for i in range(1,instance.n.value)):
		#print("max_flow cuts found")
		a.remove(0)
		return [a]
	return []

def find_components_given_demand(x,n,demands,single_neighboor = False):
    # does exactly the same as find_components except it will stop expanding a set once the target demand is reached
	# this is shown to yield good results in some cases
	comps = Components(n)
	first_set = [ i for i in range(1,n) if x[i,0].value>=1-eps ]
	for i in first_set:
		if comps.free(i) :
			comps.c.append([])
			k = len(comps.c)-1
			explore_component_given_demand(x,i,comps,k,demands,single_neighboor=single_neighboor)
	for i in comps.r.copy():
		if comps.free(i) :
			comps.c.append([])
			k = len(comps.c)-1
			explore_component_given_demand(x,i,comps,k,demands,single_neighboor=single_neighboor)
	cuts = []
	for c in comps.c:
		if len(c)>1:
			cuts.append(c)
	return cuts

def find_random_cuts(x,n,amount = 5):
	# find truly random cuts, used to put into perspective the effectiveness of the other methods
	cuts = [[]*amount]
	for i in range(len(cuts)):
		for j in range(max(1,int(random()*n))):
			k = max(1,int(random()*n))
			if not(k in cuts[i]):
				cuts[i].append(k)
	return cuts

def find_random_shrink_components(instance):
	# finds cuts by randomly shrinking the instance 
	S = pyo.value(sum(x for x in instance.x.values()))
	probs = [ instance.x[i,j].value/S for i,j in instance.x.keys() ]
	elements = [ (i,j) for i,j in instance.x.keys() ]
	representative = [i for i in range(instance.n.value)]
	while len(probs)-probs.count(0)>1:
		if len(probs)==probs.count(0):
			break
		try:
			index = elements.index(choices(elements,weights=probs)[0])
		except IndexError:
			raise NameError("could not apply choices : "+str(elements)+" "+str(probs))
		prob = probs.pop(index)
		i0,j0 = elements.pop(index)
		remainer,goner = (i0,j0) if j0!=0 else (j0,i0)
		representative[goner]=remainer
		for i in range(len(representative)):
			if representative[i]==goner:
				representative[i] = remainer
		for i in range(instance.n.value):
			if i in [i0,j0] or representative[i]!=i:
				continue
			try:
				index_remainer,index_goner = elements.index((max(i,remainer),min(i,remainer))),elements.index((max(i,goner),min(i,goner)))
			except ValueError:
				print(elements)
				raise NameError("index does not exist: "+str(((max(i,remainer),min(i,remainer)),(max(i,goner),min(i,goner))))+" "+str(representative)+" "+str((i0,j0)))
			probs[index_remainer] = abs((probs[index_remainer]*S + probs[index_goner])/(S-prob))
			probs[index_goner] = 0
			elements.pop(index_goner)
			probs.pop(index_goner)
		S -= prob
	clusters = {}
	for i in range(instance.n.value):
		parent = representative[i]
		if clusters.__contains__(parent):
			clusters[parent].append(i)
		else:
			clusters[parent] = [i]
	clusters,output = list(clusters.values()),[]
	for cluster in clusters:
		if 0 in cluster:
			continue
		output.append(cluster)
	return output

###############
### SUPPORT ###
###############



# simply a writing shortcut to find the complementary of a cut (excluding the depot)
complementary = lambda instance,cut : [i for i in range(1,instance.n.value) if not(i in cut)]

# applies previous method to a set of cuts
complementaries = lambda instance,cuts : [ complementary(instance,c) for c in cuts if complementary(instance,c)!=[] ]

def explore_component(x,i,comps,k,thresh=None,random_stop = False,random_stop_thresh = None, single_neighboor = True):
	# method to find the connected components by expanding a given set of nodes written in the class "comps"
	comps.take(i,k)
	if random_stop and connected(x,i,0) and (random()>random_stop_thresh if random_stop_thresh!=None else 0.7):
		return
	for j in comps.r:
		if connected(x,i,j,thresh=thresh):
			explore_component(x,j,comps,k)
			if single_neighboor:
				break


connected =  lambda x,i,j,thresh=None: x[max(i,j),min(i,j)].value>(connected_threshold.value if thresh==None else thresh)	



'''	 CONNETED COMPONENTS CUTS SUPPORT ''' 
  


def explore_component_given_demand(x,i,comps,k,demands,single_neighboor = False):
	# method to expand a given set of nodes written in the class "comps"
	# follows the given demand method
	comps.take(i,k)
	if sum(demands[i] for i in comps.c[k])>=8 and connected(x,i,0):
		return
	for j in comps.r:
		if connected(x,i,j):
			explore_component_given_demand(x,j,comps,k,demands)
			if single_neighboor:
				break

"""