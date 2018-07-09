

''' librairies nécéssaires '''
from globals import *
from support import pprint
from support import reset_timer
reset_timer()
if debug:
	print("starting to import necessary librairies...")
import pyomo.environ as pyo
from pyomo.opt import SolverFactory
from numpy.random import random as rd
from numpy.random import randint as rdi
from numpy import float64
from numpy import sqrt 
from numpy import Infinity
from pyomo.core import Param
import numpy as np
import support
import constraint
import debugging
import sys
import cvrpGoogleOpt as google
import cbc

if debug:
	pprint("finished importing all modules")

##################
### HYPOTHESIS ###
##################
'''
the depot will always be at index 0 and have a demand of 0
we code the variable x into a matrix n*n that is symmetric and has a null diagonal. We choose to consider as valid, indices (i,j) such that i>j (all others will not be computed)
a solution is a varialbe x whose values ( for i>j ) are all integer a which verifies all capacity constraints and degree constraints
a feasible solution is a solution that satisfies all constraints (cap and degree) but whose values are not all integer

'''


################
### PATCHING ###
################

'''only needs to be done once (already done)
from pyomo.environ import *
import pyomo.version
if pyomo.version.version_info >= (4, 2, 0, '', 0):
    # Pyomo 4.2 mistakenly discards the original expression or rule during 
    # Expression.construct(). This makes it impossible to reconstruct expressions
    # (e.g., for iterated models). So we patch it.
    # Test whether patch is still needed:
    test_model = ConcreteModel()
    test_model.e = Expression(rule=lambda m: 0)
    if hasattr(test_model.e, "_init_rule") and test_model.e._init_rule is None:
        print ("Patching incompatible version of Pyomo." ) 
        old_construct = pyomo.environ.Expression.construct
        def new_construct(self, *args, **kwargs):
            # save rule and expression, call the function, then restore them
            _init_rule = self._init_rule
            _init_expr = self._init_expr
            old_construct(self, *args, **kwargs)
            self._init_rule = _init_rule
            self._init_expr = _init_expr
        pyomo.environ.Expression.construct = new_construct
    else:
        print ( "NOTE: Pyomo no longer removes _init_rule during Expression.construct()." )
        print ("      The Pyomo patch for Expression.construct() is probably obsolete." ) 
    del test_model
'''

######################
### INITIALISATION ###
######################


''' initalise the solver and the abstract model '''
opt = SolverFactory('glpk')
model = pyo.AbstractModel("CVRP")



''' define model parameters '''
model.number_of_vehicles = pyo.Param(within=pyo.PositiveIntegers)
model.n = pyo.Param(within=pyo.PositiveIntegers) #pour l'instant = nombre de noeuds y compris le dépot
model.capacity = pyo.Param(within=pyo.PositiveIntegers)
model.nodes = pyo.RangeSet(0,model.n-1) 
model.locations = pyo.Param(model.nodes,pyo.RangeSet(0,1))
model.demands = pyo.Param(model.nodes)



''' instanciate model with inbound data '''
if len(sys.argv)!=2:
	file = "test3.dat"
	instance = model.create_instance(file)
else:
	file = sys.argv[1]
	try:
		instance = model.create_instance(file)
	except:
		print("file not found, using test2.dat instead")
		print()
		file = "test2.dat"
		instance = model.create_instance(file)
		


''' contruct de distance parameter to save time '''
distances = {}
#distance function (euclidian)
dist = lambda x,y : sqrt( (x[0]-y[0])**2 + (x[1]-y[1])**2 )
for i in instance.nodes:
	for j in instance.nodes:
		if(i==j):
			distances[(i,j)] = 0
		else:
			distances[(i,j)] = dist((instance.locations[i,0],instance.locations[i,1]),(instance.locations[j,0],instance.locations[j,1]))
instance.costs = Param(instance.nodes,instance.nodes, initialize = distances)
locations = support.to_list_locations(instance)
del(instance.locations)

''' define variable x '''
instance.x = pyo.Var(instance.nodes,instance.nodes, bounds = support.set_bounds)
#deleting uneused variables (i<=j)
for i in instance.nodes:
	for j in instance.nodes:
		if i<=j:
			del(instance.x[i,j])



''' define the objective function '''
instance.objective = pyo.Objective( expr = sum( instance.costs[i,j]*instance.x[i,j] for i in instance.nodes for j in instance.nodes if i>j ) )



''' define degree constraints '''
instance.c_deg = pyo.Constraint(instance.nodes, rule=support.rule_deg)



''' define capacity constraints as an empty list for now '''
instance.c_cap = pyo.ConstraintList()  # on utilise cette structure pour pouvoir ajouter et supprimer des contraintes par la suite
#liste vide pour commencer

if debug:
	pprint("finished constructing model")


######################
### MAIN FUNCTIONS ###
######################

upper_bound = Infinity                #initial best objective function value found 
feasible_solution_instances = []      #global list of feasible solutions found at time of cutting
id = support.get_safe_counter()       #global inrementing id to name the different instances 

	
class instance_manager():
	#CAREFUL class is not thread safe!!
	def __init__(self):
		self.queue = []
		self.upper_bound = google.upper_bound(instance,locations)
		self.length = 0
		self.best_feasible_integer_solution = None
		self.branches_cut = 0
		self.partial_solution_recorded = []
		self.total_length = 0

	def add(self,instance):
		if instance.lower_bound <= self.upper_bound:
			index = 0
			while index<self.length and self.queue[index].lower_bound<instance.lower_bound :
				index +=1
			if index == self.length:
				self.queue.append(instance)
			else :
				self.queue.insert(index,instance)
			self.length += 1
			self.total_length += 1
		else:
			self.branches_cut += 1
	
	def pop(self):
		if self.length>0:
			instance = self.queue.pop()
			self.length -= 1
			return instance
		return None
	
	def record_feasible_integer_solution(self,instance):
		if pyo.value(instance.objective) <  self.upper_bound : 
			upper_bound = pyo.value(instance.objective)	
			self.best_feasible_integer_solution = instance
	
	def record_partial_solution(self,instance):
		index = 0
		while index<self.length and self.queue[index].lower_bound<instance.lower_bound :
			index +=1
		if index == self.length:
			self.partial_solution_recorded.append(instance)
		else :
			self.partial_solution_recorded.insert(index,instance)
	
			
		
def branch(instance,instance_manager):
	#branches instance problem into two complementary sub problem instances over a specific index
	
	if debug:
		pprint(support.list_to_string(instance.id)+"+++++ entering branching")
	
	# we choose the index for branching whose corresponding value is closest to 0.5
	index = -1,-1	
	dist = 1 # max theoretical dist for bridges considered is actually 0.5
	
	for bridge in instance.x.keys():
		#do not consider bridge with depot for their bound is (0,2) and we would have to branch over 3 instances
		if 0 in bridge:
			continue
		if abs(instance.x[bridge].value-0.5)<dist:
			index = bridge
			dist = abs(instance.x[bridge].value-0.5)
	
	if index==(-1,-1):
		raise NameError("No branching index found")
	
	if debug:
		pprint(support.list_to_string(instance.id)+"----- branching done over index "+str(index)+" with value "+str(instance.x[index].value))
		
		
	#creating new instances ! recycling the old one into one of the branches 		
	instance.x[index].fixed = True
	instance2 = instance.clone()
	instance.x[index].value = 0
	instance2.x[index].value = 1
	
	global id
	id0 = instance.id
	depth = instance.depth
	instance2.depth = depth+1
	instance.depth = depth+1
	instance.id = id0+[id.get_and_increment()]
	instance2.id = id0+[id.get_and_increment()]	
	
	if debug:
		log = instance.log
		instance.log = log+str(index)+" : 0 / "	
		instance.log = log+str(index)+" : 1 / "
		print(support.list_to_string(id0)+"----- new value of objective function for instance "+str(instance.id[-1])+" fixed 0 : "+str(round(pyo.value(instance.objective),2)))
		pprint(support.list_to_string(id0)+"----- new value of objective function for instance "+str(instance2.id[-1])+" fixed 1 : "+str(round(pyo.value(instance2.objective),2)))
		
	instance.lower_bound = cbc.lower_bound(instance)
	instance2.lower_bound = cbc.lower_bound(instance2)
	
	instance_manager.add(instance)
	instance_manager.add(instance2)
	


	
def column_generation(instance,instance_manager):
	#iteratively adds constraints and solves the instance in order to strengthen the linear relaxation
	#STRUCTURE OF FUNCTION : 
	#loop untill the solution is "good enough" OR too many iterations :	
		#1) find violated constraints in specific order	
		#2) if non found : exit ; we have found a solution that is feasible and necessarily optimal within the current branch (since it a solution found by the solver)
			#2b : if that solution is also integer, we need not continue this branch!
		#3) else : add found constraints to constraints list, re-solve linear problem and continue
	
	if debug:
		pprint(support.list_to_string(instance.id)+"+++++ entering column generation")
	
	loop_count = 1 
	failure_count = 0
	while support.continue_column_generation(instance,loop_count):
	
		#we first add capacity constraints
		success, count = constraint.add_c_cap(instance)
		connected_threshold.update()
		
		if debug:
			print(support.list_to_string(instance.id)+"----- we found " +str(count) + " capacity cuts with threshold of " +str(connected_threshold.value))
			
		#in the future : add other constraints 
		
		#remove_inactive_constraints(instance)
		
		#if we have not found a single cutting plane (=violated constraint) --> feasible and optimal solution found
		#ATTENTION : for feasible_integer_found to be true we only need to verify is_integer and connected components heuristic (not all other constraint generation heuristics)
		#therefore, when adding new heursitics in the future, this breaking condition will have to be placed else where to happen also right after said heuristic
		if not(success):
			failure_count+=1
			if debug:
				print(support.list_to_string(instance.id)+"----- no cutting planes found with threshold of " +str(connected_threshold.value))
			if failure_count>len(vals):		
				if support.solution_is_integer(instance):
					instance_manager.record_feasible_integer_solution(instance)
					if debug :
						print(support.list_to_string(instance.id)+"----- feasible solution found during column generation is integer!")
						print(support.list_to_string(instance.id)+"----- branch is recorded and cut")
						print(support.list_to_string(instance.id)+"----- value of objective function for solution is "+str(round(pyo.value(instance.objective),2)))
						print()
				else:
					instance_manager.record_partial_solution(instance)
				break
		else:
			failure_count = 0
		results = opt.solve(instance)
		
		if debug:
			pprint( support.list_to_string(instance.id)+"----- value of objective function during column generation "+str(loop_count)+" is "+str(round(pyo.value(instance.objective),10))+" and is "+str(support.integer_percent(instance))+"% integer")
		
		loop_count+=1
	
	
def remove_inactive_constraints(instance):
	#must disable constraints that are inactive being careful of the fact that they may become active again later on...
	raise NameError("!!!!! remove_inactive_constraints must be implemented") 


def main_loop(instance_manager):

	instance = instance_manager.pop()
	
	while instance!=None:
		
		#adding constriants and resolving and verifying if we have, by chance, found an integer and feasible solution
		feasible_integer_found = column_generation(instance,instance_manager)
			
		#if we consider that we have done "enough", we also stop (typically : too many iterations)
		if instance.depth < max_depth and support.max_time_not_reached() and instance_manager.best_feasible_integer_solution==None:
		
			#branch and apply main_loop to the two new instances
			branch(instance,instance_manager)
		else : 
			instance_manager.record_partial_solution(instance)
		instance = instance_manager.pop() #will return none if there are no instances left in queue


############
### MAIN ###
############

print()
print("using input file "+file)
instance.file = file
print()
print("++++++++++ starting CVRP solving for "+str(instance.n.value)+" nodes, "+str(instance.number_of_vehicles.value)+" vehicles with capacity of "+str(instance.capacity.value))
print()

#computing lower_bound
instance.lower_bound = cbc.lower_bound(instance)

#initialising instance manager
instance_manager = instance_manager()
instance_manager.add(instance)

#solving the initial instance in order to initialize instance.x values
results = opt.solve(instance)

#printing initial value of objective function
if debug:
	print("0----- initial value of objective function "+str(round(pyo.value(instance.objective),2))+" and is "+str(support.integer_percent(instance))+"% integer")
	print()	
	instance.log = ""
	
instance.id = [id.get_and_increment()]	
instance.depth = 0
main_loop(instance_manager)
pprint("++++++++++ finished solving")

if instance_manager.best_feasible_integer_solution==None:
		print("no optimal integer solution found")
		print("lower bound found :" +str(pyo.value(instance_manager.partial_solution_recorded[0].objective)))
else:
	if input("show instance ? (y/n) (yes/no) \n") in ["y","yes","oui","hell","yeah"]:
		instance_manager.best_feasible_integer_solution.display()
		


''' testing '''
''' this works well
results = opt.solve(instance)
instance.display()
success = constraint.add_c_cap(instance)
results = opt.solve(instance)
instance.display()
'''
