

''' librairies nécéssaires '''
from globals import *
from logging_cvrp import *
reset_timer()
log.write_globals()
log.title("starting to import necessary librairies...")
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
import graph
import matplotlib.pyplot as plt

log.write_timed("finished importing all modules")

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

log.title("initialising model")


''' initalise the solver and the abstract model '''
opt = SolverFactory('glpk')
model = pyo.AbstractModel("CVRP")



''' define model parameters '''
model.number_of_vehicles = pyo.Param(within=pyo.PositiveIntegers)
model.n = pyo.Param(within=pyo.PositiveIntegers) #pour l'instant = nombre de noeuds y compris le dépot
model.capacity = pyo.Param(within=pyo.PositiveIntegers)
model.nodes = pyo.RangeSet(0,model.n-1) #attention nodes[1] = 0 ....!!!
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
del(instance.locations) # DOESNT WORK!!

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

log.write_timed("finished constructing model")


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
			self.queue.append(instance)
			self.queue = sorted(self.queue,key = lambda x : x.lower_bound)
			self.length += 1
			self.total_length += 1
		else:
			self.branches_cut += 1
	
	def pop(self):
		while self.length>0:
			instance = self.queue.pop(0)
			self.length -= 1
			#we must verify that the condition still holds because the upper_bound may have changed since time of adding
			if instance.lower_bound < self.upper_bound :
				return instance
		return None
	
	def record_feasible_integer_solution(self,instance):
		if pyo.value(instance.objective) <  self.upper_bound : 
			self.upper_bound = pyo.value(instance.objective)
			support.integerize_solution(instance)
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
	
	log.subtitle("entering branching",instance.id)
	
	# singular branching : can lead to unsolvable problems
	# we choose the index for branching whose corresponding value is closest to 0.5
	index = -1,-1	
	dist = 1 # initialised at max value (actually max is 0.5 but oh well)
	
	available_bridges= verify_node_saturation(instance) #takes away bridges whose nodes are saturated in constraints
	
	for bridge in available_bridges:
		#do not consider bridge with depot for their bound is (0,2) and we would have to branch over 3 instances
		if abs(instance.x[bridge].value-0.5)<dist:
			index = bridge
			dist = abs(instance.x[bridge].value-0.5)
	
	if index==(-1,-1):
		#No branching index found
		log.write_timed("no branching found, branch must be cut",instance.id)
		#do nothing, branch will thus die out
		
	log.write_timed("branching found over index "+str(index)+" with value "+str(round(instance.x[index].value,4)),instance.id)	
	
	
	#creating new instances ! recycling the old one into one of the new branches 		
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
		
	instance.lower_bound = max(cbc.lower_bound(instance),pyo.value(instance.objective))
	instance2.lower_bound = max(cbc.lower_bound(instance2),pyo.value(instance2.objective))
	
	
	log.write("value of objective function is " +str(round(pyo.value(instance.objective),2)) + " for new instance " +support.list_to_string(instance.id)+" which fixed variable at 0 is. Lower bound is "+str(instance.lower_bound),id0)
	log.write("value of objective function is " +str(round(pyo.value(instance2.objective),2)) + " for new instance " +support.list_to_string(instance2.id)+" which fixed variable at 0 is. Lower bound is "+str(instance2.lower_bound),id0)

	
	instance_manager.add(instance)
	instance_manager.add(instance2)
	
	
def verify_node_saturation(instance):
	dic = { i : [0,0] for i in instance.nodes }
	for bridge in instance.x.keys():
		if bridge[1]==0:
			continue
		if instance.x[bridge].fixed:
			dic[bridge[0]][instance.x[bridge].value] += 1 
			dic[bridge[1]][instance.x[bridge].value] += 1 
			continue
	# add_implicit_constraints(instance,dic)
	return [ b for b in instance.x.keys() if ( dic[b[0]] < [instance.n-1-2,2] and dic[b[1]] < [instance.n-1-2,2] ) ]
	
def add_implicit_constraints(instance,dic):
	for i in dic.keys():
		if dic[i][0] == instance.n-1-2:
			#remaining unfixed bridges must be fixed
			return

def column_generation(instance,instance_manager):
	#iteratively adds constraints and solves the instance in order to strengthen the linear relaxation
	#STRUCTURE OF FUNCTION : 
	#loop untill the solution is "good enough" OR too many iterations :	
		#1) find violated constraints in specific order	
		#2) if non found : exit ; we have found a solution that is feasible and necessarily optimal within the current branch (since it a solution found by the solver)
			#2b : if that solution is also integer, we need not continue this branch!
		#3) else : add found constraints to constraints list, re-solve linear problem and continue
	
	log.subtitle("entering column generation",instance.id)
	
	feasible_integer_found = False
	loop_count = 1 
	unmoving_count = 0
	obj_val_old = pyo.value(instance.objective)
	while support.continue_column_generation(instance,loop_count):
		log.write("column generation loop "+str(loop_count),instance.id)
		
		#we first add capacity constraints
		success, count = constraint.add_c_cap(instance)		
		log.write_timed("we found " +str(count) + " capacity cuts",instance.id)
		
		#if we have not found a single cutting plane (=violated constraint) --> feasible and optimal solution found
		if not(success) and support.solution_is_integer(instance):
			#the solution found is valid and integer --> optimal within the branch
			feasible_integer_found = True
			
			#code to formulate correct log 
			log.write("!!!!! feasible integer solution found with objective value of "+str(round(pyo.value(instance.objective),2)),instance.id)
			old = instance_manager.upper_bound
			assertion = " " if old>pyo.value(instance.objective) else " not "
			log.write("new solution is"+assertion+"better than one previously found, this branch is dropped and its solution is"+assertion+"recorded")
			
			#actual conditional recording
			instance_manager.record_feasible_integer_solution(instance)
			
		#we add other constraints : multi_start,comb... success boolean is update at each new heuristic
		
		#remove_inactive_constraints(instance)
		
		obj_val = pyo.value(instance.objective)
		log.write( "objective function after loop "+str(loop_count)+": "+str(round(obj_val,4))+" ( "+str(support.integer_percent(instance))+"% integer )",instance.id)
		if not(success):
			instance_manager.record_partial_solution(instance)
			break
			
		#resolve instance before reiterating 
		results = opt.solve(instance)
		
		#we verify that we are not stuck in an unmoving loop
		if abs(obj_val-obj_val_old)<0.000000001 :
			unmoving_count +=1
			if unmoving_count >= max_unmoving_count : 
				log.write("no evolution in column_generation, moving on to branching",instance.id)
				break
		else:
			unmoving_count = 0
			
	
		obj_val_old = obj_val
		
		loop_count+=1
		
	return feasible_integer_found
	
def remove_inactive_constraints(instance):
	#must disable constraints that are inactive being careful of the fact that they may become active again later on...
	raise NameError("!!!!! remove_inactive_constraints must be implemented") 


def main_loop(instance_manager):

	instance = instance_manager.pop()
	
	while instance!=None:
		
		log.subtitle("starting processing of new instance with lower_bound of "+str(instance.lower_bound),instance.id)
		
		#adding constriants and resolving and verifying if we have, by chance, found an integer and feasible solution
		feasible_integer_found = column_generation(instance,instance_manager)
			
		#if we consider that we have done "enough", we also stop (typically : too many iterations)
		if instance.depth < max_depth and max_time_not_reached() and not(feasible_integer_found):
			#branch and apply main_loop to the two new instances
			branch(instance,instance_manager)
		else : 
			log.write_timed("!!!!!! branch is cut because",instance.id)
			if instance.depth >= max_depth:
				log.write(">= max_depth",instance.id)
			if not(max_time_not_reached()) :
				log.write("max time reached",instance.id)
			if feasible_integer_found :
				log.write("feasible integer solution already found",instance.id)
			instance_manager.record_partial_solution(instance)
			
		instance = instance_manager.pop() #will return none if there are no instances left in queue

################
### GRAPHING ###
################

def full_graph(instance,locations,status):
	log.write("saving "+status+" solution to "+log.name+"_"+status+"_solution_graph.png in current folder")
	g = graph.Graph(instance,locations)
	g.update_with_x(instance.x)
	fig = plt.figure(figsize=(15,15))
	plt.axis('off')
	plt.title("graph of "+status+" solution found for CVRP solving with \n"+str(instance.n.value)+" nodes, "+str(instance.number_of_vehicles.value)+" vehicles with capacity of "+str(instance.capacity.value))
	g.show()
	# fig.show()
	fig.savefig(log.name+"_"+status+"_solution_graph.png",bbox_inches='tight',dpi=fig.dpi*2)

############
### MAIN ###
############

log.write("using input file "+file)
instance.file = file
log.title("starting CVRP solving for "+str(instance.n.value)+" nodes, "+str(instance.number_of_vehicles.value)+" vehicles with capacity of "+str(instance.capacity.value))


#computing lower_bound
instance.lower_bound = cbc.lower_bound(instance)

#initialising instance manager
instance_manager = instance_manager()
instance_manager.add(instance)

log.write("initial upper bound of cvrp problem is "+str(instance_manager.upper_bound))

#solving the initial instance in order to initialize instance.x values
results = opt.solve(instance)

#printing initial value of objective function
log.write("initial value of objective function "+str(round(pyo.value(instance.objective),2))+" and is "+str(support.integer_percent(instance))+"% integer")

#saving initial graph 
full_graph(instance,locations,"initial")
log.write_timed("finished saving graph")

#printing initial value of objective function
log.write("initial value of objective function "+str(round(pyo.value(instance.objective),2))+" and is "+str(support.integer_percent(instance))+"% integer")
	
instance.id = [id.get_and_increment()]	
instance.depth = 0
main_loop(instance_manager)
log.title("finished solving")
log.write_timed("")


if instance_manager.best_feasible_integer_solution==None:
	log.write("no optimal integer solution found")
	if len(instance_manager.partial_solution_recorded)>0:
		log.write("best lower bound found :" +str(pyo.value(instance_manager.partial_solution_recorded[0].objective))+" and is "+str(support.integer_percent(instance_manager.partial_solution_recorded[0]))+"% integer")
		log.write(support.print_solution_routes(instance_manager.partial_solution_recorded[0]))
		full_graph(instance_manager.partial_solution_recorded[0],locations,"partial")
		log.write_timed("finished saving graph")
	else:
		log.write("not a single partial solution was recorded...")
else:
	# if input("show instance ? (y/n) (yes/no) \n") in ["y","yes","oui","hell","yeah"]:
		# instance_manager.best_feasible_integer_solution.display()
	log.write("best feaible integer solution found has objective value of "+str(pyo.value(instance_manager.best_feasible_integer_solution.objective)))
	log.write(support.print_solution_routes(instance_manager.best_feasible_integer_solution))
	full_graph(instance_manager.best_feasible_integer_solution,locations,"final")
	log.write_timed("finished saving graph")