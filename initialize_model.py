from logging_cvrp import log
from globals import id, pyo, opt, dilate, branching_type, force_integer_to_depot, reduce_problem, initial_problem_reduction, force_all_edges_integer, allow_single_node_roads, flow_constraints
import sys
from numpy import sqrt 
from pyomo.core import Param
from formatting_data import create_dat_smart
from random import random
import instance_managing as managing


###############################
### INITIALISATION OF MODEL ###
###############################

def full_init(file):
	model = initialize_model()
	instance = construct_instance(model,file)
	instance,locations = initialize_instance(instance)
	return instance,locations

def initialize_model(locations=True):
	log.title("initialising model")
	''' initalise the abstract model '''
	model = pyo.AbstractModel("CVRP")
	''' define model parameters '''
	model.number_of_vehicles = pyo.Param(within=pyo.PositiveIntegers)
	model.n = pyo.Param(within=pyo.PositiveIntegers) #pour l'instant = nombre de noeuds y compris le dépot
	model.capacity = pyo.Param(within=pyo.PositiveIntegers)
	model.entry_type = pyo.Param(within=pyo.Any)
	model.nodes = pyo.RangeSet(0,model.n-1) #attention nodes[1] = 0 ....!!!
	model.demands = pyo.Param(model.nodes)
	model.locations = pyo.Param(model.nodes,pyo.RangeSet(0,1))
	model.costs = pyo.Param(model.nodes,model.nodes)
	return model


def construct_instance(model,file):
	if ".dat" in file:
		try:
			instance = model.create_instance(file)
		except:
			raise "file not found"
	elif ".vrp" in file:
		try:
			file_name = file[:-4]+".dat"
			create_dat_smart(file,file_name)
			instance = model.create_instance(file_name)
		except:
			raise "vrp not found or could not be translated correctly"
	else:
		raise NameError("given file name could not be resolved")
	instance.file = file
	return instance

def initialize_instance(instance):
	if instance.entry_type.value == "COORD":
		''' we need to retrieve the locations and delete them from all instances for smaller size'''
		locations = to_list_locations(instance) #global parameter used for graphing of solutions
		del(instance.locations) # WORKS! 

	else: #entry type is necessarily "WEIGHT"
		''' we need to generate random coordinates for the plotting '''
		locations = [ (random()*20,random()*20) for i in range(instance.n.value)]

	max_cost = 0 
	for k in instance.costs.values():
		if k>max_cost:
			max_cost = k
	instance.max_cost = max_cost
	#managing.normalize_costs(instance)



	''' define variable x '''
	instance.x = pyo.Var(instance.nodes,instance.nodes, bounds =set_bounds)
	#deleting uneused variables (i<=j)
	for i in instance.nodes:
		for j in instance.nodes:
			if i<=j:
				del(instance.x[i,j])
				# instance.x[i,j].deactivate()
	
	''' define flow variable '''
	instance.flow = pyo.Var(instance.nodes,instance.nodes, bounds = set_bounds_flow)


	''' define the objective function '''
	instance.objective = pyo.Objective( expr = sum( instance.costs[i,j]*instance.x[i,j] for i in instance.nodes for j in instance.nodes if i>j ) )


	''' define degree constraints '''
	instance.c_deg = pyo.Constraint(instance.nodes, rule=rule_deg)

	''' define flow constraints '''
	if flow_constraints:
		instance.c_flow = pyo.Constraint(instance.nodes, rule=rule_flow)
		instance.c_flow_deg = pyo.Constraint(instance.nodes, instance.nodes, rule=rule_flow_deg)
		instance.c_flow_deg_indirect = pyo.Constraint(instance.nodes, instance.nodes, rule=rule_flow_deg_indirect)

	''' define capacity constraints as an empty list for now '''
	instance.c_cap = pyo.ConstraintList()  # on utilise cette structure pour pouvoir ajouter et supprimer des contraintes par la suite
	#liste vide pour commencer

	''' defining dual values '''
	instance.dual = pyo.Suffix(direction=pyo.Suffix.IMPORT)

	''' definig list of values that will be fixed by branching (used for problem reduction) if index branching is used '''
	''' or defining list of constraints used for branching if constraint branching is used '''
	if branching_type == "index" or branching_type == "integer_fix":
		instance.branched_indexes = []
	elif branching_type == "constraint":
		instance.c_branch = pyo.ConstraintList()
		instance.branching_sets = []
		instance.branched_indexes = []
	else:
		log.subtitle("instance generation failed because no valid branching type is given")
		raise NameError("enter a valid brancing type in globals settings")
		
	instance.reduction = 1
	instance.id = [id.get_and_increment()]	
	instance.depth = 0
	instance.constraints_inactivity = []
	instance.branching_indexes = []
	if reduce_problem:
		managing.reduce_problem(instance,initial_problem_reduction)
	#managing.reduce_problem_neighboors(instance,5)
	if force_integer_to_depot:
		managing.fix_edges_to_depot(instance)
	if force_all_edges_integer:
		managing.fix_all_edges(instance)

	'''solving the initial instance in order to initialize instance.x values '''
	opt.solve(instance)
	instance.objective_value = pyo.value(instance.objective) #recording this as attribute to save computation time

	
	log.write_timed("finished constructing instance model")
	return instance,locations

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


def set_bounds_flow(instance, i, j):
	return (0,1)   #other bridge

def lower_tri_filter(instance,i,j):
	# unused?) pyomo filter for selecting only valid bridges (i,j)
	return j<i


def rule_deg(instance,i):
	#returns the rule for constructing degree constraints
	return sum( ( instance.x[i,j] if i>j else instance.x[j,i] ) for j in instance.nodes if i!=j  ) == (2 if i>0 else 2*instance.number_of_vehicles)


def rule_flow(instance,i):
	return pyo.Constraint.Skip if i==0 else ( sum( instance.flow[i,j] for j in instance.nodes if i!=j ) >= sum( instance.flow[j,i] for j in instance.nodes if i!=j ) + instance.demands[i]/instance.capacity ) 

def rule_flow_deg(instance,i,j):
	return pyo.Constraint.Skip if j>=i else (instance.x[i,j] >= instance.flow[i,j])

def rule_flow_deg_indirect(instance,i,j):
	return pyo.Constraint.Skip if j>=i else (instance.x[i,j] >= instance.flow[j,i])
	
def to_list_locations(instance):
	d = []
	for i in instance.nodes:
		d.append((instance.locations[i,0],instance.locations[i,1]))
	return d


if __name__=="main":
	''' instanciate model with inbound data '''
	model = initialize_model()
	if len(sys.argv)==1:
		file = "test3.dat"
		instance = model.create_instance(file)
	elif len(sys.argv)==2:
		file = sys.argv[1]
		instance = construct_instance(model,file)
	else:
		raise "erroneous entry arguments"
	log.write("using input file "+file)
	instance,locations = initialize_instance(instance)