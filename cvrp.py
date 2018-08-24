

''' necessary librairies '''
import sys
from random import choice

import matplotlib.pyplot as plt
import numpy as np
from numpy import float64
from numpy.random import randint as rdi
from numpy.random import random as rd

import initialize_model as init
#personnal librairies
import instance_managing as managing  # instance managing
from branching import branch  # branching function
from constraint import column_generation  # global file for all constraint managing and seperation
import constraint
from globals import graph_current_solution, last_push_integer, last_push_integer_thresh, max_depth, pyo, queue_logging, search_tree, relative_distance_closest_neighboor, last_push_integer_thresh
from graph import SearchTree, full_graph  # plotting function
from logging_cvrp import global_time, log, max_time_not_reached, queue_log,reset_timer

reset_timer()
log.write_globals()
log.title("starting to import necessary librairies...")

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

############
### MAIN ###
############
if len(sys.argv)==1:
    raise "Enter a file name for solving"
elif "carp" in sys.argv[1]:
    raise "Carp problems (without specified number of vehicles) are not yet supported"
else:
	instance,locations = init.full_init(sys.argv[1])

log.title("starting CVRP solving for "+str(instance.n.value)+" nodes, "+str(instance.number_of_vehicles.value)+" vehicles with capacity of "+str(instance.capacity.value))

''''initialise instance manager '''
instance_manager = managing.instance_manager(instance)
full_graph(instance_manager.best_feasible_integer_solution,locations,"Google Opt solution")

'''computing lower_bound and adding to queue for column generation and branching '''
managing.set_lower_bound(instance)
instance_manager.add(instance)

log.write("initial upper bound of cvrp problem is "+str(instance_manager.upper_bound))

''' printing initial value of objective function '''
log.write("initial value of objective function "+str(round(instance.objective_value,2))+" and is "+str(managing.integer_percent(instance))+"% integer")

'''saving initial graph  '''
full_graph(instance,locations,"initial")

''' list of old instances used for the graping of the search Tree '''
old_nodes = []

''' actual branch and cut '''
instance = instance_manager.pop()
while instance!=None and max_time_not_reached() and instance.depth<=max_depth:		
	log.subtitle("starting processing of new instance with lower_bound of "+str(instance.lower_bound)+" ( upper_bound is "+str(instance_manager.upper_bound)+") ,depth: "+ str(instance.depth) +", global time: "+str(global_time()),instance.id)
	
	if search_tree == "complete" or search_tree == "end":
		old_nodes.append(instance.clone())
	if search_tree == "complete":
		SearchTree(instance_manager,old_nodes).show()
	if queue_logging:
		queue_log.write(instance_manager)
	
	if graph_current_solution:
		full_graph(instance,locations,"current solution",True)

	if last_push_integer and (instance_manager.upper_bound - instance.objective_value)/instance_manager.upper_bound <= last_push_integer_thresh :
		instance.disable_constraint_deactivation = True
		integerized = managing.integerize_edges(instance,smart = True)
		log.subtitle("instance is close enough to upper bound "+ str(round((1-(instance_manager.upper_bound - instance.objective_value)/instance_manager.upper_bound)*100,2)) +"% : we integerize "+str(len(integerized))+" variables",instance.id)
		

	#adding constriants and resolving and verifying if we have, by chance, found an integer and feasible solution
	feasible_integer_found, solver_success = column_generation(instance,instance_manager)
	#if we consider that we have done "enough", we also stop (typically : too many iterations)
	if max_time_not_reached() and not(feasible_integer_found) and solver_success and instance.objective_value < instance_manager.upper_bound:
		
		#branch and apply main_loop to the two new instances
		success = branch(instance,instance_manager)
		if not(success):
			log.write("unbrancheable leaf is cut") #it is cut in the sens that it no longer is in the instance manager queue and that no descending branches have been added to it
		if search_tree == "complete":
			SearchTree(instance_manager,old_nodes).show()
		if queue_logging:
			queue_log.write(instance_manager)
	else : 
		log.write_awaiting_answer("!!!!!! branch is cut because")
		if not(max_time_not_reached()) :
			log.write("max time reached",instance.id)
			instance_manager.record_partial_solution(instance)
		elif feasible_integer_found :
			log.write("feasible integer solution already found",instance.id)
		elif not(solver_success):
			log.write("instance could not be solved after column generation")
		elif instance.objective_value >= instance_manager.upper_bound:
			log.write("instance lower bound exceeds upper bound of problem")
		full_graph(instance,locations,"partial solution that was cut "+str(round(global_time(),2)),True)

	instance = instance_manager.pop() #will return none if there are no instances left in queue

''' solving is finished '''
log.title("finished solving")
log.write_timed("")

if search_tree == "end":
	SearchTree(instance_manager,old_nodes).show()

''' printing results '''
if instance_manager.best_feasible_integer_solution==None: #no actual solution was found
	log.subtitle("no optimal integer solution found")
	if len(instance_manager.partial_solution_recorded)>0:
		log.write("best lower bound found :" +str(pyo.value(instance_manager.partial_solution_recorded[0].objective))+" and is "+str(managing.integer_percent(instance_manager.partial_solution_recorded[0]))+"% integer")
		log.write(managing.print_solution_routes(instance_manager.partial_solution_recorded[0]))
		full_graph(instance_manager.partial_solution_recorded[0],locations,"partial")
	else:
		log.write("not a single partial solution was recorded...")
else:
	log.subtitle("best feasible integer solution found has objective value of "+str(pyo.value(instance_manager.best_feasible_integer_solution.objective)))
	log.write(managing.print_solution_routes(instance_manager.best_feasible_integer_solution))
	full_graph(instance_manager.best_feasible_integer_solution,locations,"final")
