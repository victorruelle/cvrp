

''' necessary librairies '''
from logging_cvrp import log, reset_timer,max_time_not_reached
from statistics import stats
import sys
reset_timer()
log.write_globals()
log.title("starting to import necessary librairies...")
from globals import id,max_depth,pyo
from numpy.random import random as rd
from numpy.random import randint as rdi
from numpy import float64
import numpy as np
import matplotlib.pyplot as plt
from random import choice

#personnal librairies
import instance_managing as managing 			#instance managing
from constraint import column_generation        #global file for all constraint managing and seperation
import debugging                     			#debugging functions
from graph import full_graph         			#plotting function
from branching import branch					#branching function
import initialize_model as init
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
stats.init_globals(instance,instance_manager)  #initialise global parameters of stats monitoring using the constructed model and instance_manager

'''computing lower_bound and adding to queue for column generation and branching '''
managing.set_lower_bound(instance)
instance_manager.add(instance)

log.write("initial upper bound of cvrp problem is "+str(instance_manager.upper_bound))

''' printing initial value of objective function '''
log.write("initial value of objective function "+str(round(instance.objective_value,2))+" and is "+str(managing.integer_percent(instance))+"% integer")

'''saving initial graph  '''
full_graph(instance,locations,"initial")

''' actual branch and cut '''
stats.update([0,0,managing.number_of_active_constraints(instance.c_cap)],"start branch and cut")
instance = instance_manager.pop()
while instance!=None and max_time_not_reached() and instance.depth<=max_depth:		
	log.subtitle("starting processing of new instance with lower_bound of "+str(instance.lower_bound),instance.id)
	stats.update([instance.objective_value,instance.depth,managing.number_of_active_constraints(instance.c_cap)],"start instance processing")
	

	#adding constriants and resolving and verifying if we have, by chance, found an integer and feasible solution
	feasible_integer_found, solver_success = column_generation(instance,instance_manager)
	
	#if we consider that we have done "enough", we also stop (typically : too many iterations)
	if max_time_not_reached() and not(feasible_integer_found) and solver_success:
		#branch and apply main_loop to the two new instances
		success = branch(instance,instance_manager)
		if not(success):
			log.write("unbrancheable leaf is cut") #it is cut in the sens that it no longer is in the instance manager queue and that no descending branches have been added to it
	else : 
		log.write_timed("!!!!!! branch is cut because",instance.id)
		if not(max_time_not_reached()) :
			log.write("max time reached",instance.id)
			instance_manager.record_partial_solution(instance)
		if feasible_integer_found :
			log.write("feasible integer solution already found",instance.id)
		if not(solver_success):
			log.write("instance could not be solved after column generation")
	
	stats.update([instance.objective_value,instance.depth,managing.number_of_active_constraints(instance.c_cap)],"end instance processing")
	instance = instance_manager.pop() #will return none if there are no instances left in queue

''' solving is finished '''
log.title("finished solving")
log.write_timed("")

''' printing results '''
if instance_manager.best_feasible_integer_solution==None: #no actual solution was found
	log.subtitle("no optimal integer solution found")
	if len(instance_manager.partial_solution_recorded)>0:
		stats.update([instance_manager.partial_solution_recorded[0].objective_value,instance_manager.partial_solution_recorded[0].depth,managing.number_of_active_constraints(instance_manager.partial_solution_recorded[0].c_cap)],"end branch and cut")
		log.write("best lower bound found :" +str(pyo.value(instance_manager.partial_solution_recorded[0].objective))+" and is "+str(managing.integer_percent(instance_manager.partial_solution_recorded[0]))+"% integer")
		log.write(managing.print_solution_routes(instance_manager.partial_solution_recorded[0]))
		full_graph(instance_manager.partial_solution_recorded[0],locations,"partial")
	else:
		log.write("not a single partial solution was recorded...")
		stats.update([0,0,0],"end branch and cut")
else:
	stats.update([instance_manager.best_feasible_integer_solution.objective_value,instance_manager.best_feasible_integer_solution.depth,managing.number_of_active_constraints(instance_manager.best_feasible_integer_solution.c_cap)],"end branch and cut")
	log.subtitle("best feaible integer solution found has objective value of "+str(pyo.value(instance_manager.best_feasible_integer_solution.objective)))
	log.write(managing.print_solution_routes(instance_manager.best_feasible_integer_solution))
	full_graph(instance_manager.best_feasible_integer_solution,locations,"final")

stats.show()