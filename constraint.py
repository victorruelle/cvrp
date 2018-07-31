# -*- coding: utf-8 -*-
"""
Created on Tue Jun 26 13:39:47 2018

@author: GVKD1542
"""
from globals import id,attempt_freeze, max_column_generation_count, max_unmoving_count, unmoving_constraint_threshold, max_inactivity_period, max_column_generation_count_first_instance, max_unmoving_count_first_instance, flow_constraints, max_failure_count_cuts
from statistics import stats
from logging_cvrp import log
from time import time
import instance_managing as managing

from cap_constraint import add_c_cap
from cap_constraint import feasible_paths
from random import random
from numpy import Infinity
# from comb_constraint import *
# from multistar_constraint import *
# from hypotour_constraint import * 

""" missing :
    gomory cuts
    framed capacity cuts
"""

def column_generation(instance,instance_manager,mile_stone=Infinity):    
    log.subtitle("entering column generation",instance.id)
    stats.update([instance.objective_value,instance.depth,managing.number_of_active_constraints(instance.c_cap)],"start column generation")
    initial_time,feasible_integer_found,loop_count,unmoving_count,solver_success,failure_count,obj_val_old,obj_val_old_global = round(time(),2),False,1,0,True,0,instance.objective_value,instance.objective_value
    while loop_count<(max_column_generation_count if instance.id[-1]>0 else max_column_generation_count_first_instance) and unmoving_count <= (max_unmoving_count if instance.id[-1]>0 else max_unmoving_count_first_instance) and not(feasible_integer_found) and solver_success and instance.objective_value<mile_stone:
        log.write_awaiting_answer("loop "+str(loop_count)+ "--> ",instance.id)
        feasible_integer_found,cuts_found = column_generation_loop(instance_manager,instance,unmoving_count)
        if cuts_found:
            solver_success = managing.solve(instance)
        log.write_answer( "objective: "+str(round(instance.objective_value,5))+", "+str(managing.integer_percent(instance))+'%'+" integer, unmoving count:"+ str(unmoving_count)+ ", reduction: "+str(instance.reduction))
        if attempt_freeze and 100>managing.integer_percent(instance)>=97:
            log.write_awaiting_answer("trying an integer solution--> ",instance.id)
            instance2 = instance.clone()
            managing.freeze_integer_edges(instance2)
            solver_success = managing.solve(instance2,silent=True)
            feasible_integer_found,cuts_found = column_generation_loop(instance_manager,instance2,unmoving_count)
            log.write_answer("success!!" if feasible_integer_found else "no success")
            del(instance2)
        if instance.objective_value-obj_val_old<unmoving_constraint_threshold:
            unmoving_count +=1
            if unmoving_count >= (max_unmoving_count if instance.depth>0 else max_unmoving_count_first_instance) : 
                log.write("no evolution in column_generation, moving on to branching",instance.id)
                break
        else:
            unmoving_count = 0
        if instance.objective_value-obj_val_old<-0.1:
            log.write("we are losing objective function value! useful constraints were dropped",instance.id)
        if not(cuts_found):
            failure_count += 1
            if failure_count>max_failure_count_cuts:
                log.write("no evolution in column_generation, moving on to branching",instance.id)
                break
        else:
            failure_count = 0
        obj_val_old = instance.objective_value
        loop_count+=1

    log.end_subtitle("end of column generation ("+str(round(time()-initial_time,2))+"s), gain "+str(instance.objective_value-obj_val_old_global)+", objective: "+str(instance.objective_value),instance.id)

    stats.update([instance.objective_value,instance.depth,managing.number_of_active_constraints(instance.c_cap)],"end column generation")
    return feasible_integer_found, solver_success
    
def column_generation_loop(instance_manager,instance,unmoving_count,fixing = False):
    stats.update([instance.objective_value,instance.depth,managing.number_of_active_constraints(instance.c_cap)],"start iteration column generation")
    test = random()>0.9 and fixing
    if test:
        log.write(" (edges fixed) ",instance.id)
        managing.fix_edges_to_depot(instance)
    #0) check if solution is integer and has valid paths
    if managing.solution_is_integer(instance):
        managing.integerize_solution(instance)
        if feasible_paths(instance):
            log.subtitle("!!!!! feasible integer solution found with objective value of "+str(round(instance.objective_value,2)),instance.id)
            assertion = " " if instance_manager.upper_bound>instance.objective_value else " not "
            log.write("new solution is"+assertion+"better than one previously found, this branch is dropped and its solution is"+assertion+"recorded")
            instance_manager.record_feasible_integer_solution(instance)
            return True,False
    #1) 
    success, count = add_c_cap(instance)        
    log.write_awaiting_answer("capacity cuts: " +str(count))

    #2),3),4) ...
    #to be added

    remove_inactive_constraints(instance)
    
    if test:
        managing.unfix_edges_to_depot(instance)

    #asserting success of all seperation heuristics
    if not(success):
        log.write_awaiting_answer("all heurisitcs have failed")
        return False,False #it's okay, let's try more and wait for the unmoving stopping criteria to take over
    
    stats.update([instance.objective_value,instance.depth,managing.number_of_active_constraints(instance.c_cap)],"end iteration column generation")
    return False,True
    
    
def remove_inactive_constraints(instance):
    #renders contraints inactive if they have not been used for over max_inactivity period calls
    instance.constraints_inactivity += [0 for i in range(len(instance.c_cap)-len(instance.constraints_inactivity))]
    i = 0
    for c,val in list(instance.dual.items())[instance.n.value +(instance.n.value+2*(instance.n.value**2) if flow_constraints else 0):]:
        if val == 0:
            instance.constraints_inactivity[i] += 1
            if instance.constraints_inactivity[i] > max_inactivity_period :
                c.deactivate()    
        else :
            instance.constraints_inactivity[i] = 0    
        i += 1
