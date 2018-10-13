# -*- coding: utf-8 -*-
"""
Created on Tue Jun 26 13:39:47 2018

@author: GVKD1542
"""
from globals import eps,column_input, force_integer_to_depot, force_closest_neighboors_integer, relative_distance_closest_neighboor,random_depot_fixing, max_column_generation_count, max_unmoving_count, unmoving_constraint_threshold, max_inactivity_period, max_column_generation_count_first_instance, max_unmoving_count_first_instance, flow_constraints, max_failure_count_cuts
from logging_cvrp import log
from time import time
import instance_managing as managing
import cpp_bridge as cpp

import cap_constraint as cap
from cap_constraint import paths_are_feasible
from random import random
from numpy import Infinity

def column_generation(instance,instance_manager):
    # general method that will apply all available cut finding algorithms in a loop whose parameters are defined in globals.py  
    log.subtitle("entering column generation",instance.id)
    initial_time,feasible_integer_found,loop_count,unmoving_count,solver_success,failure_count,obj_val_old,obj_val_old_global = round(time(),2),False,1,0,True,0,instance.objective_value,instance.objective_value
    while instance.objective_value<instance_manager.upper_bound and loop_count<=(max_column_generation_count if len(instance.id)>1 else max_column_generation_count_first_instance) and unmoving_count <= (max_unmoving_count if len(instance.id)>1 else max_unmoving_count_first_instance) and not(feasible_integer_found) and solver_success:
        log.write_awaiting_answer("loop "+str(loop_count)+ "--> ",instance.id)
        # adding cuts
        feasible_integer_found,cuts_found = column_generation_loop(instance,instance_manager)
        #solving updated instance
        if cuts_found:
            solver_success = managing.solve(instance)  

        #different cases are verified 

        # we verify if the adding cuts method is a failure 
        if not(cuts_found) and not(feasible_integer_found):
            log.write_awaiting_answer("all heurisitcs have failed")  
        
        if not(feasible_integer_found):
            log.write_answer( "improvement: "+str(round(100*(instance.objective_value-obj_val_old)/obj_val_old,5))+'%' +", objective: "+str(round(instance.objective_value,5))+", "+str(managing.integer_percent(instance))+'%'+" integer, unmoving count:"+ str(unmoving_count)+ ", reduction: "+str(instance.reduction))
        
        # we verify if the objective value is increasing, if not : the cuts found have no effect
        if (instance.objective_value-obj_val_old)/obj_val_old<unmoving_constraint_threshold:
            unmoving_count +=1
            if unmoving_count >= (max_unmoving_count if instance.depth>0 else max_unmoving_count_first_instance) : 
                log.write("no evolution in column_generation, moving on to branching",instance.id)
                break
        else:
            unmoving_count = 0

        # we verifiy if the objective value is decreasing : if it is, we have probably dropped constraints that were usefull
        if (instance.objective_value-obj_val_old)/obj_val_old<-0.0001:
            log.write("we are losing objective function value! useful constraints were dropped",instance.id)

        # we count the number of consecutive total failure 
        if not(cuts_found):
            failure_count += 1
            if failure_count>max_failure_count_cuts:
                log.write("no evolution in column_generation, moving on to branching",instance.id)
                break
        else:
            failure_count = 0

        obj_val_old = instance.objective_value
        loop_count+=1

    log.end_subtitle("end of column generation, gain "+str(instance.objective_value-obj_val_old_global)+", objective: "+str(instance.objective_value)+", time: " +str(round(time()-initial_time,2)),instance.id)
    return feasible_integer_found, solver_success
    
def column_generation_loop(instance,instance_manager):
    # a signle loop of column generation : adds all available types of cuts to the instance
    # also verifies if a feasible integer solution is found

    #0) check if solution is integer and has valid paths
    if managing.solution_is_integer(instance):
        instance0 = instance.clone()
        managing.integerize_solution(instance0)
        if paths_are_feasible(instance0):
            log.subtitle("!!!!! feasible integer solution found with objective value of "+str(round(instance0.objective_value,2)),instance0.id)
            assertion = " " if instance_manager.upper_bound>instance0.objective_value else " not "
            log.write("new solution is"+assertion+"better than one previously found, this branch is dropped and its solution is"+assertion+"recorded")
            instance_manager.record_feasible_integer_solution(instance0)
            return True,False
    
    if column_input == "python":
        raise Exception("hand implemented heuristics have been commented out in cap_constraint.py for more clarity in code reading, these methods are outdated in comparaison to the c++ library. If you whish to test them, uncomment them and comment this exception out")
        #1) adding capacity cuts
        #success, count = cap.add_c_cap(instance)        
        #log.write_awaiting_answer("capacity cuts: " +str(count))

        #2),3),4) ...
        # other types of cuts to be added
    
    elif column_input == "c++":
        success = cpp.generate_and_add_columns(instance)
    
    else:
        raise NameError("Enter a valide column input type")

    remove_inactive_constraints(instance)

    return False,success
    
    
def remove_inactive_constraints(instance):
    # renders contraints inactive if they have not been used for over max_inactivity period calls
    # if disable_constraint_deactivation = True in globals.py, this method will not do anything
    if instance.disable_constraint_deactivation:
        return
    for c,val in list(instance.dual.items()):
        parent,index = c.parent_component().name,c.index()
        if parent == 'c_deg':
            continue
        instance.constraints_inactivity[(parent,index)] = 0 if val!=0 else (instance.constraints_inactivity[(parent,index)]+1 if instance.constraints_inactivity.__contains__((parent,index)) else 1 )
        if instance.constraints_inactivity[(parent,index)] > max_inactivity_period :
            c.deactivate()
