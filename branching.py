from globals import id,max_index_branching_candidates,max_constraint_branching_candidates,opt, branching_type,max_branching_search_time,max_branching_search_attempts,pyo,simple_index_branching,branching_lower_bound,branching_upper_bound,max_branching_set_length, constraint_branching_strategy
from logging_cvrp import log,timer
from statistics import stats
from instance_managing import solve
from pyomo.environ import value
from instance_managing import number_of_active_constraints
from instance_managing import set_lower_bound
from random import random, shuffle
from numpy import inf
from time import time


def branch(instance,instance_manager):
    #branches instance problem into two complementary sub problem instances over a specific index
    #adds two new instances that have been correctly initialised to instance_manager 
    #returns success of branching 
    log.subtitle("entering branching",instance.id)
    stats.update([instance.objective_value,instance.depth,number_of_active_constraints(instance.c_cap)],"start branching")
    initial_time = time()
    if branching_type == "index":
        branched_instances = branch_over_index(instance)
    if branching_type == "constraint":
        branched_instances = branch_over_constraint(instance)
    if branching_type == "integer_fix":
        branched_instances = integer_pseudo_branch(instance)
    #at this point, branching type should be valid since it is verified in the main
    
    if instance==None:
        return False

    #adding new instances to solving queue
    for instance in branched_instances:
        solver_success = solve(instance)
        if solver_success :
            set_lower_bound(instance)
            log.write("value of objective function is " +str(round(instance.objective_value,2)) + " for new instance " +list_to_string(instance.id)+" with lower bound "+str(instance.lower_bound),instance.id[:-1])
            instance_manager.add(instance)
    
    log.end_subtitle("end of branching ("+str(round(time()-initial_time,2))+"s)",instance.id)
    stats.update([instance.objective_value,instance.depth,number_of_active_constraints(instance.c_cap)],"end branching")
    return True


def branch_over_index(instance):
    #must return the instances that have been created an initialised
    index = find_branching_index(instance)
    if index==(-1,-1):
        log.write_timed("no branching index found, branch must be cut",instance.id)
        return None,None
    log.write_timed("branching index found over index "+str(index)+" with value "+str(round(instance.x[index].value,4)),instance.id)    
    instance.branching_indexes.append(index)
    
    #creating new instances ! recycling the old one into one of the new branches
    instance.x[index].fixed = True
    instance.branched_indexes.append(index)
    instance2 = instance.clone()
    instance.x[index].value = 0
    instance2.x[index].value = 1

    increment_depth_and_id([instance,instance2])
    
    log.write_awaiting_answer("finished creating new instances",instance.id)
    return [instance,instance2]


def branch_over_constraint(instance):
    branch_set = find_branching_set(instance)

    if branch_set == []:
        log.write("no branching set found, returning to a branching over index",instance.id)
        return branch_over_index(instance)

    log.write_timed("branching constraint found over set "+str(branch_set)+" with demand "+str(round(sum(instance.demands[i]/instance.capacity.value for i in branch_set),4))+" and coboundary "+str(sum( sum( instance.x[max(node,node_out),min(node,node_out)].value for node_out in instance.nodes if not(node_out in branch_set)) for node in branch_set)),instance.id) 
    
    instance.branching_sets.append(branch_set)
    instance2 = instance.clone()
    instance3 = instance.clone()
    expression = sum( sum( instance.x[max(i,j),min(i,j)] for j in instance.nodes if not(j in branch_set) ) for i in branch_set )
    instance.c_branch.add(expression==2)
    expression = sum( sum( instance2.x[max(i,j),min(i,j)] for j in instance2.nodes if not(j in branch_set) ) for i in branch_set ) #obligé car il faut remplacer les éléments de instance par ceux de instance2
    instance2.c_branch.add(expression==4)
    expression = sum( sum( instance3.x[max(i,j),min(i,j)] for j in instance3.nodes if not(j in branch_set) ) for i in branch_set ) #obligé car il faut remplacer les éléments de instance par ceux de instance2
    instance3.c_branch.add(expression>=6)

    instances = [instance,instance2,instance3]
    
    increment_depth_and_id(instances)

    log.write_awaiting_answer("finished creating new instances",instance.id)
    return instances


def integer_pseudo_branch(instance):
    index = find_branching_index(instance)
    instance.x[index].domain = pyo.NonNegativeIntegers
    instance.branched_indexes.append(index)
    instance.depth += 1
    instance.id = id.get_and_increment()
    log.write_awaiting_answer("finished creating new instance",instance.id)
    return [instance]
        
def increment_depth_and_id(instances):
    #global id
    id0 = instances[0].id
    depth = instances[0].depth
    for inst in instances: 
        inst.depth = depth+1
        #inst.id = id0+[id.get_and_increment()]   
        inst.id = id0+[instances.index(inst)] 


def find_branching_index(instance):
    #must return an index for branching
    #simple mode chooses the closest to 0.5, if false, it will look for the best branching index among max_branching_candidates number of candidates
    index = -1,-1
    if simple_index_branching: 
        dist = 1
        for bridge in instance.x.keys():
            if abs(instance.x[bridge].value-0.5)<dist:
                index = bridge
                dist = abs(instance.x[bridge].value-0.5)
    else:
        bridges = [(i,j) for i in range(instance.n.value) for j in range(i)]
        indexes = sorted(bridges,key = lambda bridge:min(abs(instance.x[bridge].value-0.5),abs(instance.x[bridge].value-0.75)))[:min(len(bridges),max_index_branching_candidates)] 
        best_boost = 0
        for ind in indexes:
            instance.x[ind].value=1
            instance.x[ind].fixed=True
            opt.solve(instance)
            a = value(instance.objective)
            instance.x[ind].value=0
            opt.solve(instance)
            b = value(instance.objective)
            c = min(a,b)
            if c>best_boost:
                best_boost = c
                index = ind
            instance.x[ind].fixed = False
    return index

def find_branching_set(instance):
    sets,count = [],0
    while count < max_branching_search_attempts:
        #print("none found, searched",len(sets.searched_sets),"sets")
        sets += find_sets_on_coboundary(instance)
        count += 1
    
    if sets == []:
        return []

    log.write("testing best possible branching between "+str(len(sets))+" sets",instance.id)

    if constraint_branching_strategy == "distance_to_depot":
        sets = sorted(sets, key = lambda a_set : sorted(a_set, key = lambda node : instance.costs[(node,0)])[0]+sorted(a_set, key = lambda node : instance.costs[(node,0)])[1], reverse = True )
        return sets[0]

    if constraint_branching_strategy == "demand":
        sets = sorted( sets, key = lambda a_set : sum( instance.demands[node] for node in a_set), reverse = True)
        return sets[0]

    if constraint_branching_strategy == "distance_demand_mix":
        sets = sorted(sets, key = lambda a_set : (sorted(a_set, key = lambda node : instance.costs[(node,0)])[0]+sorted(a_set, key = lambda node : instance.costs[(node,0)])[1])/instance.max_cost + sum( instance.demands[node] for node in a_set)/instance.capacity, reverse = True )
        return sets[0]

    if constraint_branching_strategy == "exhaustive":
        sets = sorted(sets, key = lambda a_set : sorted(a_set, key = lambda node : (instance.costs[(node,0)])[0]+sorted(a_set, key = lambda node : instance.costs[(node,0)])[1])/instance.max_cost + sum( instance.demands[node] for node in a_set)/instance.capacity, reverse = True )
        best_set,best_delta,initial_value = 0,0,instance.objective_value
        for branch_set in sets:
            expression = sum( sum( instance.x[max(i,j),min(i,j)] for j in instance.nodes if not(j in branch_set) ) for i in branch_set )
            instance.c_branch.add(expression==2)
            solve(instance,silent=True)
            delta = instance.objective_value - initial_value
            del(instance.c_branch[list(instance.c_branch.keys())[-1]])

            expression = sum( sum( instance.x[max(i,j),min(i,j)] for j in instance.nodes if not(j in branch_set) ) for i in branch_set )
            instance.c_branch.add(expression==4)
            solve(instance,silent=True)
            delta = min(delta,instance.objective_value - initial_value)
            del(instance.c_branch[list(instance.c_branch.keys())[-1]])

            if delta > best_delta:
                best_delta = delta
                best_set = branch_set

        log.write("resolving instance to its initial state after best gain found is "+str(best_delta),instance.id)
        solve(instance)
        return best_set   

    raise NameError("enter a valid constraint_branching_strategy") 

def find_sets_on_coboundary(instance,lb=branching_lower_bound,ub=branching_upper_bound): 
    sets = Search_set_manager(lb,ub)
    nodes = sorted(instance.nodes,key = lambda node : instance.costs[node,0],reverse=True)
    #for node in sorted(nodes,key = lambda node : nodes.index(node)*random()):
    for node in sorted(nodes,key = lambda node : random()):
        if node==0:
            continue
        a_set = [node]
        explore_set(instance,a_set,sets)
    return sets.valid_sets

def explore_set(instance,a_set,sets):
    #print("valid sets",len(sets.valid_sets),"and searched sets",len(sets.searched_sets),"current set demand",a_set.demand,"and current set coboundary",a_set.coboundary)
    if len(a_set)>max_branching_set_length or sets.is_invalid(a_set,instance) or len(sets.valid_sets)>5 or sets.timer.global_time()>max_branching_search_time:
        return
    sets.searched_sets.append(a_set)
    if sets.is_valid(a_set,instance):
        sets.record_set(a_set)
        return #THIS IS A TEST! 
    #print(a_set.demand,a_set.set)
    sorted_nodes = sorted(instance.nodes, key = lambda node : instance.costs[(a_set[-1],node)])
    #sorted_nodes = sorted(instance.nodes, key = lambda node : instance.x[max(a_set[-1],node),min(a_set[-1],node)].value if node!=a_set[-1] else inf)
    for node in sorted_nodes:
        if not(node in a_set) and node!=0:
            a_new_set = a_set.copy()
            a_new_set.append(node)
            explore_set(instance,a_new_set,sets)
    del(a_set)
    
class Search_set_manager():
    def __init__(self,lb,ub):
        self.valid_sets = []
        self.searched_sets = []
        self.ub = ub
        self.lb = lb
        self.timer = timer()
    
    def is_valid(self,a_set,instance):
        if self.lb<=coboundary(a_set,instance)<=self.ub:
            return True
        return False

    def is_invalid(self,a_set,instance):
        return sum(instance.demands[node] for node in a_set)/instance.capacity.value > 4 or a_set in self.searched_sets

    def record_set(self,a_set):
        self.valid_sets.append(a_set)


coboundary = lambda a_set,instance: sum( sum( instance.x[max(node,node_out),min(node,node_out)].value for node_out in instance.nodes if not(node_out in a_set) ) for node in a_set )
    

def list_to_string(ls):
	#returns more readible string of a list
	expr = ""
	for el in ls:
		expr += str(el)+"/"
	return expr[:-1]