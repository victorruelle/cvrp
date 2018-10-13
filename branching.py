from globals import cpp_constraint_branching,constraint_branching_amount, eps,precision,constraint_branching_strategy,constraint_set_sorting_strategy,integer_branching_strategy,index_branching,integer_branching,index_branching_strategy,max_index_branching_candidates,max_constraint_branching_candidates,opt,pyo
from logging_cvrp import log,timer
from instance_managing import solve
from pyomo.environ import value
from instance_managing import number_of_active_constraints
from instance_managing import set_lower_bound
from random import random, shuffle, choices,randint
from numpy import inf
from time import time
import cpp_bridge as cpp

def branch(instance,instance_manager):
    #branches instance problem into complementary sub problem instances following various rules
    #adds new instances that have been correctly initialised to instance_manager 
    #returns success of branching 
    log.subtitle("entering branching",instance.id)
    initial_time = time()

    branching_candidates = []
    if index_branching:
        partial_time = time()
        log.write_awaiting_answer("index branching--> ")
        candidate = index_branching_candidates(instance)
        if candidate["instances"] != None:
            branching_candidates.append(candidate)
            log.write_answer(", score: "+str(branching_candidates[-1]["score"])+", time: "+str(round(time()-partial_time,2)))
            log.write_answer(", score: "+str(branching_candidates[-1]["score"])+", time: "+str(round(time()-partial_time,2)))
    if integer_branching:
        partial_time = time()
        log.write_awaiting_answer("integer branching--> ")
        candidate = integer_branching_candidates(instance)
        if candidate["instances"] != None:
            branching_candidates.append(candidate)
            log.write(", score: "+str(branching_candidates[-1]["score"])+", time: "+str(round(time()-partial_time,2)))
    
    if cpp_constraint_branching:
        partial_time = time()
        log.write_awaiting_answer("c++ constraint branching--> ")
        candidate = cpp_constraint_branching_candidate(instance)
        if candidate["instances"] != None:
            branching_candidates.append(candidate)
            log.write(", score: "+str(branching_candidates[-1]["score"])+", time: "+str(round(time()-partial_time,2)))
        
    if branching_candidates==[]:
        return False

    #coefficients = {"integer branching":1.1,"constraint branching":0.8,"index branching":1} #the bigger the coefficient, the lyklier it will be selected
    #best_candidate = sorted( branching_candidates, key = lambda candidate : candidate["score"]*coefficients[candidate["method"]], reverse=True )[0]
    best_candidate = select_best_candidate(branching_candidates,instance)
    log.write("BEST STRATEGY: "+str(best_candidate["method"]))
    
    #adding new instances to solving queue
    for instance in best_candidate["instances"]:
        set_lower_bound(instance)
        log.write("value of objective function is " +str(round(instance.objective_value,2)) + " for new instance " +list_to_string(instance.id)+" with lower bound "+str(instance.lower_bound)[:-1])
        instance_manager.add(instance)
    
    log.end_subtitle("end of branching, method: "+best_candidate["method"]+", smallest boost: "+str(best_candidate["score"]) +", time: "+str(round(time()-initial_time,2))+" seconds")
    return True

def select_best_candidate(branching_candidates,initial_instance):
    # heuristics to select "best" candidate
    # we want to avoid having too many integer branchings to avoid too great calculation time. 

    # first ranking factor : minimum boost
    for candidate in branching_candidates:
        log.write_awaiting_answer(candidate["method"]+":" )
        for instance in candidate["instances"]:
            log.write_awaiting_answer(str(instance.id[-1])+", boost : "+str(instance.objective_value-initial_instance.objective_value))
        log.write_answer("")
    if len(branching_candidates)==1:
        return branching_candidates[0]
    for candidate in branching_candidates:
        for inst in candidate["instances"]:
            if inst.objective_value-initial_instance.objective_value<precision:
                branching_candidates.remove(candidate)
                break
    if len(branching_candidates)==1:
        return branching_candidates[0]
    clear_winner_thresh = 2
    min_boosts = {branching_candidate["method"]:(min( branching_candidate["instances"], key = lambda instance : instance.objective_value - initial_instance.objective_value ).objective_value-initial_instance.objective_value) for branching_candidate in branching_candidates}
    branching_candidates = sorted( branching_candidates, key = lambda candidate :   min_boosts[candidate["method"]], reverse = True ) #sorts them in decreasing order of smallest boost
    # we have at least 2
    try:
        if min_boosts[branching_candidates[0]["method"]] / min_boosts[branching_candidates[1]["method"]] >= clear_winner_thresh:
            log.write("biggest smallest boost is clear winner")
            return branching_candidates[0]
        # if we get here, the two "best" candidates are not distinguishable by their smallest boost ("best" as in biggest smallest boost)
        if len(branching_candidates) == 3 and min_boosts[branching_candidates[2]["method"]] / min_boosts[branching_candidates[0]["method"]] >= clear_winner_thresh: # checking if the last candidate holds a chance
            branching_candidates.pop(2)
    except ZeroDivisionError:
        raise Exception(str([inst.objective_value for inst in branching_candidates[0]["instances"]]) + " " + str([inst.objective_value for inst in branching_candidates[1]["instances"]]) + " " + str([inst.objective_value for inst in branching_candidates[2]["instances"]]) + " " + str(initial_instance.objective_value) )
    # at this point, all remaining candidates have a similar smallest boost, other factors must be taken into account 
    # second class ranking factors : number of instances created and their respective boost
    # we like having only one instance, but that considerably slows down solving process (integer fixing): we must find a trade-off
    branching_candidates = sorted( branching_candidates, key = lambda candidate : len(candidate["instances"]) )
    for candidate in branching_candidates:
        if len(candidate["instances"])==1 and candidate["method"]!= "integer branching":
            log.write("not intger branching with cut branches wins")
            return candidate # smallest amount of instances without slowing down solving time: perfect! (happens for instance when index branching has one of its branches unfeasible)
    m = sum( 1 for b in initial_instance.id if b==-1 ) # number of fixed integers
    if m<=6 and random()>0.6: #we can still fix integers without too much trouble
        for candidate in branching_candidates: 
            if len(candidate["instances"])==1:
                log.write("not too many integer branching")
                return candidate # this will always only return integer_branching if it is still in the remaining candidates
    # now we will simply assess if the remaining candidates have highly unbalanced instances, which is good. If so, we choose them, else we will simply take that with least amount of instances
    highly_unbalanced_candidates = []
    for candidate in branching_candidates:
        if len(candidate["instances"])>1 and max( (abs(candidate["instances"][j].objective_value-initial_instance.objective_value))/max((candidate["instances"][i].objective_value-initial_instance.objective_value),0.00001) for i in range(len(candidate["instances"])) for j in range(len(candidate["instances"])) if i!=j ) > 10 :
            highly_unbalanced_candidates.append(candidate)
    if len(highly_unbalanced_candidates)==1:
        log.write("unique highly unbalanced is winner")
        return highly_unbalanced_candidates[0]
    if len(highly_unbalanced_candidates)>=2: #can only be equal to 1 or 2
        log.write("smallest highly unbalanced is winner")
        return min( highly_unbalanced_candidates, key = lambda candidate : len(candidate["instances"]) )
    log.write("smallest balanced is winner")
    return branching_candidates[0]

def candidate_score(candidate,initial_objective_value):
    # candidate = [instance1,instance2,...]
    # returns a score based on the objective value of the given instances
    if len(candidate)==0:
        return 0
    candidate = sorted( candidate, key = lambda inst : inst.objective_value )
    weights = [1,0.6,0.4]
    return sum( (candidate[i].objective_value-initial_objective_value)*weights[i] for i in range(len(candidate)) )

''' INTEGER BRANCHING '''
 
def integer_branching_candidates(instance,strategy = integer_branching_strategy):
    # returns candidates for integer_branching
    indexes = order_best_indexes(instance)
    if len(indexes)==0:
        return {"instances":None,"score":None, "method":"integer branching"}
    if strategy == "simple": 
        index = indexes[0]
        log.write_awaiting_answer("branching index: "+str(index)+", value "+str(round(instance.x[index].value,4))) 
        instance0 = instance.clone()
        instance0.branching_indexes.append(index)
        instance0.x[index].domain = pyo.NonNegativeIntegers
        if not(solve(instance0,silent=True)):
            return {"instances":None,"score":None, "method":"integer branching"}
        candidate = [instance0]
        score = candidate_score(candidate,instance.objective_value)
        increment_depth_and_id(candidate,"integer branching")
        return {"instances":candidate, "score":score, "method":"integer branching"}

    if strategy == "exhaustive":
        candidate_indexes = indexes[:max_index_branching_candidates] 
        best_score,best_instance,best_index = 0,None, (-1,-1)
        for ind in candidate_indexes:
            instance0 = instance.clone()
            instance0.x[ind].domain = pyo.NonNegativeIntegers
            if not(solve(instance0,silent=True)):
                continue
            score = candidate_score([instance0],instance.objective_value)
            if score > best_score:
                best_score,best_instance,best_index = score,instance0.clone(),ind
            del(instance0)
        if best_index == (-1,-1): #should never happen
            log.write_answer("no branching index found, branch must be cut")
            return {"instances":None,"score":None, "method":"integer branching"}
        log.write_awaiting_answer("branching index: "+str(best_index)+", value "+str(round(instance.x[best_index].value,4))) 
        best_instance.branching_indexes.append(best_index)
        increment_depth_and_id([best_instance],"integer branching")
        return {"instances":[best_instance],"score":best_score, "method":"integer branching"}



''' INDEX BRANCHING '''
    
def index_branching_candidates(instance,strategy = index_branching_strategy):
    # return index branching candidates
    #simple mode strategy the closest to 0.5, if false, it will look for the best branching index among max_branching_candidates number of candidates
    indexes = order_best_indexes2(instance)
    if strategy == "simple":
        index = indexes[0]
        log.write_awaiting_answer("branching index: "+str(index)+", value "+str(round(instance.x[index].value,4))) 
        instance0 = instance.clone()
        instance0.branching_indexes.append(index)
        instance0.x[index].fixed = True
        instance0.branching_indexes.append(index)
        instance1 = instance0.clone()
        instance0.x[index].value = 0
        instance1.x[index].value = 1
        success0 = solve(instance0,silent=True)
        success1 = solve(instance1,silent=True)
        candidate  = ( [instance0] if success0 else [] )  + ( [instance1] if success1 else [] )
        increment_depth_and_id(candidate,"index branching")
        return {"instances":candidate, "score":candidate_score(candidate,instance.objective_value), "method":"index branching"}

    if strategy == "exhaustive":
        candidate_indexes = indexes[:max_index_branching_candidates] #slicing beyond length generates no exception
        log.write_awaiting_answer("candidate indexes: "+str(len(candidate_indexes)))
        best_score,best_candidate,best_index = 0,[instance.clone()], (-1,-1)
        for ind in candidate_indexes:
            instance0 = instance.clone()
            instance0.x[ind].fixed=True
            instance0.x[ind].value=0
            success0 = solve(instance0,silent=True)
            instance1 = instance.clone()
            instance1.x[ind].value = 1
            instance1.x[ind].fixed = True
            success1 = solve(instance1,silent=True)
            candidate = ([instance0] if success0 else []) + ([instance1] if success1 else [])
            score = candidate_score( candidate,instance.objective_value )
            if score>best_score:
                best_score,best_candidate,best_index = score,candidate, ind
            else:
                del(instance0,instance1)
        if best_index == (-1,-1): #shoud never happen...
            log.write_awaiting_answer("(failure, applying simple strategy)")
            return index_branching_candidates(instance,strategy = "simple")
        log.write_awaiting_answer("branching index: "+str(best_index)+", value "+str(round(instance.x[best_index].value,4))) 
        for inst in best_candidate:
            inst.branching_indexes.append(best_index)
        increment_depth_and_id(best_candidate,"index branching")
        return {"instances":best_candidate,"score":best_score, "method":"index branching"}

    raise NameError("enter a valid index_branching_strategy") 

def order_best_indexes(instance):
    # returns all the variables in instance sorted in increasing order of distance from 0.5 or 0.75
    return sorted(instance.x.keys(), key = lambda k : inf if k in instance.branching_indexes else min( abs(instance.x[k].value-0.5) , abs(instance.x[k].value - 0.75) ) )

def order_best_indexes2(instance):
     # returns all the variables in instance sorted in the following way:
     # we take the node that has the most edges connecting it > 0 and select its edge that is closest to 0.5, we then continue with the node that has the second most edges ...
    frac_nodes = sorted( instance.nodes, key = lambda node : 0 if node== 0 else sum( 1 if instance.x[max(node,node_out),min(node,node_out)].value > eps else 0 for node_out in instance.nodes if node_out != node), reverse = True )
    sorted_indexes = []
    for node in frac_nodes:
        sorted_indexes += sorted( [ (max(node,node_out),min(node,node_out)) for node_out in instance.nodes if node_out != node and node_out != 0], key = lambda k : abs(instance.x[k].value-0.5), reverse = False )[:1]
    return sorted_indexes

''' c++ constraint branching '''

def cpp_constraint_branching_candidate(instance,branching_strategy=constraint_branching_strategy):
    NoOfCustomers,Demand,CAP,NoOfEdges,EdgeX,EdgeHead,EdgeTail,QMin = cpp.convert_for_cvrpsep(instance)
    sets = cpp.cvrpsep.branching(NoOfCustomers,Demand,CAP,NoOfEdges,EdgeX,EdgeHead,EdgeTail)
    sets = order_best_sets(sets,instance)
    if sets == []:
        log.write_answer("no branching set were found")
        return {"instances":None,"score":None, "method":"constraint branching"}

    log.write_awaiting_answer("candidate sets: "+str(len(sets)))

    if branching_strategy == "simple":
        branch_set = sets[0]
        instance.branching_sets.append(branch_set)
        log.write_awaiting_answer(", best set: "+str(branch_set)+", demand: "+str(round(sum(instance.demands[i] for i in branch_set)/instance.capacity.value,4))+", coboundary: "+str(coboundary(branch_set,instance))) 
        candidate = branch_over_constraint(branch_set,instance)
        if candidate == []:
            return {"instances":None,"score":None, "method":"constraint branching"}
        increment_depth_and_id(candidate,"constraint branching")
        return {"instances":candidate,"score":candidate_score(candidate,instance.objective_value), "method":"constraint branching"}

    if branching_strategy == "exhaustive":
        branch_sets = sets[0:max_constraint_branching_candidates]
        best_candidate,best_set,best_score = [],0,0
        for branch_set in branch_sets:
            candidate = branch_over_constraint(branch_set,instance)
            score = candidate_score(candidate,instance.objective_value)
            if score > best_score:
                best_candidate,best_set,best_score = candidate,branch_set,score
            else:
                for inst in candidate:
                    del(inst)
        if best_candidate == []:
            log.write_answer("no branching over constraint could be found")
            return {"instances":None,"score":None, "method":"constraint branching"}
        for inst in best_candidate:
            inst.branching_sets.append(best_set)
        increment_depth_and_id(best_candidate,"constraint branching")
        log.write_awaiting_answer("branching set: "+str(best_set)+", demand: "+str(round(sum(instance.demands[i] for i in branch_set)/instance.capacity.value,4))+", coboundary: "+str(coboundary(best_set,instance)))
        return {"instances":best_candidate, "score":best_score, "method":"constraint branching"}
    
    raise NameError("enter a valid constraint_branching_strategy") 


def branch_over_constraint(branch_set,instance):
    # returns the new instances created from instance using branch_set by performing a constraint branching
    instance2 = instance.clone()
    instance4 = instance.clone()
    instance6 = instance.clone()
    n = int(coboundary(branch_set,instance))+1
    n = n if n%2==0 else n+1
    if constraint_branching_amount == 3:
        expression2 = sum( sum( instance2.x[max(i,j),min(i,j)] for j in instance2.nodes if not(j in branch_set) ) for i in branch_set )
        instance2.c_branch.add(expression2<=n-2)
        success2 = solve(instance2,silent=True)
        expression4 = sum( sum( instance4.x[max(i,j),min(i,j)] for j in instance4.nodes if not(j in branch_set) ) for i in branch_set )
        instance4.c_branch.add(expression4==n)
        success4 = solve(instance4,silent=True)
        expression6 = sum( sum( instance6.x[max(i,j),min(i,j)] for j in instance6.nodes if not(j in branch_set) ) for i in branch_set )
        instance6.c_branch.add(expression6>=n+2)
        success6 = solve(instance6,silent=True)
        return  ( [instance2] if success2 else []) + ([instance4] if success4 else []) + ([instance6] if success6 else [])
    if constraint_branching_amount == 2:
        expression2 = sum( sum( instance2.x[max(i,j),min(i,j)] for j in instance2.nodes if not(j in branch_set) ) for i in branch_set )
        if n-2 == 2:
            instance2.c_branch.add(expression2==n-2)
        else:
            instance2.c_branch.add(expression2<=n-2)
        success2 = solve(instance2,silent=True)
        expression4 = sum( sum( instance4.x[max(i,j),min(i,j)] for j in instance4.nodes if not(j in branch_set) ) for i in branch_set )
        instance4.c_branch.add(expression4>=n)
        success4 = solve(instance4,silent=True)
        return  ( ([instance2] if success2 else []) + ([instance4] if success4 else []) )


def order_best_sets(sets,instance, set_sorting_strategy=constraint_set_sorting_strategy):
    # returns a set of possible branching sets to be used for constraint branching
    # uses one of several set finding strategies and then sorts the sets found by using one of sever set sorting strategies

    if set_sorting_strategy == "distance to depot":
        sets = sorted(sets, key = lambda a_set : sorted(a_set, key = lambda node : instance.costs[(node,0)])[0]+sorted(a_set, key = lambda node : instance.costs[(node,0)])[1], reverse = True )
    elif set_sorting_strategy == "demand":
        sets = sorted( sets, key = lambda a_set : sum( instance.demands[node] for node in a_set), reverse = True)
    elif set_sorting_strategy == "distance demand mix":
        sets = sorted(sets, key = lambda a_set : (sorted(a_set, key = lambda node : instance.costs[(node,0)])[0]+sorted(a_set, key = lambda node : instance.costs[(node,0)])[1])/instance.max_cost + sum( instance.demands[node] for node in a_set)/instance.capacity, reverse = True )
    else:
        raise NameError("Enter a valid constraint set sorting strategy, entered strategy: "+str(set_sorting_strategy))
    return sets

coboundary = lambda a_set,instance: sum( sum( instance.x[max(node,node_out),min(node,node_out)].value for node_out in instance.nodes if not(node_out in a_set) ) for node in a_set )


''' GENEREAL SUPPORT '''
       
def increment_depth_and_id(instances,type):
    # updates the ids and depths of instances = [instance1,instance2...] 
    # update method depends on the type of branching that was performed
    id0 = instances[0].id
    depth = instances[0].depth
    if type == "constraint branching":
        n = 1
        for inst in instances: 
            inst.depth = depth+1
            c = inst.c_branch[list(inst.c_branch.keys())[-1]]
            if pyo.value(c.lower) == None:
                new_id = 'c'+str(n)+'_'+str(int(pyo.value(c.upper)))
            elif pyo.value(c.upper) == None:
                new_id = 'c'+str(n)+'_'+str(int(pyo.value(c.lower)))
            else: #necessarily an equality constraint
                new_id = 'c'+str(n)+'_'+str(int(pyo.value(c.upper)))
            inst.id = id0+[new_id]
            n += 1
    elif type == "index branching":
        for inst in instances: 
            inst.depth = depth+1
            inst.id = id0+["i"+str(inst.x[inst.branching_indexes[-1]].value)]
    elif type == "integer branching":
        for inst in instances: 
            inst.depth = depth+1
            inst.id = id0+["0"]
    else:
        raise NameError("Enter a valid branching type for increment depth and id function")

def list_to_string(ls):
	#returns more readible string of a list
	expr = ""
	for el in ls:
		expr += str(el)+"/"
	return expr[:-1]