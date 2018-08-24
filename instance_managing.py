from globals import max_time_upper_bound,opt,eps,node_selection,force_integer_to_depot, relative_distance_closest_neighboor
from logging_cvrp import log
import pyomo.environ as pyo
import cvrpGoogleOpt as google       #upper bound function
from cap_constraint import find_integer_components



dist = lambda x,y : pow(( (x[0]-y[0])**2 + (x[1]-y[1])**2 ),0.5)

class instance_manager():
	# class for managing the instance queue from the search tree
	def __init__(self,instance):
		self.queue = []
		ub,roads = google.solve_distances(instance,instance.costs,max_time_upper_bound)
		self.upper_bound = ub
		self.length = 0
		self.best_feasible_integer_solution = create_instance_from_roads(instance,roads)
		self.branches_cut = 0
		self.partial_solution_recorded = []
		self.total_length = 0

	def add(self,instance):
    	# adds instance only if its lower_bound <= the upper bound of the problem
		# orders the queue in increasing ordere of lower_bound
		if instance.lower_bound <= self.upper_bound:
			self.queue.append(instance)
			self.queue = sorted(self.queue,key = lambda x : x.lower_bound)
			self.length += 1
			self.total_length += 1
		else:
			self.branches_cut += 1
	
	def pop(self):
    	# pops an instance from the search tree queue, order of popping depends on node_selection 
		while self.length>0:
			if node_selection == "best_bound_first":
				instance = self.queue.pop(0)
			elif node_selection == "depth_search":
				instance = self.queue.pop(-1)
			elif node_selection == "best_bound_frist_on_smallest_depth":
				min_depth,instance = min(self.queue,key = lambda instance: instance.depth).depth,None
				for inst in self.queue:
					if inst.depth == min_depth:
						instance = inst
						break
				if instance == None:
					raise Exception("Implementation bug in finding best bound first on smallest depth instance")
				self.queue.remove(instance)
			else:
				raise NameError("Enter a valide node_selection stategy")
			self.length -= 1
			#we must verify that the condition still holds because the upper_bound may have changed since time of adding
			if instance.lower_bound < self.upper_bound and instance.x[1,0].value!=None :
				return instance
			else:
				self.branches_cut += 1
		return None
	
	def record_feasible_integer_solution(self,instance):
    	# places an instance that has been verified as integer as feasible in the corresponding attribute of the instance_manager
		if instance.objective_value <  self.upper_bound : 
			self.upper_bound = instance.objective_value
			integerize_solution(instance)
			self.best_feasible_integer_solution = instance
			
	
	def record_partial_solution(self,instance):
    	# adds a 'partial' solution to the corresponding list of the instance_manager. This is used to retrieve instances for debugging
		index = 0
		while index<self.length and self.queue[index].lower_bound<instance.lower_bound :
			index +=1
		if index == self.length:
			self.partial_solution_recorded.append(instance)
		else :
			self.partial_solution_recorded.insert(index,instance)
			

def solve(instance,silent=False):
    # solves an instance and updates its attribute objective_value
	# returns the boolean describing the success of the solver
	# takes into account the possible problem reduction (reduce problem in globals) an relaxes the reduction if no solution is found
	if silent:
		res = opt.solve(instance)
		if res.Solver[0]['Termination condition'].key=='infeasible':
			if instance.reduction >= 1:
				return False
			else:
				reduce_problem(instance,min(instance.reduction+0.2,1))
				solve(instance,silent=True)
		instance.objective_value = pyo.value(instance.objective)
		return True
	#else:
	log.write_awaiting_answer("active constraints: "+str(number_of_active_constraints(instance)))
	res = opt.solve(instance)
	if res.Solver[0]['Termination condition'].key=='infeasible':
		if instance.reduction >= 1:
			log.write_timed("no solution found to LP problem",instance.id)
			return False
		else:
			log.write_timed("no solution found to LP problem with reduction at "+str(instance.reduction)+" going to + 0.2",instance.id)
			reduce_problem(instance,min(instance.reduction+0.2,1))
			solve(instance)
	instance.objective_value = pyo.value(instance.objective)
	log.write_awaiting_answer("solved in",timed=True)
	return True


def set_lower_bound(instance):
    # sets the lower bound of an instance
	# for now this is set to attribute simply the objective_value 
	#log.write_awaiting_answer("lower bound is now being calculated...",instance.id)
	instance.lower_bound = instance.objective_value
	'''instance2 = instance.clone()
	for var in instance2.x.values():
		var.domain = pyo.NonNegativeIntegers
	for i in range(1,len(instance.c_cap)-max_constraints_for_lower_bound):
		instance2.c_cap[i].deactivate()	
	res = opt.solve(instance2)
	instance.lower_bound = max(pyo.value(instance2.objective),instance.objective_value)
	del(instance2)'''
	#log.write_timed("DONE")

######################
### SIMPLIFICATION ###
######################

def reduce_problem(instance,threshold):
    # eliminates variables based on the their cost (cost of the corresponding edge)
	# if cost >= threshold, the edge will be fixed at 0
	for i,j in [(a,b) for a in range(instance.n.value) for b in range(a)]:
		if not ((i,j) in instance.branching_indexes):
			if instance.costs[(i,j)] > threshold*instance.max_cost:
				instance.x[(i,j)].value = 0
				instance.x[(i,j)].fixed = True
			else:
				instance.x[(i,j)].fixed = False
	instance.reduction = threshold	

def integerize_edges_to_depot(instance):
	# force the edges connecting every node to the depot to be integer
	for i in range(1,instance.n.value):
		instance.x[i,0].domain = pyo.NonNegativeIntegers

def unintegerize_edges_to_depot(instance):
	# force the edges connecting every node to the depot to be float
	for i in range(1,instance.n.value):
		instance.x[i,0].domain = pyo.NonNegativeReals

def integerize_all_edges(instance):
	# force all the edges to be integer
	for i in range(instance.n.value):
		for j in range(i):
			instance.x[i,j].domain = pyo.NonNegativeIntegers

def integerize_edges(instance,max_dist=None,smart = False):
    # force edges to be integer if their cost >= max_dist 
	# if no max_dist is given, it will use the maximum distance of the instance (defined as the maximum distance between to nodes of the instance)
	# returns the list of integers that have been processed, this can be used to undo this action
	max_dist = max_dist*instance.max_cost if max_dist != None else instance.max_cost*relative_distance_closest_neighboor
	integerized = []
	for i,j in instance.x.keys():
		if (i,j) in instance.branching_indexes or (  0 in [i,j]  if force_integer_to_depot else False ):
			continue
		if instance.costs[i,j] > max_dist:
			continue
		if smart and max( sum( 1 if instance.x[max(i,k),min(i,k)].value>eps else 0 for k in instance.nodes if k!=i ), sum( 1 if instance.x[max(j,k),min(j,k)].value>eps else 0 for k in instance.nodes if k!=j ) ) < 3:
			continue
		instance.x[i,j].domain = pyo.NonNegativeIntegers
		integerized.append((i,j))
	return integerized

def unintegerize_edges(instance,integerized=None):
    # force edges to be float 
	# if integerized is given, only contained indexes will be processed
	# else it will process all nodes for which cost >= maximum distance of the instance (defined as the maximum distance between to nodes of the instance)
	if integerized != None:
		for i,j in integerized:
			instance.x[i,j].domain = pyo.NonNegativeReals
	else:
		for i in range(instance.n.value):
			for j in range(i):
				if not((i,j) in instance.branching_indexes) and ( not( 0 in [i,j] ) if force_integer_to_depot else True ):
					instance.x[i,j].domain = pyo.NonNegativeReals

def fix_specific_edges(instance,instance_manager,thresh=eps*3):
    # force specific nodes to be at 1
	# will apply to nodes such that their value >= 1 - thresh
	roads = find_integer_components(instance.x,instance.n.value)
	for k in instance.x.keys():
		if instance.x[k].value >= 1 - eps and not(instance.x[k].fixed):
			i,j = k
			new_road = [i,j]
			for r in roads:
				if r[0]==i or r[-1]==i:
					new_road += r
					new_road.remove(i) #duplicate i
				if r[0]==j or r[-1]==j:
					new_road += r
					new_road.remove(j) #duplicate i
			if sum( instance.demands[n] for n in new_road ) <= instance.capacity.value:
				instance.x[k].value = 1
				instance.x[k].fixed = True
	solve(instance,silent=True)
	set_lower_bound(instance)
	instance.depth += 1
	instance.id.append("0")
	instance_manager.add(instance)

###############
### SUPPORT ###
###############

def number_of_active_constraints(instance):
	# returns the number of constraints that are active in a constraint list clist
	count = 0
	for c,val in list(instance.dual.items()):
		parent,index = c.parent_component().name,c.index()
		#if parent == 'c_deg':
		#	continue
		if c.active :
			count += 1
	return count

def solution_is_integer(instance):
    #returns bool describing wheter instance.x only has integers 
	# with slack epsilon (eps)
	for b in [(i,j) for i in range(instance.n.value) for j in range(i)]:
		val = instance.x[b].value
		expr = abs(round(val)-val)>eps
		if expr:
			return False
	return True

def integerize_solution(instance):
	#rounds solution values of instance that has been considered integer enough
	for b in instance.x.keys():
		instance.x[b].value = round(instance.x[b].value)

def integer_percent(instance):
	# returns float describing the percentage of variables that are integer
	count = 0
	pos = 0
	for b in instance.x.keys():
		count += 1
		if is_int(instance.x[b]):
			pos += 1
	return round(pos/count*100,2)

def is_int(x):
	#returns bool describing whether pyo.Par/pyo.Var x is integer 
	# with slack epsilon (eps)
	return abs(round(x.value)-x.value)<eps

def print_solution_routes(instance):
    # returns a string containing a verbose description of the routes that are contained in an instance solution
	comps = find_integer_components(instance.x,instance.n.value)
	message = "\n+++++ routes found : \n\n"
	total_distance = 0
	for i in range(len(comps)):
		distance = 0
		demand = 0
		message += "road "+str(i)+": "
		old_node = 0
		message += str(old_node) + " -> "
		demand += instance.demands[old_node]
		for node in comps[i]:
			message += str(node) + " -> "
			demand += instance.demands[node]
			distance += instance.costs[old_node,node]*instance.x[max(old_node,node),min(old_node,node)].value
		message = message[:-4]
		message += "\ndistance of route is "+str(distance)+" and demand served during route is "+str(demand)
		total_distance += distance
		message += "\n\n" 
	message += "total distance of route is "+str(total_distance)
	return message

def create_instance_from_roads(instance,roads):
	instance_gg = instance.clone()
	for k in instance_gg.x.keys():
		instance_gg.x[k].value = 0
	for road in roads:
		for i in range(len(road)-1):
			instance_gg.x[max(road[i],road[i+1]),min(road[i],road[i+1])].value = 1
	instance_gg.objective_value = pyo.value(instance_gg.objective)
	return instance_gg
