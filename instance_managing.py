from globals import id,opt,eps,node_selection,force_integer_to_depot
from statistics import stats
from logging_cvrp import log
import pyomo.environ as pyo
import cvrpGoogleOpt as google       #upper bound function
from cap_constraint import find_components



dist = lambda x,y : pow(( (x[0]-y[0])**2 + (x[1]-y[1])**2 ),0.5)

class instance_manager():
	#CAREFUL class is not thread safe!!
	def __init__(self,instance):
		self.queue = []
		self.upper_bound = google.upper_bound(instance,instance.costs)
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
		stats.update([instance.objective_value,instance.depth,len(instance.c_cap)],"add instance to queue")
	
	def pop(self):
		while self.length>0:
			if node_selection == "best_bound_first":
				instance = self.queue.pop(0)
			elif node_selection == "depth_search":
				instance = self.queue.pop(-1)
			else:
				raise NameError("enter a valide node_selection stategy")
			self.length -= 1
			stats.update([instance.objective_value,instance.depth,len(instance.c_cap)],"remove instance from queue")
			#we must verify that the condition still holds because the upper_bound may have changed since time of adding
			if instance.lower_bound < self.upper_bound and instance.x[1,0].value!=None :
				return instance
			else:
				self.branches_cut += 1
		return None
	
	def record_feasible_integer_solution(self,instance):
		if instance.objective_value <  self.upper_bound : 
			self.upper_bound = instance.objective_value
			integerize_solution(instance)
			self.best_feasible_integer_solution = instance
			
	
	def record_partial_solution(self,instance):
		index = 0
		while index<self.length and self.queue[index].lower_bound<instance.lower_bound :
			index +=1
		if index == self.length:
			self.partial_solution_recorded.append(instance)
		else :
			self.partial_solution_recorded.insert(index,instance)
			

def solve(instance,silent=False):
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
	stats.update([instance.objective_value,instance.depth,number_of_active_constraints(instance.c_cap)],"start instance solving")
	log.write_awaiting_answer("active constraints: "+str(number_of_active_constraints(instance.c_cap)))
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
	stats.update([instance.objective_value,instance.depth,number_of_active_constraints(instance.c_cap)],"end instance solving")
	log.write_awaiting_answer("solved in",timed=True)
	return True

def set_lower_bound(instance):
	log.write_awaiting_answer("lower bound is now being calculated...",instance.id)
	stats.update([instance.objective_value,instance.depth,len(instance.c_cap)],"start lb solving")
	instance.lower_bound = instance.objective_value
	'''instance2 = instance.clone()
	for var in instance2.x.values():
		var.domain = pyo.NonNegativeIntegers
	for i in range(1,len(instance.c_cap)-max_constraints_for_lower_bound):
		instance2.c_cap[i].deactivate()	
	res = opt.solve(instance2)
	instance.lower_bound = max(pyo.value(instance2.objective),instance.objective_value)
	del(instance2)'''
	stats.update([instance.objective_value,instance.depth,len(instance.c_cap)],"end lb solving")
	log.write_timed("DONE")


###############
### SUPPORT ###
###############

def reduce_problem(instance,threshold):
	for i,j in [(a,b) for a in range(instance.n.value) for b in range(a)]:
		if not ((i,j) in instance.branching_indexes):
			if instance.costs[(i,j)] > threshold*instance.max_cost:
				instance.x[(i,j)].value = 0
				instance.x[(i,j)].fixed = True
			else:
				instance.x[(i,j)].fixed = False
	instance.reduction = threshold

def reduce_problem_neighboors(instance,amount):
	#reduces problem by disabling links between every node and it's neighboors
	#always leaves the depot as neighboor and leaves also the amount closest neighboors (graph is not oriented so there will possibly more than amount neighboors for some nodes)
	for i,j in [(a,b) for a in range(1,instance.n.value) for b in range(1,a)]:
		instance.x[i,j].value = 0
		instance.x[i,j].fixed = True		
	for i in range(1,instance.n.value):
		for j in sorted([k for k in range(1,instance.n.value)],key = lambda k : instance.costs[i,k])[:amount]:
			instance.x[i,j].fixed = False
			instance.x[i,j].value = 0
	

def number_of_active_constraints(clist):
	count = 0
	for k in clist.keys():
		if clist[k].active :
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
	comps = find_components(instance.x,instance.n.value)
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

def dilate_problem(instance,factor=2):
	for i in range(instance.n.value):
		for j in range(i):
			instance.costs[(i,j)] = pow(instance.costs[(i,j)],factor)

def normalize_demands(instance,max=1):
	max_demand = 0
	for i in instance.nodes:
		if instance.demands[i]>max_demand:
			max_demand = instance.demands[i]
	for i in instance.nodes:
		instance.demands[i]/=max_demand
		instance.demands[i]*=max

def normalize_costs(instance,max=1):
	max_costs = 0
	for i in range(instance.n.value):
		for j in range(i):
			if instance.costs[(i,j)]>max_costs:
				max_costs = instance.costs[(i,j)]
	for i in range(instance.n.value):
		for j in range(i):
			instance.costs[(i,j)]/=max_costs
			instance.costs[(i,j)]*=max

def fix_edges_to_depot(instance):
	for i in range(1,instance.n.value):
		instance.x[i,0].domain = pyo.NonNegativeIntegers

def unfix_edges_to_depot(instance):
	for i in range(1,instance.n.value):
		if not((i,0) in instance.branching_indexes or (0,i) in instance.branching_indexes):
			instance.x[i,0].domain = pyo.NonNegativeReals

def fix_all_edges(instance):
	for i in range(instance.n.value):
		for j in range(i):
			instance.x[i,j].domain = pyo.NonNegativeIntegers

def unfix_edges(instance):
	for i in range(instance.n.value):
		for j in range(i):
			if not((i,j) in instance.branching_indexes) and not((j,i) in instance.branching_indexes) and ( not( 0 in [i,j] ) if fix_edges_to_depot else True ):
				instance.x[i,j].domain = pyo.NonNegativeReals

def freeze_integer_edges(instance):
	frozen = []
	integerized = []
	for i in range(instance.n.value):
		for j in range(i):
			if (i,j) in instance.branching_indexes or (j,i) in instance.branching_indexes or (  0 in [i,j]  if fix_edges_to_depot else False ):
				continue
			if is_int(instance.x[i,j]):
				instance.x[i,j].fixed = True
				frozen.append((i,j))
			else:
				instance.x[i,j].domain = pyo.NonNegativeIntegers
				integerized.append((i,j))
	return frozen,integerized

def unfreeze_integer_edges(instance,frozen,integerized):
	for i,j in frozen:
		instance.x[i,j].fixed = False
	for i,j in integerized:
		instance.x[i,j].domain = pyo.NonNegativeReals