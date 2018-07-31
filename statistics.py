from globals import *
from logging_cvrp import *
from time import time
from pyomo.environ import value
from logging_cvrp import global_time
import matplotlib.pyplot as plt

class stat_keeper():

	def __init__(self):
		self.max_nb_constr_lb = max_constraints_for_lower_bound
		self.max_nb_constr_colgen = max_column_generation_count
		self.colgen_total_time = stat("total time of column generation")
		self.colgen_nb_constraint = stat("number of constraints found during column generation")
		self.colgen_objective_delta = stat("change in objective function after column genereations")
		self.colgen_partial_objective_delta = stat("change in objective function after single iterations of column genereation")
		self.colgen_instance_solving_time = stat("LP solving time of a given instance")
		self.colgen_nb_active_constraints = stat("number of active constraints")
		self.branch_lb_time = stat("time of lower bound calculation")
		self.branch_total_time = stat("total time taken for branching")
		self.global_nb_cuts = stat("global number of cuts made in the search tree")
		self.global_nb_instance = stat("number of instances in the instance queue")
		self.global_total_time = stat("total time for cvrp solving")
		self.global_max_depth = stat("maximum depth reached")
		self.global_max_memory = stat("maximum amount of used memory")
		self.main_node_processing_time = stat("total time for the processing of a node")
		self.heur_random_components = stat("number of valid constraints found by the random components heuristic")
		self.heur_normal_components = stat("number of valid constraints found by the normal_components heuristic")
		self.heur_max_flow_cuts = stat("number of valid constraints found by the max_flow_cuts heuristic")
		self.heur_random_cuts = stat("number of valid constraints found by the random cuts heuristic")
		self.heur_given_demand_components = stat("number of valid constraints found by the given_demand_components heuristic")
		self.heur_given_demand_components_correct = stat("number of valid constraints found by the given_demand_components_correct heuristic")
		self.heur_random_components_slack = stat("slack of constraints found by the random components heuristic")
		self.heur_normal_components_slack = stat("slack of constraints found by the normal_components heuristic")
		self.heur_max_flow_cuts_slack = stat("slack of constraints found by the max_flow_cuts heuristic")
		self.heur_random_cuts_slack = stat("slack of constraints found by the random cuts heuristic")
		self.heur_given_demand_components_slack = stat("slack of constraints found by the given_demand_components heuristic")
		self.heur_given_demand_components_correct_slack = stat("slack of constraints found by the given_demand_components_correct heuristic")

		
	def init_globals(self,instance,instance_manager):
		self.n = instance.n.values
		self.k = instance.number_of_vehicles.values
		self.instance_manager = instance_manager

	def get_instance_manager_pars(self):
		return [self.instance_manager.branches_cut,self.instance_manager.length]
		
	def update(self,instance_pars,position,extra_val = None):
		#instance_manager_pars = [nb_cuts,len(queue)]
		#instance_pars = [objective_value,depth,len(c_cap)] 
		instance_pars[0] = value(instance_pars[0])
		instance_manager_pars = self.get_instance_manager_pars()
		if position == "start branch and cut":
			self.global_total_time.add_entry(time(),instance_pars)
			self.global_max_depth.add_max_entry(0,instance_pars)
			self.global_max_memory.add_max_entry(0,instance_pars)
			self.global_nb_instance.add_entry(instance_manager_pars[1],instance_pars)
		
		elif position == "start instance processing":
			self.main_node_processing_time.add_entry(time(),instance_pars)
		
		elif position == "start column generation":
			self.colgen_total_time.add_entry(time(),instance_pars)
			self.colgen_objective_delta.add_entry(instance_pars[0],instance_pars)
			
		elif position == "start iteration column generation":
			self.colgen_nb_constraint.add_entry(instance_pars[2],instance_pars)
			self.colgen_partial_objective_delta.add_entry(instance_pars[0],instance_pars,connected_threshold.value)
		
		elif position == "number of valid cuts normal components":
			self.heur_normal_components.add_entry(extra_val,instance_pars)

		elif position == "slack normal components":
			self.heur_normal_components_slack.add_entry(extra_val,instance_pars)


		elif position == "number of valid cuts random components":
    			self.heur_random_components.add_entry(extra_val,instance_pars)

		elif position == "slack random components":
			self.heur_random_components_slack.add_entry(extra_val,instance_pars)
		
		elif position == "number of valid cuts max flow cuts":
    			self.heur_max_flow_cuts.add_entry(extra_val,instance_pars)

		elif position == "slack max flow cuts":
			self.heur_max_flow_cuts_slack.add_entry(extra_val,instance_pars)
		
		elif position == "number of valid cuts random cuts":
    			self.heur_random_cuts.add_entry(extra_val,instance_pars)

		elif position == "slack random cuts":
			self.heur_random_cuts_slack.add_entry(extra_val,instance_pars)
		
		elif position == "number of valid cuts given demand components":
    			self.heur_given_demand_components.add_entry(extra_val,instance_pars)

		elif position == "slack given demand components":
			self.heur_given_demand_components_slack.add_entry(extra_val,instance_pars)
		
		elif position == "number of valid cuts given demand components correct":
    			self.heur_given_demand_components_correct.add_entry(extra_val,instance_pars)

		elif position == "slack given demand components correct":
			self.heur_given_demand_components_correct_slack.add_entry(extra_val,instance_pars)	
		
		elif position == "start instance solving":
			self.colgen_nb_active_constraints.add_entry(instance_pars[2],instance_pars)
			self.colgen_instance_solving_time.add_entry(time(),instance_pars)
		
		elif position == "end instance solving":
			self.colgen_instance_solving_time.wrap_entry(time())
			
		elif position == "end iteration column generation":
			self.colgen_nb_constraint.wrap_entry(instance_pars[2])
			self.colgen_partial_objective_delta.wrap_entry(instance_pars[0])
		
		elif position == "end column generation":
			self.colgen_total_time.wrap_entry(time())
			self.colgen_objective_delta.wrap_entry(instance_pars[0])
		
		elif position == "start branching":
			self.branch_total_time.add_entry(time(),instance_pars)
		
		elif position == "start lb solving":
			self.branch_lb_time.add_entry(time(),instance_pars)
		
		elif position == "end lb solving":
			self.branch_lb_time.wrap_entry(time())
			
		elif position == "end branching":
			self.branch_total_time.wrap_entry(time())
			
		elif position == "add instance to queue":
			self.global_nb_instance.add_entry(instance_manager_pars[1],instance_pars)
			self.global_nb_cuts.add_entry(instance_manager_pars[0],instance_pars)
		
		elif position == "end instance processing":
			self.main_node_processing_time.wrap_entry(time())
			
		elif position == "remove instance from queue":
			self.global_nb_instance.add_entry(instance_manager_pars[1],instance_pars)
			self.global_nb_cuts.add_entry(instance_manager_pars[0],instance_pars)
		
		elif position == "end branch and cut":
			self.global_total_time.wrap_entry(time())
			
		else:
			raise NameError("incorrect position :" + position)
		
	def curve(self,stat,par):
	#par must verbose descritption of a parameter in the stat.entries 
		X,Y = [],[]
		Z = []
		fig = plt.figure()
		index = index_of_par(par)
		for entry in stat.entries:
			Z.append((entry[0],entry[index]))
		Z = sorted(Z,key = lambda z : z[1])
		for y,x in Z:
			X.append(x)
			Y.append(y)
		i = 0
		while i<len(X):
			j = i+1
			mean = Y[i]
			while j<len(X) and X[i]==X[j]:
				mean += Y[j]
				j+=1
			X = X[:i+1]+X[j:]
			Y = Y[:i]+[mean/(j-i)]+Y[j:]	
			i += 1
		plt.title(stat.name + "\n in function of "+par)
		plt.xlabel(par)
		plt.ylabel(stat.name)
		plt.plot(X,Y)
		fig.savefig(log.name+"/"+stat.name+" in function of "+par+".png",dpi=fig.dpi*3)
		plt.close(fig)
		del(fig)
		# plt.show()
	
	def curve_dot(self,stat,par):
	#par must verbose descritption of a parameter in the stat.entries 
		X,Y = [],[]
		fig = plt.figure()
		index = index_of_par(par)
		for entry in stat.entries:
			Y.append(entry[0])
			X.append(entry[index])
		plt.title(stat.name + "\n in function of "+par)
		plt.xlabel(par)
		plt.ylabel(stat.name)
		plt.plot(X,Y,'ro')
		fig.savefig(log.name+"/"+stat.name+" in function of "+par+".png",dpi=fig.dpi*3)
		plt.close(fig)
		del(fig)
		# plt.show()
			
		
	def show(self):		
		self.curve(self.colgen_total_time,"time")
		self.curve(self.colgen_total_time,"depth")
		self.curve(self.colgen_total_time,"number of constraints")
		self.curve(self.colgen_objective_delta,"time")
		self.curve(self.colgen_objective_delta,"depth")
		self.curve(self.colgen_objective_delta,"number of constraints")
		self.curve(self.colgen_nb_constraint,"time")
		self.curve(self.colgen_nb_constraint,"depth")
		self.curve(self.colgen_nb_constraint,"number of constraints")
		self.curve(self.branch_lb_time,"time")
		self.curve(self.branch_lb_time,"depth")
		self.curve(self.branch_lb_time,"number of constraints")
		self.curve(self.colgen_instance_solving_time,"time")
		self.curve(self.colgen_instance_solving_time,"depth")
		self.curve(self.colgen_instance_solving_time,"number of constraints")
		self.curve(self.main_node_processing_time,"time")
		self.curve(self.main_node_processing_time,"depth")
		self.curve(self.main_node_processing_time,"number of constraints")
		self.curve(self.global_nb_instance,"time")
		self.curve(self.global_nb_cuts,"time")
		self.curve_dot(self.colgen_partial_objective_delta,"thresh")
		self.curve(self.colgen_nb_active_constraints,"time")
		self.curve(self.colgen_nb_active_constraints,"depth")
		self.curve(self.colgen_nb_active_constraints,"number of constraints")
		self.curve(self.heur_random_components,"time")
		self.curve(self.heur_normal_components,"time")
		self.curve(self.heur_max_flow_cuts,"time")
		self.curve(self.heur_random_cuts,"time")
		self.curve(self.heur_given_demand_components,"time")
		self.curve(self.heur_given_demand_components_correct,"time")
		self.curve(self.heur_random_components_slack,"time")
		self.curve(self.heur_normal_components_slack,"time")
		self.curve(self.heur_max_flow_cuts_slack,"time")
		self.curve(self.heur_random_cuts_slack,"time")
		self.curve(self.heur_given_demand_components_slack,"time")
		self.curve(self.heur_given_demand_components_correct_slack,"time")
		
		
		
class empty_stat_keeper:
	def __init__(self):
		return
		
	def init_globals(self,instance,instance_manager):
		return
		
	def update(self,instance_pars,position,extra_val=None):
		return
		
	def curve(self,stat,par):
		return
	
	def show(self):
		return
		
class stat:

	def __init__(self,name):
		self.name = name
		self.entries = []
		
	def add_entry(self,variable,instance_pars,thresh = None):
		if thresh == None:
			self.entries.append([variable,instance_pars[0],instance_pars[1],instance_pars[2],global_time()])
		else : 
			self.entries.append([variable,instance_pars[0],instance_pars[1],instance_pars[2],global_time(),thresh])
	
	def wrap_entry(self,variable):
		#delta type of entry
		self.entries[-1][0] = variable - self.entries[-1][0]
		
	def evolve_entry(self,variable):
		#evol type of entry
		self.entries[-1][0].append(variable)
		
	def add_max_entry(self,variable,instance_pars):
		self.entries = [variable,instance_pars[0],instance_pars[1],instance_pars[2],global_time()]
		
	def update_max_entry(self,variable):
		if self.entries[0]<variable:
			self.entries[0] = variable
	
def index_of_par(name):
	if name == "objective" :
		return 1
	if name == "depth" :
		return 2
	if name == "number of constraints" :
		return 3
	if name == "time" :
		return 4
	if name == "thresh" :
		return 5
	raise NameError("parameter is not valid")
	
if stats_monitoring:
	stats = stat_keeper()
else:
	stats = empty_stat_keeper()