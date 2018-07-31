# from numpy.random import random as rd
import threading
import pyomo.environ as pyo
from pyomo.opt import SolverFactory

''' globals '''
eps = 0.1 # int+/-epsilon is considered as int
max_global_time = 60*60*15 # global max CVRP solving time
precision = 0.00000001 # i noticed that 1+1 could equal 2-(1e-14) so yeah

''' logging options '''
logging = True #  write a complete log file
console_writing = True # write partial log in cmd
graphing = False # plot graphs of initial and final solutions
stats_monitoring = False # record real time stats throughout the CVRP process, they will be plotted at the end (slows down the process)

''' column generation options '''
max_column_generation_count = 100
max_column_generation_count_first_instance = 10 #300
max_unmoving_count = 15
max_unmoving_count_first_instance = 30
max_failure_count_cuts = 3 # maximum number of consecutive failure of all heuristics before column generation is aborted 
vals = [0.01,0.5,0.9]
unmoving_constraint_threshold  = 0.000001
max_inactivity_period = 50 #number of iteration while inactive for a constraint to be discarded

''' column generation heuristics options '''
flow_constraints = True
random_components = True
normal_components = True
max_flow_cuts = False
random_cuts = False
given_demand_components = True
attempt_freeze = False

''' branching options '''
max_depth = 20           #max number of branching
branching_type =   "constraint" # constraint index integer_fix
# constraint branching options
max_branching_search_time = 10 
max_branching_search_attempts = 10
branching_lower_bound = 2.5 # 2.75 
branching_upper_bound = 3.5 # 3
max_branching_set_length = 5 
constraint_branching_strategy = "distance_demand_mix" # distance_to_depot demand distance_demand_mix exhaustive
max_constraint_branching_candidates = 3
# index branching options
simple_index_branching = False 
max_index_branching_candidates = 5 #for complex index branching method
# search tree node selection options
node_selection =  "best_bound_first" # "best_bound_first" "depth_search"

''' instance editing options '''
max_time_for_shrinking = 5
max_size_for_shrinking = 4 #inclusive
max_constraints_for_lower_bound = 0
lower_bound_method = "objective" #"cbc"
dilate = lambda d : d #empty function far dilating node distances (deprecated)
vals_for_projection = [0,1]

''' solver options '''
opt = SolverFactory('cbc')
opt.options['threads'] = 8
if opt.name == 'cbc':
    opt.options['sec'] = 120 # cumulates all thread times
else:
    opt.options['timelimit'] = 38 # maximum of the thread times        #opt.options['ratioGap'] = 1e-1


''' simplification options '''
allow_single_node_roads = True
force_integer_to_depot = True
reduce_problem = True
initial_problem_reduction = 0.6
force_all_edges_integer = False

class modifiable_threshold():
	def __init__(self,initial_value):
		self.value = initial_value
		self.vals = vals
		
	def update(self,):
		# self.value = 0.9
		self.value = vals[(vals.index(self.value)+1)%len(vals)]

class safe_counter():
	def __init__(self,lock):
		self.val = 0
		self.lock = lock
	
	def increment(self):
		try:
			self.lock.acquire()
			self.val += 1
		finally:
			self.lock.release()
	
	def get(self):
		return self.val
		
	def get_and_increment(self):
		try : 
			self.lock.acquire()
			num = self.val
			self.val +=1
			return num
		finally :
			self.lock.release()

def get_safe_counter():
	lock = threading.Lock()
	return safe_counter(lock)
		
connected_threshold = modifiable_threshold(0.5)
id = get_safe_counter()      #global incrementing id to name the different instances 