# from numpy.random import random as rd
import threading
import pyomo.environ as pyo
from pyomo.opt import SolverFactory

''' globals '''
eps = 0.1 # int+/-epsilon is considered as int
max_global_time = 60*60*24*1 # global max CVRP solving time
precision = 0.00001 # 1+1 can be equal to 2-(1e-14) hence a manual precision

''' logging options '''
logging = True #  write a complete log file
console_writing = True # write partial log in cmd
graphing = True # plot graphs of initial and final solutions
graph_current_solution = True # plots graph of current instance solution (produces only one image that will be updated)
search_tree = "complete" #graph plot tree : complete end no 
queue_logging = True

''' model initialisation options '''
flow_constraints = False
max_time_upper_bound = 5 #120
dat_file_path = "C:\\Users\\GVKD1542\\Documents\\python\\v12\\"
vrp_file_path = "C:\\Users\\GVKD1542\\Documents\\python\\instances\\"

''' column generation options '''
column_input = "c++" # c++ - python
max_column_generation_count = 250
max_column_generation_count_first_instance = 500
max_unmoving_count = 2
max_unmoving_count_first_instance = 5
max_failure_count_cuts = 2 # maximum number of consecutive failure of all heuristics before column generation is aborted 
#vals = [0.01,0.5,0.9]
unmoving_constraint_threshold  = 0.000001
max_inactivity_period = 25 #number of iteration while inactive for a constraint to be discarded
disable_constraint_deactivation = False # must be put to true if we use integer variables (during branching or in simplifications)

''' column generation heuristics options (FOR PYTHON INPUT)'''
random_components = True #True
normal_components = True
max_flow_cuts = True
random_cuts = True
given_demand_components = True
random_depot_fixing = False
random_shrink_cuts = True

''' branching options '''
# search tree node selection options
node_selection =  "best_bound_first" # best_bound_first - depth_search - best_bound_frist_on_smallest_depth
max_depth = 40          #max number of branching
cpp_constraint_branching = True
index_branching = False
integer_branching = False 
# constraint branching options
constraint_branching_amount = 3 # 2 - 3 : the number of instances that are created by a single branching
constraint_set_sorting_strategy = "distance demand mix"  # distance to depot - demand  - distance demand mix 
constraint_branching_strategy = "exhaustive" # exhaustive - simple
max_constraint_branching_candidates = 5
# index branching options
index_branching_strategy = "exhaustive" # exhaustive - simple
max_index_branching_candidates = 15 #for complex index branching method
#integer branching options
integer_branching_strategy = "exhaustive" # exhaustive - simple


''' instance editing options '''
max_constraints_for_lower_bound = 0
lower_bound_method = "objective" #"cbc"

''' solver options '''
opt = SolverFactory('cbc') # cbc glpk
opt.options['threads'] = 8
'''
if opt.name == 'cbc':
    opt.options['sec'] = 200 # cumulates all thread times
else:
    opt.options['timelimit'] = 38 # maximum of the thread times        #opt.options['ratioGap'] = 1e-1
'''

''' simplification options '''
allow_single_node_roads = False
force_integer_to_depot = False #True
reduce_problem = True
initial_problem_reduction = 0.8
force_all_edges_integer = False
force_closest_neighboors_integer = False #True VERY USEFUL FOR N<50 --> this simplification is further detailed in the documentation
relative_distance_closest_neighboor = 0.15
last_push_integer = False
last_push_integer_thresh = 0.10
integer_simplifications = allow_single_node_roads or force_integer_to_depot or force_all_edges_integer or (force_closest_neighboors_integer and relative_distance_closest_neighboor>0)



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
		

'''


class modifiable_threshold():
	def __init__(self,initial_value):
		self.value = initial_value
		self.vals = vals
		
	def update(self,):
		# self.value = 0.9
		self.value = vals[(vals.index(self.value)+1)%len(vals)]

connected_threshold = modifiable_threshold(0.5)
id = get_safe_counter()      #global incrementing id to name the different instances 

'''