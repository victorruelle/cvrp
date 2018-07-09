"""Capacitated Vehicle Routing Problem"""
from __future__ import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from math import sqrt
import time
from globals import *
from support import pprint

#calculating upper_bound of problem

###########################
# Problem Data Definition #
###########################
class Vehicle():
	"""Stores the property of a vehicle"""
	def __init__(self,cap):
		"""Initializes the vehicle properties"""
		self._capacity = cap

	@property
	def capacity(self):
		"""Gets vehicle capacity"""
		return self._capacity

class DataProblem():
	"""Stores the data for the problem"""
	def __init__(self,instance_and_locations,n=-1,k=-1):
		if type(instance_and_locations)==int:
			dimension, capacity, number_of_vehicles, locations, demands = import_data(n,k)
			self._vehicle = Vehicle(capacity)
			self._num_vehicles = number_of_vehicles
			self._locations = locations
			self._depot = 0
			self._demands = demands

		else:
			self._vehicle = Vehicle(instance_and_locations[0].capacity.value)
			self._num_vehicles = instance_and_locations[0].number_of_vehicles.value
			self._locations = instance_and_locations[1]
			self._depot = 0
			self._demands = instance_and_locations[0].demands

	@property
	def vehicle(self):
		"""Gets a vehicle"""
		return self._vehicle

	@property
	def num_vehicles(self):
		"""Gets number of vehicles"""
		return self._num_vehicles

	@property
	def locations(self):
		"""Gets locations"""
		return self._locations

	@property
	def num_locations(self):
		"""Gets number of locations"""
		return len(self.locations)

	@property
	def depot(self):
		"""Gets depot location index"""
		return self._depot

	@property
	def demands(self):
		"""Gets demands at each location"""
		return self._demands

#######################
# Problem Constraints #
#######################

def euclidian_distance(position_1, position_2):
	return sqrt(pow(position_1[0]-position_2[0],2) + pow(position_1[1]-position_2[1],2))

class CreateDistanceEvaluator(object): # pylint: disable=too-few-public-methods
	"""Creates callback to return distance between points."""
	def __init__(self, data):
		"""Initializes the distance matrix."""
		self._distances = {}

		# precompute distance between location to have distance callback in O(1)
		for from_node in xrange(data.num_locations):
			self._distances[from_node] = {}
			for to_node in xrange(data.num_locations):
				if from_node == to_node:
					self._distances[from_node][to_node] = 0
				else:
					self._distances[from_node][to_node] = (
						euclidian_distance(
							data.locations[from_node],
							data.locations[to_node]))

	def distance_evaluator(self, from_node, to_node):
		"""Returns the manhattan distance between the two nodes"""
		return self._distances[from_node][to_node]

class CreateDemandEvaluator(object): # pylint: disable=too-few-public-methods
	"""Creates callback to get demands at each location."""
	def __init__(self, data):
		"""Initializes the demand array."""
		self._demands = data.demands

	def demand_evaluator(self, from_node, to_node):
		"""Returns the demand of the current node"""
		del to_node
		return self._demands[from_node]

def add_capacity_constraints(routing, data, demand_evaluator):
	"""Adds capacity constraint"""
	capacity = "Capacity"
	routing.AddDimension(
		demand_evaluator,
		0, # null capacity slack
		data.vehicle.capacity, # vehicle maximum capacity
		True, # start cumul to zero
		capacity)

		
def calculate_distance(data,routing,assignment):
	total_dist = 0
	for vehicle_id in xrange(data.num_vehicles):
		index = routing.Start(vehicle_id)
		route_dist = 0
		while not routing.IsEnd(index):
			node_index = routing.IndexToNode(index)
			next_node_index = routing.IndexToNode(assignment.Value(routing.NextVar(index)))
			route_dist += euclidian_distance(data.locations[node_index],data.locations[next_node_index])
			index = assignment.Value(routing.NextVar(index))
		node_index = routing.IndexToNode(index)
		total_dist += route_dist
	return total_dist
	
'''
###########
# Printer #
###########
class ConsolePrinter():
	"""Print solution to console"""
	def __init__(self, data, routing, assignment):
		"""Initializes the printer"""
		self._data = data
		self._routing = routing
		self._assignment = assignment

	@property
	def data(self):
		"""Gets problem data"""
		return self._data

	@property
	def routing(self):
		"""Gets routing model"""
		return self._routing

	@property
	def assignment(self):
		"""Gets routing model"""
		return self._assignment

	def print(self):
		"""Prints assignment on console"""
		# Inspect solution.
		total_dist = 0
		for vehicle_id in xrange(self.data.num_vehicles):
			index = self.routing.Start(vehicle_id)
			#plan_output = 'Route for vehicle {0}:\n'.format(vehicle_id)
			route_dist = 0
			route_load = 0
			while not self.routing.IsEnd(index):
				node_index = self.routing.IndexToNode(index)
				next_node_index = self.routing.IndexToNode(
					self.assignment.Value(self.routing.NextVar(index)))
				route_dist += euclidian_distance(
					self.data.locations[node_index],
					self.data.locations[next_node_index])
				route_load += self.data.demands[node_index]
				#plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
				index = self.assignment.Value(self.routing.NextVar(index))

			node_index = self.routing.IndexToNode(index)
			total_dist += route_dist
			#plan_output += ' {0} Load({1})\n'.format(node_index, route_load)
			#plan_output += 'Distance of the route: {0}m\n'.format(route_dist)
			#plan_output += 'Load of the route: {0}\n'.format(route_load)
			#print(plan_output)
		print('Total Distance of all routes: {0}m'.format(total_dist))
'''
def import_data(n,k):
	file = "M-n"+str(n)+"-k"+str(k)+".vrp"
	with open("C:\\Users\\GVKD1542\\Documents\\python\\Vrp-Set-X\\X\\"+file,"r") as f:
		"""reading .dat data file"""
		lines = f.readlines()	
		#values are seperated either by space or indent
		sep = ' ' if ' ' in lines[7] else '\t'		
		#number of vehicles
		line = lines[0]
		line = line.split(":")
		line = line[1]
		line = line.split("-")
		word = line[2]
		number_of_vehicles = int(sanitze(word[1:]))			
		#dimension
		line = lines[3]
		line = line.split(":")
		dimension = int(sanitze(line[1]))			
		#capacity
		line = lines[5]
		line = line.split(":")
		capacity = int(sanitze(line[1]))			
		locations_start_index = 7
		demands_start_index = locations_start_index + dimension + 1 #skip over title			
		#locations
		locations = {}
		for i in range(dimension):
			line = lines[locations_start_index+i]
			line = line.split(sep)
			try:
				n,i,j = int(sanitze(line[0]))-1,int(sanitze(line[1])),int(sanitze(line[2]))#dans les fichiers vrp, l'indexation commence à 1
			except IndexError:
				print("mauvaise indentation dans locations avec : ")
				print(line)
				print(n)
				print(i)
				print(j)
				print(len(line))
			locations[n] = i,j
		
		#demands
		demands = []
		for i in range(dimension):
			line = lines[demands_start_index+i]
			line = line.split(sep)
			try :
				demands.append(int(sanitze(line[1])))
			except ValueError:
				print("error while adding demands with : ")
				print(demands_start_index+i)
				print(line)
				print(line[1])
				print(sanitze(line[1]))
	return dimension, capacity, number_of_vehicles, locations, demands

def sanitze(str):
	chars = ['\t','\n',";"]
	for c in chars:
		while c in str:
			i = str.index(c)
			try :
				str = str[:i]+str[i+len(c):]
			except IndexError:
				print("sanitze out of range with : ")
				print(str + "c = " + c + " i = "+ str(i) + "len = " + str(len(c)) )
	return str

########
# Main #
########


def upper_bound(instance,locations):
	"""Entry point of the program"""
	# Instantiate the data problem.
	data = DataProblem((instance,locations))
	model_parameters = pywrapcp.RoutingModel.DefaultModelParameters()
	# Create Routing Model
	routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot,model_parameters)
	# Define weight of each edge
	distance_evaluator = CreateDistanceEvaluator(data).distance_evaluator
	routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
	# Add Capacity constraint
	demand_evaluator = CreateDemandEvaluator(data).demand_evaluator
	add_capacity_constraints(routing, data, demand_evaluator)
	# Setting first solution heuristic (cheapest addition).
	search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()	
	search_parameters.first_solution_strategy = (
		routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
	# Disabling Large Neighborhood Search, (this is the default behaviour)
	search_parameters.time_limit_ms = 3000
	search_parameters.local_search_operators.use_path_lns = False
	search_parameters.local_search_operators.use_inactive_lns = False
	# Routing: forbids use of TSPOpt neighborhood,
	search_parameters.local_search_operators.use_tsp_opt = False	
	search_parameters.use_light_propagation = False
	# Solve the problem.
	assignment = routing.SolveWithParameters(search_parameters)
	t = type(empty())
	count = 0
	while type(assignment)==t:
		data._num_vehicles += 1 
		count +=1
		routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot,model_parameters)
		routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
		add_capacity_constraints(routing, data, demand_evaluator)
		assignment = routing.SolveWithParameters(search_parameters) 
	return calculate_distance(data, routing, assignment)

def lower_bound(instance,locations):
	"""Entry point of the program"""
	# Instantiate the data problem.
	data = DataProblem((instance,locations))
	model_parameters = pywrapcp.RoutingModel.DefaultModelParameters()
	# Create Routing Model
	routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot,model_parameters)
	# Define weight of each edge
	distance_evaluator = CreateDistanceEvaluator(data).distance_evaluator
	routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
	# Add Capacity constraint
	demand_evaluator = CreateDemandEvaluator(data).demand_evaluator
	# add_capacity_constraints(routing, data, demand_evaluator)
	# Setting first solution heuristic (cheapest addition).
	search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()	
	search_parameters.first_solution_strategy = (
		routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
	# Disabling Large Neighborhood Search, (this is the default behaviour)
	search_parameters.time_limit_ms = 3000
	search_parameters.local_search_operators.use_path_lns = False
	search_parameters.local_search_operators.use_inactive_lns = False
	# Routing: forbids use of TSPOpt neighborhood,
	search_parameters.local_search_operators.use_tsp_opt = False	
	search_parameters.use_light_propagation = False
	# Solve the problem.
	assignment = routing.SolveWithParameters(search_parameters)
	t = type(empty())
	count = 0
	while type(assignment)==t:
		data._num_vehicles += 1 
		count +=1
		routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot,model_parameters)
		routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
		add_capacity_constraints(routing, data, demand_evaluator)
		assignment = routing.SolveWithParameters(search_parameters) 
	return calculate_distance(data, routing, assignment)


def main(n,k):
	"""Entry point of the program"""
	# Instantiate the data problem.
	data = DataProblem(-1,n,k)
	start = time.time()
	model_parameters = pywrapcp.RoutingModel.DefaultModelParameters()
	# Create Routing Model
	routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot,model_parameters)
	# Define weight of each edge
	distance_evaluator = CreateDistanceEvaluator(data).distance_evaluator
	routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
	# Add Capacity constraint
	demand_evaluator = CreateDemandEvaluator(data).demand_evaluator
	add_capacity_constraints(routing, data, demand_evaluator)
	# Setting first solution heuristic (cheapest addition).
	search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()	
	search_parameters.first_solution_strategy = (
		routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
	search_parameters.local_search_metaheuristic = (
		routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
	# Disabling Large Neighborhood Search, (this is the default behaviour)
	search_parameters.local_search_operators.use_path_lns = False
	search_parameters.local_search_operators.use_inactive_lns = False
	# Routing: forbids use of TSPOpt neighborhood,
	search_parameters.local_search_operators.use_tsp_opt = False	
	search_parameters.use_light_propagation = False
	# Solve the problem.
	search_parameters.time_limit_ms = 1000*1000
	# search_parameters.solution_limit = 1
	assignment = routing.SolveWithParameters(search_parameters) 
	t = type(empty())
	count = 0
	while type(assignment)==t:
		data._num_vehicles += 1 
		count +=1
		routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot,model_parameters)
		routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)
		add_capacity_constraints(routing, data, demand_evaluator)
		assignment = routing.SolveWithParameters(search_parameters) 
	print(calculate_distance(data, routing, assignment))

	pprint("solved upper_bound estimate after adding "+str(count)+" vehicles to solve problem")
	# printer = ConsolePrinter(data, routing, assignment)
	# printer.print()
	# print("elapsed time : " + str(round(time.time()-start,2)))

def empty():
	return 

if __name__ == '__main__':
	# list = [(101,25),(125,30),(190,8),(110,13),(106,14)]
	# for n,k in list:
		# main(n,k)
		# in_name = "X-n"+str(n)+"-k"+str(k)+".sol"
		# with open("C:\\Users\\GVKD1542\\Documents\\python\\Vrp-Set-X\\X\\"+in_name,"r") as f:
			# lines = f.readlines()
			# val = lines[0]
		# print("optimal : "+val)
	main(121,7)