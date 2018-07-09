# from numpy.random import random as rd
eps = 0.1
debug = False
max_column_generation_count = 50
max_depth = 10                  #max number of branching 
max_time_for_shrinking = 5
max_size_for_shrinking = 4 #inclusive
max_global_time = 1000
vals = [0.01,0.5,0.9]

class modifiable_global():
	def __init__(self,initial_value):
		self.value = initial_value
		
	def update(self,):
		# self.value = 0.9
		self.value = vals[(vals.index(self.value)+1)%len(vals)]
		
connected_threshold = modifiable_global(0.5)