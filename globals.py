# from numpy.random import random as rd
eps = 0.1
logging = False
console_writing = True
vals = [0.01,0.5,0.9]
max_column_generation_count = 50
max_unmoving_count = 2*len(vals)
max_depth = 30               #max number of branching 
max_time_for_shrinking = 5
max_size_for_shrinking = 4 #inclusive
max_global_time = 1000


class modifiable_threshold():
	def __init__(self,initial_value):
		self.value = initial_value
		self.vals = vals
		
	def update(self,):
		# self.value = 0.9
		self.value = vals[(vals.index(self.value)+1)%len(vals)]
		
connected_threshold = modifiable_threshold(0.5)