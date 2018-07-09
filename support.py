# -*- coding: utf-8 -*-
"""
Created on Mon Jun 25 15:58:34 2018

@author: GVKD1542
"""

''' GENERAL ''' 
from numpy.random import random as rd
from numpy import ceil
from numpy import floor
import threading
from globals import *




def phi(instance,i,j=-1):
	#(unused?) function giving correspondance between (i,j) indexing in a matrix and k (linear) indexing in a set
	#defines both direct and inverse functions 
	
	# matrix indexing --> set indexing
	if(j!=-1):
		if(i<j):
			k=i
			i=j
			j=k
		res = sum(instance.n-k for k in range(1,j))+(i-j)
		return res
		
	# set indexing --> matrix indexing
	else:
		n = instance.n
		k=i
		l,p=1,1
		while(i>(n-1)):
			i-=instance.n-1
			n-=1
			p+=1
		l=i+p
		return (l,p)



def set_bounds(instance, i, j):
	#creates the appropriate bounds for the solution values 
	if(i<=j):
		return(-1,-1)
	if(j==0):	#bridge connecting to the depot
		return (0,2) 
	return (0,1)   #other bridge




def lower_tri_filter(instance,i,j):
	# unused?) pyomo filter for selecting only valid bridges (i,j)
	return j<i


def rule_deg(instance,i):
	#returns the rule for constructing degree constraints
	return sum( ( instance.x[i,j] if i>j else instance.x[j,i] ) for j in instance.nodes if i!=j  ) == (2 if i>0 else 2*instance.number_of_vehicles)


def stop_loop():
	#returns a random bool
	return np.random.random()>0.1

def continue_column_generation(instance,loop_count):
	#must check whether enough constraints have been generated
	
	#define an arbitrary max number of iterations
	if loop_count>=max_column_generation_count :
		return False
	
	#for the future : devise a strategy based on instance (possibly on the value of the objective function)
	#to decide whether or not we must stop the constraint generation
	
	return True
	
def solution_is_integer(instance):
	#returns bool describing wheter instance.x only has integers 
	# with slack epsilon (eps)
	for b in instance.x.keys():
		try:
			val = instance.x[b].value
			expr = abs(round(val)-val)>eps
		except TypeError:
			raise NameError("found None for x[i,j] where i="+str(n)+" and j="+str(m))
		if expr:
			return False
	return True

def is_int(x):
	#returns bool describing wheter pyo.Par/pyo.Var x is integer 
	# with slack epsilon (eps)
	return abs(round(x.value)-x.value)<eps


def integerize_solution(instance):
	#rounds solution values of instance that has been considered integer enough
	for b in instance.x.keys():
		instance.x[b].value = round(instance.x[b].value)


def show_entries(instance):
	#shows the entries of instance 
	for p in instance.component_objects(Param,active=True):
		parameter = getattr(instance,str(p))
		print(parameter)
		for i in parameter :
			if (type(i) in [int,float64,float] and i > 5) or (type(i)==tuple and 6 in i):
				break
			if type(parameter[i]) in [int,str,float,float64]:
				print(parameter[i])
			else:
				print(parameter[i].value) 

def count_fixed_variables(instance):
	count = 0
	for b in instance.x.keys():
		if instance.x[b].fixed == True :
			count+=1
	return count
	
def to_dict(instance):
	#return a conversion of instance.x to a dictionary containing only valid bridges (i>j)
	convert = {}
	for brige in instance.x.keys():
		convert[bridge] = instance.x[bridge].value
	return converts

def to_list_locations(instance):
	d = []
	for i in instance.nodes:
		d.append((instance.locations[i,0],instance.locations[i,1]))
	return d

def get_safe_counter():
	lock = threading.Lock()
	return safe_counter(lock)

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

def list_to_string(ls):
	#returns more readible string of a list
	expr = ""
	for el in ls:
		expr += str(el)+"/"
	return expr[:-1]

def integer_percent(instance):
	count = 0
	pos = 0
	for b in instance.x.keys():
		count += 1
		if is_int(instance.x[b]):
			pos += 1
	return round(pos/count*100,2)
